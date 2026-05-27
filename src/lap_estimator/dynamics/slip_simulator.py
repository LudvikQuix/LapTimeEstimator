"""Top-level slip-based simulator entry points (spec §23.3).

Mirrors ``simulator.simulate`` / ``simulator.simulate_stint`` shape so the
CLI and web layers can dispatch on ``--model {point-mass, slip}`` without
restructuring.

Phase 4 status: dispatches to the real :class:`DriverController` by default.
``use_ghost=True`` keeps the Phase-3 :class:`GhostDriver` reachable for
regression. When ``driver.consistency_sigma > 0`` and ``mc_runs is None``,
runs ``MC_DEFAULT_RUNS`` (10) Monte-Carlo repetitions with the slip-target
perturbed ±5% per run. ``SlipSimResult`` is drop-in compatible with the v2
``SimResult`` writers (``report.write_trace_csv`` consumes ``distances``,
``speeds``, ``times``, ``lap_id``, ``ai_speeds``, ``two_lap``) and adds
``util_p85``, ``slip_target_rad``, ``mc_*`` fields for Phase 4 metrics.
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING, Any

import numpy as np

from ._control_params import ControlParams
from ._slip_result import (
    SlipSimResult,
    _load_pacejka_calibration,
    _trace_to_result,
)
from .driver_controller import DriverController, GhostDriver
from .ff_pi_controller import FFPIController
from .hmpc_controller import HMPCController
from .mpc_controller import MPCController
from .mpcc_controller import MPCCController
from .pi_controller import PIController
from .safe_pi_controller import SafePIController
from .solver import LapTrace, simulate_slip_lap
from .vehicle import load_car_dynamics

if TYPE_CHECKING:
    from ..car import Car
    from ..driver import Driver
    from ..setup import Setup
    from ..track import Track
    from ..tyre_state import Compound

log = logging.getLogger(__name__)


def _build_target_speed_plan(car, track, driver, ds: float):
    """Run the v2 point-mass simulator once to harvest its racing-line plan.

    Phase 4.1 used the v2 plan **unscaled** — the v2 plan is the
    friction-circle-optimal speed profile against ``mu_v2 ~ 1.20``. With
    fitted Pacejka D_lat ~ 1.03 (Tomas+BMW), that plan is geometrically
    infeasible and the controller aborts at the Sprint A chicane (s ~890 m).
    Phase 4.2 (§23.10.6) replaces it with :func:`_build_dp_plan` by default;
    this helper stays reachable via ``--plan-source v2`` for regression.
    """
    from ..simulator import simulate as _v2_simulate
    return _v2_simulate(car, track, driver=driver, ds=ds, two_lap=False)


def _build_dp_plan(
    car, track, calib,
    *,
    safety_margin: float | None = None,
    chicane_config=None,
):
    """Build a v3 longitudinal DP plan against the fitted Pacejka envelope.

    Spec §23.10.6.2: single forward-backward DP using the front-axle's
    fitted ``D_lat`` / ``D_long``, static Fz, friction-ellipse-aware
    combined-slip headroom, and the documented safety margin (Phase
    5.0.1 default 0.94; pre-Phase-5.0.1 historical 0.97).

    Phase 5.0.2 (spec §23.2-5.0.2): an extra ``chicane_config`` knob
    applies a localised conservative speed cap on tight-radius segments
    (default ``radius<60 m`` -> 20% reduction with 5-segment ramp).
    """
    from .longitudinal_planner import plan_longitudinal
    kwargs = {}
    if safety_margin is not None:
        kwargs["safety_margin"] = safety_margin
    if chicane_config is not None:
        kwargs["chicane_config"] = chicane_config
    return plan_longitudinal(car, track, calib, **kwargs)


def _build_plan(
    car, track, driver, calib, *, ds: float, source: str,
    dp_safety_margin: float | None = None,
    chicane_config=None,
    tomas_csv_path: str | None = None,
):
    """Dispatch on ``--plan-source``.

    ``source='v3_dp'`` (default) — Phase 4.2 honest plan against the fitted
    Pacejka envelope. ``source='v2'`` — Phase 4.1 regression path (the v2
    friction-circle plan that aborts at the Sprint A chicane on this
    car/calib). ``source='tomas'`` — experimental (2026-05-24): use Tomas's
    recorded lap-5 ``v(s)`` directly as the controller reference. No DP,
    no safety_margin, no chicane cap; the recorded trajectory IS the plan.
    See :mod:`tomas_trajectory_plan` and the architecture doc
    ``docs/architecture-v3-tomas-trajectory-injection.md``.
    Anything else raises.
    """
    if source == "v3_dp":
        return _build_dp_plan(
            car, track, calib,
            safety_margin=dp_safety_margin,
            chicane_config=chicane_config,
        )
    if source == "v2":
        return _build_target_speed_plan(car, track, driver, ds=ds)
    if source == "tomas":
        from .tomas_trajectory_plan import build_tomas_plan
        return build_tomas_plan(track, csv_path=tomas_csv_path)
    raise ValueError(
        f"Unknown plan_source={source!r}. Expected one of: 'v2', 'v3_dp', "
        f"'tomas'."
    )


def _compute_util_p85(alpha_front_avg_arr: np.ndarray,
                      slip_target_rad: float) -> float:
    """Compute the 85th-percentile slip-utilisation ratio.

    ``util = |alpha_front_avg(t)| / slip_target_rad``. The 85th
    percentile is the spec §11.55 headline metric. Returns 0.0 if the
    inputs are degenerate.
    """
    if slip_target_rad <= 0.0 or len(alpha_front_avg_arr) == 0:
        return 0.0
    util = np.abs(np.asarray(alpha_front_avg_arr, dtype=float)) / float(slip_target_rad)
    return float(np.quantile(util, 0.85))


def _continue_state(tr: LapTrace) -> "VehicleState":
    """Pack the final point of ``tr`` into a starting :class:`VehicleState`."""
    from .vehicle import VehicleState
    return VehicleState(
        x=float(tr.x[-1]) if len(tr.x) else 0.0,
        y=float(tr.y[-1]) if len(tr.y) else 0.0,
        psi=float(tr.psi[-1]) if len(tr.psi) else 0.0,
        v_x=float(tr.v_x[-1]) if len(tr.v_x) else 0.0,
        v_y=float(tr.v_y[-1]) if len(tr.v_y) else 0.0,
        omega_yaw=float(tr.omega_yaw[-1]) if len(tr.omega_yaw) else 0.0,
        omega_FL=float(tr.omega_w["FL"][-1]) if len(tr.omega_w["FL"]) else 0.0,
        omega_FR=float(tr.omega_w["FR"][-1]) if len(tr.omega_w["FR"]) else 0.0,
        omega_RL=float(tr.omega_w["RL"][-1]) if len(tr.omega_w["RL"]) else 0.0,
        omega_RR=float(tr.omega_w["RR"][-1]) if len(tr.omega_w["RR"]) else 0.0,
    )


def _alpha_front_avg_from_trace(tr: LapTrace) -> np.ndarray:
    """Average |alpha_front| from a :class:`LapTrace` (FL + FR mean)."""
    if not tr.alpha_rad:
        return np.zeros(0)
    fl = tr.alpha_rad.get("FL")
    fr = tr.alpha_rad.get("FR")
    if fl is None or fr is None or len(fl) == 0 or len(fr) == 0:
        return np.zeros(0)
    return 0.5 * (np.asarray(fl, dtype=float) + np.asarray(fr, dtype=float))


def simulate_slip(
    car: "Car",
    track: "Track",
    driver: "Driver",
    *,
    setup: "Setup | None" = None,
    compound: "Compound | None" = None,
    ds: float = 2.0,  # kept for API parity; not used by the slip path
    two_lap: bool = False,
    dt: float = 0.02,
    solver: str = "rk4",  # noqa: ARG001 — Phase 3 only ships RK4
    rng_seed: int | None = None,
    n_laps: int = 1,
    use_ghost: bool = False,
    mc_runs: int | None = None,
    plan_source: str = "v3_dp",
    controller: str = "reactive",
    dp_safety_margin: float | None = None,
    chicane_config=None,
    mpc_tier1_disable: bool = False,
    mpc_tier1_max_consecutive: int | None = None,
    mpc_force_static_fz: bool = False,
    mpc_cimpcc_weight: float | None = None,
    mpc_cimpcc_safety: float | None = None,
    # Phase 5.0.8 (spec OP-4): MPC Tier-0 emit source selector.
    # ``"qp"`` (default) emits the QP-solved throttle/brake directly;
    # ``"sub"`` reverts to the pre-5.0.8 reactive-sub-controller-
    # delegated emit for A/B regression.
    mpc_emit_source: str = "qp",
    # v3.3 MPCC overrides (spec §23.3.6.8).
    mpcc_horizon_m: float | None = None,
    mpcc_n_stages: int | None = None,
    mpcc_tick_hz: float | None = None,
    mpcc_w_contour: float | None = None,
    mpcc_w_lag: float | None = None,
    mpcc_w_progress: float | None = None,
    mpcc_w_du: float | None = None,
    mpcc_force_static_fz: bool = False,
    # v3.4 HMPC overrides (spec §23.4.6.7). All optional.
    hmpc_outer_horizon_m: float | None = None,
    hmpc_outer_n_stages: int | None = None,
    hmpc_outer_rate_hz: float | None = None,
    hmpc_outer_mu_circle: float | None = None,
    hmpc_outer_w_progress: float | None = None,
    hmpc_outer_w_n: float | None = None,
    hmpc_outer_w_du: float | None = None,
    hmpc_inner_horizon_m: float | None = None,
    hmpc_inner_n_stages: int | None = None,
    hmpc_inner_tick_hz: float | None = None,
    hmpc_pi_kp_n: float | None = None,
    hmpc_pi_ki_n: float | None = None,
    hmpc_pi_kp_vx: float | None = None,
    hmpc_pi_ki_vx: float | None = None,
    hmpc_pi_bound_steer: float | None = None,
    hmpc_pi_bound_pedal: float | None = None,
    hmpc_outer_disable: bool = False,
    hmpc_force_static_fz: bool = False,
    hmpc_emit_source: str = "qp",
    hmpc_inner_solver: str = "osqp",
    hmpc_debug_trace_path: str | None = None,
    # Outer line-follow mode (task brief 2026-05-26).
    hmpc_line_follow_path: str | None = None,
    hmpc_line_follow_w_n_ideal: float | None = None,
    hmpc_line_follow_w_psi_ideal: float | None = None,
    # Ideal-line bypass mode (spec 2026-05-27). Dedicated plumbing; does NOT
    # reuse the line-follow keys above. ``hmpc_reference_source="ideal_csv"``
    # + ``hmpc_ideal_line_csv`` route the inner reference straight from the CSV.
    hmpc_reference_source: str | None = None,
    hmpc_ideal_line_csv: str | None = None,
    tomas_csv_path: str | None = None,
    line_source: str = "center",
    # Grip-envelope-fix (spec dev-planning/tyre-grip-envelope-fix). Opt-in
    # multiplicative scale on the fitted lateral+longitudinal peak D of both
    # axles. ``None`` (default) keeps the legacy fitted envelope (D ~ 1.03),
    # so existing recorded laps are unchanged. ~1.24 lifts D_lat to AC's
    # DY_REF ~ 1.28 to reproduce Tomas's measured ~1.5 g friction circle.
    grip_d_scale: float | None = None,
) -> SlipSimResult:
    """Single-lap (or N-lap) slip-based simulation (Phase 4).

    The Phase-4 :class:`DriverController` (preview-target + slip-aware
    speed loop) is the default. ``use_ghost=True`` keeps the Phase-3
    :class:`GhostDriver` reachable for regression testing.

    When the driver has ``consistency_sigma > 0`` and ``mc_runs`` is None
    (default), this function runs ``MC_DEFAULT_RUNS`` Monte-Carlo
    repetitions (each with a perturbed slip_target). The representative
    result returned is the median lap; ``mc_lap_times_s`` and
    ``mc_sigma_s`` are populated on the result.
    """
    if not getattr(track, "is_csv_backed", False):
        raise ValueError("simulate_slip requires a CSV-backed track.")
    calib, fallback = _load_pacejka_calibration(driver, grip_d_scale=grip_d_scale)
    dyn = load_car_dynamics(car)
    n = int(n_laps if n_laps else 1)
    if two_lap and n < 2:
        n = 2

    # Build the controller's target speed plan. Default `v3_dp` (Phase 4.2,
    # spec §23.10.6) runs a single forward-backward DP over the fitted
    # Pacejka envelope and is the documented honest replacement for the
    # `v2` plan (mu_v2 ~ 1.20 — geometrically infeasible for the fitted
    # Pacejka D_lat ~ 1.03 on Tomas+BMW). `v2` is retained for regression /
    # comparison (--plan-source v2).
    # Phase 5.0.2 chicane-safety: CLI override > driver JSON > defaults.
    # When the caller passes a config explicitly we honour it as-is; when
    # None we resolve from the driver's ``control_params.chicane`` block
    # (defaults apply for drivers without one).
    from .chicane_safety import ChicaneSafetyConfig
    resolved_chicane = (
        chicane_config
        if chicane_config is not None
        else ChicaneSafetyConfig.from_driver(driver)
    )
    # When plan_source='tomas' the chicane cap is meaningless (the Tomas
    # plan is the recorded trajectory, no DP / chicane sweeps run); pass
    # the resolved config through anyway so _build_plan can ignore it for
    # the tomas branch without changing the v3_dp call shape.
    plan = _build_plan(
        car, track, driver, calib,
        ds=ds, source=plan_source,
        dp_safety_margin=dp_safety_margin,
        chicane_config=resolved_chicane,
        tomas_csv_path=tomas_csv_path,
    )
    # Resolve the racing-line override (Tomas-line experiment,
    # 2026-05-24). ``line_source='center'`` (default) keeps the
    # centreline; ``'tomas'`` reconstructs Tomas's recorded (x, z) from
    # his lap-5 telemetry and substitutes it on the controllers'
    # Frenet projection. The solver's off-track abort is unaffected
    # (it reads ``track.csv_data['x','z']`` directly).
    line_xs_override: np.ndarray | None = None
    line_ys_override: np.ndarray | None = None
    if line_source == "tomas":
        from .tomas_line import build_tomas_line
        line = build_tomas_line(track, csv_path=tomas_csv_path)
        line_xs_override = line.xs
        line_ys_override = line.zs  # 2nd horizontal axis: AC z, controller's "y"
        _dcl = np.hypot(
            line.xs - np.asarray(track.csv_data["x"], dtype=float),
            line.zs - np.asarray(track.csv_data["z"], dtype=float),
        )
        log.info(
            "Tomas line override: closure_err=%.2f m over %.0f m "
            "(median |d_centreline|=%.2f m, p95=%.2f m, max=%.2f m)",
            line.closure_error_m, float(line.distances[-1]),
            float(np.median(_dcl)),
            float(np.quantile(_dcl, 0.95)),
            float(_dcl.max()),
        )
    elif line_source != "center":
        raise ValueError(
            f"Unknown line_source={line_source!r}. Expected 'center' or 'tomas'."
        )
    params = ControlParams.from_driver(driver)
    sigma = float(getattr(driver, "consistency_sigma", 0.0))

    # MC mode: only meaningful when consistency_sigma > 0 and the caller
    # didn't explicitly pass mc_runs=0. Default to 10 runs (v3 brief —
    # half the v2 default of 20 since v3 wallclock is much higher).
    if mc_runs is None:
        mc_runs = MC_DEFAULT_RUNS if sigma > 0 else 1
    mc_runs = max(1, int(mc_runs))

    if mc_runs > 1:
        return _run_monte_carlo(
            car=car, track=track, driver=driver, calib=calib, dyn=dyn,
            compound=compound, plan=plan, params=params,
            dt=dt, n_laps=n, fallback=fallback, mc_runs=mc_runs,
            rng_seed=rng_seed, use_ghost=use_ghost,
            controller=controller,
            mpc_tier1_disable=mpc_tier1_disable,
            mpc_tier1_max_consecutive=mpc_tier1_max_consecutive,
            mpc_force_static_fz=mpc_force_static_fz,
            mpc_cimpcc_weight=mpc_cimpcc_weight,
            mpc_cimpcc_safety=mpc_cimpcc_safety,
            mpc_emit_source=mpc_emit_source,
            mpcc_horizon_m=mpcc_horizon_m,
            mpcc_n_stages=mpcc_n_stages,
            mpcc_tick_hz=mpcc_tick_hz,
            mpcc_w_contour=mpcc_w_contour,
            mpcc_w_lag=mpcc_w_lag,
            mpcc_w_progress=mpcc_w_progress,
            mpcc_w_du=mpcc_w_du,
            mpcc_force_static_fz=mpcc_force_static_fz,
            hmpc_outer_horizon_m=hmpc_outer_horizon_m,
            hmpc_outer_n_stages=hmpc_outer_n_stages,
            hmpc_outer_rate_hz=hmpc_outer_rate_hz,
            hmpc_outer_mu_circle=hmpc_outer_mu_circle,
            hmpc_outer_w_progress=hmpc_outer_w_progress,
            hmpc_outer_w_n=hmpc_outer_w_n,
            hmpc_outer_w_du=hmpc_outer_w_du,
            hmpc_inner_horizon_m=hmpc_inner_horizon_m,
            hmpc_inner_n_stages=hmpc_inner_n_stages,
            hmpc_inner_tick_hz=hmpc_inner_tick_hz,
            hmpc_pi_kp_n=hmpc_pi_kp_n,
            hmpc_pi_ki_n=hmpc_pi_ki_n,
            hmpc_pi_kp_vx=hmpc_pi_kp_vx,
            hmpc_pi_ki_vx=hmpc_pi_ki_vx,
            hmpc_pi_bound_steer=hmpc_pi_bound_steer,
            hmpc_pi_bound_pedal=hmpc_pi_bound_pedal,
            hmpc_outer_disable=hmpc_outer_disable,
            hmpc_force_static_fz=hmpc_force_static_fz,
            hmpc_emit_source=hmpc_emit_source,
            hmpc_inner_solver=hmpc_inner_solver,
            hmpc_debug_trace_path=hmpc_debug_trace_path,
            hmpc_line_follow_path=hmpc_line_follow_path,
            hmpc_line_follow_w_n_ideal=hmpc_line_follow_w_n_ideal,
            hmpc_line_follow_w_psi_ideal=hmpc_line_follow_w_psi_ideal,
            hmpc_reference_source=hmpc_reference_source,
            hmpc_ideal_line_csv=hmpc_ideal_line_csv,
            line_xs=line_xs_override,
            line_ys=line_ys_override,
        )

    return _run_single(
        car=car, track=track, driver=driver, calib=calib, dyn=dyn,
        compound=compound, plan=plan, params=params,
        dt=dt, n_laps=n, fallback=fallback,
        rng_seed=rng_seed, use_ghost=use_ghost,
        controller=controller,
        mpc_tier1_disable=mpc_tier1_disable,
        mpc_tier1_max_consecutive=mpc_tier1_max_consecutive,
        mpc_force_static_fz=mpc_force_static_fz,
        mpc_cimpcc_weight=mpc_cimpcc_weight,
        mpc_cimpcc_safety=mpc_cimpcc_safety,
        mpc_emit_source=mpc_emit_source,
        mpcc_horizon_m=mpcc_horizon_m,
        mpcc_n_stages=mpcc_n_stages,
        mpcc_tick_hz=mpcc_tick_hz,
        mpcc_w_contour=mpcc_w_contour,
        mpcc_w_lag=mpcc_w_lag,
        mpcc_w_progress=mpcc_w_progress,
        mpcc_w_du=mpcc_w_du,
        mpcc_force_static_fz=mpcc_force_static_fz,
        hmpc_outer_horizon_m=hmpc_outer_horizon_m,
        hmpc_outer_n_stages=hmpc_outer_n_stages,
        hmpc_outer_rate_hz=hmpc_outer_rate_hz,
        hmpc_outer_mu_circle=hmpc_outer_mu_circle,
        hmpc_outer_w_progress=hmpc_outer_w_progress,
        hmpc_outer_w_n=hmpc_outer_w_n,
        hmpc_outer_w_du=hmpc_outer_w_du,
        hmpc_inner_horizon_m=hmpc_inner_horizon_m,
        hmpc_inner_n_stages=hmpc_inner_n_stages,
        hmpc_inner_tick_hz=hmpc_inner_tick_hz,
        hmpc_pi_kp_n=hmpc_pi_kp_n,
        hmpc_pi_ki_n=hmpc_pi_ki_n,
        hmpc_pi_kp_vx=hmpc_pi_kp_vx,
        hmpc_pi_ki_vx=hmpc_pi_ki_vx,
        hmpc_pi_bound_steer=hmpc_pi_bound_steer,
        hmpc_pi_bound_pedal=hmpc_pi_bound_pedal,
        hmpc_outer_disable=hmpc_outer_disable,
        hmpc_force_static_fz=hmpc_force_static_fz,
        hmpc_emit_source=hmpc_emit_source,
        hmpc_inner_solver=hmpc_inner_solver,
        hmpc_debug_trace_path=hmpc_debug_trace_path,
        hmpc_line_follow_path=hmpc_line_follow_path,
        hmpc_line_follow_w_n_ideal=hmpc_line_follow_w_n_ideal,
        hmpc_line_follow_w_psi_ideal=hmpc_line_follow_w_psi_ideal,
        hmpc_reference_source=hmpc_reference_source,
        hmpc_ideal_line_csv=hmpc_ideal_line_csv,
        line_xs=line_xs_override,
        line_ys=line_ys_override,
    )


# Monte-Carlo default for v3 (spec §23 Phase 4 brief: 10 runs vs v2's 20,
# because each v3 lap is much more expensive).
MC_DEFAULT_RUNS = 10


def _flying_lap_initial_state(car, track, plan) -> "VehicleState":
    """Seed a :class:`VehicleState` matching the target plan's flying-lap entry.

    v2 is distance-marched and pays no time penalty for starting at rest;
    v3 is time-marched (RK4) so a ``v_x=0`` start burns ~5-10 s pulling away
    from standstill on a flying-lap comparison. We seed with the plan's
    first speed (typically ~15 m/s for Tomas Sprint A) and spin every wheel
    up to match (``omega = v_x / R_w``) so the t=0 slip ratios are zero
    rather than catastrophic. Plan source (`v2` or `v3_dp`) doesn't matter
    here — both expose a ``.speeds`` array.
    """
    from .solver import _build_track_xy, _initial_heading
    from .vehicle import VehicleState
    xs, ys, _ds, _tot = _build_track_xy(track)
    v0 = float(plan.speeds[0]) if len(plan.speeds) else 0.0
    r_f = float(car.tyre_radius_f)
    r_r = float(car.tyre_radius_r)
    return VehicleState(
        x=float(xs[0]), y=float(ys[0]),
        psi=_initial_heading(track),
        v_x=v0, v_y=0.0, omega_yaw=0.0,
        omega_FL=v0 / r_f, omega_FR=v0 / r_f,
        omega_RL=v0 / r_r, omega_RR=v0 / r_r,
    )


def _run_single(
    *,
    car, track, driver, calib, dyn, compound, plan, params,
    dt, n_laps, fallback, rng_seed, use_ghost,
    slip_target_rad_override: float | None = None,
    controller: str = "reactive",
    mpc_tier1_disable: bool = False,
    mpc_tier1_max_consecutive: int | None = None,
    mpc_force_static_fz: bool = False,
    mpc_cimpcc_weight: float | None = None,
    mpc_cimpcc_safety: float | None = None,
    mpc_emit_source: str = "qp",
    mpcc_horizon_m: float | None = None,
    mpcc_n_stages: int | None = None,
    mpcc_tick_hz: float | None = None,
    mpcc_w_contour: float | None = None,
    mpcc_w_lag: float | None = None,
    mpcc_w_progress: float | None = None,
    mpcc_w_du: float | None = None,
    mpcc_force_static_fz: bool = False,
    hmpc_outer_horizon_m: float | None = None,
    hmpc_outer_n_stages: int | None = None,
    hmpc_outer_rate_hz: float | None = None,
    hmpc_outer_mu_circle: float | None = None,
    hmpc_outer_w_progress: float | None = None,
    hmpc_outer_w_n: float | None = None,
    hmpc_outer_w_du: float | None = None,
    hmpc_inner_horizon_m: float | None = None,
    hmpc_inner_n_stages: int | None = None,
    hmpc_inner_tick_hz: float | None = None,
    hmpc_pi_kp_n: float | None = None,
    hmpc_pi_ki_n: float | None = None,
    hmpc_pi_kp_vx: float | None = None,
    hmpc_pi_ki_vx: float | None = None,
    hmpc_pi_bound_steer: float | None = None,
    hmpc_pi_bound_pedal: float | None = None,
    hmpc_outer_disable: bool = False,
    hmpc_force_static_fz: bool = False,
    hmpc_emit_source: str = "qp",
    hmpc_inner_solver: str = "osqp",
    hmpc_debug_trace_path: str | None = None,
    hmpc_line_follow_path: str | None = None,
    hmpc_line_follow_w_n_ideal: float | None = None,
    hmpc_line_follow_w_psi_ideal: float | None = None,
    hmpc_reference_source: str | None = None,
    hmpc_ideal_line_csv: str | None = None,
    line_xs: np.ndarray | None = None,
    line_ys: np.ndarray | None = None,
) -> SlipSimResult:
    """Execute one slip-sim run (no Monte Carlo)."""
    ctrl = _make_controller(
        car=car, track=track, driver=driver, plan=plan, params=params,
        calib=calib,
        rng_seed=rng_seed, use_ghost=use_ghost,
        slip_target_rad_override=slip_target_rad_override,
        controller=controller,
        mpc_tier1_disable=mpc_tier1_disable,
        mpc_tier1_max_consecutive=mpc_tier1_max_consecutive,
        mpc_force_static_fz=mpc_force_static_fz,
        mpc_cimpcc_weight=mpc_cimpcc_weight,
        mpc_cimpcc_safety=mpc_cimpcc_safety,
        mpc_emit_source=mpc_emit_source,
        mpcc_horizon_m=mpcc_horizon_m,
        mpcc_n_stages=mpcc_n_stages,
        mpcc_tick_hz=mpcc_tick_hz,
        mpcc_w_contour=mpcc_w_contour,
        mpcc_w_lag=mpcc_w_lag,
        mpcc_w_progress=mpcc_w_progress,
        mpcc_w_du=mpcc_w_du,
        mpcc_force_static_fz=mpcc_force_static_fz,
        hmpc_outer_horizon_m=hmpc_outer_horizon_m,
        hmpc_outer_n_stages=hmpc_outer_n_stages,
        hmpc_outer_rate_hz=hmpc_outer_rate_hz,
        hmpc_outer_mu_circle=hmpc_outer_mu_circle,
        hmpc_outer_w_progress=hmpc_outer_w_progress,
        hmpc_outer_w_n=hmpc_outer_w_n,
        hmpc_outer_w_du=hmpc_outer_w_du,
        hmpc_inner_horizon_m=hmpc_inner_horizon_m,
        hmpc_inner_n_stages=hmpc_inner_n_stages,
        hmpc_inner_tick_hz=hmpc_inner_tick_hz,
        hmpc_pi_kp_n=hmpc_pi_kp_n,
        hmpc_pi_ki_n=hmpc_pi_ki_n,
        hmpc_pi_kp_vx=hmpc_pi_kp_vx,
        hmpc_pi_ki_vx=hmpc_pi_ki_vx,
        hmpc_pi_bound_steer=hmpc_pi_bound_steer,
        hmpc_pi_bound_pedal=hmpc_pi_bound_pedal,
        hmpc_outer_disable=hmpc_outer_disable,
        hmpc_force_static_fz=hmpc_force_static_fz,
        hmpc_emit_source=hmpc_emit_source,
        hmpc_inner_solver=hmpc_inner_solver,
        hmpc_debug_trace_path=hmpc_debug_trace_path,
        hmpc_line_follow_path=hmpc_line_follow_path,
        hmpc_line_follow_w_n_ideal=hmpc_line_follow_w_n_ideal,
        hmpc_line_follow_w_psi_ideal=hmpc_line_follow_w_psi_ideal,
        hmpc_reference_source=hmpc_reference_source,
        hmpc_ideal_line_csv=hmpc_ideal_line_csv,
        line_xs=line_xs,
        line_ys=line_ys,
    )
    traces: list[LapTrace] = []
    # First-lap seed: match the target plan's flying-lap entry speed so v3
    # doesn't pay a ~5-10 s standstill-acceleration tax that v2 (distance-
    # marched) skips. Multi-lap continuation still uses the prior lap's
    # terminal state.
    initial: "VehicleState | None" = _flying_lap_initial_state(car, track, plan)
    for _ in range(n_laps):
        tr = simulate_slip_lap(
            car, track, compound, calib, dyn, ctrl.controls,
            initial_state=initial, dt=dt, max_time=600.0,
        )
        traces.append(tr)
        if not tr.finished:
            log.warning(
                "Slip sim lap aborted: %s",
                tr.abort_reason or "(no reason recorded)",
            )
            break
        initial = _continue_state(tr)

    result = _trace_to_result(traces, track, dt=dt, fallback=fallback)
    result.rng_seed = rng_seed
    result.used_ghost = bool(use_ghost)
    # Compute util_p85 against the controller's slip_target.
    if not use_ghost and isinstance(
        ctrl,
        (
            DriverController, MPCController, MPCCController, HMPCController,
            PIController, FFPIController, SafePIController,
        ),
    ):
        slip_target = float(ctrl.slip_target_rad)
    else:
        # Ghost path: pin to peak (6 deg) so util_p85 is at least defined.
        slip_target = math.radians(6.0)
    result.slip_target_rad = slip_target
    # Diagnostics: MPC solve times + ghost-fallback count.
    if isinstance(ctrl, MPCController):
        result.mpc_solve_times_s = ctrl.solve_times
        result.mpc_ghost_steps = ctrl.ghost_step_count
        # Phase 5.0.3 (spec §23.2-5.0.3.7): tier-distribution diagnostics.
        result.mpc_tier_counts = ctrl.tier_counts
        result.mpc_tier1_episodes = ctrl.tier1_episodes
        result.mpc_tier1_max_consecutive_steps = ctrl.tier1_max_consecutive_steps
        result.mpc_tier2_episodes = ctrl.tier2_episodes
        result.mpc_qp_status_counts = ctrl.qp_status_counts
        result.mpc_post_solve_ellipse_violation_p95 = (
            ctrl.post_solve_ellipse_violation_p95
        )
    # v3.3 MPCC diagnostics (spec §23.3.6.7). Re-uses the existing
    # ``mpc_*`` SlipSimResult slots so report / CSV layers don't need
    # per-controller branching; adds three MPCC-specific terminal metrics.
    if isinstance(ctrl, MPCCController):
        result.mpc_solve_times_s = ctrl.solve_times
        result.mpc_ghost_steps = ctrl.ghost_step_count
        result.mpc_tier_counts = ctrl.tier_counts
        result.mpc_tier1_episodes = 0  # MPCC ships without Tier 1.
        result.mpc_tier1_max_consecutive_steps = 0
        result.mpc_tier2_episodes = ctrl.tier2_episodes
        result.mpc_qp_status_counts = ctrl.qp_status_counts
        result.mpc_post_solve_ellipse_violation_p95 = (
            ctrl.post_solve_ellipse_violation_p95
        )
        result.mpcc_contour_p95 = ctrl.contour_p95
        result.mpcc_lag_p95 = ctrl.lag_p95
        result.mpcc_progress_mean = ctrl.progress_mean
    # v3.4 HMPC diagnostics (spec §23.4.6.5). Re-uses the ``mpc_*`` slots
    # for headline solve-time / tier-share + populates the ``hmpc_*``
    # block for the layer-resolved view.
    if isinstance(ctrl, HMPCController):
        result.mpc_solve_times_s = ctrl.solve_times       # alias: inner ticks
        result.mpc_ghost_steps = ctrl.ghost_step_count
        result.mpc_tier_counts = ctrl.tier_counts
        result.mpc_tier1_episodes = ctrl.tier1_episodes
        result.mpc_tier1_max_consecutive_steps = ctrl.tier1_max_consecutive_steps
        result.mpc_tier2_episodes = ctrl.tier2_episodes
        result.mpc_qp_status_counts = ctrl.qp_status_counts
        result.mpc_post_solve_ellipse_violation_p95 = (
            ctrl.post_solve_ellipse_violation_p95
        )
        result.hmpc_outer_solve_times_s = ctrl.hmpc_outer_solve_times_s
        result.hmpc_inner_solve_times_s = ctrl.hmpc_inner_solve_times_s
        result.hmpc_outer_solve_count = ctrl.hmpc_outer_solve_count
        result.hmpc_inner_solve_count = ctrl.hmpc_inner_solve_count
        result.hmpc_outer_staleness_ticks_p95 = ctrl.hmpc_outer_staleness_p95
        result.hmpc_outer_infeas_count = ctrl.hmpc_outer_infeas_count
        result.hmpc_outer_vs_inner_vref_p95 = ctrl.hmpc_outer_vs_inner_vref_p95
        trims = ctrl.hmpc_pi_trim_p95
        result.hmpc_pi_trim_steer_p95 = float(trims["steer"])
        result.hmpc_pi_trim_throttle_p95 = float(trims["throttle"])
        result.hmpc_pi_trim_brake_p95 = float(trims["brake"])
        result.hmpc_tier_counts = ctrl.tier_counts
        # Flush optional debug trace (no-op when disabled).
        ctrl.flush_debug_trace()
    # Concatenate FL+FR alpha across laps and compute util_p85.
    parts = []
    for tr in traces:
        if tr.alpha_rad and len(tr.alpha_rad.get("FL", ())) > 0:
            parts.append(_alpha_front_avg_from_trace(tr))
    if parts:
        alpha_avg = np.concatenate(parts)
        result.util_p85 = _compute_util_p85(alpha_avg, slip_target)
    return result


def _run_monte_carlo(
    *,
    car, track, driver, calib, dyn, compound, plan, params,
    dt, n_laps, fallback, mc_runs, rng_seed, use_ghost,
    controller: str = "reactive",
    mpc_tier1_disable: bool = False,
    mpc_tier1_max_consecutive: int | None = None,
    mpc_force_static_fz: bool = False,
    mpc_cimpcc_weight: float | None = None,
    mpc_cimpcc_safety: float | None = None,
    mpc_emit_source: str = "qp",
    mpcc_horizon_m: float | None = None,
    mpcc_n_stages: int | None = None,
    mpcc_tick_hz: float | None = None,
    mpcc_w_contour: float | None = None,
    mpcc_w_lag: float | None = None,
    mpcc_w_progress: float | None = None,
    mpcc_w_du: float | None = None,
    mpcc_force_static_fz: bool = False,
    hmpc_outer_horizon_m: float | None = None,
    hmpc_outer_n_stages: int | None = None,
    hmpc_outer_rate_hz: float | None = None,
    hmpc_outer_mu_circle: float | None = None,
    hmpc_outer_w_progress: float | None = None,
    hmpc_outer_w_n: float | None = None,
    hmpc_outer_w_du: float | None = None,
    hmpc_inner_horizon_m: float | None = None,
    hmpc_inner_n_stages: int | None = None,
    hmpc_inner_tick_hz: float | None = None,
    hmpc_pi_kp_n: float | None = None,
    hmpc_pi_ki_n: float | None = None,
    hmpc_pi_kp_vx: float | None = None,
    hmpc_pi_ki_vx: float | None = None,
    hmpc_pi_bound_steer: float | None = None,
    hmpc_pi_bound_pedal: float | None = None,
    hmpc_outer_disable: bool = False,
    hmpc_force_static_fz: bool = False,
    hmpc_emit_source: str = "qp",
    hmpc_inner_solver: str = "osqp",
    hmpc_debug_trace_path: str | None = None,
    hmpc_line_follow_path: str | None = None,
    hmpc_line_follow_w_n_ideal: float | None = None,
    hmpc_line_follow_w_psi_ideal: float | None = None,
    hmpc_reference_source: str | None = None,
    hmpc_ideal_line_csv: str | None = None,
    line_xs: np.ndarray | None = None,
    line_ys: np.ndarray | None = None,
) -> SlipSimResult:
    """N-run Monte-Carlo around the slip-target (±5% per run).

    Each run perturbs the slip_target_rad by Gaussian noise (~5% of
    target) and re-seeds the controller's RNG so the per-channel
    consistency noise differs run-to-run. The representative result
    returned is the *median* lap. ``mc_lap_times_s`` is populated with
    every finished-lap time across all runs.
    """
    base_slip_deg = (
        float(params.slip_target_deg)
        if params.slip_target_deg is not None
        else float(driver.derived_slip_target_deg())
    )
    base_slip_rad = math.radians(base_slip_deg)
    sigma = float(getattr(driver, "consistency_sigma", 0.0))
    # Slip-target perturbation std: 5% of target, scaled mildly by the
    # driver's consistency_sigma (Tomas has sigma≈1.5; a sigma of 1.0
    # gives a 5% spread, 2.0 gives 7%, etc.).
    perturb_frac = 0.05 * max(0.5, min(2.0, sigma))
    rng = np.random.default_rng(rng_seed if rng_seed is not None else 0)

    results: list[SlipSimResult] = []
    for i in range(mc_runs):
        # Slip-target jitter for this run.
        jitter = float(rng.normal(0.0, perturb_frac * base_slip_rad))
        slip_i = float(max(math.radians(0.5), base_slip_rad + jitter))
        # Re-seed each run for reproducibility.
        seed_i = (rng_seed if rng_seed is not None else 0) + 13 * i
        res = _run_single(
            car=car, track=track, driver=driver, calib=calib, dyn=dyn,
            compound=compound, plan=plan, params=params,
            dt=dt, n_laps=n_laps, fallback=fallback,
            rng_seed=seed_i, use_ghost=use_ghost,
            slip_target_rad_override=slip_i,
            controller=controller,
            mpc_tier1_disable=mpc_tier1_disable,
            mpc_tier1_max_consecutive=mpc_tier1_max_consecutive,
            mpc_force_static_fz=mpc_force_static_fz,
            mpc_cimpcc_weight=mpc_cimpcc_weight,
            mpc_cimpcc_safety=mpc_cimpcc_safety,
            mpc_emit_source=mpc_emit_source,
            mpcc_horizon_m=mpcc_horizon_m,
            mpcc_n_stages=mpcc_n_stages,
            mpcc_tick_hz=mpcc_tick_hz,
            mpcc_w_contour=mpcc_w_contour,
            mpcc_w_lag=mpcc_w_lag,
            mpcc_w_progress=mpcc_w_progress,
            mpcc_w_du=mpcc_w_du,
            mpcc_force_static_fz=mpcc_force_static_fz,
            hmpc_outer_horizon_m=hmpc_outer_horizon_m,
            hmpc_outer_n_stages=hmpc_outer_n_stages,
            hmpc_outer_rate_hz=hmpc_outer_rate_hz,
            hmpc_outer_mu_circle=hmpc_outer_mu_circle,
            hmpc_outer_w_progress=hmpc_outer_w_progress,
            hmpc_outer_w_n=hmpc_outer_w_n,
            hmpc_outer_w_du=hmpc_outer_w_du,
            hmpc_inner_horizon_m=hmpc_inner_horizon_m,
            hmpc_inner_n_stages=hmpc_inner_n_stages,
            hmpc_inner_tick_hz=hmpc_inner_tick_hz,
            hmpc_pi_kp_n=hmpc_pi_kp_n,
            hmpc_pi_ki_n=hmpc_pi_ki_n,
            hmpc_pi_kp_vx=hmpc_pi_kp_vx,
            hmpc_pi_ki_vx=hmpc_pi_ki_vx,
            hmpc_pi_bound_steer=hmpc_pi_bound_steer,
            hmpc_pi_bound_pedal=hmpc_pi_bound_pedal,
            hmpc_outer_disable=hmpc_outer_disable,
            hmpc_force_static_fz=hmpc_force_static_fz,
            hmpc_emit_source=hmpc_emit_source,
            hmpc_inner_solver=hmpc_inner_solver,
            hmpc_debug_trace_path=hmpc_debug_trace_path,
            hmpc_line_follow_path=hmpc_line_follow_path,
            hmpc_line_follow_w_n_ideal=hmpc_line_follow_w_n_ideal,
            hmpc_line_follow_w_psi_ideal=hmpc_line_follow_w_psi_ideal,
            hmpc_reference_source=hmpc_reference_source,
            hmpc_ideal_line_csv=hmpc_ideal_line_csv,
            line_xs=line_xs,
            line_ys=line_ys,
        )
        results.append(res)

    # Pick the median-lap result as the representative.
    finished = [r for r in results if r.finished and r.lap_time > 0]
    if not finished:
        # All runs failed; return the first.
        rep = results[0]
    else:
        finished.sort(key=lambda r: r.lap_time)
        rep = finished[len(finished) // 2]

    rep.mc_n_runs = mc_runs
    rep.mc_lap_times_s = [
        float(r.lap_time) for r in results if r.finished and r.lap_time > 0
    ]
    if len(rep.mc_lap_times_s) > 1:
        rep.mc_sigma_s = float(np.std(rep.mc_lap_times_s, ddof=1))
    return rep


def _make_controller(
    *,
    car, track, driver, plan, params, calib,
    rng_seed: int | None, use_ghost: bool,
    slip_target_rad_override: float | None = None,
    controller: str = "reactive",
    mpc_tier1_disable: bool = False,
    mpc_tier1_max_consecutive: int | None = None,
    mpc_force_static_fz: bool = False,
    mpc_cimpcc_weight: float | None = None,
    mpc_cimpcc_safety: float | None = None,
    mpc_emit_source: str = "qp",
    # v3.3 MPCC overrides (spec §23.3.6.8). All optional; defaults applied
    # inside MPCCController.__init__ after consulting the driver JSON.
    mpcc_horizon_m: float | None = None,
    mpcc_n_stages: int | None = None,
    mpcc_tick_hz: float | None = None,
    mpcc_w_contour: float | None = None,
    mpcc_w_lag: float | None = None,
    mpcc_w_progress: float | None = None,
    mpcc_w_du: float | None = None,
    mpcc_force_static_fz: bool = False,
    # v3.4 HMPC overrides (spec §23.4.6.7).
    hmpc_outer_horizon_m: float | None = None,
    hmpc_outer_n_stages: int | None = None,
    hmpc_outer_rate_hz: float | None = None,
    hmpc_outer_mu_circle: float | None = None,
    hmpc_outer_w_progress: float | None = None,
    hmpc_outer_w_n: float | None = None,
    hmpc_outer_w_du: float | None = None,
    hmpc_inner_horizon_m: float | None = None,
    hmpc_inner_n_stages: int | None = None,
    hmpc_inner_tick_hz: float | None = None,
    hmpc_pi_kp_n: float | None = None,
    hmpc_pi_ki_n: float | None = None,
    hmpc_pi_kp_vx: float | None = None,
    hmpc_pi_ki_vx: float | None = None,
    hmpc_pi_bound_steer: float | None = None,
    hmpc_pi_bound_pedal: float | None = None,
    hmpc_outer_disable: bool = False,
    hmpc_force_static_fz: bool = False,
    hmpc_emit_source: str = "qp",
    hmpc_inner_solver: str = "osqp",
    hmpc_debug_trace_path: str | None = None,
    hmpc_line_follow_path: str | None = None,
    hmpc_line_follow_w_n_ideal: float | None = None,
    hmpc_line_follow_w_psi_ideal: float | None = None,
    hmpc_reference_source: str | None = None,
    hmpc_ideal_line_csv: str | None = None,
    line_xs: np.ndarray | None = None,
    line_ys: np.ndarray | None = None,
):
    """Build the controller used for one slip-sim run.

    Phase 5.0 (spec §23.2.8): dispatches on ``controller`` kwarg with
    values ``'reactive'`` (Phase-4.x DriverController, regression path)
    and ``'mpc'`` (Phase-5.0 MPCController, new). The plan + calib are
    fed identically to both; the MPC additionally consumes ``car`` to
    build its linearised plant.

    Phase 4.1 (§23.10.5.1 step 4): the ``target_scale`` heuristic is
    **removed** — the controllers consume ``plan`` UNSCALED. Phase 4.2
    (§23.10.6) makes the default ``plan`` the v3 DP plan; ``--plan-source
    v2`` keeps the legacy v2 plan reachable for regression.
    ``use_ghost=True`` still selects the Phase-3 :class:`GhostDriver`
    with a 0.85x speed scale as the safety regression path.
    """
    # Phase 4.1: hard-coded 1.0. Spec §23.10.5.1 step 4 / §23.10.10.
    target_scale = 1.0
    if use_ghost:
        return GhostDriver(
            track,
            target_speed_ds=plan.distances,
            target_speeds=plan.speeds,
            target_speed_scale=0.85,  # Phase 3 ghost-driver hand-tune retained
            throttle_p_gain=1.0,
            brake_p_gain=1.0,
        )
    if controller == "mpc":
        return MPCController(
            driver, track, car,
            plan=plan, calib=calib, params=params,
            rng_seed=rng_seed,
            slip_target_rad_override=slip_target_rad_override,
            tier1_disable=mpc_tier1_disable,
            tier1_max_consecutive=mpc_tier1_max_consecutive,
            force_static_fz=mpc_force_static_fz,
            cimpcc_weight=mpc_cimpcc_weight,
            cimpcc_safety=mpc_cimpcc_safety,
            line_xs=line_xs,
            line_ys=line_ys,
            emit_source=mpc_emit_source,
        )
    if controller == "mpcc":
        # v3.3 MPCC (spec §23.3). Sibling of `mpc`; curvilinear contouring
        # controller. Steering-only commit; throttle/brake stay with the
        # reactive sub-controller.
        return MPCCController(
            driver, track, car,
            plan=plan, calib=calib, params=params,
            rng_seed=rng_seed,
            slip_target_rad_override=slip_target_rad_override,
            horizon_m=mpcc_horizon_m,
            n_stages=mpcc_n_stages,
            tick_hz=mpcc_tick_hz,
            w_contour=mpcc_w_contour,
            w_lag=mpcc_w_lag,
            w_progress=mpcc_w_progress,
            w_du=mpcc_w_du,
            force_static_fz=mpcc_force_static_fz,
            line_xs=line_xs,
            line_ys=line_ys,
        )
    if controller == "hmpc":
        # v3.4 Hierarchical MPC (spec §23.4). Two-layer:
        # outer point-mass + friction circle (500 m / 50 stages / 1 Hz
        # by default); inner v3.2 LTV bicycle tracking the outer's
        # reference (30 m / 15 stages / 50 Hz). PI trim on cross-track
        # and v_x error, hard-capped at ±15 % steering / ±10 % pedals.
        return HMPCController(
            driver, track, car,
            plan=plan, calib=calib, params=params,
            rng_seed=rng_seed,
            slip_target_rad_override=slip_target_rad_override,
            outer_horizon_m=hmpc_outer_horizon_m,
            outer_n_stages=hmpc_outer_n_stages,
            outer_rate_hz=hmpc_outer_rate_hz,
            outer_mu_circle=hmpc_outer_mu_circle,
            outer_w_progress=hmpc_outer_w_progress,
            outer_w_n=hmpc_outer_w_n,
            outer_w_du=hmpc_outer_w_du,
            inner_horizon_m=hmpc_inner_horizon_m,
            inner_n_stages=hmpc_inner_n_stages,
            inner_tick_hz=hmpc_inner_tick_hz,
            pi_kp_n=hmpc_pi_kp_n,
            pi_ki_n=hmpc_pi_ki_n,
            pi_kp_vx=hmpc_pi_kp_vx,
            pi_ki_vx=hmpc_pi_ki_vx,
            pi_bound_steer_frac=hmpc_pi_bound_steer,
            pi_bound_pedal_abs=hmpc_pi_bound_pedal,
            outer_disable=hmpc_outer_disable,
            force_static_fz=hmpc_force_static_fz,
            emit_source=hmpc_emit_source,
            inner_solver=hmpc_inner_solver,
            line_xs=line_xs,
            line_ys=line_ys,
            debug_trace_path=hmpc_debug_trace_path,
            line_follow_path=hmpc_line_follow_path,
            line_follow_w_n_ideal=hmpc_line_follow_w_n_ideal,
            line_follow_w_psi_ideal=hmpc_line_follow_w_psi_ideal,
            reference_source=hmpc_reference_source,
            ideal_line_csv=hmpc_ideal_line_csv,
        )
    if controller == "reactive":
        return DriverController(
            driver, track, params=params,
            rng_seed=rng_seed,
            target_speed_ds=plan.distances,
            target_speeds=plan.speeds,
            target_speed_scale=target_scale,
            slip_target_rad_override=slip_target_rad_override,
            line_xs=line_xs,
            line_ys=line_ys,
        )
    if controller == "pi":
        # Diagnostic baseline (spec §23.2.x post-mortem-PI). Always emits
        # the per-tick diagnostic CSV to .tmp/pi_diag_sprint_a.csv so the
        # gains can be tuned offline against the same trace.
        return PIController(
            driver, track,
            plan=plan,
            rng_seed=rng_seed,
            slip_target_rad_override=slip_target_rad_override,
            log_csv_path=".tmp/pi_diag_sprint_a.csv",
        )
    if controller == "ffpi":
        # Feedforward-dominant + bounded-PI baseline (spec section "FF+PI
        # baseline"). Same diagnostic-CSV approach as the dumb PI so the
        # FF and trim channels can be inspected independently.
        return FFPIController(
            driver, track, car,
            plan=plan,
            rng_seed=rng_seed,
            slip_target_rad_override=slip_target_rad_override,
            log_csv_path=".tmp/ffpi_diag_sprint_a.csv",
        )
    if controller == "safe_pi":
        # Deliberately-conservative "just finish the lap" baseline. Track
        # centerline + 0.7 * v_max_DP, two independent PI loops, slew limits
        # on every actuator. See safe_pi_controller.py and
        # docs/architecture-slip-model-safe-pi-baseline.md.
        return SafePIController(
            driver, track,
            plan=plan,
            rng_seed=rng_seed,
            slip_target_rad_override=slip_target_rad_override,
            log_csv_path=".tmp/safe_pi_diag_sprint_a.csv",
        )
    raise ValueError(
        f"Unknown controller={controller!r}. Expected 'reactive', 'mpc', "
        f"'mpcc', 'hmpc', 'pi', 'ffpi', or 'safe_pi'."
    )


def simulate_stint_slip(
    car: "Car",
    track: "Track",
    driver: "Driver",
    *,
    n_laps: int,
    setup: "Setup | None" = None,
    calibration: Any | None = None,  # noqa: ARG001 — Phase 5 layers tyre-state evolution
    compound: "Compound | None" = None,
    ds: float = 2.0,
    dt: float = 0.02,
    solver: str = "rk4",  # noqa: ARG001
    rng_seed: int | None = None,
    use_ghost: bool = False,
    mc_runs: int | None = None,
    plan_source: str = "v3_dp",
    controller: str = "reactive",
    dp_safety_margin: float | None = None,
) -> SlipSimResult:
    """Multi-lap slip-based stint (Phase 4 / 5 controller).

    Phase 5 will layer ``tyre_state.update_segments_in_place`` between laps
    with Pacejka-derived slip energy.
    """
    return simulate_slip(
        car, track, driver, setup=setup, compound=compound,
        ds=ds, two_lap=False, dt=dt, rng_seed=rng_seed, n_laps=n_laps,
        use_ghost=use_ghost, mc_runs=mc_runs, plan_source=plan_source,
        controller=controller, dp_safety_margin=dp_safety_margin,
    )
