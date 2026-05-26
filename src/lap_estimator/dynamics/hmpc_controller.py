"""HMPC controller — composes outer planner, inner tracker, PI trim.

Phase 5.1 v3.4 hierarchical-MPC controller (spec §23.4.6.4). Public
surface mirrors :class:`MPCController` / :class:`MPCCController` so
``slip_simulator._make_controller`` adds exactly one dispatch branch.

Per-tick control loop:

  1. Project chassis ``(x, y, ψ)`` to curvilinear ``(s, n, ψ_e)``.
  2. If the outer cadence timer elapsed *or* the chassis is approaching
     the outer horizon end (forced re-solve trigger, §23.4.7.4), fire
     :meth:`OuterPlanner.solve`. Cache the new
     :class:`ReferenceTrajectory`.
  3. Build the inner stage grid + sample the (frozen) outer reference
     onto it.
  4. Fire :meth:`InnerTracker.solve` against the outer reference.
  5. On QP infeasibility, retry with the DP plan as reference (Tier 1).
     On second consecutive infeasibility, hand off to the reactive
     sub-controller (Tier 2; same `_long_sub` pattern as v3.2).
  6. Apply PI trim (bounded ``e_n``, ``e_vx`` correction).
  7. Commit ``(steer, throttle, brake)``.

Build-time decisions resolved (spec §23.4.10–11):

- **μ_circle buffer.** Default ``0.85 · min(D_lat_front, D_long)``
  per Risk 1; sweep 0.85 → 0.65 if PI trim saturates (CLI:
  ``--hmpc-outer-mu-circle``).
- **`solve_sqp` signature.** Direct ``n_ref_seq`` / ``psi_e_ref_seq``
  kwargs (no wrapper). ``None`` defaults preserve bit-identical v3.2
  behaviour. Documented in :mod:`hmpc_inner`.
- **Cadence scheduling.** Outer fires at ``t = _last_outer_t +
  outer_period``; ``_last_outer_t`` is initialised to
  ``-0.5 · outer_period`` so the **first** fire is at t = 0.5 ·
  outer_period (out of phase with the inner's t=0 first solve). This
  approximates the "outer at tick N where N % 50 == 25" requested in
  the task brief without adding tick-index arithmetic. Subsequent
  fires are simple cadence ticks; the outer is also force-fired the
  moment the chassis closes within 30 m of the outer's horizon end.
- **Pedal emit path.** Inner's first-stage commit drives the chassis
  pedals (analogous to v3.2 `emit_source="qp"`). The reactive sub-
  controller runs in parallel for Tier-2 fallback only. Rationale:
  the headline acceptance test (spec §23.4.12 + task brief) requires
  the brake commit at chicane entry to move from s ≈ 429 m to s ≈
  257 m. PI trim alone (±0.10 pedal authority) cannot deliver that
  swing; the inner's QP cost on ``(v_x - v_ref_outer)²`` does. The
  PI trim layers small corrections on top. CLI ``--hmpc-emit-source
  sub`` reverts to the literal-spec emit (reactive pedals + PI trim
  only) for A/B regression.
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING

import numpy as np

from ._control_params import ControlParams
from .driver_controller import DriverController
from .hmpc_inner import (
    DEFAULT_HORIZON_M as DEFAULT_INNER_HORIZON_M,
)
from .hmpc_inner import (
    DEFAULT_N_STAGES as DEFAULT_INNER_N_STAGES,
)
from .hmpc_inner import (
    DEFAULT_TICK_HZ as DEFAULT_INNER_TICK_HZ,
)
from .hmpc_inner import (
    InnerTracker,
    InnerTrackerConfig,
    first_stage_commit,
)
from .hmpc_inner_casadi import CasadiInnerTracker
from .hmpc_outer import (
    DEFAULT_HORIZON_M as DEFAULT_OUTER_HORIZON_M,
)
from .hmpc_outer import (
    DEFAULT_N_STAGES as DEFAULT_OUTER_N_STAGES,
)
from .hmpc_outer import (
    DEFAULT_RATE_HZ as DEFAULT_OUTER_RATE_HZ,
)
from .hmpc_outer import (
    OuterPlanner,
    OuterPlannerConfig,
    OuterPlannerError,
    ReferenceTrajectory,
)
from .hmpc_pi_trim import (
    DEFAULT_PI_BOUND_PEDAL_ABS,
    DEFAULT_PI_BOUND_STEER_FRAC,
    DEFAULT_PI_KI_N,
    DEFAULT_PI_KI_VX,
    DEFAULT_PI_KP_N,
    DEFAULT_PI_KP_VX,
    PITrim,
    PITrimConfig,
)
from .mpc_controller import _suppress_pedal_overlap
from .mpc_model import NU, PlantConstants, build_plant_constants
from .mpc_physics import MpcPhysicsConfig
from .mpc_qp import MPCBounds, MPCWeights
from .mpcc_reference import (
    DEFAULT_TRACK_HALF_WIDTH,
    build_reference_path,
    sample_seq,
    to_curvilinear,
)
from .vehicle import Controls, VehicleState, load_car_dynamics

if TYPE_CHECKING:
    from ..driver import Driver
    from ..track import Track
    from .longitudinal_planner import LongitudinalPlan
    from .vehicle import PacejkaCalibration
    from .vehicle import Car


log = logging.getLogger(__name__)

# Tier IDs — share namespace with v3.2 MPC for SlipSimResult tier_counts.
TIER_HMPC = 0       # clean hierarchical (outer + inner + PI trim)
TIER_DP_INNER = 1   # Tier 1: inner tracks DP plan (no outer reference)
TIER_REACTIVE = 2   # Tier 2: reactive sub-controller

# Forced-re-solve trigger: chassis within this margin of the outer's
# horizon end forces an immediate outer fire (spec §23.4.7.4).
HORIZON_END_MARGIN_M = 30.0

# Tier-2 hysteresis-exit window (same convention as MPCC).
N_TIER2_RECOVERY_HYSTERESIS = 5

# PI-trim defaults / class moved to hmpc_pi_trim.py for modularity
# (spec §23.4.6.4). The defaults are re-exported here so the controller's
# kwarg block stays self-contained.


def _resolve_hmpc_block(driver) -> dict:
    """Return optional ``control_params.hmpc`` driver-JSON block, or ``{}``."""
    raw = getattr(driver, "raw", None)
    if not isinstance(raw, dict):
        return {}
    cp = raw.get("control_params") or {}
    if not isinstance(cp, dict):
        return {}
    block = cp.get("hmpc")
    return block if isinstance(block, dict) else {}


class HMPCController:
    """Hierarchical (two-layer) MPC controller — spec §23.4."""

    def __init__(
        self,
        driver: "Driver",
        track: "Track",
        car: "Car",
        *,
        plan: "LongitudinalPlan",
        calib: "PacejkaCalibration",
        params: ControlParams | None = None,
        rng_seed: int | None = None,
        # Outer overrides.
        outer_horizon_m: float | None = None,
        outer_n_stages: int | None = None,
        outer_rate_hz: float | None = None,
        outer_mu_circle: float | None = None,
        outer_w_progress: float | None = None,
        outer_w_n: float | None = None,
        outer_w_du: float | None = None,
        outer_w_v: float | None = None,
        outer_vref_lookahead_stages: int | None = None,
        # Inner overrides.
        inner_horizon_m: float | None = None,
        inner_n_stages: int | None = None,
        inner_tick_hz: float | None = None,
        inner_sqp_max_iter: int | None = None,
        # PI trim.
        pi_kp_n: float | None = None,
        pi_ki_n: float | None = None,
        pi_kp_vx: float | None = None,
        pi_ki_vx: float | None = None,
        pi_bound_steer_frac: float | None = None,
        pi_bound_pedal_abs: float | None = None,
        # Diagnostics / behaviour.
        outer_disable: bool = False,
        force_static_fz: bool = False,
        emit_source: str = "qp",
        # Phase 5.3 (spec §23.4 close-out): inner solver dispatch.
        # ``"osqp"`` is the v3.2 LTV-bicycle SQP path (default; bit-
        # identical to all post-5.0 builds). ``"casadi"`` swaps in
        # :class:`CasadiInnerTracker`, the nonlinear IPOPT inner that
        # mirrors the Phase 5.2 outer-pivot. Same return shape, same
        # interface; only the solver class changes.
        inner_solver: str = "osqp",
        slip_target_rad_override: float | None = None,
        line_xs: np.ndarray | None = None,
        line_ys: np.ndarray | None = None,
        debug_trace_path: str | None = None,
    ) -> None:
        if not getattr(track, "is_csv_backed", False):
            raise ValueError("HMPCController needs a CSV-backed track.")
        emit_source_norm = str(emit_source).strip().lower()
        if emit_source_norm not in ("qp", "sub"):
            raise ValueError(
                f"HMPCController: emit_source={emit_source!r} not in {{'qp', 'sub'}}"
            )
        self.emit_source = emit_source_norm
        self.driver = driver
        self.car = car
        self.params = params if params is not None else ControlParams.from_driver(driver)
        self.calib = calib
        self.plan = plan
        self._outer_disable = bool(outer_disable)
        # ---- Resolve driver-JSON block + per-knob defaults ----
        hmpc_block = _resolve_hmpc_block(driver)

        def _pick(arg, key, default):
            return float(arg) if arg is not None else float(hmpc_block.get(key, default))

        def _pick_int(arg, key, default):
            return int(arg) if arg is not None else int(hmpc_block.get(key, default))

        outer_horizon_m_v = _pick(outer_horizon_m, "outer_horizon_m", DEFAULT_OUTER_HORIZON_M)
        outer_n_stages_v = _pick_int(outer_n_stages, "outer_n_stages", DEFAULT_OUTER_N_STAGES)
        outer_rate_hz_v = _pick(outer_rate_hz, "outer_rate_hz", DEFAULT_OUTER_RATE_HZ)
        outer_w_progress_v = _pick(outer_w_progress, "outer_w_progress", 1.0)
        outer_w_n_v = _pick(outer_w_n, "outer_w_n", 5.0)
        outer_w_du_v = _pick(outer_w_du, "outer_w_du", 0.5)
        outer_w_v_v = _pick(outer_w_v, "outer_w_v", 5.0)
        # vref lookahead: pass through if explicitly set (CLI or driver JSON),
        # otherwise let OuterPlannerConfig default apply.
        if outer_vref_lookahead_stages is not None:
            outer_vref_lookahead_v = int(outer_vref_lookahead_stages)
        elif "outer_vref_lookahead_stages" in hmpc_block:
            outer_vref_lookahead_v = int(hmpc_block["outer_vref_lookahead_stages"])
        else:
            outer_vref_lookahead_v = None

        inner_horizon_m_v = _pick(inner_horizon_m, "inner_horizon_m", DEFAULT_INNER_HORIZON_M)
        inner_n_stages_v = _pick_int(inner_n_stages, "inner_n_stages", DEFAULT_INNER_N_STAGES)
        inner_tick_hz_v = _pick(inner_tick_hz, "inner_tick_hz", DEFAULT_INNER_TICK_HZ)
        inner_sqp_max_iter_v = _pick_int(
            inner_sqp_max_iter, "inner_sqp_max_iter", 3,
        )

        pi_cfg = PITrimConfig(
            K_p_n=_pick(pi_kp_n, "pi_kp_n", DEFAULT_PI_KP_N),
            K_i_n=_pick(pi_ki_n, "pi_ki_n", DEFAULT_PI_KI_N),
            K_p_vx=_pick(pi_kp_vx, "pi_kp_vx", DEFAULT_PI_KP_VX),
            K_i_vx=_pick(pi_ki_vx, "pi_ki_vx", DEFAULT_PI_KI_VX),
            bound_steer_frac=_pick(
                pi_bound_steer_frac, "pi_bound_steer", DEFAULT_PI_BOUND_STEER_FRAC,
            ),
            bound_pedal_abs=_pick(
                pi_bound_pedal_abs, "pi_bound_pedal", DEFAULT_PI_BOUND_PEDAL_ABS,
            ),
        )

        # ---- Reference path (shared with MPCC). Sprint A: ~14 m width. ----
        line_override = None
        if line_xs is not None and line_ys is not None:
            line_override = (
                np.asarray(line_xs, dtype=float),
                np.asarray(line_ys, dtype=float),
            )
        self.ref = build_reference_path(
            track, plan,
            track_half_width_default=DEFAULT_TRACK_HALF_WIDTH,
            line_override=line_override,
        )

        # ---- Plant constants (shared with v3.2 MPC / MPCC). ----
        v_ref_avg = (
            float(np.mean(plan.speeds[: len(plan.speeds) // 2 + 1]))
            if len(plan.speeds) else 30.0
        )
        self.dyn = load_car_dynamics(car)
        # Slip target (driver JSON or override).
        if slip_target_rad_override is not None:
            self.slip_target_rad = float(slip_target_rad_override)
        elif self.params.slip_target_deg is not None:
            self.slip_target_rad = math.radians(float(self.params.slip_target_deg))
        else:
            self.slip_target_rad = math.radians(float(driver.derived_slip_target_deg()))
        skill_factor = max(0.5, 0.5 + 0.5 * float(driver.skill_pct))
        apf_deg = None
        try:
            apf_deg = driver._alpha_peak_front_deg()  # noqa: SLF001
        except Exception:  # noqa: BLE001
            apf_deg = None
        if apf_deg is None or apf_deg <= 0:
            self.alpha_peak_rad = self.slip_target_rad / max(0.5, 0.5 + 0.5 * float(driver.skill_pct))
        else:
            self.alpha_peak_rad = math.radians(max(4.0, min(10.0, float(apf_deg))))
        alpha_op_front_rad = 0.5 * float(self.alpha_peak_rad) * skill_factor
        alpha_op_rear_rad = 0.5 * float(self.alpha_peak_rad) * skill_factor
        self.physics = MpcPhysicsConfig.from_mpc_block(hmpc_block)
        if force_static_fz:
            self.physics = self.physics.force_static()
        self.pc: PlantConstants = build_plant_constants(
            car, self.dyn, calib, v_ref_avg=v_ref_avg,
            alpha_op_front_rad=alpha_op_front_rad,
            alpha_op_rear_rad=alpha_op_rear_rad,
            a_x_op=0.0, a_y_op=0.0,
            dynamic_fz_enabled=bool(self.physics.dynamic_fz_enabled),
            cg_height_m=self.physics.cg_height_m,
            track_width_f_m=self.physics.track_width_f_m,
            track_width_r_m=self.physics.track_width_r_m,
        )

        # ---- Outer planner. ----
        outer_cfg_kwargs: dict = {
            "horizon_m": outer_horizon_m_v,
            "n_stages": outer_n_stages_v,
            "mu_circle": outer_mu_circle,
            "w_progress": outer_w_progress_v,
            "w_n": outer_w_n_v,
            "w_du": outer_w_du_v,
            "w_v": outer_w_v_v,
        }
        if outer_vref_lookahead_v is not None:
            outer_cfg_kwargs["vref_lookahead_stages"] = outer_vref_lookahead_v
        outer_cfg = OuterPlannerConfig(**outer_cfg_kwargs)
        self.outer = OuterPlanner(self.ref, plan, self.pc, config=outer_cfg)
        self.outer_rate_hz = float(outer_rate_hz_v)
        self.outer_period_s = 1.0 / max(self.outer_rate_hz, 1e-3)

        # ---- Inner tracker. ----
        # Hard bounds shared with v3.2 MPC default settings (spec §23.4.6.3).
        self.bounds = MPCBounds(
            delta_max=math.radians(20.0),
            delta_dot_max=8.0,
            throttle_dot_max=5.0,
            brake_dot_max=5.0,
            vx_min=0.5,
            alpha_axle_max=float(self.alpha_peak_rad),
        )
        # Inner cost weights. Default w_v bumped from v3.2's 0.5 → 10.0
        # for HMPC mode: the outer's brake-anticipation mechanism only
        # works if the inner tracks v_ref_outer aggressively enough that
        # the friction-ellipse-respecting (throttle, brake) commit
        # reflects the outer's early brake-zone start. With w_v=0.5
        # (v3.2's default, tuned for centreline-tracking against the DP
        # plan) the cost is dominated by w_lat·n² and the QP doesn't
        # bite on (v_x - v_ref). Build-time finding 2026-05-24: at
        # w_v=0.5 the chicane brake commit stays at the v3.2 baseline
        # s ≈ 429 m; bumping to 10.0 pulls the commit earlier. Spec
        # §23.4.7.3 explicitly green-lights inner-weight sweeps when
        # the inner can't track the outer reference within the bounds
        # of the PI trim — exactly the diagnosis here.
        self.weights = MPCWeights(
            w_lat=float(hmpc_block.get("inner_w_n", 50.0)),
            w_psi=float(hmpc_block.get("inner_w_psi_e", 20.0)),
            w_v=float(hmpc_block.get("inner_w_v", 10.0)),
            w_slip=float(hmpc_block.get("inner_w_slip", 200.0)),
            w_du=float(hmpc_block.get("inner_w_du", 1.0)),
            w_du2=float(hmpc_block.get("inner_w_du2", 1.0)),
            w_term=float(hmpc_block.get("inner_w_term", 100.0)),
            # Phase 5.3 inner brake-aggression knobs (CasADi inner only).
            # ``inner_w_a`` activates the a_long-tracking cost term that
            # consumes the outer's planned longitudinal accel; default
            # 0.0 leaves it off and preserves Phase 5.3 close-out
            # behaviour. ``inner_w_ellipse_soft`` is the soft-penalty
            # weight on the per-axle friction ellipse (default 5000.0
            # matches the previous hard-code).
            w_a=float(hmpc_block.get("inner_w_a", 0.0)),
            w_ellipse_soft=float(hmpc_block.get("inner_w_ellipse_soft", 5000.0)),
        )
        inner_cfg = InnerTrackerConfig(
            horizon_m=inner_horizon_m_v,
            n_stages=inner_n_stages_v,
            tick_hz=inner_tick_hz_v,
            sqp_max_iter=inner_sqp_max_iter_v,
        )
        # Phase 5.3 inner dispatch — driver-JSON override falls through
        # to the CLI/dataclass default. Same string set as
        # :func:`slip_simulator._make_controller` documents.
        inner_solver_str = str(
            hmpc_block.get("inner_solver", inner_solver)
        ).strip().lower()
        if inner_solver_str not in ("osqp", "casadi"):
            raise ValueError(
                f"HMPCController: inner_solver={inner_solver_str!r} "
                f"not in {{'osqp', 'casadi'}}"
            )
        self.inner_solver = inner_solver_str
        if inner_solver_str == "casadi":
            self.inner = CasadiInnerTracker(
                self.weights, self.bounds, self.pc, self.alpha_peak_rad,
                config=inner_cfg,
            )
        else:
            self.inner = InnerTracker(
                self.weights, self.bounds, self.pc, self.alpha_peak_rad,
                config=inner_cfg,
            )
        self._tick_period = 1.0 / inner_cfg.tick_hz
        self._inner_n_stages = int(inner_cfg.n_stages)
        self._inner_ds_stage = float(inner_cfg.horizon_m / max(inner_cfg.n_stages, 1))

        # ---- PI trim. ----
        self._pi = PITrim(
            pi_cfg, delta_max=self.bounds.delta_max, tick_period=self._tick_period,
        )

        # ---- Reactive sub-controller (Tier-2 belt; also for "sub" emit). ----
        self._long_sub = DriverController(
            driver, track,
            params=ControlParams(
                preview_distance_m=self.params.preview_distance_m,
                preview_time_s=self.params.preview_time_s,
                steering_p_gain=self.params.steering_p_gain,
                throttle_p_gain=self.params.throttle_p_gain,
                brake_p_gain=self.params.brake_p_gain,
                throttle_rate_limit_pct_s=self.params.throttle_rate_limit_pct_s,
                slip_target_deg=self.params.slip_target_deg,
                consistency_noise_std_steer_deg=0.0,
                consistency_noise_std_throttle_pct=0.0,
                consistency_noise_std_brake_pct=0.0,
                steering_softener_engage=1.49,
                steering_softener_full=1.5,
                measured=self.params.measured,
            ),
            rng_seed=rng_seed,
            target_speed_ds=plan.distances,
            target_speeds=plan.speeds,
            slip_target_rad_override=self.slip_target_rad,
            line_xs=line_xs,
            line_ys=line_ys,
        )

        # ---- Per-tick state ----
        # Initialise so the outer first fires at t = 0.5 · outer_period
        # (out of phase with the inner's t=0 first solve). Build-time
        # decision per task brief.
        self._last_outer_t = -0.5 * self.outer_period_s
        self._reference: ReferenceTrajectory | None = None
        self._reference_stale_ticks = 0
        self._reference_overrun_count = 0
        self._outer_infeas_count = 0
        # Held commit (steer, throttle, brake). Pre-cold-start zeros.
        self._actuator_delta = 0.0
        self._actuator_throttle = 0.0
        self._actuator_brake = 0.0
        self._held_steer_rad = 0.0
        self._held_throttle = 0.0
        self._held_brake = 0.0
        self._u_seq_prev: np.ndarray = np.zeros((self._inner_n_stages, NU))
        # Tier tracking.
        self._tier_counts: dict[int, int] = {
            TIER_HMPC: 0, TIER_DP_INNER: 0, TIER_REACTIVE: 0,
        }
        self._latest_tick_tier: int = TIER_HMPC
        self._in_tier2: bool = False
        self._tier2_recovery_streak: int = 0
        self._tier2_episodes: int = 0
        self._inner_consec_infeas: int = 0
        self._warned_fallback = False
        self._last_solve_t = -1.0
        # Projection hint (shared across solves; same as MPCC).
        self._proj_hint = 0
        # Outer staleness window (most recent per-tick ticks-since-outer).
        self._staleness_hist: list[int] = []
        # Outer-vs-inner v_ref disagreement (per-inner-tick |Δv|).
        self._outer_inner_dv_hist: list[float] = []
        # RNG / noise.
        self._rng = np.random.default_rng(rng_seed if rng_seed is not None else 0)
        # Debug trace (CSV per tick) — disabled unless caller asks.
        self._debug_trace_path = debug_trace_path
        self._debug_rows: list[dict] = []
        log.info(
            "HMPC built: outer=%.0f m / %d stages @ %.1f Hz "
            "(mu_circle=%.3f, w_progress=%.2f), inner=%.0f m / %d stages @ %.0f Hz "
            "(solver=%s), PI(Kp_n=%.3f,Kp_vx=%.3f, bound_steer=%.2f, bound_pedal=%.2f), "
            "emit_source=%s, outer_disable=%s",
            outer_horizon_m_v, outer_n_stages_v, self.outer_rate_hz,
            self.outer.mu_circle, outer_w_progress_v,
            inner_horizon_m_v, inner_n_stages_v, inner_tick_hz_v,
            self.inner_solver,
            pi_cfg.K_p_n, pi_cfg.K_p_vx,
            pi_cfg.bound_steer_frac, pi_cfg.bound_pedal_abs,
            self.emit_source, self._outer_disable,
        )

    # ------------------------------------------------------------------
    # Public surface — parity with MPCController / MPCCController.
    # ------------------------------------------------------------------

    @property
    def solve_times(self) -> list[float]:
        """Inner-tick solve times (for SlipSimResult.mpc_solve_times_s)."""
        return list(self.inner.solve_times)

    @property
    def tier_counts(self) -> dict[int, int]:
        return dict(self._tier_counts)

    @property
    def ghost_step_count(self) -> int:
        return self._tier_counts.get(TIER_REACTIVE, 0)

    @property
    def tier1_episodes(self) -> int:
        return self._tier_counts.get(TIER_DP_INNER, 0)

    @property
    def tier1_max_consecutive_steps(self) -> int:
        return 0  # not tracked separately

    @property
    def tier2_episodes(self) -> int:
        return self._tier2_episodes

    @property
    def qp_status_counts(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for s in self.inner.status_history:
            counts[s] = counts.get(s, 0) + 1
        return counts

    @property
    def post_solve_ellipse_violation_p95(self) -> float:
        return 0.0  # not tracked at the inner; same field carries no signal

    # HMPC-specific properties (used by slip_simulator surface).
    @property
    def hmpc_outer_solve_times_s(self) -> list[float]:
        return list(self.outer.solve_times)

    @property
    def hmpc_inner_solve_times_s(self) -> list[float]:
        return list(self.inner.solve_times)

    @property
    def hmpc_outer_solve_count(self) -> int:
        return int(self.outer.solve_count)

    @property
    def hmpc_inner_solve_count(self) -> int:
        return int(self.inner.solve_count)

    @property
    def hmpc_outer_staleness_p95(self) -> float:
        if not self._staleness_hist:
            return 0.0
        return float(np.quantile(np.asarray(self._staleness_hist), 0.95))

    @property
    def hmpc_outer_vs_inner_vref_p95(self) -> float:
        if not self._outer_inner_dv_hist:
            return 0.0
        return float(np.quantile(np.asarray(self._outer_inner_dv_hist), 0.95))

    @property
    def hmpc_pi_trim_p95(self) -> dict[str, float]:
        return self._pi.p95()

    @property
    def hmpc_outer_infeas_count(self) -> int:
        return int(self.outer.infeas_count)

    # ------------------------------------------------------------------
    # Main per-step entry point.
    # ------------------------------------------------------------------

    def controls(
        self,
        state: VehicleState,
        t: float,
        track: "Track | None" = None,  # noqa: ARG002
    ) -> Controls:
        """One control step.

        ODE solver calls this multiple times per inner tick (RK4 sub-
        steps). We re-solve the MPC stack only when we cross a tick
        boundary; intermediate calls re-emit the held commit (matches
        :class:`MPCController` / :class:`MPCCController`).
        """
        # 1. Tick boundary?
        if (t - self._last_solve_t) >= self._tick_period:
            self._resolve(state, t)
            self._last_solve_t = t

        # 2. Chassis-state divergence -> Tier 2 immediately.
        s_now, n_now, e_psi_now, self._proj_hint = to_curvilinear(
            float(state.x), float(state.y), float(state.psi),
            self.ref, hint_idx=self._proj_hint,
        )
        if abs(n_now) > 4.0 or abs(e_psi_now) > math.radians(20.0):
            self._enter_tier2(t, reason="chassis-divergence")
            self._record_tier(TIER_REACTIVE)
            return self._long_sub.controls(state, t)

        # 3. Tier-2 hysteresis-exit window.
        if self._in_tier2:
            if self._latest_tick_tier == TIER_HMPC:
                self._tier2_recovery_streak += 1
                if self._tier2_recovery_streak >= N_TIER2_RECOVERY_HYSTERESIS:
                    self._in_tier2 = False
                    self._tier2_recovery_streak = 0
                else:
                    self._record_tier(TIER_REACTIVE)
                    return self._long_sub.controls(state, t)
            else:
                self._tier2_recovery_streak = 0
                self._record_tier(TIER_REACTIVE)
                return self._long_sub.controls(state, t)

        # 4. Tier 0/1 emit — held commit + PI trim + (optional) reactive pedals.
        return self._emit(state, t, s_now, n_now)

    # ------------------------------------------------------------------
    # Tick resolution.
    # ------------------------------------------------------------------

    def _resolve(self, state: VehicleState, t: float) -> None:
        """One MPC tick: project, maybe fire outer, fire inner, commit."""
        s_now, n_now, e_psi_now, self._proj_hint = to_curvilinear(
            float(state.x), float(state.y), float(state.psi),
            self.ref, hint_idx=self._proj_hint,
        )

        # ---- (a) Outer cadence + forced-re-solve trigger ----
        outer_fired = False
        outer_due = (t - self._last_outer_t) >= self.outer_period_s
        outer_overrun = (
            self._reference is not None
            and (s_now >= self._reference.s_horizon_end - HORIZON_END_MARGIN_M)
        )
        if not self._outer_disable and (outer_due or outer_overrun or self._reference is None):
            try:
                self._reference = self.outer.solve(
                    (float(s_now), float(n_now), float(e_psi_now), float(state.v_x)),
                )
                self._last_outer_t = t
                outer_fired = True
                self._reference_stale_ticks = 0
                # Risk 8: bump inner SQP iter cap on the very next solve so
                # the linearisation point catches up to the new reference.
                self.inner.request_extra_sqp_iter_next()
            except OuterPlannerError as exc:
                self._outer_infeas_count += 1
                if self._reference is None:
                    log.warning(
                        "HMPC: outer cold-start infeasible at t=%.2fs (%s); "
                        "inner falls back to DP-plan tracking.",
                        t, exc,
                    )
                else:
                    log.info("HMPC: outer infeasible at t=%.2fs, keeping stale ref.", t)
        else:
            self._reference_stale_ticks += 1

        # ---- (b) Build inner stage grid + reference samples ----
        s_seq = s_now + np.arange(self._inner_n_stages) * self._inner_ds_stage
        s_seq = np.clip(s_seq, 0.0, max(self.ref.total_length - 1e-3, 0.0))
        kappa_seq, v_dp_seq, _ = sample_seq(s_seq, self.ref)
        v_dp_seq = np.maximum(v_dp_seq, self.bounds.vx_min + 0.1)
        # Outer reference samples (None when no outer ref yet).
        n_ref_seq: np.ndarray | None = None
        psi_e_ref_seq: np.ndarray | None = None
        # Phase 5.3 brake-aggression tune (Lever 3): the outer's
        # planned a_long is fed to the inner so the inner has a
        # *deceleration* target, not just a *speed* target. Without
        # this, the inner sees only ``v_ref`` and the brake commit
        # builds slowly through the v_ref-tracking cost. None on
        # Tier-1 / cold-start.
        a_long_ref_seq: np.ndarray | None = None
        v_ref_seq = v_dp_seq
        if (
            self._reference is not None
            and not self._outer_disable
            and s_seq[0] >= self._reference.s_outer[0] - 1e-3
            and s_seq[-1] <= self._reference.s_outer[-1] + 1e-3
        ):
            v_ref_outer = np.array(
                [self._reference.v_ref_at(s) for s in s_seq], dtype=float,
            )
            n_ref_outer = np.array(
                [self._reference.n_ref_at(s) for s in s_seq], dtype=float,
            )
            psi_e_ref_outer = np.array(
                [self._reference.psi_e_ref_at(s) for s in s_seq], dtype=float,
            )
            # Sanity-check the outer reference: reject NaN/Inf and
            # out-of-bound n / ψ_e values. Fall through to Tier-1
            # (DP-plan tracking) on rejection — keeps the inner's QP
            # well-posed even if a future outer-planner bug leaks
            # garbage into the reference table.
            ref_ok = (
                np.all(np.isfinite(v_ref_outer))
                and np.all(np.isfinite(n_ref_outer))
                and np.all(np.isfinite(psi_e_ref_outer))
                and np.max(np.abs(n_ref_outer)) <= 10.0
                and np.max(np.abs(psi_e_ref_outer)) <= math.radians(45.0)
            )
            if ref_ok:
                v_ref_seq = np.maximum(v_ref_outer, self.bounds.vx_min + 0.1)
                n_ref_seq = n_ref_outer
                psi_e_ref_seq = psi_e_ref_outer
                # Phase 5.3 brake-aggression tune (Lever 3): sample
                # ``a_long_ref`` at the inner stage grid. Inner cost
                # ignores this sequence when ``weights.w_a == 0`` so
                # callers without the tune see bit-identical behaviour.
                a_long_ref_seq = np.array(
                    [self._reference.a_long_ref_at(s) for s in s_seq],
                    dtype=float,
                )
                self._latest_tick_tier = TIER_HMPC
                self._outer_inner_dv_hist.append(
                    float(np.max(np.abs(v_ref_seq - v_dp_seq))),
                )
            else:
                log.info(
                    "HMPC: outer reference rejected (NaN or out-of-bounds); "
                    "Tier-1 DP-plan fallback this tick.",
                )
                self._latest_tick_tier = TIER_DP_INNER
        else:
            self._latest_tick_tier = TIER_DP_INNER
            if self._reference is not None and s_now > self._reference.s_horizon_end:
                self._reference_overrun_count += 1

        # ---- (c) Inner initial state (matches v3.2 MPC layout exactly) ----
        # e_lat / e_psi against the *centreline*. The cost terms incorporate
        # n_ref / psi_e_ref via solve_sqp's new kwargs (HMPC mode), so the
        # state-space x0 stays in the v3.2 plant coords.
        x0 = np.array([
            float(n_now),
            float(e_psi_now),
            float(state.v_x),
            float(state.v_y),
            float(state.omega_yaw),
            float(self._actuator_delta),
            float(self._actuator_throttle),
            float(self._actuator_brake),
        ], dtype=float)

        # ---- (d) Inner solve. ----
        u_prev = self._u_seq_prev[0]
        result = self.inner.solve(
            x0=x0,
            kappa_seq=kappa_seq,
            v_ref_seq=v_ref_seq,
            n_ref_seq=n_ref_seq,
            psi_e_ref_seq=psi_e_ref_seq,
            u_prev=u_prev,
            a_long_ref_seq=a_long_ref_seq,
        )
        if result.infeasible:
            self._inner_consec_infeas += 1
            # Tier-1 retry: same inner, DP plan only.
            if n_ref_seq is not None or psi_e_ref_seq is not None:
                result_retry = self.inner.solve(
                    x0=x0,
                    kappa_seq=kappa_seq,
                    v_ref_seq=v_dp_seq,
                    n_ref_seq=None, psi_e_ref_seq=None,
                    u_prev=u_prev,
                    a_long_ref_seq=None,  # Tier-1: ignore outer a_long plan.
                )
                if not result_retry.infeasible:
                    result = result_retry
                    self._latest_tick_tier = TIER_DP_INNER
                    self._inner_consec_infeas = 0
            if result.infeasible and self._inner_consec_infeas >= 2:
                # Tier-2 fall-through; hand-off to reactive sub-controller.
                self._enter_tier2(t, reason="inner-infeasible-x2")
                self._latest_tick_tier = TIER_REACTIVE
                return
        else:
            self._inner_consec_infeas = 0

        # ---- (e) Commit first-stage rate-controls ----
        self._u_seq_prev = result.u_seq
        new_delta, new_thr, new_brk = first_stage_commit(
            result.u_seq,
            actuator_delta=self._actuator_delta,
            actuator_throttle=self._actuator_throttle,
            actuator_brake=self._actuator_brake,
            bounds=self.bounds,
            ds_stage=self._inner_ds_stage,
            v_lin_first=float(v_ref_seq[0]),
            tick_period=self._tick_period,
        )
        # Standing-start guard (matches MPCCController):
        if float(state.v_x) < 1.0 and float(v_ref_seq[0]) > 2.0:
            new_thr = 1.0
            new_brk = 0.0
        # Pedal-overlap suppression (same as v3.2 emit_source="qp").
        new_thr, new_brk = _suppress_pedal_overlap(new_thr, new_brk, self.pc)
        self._actuator_delta = new_delta
        self._actuator_throttle = new_thr
        self._actuator_brake = new_brk
        self._held_steer_rad = new_delta
        self._held_throttle = new_thr
        self._held_brake = new_brk
        # Staleness diagnostic (ticks since last outer fire).
        ticks_since_outer = int(
            round((t - self._last_outer_t) * 1.0 / max(self._tick_period, 1e-3))
        )
        self._staleness_hist.append(ticks_since_outer)

        # ---- (f) Optional debug trace ----
        if self._debug_trace_path is not None:
            self._debug_rows.append({
                "t": float(t),
                "s": float(s_now),
                "v_x": float(state.v_x),
                "outer_age_ticks": ticks_since_outer,
                "outer_fired": int(outer_fired),
                "v_ref_at_s": (
                    float(self._reference.v_ref_at(s_now))
                    if self._reference is not None else float("nan")
                ),
                "n_ref_at_s": (
                    float(self._reference.n_ref_at(s_now))
                    if self._reference is not None else float("nan")
                ),
                "thr_inner_emit": float(new_thr),
                "brk_inner_emit": float(new_brk),
                "tier": int(self._latest_tick_tier),
                "inner_solve_ms": float(result.solve_time_s * 1000.0),
            })

    # ------------------------------------------------------------------
    # Per-step emit (between MPC ticks).
    # ------------------------------------------------------------------

    def _emit(
        self,
        state: VehicleState,
        t: float,
        s_now: float,
        n_now: float,
    ) -> Controls:
        """Emit the held commit, applying PI trim each step.

        Spec §23.4.6.4:
          - emit_source="qp": commit = held (steer, throttle, brake) +
            PI trim. Inner's anticipated pedals reach the chassis.
          - emit_source="sub": commit = held steer + sub-controller
            (throttle, brake) + PI trim. Legacy "spec literal" path.
        """
        # PI trim — compute against the current frozen reference (if any).
        if self._reference is not None and not self._outer_disable:
            e_n = float(n_now - self._reference.n_ref_at(s_now))
            e_vx = float(state.v_x - self._reference.v_ref_at(s_now))
        else:
            # Tier-1 / cold start: trim against centreline + DP plan v_ref.
            e_n = float(n_now)
            v_ref_here = float(np.interp(
                float(np.clip(s_now, 0.0, self.ref.total_length)),
                self.ref.s_grid, self.ref.v_ref,
            ))
            e_vx = float(state.v_x - v_ref_here)
        steer_trim, thr_trim, brk_trim = self._pi.apply(e_n=e_n, e_vx=e_vx)
        steer = float(self._held_steer_rad + steer_trim)
        if self.emit_source == "qp":
            throttle = float(self._held_throttle) + thr_trim
            brake = float(self._held_brake) + brk_trim
        else:
            sub_cmd = self._long_sub.controls(state, t)
            throttle = float(sub_cmd.throttle) + thr_trim
            brake = float(sub_cmd.brake) + brk_trim
        # Hard clip to actuator envelopes.
        steer = float(np.clip(steer, -self.bounds.delta_max, self.bounds.delta_max))
        throttle = float(np.clip(throttle, 0.0, 1.0))
        brake = float(np.clip(brake, 0.0, 1.0))
        self._record_tier(self._latest_tick_tier)
        return Controls(steer_rad=steer, throttle=throttle, brake=brake)

    # ------------------------------------------------------------------
    # Tier housekeeping.
    # ------------------------------------------------------------------

    def _record_tier(self, tier: int) -> None:
        self._tier_counts[tier] = self._tier_counts.get(tier, 0) + 1

    def _enter_tier2(self, t: float, *, reason: str) -> None:
        if not self._in_tier2:
            self._tier2_episodes += 1
            self._in_tier2 = True
        self._tier2_recovery_streak = 0
        if not self._warned_fallback:
            log.warning(
                "HMPCController -> Tier 2 reactive at t=%.2fs (%s).", t, reason,
            )
            self._warned_fallback = True

    # ------------------------------------------------------------------
    # Lifecycle hooks.
    # ------------------------------------------------------------------

    def flush_debug_trace(self) -> None:
        """Write the per-tick debug trace CSV (if enabled)."""
        if self._debug_trace_path is None or not self._debug_rows:
            return
        try:
            from .hmpc_debug import write_inner_trace_csv
            write_inner_trace_csv(self._debug_trace_path, self._debug_rows)
        except Exception as exc:  # noqa: BLE001
            log.warning("HMPC: failed to write debug trace: %s", exc)


__all__ = ["HMPCController", "TIER_HMPC", "TIER_DP_INNER", "TIER_REACTIVE"]
