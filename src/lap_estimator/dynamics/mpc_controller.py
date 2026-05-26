"""Phase-5.0+ receding-horizon MPC driver controller (spec §23.2).

Public surface mirrors :class:`DriverController` so :func:`_make_controller`
in :mod:`slip_simulator` can dispatch on ``--controller {reactive, mpc}``.

The MPC re-solves a finite-horizon optimal control problem every
``tick_period_s`` (default 20 ms / 50 Hz) and holds the first stage's
controls between solves. The solver hardware (``mpc_qp.py``) is a hand-
rolled SQP with OSQP inner-QP; the plant (``mpc_model.py``) is a
linear-time-varying bicycle model linearised about a rolled-forward
reference trajectory.

Inputs / outputs (matches :class:`DriverController`):

- ``__init__`` consumes a :class:`Driver`, :class:`Track`, the v3 DP
  plan as ``LongitudinalPlan``, and the fitted :class:`PacejkaCalibration`.
- ``.controls(state, t, track=None) -> Controls`` — called once per ODE
  step. Between MPC ticks we hold the prior solve's first-stage commit.

Phase 5.0.3 (spec §23.2-5.0.3) three-tier fallback ladder:

- **Tier 0 — MPC.** OSQP solved cleanly; commit the first-stage step.
- **Tier 1 — Ellipse-saturation feedforward (NEW).** QP hard-infeasible
  OR 2+ consecutive soft-divergence ticks. Analytical feedforward that
  saturates the per-axle friction ellipse along the planned direction;
  no QP solve. ~270 LoC across the controller + ``mpc_controller_tiers``
  helpers + ``mpc_qp`` stats.
- **Tier 2 — Reactive sub-controller.** Chassis-state divergence
  (cross-track > 4 m / |e_psi| > 20°) OR Tier 1 ran for
  ``N_TIER1_CONSECUTIVE_MAX`` ticks without recovery. Hands the wheel
  to the embedded reactive ``DriverController`` (``self._long_sub``).

The historical ``_commit_ghost`` path that delegated to
:class:`GhostDriver` is **deprecated** in Phase 5.0.3 (the embedded
ghost was shown in 5.0.2 testing to fail on the same chicane it was
meant to rescue). ``GhostDriver`` import is kept for back-compat /
out-of-band ghost-only regression runs.

Phase 5.0 / 5.0.x keep the per-channel consistency noise applied
post-solve to match v3.1 semantics (the MPC plans the noiseless
trajectory; the driver-execution noise is layered on the commit).
"""

from __future__ import annotations

import logging
import math
from collections import deque
from dataclasses import dataclass
from time import perf_counter
from typing import TYPE_CHECKING

import numpy as np

from ._control_params import ControlParams
from ._ghost_driver import GhostDriver
from .driver_controller import DriverController
from .longitudinal_planner import LongitudinalPlan
from .mpc_controller_geom import (
    GeometryState,
    kappa_at,
    line_tangent,
    nearest_index,
)
from .mpc_controller_tiers import (
    TIER_ELLIPSE,
    TIER_MPC,
    TIER_REACTIVE,
    classify_qp_status,
    compute_planned_direction,
    emit_ellipse_saturation,
    stanley_forces_from_controls,
)
from .mpc_model import (
    NU,
    PlantConstants,
    _axle_fz_at_op,
    build_plant_constants,
    compute_axle_force_from_state,
)
from .mpc_physics import MpcPhysicsConfig
from .mpc_qp import MPCBounds, MPCWeights, solve_sqp
from .mpc_qp_cimpcc import CiMPCCParams
from .vehicle import Controls, PacejkaCalibration, VehicleState, load_car_dynamics

if TYPE_CHECKING:
    from ..car import Car
    from ..driver import Driver
    from ..track import Track

log = logging.getLogger(__name__)


# Default horizon parameters. Deferred from spec §23.2.3:
#
# - **Tick raised 10 Hz -> 50 Hz.** At 10 Hz the chassis state drifts
#   ~30-50° in yaw between ticks at speed, blowing the LTV linearisation
#   and forcing the SQP outer loop to chase phantom errors. 50 Hz keeps
#   each tick coherent with the chassis without over-solving (solve time
#   ~14 ms median on this car; well under the 20 ms inter-tick budget).
# - **Horizon extended 30 m -> 80 m.** At 70 m/s straights Sprint A
#   requires ~180 m of braking lookahead before the chicane (70 m/s ->
#   15 m/s under combined-slip braking). 30 m is < 0.5 s of lookahead at
#   straight-line speed; the MPC cannot see the apex from the brake
#   point and overcooks the corner. 80 m / 20 stages (ds = 4 m) covers
#   most of the brake zone; the soft cross-track + slip costs absorb
#   the residual.
DEFAULT_HORIZON_M = 30.0
DEFAULT_N_STAGES = 15
DEFAULT_TICK_HZ = 50.0
DEFAULT_SQP_MAX_ITER = 3

# Phase 5.0.3 (spec §23.2-5.0.3.5) — escalate to Tier 2 after this many
# consecutive Tier 1 ticks. At 50 Hz that's 200 ms of saturation — covers
# the typical Sprint A chicane transient measured at <100 ms.
N_TIER1_CONSECUTIVE_MAX_DEFAULT = 10

# Phase 5.0.3 — Tier 2 → Tier 0 re-engagement hysteresis (spec
# §23.2-5.0.3.6): 3 consecutive clean ticks before re-trusting MPC.
N_TIER2_RECOVERY_HYSTERESIS = 3


@dataclass(frozen=True)
class Tier1Config:
    """Phase 5.0.3 Tier-1 configuration (spec §23.2-5.0.3.8).

    Loaded from the optional ``control_params.mpc.tier1`` driver-JSON
    block. CLI overrides applied at controller construction. Defaults
    match spec §23.2-5.0.3.3 thresholds.
    """

    enabled: bool = True
    max_consecutive_ticks: int = N_TIER1_CONSECUTIVE_MAX_DEFAULT
    j_residual_multiplier: float = 50.0
    j_residual_window: int = 100
    ellipse_check_stages: int = 4
    # Phase 5.0.6 (Sprint A chicane fix): raised from 0.15 -> 0.40.
    # Investigation (TIER-DIAG instrumented run, --inertia-zz 2400
    # --chicane-safety-mult 0.85) showed the Tier 1 escalation path was
    # driven by a positive feedback loop: a minor pre-chicane transient
    # (sqp_du_inf ~0.011, ellipse_viol ~0.18) tripped detection 5/4 on
    # 2 consecutive ticks, Tier-1 saturation forced the actuators off
    # the linearisation point, the next QP saw du_inf 0.05 -> 0.30 ->
    # 3.7 and ellipse_viol 0.2 -> 0.7 -> 0.9 within ~200 ms, hit the
    # 10-tick streak cap, escalated to Tier 2 (reactive) at t≈10.9s,
    # then aborted off-track at the chicane apex (s≈655 m). The
    # 0.15 threshold was tuned against the spec's "10% linearisation
    # error" estimate but the natural pre-chicane transient already
    # spends a few ticks at 0.15-0.25. 0.40 keeps detection 4 sensitive
    # to genuine ellipse blow-up (post-Tier1 cascade reaches >0.6) while
    # absorbing the natural-transient class.
    ellipse_violation_threshold: float = 0.40
    # Phase 5.0.6: raised from 0.01 -> 0.25. Same diagnosis: the 0.01
    # threshold was effectively a noise floor — the SQP routinely lands
    # at ||Δu||_∞ ~ 0.011-0.014 on clean ticks at corner entries, which
    # tripped detection 5 on 2 consecutive ticks and engaged Tier 1's
    # saturation feedforward; that feedforward then drove the next QP's
    # du_inf to 0.05-0.20+ (cascading) within 1-2 ticks. 0.25 sits well
    # above the natural per-tick SQP step at corner-entry transients
    # (max observed at clean ticks ~0.05) and well below the post-Tier-1
    # cascade values (1.0-3.7) where the linearisation truly is stale.
    # Conservatively above the post-handoff transient ceiling (~0.20)
    # but below the runaway-cascade floor (~0.5+) so the soft signal is
    # still reliable when the QP truly diverges.
    sqp_du_inf_threshold: float = 0.25
    direction_blend_stale_alpha: float = 0.5
    saturation_safety: float = 0.95
    # Phase 5.0.5 (spec inline): rolling-window planned-direction smoother
    # to damp the 50 Hz emit bistability observed in the Sprint A chicane.
    # The Tier-1 emit appends each tick's planned (n_front, n_rear) unit
    # vector pair to a deque of this length; ``compute_planned_direction``
    # returns the time-averaged direction once the buffer is full,
    # bootstrapping to the legacy fixed-alpha blend before then. Default 5
    # ticks = 100 ms at 50 Hz — longer than the observed 20 ms limit cycle
    # so the FIR average attenuates the mode by ≥4×. Set to 0 or 1 to
    # restore Phase 5.0.4 behaviour.
    blend_window_ticks: int = 5

    @classmethod
    def from_block(cls, block: dict | None) -> "Tier1Config":
        if not isinstance(block, dict):
            return cls()
        def _g(name: str, default, cast):
            try:
                return cast(block.get(name, default))
            except (TypeError, ValueError):
                return default
        return cls(
            enabled=bool(block.get("enabled", True)),
            max_consecutive_ticks=_g(
                "max_consecutive_ticks",
                N_TIER1_CONSECUTIVE_MAX_DEFAULT, int,
            ),
            j_residual_multiplier=_g(
                "j_residual_multiplier", 50.0, float,
            ),
            j_residual_window=_g("j_residual_window", 100, int),
            ellipse_check_stages=_g("ellipse_check_stages", 4, int),
            ellipse_violation_threshold=_g(
                "ellipse_violation_threshold", 0.40, float,
            ),
            sqp_du_inf_threshold=_g(
                "sqp_du_inf_threshold", 0.25, float,
            ),
            direction_blend_stale_alpha=_g(
                "direction_blend_stale_alpha", 0.5, float,
            ),
            saturation_safety=_g("saturation_safety", 0.95, float),
            blend_window_ticks=_g("blend_window_ticks", 5, int),
        )


def _suppress_pedal_overlap(
    throttle: float, brake: float, pc: PlantConstants,
) -> tuple[float, float]:
    """Project simultaneous (throttle>0, brake>0) onto a single-pedal commit.

    Phase 5.0.8: the MPC QP has no cost or constraint preventing the
    decision vector from holding both pedals down at once. The real 4-
    wheel slip plant cannot accept that — the front brake just heats
    the discs while the rear brake fights the drive torque, net wasted
    energy. The QP's *intent* (net Fx_rear) is well-defined though:
    ``Fx_rear_intent = pc.k_throttle · throttle − pc.k_brake_rear · brake``.
    We project to a single-pedal commit that delivers the same intent:

    - ``Fx_rear_intent ≥ 0`` (drive): emit
      ``throttle' = min(1, Fx_rear_intent / pc.k_throttle)``, ``brake' = 0``.
    - ``Fx_rear_intent < 0`` (decel): emit ``throttle' = 0``,
      ``brake' = min(1, -Fx_rear_intent / pc.k_brake_rear)``.

    Note this *under*-projects the brake commit by the front-brake
    contribution: the original (throttle, brake) pair had front brake
    ``Fx_front = -pc.k_brake_front · brake`` providing extra
    deceleration; the projected brake' carries only the rear-axle
    intent. This is a deliberate conservative bias: the QP's bicycle
    model treats both axles identically, but the real plant brake
    distribution is biased forward, so under-projecting on brake
    leaves the front axle available for lateral grip. Phase 5.0.8
    diagnostic confirmed the QP-emit overshoot was the dominant
    failure mode; this projection is the spec-permitted emit-path
    mitigation. Re-measured smoke results in
    ``docs/architecture-slip-model-phase5_0_8-v32-first-class-longitudinal.md``.
    """
    if throttle <= 1e-3 or brake <= 1e-3:
        # No overlap — pass through unchanged. Saves the divide.
        return (float(throttle), float(brake))
    k_thr = max(float(pc.k_throttle), 1.0)
    k_brk_r = max(float(pc.k_brake_rear), 1.0)
    fx_intent = k_thr * throttle - k_brk_r * brake
    if fx_intent >= 0.0:
        thr_emit = min(1.0, max(0.0, fx_intent / k_thr))
        return (float(thr_emit), 0.0)
    brk_emit = min(1.0, max(0.0, -fx_intent / k_brk_r))
    return (0.0, float(brk_emit))


def _resolve_mpc_block(driver: "Driver") -> dict:
    """Return the optional ``control_params.mpc`` JSON block, or ``{}``."""
    raw = getattr(driver, "raw", None)
    if not isinstance(raw, dict):
        return {}
    cp = raw.get("control_params") or {}
    if not isinstance(cp, dict):
        return {}
    block = cp.get("mpc")
    return block if isinstance(block, dict) else {}


class MPCController:
    """Receding-horizon MPC controller (spec §23.2).

    Parameters
    ----------
    driver : Driver
        For skill_pct, alpha_peak lookup, and consistency noise.
    track : Track
        CSV-backed track; the racing-line samples come from
        ``track.csv_data['x', 'z', 'distance_m']``.
    car : Car
        Required: the plant constants are derived once at construction.
    plan : LongitudinalPlan
        The v3 DP plan; consumed as ``v_ref(s)`` along the line.
    calib : PacejkaCalibration
        Per-axle Pacejka fit; used to derive linearised cornering stiffness.
    params : ControlParams, optional
    rng_seed : int, optional
        Seed for the consistency-noise RNG.
    horizon_m : float
        Total arc-distance of the MPC lookahead (default 30 m).
    n_stages : int
        Number of MPC stages (default 15; ds = horizon_m / n_stages).
    tick_hz : float
        How often (Hz) the MPC re-solves. Between ticks we hold the prior
        first-stage commit (default 10 Hz).
    sqp_max_iter : int
        Outer SQP iterations per tick (default 3).
    slip_target_rad_override : float, optional
        Monte-Carlo perturbation of the slip target.
    emit_source : {"qp", "sub"}
        Phase 5.0.8 (spec OP-4): Tier 0 longitudinal emit source.
        ``"qp"`` (default) emits the QP-solved
        ``(throttle, brake)`` directly — first-class longitudinal,
        closing the loop the Phase 5.0.7 weight retune diagnosed.
        ``"sub"`` reverts to the pre-5.0.8 delegation: throttle / brake
        come from the reactive sub-controller ``_long_sub``. The latter
        is the A/B regression knob and keeps Phase 5.0.7 bit-identical
        when selected. Tier 1 + Tier 2 emit paths are unaffected.
    """

    def __init__(
        self,
        driver: "Driver",
        track: "Track",
        car: "Car",
        *,
        plan: LongitudinalPlan,
        calib: PacejkaCalibration,
        params: ControlParams | None = None,
        rng_seed: int | None = None,
        horizon_m: float | None = None,
        n_stages: int | None = None,
        tick_hz: float | None = None,
        sqp_max_iter: int | None = None,
        slip_target_rad_override: float | None = None,
        tier1_disable: bool = False,
        tier1_max_consecutive: int | None = None,
        force_static_fz: bool = False,
        cimpcc_weight: float | None = None,
        cimpcc_safety: float | None = None,
        line_xs: np.ndarray | None = None,
        line_ys: np.ndarray | None = None,
        emit_source: str = "qp",
    ) -> None:
        if not getattr(track, "is_csv_backed", False):
            raise ValueError("MPCController needs a CSV-backed track.")
        # Phase 5.0.8 emit-source plumbing (spec OP-4). Validated up-front
        # so the wrong CLI value fails loudly rather than silently picking
        # a default.
        emit_source_norm = str(emit_source).strip().lower()
        if emit_source_norm not in ("qp", "sub"):
            raise ValueError(
                f"MPCController: emit_source={emit_source!r} not in "
                "{'qp', 'sub'}"
            )
        self.emit_source = emit_source_norm
        self.driver = driver
        self.car = car
        self.params = params if params is not None else ControlParams.from_driver(driver)
        self.calib = calib
        self.plan = plan
        # Driver-JSON MPC sub-block overrides any kwarg defaults.
        mpc_block = _resolve_mpc_block(driver)
        self.horizon_m = float(
            horizon_m if horizon_m is not None
            else mpc_block.get("horizon_m", DEFAULT_HORIZON_M)
        )
        self.n_stages = int(
            n_stages if n_stages is not None
            else mpc_block.get("n_stages", DEFAULT_N_STAGES)
        )
        self.tick_hz = float(
            tick_hz if tick_hz is not None
            else mpc_block.get("tick_hz", DEFAULT_TICK_HZ)
        )
        self.sqp_max_iter = int(
            sqp_max_iter if sqp_max_iter is not None
            else mpc_block.get("sqp_max_iter", DEFAULT_SQP_MAX_ITER)
        )
        if not (1 <= self.n_stages <= 50):
            raise ValueError(f"mpc.n_stages={self.n_stages} out of [1, 50]")
        if not (1 <= self.tick_hz <= 100):
            raise ValueError(f"mpc.tick_hz={self.tick_hz} out of [1, 100]")
        if not (1.0 <= self.horizon_m <= 200.0):
            raise ValueError(f"mpc.horizon_m={self.horizon_m} out of [1, 200]")

        # Cost weights — spec §23.2.6 seed values, modulo two tunings
        # documented in the arch doc:
        # - w_psi 5 -> 20: heading error matters more than cross-track for
        #   chicane prediction. With the MPC's 30 m / 0.4 s horizon at
        #   speed, e_psi accumulates faster than e_lat, and the cost ratio
        #   dictates how aggressively the controller fights small heading
        #   drift.
        # - w_v 2 -> 0.5: under pre-Phase-5.0.8 (emit_source="sub")
        #   throttle / brake were delegated to the reactive sub-controller
        #   (see _long_sub), so the QP's speed-tracking cost only needed
        #   to influence the steering channel's linearisation. Under
        #   Phase 5.0.8 first-class emit (emit_source="qp", default) the
        #   QP-computed throttle / brake DO leave the controller, so w_v
        #   now drives the real longitudinal commit too. We keep 0.5 as
        #   the default — the Phase 5.0.7 side-check (open-points OP-4)
        #   showed that raising w_v alone makes things worse, because the
        #   QP's longitudinal channel was being thrown away. With 5.0.8's
        #   emit fix, the lever finally bites; retuning is a follow-up
        #   knob the user can sweep via the ``control_params.mpc.w_v``
        #   driver-JSON override.
        self.weights = MPCWeights(
            w_lat=float(mpc_block.get("w_lat", 50.0)),
            w_psi=float(mpc_block.get("w_psi", 20.0)),
            w_v=float(mpc_block.get("w_v", 0.5)),
            w_slip=float(mpc_block.get("w_slip", 200.0)),
            w_du=float(mpc_block.get("w_du", 1.0)),
            w_du2=float(mpc_block.get("w_du2", 1.0)),
            w_term=float(mpc_block.get("w_term", 100.0)),
        )

        # Hard bounds. The rate caps here are aggressive (10 rad/s ~ very
        # fast hand-over-hand) — the per-tick smoothing applied at commit
        # time bounds chatter without forcing the QP to cripple itself.
        self.bounds = MPCBounds(
            delta_max=math.radians(20.0),
            delta_dot_max=8.0,        # rad/s; equates to 0.8 rad in tick
            throttle_dot_max=5.0,     # 5/s = full pedal in 200 ms
            brake_dot_max=5.0,
            vx_min=0.5,
            alpha_axle_max=math.radians(10.0),  # placeholder; updated below
        )

        # Slip target — same v3.1 mapping kept.
        if slip_target_rad_override is not None:
            self.slip_target_rad = float(slip_target_rad_override)
        elif self.params.slip_target_deg is not None:
            self.slip_target_rad = math.radians(float(self.params.slip_target_deg))
        else:
            self.slip_target_rad = math.radians(float(driver.derived_slip_target_deg()))
        # alpha_peak_front for the slip-budget envelope.
        apf_deg = None
        try:
            apf_deg = driver._alpha_peak_front_deg()  # noqa: SLF001
        except Exception:  # noqa: BLE001
            apf_deg = None
        if apf_deg is None or apf_deg <= 0:
            # Fall back to slip_target (which already includes the skill mapping).
            self.alpha_peak_rad = self.slip_target_rad / max(0.5, 0.5 + 0.5 * float(driver.skill_pct))
        else:
            apf_clamped = max(4.0, min(10.0, float(apf_deg)))
            self.alpha_peak_rad = math.radians(apf_clamped)
        # The soft slip cap used inside the QP is the slip_target_rad
        # (driver-skill-modulated alpha_peak), matching v3.1 semantics
        # (skill=1.0 -> full alpha_peak; skill=0.5 -> 0.75 * alpha_peak).
        self.bounds = MPCBounds(
            delta_max=self.bounds.delta_max,
            delta_dot_max=self.bounds.delta_dot_max,
            throttle_dot_max=self.bounds.throttle_dot_max,
            brake_dot_max=self.bounds.brake_dot_max,
            vx_min=self.bounds.vx_min,
            alpha_axle_max=float(self.slip_target_rad),
        )

        # Build plant constants (one-shot). Use the average v_ref over the
        # first ~half of the plan as the linearisation seed for the engine
        # traction slope.
        v_ref_avg = float(np.mean(plan.speeds[: len(plan.speeds) // 2 + 1])) if len(plan.speeds) else 30.0
        self.dyn = load_car_dynamics(car)
        # Phase 5.0.1 (spec §23.2-5.0.1.3): operating-point Pacejka
        # linearisation about alpha = 0.5 * alpha_peak_axle * skill_factor.
        # Fixed at controller construction (per-stage refresh is the
        # 5.0.2 backlog). skill_factor follows the v3.1 mapping
        # (slip_target = alpha_peak * (0.5 + 0.5 * skill_pct)):
        # skill=1.0 -> alpha_op = 0.5 * alpha_peak; skill=0.5 ->
        # alpha_op = 0.375 * alpha_peak. Resolved decision (open question
        # in spec §23.2-5.0.1.9): tied to skill_pct rather than a separate
        # mpc.alpha_op_frac JSON field — minimum surface area; revisit in
        # 5.0.2 if a driver needs a different operating point for
        # stability.
        skill_factor = max(0.5, 0.5 + 0.5 * float(driver.skill_pct))
        alpha_op_front_rad = 0.5 * float(self.alpha_peak_rad) * skill_factor
        # Rear axle uses the same alpha_peak heuristic — the front-axle peak
        # is the binding axle on the BMW 1M, and the Pacejka per-axle fit
        # for the rear has a similar magnitude. Pre-Phase-5.0.1 build had
        # no alpha-op concept on the rear; keeping symmetric here keeps
        # the bias balanced between axles.
        alpha_op_rear_rad = 0.5 * float(self.alpha_peak_rad) * skill_factor
        # Phase 5.0.4 (spec §23.2-5.0.4.8): per-driver MPC physics
        # overrides. CLI --static-fz (forwarded as ``force_static_fz``)
        # forces the static-Fz path regardless of driver JSON.
        self.physics = MpcPhysicsConfig.from_mpc_block(mpc_block)
        if force_static_fz:
            self.physics = self.physics.force_static()
        self.pc: PlantConstants = build_plant_constants(
            car, self.dyn, calib, v_ref_avg=v_ref_avg,
            alpha_op_front_rad=alpha_op_front_rad,
            alpha_op_rear_rad=alpha_op_rear_rad,
            a_x_op=0.0, a_y_op=0.0,  # static op-point for the construction
                                     # scalars; SQP refreshes per stage.
            dynamic_fz_enabled=bool(self.physics.dynamic_fz_enabled),
            cg_height_m=self.physics.cg_height_m,
            track_width_f_m=self.physics.track_width_f_m,
            track_width_r_m=self.physics.track_width_r_m,
        )
        # Phase 5.0.4 init-time log of the resolved geometry (spec
        # §23.2-5.0.4.9 risk #4 — make the cg_height_m value used
        # visible in the run output for diagnostics).
        log.info(
            "MPC dynamic-Fz: enabled=%s, h_cg=%.3f m, "
            "track_f=%.3f m, track_r=%.3f m, k_lat_loss=%.3f",
            self.physics.dynamic_fz_enabled,
            self.pc.h_cg, self.pc.track_f, self.pc.track_r,
            self.pc.k_lat_loss,
        )

        # Cached track-line geometry. Optional ``line_xs/line_ys`` override
        # (Tomas-line experiment, 2026-05-24) substitutes a non-centreline
        # path for the MPC's Frenet projection + curvature lookup. The
        # override arrays MUST match the track's ``distance_m`` grid; the
        # solver's off-track abort still uses ``track.csv_data['x','z']``
        # (centreline) independently.
        data = track.csv_data
        self._ds = np.asarray(data["distance_m"], dtype=float)
        if line_xs is not None and line_ys is not None:
            xs_line = np.asarray(line_xs, dtype=float)
            ys_line = np.asarray(line_ys, dtype=float)
            if len(xs_line) != len(self._ds) or len(ys_line) != len(self._ds):
                raise ValueError(
                    f"MPCController line override length mismatch: "
                    f"line_xs={len(xs_line)}, line_ys={len(ys_line)}, "
                    f"track distance_m={len(self._ds)}"
                )
            self._xs = xs_line
            self._ys = ys_line
        else:
            self._xs = np.asarray(data["x"], dtype=float)
            self._ys = np.asarray(data["z"], dtype=float)
        self._radius_m = np.asarray(data["radius_m"], dtype=float)
        self._total_len = float(self._ds[-1])
        # Plan resampled onto the track's distance grid (same as v3.1).
        self._v_ref_grid = np.interp(self._ds, plan.distances, plan.speeds)

        # State machine.
        self._tick_period = 1.0 / self.tick_hz
        self._ds_stage = self.horizon_m / self.n_stages
        self._u_seq = np.zeros((self.n_stages, NU))  # warm-start
        self._last_solve_t = -1.0  # force first-step solve
        self._held_steer_rad = 0.0
        self._held_throttle = 0.0
        self._held_brake = 0.0
        # State-tracked actuator positions (the MPC consumes u as rates so
        # we integrate them across ticks).
        self._actuator_delta = 0.0
        self._actuator_throttle = 0.0
        self._actuator_brake = 0.0
        # Diagnostics.
        self._solve_times: list[float] = []
        self._infeasible_ticks = 0
        self._ghost_steps = 0
        self._ghost: GhostDriver | None = None

        # Phase 5.0.3 tier-state (spec §23.2-5.0.3). CLI overrides take
        # precedence over the JSON tier1 block, which takes precedence
        # over the dataclass defaults.
        tier1_block = mpc_block.get("tier1") if isinstance(mpc_block, dict) else None
        self.tier1 = Tier1Config.from_block(tier1_block)
        if tier1_disable:
            self.tier1 = Tier1Config(
                **{**self.tier1.__dict__, "enabled": False}
            )
        if tier1_max_consecutive is not None:
            self.tier1 = Tier1Config(
                **{**self.tier1.__dict__,
                   "max_consecutive_ticks": int(tier1_max_consecutive)},
            )

        # Phase 5.0.8 CiMPCC overlay: CLI > driver JSON > defaults. The
        # overlay is OPT-IN: it stays disabled unless the caller passes a
        # non-None weight (CLI default in lap.py is None, not 0, so users
        # who don't touch the flag get the bit-identical Phase 5.0.7 QP).
        cimpcc_block = (
            mpc_block.get("cimpcc")
            if isinstance(mpc_block, dict) else None
        ) or {}
        if cimpcc_weight is not None:
            _w_kappa = float(cimpcc_weight)
        else:
            _w_kappa = float(cimpcc_block.get("weight", 0.0))
        if cimpcc_safety is not None:
            _safety_kappa = float(cimpcc_safety)
        else:
            _safety_kappa = float(cimpcc_block.get("safety", 0.95))
        self.cimpcc_params = CiMPCCParams(
            enabled=(_w_kappa > 0.0),
            weight=_w_kappa,
            safety=_safety_kappa,
        )

        # Tier counters / state machine.
        self._tier_counts: dict[int, int] = {
            TIER_MPC: 0, TIER_ELLIPSE: 0, TIER_REACTIVE: 0,
        }
        self._tier1_consecutive: int = 0
        self._tier1_max_consecutive_observed: int = 0
        self._tier1_episodes: int = 0
        self._tier2_episodes: int = 0
        self._prev_tier: int = TIER_MPC
        self._tier2_recovery_streak: int = 0
        self._soft_divergence_streak: int = 0
        # Rolling J_residual window for the soft-divergence baseline.
        self._j_residual_history: deque = deque(
            maxlen=max(1, int(self.tier1.j_residual_window)),
        )
        # Per-axle planned forces from the last clean MPC commit (used
        # by the Tier 1 planned-direction primary). Format:
        # ((F_x_f, F_y_f), (F_x_r, F_y_r)). None at controller startup
        # / between resets — Tier 1 falls back to pure-Stanley.
        self._prev_planned_axle_forces: (
            tuple[tuple[float, float], tuple[float, float]] | None
        ) = None
        # Phase 5.0.5: rolling buffer of recent Tier-1 planned-direction
        # unit-vector pairs ``((nx_f, ny_f), (nx_r, ny_r))``. The
        # `compute_planned_direction` helper averages over this buffer to
        # damp the 50 Hz emit bistability spotted at the Sprint A chicane
        # (docs/architecture-slip-model-phase5_0_5-v32-tier1-bistability.md).
        # ``maxlen`` is clamped to >=1 to keep the deque well-formed; a
        # window of 0 or 1 collapses to the legacy single-tick semantics.
        self._tier1_direction_history: deque = deque(
            maxlen=max(1, int(self.tier1.blend_window_ticks)),
        )
        # OSQP status string -> count, surfaced to SlipSimResult at end-of-run.
        self._qp_status_counts: dict[str, int] = {}
        # Post-solve ellipse residuals (per clean Tier-0 tick) for p95.
        self._post_solve_violations: list[float] = []
        # Phase 5.0.8 emit-source diagnostics: per-Tier-0-emit tuple
        # ``(s_m, v_x, throttle_qp, brake_qp, throttle_sub, brake_sub)``.
        # Captured once per Tier 0 emit (so on intermediate ODE steps when
        # the held commit is re-emitted, we record both numbers each
        # step). Inspected in .tmp/phase5_0_8_*.py to compare the QP's
        # longitudinal plan against what the reactive sub-controller
        # would have committed at the same chassis state. Empty when
        # the controller never reaches Tier 0.
        self._emit_diagnostics: list[
            tuple[float, float, float, float, float, float]
        ] = []
        # Whether Tier 1 fired at the most recent MPC tick. Held between
        # ticks so the controls() loop knows to keep emitting saturation
        # commits on intermediate ODE steps.
        self._latest_tick_tier: int = TIER_MPC
        # Whether the controller is currently in a Tier 2 episode
        # (chassis-state divergence or Tier1-escalation). controls()
        # checks this once per step and applies the hysteresis on exit.
        self._in_tier2: bool = False

        # Index hint for nearest-point projection (mutable holder so
        # the geometry helpers can keep amortised-O(window) projection
        # across calls).
        self._geom_state = GeometryState()
        # RNG.
        self._rng = np.random.default_rng(rng_seed if rng_seed is not None else 0)
        self._rng_seed = rng_seed
        self._warned_fallback = False
        # Max steering rate observed for diagnostics.
        self._last_t = 0.0
        # Reactive sub-controller used for throttle/brake (spec §23.2.4
        # deferred decision: combined-slip ellipse is hard to encode as
        # linear inequality in the QP, and the LTV plant's k_throttle *
        # throttle approximation is too coarse for honest plan tracking.
        # We use the MPC for steering only and delegate longitudinal to
        # the v3.1 DriverController, which has been tuned end-to-end
        # against the same plan. Disables the softener on the sub-
        # controller (kill-switch values) so we don't double-apply.
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
                consistency_noise_std_steer_deg=0.0,  # noise applied at MPC level
                consistency_noise_std_throttle_pct=0.0,
                consistency_noise_std_brake_pct=0.0,
                steering_softener_engage=1.49,  # disable softener
                steering_softener_full=1.5,
                measured=self.params.measured,
            ),
            rng_seed=rng_seed,
            target_speed_ds=plan.distances,
            target_speeds=plan.speeds,
            slip_target_rad_override=self.slip_target_rad,
            # Forward the line override so the sub-controller's longitudinal
            # preview projects onto the same line the MPC steers against.
            # ``self._xs / self._ys`` were resolved above (centreline by
            # default, Tomas line when ``line_xs/line_ys`` were passed).
            line_xs=self._xs,
            line_ys=self._ys,
        )

    # ------------------------------------------------------------------
    # Public surface — matches DriverController interface contract.
    # ------------------------------------------------------------------

    @property
    def ghost_step_count(self) -> int:
        return self._ghost_steps

    @property
    def solve_times(self) -> list[float]:
        return list(self._solve_times)

    # ------------------------------------------------------------------
    # Phase 5.0.3 tier-state surface (spec §23.2-5.0.3.7).
    # ------------------------------------------------------------------

    @property
    def tier_counts(self) -> dict[int, int]:
        """Per-ODE-step tier counts ``{0: clean, 1: ellipse, 2: reactive}``."""
        return dict(self._tier_counts)

    @property
    def tier1_episodes(self) -> int:
        return self._tier1_episodes

    @property
    def tier1_max_consecutive_steps(self) -> int:
        return self._tier1_max_consecutive_observed

    @property
    def tier2_episodes(self) -> int:
        return self._tier2_episodes

    @property
    def qp_status_counts(self) -> dict[str, int]:
        return dict(self._qp_status_counts)

    @property
    def post_solve_ellipse_violation_p95(self) -> float:
        if not self._post_solve_violations:
            return 0.0
        return float(np.quantile(np.asarray(self._post_solve_violations), 0.95))

    @property
    def emit_diagnostics(
        self,
    ) -> list[tuple[float, float, float, float, float, float]]:
        """Phase 5.0.8 per-Tier-0-emit diagnostics.

        Each entry is ``(s_m, v_x, throttle_qp, brake_qp, throttle_sub,
        brake_sub)`` recorded once per ODE step that emits a Tier 0
        command. The QP values are what was actually committed in
        ``emit_source="qp"`` mode (or would have been); the ``_sub``
        values are the reactive sub-controller's reading at the same
        chassis state. Used by ``.tmp/phase5_0_8_diag.py`` to verify
        the chicane-entry brake gap predicted by Phase 5.0.7's
        open-points OP-4 diagnosis.
        """
        return list(self._emit_diagnostics)

    def controls(
        self, state: VehicleState, t: float, track: "Track | None" = None,  # noqa: ARG002
    ) -> Controls:
        """Return per-step :class:`Controls` dispatching on the Phase 5.0.3 tier ladder.

        Order per ODE step:

        1. **Chassis-state divergence check** (cross-track > 4 m / |e_psi|
           > 20°). Fires Tier 2 directly per spec §23.2-5.0.3.5 — the
           linearisation is too stale for Tier 1 to be safe.
        2. **MPC re-solve** if we've crossed a tick boundary.
           Updates ``_latest_tick_tier`` to {0, 1, 2}.
        3. **Emit** per the latest tick's tier:
           - Tier 0: held first-stage commit + reactive longitudinal.
           - Tier 1: ellipse-saturation feedforward (per-step recompute
             so the rate clip tracks the latest commit on intermediate
             ODE steps).
           - Tier 2: reactive sub-controller.

        Tier counts increment per ODE step (not per MPC tick) so the
        §11.55-5.0.3 acceptance gates measure user-visible behaviour.
        """
        # 1. MPC tick boundary (always re-attempt so the controller can
        # recover from Tier 1/2 episodes when the chassis returns to
        # the trust region). The classifier inside _resolve_mpc sets
        # _latest_tick_tier.
        if (t - self._last_solve_t) >= self._tick_period:
            self._resolve_mpc(state, t)
            self._last_solve_t = t

        # 2. Chassis-state divergence — routes directly to Tier 2 per
        # spec §23.2-5.0.3.5 (the linearisation is too stale for Tier 1
        # to be safe). Checked AFTER the MPC re-solve so the classifier
        # has the latest QP status when the chassis comes back inside.
        diverged = self._chassis_diverged(state)
        if diverged:
            self._enter_tier2(t, reason="chassis-divergence")
            self._record_tier(TIER_REACTIVE)
            return self._long_sub.controls(state, t)

        # 3. Tier 2 hysteresis-exit window. Once chassis is back inside
        # the trust region the recovery streak ticks up; we re-engage
        # MPC at TIER_MPC clean only after N_TIER2_RECOVERY_HYSTERESIS
        # clean ticks (spec §23.2-5.0.3.6). Until then, keep emitting
        # the reactive sub-controller's command.
        if self._in_tier2:
            if self._latest_tick_tier == TIER_MPC:
                self._tier2_recovery_streak += 1
                if self._tier2_recovery_streak >= N_TIER2_RECOVERY_HYSTERESIS:
                    self._in_tier2 = False
                    self._tier2_recovery_streak = 0
                else:
                    self._record_tier(TIER_REACTIVE)
                    return self._long_sub.controls(state, t)
            else:
                # Latest MPC tick was Tier 1 or Tier 2; stay reactive.
                self._tier2_recovery_streak = 0
                self._record_tier(TIER_REACTIVE)
                return self._long_sub.controls(state, t)

        # 4. Tier 1 — emit ellipse-saturation feedforward. Recomputed
        # per-ODE-step so the rate clip tracks the most recent commit
        # (which it always does in the MPC's held-commit path too).
        if self._latest_tick_tier == TIER_ELLIPSE and self.tier1.enabled:
            self._record_tier(TIER_ELLIPSE)
            return self._emit_tier1_controls(state, t)

        # 5. Tier 0 — held commit + reactive longitudinal. This is the
        # Phase 5.0 happy path, byte-for-byte identical to pre-5.0.3.
        self._record_tier(TIER_MPC)
        return self._emit_tier0_controls(state, t)

    # ------------------------------------------------------------------
    # Phase 5.0.3 tier-emit helpers.
    # ------------------------------------------------------------------

    def _chassis_diverged(self, state: VehicleState) -> bool:
        """Cross-track > 4 m OR |e_psi| > 20° at the current chassis state."""
        idx = self._nearest_index(state.x, state.y)
        ln_x = float(self._xs[idx])
        ln_y = float(self._ys[idx])
        body_dx = float(state.x) - ln_x
        body_dy = float(state.y) - ln_y
        tan_local = self._line_tangent(idx)
        cross_now = math.sqrt(body_dx * body_dx + body_dy * body_dy)
        e_psi_now = math.atan2(
            math.sin(float(state.psi) - tan_local),
            math.cos(float(state.psi) - tan_local),
        )
        return cross_now > 4.0 or abs(e_psi_now) > math.radians(20.0)

    def _enter_tier2(self, t: float, *, reason: str) -> None:
        """Mark a Tier 2 transition + log once per controller lifetime."""
        if not self._in_tier2:
            self._tier2_episodes += 1
            self._in_tier2 = True
        self._tier2_recovery_streak = 0
        self._tier1_consecutive = 0
        # Phase 5.0.5: discard the Tier-1 direction history when handing
        # off to the reactive sub-controller; if MPC re-engages later,
        # the next Tier-1 episode starts fresh.
        self._tier1_direction_history.clear()
        self._ghost_steps += 1  # back-compat alias for Tier 2 count.
        if not self._warned_fallback:
            log.warning(
                "MPCController -> Tier 2 reactive-fallback at t=%.2fs "
                "(%s). Subsequent fallbacks silent.",
                t, reason,
            )
            self._warned_fallback = True

    def _record_tier(self, tier: int) -> None:
        """Increment the per-ODE-step tier counter + episode tracking."""
        self._tier_counts[tier] = self._tier_counts.get(tier, 0) + 1

    def _emit_tier0_controls(self, state: VehicleState, t: float) -> Controls:
        """Tier 0 emit — held MPC commit, longitudinal per ``emit_source``.

        Phase 5.0.8 (spec OP-4 / Phase 5.0.7 diagnosis) promotes throttle
        / brake from "delegated to ``_long_sub``" to a first-class
        decision-variable emit. The QP already solved for them — the
        bug was that the pre-5.0.8 emit threw them away and called the
        reactive sub-controller's throttle / brake instead, which read
        its own Stanley ``steer_cmd`` (not the MPC's δ). That coupling
        was second-order (~0.1 m/s out of +1.7 m/s on Sprint A) and
        explained why every QP cost-weight sweep in Phase 5.0.7 failed:
        the actuators on the longitudinal axis were outside the QP's
        control. See
        ``docs/architecture-slip-model-phase5_0_7-v32-qp-weight-retune.md``
        and ``docs/architecture-slip-model-phase5_0_8-v32-first-class-longitudinal.md``
        for the full diagnosis.

        Behaviour:

        - ``emit_source == "qp"`` (default): commit
          ``(self._held_steer_rad, self._held_throttle,
          self._held_brake)``. All three were already integrated from
          the QP's first-stage rate-controls and rate-clipped /
          saturated inside ``_commit_clean_mpc``, so this path inherits
          the same actuator bounds the steering channel always had.
        - ``emit_source == "sub"``: keep the pre-5.0.8 delegated-
          longitudinal behaviour for A/B regression. The MPC's δ still
          comes from ``self._held_steer_rad`` (unchanged), throttle /
          brake come from ``self._long_sub.controls(state, t)``.

        Diagnostic capture (both modes): record
        ``(s, v_x, throttle_qp, brake_qp, throttle_sub, brake_sub)`` so
        the chicane-entry brake gap is verifiable post-run. The sub-
        controller probe is one extra ``controls()`` call per Tier-0 emit;
        cheap and stateless from the MPC's point of view because
        ``_long_sub`` is consulted on the same chassis state we're
        emitting against (its own internal speed-PI integrator state
        does advance, but only the QP-mode emit is committed to the
        plant — the sub probe is read-only as far as the chassis is
        concerned).
        """
        steer = self._held_steer_rad
        # Phase 5.0.8 pedal-overlap suppression: the QP's longitudinal
        # model is ``Fx_rear = k_thr·throttle − k_brk_r·brake`` and the
        # cost is purely on rates (``w_du`` / ``w_du2``) — there is NO
        # cost or constraint preventing the QP from emitting
        # ``throttle > 0`` AND ``brake > 0`` simultaneously. On the real
        # 4-wheel slip plant that means pressing both pedals at once,
        # which is physically nonsensical (the rear brake fights the
        # rear-axle drive torque and the front brake just dissipates).
        # The 5.0.8 emit smoke (`.tmp/phase5_0_8_diag.py`) observed the
        # QP emitting ``thr_qp ≈ 0.91, brk_qp ≈ 0.29`` on the pre-chicane
        # straight; net rear-axle Fx was nearly zero so the chassis
        # blew through the brake zone at v_x ≈ 59 m/s (vs reactive's
        # 37 m/s) and overshot the chicane entry. Suppression projects
        # to a single pedal preserving net rear-axle Fx — keeps the QP's
        # longitudinal "direction" honest, removes the bilinear artefact.
        throttle_qp, brake_qp = _suppress_pedal_overlap(
            float(self._held_throttle), float(self._held_brake), self.pc,
        )
        # Always run the sub-controller for diagnostics + the "sub" path.
        sub_cmd = self._long_sub.controls(state, t)
        throttle_sub = float(sub_cmd.throttle)
        brake_sub = float(sub_cmd.brake)
        # Diagnostic capture — both modes record both numbers so a single
        # log file is comparable across the A/B sweep.
        idx_now = self._nearest_index(float(state.x), float(state.y))
        s_now = float(self._ds[idx_now])
        self._emit_diagnostics.append(
            (s_now, float(state.v_x),
             throttle_qp, brake_qp, throttle_sub, brake_sub),
        )
        if self.emit_source == "qp":
            throttle = throttle_qp
            brake = brake_qp
        else:
            throttle = throttle_sub
            brake = brake_sub
        return self._apply_consistency_noise(steer, throttle, brake, t)

    def _current_dynamic_fz(self, state: VehicleState) -> tuple[float, float]:
        """Phase 5.0.4: per-axle dynamic Fz at the current chassis state.

        Used by the Tier 1 saturation feedforward so the per-axle
        projection lands on the **dynamic** ellipse rather than the
        controller-construction static one. Returns the static values
        when ``physics.dynamic_fz_enabled`` is False.

        ``a_x`` is approximated by the previous-tick MPC's planned
        v_x_dot (cheap; falls back to 0 when we don't have a stale
        plan). ``a_y`` uses the steady-turn approximation ``v_x · omega``
        — same form the truth model's ``compute_derivatives`` passes
        into ``_weight_transfer`` (vehicle.py line 383-384).
        """
        if not bool(self.physics.dynamic_fz_enabled):
            return float(self.pc.Fz_front), float(self.pc.Fz_rear)
        # Quasi-static a_x estimate. The previous-tick MPC solution's
        # ``v_x_dot`` over the first stage is the most relevant value;
        # we sample it from the rolled trajectory if available. Cheap
        # fallback: 0.0 (under-predicts braking transfer, slight bias
        # toward static-Fz on the front axle entering the corner).
        a_x = 0.0  # truth-model uses 0 too inside _weight_transfer
        a_y = float(state.v_x) * float(state.omega_yaw)
        F_down = float(self.car.downforce(max(float(state.v_x), 0.0)))
        cgf = float(self.pc.cg_front)
        Fz_f_static = self.pc.mass * 9.81 * (1.0 - cgf) + F_down * (1.0 - cgf)
        Fz_r_static = self.pc.mass * 9.81 * cgf + F_down * cgf
        return _axle_fz_at_op(
            Fz_f_static, Fz_r_static,
            mass=self.pc.mass, h_cg=self.pc.h_cg, wb=self.pc.wheelbase,
            track_f=self.pc.track_f, track_r=self.pc.track_r,
            cg_front=cgf, a_x=a_x, a_y=a_y, k_lat_loss=self.pc.k_lat_loss,
        )

    def _emit_tier1_controls(self, state: VehicleState, t: float) -> Controls:
        """Tier 1 emit — ellipse-saturation feedforward (spec §23.2-5.0.3.4).

        Resolves the planned direction per axle from the most recent
        clean MPC commit (primary) blended with a Stanley-style direction
        (when prior tier was also Tier 1). Saturates per-axle on the
        friction ellipse with the 0.95 safety scalar. Inverse-maps to
        ``(δ, throttle, brake)`` through the Phase 5.0.1 affine Pacejka
        and applies the single-tick rate clip identical to the MPC's.
        """
        # Stanley-style direction = sub-controller's command projected
        # through the affine Pacejka at the current chassis state.
        sub_cmd = self._long_sub.controls(state, t)
        stanley_forces = stanley_forces_from_controls(state, sub_cmd, self.pc)

        # Planned direction (resolved by tier-history + Phase 5.0.5
        # rolling-window FIR). The history buffer holds prior emitted
        # unit-vector pairs; the helper averages the last
        # ``blend_window_ticks`` entries once the buffer is full,
        # bootstrapping to the instantaneous value before then.
        n_front, n_rear = compute_planned_direction(
            self._prev_tier,
            self._prev_planned_axle_forces,
            stanley_forces,
            direction_blend_stale_alpha=self.tier1.direction_blend_stale_alpha,
            direction_history=self._tier1_direction_history,
            blend_window_ticks=int(self.tier1.blend_window_ticks),
        )
        # Append the resolved (post-FIR) direction so the next Tier-1
        # tick's average includes this emit. Buffer maxlen enforces the
        # rolling-window semantics.
        self._tier1_direction_history.append((n_front, n_rear))

        # v_ref at the current track position — used by the standing-
        # start guard inside emit_ellipse_saturation.
        idx_now = self._nearest_index(state.x, state.y)
        v_ref_here = float(self._v_ref_grid[idx_now])

        # Use the last-EMITTED command (held variables) as the rate-clip
        # anchor so the per-ODE-step Tier 1 emit ramps smoothly from the
        # last actual command. The MPC's internal ``_actuator_*``
        # tracking stays pinned to the last CLEAN MPC commit — pushing
        # it forward with the saturated Tier 1 values feeds an
        # ever-more-extreme x0 into the next QP solve and demonstrably
        # cascades into more primal-infeasibilities (Phase 5.0.3
        # build-time observation; documented in
        # ``docs/architecture-slip-model-phase5_0_3-v32-tier1.md``).
        # Phase 5.0.4: dynamic Fz at the chassis state right now so the
        # saturation honours the actual loaded / unloaded axle distribution
        # rather than the static-Fz one.
        Fz_f_now, Fz_r_now = self._current_dynamic_fz(state)
        ctrl, new_delta, new_thr_sat, new_brk_sat = emit_ellipse_saturation(
            state,
            n_front=n_front, n_rear=n_rear,
            pc=self.pc,
            delta_max=self.bounds.delta_max,
            delta_dot_max=self.bounds.delta_dot_max,
            throttle_dot_max=self.bounds.throttle_dot_max,
            brake_dot_max=self.bounds.brake_dot_max,
            tick_period=self._tick_period,
            last_commit=(
                self._held_steer_rad,
                self._held_throttle,
                self._held_brake,
            ),
            actuator_delta=self._held_steer_rad,
            actuator_throttle=self._held_throttle,
            actuator_brake=self._held_brake,
            v_ref=v_ref_here,
            saturation_safety=self.tier1.saturation_safety,
            fz_front_now=Fz_f_now,
            fz_rear_now=Fz_r_now,
        )
        # Phase 5.0.6 (Sprint A chicane fix): Tier 1 saturation OVERRIDES
        # only the STEERING channel. The longitudinal pedals stay with the
        # reactive sub-controller's trail-braking output.
        #
        # Root cause from the TIER-DIAG instrumented run (`--inertia-zz
        # 2400 --chicane-safety-mult 0.75/0.80/0.85`): the saturated
        # ``_invert_pedals`` mapping reduces the per-axle longitudinal
        # force target to "either full throttle OR full brake", because
        # the saturation projects the planned (Fx, Fy) onto the friction
        # ellipse along the planned unit direction — when the planned
        # direction has a non-trivial longitudinal-deceleration
        # component (corner entry), the rear-axle force lands on the
        # ellipse boundary with |Fx| ≈ D_long·Fz, which inverse-maps to
        # brake = 1.0 (full lockup). The chassis at handoff was logged
        # at ``brake=1.000`` while the reactive standalone was at
        # brake ≈ 0.5-0.7 (trail-braking with > 50% lateral allocation).
        # Full brake at 24-26 m/s + steering into the chicane is
        # ABS-locking + lateral-grip exhaustion = off-track.
        #
        # The reactive sub-controller's slip-band P-loop + measured
        # trail-brake taper produces the trail-braking the chicane
        # needs. Use ``sub_cmd.throttle`` / ``sub_cmd.brake`` directly;
        # Tier 1's value adds only to the steering channel where the
        # ellipse-saturation analytical solve genuinely helps.
        new_thr = float(sub_cmd.throttle)
        new_brk = float(sub_cmd.brake)
        # Track held variables only — actuator state stays at the
        # last clean MPC commit so the next QP's x0 has a clean
        # linearisation point.
        self._held_steer_rad = new_delta
        self._held_throttle = new_thr
        self._held_brake = new_brk
        return self._apply_consistency_noise(
            float(ctrl.steer_rad), new_thr, new_brk, t,
        )

    def _apply_consistency_noise(
        self, steer: float, throttle: float, brake: float, t: float,
    ) -> Controls:
        """Apply per-channel consistency noise (matches Phase 5.0 semantics)."""
        sigma = float(getattr(self.driver, "consistency_sigma", 0.0))
        if sigma > 0:
            steer_noise = math.radians(
                self.params.consistency_noise_std_steer_deg
            ) * float(self._rng.standard_normal())
            throttle_noise = (
                self.params.consistency_noise_std_throttle_pct / 100.0
            ) * float(self._rng.standard_normal())
            brake_noise = (
                self.params.consistency_noise_std_brake_pct / 100.0
            ) * float(self._rng.standard_normal())
            steer = float(np.clip(
                steer + steer_noise,
                -self.bounds.delta_max, self.bounds.delta_max,
            ))
            throttle = float(np.clip(throttle + throttle_noise, 0.0, 1.0))
            brake = float(np.clip(brake + brake_noise, 0.0, 1.0))
        self._last_t = t
        return Controls(
            steer_rad=float(steer),
            throttle=float(throttle),
            brake=float(brake),
        )

    # ------------------------------------------------------------------
    # MPC tick: project, build reference, solve, commit.
    # ------------------------------------------------------------------

    def _resolve_mpc(self, state: VehicleState, t: float) -> None:
        """Run one MPC tick: rebuild plant linearisation + solve + commit."""
        t0 = perf_counter()
        # 1. Nearest racing-line index + line frame (e_lat, e_psi).
        idx = self._nearest_index(state.x, state.y)
        s_here = float(self._ds[idx])
        tangent_here = self._line_tangent(idx)
        ln_x = float(self._xs[idx])
        ln_y = float(self._ys[idx])
        body_dx = float(state.x) - ln_x
        body_dy = float(state.y) - ln_y
        # Cross-track: positive => car to the LEFT of the line (Frenet
        # convention; matches the LTV bicycle dynamics
        # e_lat_dot = v_x*sin(e_psi) + v_y*cos(e_psi) where positive e_psi
        # = heading turned left of tangent, positive omega_yaw reduces both
        # e_psi and e_lat). Sign is the negative of the Stanley
        # cross = body_dx*sin(tan) - body_dy*cos(tan) used by the reactive
        # controller — there ``cross > 0 => right of line``.
        e_lat = -(body_dx * math.sin(tangent_here) - body_dy * math.cos(tangent_here))
        # Heading error.
        e_psi = math.atan2(
            math.sin(float(state.psi) - tangent_here),
            math.cos(float(state.psi) - tangent_here),
        )

        # 2. Reference kappa(s) and v_ref(s) over the horizon.
        s_seq = s_here + np.arange(self.n_stages) * self._ds_stage
        # Wrap s if it crosses the lap end (treat as flat extension; the
        # MPC ignores wraparound — at lap end we're effectively starting
        # over next lap).
        s_seq = np.clip(s_seq, 0.0, self._total_len - 1e-3)
        kappa_seq = self._kappa_at(s_seq)
        # The MPC tracks the raw plan v_ref; the longitudinal preview
        # (reactive split) drives the actual throttle / brake commit. We
        # still pass v_ref_seq to the QP so its w_v cost stays consistent
        # with the chassis speed the steering channel needs to plan
        # against (e.g. less steering authority at high v_x).
        v_ref_seq = np.interp(s_seq, self._ds, self._v_ref_grid)
        v_ref_seq = np.maximum(v_ref_seq, self.bounds.vx_min + 0.1)

        # 3. Build the MPC initial state x0.
        x0 = np.array([
            float(e_lat),
            float(e_psi),
            float(state.v_x),
            float(state.v_y),
            float(state.omega_yaw),
            float(self._actuator_delta),
            float(self._actuator_throttle),
            float(self._actuator_brake),
        ])

        # 4. Warm start u_seq from the previous tick (shift by one stage).
        u_seq_init = np.zeros_like(self._u_seq)
        u_seq_init[:-1] = self._u_seq[1:]
        u_seq_init[-1] = self._u_seq[-1]

        # 5. Solve.
        u_seq, stats = solve_sqp(
            x0,
            u_seq_init=u_seq_init,
            kappa_seq=kappa_seq,
            v_ref_seq=v_ref_seq,
            pc=self.pc,
            ds=self._ds_stage,
            weights=self.weights,
            bounds=self.bounds,
            alpha_max=self.bounds.alpha_axle_max,
            u_prev=self._u_seq[0],
            sqp_max_iter=self.sqp_max_iter,
            cimpcc_params=self.cimpcc_params,
        )
        solve_t = perf_counter() - t0
        self._solve_times.append(solve_t)
        # Surface the last OSQP status to the per-run counter.
        if stats.get("status_history"):
            last_status = str(stats["status_history"][-1])
            self._qp_status_counts[last_status] = (
                self._qp_status_counts.get(last_status, 0) + 1
            )

        # 6. Phase 5.0.3 tier classification (spec §23.2-5.0.3.3).
        # The classifier wraps:
        #   - hard infeasibility codes → Tier 1 immediately
        #   - soft divergence (cost residual / post-solve ellipse /
        #     SQP non-convergence) → Tier 1 on 2nd consecutive tick
        #   - clean → Tier 0
        baseline = (
            float(np.median(self._j_residual_history))
            if len(self._j_residual_history) > 0 else 0.0
        )
        verdict = classify_qp_status(
            stats,
            j_residual_baseline=baseline,
            j_residual_multiplier=self.tier1.j_residual_multiplier,
            sqp_max_iter=self.sqp_max_iter,
            sqp_du_inf_threshold=self.tier1.sqp_du_inf_threshold,
            ellipse_violation_threshold=self.tier1.ellipse_violation_threshold,
            pc=self.pc,
            ellipse_check_stages=self.tier1.ellipse_check_stages,
            # Phase 5.0.4: per-stage Fz from the final SQP iterate so
            # the post-solve violation check compares against the same
            # envelope the QP solved against.
            fz_front_per_stage=stats.get("fz_front_per_stage_last"),
            fz_rear_per_stage=stats.get("fz_rear_per_stage_last"),
        )
        post_solve_viol = float(
            stats.get("post_solve_ellipse_violation", 0.0)
        )

        if not self.tier1.enabled and verdict in ("hard", "soft"):
            # Tier 1 explicitly disabled (CLI flag) — fall straight to
            # the legacy reactive-fallback path. Counted as Tier 2.
            self._infeasible_ticks += 1
            self._latest_tick_tier = TIER_REACTIVE
            self._enter_tier2(t, reason="tier1-disabled-via-flag")
            # No actuator update: tier 2 emits via _long_sub.
            return

        if verdict == "hard":
            self._latest_tick_tier = TIER_ELLIPSE
            self._tier1_consecutive += 1
            self._soft_divergence_streak = 0
            self._infeasible_ticks += 1
            self._maybe_escalate_to_tier2(t, reason="hard-infeasible")
            if not self._warned_fallback and self._in_tier2:
                pass  # already warned by _enter_tier2.
            return

        if verdict == "soft":
            self._soft_divergence_streak += 1
            if self._soft_divergence_streak >= 2:
                # Treat as Tier 1.
                self._latest_tick_tier = TIER_ELLIPSE
                self._tier1_consecutive += 1
                self._maybe_escalate_to_tier2(t, reason="soft-divergence")
                return
            # First soft signal — absorb. Still commit the QP solution
            # as Tier 0 (since the QP "solved" or "solved inaccurate").
            # Fall through into the clean-commit path.

        # verdict == "clean" OR soft streak below threshold → commit MPC.
        if verdict == "clean":
            # Reset both streaks; record the J_residual for the rolling
            # baseline so the next ticks have a usable threshold.
            self._soft_divergence_streak = 0
            jr = float(stats.get("J_residual", 0.0))
            if jr > 0.0:
                self._j_residual_history.append(jr)
            self._post_solve_violations.append(post_solve_viol)
        # Tier 1 streak resets only on clean (per spec §23.2-5.0.3.6 —
        # "first tick that returns Tier-0-clean immediately switches
        # back"). A soft-with-streak-1 leaves it at zero too.
        if verdict == "clean":
            if self._tier1_consecutive > self._tier1_max_consecutive_observed:
                self._tier1_max_consecutive_observed = self._tier1_consecutive
            self._tier1_consecutive = 0

        self._latest_tick_tier = TIER_MPC
        self._u_seq = u_seq
        self._commit_clean_mpc(u_seq, v_ref_seq, state, x0)

    def _commit_clean_mpc(
        self,
        u_seq: np.ndarray,
        v_ref_seq: np.ndarray,
        state: VehicleState,
        x0: np.ndarray,
    ) -> None:
        """Apply rate clips to the first-stage commit + stash planned forces.

        Identical to the Phase 5.0 rate-clip pass that lived inline in
        the old ``_resolve_mpc``; extracted here so the tier classifier
        can fall through cleanly. Also caches the per-axle planned
        (F_x, F_y) so the next Tier 1 episode can use it as the primary
        planned direction (spec §23.2-5.0.3.4 candidate (a)).
        """
        ts0 = self._ds_stage / max(float(v_ref_seq[0]), 5.0)
        d_delta = float(u_seq[0, 0]) * ts0
        d_thr = float(u_seq[0, 1]) * ts0
        d_brk = float(u_seq[0, 2]) * ts0
        tick = self._tick_period
        cap_delta = self.bounds.delta_dot_max * tick
        cap_thr = self.bounds.throttle_dot_max * tick
        cap_brk = self.bounds.brake_dot_max * tick
        d_delta = float(np.clip(d_delta, -cap_delta, cap_delta))
        d_thr = float(np.clip(d_thr, -cap_thr, cap_thr))
        d_brk = float(np.clip(d_brk, -cap_brk, cap_brk))
        new_delta = float(np.clip(
            self._actuator_delta + d_delta,
            -self.bounds.delta_max, self.bounds.delta_max,
        ))
        new_throttle = float(np.clip(
            self._actuator_throttle + d_thr,
            0.0, 1.0,
        ))
        new_brake = float(np.clip(
            self._actuator_brake + d_brk,
            0.0, 1.0,
        ))
        # Standing-start soft start (matches Phase 5.0 line 560-561).
        if float(state.v_x) < 1.0 and float(v_ref_seq[0]) > 2.0:
            new_throttle = 1.0
            new_brake = 0.0
        self._actuator_delta = new_delta
        self._actuator_throttle = new_throttle
        self._actuator_brake = new_brake
        self._held_steer_rad = self._actuator_delta
        self._held_throttle = self._actuator_throttle
        self._held_brake = self._actuator_brake
        # Cache the planned per-axle force at the *committed* state
        # (chassis state x0 + actuator deltas) for Tier 1's planned
        # direction. Build a synthetic state-vec matching the
        # MPC's NX layout so compute_axle_force_from_state can read
        # the new actuator positions. Phase 5.0.3 (spec §23.2-5.0.3.4
        # candidate (a)).
        x_lin_committed = x0.copy()
        x_lin_committed[5] = new_delta     # IDX_DELTA
        x_lin_committed[6] = new_throttle  # IDX_THR
        x_lin_committed[7] = new_brake     # IDX_BRK
        self._prev_planned_axle_forces = compute_axle_force_from_state(
            x_lin_committed, self.pc,
        )
        # Tier history is tracked at clean commit time as well so the
        # next Tier 1 episode reads the right ``prev_tier``.
        self._prev_tier = TIER_MPC
        # Phase 5.0.5: clear the rolling-window direction buffer once
        # MPC re-engages cleanly. A fresh Tier-1 episode starts from
        # the bootstrap path (instantaneous direction) rather than
        # averaging in stale samples from the prior episode.
        self._tier1_direction_history.clear()

    def _maybe_escalate_to_tier2(self, t: float, *, reason: str) -> None:
        """If Tier 1 has fired ``max_consecutive_ticks`` in a row, go Tier 2.

        Resets the streak and marks ``_latest_tick_tier`` so the
        per-step ``controls()`` emits the reactive sub-controller.
        Updates the tier-1-episode counter when this is the first
        Tier 1 tick (the streak just stepped from 0 to 1).

        ``_prev_tier`` tracks the *previous* tick. On a streak step
        from 1 to 2, the *previous* tick was already Tier 1, so the
        blend kicks in; on the *first* Tier 1 tick (streak == 1)
        ``_prev_tier`` is still TIER_MPC from the last clean commit.
        We set it to TIER_ELLIPSE only after the FIRST tick — i.e.
        when streak >= 2 — to honour the spec's "if last tick was
        already a fallback, blend" semantics (§23.2-5.0.3.4 candidate
        (d)).
        """
        # Record the episode boundary when the streak ticks up from 0.
        if self._tier1_consecutive == 1:
            self._tier1_episodes += 1
        if self._tier1_consecutive > self._tier1_max_consecutive_observed:
            self._tier1_max_consecutive_observed = self._tier1_consecutive
        if self._tier1_consecutive > int(self.tier1.max_consecutive_ticks):
            # Escalate.
            self._latest_tick_tier = TIER_REACTIVE
            self._enter_tier2(t, reason=f"tier1-escalation-{reason}")
            self._tier1_consecutive = 0
            self._prev_tier = TIER_REACTIVE
        elif self._tier1_consecutive >= 2:
            # The PRIOR tick was Tier 1 too — engage the stale-MPC +
            # Stanley blend for this and subsequent emits.
            self._prev_tier = TIER_ELLIPSE
        # else (streak == 1): leave _prev_tier as TIER_MPC so the
        # first emit uses pure stale-MPC direction.

    # ------------------------------------------------------------------
    # Track-line geometry helpers — thin delegators to mpc_controller_geom.
    # ------------------------------------------------------------------

    def _nearest_index(self, x: float, y: float) -> int:
        return nearest_index(self._xs, self._ys, x, y, self._geom_state)

    def _line_tangent(self, idx: int) -> float:
        return line_tangent(self._xs, self._ys, idx)

    def _kappa_at(self, s_seq: np.ndarray) -> np.ndarray:
        return kappa_at(s_seq, self._ds, self._radius_m, self._xs, self._ys)


__all__ = ["MPCController"]
