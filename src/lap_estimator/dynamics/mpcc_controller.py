"""v3.3 MPCC controller (spec §23.3.6.6).

Sibling of :class:`MPCController`. Operates in curvilinear (s, n, ψ_e)
coordinates along a fixed reference path; rewards progress while penalising
contour + lag error. Steering channel only — throttle / brake are delegated
to the embedded reactive ``DriverController`` (the ``_long_sub`` pattern
inherited from v3.2 — spec §23.3.3, deferred slip-aware longitudinal MPC).

Tier ladder (spec §23.3.6.6):

- **Tier 0** — clean MPCC commit. Held steering between MPCC ticks.
- **Tier 2** — reactive fallback. Triggered by chassis-state divergence
  (cross-track > 4 m or |ψ_e| > 20°), QP infeasibility, or a NaN/clip on the
  curvilinear projection. Hysteresis: 3 consecutive clean ticks before MPCC
  re-engages.

Tier 1 (Phase 5.0.3 ellipse-saturation feedforward) is **not** ported — the
curvilinear coordinate transform makes the v3.2 planned-direction logic
non-trivial to migrate. The progress reward in MPCC's cost shape should
remove the need for it; if MC completions fall short, porting Tier 1 is the
first fallback per spec §23.3.11 risk #3.

Diagnostics surface (spec §23.3.6.7):

- ``solve_times``, ``tier_counts`` — same property names as MPCController so
  the existing :class:`SlipSimResult` plumbing picks them up without per-
  controller branching.
- ``contour_p95``, ``lag_p95``, ``progress_mean`` — MPCC-specific terminals
  read by :mod:`slip_simulator` under an ``isinstance`` guard.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from time import perf_counter
from typing import TYPE_CHECKING

import numpy as np

from ._control_params import ControlParams
from .driver_controller import DriverController
from .longitudinal_planner import LongitudinalPlan
from .mpc_model import PlantConstants, build_plant_constants
from .mpc_physics import MpcPhysicsConfig
from .mpcc_model import (
    IDX_BRK,
    IDX_DELTA,
    IDX_E_PSI,
    IDX_N,
    IDX_OMEGA,
    IDX_S,
    IDX_THETA,
    IDX_THR,
    IDX_VX,
    IDX_VY,
    NU_C,
    NX_C,
)
from .mpcc_qp import MPCCBounds, MPCCWeights, solve_sqp_mpcc
from .mpcc_reference import (
    DEFAULT_EDGE_SAFETY,
    DEFAULT_TRACK_HALF_WIDTH,
    build_reference_path,
    sample_seq,
    signed_shortest,
    to_curvilinear,
)
from .vehicle import Controls, PacejkaCalibration, VehicleState, load_car_dynamics

if TYPE_CHECKING:
    from ..car import Car
    from ..driver import Driver
    from ..track import Track

log = logging.getLogger(__name__)


# Tier markers (re-used IDs from v3.2 so the SlipSimResult tier_counts dict
# is interpretable across controllers).
TIER_MPCC = 0
TIER_REACTIVE = 2


# Spec §23.3.6.4 defaults (Liniger regime; spec §23.3.10.1 tick rate decision).
DEFAULT_HORIZON_M = 50.0
DEFAULT_N_STAGES = 25
DEFAULT_TICK_HZ = 10.0
DEFAULT_SQP_MAX_ITER = 3

# Tier-2 hysteresis (spec §23.3.6.6): re-engage MPCC after this many clean
# ticks following a Tier-2 episode.
N_TIER2_RECOVERY_HYSTERESIS = 3


def _resolve_mpcc_block(driver: "Driver") -> dict:
    """Return the optional ``control_params.mpcc`` JSON block, or ``{}``."""
    raw = getattr(driver, "raw", None)
    if not isinstance(raw, dict):
        return {}
    cp = raw.get("control_params") or {}
    if not isinstance(cp, dict):
        return {}
    block = cp.get("mpcc")
    return block if isinstance(block, dict) else {}


@dataclass
class _DiagSample:
    """Per-tick MPCC-specific diagnostics."""

    n_now: float
    lag_now: float
    v_theta: float


class MPCCController:
    """Receding-horizon MPCC controller (spec §23.3).

    Parameters mirror :class:`MPCController` so :func:`_make_controller`
    dispatch only needs one new branch.
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
        w_contour: float | None = None,
        w_lag: float | None = None,
        w_progress: float | None = None,
        w_du: float | None = None,
        w_du2: float | None = None,
        w_term: float | None = None,
        force_static_fz: bool = False,
        enable_ellipse: bool = True,
        line_xs: np.ndarray | None = None,
        line_ys: np.ndarray | None = None,
    ) -> None:
        if not getattr(track, "is_csv_backed", False):
            raise ValueError("MPCCController needs a CSV-backed track.")
        self.driver = driver
        self.car = car
        self.params = params if params is not None else ControlParams.from_driver(driver)
        self.calib = calib
        self.plan = plan

        mpcc_block = _resolve_mpcc_block(driver)

        # Horizon / tick / SQP iters.
        self.horizon_m = float(
            horizon_m if horizon_m is not None
            else mpcc_block.get("horizon_m", DEFAULT_HORIZON_M)
        )
        self.n_stages = int(
            n_stages if n_stages is not None
            else mpcc_block.get("n_stages", DEFAULT_N_STAGES)
        )
        self.tick_hz = float(
            tick_hz if tick_hz is not None
            else mpcc_block.get("tick_hz", DEFAULT_TICK_HZ)
        )
        self.sqp_max_iter = int(
            sqp_max_iter if sqp_max_iter is not None
            else mpcc_block.get("sqp_max_iter", DEFAULT_SQP_MAX_ITER)
        )
        if not (1 <= self.n_stages <= 60):
            raise ValueError(f"mpcc.n_stages={self.n_stages} out of [1, 60]")
        if not (1 <= self.tick_hz <= 100):
            raise ValueError(f"mpcc.tick_hz={self.tick_hz} out of [1, 100]")
        if not (5.0 <= self.horizon_m <= 200.0):
            raise ValueError(f"mpcc.horizon_m={self.horizon_m} out of [5, 200]")

        # Cost weights (spec §23.3.7.3).
        self.weights = MPCCWeights(
            w_contour=float(w_contour if w_contour is not None
                            else mpcc_block.get("w_contour", 100.0)),
            w_lag=float(w_lag if w_lag is not None
                        else mpcc_block.get("w_lag", 1000.0)),
            w_progress=float(w_progress if w_progress is not None
                             else mpcc_block.get("w_progress", 2.0)),
            w_du=float(w_du if w_du is not None
                       else mpcc_block.get("w_du", 1.0)),
            w_du2=float(w_du2 if w_du2 is not None
                        else mpcc_block.get("w_du2", 1.0)),
            w_term=float(w_term if w_term is not None
                         else mpcc_block.get("w_term", 100.0)),
        )

        # Build the reference path (spec §23.3.6.1). Optional ``line_xs/ys``
        # override (Tomas-line experiment, 2026-05-24) replaces the
        # centreline ``(x, z)`` with a user-supplied racing line; the
        # tangent / curvature recompute from the override path.
        _line_override = None
        if line_xs is not None and line_ys is not None:
            _line_override = (
                np.asarray(line_xs, dtype=float),
                np.asarray(line_ys, dtype=float),
            )
        self.ref = build_reference_path(
            track, plan,
            track_half_width_default=DEFAULT_TRACK_HALF_WIDTH,
            line_override=_line_override,
        )

        # Hard bounds. V_θ_max = 1.5 · max(v_ref) as a global ceiling, but
        # the per-stage v_theta_max_seq passed at solve time tightens this
        # to 1.1 × v_ref(s_predicted_k) so the progress reward can't push
        # the virtual progress arbitrarily far past what the plan permits.
        # v_theta_min = 0 keeps the lag cost monotone (spec §23.3.11 risk #2).
        v_ref_max = float(np.max(self.ref.v_ref)) if len(self.ref.v_ref) else 50.0
        self.bounds = MPCCBounds(
            delta_max=math.radians(20.0),
            delta_dot_max=8.0,
            throttle_dot_max=5.0,
            brake_dot_max=5.0,
            v_theta_min=0.0,
            v_theta_max=1.5 * max(v_ref_max, 10.0),
            vx_min=0.5,
            edge_safety_buffer=DEFAULT_EDGE_SAFETY,
        )

        # Slip target — same v3.1 / v3.2 mapping. Held for SlipSimResult.
        if slip_target_rad_override is not None:
            self.slip_target_rad = float(slip_target_rad_override)
        elif self.params.slip_target_deg is not None:
            self.slip_target_rad = math.radians(float(self.params.slip_target_deg))
        else:
            self.slip_target_rad = math.radians(float(driver.derived_slip_target_deg()))

        # Alpha-peak heuristic (kept identical to MPCController so the
        # plant constants land on the same operating point).
        apf_deg = None
        try:
            apf_deg = driver._alpha_peak_front_deg()  # noqa: SLF001
        except Exception:  # noqa: BLE001
            apf_deg = None
        if apf_deg is None or apf_deg <= 0:
            self.alpha_peak_rad = self.slip_target_rad / max(0.5, 0.5 + 0.5 * float(driver.skill_pct))
        else:
            apf_clamped = max(4.0, min(10.0, float(apf_deg)))
            self.alpha_peak_rad = math.radians(apf_clamped)

        v_ref_avg = (
            float(np.mean(plan.speeds[: len(plan.speeds) // 2 + 1]))
            if len(plan.speeds) else 30.0
        )
        self.dyn = load_car_dynamics(car)
        skill_factor = max(0.5, 0.5 + 0.5 * float(driver.skill_pct))
        alpha_op_front_rad = 0.5 * float(self.alpha_peak_rad) * skill_factor
        alpha_op_rear_rad = 0.5 * float(self.alpha_peak_rad) * skill_factor

        self.physics = MpcPhysicsConfig.from_mpc_block(mpcc_block)
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
        log.info(
            "MPCC plant: dynamic-Fz=%s, h_cg=%.3f, "
            "horizon=%.1f m / %d stages @ %.1f Hz; weights w_contour=%.1f "
            "w_lag=%.1f w_progress=%.2f w_du=%.2f",
            self.physics.dynamic_fz_enabled, self.pc.h_cg,
            self.horizon_m, self.n_stages, self.tick_hz,
            self.weights.w_contour, self.weights.w_lag,
            self.weights.w_progress, self.weights.w_du,
        )

        # Stage gridding in dθ (spec §23.3.6.4).
        self._tick_period = 1.0 / self.tick_hz
        self._d_theta_stage = self.horizon_m / self.n_stages

        # Warm-start u_seq. V_θ_0 seed = v_ref at start position.
        self._u_seq = np.zeros((self.n_stages, NU_C))
        self._u_seq[:, 3] = float(self.ref.v_ref[0]) if len(self.ref.v_ref) else 20.0
        self._last_solve_t = -1.0
        self._held_steer_rad = 0.0
        self._actuator_delta = 0.0
        self._actuator_throttle = 0.0
        self._actuator_brake = 0.0

        # Theta state — virtual progress carried across ticks.
        self._theta = 0.0

        # Index hint for the nearest-s projection.
        self._proj_hint = 0

        # Diagnostics.
        self._solve_times: list[float] = []
        self._tier_counts: dict[int, int] = {TIER_MPCC: 0, TIER_REACTIVE: 0}
        self._diag_samples: list[_DiagSample] = []
        self._qp_status_counts: dict[str, int] = {}
        self._post_solve_violations: list[float] = []
        self._latest_tick_tier: int = TIER_MPCC
        self._in_tier2: bool = False
        self._tier2_recovery_streak: int = 0
        self._tier2_episodes: int = 0
        self._warned_fallback = False

        # RNG (for the consistency-noise pass).
        self._rng = np.random.default_rng(rng_seed if rng_seed is not None else 0)
        self._rng_seed = rng_seed

        # Reactive sub-controller for throttle/brake (same plumbing as
        # MPCController — softener disabled, no double-noise).
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
            # Forward the line override so the longitudinal sub-controller
            # projects onto the same line MPCC tracks (Tomas-line
            # experiment, 2026-05-24).
            line_xs=line_xs,
            line_ys=line_ys,
        )
        # Whether to attach the friction-ellipse hard constraint (re-used
        # from v3.2). Off only by explicit toggle for A/B experiments.
        # Build-time observation (2026-05-24): the ellipse constraint
        # **stabilises the QP at corner entries** even though its
        # re-shaping via a v3.2 (Phi, g) shim adds complexity (see
        # `mpcc_qp._phi_g_to_v32_view`). Disabling it makes the lap abort
        # ~300 m earlier at the s≈350 m fast left-hander (Tier-2 share
        # jumps to ~73 %). Default ON.
        self._enable_ellipse = bool(enable_ellipse)

    # ------------------------------------------------------------------
    # Public surface.
    # ------------------------------------------------------------------

    @property
    def solve_times(self) -> list[float]:
        return list(self._solve_times)

    @property
    def tier_counts(self) -> dict[int, int]:
        return dict(self._tier_counts)

    @property
    def ghost_step_count(self) -> int:
        return self._tier_counts.get(TIER_REACTIVE, 0)

    # MPCC-specific.
    @property
    def contour_p95(self) -> float:
        if not self._diag_samples:
            return 0.0
        ns = np.asarray([abs(s.n_now) for s in self._diag_samples])
        return float(np.quantile(ns, 0.95))

    @property
    def lag_p95(self) -> float:
        if not self._diag_samples:
            return 0.0
        ls = np.asarray([abs(s.lag_now) for s in self._diag_samples])
        return float(np.quantile(ls, 0.95))

    @property
    def progress_mean(self) -> float:
        if not self._diag_samples:
            return 0.0
        ps = np.asarray([s.v_theta for s in self._diag_samples])
        return float(np.mean(ps))

    @property
    def qp_status_counts(self) -> dict[str, int]:
        return dict(self._qp_status_counts)

    @property
    def post_solve_ellipse_violation_p95(self) -> float:
        if not self._post_solve_violations:
            return 0.0
        return float(np.quantile(np.asarray(self._post_solve_violations), 0.95))

    @property
    def tier1_episodes(self) -> int:
        return 0  # Tier 1 is not ported; spec §23.3.6.6.

    @property
    def tier1_max_consecutive_steps(self) -> int:
        return 0

    @property
    def tier2_episodes(self) -> int:
        return self._tier2_episodes

    # ------------------------------------------------------------------
    # Main step.
    # ------------------------------------------------------------------

    def controls(
        self, state: VehicleState, t: float, track: "Track | None" = None,  # noqa: ARG002
    ) -> Controls:
        """Emit per-step :class:`Controls` using the MPCC tier ladder."""
        # 1. MPCC tick boundary.
        if (t - self._last_solve_t) >= self._tick_period:
            self._resolve_mpcc(state, t)
            self._last_solve_t = t

        # 2. Chassis-state divergence → Tier 2.
        diverged = self._chassis_diverged(state)
        if diverged:
            self._enter_tier2(t, reason="chassis-divergence")
            self._record_tier(TIER_REACTIVE)
            return self._long_sub.controls(state, t)

        # 3. Tier 2 hysteresis-exit window.
        if self._in_tier2:
            if self._latest_tick_tier == TIER_MPCC:
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

        # 4. Tier 0 — held MPCC steering + reactive longitudinal.
        self._record_tier(TIER_MPCC)
        return self._emit_tier0_controls(state, t)

    # ------------------------------------------------------------------
    # Tier emit / state helpers.
    # ------------------------------------------------------------------

    def _emit_tier0_controls(self, state: VehicleState, t: float) -> Controls:
        steer = self._held_steer_rad
        sub_cmd = self._long_sub.controls(state, t)
        return self._apply_consistency_noise(
            steer, float(sub_cmd.throttle), float(sub_cmd.brake), t,
        )

    def _chassis_diverged(self, state: VehicleState) -> bool:
        s_here, n_here, e_psi_here, _ = to_curvilinear(
            float(state.x), float(state.y), float(state.psi),
            self.ref, hint_idx=self._proj_hint,
        )
        if abs(n_here) > 4.0:
            return True
        if abs(e_psi_here) > math.radians(20.0):
            return True
        return False

    def _enter_tier2(self, t: float, *, reason: str) -> None:
        if not self._in_tier2:
            self._tier2_episodes += 1
            self._in_tier2 = True
        self._tier2_recovery_streak = 0
        if not self._warned_fallback:
            log.warning(
                "MPCCController -> Tier 2 reactive at t=%.2fs (%s). "
                "Subsequent fallbacks silent.",
                t, reason,
            )
            self._warned_fallback = True

    def _record_tier(self, tier: int) -> None:
        self._tier_counts[tier] = self._tier_counts.get(tier, 0) + 1

    def _apply_consistency_noise(
        self, steer: float, throttle: float, brake: float, t: float,  # noqa: ARG002
    ) -> Controls:
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
        return Controls(
            steer_rad=float(steer),
            throttle=float(throttle),
            brake=float(brake),
        )

    # ------------------------------------------------------------------
    # MPCC tick.
    # ------------------------------------------------------------------

    def _resolve_mpcc(self, state: VehicleState, t: float) -> None:
        """Run one MPCC tick: project + build reference + solve + commit."""
        t0 = perf_counter()

        # 1. Project chassis → curvilinear.
        s_here, n_here, e_psi_here, idx_hint = to_curvilinear(
            float(state.x), float(state.y), float(state.psi),
            self.ref, hint_idx=self._proj_hint,
        )
        self._proj_hint = idx_hint

        # 2. Re-sync theta to s every tick. Spec §23.3.6 expects θ as a
        # virtual progress variable carried across ticks, but in practice
        # the linearised plant cannot prevent θ from running ahead of s
        # without a hand-tuned w_progress / V_θ_max combination. Re-syncing
        # θ = s at the start of every tick reduces the OCP to a pure
        # contour + (predicted lag accumulation over horizon) cost: the
        # lag cost binds V_θ to ≈ chassis ds/dt within the horizon, the
        # progress reward pushes for the maximum sustainable progress, and
        # the chassis-state integration determines actual lap progress.
        # Equivalent to Liniger's "θ initialised by Frenet projection at
        # every solve" in his MATLAB MPCC code. Re-sync also applies after
        # a Tier-2 episode (otherwise θ would carry a stale offset).
        self._theta = s_here

        # 3. Build the per-stage reference profile based on theta_predicted.
        # Use a constant-V_θ extrapolation seeded from the warm-start V_θ for
        # the kappa / v_ref / half_width lookup; the SQP loop re-grids
        # internally as V_θ evolves.
        # theta_predicted_k = theta_0 + k * d_theta_stage
        theta_seq = self._theta + np.arange(self.n_stages) * self._d_theta_stage
        theta_seq = np.mod(theta_seq, max(self.ref.total_length, 1.0))
        kappa_seq, v_ref_seq, half_width_seq = sample_seq(theta_seq, self.ref)
        v_ref_seq = np.maximum(v_ref_seq, self.bounds.vx_min + 0.1)
        # Per-stage cap on V_θ — keeps the progress reward from running
        # virtual progress past the planner's max-feasible speed at the
        # predicted θ (spec §23.3.11 risk #5, also #2).
        v_theta_max_seq = np.minimum(1.1 * v_ref_seq, self.bounds.v_theta_max)

        # 4. Build initial state x0 (length NX_C).
        x0 = np.array([
            float(s_here),
            float(n_here),
            float(e_psi_here),
            float(state.v_x),
            float(state.v_y),
            float(state.omega_yaw),
            float(self._actuator_delta),
            float(self._actuator_throttle),
            float(self._actuator_brake),
            float(self._theta),
        ], dtype=float)

        # 5. Warm-start u_seq (shift by one).
        u_seq_init = np.zeros_like(self._u_seq)
        u_seq_init[:-1] = self._u_seq[1:]
        u_seq_init[-1] = self._u_seq[-1]

        # 6. Solve.
        u_seq, stats = solve_sqp_mpcc(
            x0,
            u_seq_init=u_seq_init,
            ref=self.ref,
            v_ref_seq=v_ref_seq,
            half_width_seq=half_width_seq,
            pc=self.pc,
            ds_stage=self._d_theta_stage,
            weights=self.weights,
            bounds=self.bounds,
            u_prev=self._u_seq[0],
            sqp_max_iter=self.sqp_max_iter,
            enable_ellipse=self._enable_ellipse,
            v_theta_max_seq=v_theta_max_seq,
        )
        solve_t = perf_counter() - t0
        self._solve_times.append(solve_t)

        if stats.get("status_history"):
            last_status = str(stats["status_history"][-1])
            self._qp_status_counts[last_status] = (
                self._qp_status_counts.get(last_status, 0) + 1
            )

        infeasible = bool(stats.get("infeasible_recovery", False))
        if infeasible:
            # Don't commit; route emit through Tier 2.
            self._latest_tick_tier = TIER_REACTIVE
            self._enter_tier2(t, reason="qp-infeasible")
            return

        # 7. Commit the first-stage rate-controls.
        self._u_seq = u_seq
        self._commit_clean(u_seq, state, x0, v_ref_seq[0])

        # 8. Diagnostics.
        self._latest_tick_tier = TIER_MPCC
        V_theta_now = float(u_seq[0, 3])
        # Lag = signed-shortest (s − θ).
        lag_now = signed_shortest(s_here - self._theta, self.ref.total_length)
        self._diag_samples.append(_DiagSample(
            n_now=float(n_here),
            lag_now=float(lag_now),
            v_theta=float(V_theta_now),
        ))
        # Build-time debug at the first three ticks (visible at log level INFO).
        if len(self._solve_times) <= 3:
            log.info(
                "MPCC tick %d: s=%.2f θ=%.2f n=%.3f e_psi=%.3f v_x=%.2f "
                "V_θ_committed=%.2f v_theta_max_seq[0]=%.2f",
                len(self._solve_times),
                s_here, self._theta, n_here, e_psi_here, state.v_x,
                V_theta_now, float(v_theta_max_seq[0]),
            )

    def _commit_clean(
        self,
        u_seq: np.ndarray,
        state: VehicleState,
        x0: np.ndarray,
        v_ref_first: float,
    ) -> None:
        """Apply rate-clips and integrate u_seq[0] into the actuator state."""
        # Stage time for the first stage (V_θ or v_ref).
        V_theta_0 = max(float(u_seq[0, 3]), 5.0)
        ts0 = self._d_theta_stage / max(V_theta_0, 5.0)
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
            self._actuator_throttle + d_thr, 0.0, 1.0,
        ))
        new_brake = float(np.clip(
            self._actuator_brake + d_brk, 0.0, 1.0,
        ))
        # Standing-start soft start (matches MPCController). If we're crawling
        # below 1 m/s while v_ref says > 2 m/s, force throttle to 1.0.
        if float(state.v_x) < 1.0 and float(v_ref_first) > 2.0:
            new_throttle = 1.0
            new_brake = 0.0
        self._actuator_delta = new_delta
        self._actuator_throttle = new_throttle
        self._actuator_brake = new_brake
        self._held_steer_rad = new_delta
        # Advance theta by V_θ_0 over the tick period (matches the inner
        # explicit-Euler integration of θ).
        V_theta_first = max(float(u_seq[0, 3]), 0.0)
        self._theta = self._theta + V_theta_first * tick
        # Wrap modulo track length so the next projection lands in-range.
        if self.ref.total_length > 0:
            self._theta = self._theta % self.ref.total_length


__all__ = ["MPCCController"]
