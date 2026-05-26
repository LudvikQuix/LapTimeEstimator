"""Phase 5.0.3 Tier-1 ellipse-saturation feedforward helpers (spec §23.2-5.0.3).

This module hosts the pure-function machinery the :class:`MPCController`
uses for its three-tier fallback ladder:

- :func:`classify_qp_status` — read the OSQP/SQP stats payload from
  :func:`mpc_qp.solve_sqp` and return one of ``"clean"``, ``"hard"``,
  ``"soft"``. Hard / soft are the two trigger classes spec'd in
  §23.2-5.0.3.3; the caller (:class:`MPCController`) applies the
  two-tick hysteresis on the soft branch.
- :func:`compute_planned_direction` — resolve the unit (Fx, Fy) per
  axle the Tier 1 saturation will aim at (spec §23.2-5.0.3.4).
- :func:`emit_ellipse_saturation` — invert (Fx_ff, Fy_ff) per axle back
  into (δ, throttle, brake) via the Phase 5.0.1 affine Pacejka, apply
  the rate clip, and return a :class:`Controls`.

Extracted from :mod:`mpc_controller` to keep that module under the 500-
line soft cap (CLAUDE.md golden rule). All helpers are pure: state is
passed in / out explicitly, no module-level mutable state.

Tier numbering (spec §23.2-5.0.3.5):
- ``TIER_MPC = 0`` — clean MPC commit (the happy path).
- ``TIER_ELLIPSE = 1`` — saturation feedforward (this module).
- ``TIER_REACTIVE = 2`` — reactive sub-controller (chassis-state
  divergence OR ≥N_TIER1_CONSECUTIVE_MAX Tier 1 ticks).

Build-time decisions (spec §23.2-5.0.3.12 open questions resolved here
and documented in ``docs/architecture-slip-model-phase5_0_3-v32-tier1.md``):

- **Q1 (state-as-is vs half-tick rollforward):** use ``state`` as-is.
  Simpler, no extra plant roll, and the 20 ms tick period is short
  enough that the half-tick lag is < 10 ms of chassis motion. If
  diagnostics show oscillation, the rollforward is a 10-line patch.
"""

from __future__ import annotations

import math

import numpy as np

from .mpc_model import (
    IDX_BRK,
    IDX_DELTA,
    IDX_OMEGA,
    IDX_THR,
    IDX_VX,
    IDX_VY,
    NX,
    PlantConstants,
    compute_axle_force_from_state,
)
from .vehicle import Controls

# Tier-level integer constants. Used as keys in the tier-count dict on
# SlipSimResult so a `0`/`1`/`2` mapping is intelligible without an enum.
TIER_MPC = 0
TIER_ELLIPSE = 1
TIER_REACTIVE = 2


# --- Hard / soft status classification -----------------------------------


# OSQP status strings that trigger Tier 1 immediately (spec §23.2-5.0.3.3).
# Includes the inaccurate variants because OSQP's "inaccurate" path
# (relaxed eps_abs/eps_rel) still implies the original solve was at the
# edge of feasibility.
HARD_STATUS_CODES: frozenset[str] = frozenset(
    {
        "primal infeasible",
        "dual infeasible",
        "primal infeasible inaccurate",
        "dual infeasible inaccurate",
        "non-convex",
        "non_convex",
    }
)

# Statuses considered "clean" — the QP solved within OSQP's eps_abs/eps_rel.
CLEAN_STATUS_CODES: frozenset[str] = frozenset(
    {"solved", "solved inaccurate"}
)


def classify_qp_status(
    stats: dict,
    *,
    j_residual_baseline: float,
    j_residual_multiplier: float,
    sqp_max_iter: int,
    sqp_du_inf_threshold: float,
    ellipse_violation_threshold: float,
    pc: PlantConstants,
    ellipse_check_stages: int,
    fz_front_per_stage: np.ndarray | None = None,
    fz_rear_per_stage: np.ndarray | None = None,
) -> str:
    """Return one of ``"clean"``, ``"hard"``, ``"soft"`` per spec §23.2-5.0.3.3.

    Parameters
    ----------
    stats : dict
        Output of :func:`mpc_qp.solve_sqp`. Reads ``status_history``,
        ``infeasible_recovery``, ``J_residual``, ``sqp_du_inf``,
        ``iters``, ``x_seq_last``.
    j_residual_baseline : float
        Rolling median of recent clean-tick ``J_residual`` values.
        Caller maintains this; we just compare.
    j_residual_multiplier : float
        ``J_residual > j_residual_multiplier × j_residual_baseline``
        ⇒ soft (spec §23.2-5.0.3.3 detection 3).
    sqp_max_iter : int
        From the MPC config. SQP non-convergence (detection 5) requires
        the loop hit this limit without ``||Δu||_∞`` dropping below the
        threshold.
    sqp_du_inf_threshold : float
        Below this value the SQP is considered converged. Above it
        after hitting ``sqp_max_iter`` ⇒ soft.
    ellipse_violation_threshold : float
        Post-solve nonlinear ellipse residual cap. Detection 4 fires when
        ``max_k g_k > threshold`` (default 0.15 = 15 % above linearised
        envelope).
    pc : PlantConstants
        Used to evaluate the nonlinear ellipse on the rolled trajectory.
    ellipse_check_stages : int
        How many post-solve stages to evaluate (defaults to 4 at the spec
        default). Capped at the rolled-trajectory length.
    """
    history = stats.get("status_history") or []
    if not history:
        # No QP solve happened — controller is between ticks. Tier
        # holds; the caller treats this as "no change".
        return "clean"
    last = history[-1]
    # 1. Hard infeasibility codes (immediate).
    if last in HARD_STATUS_CODES:
        return "hard"
    # 2. Max-iter after a slip-bump retry (structural; spec
    # §23.2-5.0.3.3 detection 2). The recovery flag is set inside
    # solve_sqp once the bump fires.
    if (
        last == "max_iter"
        and bool(stats.get("infeasible_recovery", False))
    ):
        return "hard"
    if last not in CLEAN_STATUS_CODES:
        # Any other unexpected status ("max_iter" without recovery, an
        # OSQP version producing a code we don't recognise) maps to
        # soft so the controller still gets one transient absorbed
        # before flipping to Tier 1.
        return "soft"

    # ---- Soft divergence checks (post-clean-solve) ----
    j_residual = float(stats.get("J_residual", 0.0))
    if (
        j_residual_baseline > 0.0
        and j_residual > j_residual_multiplier * j_residual_baseline
    ):
        return "soft"

    # Post-solve nonlinear ellipse check (detection 4). Reuse the
    # rolled trajectory from solve_sqp; if the QP didn't carry it
    # (older path), skip silently. Phase 5.0.4: per-stage Fz arrays
    # are passed through so the residual matches the dynamic-Fz
    # envelope the QP saw.
    x_seq = stats.get("x_seq_last")
    if x_seq is not None and len(x_seq) > 1:
        kmax = min(int(ellipse_check_stages), len(x_seq) - 1)
        fz_f_w = (
            np.asarray(fz_front_per_stage, dtype=float)[:kmax]
            if fz_front_per_stage is not None
            else None
        )
        fz_r_w = (
            np.asarray(fz_rear_per_stage, dtype=float)[:kmax]
            if fz_rear_per_stage is not None
            else None
        )
        viol = _max_ellipse_residual(
            x_seq[: kmax + 1], pc,
            fz_front_per_stage=fz_f_w,
            fz_rear_per_stage=fz_r_w,
        )
        # cache on stats for telemetry surfacing.
        stats["post_solve_ellipse_violation"] = float(viol)
        if viol > ellipse_violation_threshold:
            return "soft"
    else:
        stats["post_solve_ellipse_violation"] = 0.0

    # SQP non-convergence (detection 5).
    if (
        int(stats.get("iters", 0)) >= sqp_max_iter
        and float(stats.get("sqp_du_inf", 0.0)) > sqp_du_inf_threshold
    ):
        return "soft"

    return "clean"


def _max_ellipse_residual(
    x_seq_window: np.ndarray,
    pc: PlantConstants,
    *,
    fz_front_per_stage: np.ndarray | None = None,
    fz_rear_per_stage: np.ndarray | None = None,
) -> float:
    """Return ``max_k max_axle ((F_x/(D_long·Fz))² + (F_y/(D_lat·Fz))² − 1)``.

    Evaluated on the nonlinear-rolled trajectory ``x_seq_window`` from
    :func:`mpc_qp.integrate_reference`. Cheap: 8 Pacejka evaluations +
    8 ellipse residuals per tick at the default 4-stage window.

    Phase 5.0.4: when per-stage Fz arrays are provided, the envelope at
    each stage uses the dynamic-Fz value the QP just solved against;
    otherwise the static ``pc.Fz_front`` / ``pc.Fz_rear`` are used
    (pre-Phase-5.0.4 behaviour).
    """
    D_long = float(pc.D_long)
    D_lat_f = float(pc.D_lat_front)
    D_lat_r = float(pc.D_lat_rear)
    static_Fz_f = float(pc.Fz_front)
    static_Fz_r = float(pc.Fz_rear)
    worst = 0.0
    for k in range(1, len(x_seq_window)):
        if (
            fz_front_per_stage is not None
            and fz_rear_per_stage is not None
            and k - 1 < len(fz_front_per_stage)
            and k - 1 < len(fz_rear_per_stage)
        ):
            Fz_f_k = float(fz_front_per_stage[k - 1])
            Fz_r_k = float(fz_rear_per_stage[k - 1])
        else:
            Fz_f_k = static_Fz_f
            Fz_r_k = static_Fz_r
        D_long_F_f2 = max(D_long * Fz_f_k, 1.0) ** 2
        D_long_F_r2 = max(D_long * Fz_r_k, 1.0) ** 2
        D_lat_F_f2 = max(D_lat_f * Fz_f_k, 1.0) ** 2
        D_lat_F_r2 = max(D_lat_r * Fz_r_k, 1.0) ** 2
        (Fx_f, Fy_f), (Fx_r, Fy_r) = compute_axle_force_from_state(
            x_seq_window[k], pc,
        )
        g_f = (Fx_f * Fx_f) / D_long_F_f2 + (Fy_f * Fy_f) / D_lat_F_f2 - 1.0
        g_r = (Fx_r * Fx_r) / D_long_F_r2 + (Fy_r * Fy_r) / D_lat_F_r2 - 1.0
        worst = max(worst, g_f, g_r)
    return float(worst)


# --- Planned-direction resolution ----------------------------------------


def compute_planned_direction(
    prev_tier: int,
    prev_planned_forces: tuple[tuple[float, float], tuple[float, float]] | None,
    stanley_forces: tuple[tuple[float, float], tuple[float, float]] | None,
    *,
    direction_blend_stale_alpha: float = 0.5,
    direction_history: (
        "list[tuple[tuple[float, float], tuple[float, float]]] | None"
    ) = None,
    blend_window_ticks: int = 1,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Return ``((nx_f, ny_f), (nx_r, ny_r))`` unit directions per axle.

    Spec §23.2-5.0.3.4 candidate (d) + Phase 5.0.5 rolling-window FIR.

    1. Compute the **instantaneous** target direction from the tier
       history (legacy behaviour):

       - ``prev_tier == TIER_MPC`` → use the previous clean MPC solve's
         planned (Fx, Fy) per axle as-is.
       - ``prev_tier == TIER_ELLIPSE`` → blend stale MPC and Stanley at
         ``direction_blend_stale_alpha`` (default 0.5).
       - ``prev_tier == TIER_REACTIVE`` or no MPC history → pure Stanley.

    2. Phase 5.0.5: if ``direction_history`` is provided and contains at
       least ``blend_window_ticks`` recent emitted unit-vector pairs,
       return the **time-averaged direction over the window** (re-
       normalised to unit length) **instead** of the instantaneous
       value. Before the buffer is full we fall back to the instantaneous
       value — the bootstrap behaviour the spec called out
       (longer-window damping kicks in only once enough samples are
       available).

       The averaging is a classic FIR smoother on the directional
       control signal, sized to damp the 50 Hz limit cycle that drove
       the Sprint A chicane abort across Phases 5.0 through 5.0.4 (see
       ``docs/architecture-slip-model-phase5_0_5-v32-tier1-bistability.md``).

    All four branches collapse to "pure Stanley" if MPC history is
    missing; the caller should never have to special-case the first
    Tier 1 tick.
    """
    p_f, p_r = (None, None) if prev_planned_forces is None else prev_planned_forces
    s_f, s_r = (None, None) if stanley_forces is None else stanley_forces

    def _blend(
        a: tuple[float, float] | None,
        b: tuple[float, float] | None,
        alpha: float,
    ) -> tuple[float, float]:
        if a is None and b is None:
            return (0.0, 0.0)
        if a is None:
            return b  # type: ignore[return-value]
        if b is None:
            return a
        return (alpha * a[0] + (1.0 - alpha) * b[0],
                alpha * a[1] + (1.0 - alpha) * b[1])

    if prev_tier == TIER_MPC and p_f is not None and p_r is not None:
        f_front = p_f
        f_rear = p_r
    elif prev_tier == TIER_ELLIPSE and p_f is not None and p_r is not None:
        f_front = _blend(p_f, s_f, direction_blend_stale_alpha)
        f_rear = _blend(p_r, s_r, direction_blend_stale_alpha)
    else:
        # Tier 2 or no MPC history — Stanley only.
        f_front = s_f if s_f is not None else (0.0, 0.0)
        f_rear = s_r if s_r is not None else (0.0, 0.0)

    n_front_inst = _unit2d(*f_front)
    n_rear_inst = _unit2d(*f_rear)

    # Phase 5.0.5 FIR smoother. Only engages once the buffer holds the
    # full window — otherwise return the instantaneous direction (the
    # bootstrap path mandated by the spec).
    window = int(blend_window_ticks)
    if (
        window > 1
        and direction_history is not None
        and len(direction_history) >= window
    ):
        # Average the most-recent ``window`` entries. ``deque`` slicing
        # is not supported, but a small list comprehension is cheap (≤12
        # entries by spec ceiling).
        recent = list(direction_history)[-window:]
        sx_f = sum(item[0][0] for item in recent) / window
        sy_f = sum(item[0][1] for item in recent) / window
        sx_r = sum(item[1][0] for item in recent) / window
        sy_r = sum(item[1][1] for item in recent) / window
        # Re-normalise to unit length; falls back to the instantaneous
        # value if the average collapses (e.g. two opposite directions
        # exactly cancel).
        n_front = _unit2d(sx_f, sy_f)
        n_rear = _unit2d(sx_r, sy_r)
        if (n_front[0] == 0.0 and n_front[1] == 1.0
                and abs(sx_f) < 1e-6 and abs(sy_f) < 1e-6):
            n_front = n_front_inst
        if (n_rear[0] == 0.0 and n_rear[1] == 1.0
                and abs(sx_r) < 1e-6 and abs(sy_r) < 1e-6):
            n_rear = n_rear_inst
        return n_front, n_rear

    return n_front_inst, n_rear_inst


def _unit2d(x: float, y: float) -> tuple[float, float]:
    n = math.hypot(float(x), float(y))
    if n < 1e-6:
        # Degenerate direction — pick pure-lateral as a benign default so
        # the saturation still places force on the ellipse boundary. The
        # rate clip will keep the emitted action tame.
        return (0.0, 1.0)
    return (float(x) / n, float(y) / n)


# --- Saturation + inverse map --------------------------------------------


def emit_ellipse_saturation(
    state,  # VehicleState — duck-typed
    *,
    n_front: tuple[float, float],
    n_rear: tuple[float, float],
    pc: PlantConstants,
    delta_max: float,
    delta_dot_max: float,
    throttle_dot_max: float,
    brake_dot_max: float,
    tick_period: float,
    last_commit: tuple[float, float, float],
    actuator_delta: float,
    actuator_throttle: float,
    actuator_brake: float,
    v_ref: float,
    saturation_safety: float = 0.95,
    fz_front_now: float | None = None,
    fz_rear_now: float | None = None,
) -> tuple[Controls, float, float, float]:
    """Compute the Tier 1 :class:`Controls` and the implied (δ, thr, brk).

    Returns ``(Controls, actuator_delta, actuator_throttle,
    actuator_brake)`` — the controller tracks the absolute actuator
    positions across ticks (so the rate clip can hold against the
    most-recent commit), so the caller writes those back into its
    ``self._actuator_*`` slots verbatim.

    Spec §23.2-5.0.3.4. Reads ``state.v_x, state.v_y, state.omega_yaw``
    to invert the front-axle slip-angle expression. Saturates at
    ``saturation_safety × ellipse`` (default 0.95) — spec
    §23.2-5.0.3.12 risk #3 mitigation against the static-Fz
    approximation over-stating load on the unloaded axle in a corner.

    Standing-start guard inherited from Phase 5.0 (`mpc_controller.py`
    line 560-561): if ``v_x < 1.0`` and ``v_ref > 2.0``, force
    throttle = 1.0, brake = 0.0. Skips the saturation pass entirely
    because the ellipse saturation against ~0 m/s gives degenerate
    forces.

    Phase 5.0.4 (spec §23.2-5.0.4.6 #3): when ``fz_front_now`` /
    ``fz_rear_now`` are provided, the per-axle saturation projects onto
    the **dynamic** ellipse ``D · Fz_dyn`` rather than the static one.
    This is the primary lever for whether 5.0.4 closes the chicane —
    if Tier-1 keeps over-trusting the unloaded axle, the saturation
    feedforward will keep sending too much steering.
    """
    last_delta, last_thr, last_brk = last_commit

    # Standing-start: bypass saturation, emit the soft-start commit.
    if float(state.v_x) < 1.0 and float(v_ref) > 2.0:
        ctrl = Controls(
            steer_rad=float(np.clip(actuator_delta, -delta_max, delta_max)),
            throttle=1.0,
            brake=0.0,
        )
        return ctrl, actuator_delta, 1.0, 0.0

    # 1. Saturated (F_x, F_y) per axle along the planned direction.
    (Fx_f_ff, Fy_f_ff), (Fx_r_ff, Fy_r_ff) = _saturate_per_axle(
        n_front, n_rear, pc, safety=saturation_safety,
        fz_front_now=fz_front_now, fz_rear_now=fz_rear_now,
    )

    # 2. Inverse-map lateral force → steering.
    delta_target = _invert_steering(
        state, Fy_f_ff, pc, delta_max=delta_max,
    )

    # 3. Inverse-map longitudinal force → throttle / brake.
    throttle_target, brake_target = _invert_pedals(
        Fx_f_ff, Fx_r_ff, pc,
    )

    # 4. Single-tick rate clip — exactly the same caps applied to the
    # MPC's commit in mpc_controller._resolve_mpc (lines 539-544).
    # Tier 1 inherits the rate budget from MPCBounds so the worst-case
    # steering rate is by construction ≤ delta_dot_max × tick_period.
    cap_delta = delta_dot_max * tick_period
    cap_thr = throttle_dot_max * tick_period
    cap_brk = brake_dot_max * tick_period
    delta_emit = float(np.clip(
        delta_target,
        last_delta - cap_delta, last_delta + cap_delta,
    ))
    delta_emit = float(np.clip(delta_emit, -delta_max, delta_max))
    thr_emit = float(np.clip(
        throttle_target,
        last_thr - cap_thr, last_thr + cap_thr,
    ))
    thr_emit = float(np.clip(thr_emit, 0.0, 1.0))
    brk_emit = float(np.clip(
        brake_target,
        last_brk - cap_brk, last_brk + cap_brk,
    ))
    brk_emit = float(np.clip(brk_emit, 0.0, 1.0))

    ctrl = Controls(
        steer_rad=delta_emit,
        throttle=thr_emit,
        brake=brk_emit,
    )
    return ctrl, delta_emit, thr_emit, brk_emit


def _saturate_per_axle(
    n_front: tuple[float, float],
    n_rear: tuple[float, float],
    pc: PlantConstants,
    *,
    safety: float = 0.95,
    fz_front_now: float | None = None,
    fz_rear_now: float | None = None,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Project the unit direction onto the per-axle friction ellipse.

    For direction ``n = (n_x, n_y)`` (unit), the per-axle peak force
    along ``n`` is::

        denom = sqrt((n_x / D_long_a)^2 + (n_y / D_lat_a)^2) / Fz_a
        (F_x, F_y) = safety * (n_x / denom, n_y / denom)

    placing the force on ``safety × ellipse`` along ``n``. ``safety``
    defaults to 0.95 (spec §23.2-5.0.3.12 risk #3 — guard against the
    static-Fz approximation under-conservatism on the unloaded axle).

    Phase 5.0.4: ``fz_front_now`` / ``fz_rear_now`` (N) override
    ``pc.Fz_front`` / ``pc.Fz_rear`` when provided. Callers pass the
    dynamic Fz at the current chassis state so the saturation honours
    the load that is actually on the axle right now.
    """
    def _project(n: tuple[float, float], D_long: float, D_lat: float, Fz: float) -> tuple[float, float]:
        nx, ny = n
        # Guard against zero traction (uninitialised plant constants).
        D_long_eff = max(float(D_long), 1e-3)
        D_lat_eff = max(float(D_lat), 1e-3)
        Fz_eff = max(float(Fz), 1.0)
        denom = math.sqrt(
            (nx / D_long_eff) ** 2 + (ny / D_lat_eff) ** 2
        ) / Fz_eff
        if denom < 1e-6:
            return (0.0, 0.0)
        scale = safety / denom
        return (float(nx * scale), float(ny * scale))

    Fz_f = float(fz_front_now) if fz_front_now is not None else pc.Fz_front
    Fz_r = float(fz_rear_now) if fz_rear_now is not None else pc.Fz_rear
    f_front = _project(n_front, pc.D_long, pc.D_lat_front, Fz_f)
    f_rear = _project(n_rear, pc.D_long, pc.D_lat_rear, Fz_r)
    return f_front, f_rear


def _invert_steering(
    state,
    F_y_front_target: float,
    pc: PlantConstants,
    *,
    delta_max: float,
) -> float:
    """Solve front-axle slip-angle relation for ``δ``.

    Phase 5.0.1 affine Pacejka (front): ``F_y = F_y_bias + C_α · α``.
    Front slip-angle: ``α_front = δ − atan2(v_y + a_f · ω, v_x)``.

    With ``F_y_target`` and ``F_y_bias`` known::

        α_target = (F_y_target − F_y_bias_front) / C_α_front
        δ        = α_target + atan2(v_y + a_f · ω, v_x)

    Clipped to ``±delta_max``. Defensive against zero C_α (degenerate
    plant constants on a stalled controller).
    """
    C_alpha = float(pc.C_alpha_front)
    if abs(C_alpha) < 1.0:
        # No useful cornering stiffness — emit zero steer and let the
        # rate-clip outer layer hold the current steering position.
        return 0.0
    alpha_target = (
        float(F_y_front_target) - float(pc.F_y_bias_front)
    ) / C_alpha
    vx_safe = max(float(state.v_x), 1.0)
    side_slip = math.atan2(
        float(state.v_y) + pc.a_f * float(state.omega_yaw),
        vx_safe,
    )
    delta = alpha_target + side_slip
    return float(np.clip(delta, -delta_max, delta_max))


def _invert_pedals(
    F_x_front_target: float,
    F_x_rear_target: float,
    pc: PlantConstants,
) -> tuple[float, float]:
    """Solve longitudinal axle forces for ``(throttle, brake)``.

    Phase 5.0.1 affine longitudinal::

        F_x_front =  −k_brake_front · brake
        F_x_rear  =   k_throttle    · throttle  −  k_brake_rear · brake

    Net forward force is dominated by the rear (RWD) axle; the
    front-axle target only ever requires brake (sign convention:
    F_x_front_target ≤ 0 on the BMW 1M).

    Decision rule: if the rear axle wants to accelerate, set
    ``brake = 0`` and solve for ``throttle``; if it wants to brake,
    set ``throttle = 0`` and solve for ``brake`` from whichever axle
    target is more demanding. Pure-clip semantics for the [0, 1]
    pedal range — the rate clip in the outer layer absorbs any
    chatter.
    """
    k_thr = max(float(pc.k_throttle), 1.0)
    k_brk_f = max(float(pc.k_brake_front), 1.0)
    k_brk_r = max(float(pc.k_brake_rear), 1.0)

    if float(F_x_rear_target) >= 0.0:
        # Wants to accelerate (RWD). Pure throttle, no brake.
        throttle = float(F_x_rear_target) / k_thr
        brake = 0.0
    else:
        # Wants to brake. brake_value is the magnitude that satisfies
        # whichever axle is asking for more deceleration.
        throttle = 0.0
        # Rear brake force = k_brk_r * brake (since F_x_rear_target <0
        # and throttle=0 ⇒ F_x_rear = -k_brk_r * brake).
        brake_rear = -float(F_x_rear_target) / k_brk_r
        # Front brake force = k_brk_f * brake. F_x_front_target ≤ 0 in
        # normal operation; if it's positive (planned-direction said
        # "drive the front axle forward"), the affine model can't
        # produce that, so we ignore and bind on the rear.
        if float(F_x_front_target) < 0.0:
            brake_front = -float(F_x_front_target) / k_brk_f
        else:
            brake_front = 0.0
        brake = max(brake_rear, brake_front)
    return float(np.clip(throttle, 0.0, 1.0)), float(np.clip(brake, 0.0, 1.0))


# --- Stanley-style direction helper -------------------------------------


def stanley_forces_from_controls(
    state,
    sub_controls: Controls,
    pc: PlantConstants,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Compute per-axle (F_x, F_y) the reactive sub-controller would produce.

    Substitute the sub-controller's (steer, throttle, brake) command
    into the affine Pacejka relations evaluated at the current chassis
    state. Spec §23.2-5.0.3.4 candidate (b) — used as the Stanley-side
    of the direction blend when MPC history is stale or absent.

    Uses the same machinery as :func:`compute_axle_force_from_state`
    but with the sub-controller's commanded (δ, thr, brk) substituted
    for the chassis-state actuator positions. The slip-angle
    denominator (v_x) is taken from ``state`` so the side-slip term
    matches the moment the action would land.
    """
    x_vec = np.zeros(NX)
    x_vec[IDX_VX] = float(state.v_x)
    x_vec[IDX_VY] = float(state.v_y)
    x_vec[IDX_OMEGA] = float(state.omega_yaw)
    x_vec[IDX_DELTA] = float(sub_controls.steer_rad)
    x_vec[IDX_THR] = float(sub_controls.throttle)
    x_vec[IDX_BRK] = float(sub_controls.brake)
    return compute_axle_force_from_state(x_vec, pc)


__all__ = [
    "TIER_MPC",
    "TIER_ELLIPSE",
    "TIER_REACTIVE",
    "HARD_STATUS_CODES",
    "CLEAN_STATUS_CODES",
    "classify_qp_status",
    "compute_planned_direction",
    "emit_ellipse_saturation",
    "stanley_forces_from_controls",
]
