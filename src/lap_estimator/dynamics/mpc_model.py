"""LTV bicycle plant + linearisation for the Phase-5.0 MPC (spec §23.2.4).

Phase 5.0.4 (spec §23.2-5.0.4): the per-axle Fz that drives ``D' = D · Fz``
in the Pacejka linearisation and the ellipse hard constraint is no longer
a controller-construction-time **scalar**. The MPC now consumes a
**per-stage Fz** profile that the SQP outer loop refreshes between
iterations from the predicted ``(a_x_k, a_y_k)`` trajectory (Picard
update with damping ``β = 0.4``). The plant constants schema retains
scalar ``Fz_front``, ``Fz_rear`` for back-compat (used by tests + the DP
planner); new fields ``Fz_front_stage``, ``Fz_rear_stage`` carry the
per-stage arrays the QP and post-solve ellipse check consume.

The MPC's internal plant is a **linear time-varying bicycle** with the
following state vector (8 vars; §23.2.4):

    x = [e_lat, e_psi, v_x, v_y, omega_yaw, delta, throttle, brake]

and rate-controls (3 vars):

    u = [delta_dot, throttle_dot, brake_dot]

The continuous-time dynamics are

    e_lat_dot      = v_x*sin(e_psi) + v_y*cos(e_psi)
    e_psi_dot      = omega_yaw - kappa_ref(s)*v_x
    v_x_dot        = (Fx_front*cos(delta) + Fx_rear + drag) / m + v_y*omega_yaw
    v_y_dot        = (Fy_front*cos(delta) + Fy_rear) / m - v_x*omega_yaw
    omega_yaw_dot  = (a_f*Fy_front*cos(delta) - a_r*Fy_rear) / I_zz
    delta_dot      = u0
    throttle_dot   = u1
    brake_dot      = u2

with the **operating-point slope-only Pacejka** axle forces (Phase 5.0.1,
spec §23.2-5.0.1.3 alternative row 1):

    Fy_axle = C_alpha_op_axle * alpha_axle      (F_y_bias_axle = 0)
    Fx_rear  =  k_thr * throttle - k_brk_r * brake   (drive axle = RWD assumed)
    Fx_front = -k_brk_f * brake                       (front brakes only on brake)

The slope-only form takes ``C_alpha_op = dFy/dalpha |_{alpha_op}`` (the
Magic Formula tangent slope at the operating point, where
``alpha_op = 0.5 * alpha_peak * skill_factor``). Pre-Phase-5.0.1 used the
small-signal slope ``C_alpha = D * Fz * B * C`` at alpha=0, which on the
Tomas/BMW 1M calib gives ~2.76× the slope at alpha_op and overestimates
lateral force at peak slip by ~2× — the dominant Phase 5.0 chicane
failure mode (architecture doc:
``architecture-slip-model-phase5_0-v32-mpc.md``).

Build-time decision (deviates from spec §23.2-5.0.1.3 chosen row):
the spec's chosen affine form ``Fy = F_y_bias - C'_op * alpha`` (with
non-zero bias) was found at build time to produce a sign mismatch
through the Sprint A chicane's right-then-left transition, where
alpha_front sweeps from large negative to large positive in <0.5 s.
The Magic Formula is odd; an affine line about +alpha_op predicts
~zero Fy at -alpha_op (true value: -Fy_peak). The MPC then misjudges
the lateral force balance through the chicane. The slope-only
alternative (row 1 of the spec's evaluation table) is symmetric, has
no bias, and underestimates Fy at +alpha_op by ~30 %; it closes the
headline 2× overshoot at alpha_peak without the sign artefact.
``F_y_bias_axle`` is preserved in :class:`PlantConstants` for
diagnostics / future per-stage-signed upgrades (5.0.2 backlog).

Front / rear axle stiffnesses (``C_alpha_op_front``, ``C_alpha_op_rear``)
are computed once at controller construction from the static axle Fz
and the per-axle Pacejka ``(B, C, D, E)`` block; weight-transfer drift
is absorbed by the SQP outer loop's central-difference Jacobian.

The discretisation is **explicit Euler with ds = 2 m** stages:

    x_{k+1} = x_k + ts(k) * f(x_k, u_k; kappa_ref(s_k), v_ref(s_k))

where ``ts(k) = ds / max(v_x_lin(k), V_FLOOR)`` keeps stages distance-
uniform on a 2 m grid (matches the controller's existing ds gridding,
spec §23.2.3). Linearising about a known reference trajectory
``(x_lin(k), u_lin(k))`` for each stage gives the standard tracking-MPC
state-space pair

    x_{k+1} = A_k * x_k + B_k * u_k + c_k

This module is pure-NumPy and has no OSQP / scipy.sparse dependency.
The QP-build layer (mpc_qp.py) consumes the (A_k, B_k, c_k) triples plus
the per-stage cost weights.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .pacejka import PacejkaCoeffs

# State vector layout — keep these constants in sync with mpc_qp.py.
NX = 8
NU = 3
IDX_E_LAT = 0
IDX_E_PSI = 1
IDX_VX = 2
IDX_VY = 3
IDX_OMEGA = 4
IDX_DELTA = 5
IDX_THR = 6
IDX_BRK = 7

V_FLOOR = 5.0  # m/s — denominator floor for distance-time conversion.
G = 9.81


@dataclass(frozen=True)
class PlantConstants:
    """Time-invariant chassis + tyre constants used by the linearised plant.

    These do not change during a lap; computed once at controller
    construction (static Fz, fitted Pacejka peaks, drag coefficient).

    Phase 5.0.1 (spec §23.2-5.0.1.3, slope-only alternative row 1): the
    lateral stiffnesses are the **operating-point slopes**
    ``C_alpha_op = +dFy/d alpha |_{alpha_op}`` (positive in this
    codebase's convention; positive alpha => positive Fy) rather than
    the small-signal ``D*Fz*B*C`` at alpha=0. The pre-Phase-5.0.1 form
    over-predicted Fy at peak slip by ~2x — the dominant chicane
    failure mode. ``F_y_bias_axle`` is kept in the schema for diagnostics
    / per-stage-signed upgrades (5.0.2 backlog) but is set to 0.0 in
    this phase to preserve the Magic Formula's odd symmetry through
    left/right corner transitions (see module docstring for rationale).

    Phase 5.0.4 (spec §23.2-5.0.4): ``Fz_front`` and ``Fz_rear`` are now
    **operating-point values** rather than always-static. When the SQP
    outer loop refreshes the per-stage ``(a_x_k, a_y_k)`` predicted
    trajectory, ``Fz_front_stage`` / ``Fz_rear_stage`` carry the
    horizon-resolved values consumed by the ellipse hard constraint and
    the post-solve violation check. The scalar ``Fz_front`` / ``Fz_rear``
    still equal the controller-construction static values; ``slope_op_*_per_fz``
    are the load-independent Magic-Formula slopes (per unit Fz) so the
    QP build can compute per-stage ``C_alpha_k = slope_op_per_fz · Fz_k``.
    ``dynamic_fz_enabled`` is a diagnostic flag (the static-Fz path is
    triggered by ``Fz_*_stage = None`` in the consumer modules).
    """

    mass: float
    I_zz: float
    a_f: float  # CG -> front axle (m, positive forward)
    a_r: float  # CG -> rear axle (m, positive rearward, stored positive)
    # Operating-point Pacejka stiffnesses (N/rad), front+rear axle pairs
    # (axle-level total force; bicycle uses axle-level forces directly).
    # Sign convention: Fy_axle = F_y_bias_axle + C_alpha_op_axle * alpha_axle
    # — positive alpha => positive Fy at alpha=alpha_op (matches v3.1
    # vehicle.py convention; pre-5.0.1 used C_alpha * alpha directly,
    # with C_alpha the slope at alpha=0).
    C_alpha_front: float
    C_alpha_rear: float
    # Phase 5.0.1: affine offset folded into the bicycle dynamics. Computed
    # so the tangent line through (alpha_op, F_y(alpha_op)) reproduces the
    # true Pacejka magnitude at the operating point:
    #     Fy(alpha_op) = F_y_bias + C_alpha_op * alpha_op
    #     -> F_y_bias = Fy_op - C_alpha_op * alpha_op
    # For ``alpha_op > 0`` and a concave Magic Formula curve below the peak,
    # F_y_bias < 0 (the tangent crosses zero at some alpha < alpha_op);
    # this is the "phantom Fy" at alpha=0 the spec discusses.
    F_y_bias_front: float
    F_y_bias_rear: float
    # Operating points used for the linearisation (rad). Sign is conventional
    # — magnitude = 0.5 * alpha_peak_axle * skill_factor; the slope_op /
    # Fy_op are evaluated at the positive side of the Magic Formula
    # (Magic Formula is symmetric about 0, so the linearisation is
    # equivalent up to a sign).
    alpha_op_front: float
    alpha_op_rear: float
    # Phase 5.0.4: load-independent Magic-Formula slope per unit Fz, kept
    # so the SQP loop can compute per-stage C_alpha_k = slope_op_per_fz · Fz_k
    # without re-evaluating the analytic derivative every stage.
    slope_op_front_per_fz: float
    slope_op_rear_per_fz: float
    # Longitudinal force coefficients: throttle * k_throttle = Fx on the drive
    # axle (RWD assumed for Phase 5.0; FWD/AWD readers would split). brake *
    # k_brake_<f|r> = magnitude of Fx on that axle (opposing motion).
    k_throttle: float  # N per unit throttle, at typical v_x. Computed from
                       # car.max_traction_force at v_ref average.
    k_brake_front: float  # N per unit brake
    k_brake_rear: float
    # Aero drag: F_drag = drag_coeff * v_x^2 (sign opposes motion).
    drag_coeff: float
    # Linearised pacejka peaks (used to express slip-budget constraint).
    D_lat_front: float  # mu peak; F_y_max_front_axle ~ D_lat_front * Fz_front
    D_lat_rear: float
    D_long: float       # mu peak longitudinal
    Fz_front: float     # static (N) — operating-point value for the linearisation
    Fz_rear: float
    # Phase 5.0.4: dynamic-Fz geometry (consumed by the SQP per-stage
    # refresh helper :func:`compute_dynamic_fz_per_stage`). Carried on
    # PlantConstants so the helper doesn't need a separate (car, dyn)
    # plumbing path.
    cg_front: float = 0.5       # mass fraction on front axle (0..1)
    h_cg: float = 0.45          # CG height (m)
    wheelbase: float = 2.66     # m
    track_f: float = 1.55       # front track (m)
    track_r: float = 1.55       # rear track (m)
    F_down_op: float = 0.0      # downforce at the v_ref linearisation point (N)
    dynamic_fz_enabled: bool = False
    # Phase 5.0.4 lateral-split correction coefficient (spec §23.2-5.0.4.4).
    # Empirical calibration against truth ``vehicle._weight_transfer`` +
    # per-wheel Pacejka with the fitted (B, C, D, E) on Tomas/BMW 1M:
    # **0.000** is best-fit. The fitted Pacejka has load-linear D (Fy =
    # D · Fz · sin(...)), so the per-axle Pacejka call at the axle-total
    # Fz exactly equals the sum of two per-wheel Pacejka calls at the
    # lateral-split half-Fz. No structural loss exists. Build-time
    # ``.tmp/calibrate_k_lat_loss.py`` confirms this within 0.0 % across
    # the chicane regime (a_x ∈ [-10, 4], a_y ∈ [-12, 12], α ∈ [2, 10] °).
    # Field retained for diagnostics / re-enabling if a future Pacejka
    # fit adds a load-sensitivity term.
    k_lat_loss: float = 0.0
    # AC non-linear-load knobs (``LS_EXPY`` / ``LS_EXPX`` with per-axle
    # reference load ``FZ0``). Default ``None`` keeps the legacy linear-D
    # ellipse semi-axes (``D · Fz``); when set, the QP linearisation
    # multiplies each per-stage ``D · Fz`` term by
    # ``(Fz_k / FZ0)**(LS_EXP - 1)`` (front + rear, lateral + longitudinal)
    # so the ellipse cap tracks the truth-model peak. See
    # :mod:`mpc_qp_ellipse` for the math.
    Fz0_front: float | None = None
    Fz0_rear: float | None = None
    ls_exp_lat_front: float | None = None
    ls_exp_lat_rear: float | None = None
    ls_exp_long_front: float | None = None
    ls_exp_long_rear: float | None = None


def _axle_fz_at_op(
    Fz_front_static: float,
    Fz_rear_static: float,
    *,
    mass: float,
    h_cg: float,
    wb: float,
    track_f: float,
    track_r: float,
    cg_front: float,
    a_x: float,
    a_y: float,
    k_lat_loss: float = 0.0,
) -> tuple[float, float]:
    """Phase 5.0.4: per-axle Fz at one ``(a_x, a_y)`` operating point.

    Mirrors the truth model's :func:`vehicle._weight_transfer`
    aggregation up to per-axle totals (lateral transfer is intra-axle;
    it leaves the axle sum unchanged, but the non-linear-Fy effect of
    the inner/outer split is captured via the ``k_lat_loss`` term —
    calibrated to 0.0 against the fitted Pacejka; see
    :class:`PlantConstants` for the build-time finding).

    Returns ``(Fz_front_eff, Fz_rear_eff)`` in N. Floored at 100 N to
    match :func:`vehicle._weight_transfer`. ``a_x > 0`` = forward accel
    (so the front loses load); ``a_y`` magnitude only matters for the
    intra-axle split when ``k_lat_loss > 0``.
    """
    dFz_long = mass * float(a_x) * h_cg / max(wb, 1e-3)
    Fz_front_axle = Fz_front_static - dFz_long
    Fz_rear_axle = Fz_rear_static + dFz_long
    # Lateral split per axle (signed magnitude; halves of axle total).
    dFz_lat_f = (
        mass * float(a_y) * h_cg / max(track_f, 1e-3) * (1.0 - cg_front)
    )
    dFz_lat_r = mass * float(a_y) * h_cg / max(track_r, 1e-3) * cg_front
    if k_lat_loss > 0.0:
        # Effective Fz reduction from the inner/outer lateral split
        # (spec §23.2-5.0.4.4). With our fitted Pacejka the term is
        # numerically zero; kept for diagnostics.
        ratio_f = abs(dFz_lat_f) / max(Fz_front_axle, 1.0)
        ratio_r = abs(dFz_lat_r) / max(Fz_rear_axle, 1.0)
        Fz_front_axle *= 1.0 - k_lat_loss * (ratio_f * ratio_f)
        Fz_rear_axle *= 1.0 - k_lat_loss * (ratio_r * ratio_r)
    # Floor at 100 N — matches truth model (vehicle._weight_transfer).
    return (
        float(max(Fz_front_axle, 100.0)),
        float(max(Fz_rear_axle, 100.0)),
    )


def compute_dynamic_fz_per_stage(
    pc: PlantConstants,
    a_x_seq: np.ndarray,
    a_y_seq: np.ndarray,
    *,
    Fz_front_prev: np.ndarray | None = None,
    Fz_rear_prev: np.ndarray | None = None,
    damping: float = 0.4,
) -> tuple[np.ndarray, np.ndarray]:
    """Phase 5.0.4: per-stage dynamic Fz with SQP Picard damping.

    Inputs are length-``N_horizon`` arrays of predicted ``(a_x, a_y)`` in
    m/s² at each stage's *entry* time. Returns per-axle Fz arrays of the
    same length. Damped Picard update against the previous SQP iter's
    Fz profile (``β = 0.4`` from spec §23.2-5.0.4.5, Risk 2):

        Fz_k^{i+1} = (1 − β) · Fz_k^i + β · Fz_k_from_a

    When no previous profile is given (first SQP iter on a fresh tick),
    the function returns the un-damped per-stage values.

    The static / dynamic switch is the *caller's* job — when the static
    path is desired, pass ``a_x_seq = a_y_seq = 0`` (giving the static
    operating-point Fz) and skip this helper.
    """
    F_down = float(pc.F_down_op)
    cgf = float(pc.cg_front)
    m = float(pc.mass)
    h = float(pc.h_cg)
    wb = max(float(pc.wheelbase), 1e-3)
    t_f = max(float(pc.track_f), 1e-3)
    t_r = max(float(pc.track_r), 1e-3)
    Fz_f_static = m * G * (1.0 - cgf) + F_down * (1.0 - cgf)
    Fz_r_static = m * G * cgf + F_down * cgf
    ax = np.asarray(a_x_seq, dtype=float)
    ay = np.asarray(a_y_seq, dtype=float)
    n = len(ax)
    dFz_long = m * ax * h / wb
    out_f = Fz_f_static - dFz_long
    out_r = Fz_r_static + dFz_long
    if pc.k_lat_loss > 0.0:
        dFz_lat_f = m * ay * h / t_f * (1.0 - cgf)
        dFz_lat_r = m * ay * h / t_r * cgf
        ratio_f = np.abs(dFz_lat_f) / np.maximum(out_f, 1.0)
        ratio_r = np.abs(dFz_lat_r) / np.maximum(out_r, 1.0)
        out_f = out_f * (1.0 - pc.k_lat_loss * (ratio_f * ratio_f))
        out_r = out_r * (1.0 - pc.k_lat_loss * (ratio_r * ratio_r))
    out_f = np.maximum(out_f, 100.0)
    out_r = np.maximum(out_r, 100.0)
    if (
        Fz_front_prev is not None
        and Fz_rear_prev is not None
        and len(Fz_front_prev) == n
        and len(Fz_rear_prev) == n
        and 0.0 < damping < 1.0
    ):
        out_f = (1.0 - damping) * np.asarray(Fz_front_prev, dtype=float) + damping * out_f
        out_r = (1.0 - damping) * np.asarray(Fz_rear_prev, dtype=float) + damping * out_r
    return out_f, out_r


def axle_accelerations_from_trajectory(
    x_seq: np.ndarray,
    *,
    v_ref_seq: np.ndarray,
    ds: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Phase 5.0.4: extract ``(a_x_k, a_y_k)`` per stage from a rolled trajectory.

    The MPC plant state ``[e_lat, e_psi, v_x, v_y, omega, ...]`` doesn't
    carry accelerations explicitly; we recover them from finite
    differences in v_x (longitudinal) and the steady-turn approximation
    ``v_x · omega`` (lateral, same convention as
    :func:`vehicle.compute_derivatives`'s ``a_y_est`` for the truth
    model's weight-transfer call).

    ``x_seq`` is the rolled state trajectory of length N+1 (output of
    :func:`integrate_reference`). Returns length-N arrays aligned with
    each MPC stage's *entry* state ``x_seq[k]``.
    """
    x_arr = np.asarray(x_seq, dtype=float)
    n = x_arr.shape[0] - 1
    v_ref = np.asarray(v_ref_seq, dtype=float)
    if v_ref.shape[0] < n:
        # Pad to ensure alignment.
        v_ref = np.concatenate([v_ref, np.full(n - v_ref.shape[0], v_ref[-1])])
    ts = float(ds) / np.maximum(v_ref[:n], V_FLOOR)
    v_x_k = x_arr[:n, IDX_VX]
    v_x_next = x_arr[1:n + 1, IDX_VX]
    a_x = (v_x_next - v_x_k) / np.maximum(ts, 1e-3)
    omega_k = x_arr[:n, IDX_OMEGA]
    a_y = v_x_k * omega_k
    return a_x, a_y


def _magic_formula_fy_per_unit_fz(alpha: float, coeffs) -> float:
    """Return ``Fy / Fz`` from the Magic Formula at slip angle ``alpha`` (rad).

    Closed-form Pacejka with the standard four-parameter ``(B, C, D, E)``
    form: ``Fy/Fz = D * sin(C * atan(B*alpha - E*(B*alpha - atan(B*alpha))))``.
    """
    B = float(coeffs.B)
    C = float(coeffs.C)
    D = float(coeffs.D)
    E = float(coeffs.E)
    Bx = B * float(alpha)
    inner = Bx - E * (Bx - math.atan(Bx))
    return D * math.sin(C * math.atan(inner))


def _magic_formula_slope_per_unit_fz(alpha: float, coeffs) -> float:
    """Return ``d(Fy/Fz)/d alpha`` at slip angle ``alpha`` (rad).

    Analytic derivative of :func:`_magic_formula_fy_per_unit_fz`:

        let Bx     = B * alpha
            phi    = Bx - E * (Bx - atan(Bx))
            theta  = C * atan(phi)
        Fy/Fz = D * sin(theta)
        dFy/Fz / dalpha = D * cos(theta) * C / (1 + phi^2) * dphi/dalpha
        dphi/dalpha     = B * (1 - E) + E * B / (1 + (B*alpha)^2)
    """
    B = float(coeffs.B)
    C = float(coeffs.C)
    D = float(coeffs.D)
    E = float(coeffs.E)
    Bx = B * float(alpha)
    inner = Bx - E * (Bx - math.atan(Bx))
    phi_prime = B * (1.0 - E) + E * B / (1.0 + Bx * Bx)
    theta = C * math.atan(inner)
    return D * math.cos(theta) * C / (1.0 + inner * inner) * phi_prime


def build_plant_constants(
    car,
    dyn,
    calib,
    v_ref_avg: float = 30.0,
    *,
    alpha_op_front_rad: float = 0.0,
    alpha_op_rear_rad: float = 0.0,
    a_x_op: float = 0.0,
    a_y_op: float = 0.0,
    dynamic_fz_enabled: bool = False,
    cg_height_m: float | None = None,
    track_width_f_m: float | None = None,
    track_width_r_m: float | None = None,
) -> PlantConstants:
    """One-shot factory: derive :class:`PlantConstants` from the car + calib.

    Parameters
    ----------
    car : Car
        Provides total_mass, aero, wheelbase, brake_torque, traction-force LUT.
    dyn : CarDynamics
        CG split + I_zz (already wheelbase * (1 - cg_front) gives ``a_f``).
    calib : PacejkaCalibration
        Front + rear axle (lateral + longitudinal) Magic-Formula coeffs.
    v_ref_avg : float
        Representative speed at which to evaluate engine traction. Default
        30 m/s ~ Sprint A average. Re-linearised inside the SQP loop if
        the binding stage diverges far from this.
    alpha_op_front_rad, alpha_op_rear_rad : float
        Operating points (rad) about which to linearise the Magic Formula
        per axle. Phase 5.0.1 callers pass ``0.5 * alpha_peak_axle *
        skill_factor`` (spec §23.2-5.0.1.3); a default of 0 reproduces the
        pre-Phase-5.0.1 small-signal behaviour (used only as a back-compat
        path when constructing a plant without skill information).
    a_x_op, a_y_op : float
        Phase 5.0.4 dynamic-Fz operating-point accelerations (m/s²) for
        the controller-construction-time scalar ``Fz_front``, ``Fz_rear``.
        Defaults to 0 (static Fz). The SQP loop overrides these per stage
        via :func:`compute_dynamic_fz_per_stage`.
    dynamic_fz_enabled : bool
        Phase 5.0.4 diagnostic flag — does the controller plan with
        per-stage dynamic Fz? When False, the SQP keeps the
        controller-construction static Fz.
    cg_height_m, track_width_f_m, track_width_r_m : float, optional
        Driver-JSON / CLI overrides for the geometry consumed by dynamic
        Fz. ``None`` falls through to the ``CarDynamics`` value.
    """
    mass = float(car.total_mass)
    wb = float(dyn.wheelbase)
    cgf = float(dyn.cg_front)
    h_cg = float(cg_height_m if cg_height_m is not None else dyn.h_cg)
    t_f = float(track_width_f_m if track_width_f_m is not None else dyn.track_f)
    t_r = float(track_width_r_m if track_width_r_m is not None else dyn.track_r)
    # AC's CG_LOCATION fraction of mass on FRONT axle. a_f = CG -> front
    # axle = wb * (1 - cgf); a_r = CG -> rear = wb * cgf.
    a_f = wb * (1.0 - cgf)
    a_r = wb * cgf
    # Per-axle Fz at the (a_x_op, a_y_op) operating point (Phase 5.0.4).
    # Same quasi-static formulation the truth model uses in
    # ``vehicle._weight_transfer``: longitudinal transfer shifts axle
    # totals; lateral transfer is intra-axle so axle totals only see
    # the |dFz_lat / Fz_axle|² loss term (k_lat_loss = 0 by build-time
    # calibration; see PlantConstants docstring).
    F_down = float(car.downforce(max(v_ref_avg, 0.0)))
    Fz_front_static = mass * G * (1.0 - cgf) + F_down * (1.0 - cgf)
    Fz_rear_static = mass * G * cgf + F_down * cgf
    Fz_front, Fz_rear = _axle_fz_at_op(
        Fz_front_static, Fz_rear_static,
        mass=mass, h_cg=h_cg, wb=wb,
        track_f=t_f, track_r=t_r, cg_front=cgf,
        a_x=float(a_x_op), a_y=float(a_y_op),
    )
    # Phase 5.0.1 (spec §23.2-5.0.1.3): operating-point linearisation of
    # the Magic Formula. Spec's recommended form was the AFFINE
    #     Fy = F_y_bias - C'_op * alpha
    # which is exact at alpha=alpha_op but predicts a non-zero "phantom"
    # Fy at alpha=0 of order 0.5*Fy_peak. The spec accepted this trade-off
    # under the assumption "the chassis is always rotating against the
    # line; pure alpha=0 is a measure-zero state".
    #
    # On Sprint A this assumption is broken: the chicane at s~645 m is a
    # right-then-left transition where alpha_front sweeps from large
    # negative to large positive in <0.5 s. The affine linearisation
    # about a fixed positive (or negative) operating point gets the WRONG
    # SIGN for one half of the transition (the Magic Formula is odd, so
    # an affine line about +alpha_op predicts ~zero Fy at -alpha_op when
    # reality says -Fy_peak). The MPC's planned trajectory through the
    # chicane therefore mis-predicts the lateral force balance, and the
    # ellipse/state propagation in the LTV plant compound the error.
    #
    # Build-time decision (deviates from spec §23.2-5.0.1.3 chosen row):
    # ship the **slope-only operating-point** form
    #     Fy = +C_alpha_op * alpha
    # which is symmetric, has no bias, and underestimates Fy at alpha_op
    # by ~30 % (consistent with the spec's analysis of this alternative).
    # This still closes the headline Phase 5.0 failure mode (the C_alpha
    # at alpha=0 was 2.76x the true slope at alpha_op — see arch doc),
    # so the MPC now asks for proportionally more steering than the
    # small-signal Phase 5.0 form. The chicane left/right transition no
    # longer suffers a sign mismatch.
    cof = abs(float(alpha_op_front_rad))
    cor = abs(float(alpha_op_rear_rad))
    slope_op_front_per_fz = float(_magic_formula_slope_per_unit_fz(
        cof, calib.front.lateral,
    ))
    slope_op_rear_per_fz = float(_magic_formula_slope_per_unit_fz(
        cor, calib.rear.lateral,
    ))
    C_alpha_front = float(slope_op_front_per_fz * Fz_front)
    C_alpha_rear = float(slope_op_rear_per_fz * Fz_rear)
    # Slope-only form: F_y_bias = 0. Affine offset is preserved in the
    # PlantConstants schema for diagnostics / 5.0.2 per-stage upgrades.
    F_y_bias_front = 0.0
    F_y_bias_rear = 0.0
    # Longitudinal: k_throttle approximates dFx/dthrottle at v_ref. We use
    # the engine's full traction force / 1.0 as the slope (throttle in
    # [0, 1] maps linearly to F_engine up to traction-limit).
    k_throttle = float(car.max_traction_force(max(v_ref_avg, 1.0)))
    # Brake force per axle from brake_torque * front_share / R_tyre (sum of
    # both wheels in the axle). We treat the front+rear brake forces as
    # proportional to brake pedal with the car's brake_front_share.
    brake_torque_total = float(getattr(car, "brake_torque", 2500.0)) * 2.0
    # Two wheels per axle.
    front_share = float(getattr(car, "brake_front_share", 0.6))
    r_f = float(car.tyre_radius_f)
    r_r = float(car.tyre_radius_r)
    k_brake_front = brake_torque_total * front_share / max(r_f, 1e-3)
    k_brake_rear = brake_torque_total * (1.0 - front_share) / max(r_r, 1e-3)
    # Aero drag coefficient: F_drag = 0.5 * rho * cd * A * v^2.
    drag_coeff = 0.5 * 1.225 * float(car.aero_cd) * float(car.frontal_area)
    return PlantConstants(
        mass=mass,
        I_zz=float(dyn.I_zz),
        a_f=a_f,
        a_r=a_r,
        C_alpha_front=C_alpha_front,
        C_alpha_rear=C_alpha_rear,
        F_y_bias_front=F_y_bias_front,
        F_y_bias_rear=F_y_bias_rear,
        alpha_op_front=cof,
        alpha_op_rear=cor,
        slope_op_front_per_fz=slope_op_front_per_fz,
        slope_op_rear_per_fz=slope_op_rear_per_fz,
        k_throttle=k_throttle,
        k_brake_front=k_brake_front,
        k_brake_rear=k_brake_rear,
        drag_coeff=drag_coeff,
        D_lat_front=float(calib.front.lateral.D),
        D_lat_rear=float(calib.rear.lateral.D),
        D_long=float(calib.front.longitudinal.D),
        Fz_front=Fz_front,
        Fz_rear=Fz_rear,
        cg_front=cgf,
        h_cg=h_cg,
        wheelbase=wb,
        track_f=t_f,
        track_r=t_r,
        F_down_op=float(F_down),
        dynamic_fz_enabled=bool(dynamic_fz_enabled),
        k_lat_loss=0.0,
        Fz0_front=getattr(calib.front, "Fz0", None),
        Fz0_rear=getattr(calib.rear, "Fz0", None),
        ls_exp_lat_front=getattr(calib.front, "ls_exp_lat", None),
        ls_exp_lat_rear=getattr(calib.rear, "ls_exp_lat", None),
        ls_exp_long_front=getattr(calib.front, "ls_exp_long", None),
        ls_exp_long_rear=getattr(calib.rear, "ls_exp_long", None),
    )


@dataclass(frozen=True)
class StageLinearisation:
    """Discrete-time linear model for a single MPC stage.

    ``x_{k+1} = A @ x_k + B @ u_k + c``
    The reference operating point (x_lin, u_lin) and the stage time ts
    are kept for downstream diagnostics; the QP only consumes (A, B, c).
    """

    A: np.ndarray  # (NX, NX)
    B: np.ndarray  # (NX, NU)
    c: np.ndarray  # (NX,)
    x_lin: np.ndarray
    u_lin: np.ndarray
    ts: float
    kappa_ref: float
    v_ref: float


def f_continuous(
    x: np.ndarray,
    u: np.ndarray,
    *,
    kappa_ref: float,
    pc: PlantConstants,
) -> np.ndarray:
    """Continuous-time dynamics ``dx/dt = f(x, u; kappa_ref)``.

    Linearised tyre forces with the operating point at alpha=0 (the LTV
    machinery wraps this and re-evaluates around the reference trajectory).
    """
    e_lat, e_psi, v_x, v_y, omega, delta, throttle, brake = x
    # Body-slip approximations at each axle (small-angle linearisation; the
    # MPC uses these only inside the linear model — the ODE measures the
    # true Pacejka response). Project convention (vehicle.py): positive y in
    # body frame = leftward, positive delta = left steer, positive alpha
    # gives positive Fy (=leftward force). The slip-angle definition mirrors
    # vehicle.py: alpha = delta - atan2(v_y + a_f*omega, v_x) for the front
    # axle; alpha = -atan2(v_y - a_r*omega, v_x) for the rear (no steer).
    vx_safe = max(abs(v_x), V_FLOOR)
    alpha_front = delta - math.atan2(v_y + pc.a_f * omega, vx_safe)
    alpha_rear = -math.atan2(v_y - pc.a_r * omega, vx_safe)
    # Phase 5.0.1 operating-point slope-only Pacejka (spec §23.2-5.0.1.3
    # alternative row 1, slope-only): symmetric tangent through the origin
    # with the slope evaluated at alpha_op. F_y_bias is 0.0 to preserve
    # odd symmetry across left/right corner transitions (Sprint A
    # chicane). The bias term remains in the formula for diagnostics
    # only — if a future phase per-stage-signs the linearisation, it can
    # be re-enabled without touching this call site.
    Fy_front = pc.F_y_bias_front + pc.C_alpha_front * alpha_front
    Fy_rear = pc.F_y_bias_rear + pc.C_alpha_rear * alpha_rear
    # Saturate inside the friction ellipse using the static Fz peak. This
    # caps the tangent line at the true peak; without it, the affine form
    # would predict ~2× the real Fy at very large alpha.
    Fy_front = float(np.clip(
        Fy_front, -pc.D_lat_front * pc.Fz_front, pc.D_lat_front * pc.Fz_front,
    ))
    Fy_rear = float(np.clip(
        Fy_rear, -pc.D_lat_rear * pc.Fz_rear, pc.D_lat_rear * pc.Fz_rear,
    ))
    # Longitudinal (RWD assumed; brake on both axles).
    Fx_rear = pc.k_throttle * throttle - pc.k_brake_rear * brake
    Fx_front = -pc.k_brake_front * brake
    F_drag = pc.drag_coeff * v_x * abs(v_x)
    cd = math.cos(delta)
    sd = math.sin(delta)
    # Equations (chassis frame, planar). Note Fy_front entering vx via
    # sin(delta) — small-angle term, kept for accuracy.
    e_lat_dot = v_x * math.sin(e_psi) + v_y * math.cos(e_psi)
    e_psi_dot = omega - kappa_ref * v_x
    vx_dot = (Fx_front * cd - Fy_front * sd + Fx_rear - F_drag) / pc.mass + v_y * omega
    vy_dot = (Fx_front * sd + Fy_front * cd + Fy_rear) / pc.mass - v_x * omega
    omega_dot = (pc.a_f * (Fx_front * sd + Fy_front * cd)
                 - pc.a_r * Fy_rear) / max(pc.I_zz, 1e-3)
    return np.array([
        e_lat_dot,
        e_psi_dot,
        vx_dot,
        vy_dot,
        omega_dot,
        u[0],  # delta_dot
        u[1],  # throttle_dot
        u[2],  # brake_dot
    ], dtype=float)


def linearise_stage(
    x_lin: np.ndarray,
    u_lin: np.ndarray,
    *,
    kappa_ref: float,
    v_ref: float,
    pc: PlantConstants,
    ds: float,
) -> StageLinearisation:
    """Build the discrete-time (A, B, c) for one stage by numerical Jacobian.

    Stage time is ``ts = ds / max(v_ref, V_FLOOR)`` — distance-uniform on a
    2 m grid (spec §23.2.3). The Jacobian is computed via central
    differences with hand-tuned step sizes; this is cheap (NX+NU = 11
    perturbations, each one extra ``f_continuous`` call).
    """
    ts = float(ds) / max(float(v_ref), V_FLOOR)
    f0 = f_continuous(x_lin, u_lin, kappa_ref=kappa_ref, pc=pc)
    # Step sizes per state component — picked to be a few percent of the
    # typical operating magnitude.
    eps_x = np.array([1e-2, 1e-3, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3, 1e-3])
    eps_u = np.array([1e-2, 1e-2, 1e-2])
    A_cont = np.zeros((NX, NX))
    for i in range(NX):
        dx = np.zeros(NX)
        dx[i] = eps_x[i]
        fp = f_continuous(x_lin + dx, u_lin, kappa_ref=kappa_ref, pc=pc)
        fm = f_continuous(x_lin - dx, u_lin, kappa_ref=kappa_ref, pc=pc)
        A_cont[:, i] = (fp - fm) / (2.0 * eps_x[i])
    B_cont = np.zeros((NX, NU))
    for j in range(NU):
        du = np.zeros(NU)
        du[j] = eps_u[j]
        fp = f_continuous(x_lin, u_lin + du, kappa_ref=kappa_ref, pc=pc)
        fm = f_continuous(x_lin, u_lin - du, kappa_ref=kappa_ref, pc=pc)
        B_cont[:, j] = (fp - fm) / (2.0 * eps_u[j])
    # Explicit Euler discretisation. For ts ~ 60 ms and the chassis
    # eigenvalues of a typical road car (~5-10 rad/s), explicit Euler is
    # stable; we keep it for speed.
    A = np.eye(NX) + ts * A_cont
    B = ts * B_cont
    # Affine offset: c = ts * (f0 - A_cont @ x_lin - B_cont @ u_lin). This is
    # the standard expansion-point correction for x_{k+1} = x_k + ts * (A_cont
    # (x - x_lin) + B_cont (u - u_lin) + f0) = A x + B u + c.
    c = ts * (f0 - A_cont @ x_lin - B_cont @ u_lin)
    return StageLinearisation(
        A=A, B=B, c=c,
        x_lin=x_lin.copy(), u_lin=u_lin.copy(),
        ts=ts, kappa_ref=kappa_ref, v_ref=v_ref,
    )


def integrate_reference(
    x0: np.ndarray,
    u_seq: np.ndarray,
    *,
    kappa_seq: np.ndarray,
    v_seq: np.ndarray,
    pc: PlantConstants,
    ds: float,
) -> np.ndarray:
    """Forward-roll the nonlinear plant over the horizon.

    Used to (a) seed the SQP linearisation point, (b) re-linearise after
    each outer iteration. Same explicit-Euler discretisation as
    :func:`linearise_stage`.

    Returns
    -------
    x_seq : np.ndarray, shape (N+1, NX)
        Rolled states; x_seq[0] = x0; x_seq[k+1] = x_seq[k] + ts(k) * f(x_seq[k], u_seq[k]).
    """
    n = len(u_seq)
    x_seq = np.zeros((n + 1, NX))
    x_seq[0] = x0
    for k in range(n):
        ts = float(ds) / max(float(v_seq[k]), V_FLOOR)
        f = f_continuous(x_seq[k], u_seq[k], kappa_ref=float(kappa_seq[k]), pc=pc)
        x_seq[k + 1] = x_seq[k] + ts * f
    return x_seq


def compute_axle_force_from_state(
    x_vec: np.ndarray,
    pc: PlantConstants,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Return ``((F_x_front, F_y_front), (F_x_rear, F_y_rear))`` (N) at a state.

    Phase 5.0.3 (spec §23.2-5.0.3.11): extracted from the per-axle
    expressions in :mod:`mpc_qp_ellipse._eval_axle_force_at_lin` so the
    Tier 1 ellipse-saturation feedforward can read the planned-direction
    (F_x, F_y) per axle without going through the QP machinery. Uses the
    same affine Pacejka form (Phase 5.0.1, slope-only operating-point
    linearisation) the QP uses internally so saturation matches the QP's
    envelope view.

    Parameters
    ----------
    x_vec : np.ndarray
        Length-NX MPC state vector. Reads ``v_y, omega, delta, throttle,
        brake`` and uses ``v_x`` (with V_FLOOR floor) as the slip-angle
        denominator.
    pc : PlantConstants

    Returns
    -------
    ((F_x_front, F_y_front), (F_x_rear, F_y_rear)) : tuple of float pairs
        Axle-level forces in Newtons. Sign conventions match
        :func:`f_continuous`: positive lateral = leftward, positive
        longitudinal = forward.
    """
    v_x_safe = max(float(x_vec[IDX_VX]), V_FLOOR)
    v_y = float(x_vec[IDX_VY])
    omega = float(x_vec[IDX_OMEGA])
    delta = float(x_vec[IDX_DELTA])
    throttle = float(x_vec[IDX_THR])
    brake = float(x_vec[IDX_BRK])
    alpha_front = delta - math.atan2(v_y + pc.a_f * omega, v_x_safe)
    alpha_rear = -math.atan2(v_y - pc.a_r * omega, v_x_safe)
    Fy_front = pc.F_y_bias_front + pc.C_alpha_front * alpha_front
    Fy_rear = pc.F_y_bias_rear + pc.C_alpha_rear * alpha_rear
    # Same ellipse-cap as f_continuous, so the planned direction never
    # exceeds the per-axle peak even before Tier 1's saturation pass.
    Fy_front = float(np.clip(
        Fy_front, -pc.D_lat_front * pc.Fz_front, pc.D_lat_front * pc.Fz_front,
    ))
    Fy_rear = float(np.clip(
        Fy_rear, -pc.D_lat_rear * pc.Fz_rear, pc.D_lat_rear * pc.Fz_rear,
    ))
    Fx_front = -pc.k_brake_front * brake
    Fx_rear = pc.k_throttle * throttle - pc.k_brake_rear * brake
    return (Fx_front, Fy_front), (Fx_rear, Fy_rear)


__all__ = [
    "NX", "NU",
    "IDX_E_LAT", "IDX_E_PSI", "IDX_VX", "IDX_VY",
    "IDX_OMEGA", "IDX_DELTA", "IDX_THR", "IDX_BRK",
    "V_FLOOR", "G",
    "PacejkaCoeffs",
    "PlantConstants",
    "StageLinearisation",
    "build_plant_constants",
    "compute_axle_force_from_state",
    "compute_dynamic_fz_per_stage",
    "axle_accelerations_from_trajectory",
    "f_continuous",
    "linearise_stage",
    "integrate_reference",
]
