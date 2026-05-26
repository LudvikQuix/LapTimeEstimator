"""Phase-5.0.1 friction-ellipse-proxy QP constraints (spec §23.2-5.0.1.4).

Phase 5.0.4 (spec §23.2-5.0.4.6): the per-stage ``D · Fz`` ellipse
denominators are now consumed per-stage instead of computed once from
``pc.Fz_front`` / ``pc.Fz_rear``. Callers (currently
:func:`mpc_qp.solve_sqp`) pass ``fz_front_per_stage`` / ``fz_rear_per_stage``
arrays; back-compat callers (tests, static-Fz regression path) omit the
kwargs and get the pre-Phase-5.0.4 behaviour.


Per-stage per-axle tangent-half-space encoding of the combined-slip
envelope

    (F_x_axle / (D_long * Fz_axle))^2 + (F_y_axle / (D_lat * Fz_axle))^2 <= 1

linearised about the reference operating point ``(F_x_ref, F_y_ref)``
on each SQP outer iteration. The resulting linear inequality is

    a_x_k * F_x + a_y_k * F_y <= g_ref_k + 2

where
    a_x_k = 2 * F_x_ref / (D_long * Fz)^2
    a_y_k = 2 * F_y_ref / (D_lat  * Fz)^2
    g_ref_k = (F_x_ref / (D_long * Fz))^2 + (F_y_ref / (D_lat * Fz))^2 - 1

(Spec §23.2-5.0.1.4 has ``2 - g_ref_k`` in its algebraic simplification;
the correct expansion is ``a_x*F_x_ref + a_y*F_y_ref - g_ref
= 2*(g_ref+1) - g_ref = g_ref + 2``. We carry the corrected form.)

This is the spec's chosen approach over a fixed M-sided polytopic outer
approximation (constraint count 2N · M = 240+ at M=8) or a fixed inner
polygon (allows infeasible commits near the boundary). One half-space
per axle per stage is 4N rows at N=15 = 60 new rows in OSQP; OSQP solve
time impact is +1–3 ms per solve (well inside the §11.55-5.0.1.G budget
of mean < 30 ms / p99 < 50 ms).

``F_x_axle`` and ``F_y_axle`` are affine in the decision vector through
the LTV bicycle plant:

    F_y_axle = F_y_bias + C_alpha_op * alpha_axle
    alpha_front = delta - (v_y + a_f * omega) / v_x_lin
    alpha_rear  = -(v_y - a_r * omega) / v_x_lin
    F_x_rear    =  k_throttle * throttle - k_brake_rear * brake
    F_x_front   = -k_brake_front * brake

with ``v_x_lin`` from the reference trajectory (held constant inside the
QP per stage). Substituting these into the tangent half-space produces
one linear row per axle per stage.

The composition rule with the pre-existing slack-var soft α-cap is
documented in spec §23.2-5.0.1.4: the hard ellipse takes precedence; the
slack vars exist as a feasibility-restoration mechanism (tier-1 of the
parent-spec recovery ladder). The α-soft constraint also adds an
extra-conservative cap on the lateral half when the longitudinal half is
idle (low-skill driver semantics).

Reference selection (spec §23.2-5.0.1.9 open question): the
``(F_x_ref_k, F_y_ref_k)`` linearisation point comes from the SQP outer
loop's rolled-forward trajectory at the start of the current iteration
— i.e., the planned ``x_lin[k]`` and ``u_lin[k]`` from
``integrate_reference``. This is the operating-point philosophy
consistent with Fix #1: track the planned trajectory through the
iterations. Resolved during build.
"""

from __future__ import annotations

import numpy as np
import scipy.sparse as sp

from .mpc_model import (
    IDX_BRK,
    IDX_DELTA,
    IDX_OMEGA,
    IDX_THR,
    IDX_VX,
    IDX_VY,
    StageLinearisation,
)


# Numerical floor for the squared ellipse denominator. Avoids division-
# by-zero in degenerate Fz / D situations (airborne wheel, calib glitch).
_DENOM_FLOOR_N2 = 1.0  # N^2


def _ls_factor(Fz: float, Fz0: float | None, ls_exp: float | None) -> float:
    """Scalar load-sensitivity factor ``(Fz/Fz0)**(ls_exp - 1)``.

    Returns ``1.0`` when either knob is unset (legacy linear-D ellipse).
    Mirrors :func:`pacejka._load_sens_factor` and
    :func:`longitudinal_planner._ls_factor` for the QP per-stage cap.
    """
    if Fz0 is None or ls_exp is None or Fz0 <= 0.0:
        return 1.0
    if abs(float(ls_exp) - 1.0) < 1e-12:
        return 1.0
    return float(max(Fz, 1.0) / float(Fz0)) ** (float(ls_exp) - 1.0)


def _axle_state_force_coeffs(
    a_offset_signed: float,
    v_x_lin: float,
    pc,
    Phi_k: np.ndarray,
    g_k: np.ndarray,
    *,
    is_front: bool,
) -> tuple[np.ndarray, float, np.ndarray, float]:
    """Return (F_x, F_y) per-axle as affine in the u-decision vector.

    Each axle's force expressions are functions of the state, which is
    itself affine in u via ``x_k = Phi_k @ u_seq + g_k``. We project the
    affine relations through to give

        F_x_axle = a_x_coef @ u + b_x
        F_y_axle = a_y_coef @ u + b_y

    where ``a_x_coef`` and ``a_y_coef`` are length-(N*NU) vectors and
    ``b_x``, ``b_y`` are scalar offsets evaluated at the linearisation
    point (``g_k``).

    Parameters
    ----------
    a_offset_signed : float
        Signed CG-to-axle distance: ``+a_f`` for front, ``-a_r`` for rear.
        Used in the slip-angle expression (positive when the axle is
        ahead of the CG, producing positive alpha for positive omega).
    v_x_lin : float
        Linearisation-point ``v_x`` (m/s, floored above zero).
    pc : PlantConstants
        Carries axle-level Pacejka stiffness / bias, longitudinal
        coefficients, and (implicitly) the ``F_y_bias`` Phase 5.0.1
        offset.
    Phi_k, g_k : np.ndarray
        State propagators for stage k (output of ``_build_propagators``).
    is_front : bool
        Selects axle-specific coefficients. Front axle gets the
        ``delta`` contribution to alpha and (negative) brake-only Fx.
        Rear axle has alpha = -(...) (no delta) and Fx from throttle and
        rear brake.
    """
    # F_y_axle = F_y_bias + C_alpha_op * alpha_axle (Phase 5.0.1 affine form).
    # alpha_axle = sign * delta_term - (v_y + a_offset * omega) / v_x_lin
    coeff_delta = Phi_k[IDX_DELTA, :]
    coeff_vy = Phi_k[IDX_VY, :]
    coeff_om = Phi_k[IDX_OMEGA, :]
    if is_front:
        # alpha_front = delta - (v_y + a_f * omega) / v_x_lin
        alpha_coef = (
            coeff_delta - (coeff_vy + a_offset_signed * coeff_om) / v_x_lin
        )
        alpha_off = (
            g_k[IDX_DELTA]
            - (g_k[IDX_VY] + a_offset_signed * g_k[IDX_OMEGA]) / v_x_lin
        )
        C_alpha = pc.C_alpha_front
        F_y_bias = pc.F_y_bias_front
    else:
        # alpha_rear = -(v_y - a_r * omega) / v_x_lin  (a_offset_signed = -a_r)
        # Equivalent form: -(v_y + a_offset_signed * omega) / v_x_lin where
        # a_offset_signed = -a_r. For rear `is_front=False`, pass
        # a_offset_signed = -a_r so the same formula gives the right sign.
        alpha_coef = -(coeff_vy + a_offset_signed * coeff_om) / v_x_lin
        alpha_off = -(g_k[IDX_VY] + a_offset_signed * g_k[IDX_OMEGA]) / v_x_lin
        C_alpha = pc.C_alpha_rear
        F_y_bias = pc.F_y_bias_rear

    F_y_coef = C_alpha * alpha_coef
    F_y_off = F_y_bias + C_alpha * alpha_off

    # F_x_axle: front = -k_brake_front * brake; rear = k_throttle * throttle
    # - k_brake_rear * brake. Both are linear in the state (throttle and
    # brake are states); we read their sensitivity from Phi_k.
    coeff_thr = Phi_k[IDX_THR, :]
    coeff_brk = Phi_k[IDX_BRK, :]
    if is_front:
        F_x_coef = -pc.k_brake_front * coeff_brk
        F_x_off = -pc.k_brake_front * g_k[IDX_BRK]
    else:
        F_x_coef = (
            pc.k_throttle * coeff_thr - pc.k_brake_rear * coeff_brk
        )
        F_x_off = (
            pc.k_throttle * g_k[IDX_THR] - pc.k_brake_rear * g_k[IDX_BRK]
        )

    return F_x_coef, float(F_x_off), F_y_coef, float(F_y_off)


def _eval_axle_force_at_lin(
    a_offset_signed: float,
    v_x_lin: float,
    pc,
    x_lin_k: np.ndarray,
    *,
    is_front: bool,
) -> tuple[float, float]:
    """Evaluate (F_x_ref, F_y_ref) at the stage linearisation point.

    The reference operating point for the tangent half-space is taken
    from the SQP outer loop's most recent rolled-forward trajectory
    (``stages[k].x_lin``). This matches the operating-point philosophy
    of Fix #1: the ellipse is linearised about the planned-trajectory
    forces, not the integrate-reference free-response forces.
    """
    # alpha at the linearisation point.
    if is_front:
        alpha = (
            float(x_lin_k[IDX_DELTA])
            - (
                float(x_lin_k[IDX_VY])
                + a_offset_signed * float(x_lin_k[IDX_OMEGA])
            )
            / v_x_lin
        )
        C_alpha = pc.C_alpha_front
        F_y_bias = pc.F_y_bias_front
    else:
        alpha = -(
            float(x_lin_k[IDX_VY])
            + a_offset_signed * float(x_lin_k[IDX_OMEGA])
        ) / v_x_lin
        C_alpha = pc.C_alpha_rear
        F_y_bias = pc.F_y_bias_rear
    F_y_ref = F_y_bias + C_alpha * alpha
    if is_front:
        F_x_ref = -pc.k_brake_front * float(x_lin_k[IDX_BRK])
    else:
        F_x_ref = (
            pc.k_throttle * float(x_lin_k[IDX_THR])
            - pc.k_brake_rear * float(x_lin_k[IDX_BRK])
        )
    return float(F_x_ref), float(F_y_ref)


def build_ellipse_rows(
    stages: list[StageLinearisation],
    Phi: np.ndarray,
    g: np.ndarray,
    *,
    pc,
    n_decision: int,
    fz_front_per_stage: np.ndarray | None = None,
    fz_rear_per_stage: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (rows, lb, ub) arrays for the per-stage per-axle ellipse rows.

    One linear row per axle per stage. Stage 0 has no decision dependence
    (the initial state x_0 is fixed); the constraint at stage 0 would be
    trivially satisfied or impossible regardless of u, so we omit it —
    the loop runs over k = 1..N.

    The output ``rows`` has shape ``(num_rows, n_decision)``; OSQP wants
    a row-orderable matrix, so we return dense and let the caller
    sparsify after concatenation. ``ub`` matches per-row; ``lb`` is set
    to ``-inf`` (one-sided inequality).

    Phase 5.0.4 (spec §23.2-5.0.4.6 #1): when ``fz_front_per_stage`` and
    ``fz_rear_per_stage`` are provided (length ``len(stages)``), the
    ellipse denominators ``D · Fz`` are recomputed **per stage** so the
    constraint matches the dynamic-Fz-aware operating point. When
    omitted, the function falls back to ``pc.Fz_front`` / ``pc.Fz_rear``
    broadcast across all stages — the pre-Phase-5.0.4 behaviour.
    """
    N = len(stages)
    a_f = float(pc.a_f)
    a_r = float(pc.a_r)
    fz_f_seq = (
        np.asarray(fz_front_per_stage, dtype=float)
        if fz_front_per_stage is not None
        else np.full(N, float(pc.Fz_front), dtype=float)
    )
    fz_r_seq = (
        np.asarray(fz_rear_per_stage, dtype=float)
        if fz_rear_per_stage is not None
        else np.full(N, float(pc.Fz_rear), dtype=float)
    )
    D_long = float(pc.D_long)
    D_lat_f = float(pc.D_lat_front)
    D_lat_r = float(pc.D_lat_rear)
    # AC non-linear-load knobs from PlantConstants (None => linear-D).
    fz0_f = getattr(pc, "Fz0_front", None)
    fz0_r = getattr(pc, "Fz0_rear", None)
    ls_lat_f_exp = getattr(pc, "ls_exp_lat_front", None)
    ls_lat_r_exp = getattr(pc, "ls_exp_lat_rear", None)
    ls_long_f_exp = getattr(pc, "ls_exp_long_front", None)
    ls_long_r_exp = getattr(pc, "ls_exp_long_rear", None)

    rows: list[np.ndarray] = []
    ubs: list[float] = []
    for k in range(1, N + 1):
        v_x_lin = max(float(stages[k - 1].x_lin[IDX_VX]), 1.0)
        x_lin_k = stages[k - 1].x_lin
        # Per-stage ellipse denominators (Phase 5.0.4). With AC's
        # non-linear load sensitivity each per-axle ``D · Fz_k`` is
        # scaled by ``(Fz_k / FZ0)**(LS_EXP - 1)`` so the cap matches
        # the truth model's per-wheel ``D(Fz) · Fz`` peak. Without the
        # knobs the factor is 1.0 and the original Phase-5.0.4
        # denominators are recovered exactly.
        Fz_f_k = float(fz_f_seq[k - 1])
        Fz_r_k = float(fz_r_seq[k - 1])
        # FZ0 in the driver JSON is per-wheel (matches AC tyres.ini);
        # the per-stage Fz from compute_dynamic_fz_per_stage is per-axle
        # (two wheels), so halve before computing the factor.
        ls_lat_f_k = _ls_factor(0.5 * Fz_f_k, fz0_f, ls_lat_f_exp)
        ls_lat_r_k = _ls_factor(0.5 * Fz_r_k, fz0_r, ls_lat_r_exp)
        ls_long_f_k = _ls_factor(0.5 * Fz_f_k, fz0_f, ls_long_f_exp)
        ls_long_r_k = _ls_factor(0.5 * Fz_r_k, fz0_r, ls_long_r_exp)
        D_long_front_Fz = max(D_long * ls_long_f_k * Fz_f_k, 1.0)
        D_long_rear_Fz = max(D_long * ls_long_r_k * Fz_r_k, 1.0)
        D_lat_front_Fz = max(D_lat_f * ls_lat_f_k * Fz_f_k, 1.0)
        D_lat_rear_Fz = max(D_lat_r * ls_lat_r_k * Fz_r_k, 1.0)
        for is_front, denom_long2, denom_lat2, a_offset_signed in (
            (True, D_long_front_Fz, D_lat_front_Fz, a_f),
            # Rear: alpha_rear = -(v_y - a_r * omega) / v_x_lin -> pass
            # signed offset -a_r so the same alpha = -(v_y + offset*omega)/v_x
            # template gives -(v_y - a_r*omega)/v_x.
            (False, D_long_rear_Fz, D_lat_rear_Fz, -a_r),
        ):
            F_x_coef, F_x_off, F_y_coef, F_y_off = _axle_state_force_coeffs(
                a_offset_signed, v_x_lin, pc,
                Phi[k], g[k], is_front=is_front,
            )
            F_x_ref, F_y_ref = _eval_axle_force_at_lin(
                a_offset_signed, v_x_lin, pc,
                x_lin_k, is_front=is_front,
            )
            # Floor squared denominators well above zero (defensive).
            d_long_sq = max(denom_long2 * denom_long2, _DENOM_FLOOR_N2)
            d_lat_sq = max(denom_lat2 * denom_lat2, _DENOM_FLOOR_N2)
            # Tangent half-space coefficients (gradient of g at ref).
            a_x = 2.0 * F_x_ref / d_long_sq
            a_y = 2.0 * F_y_ref / d_lat_sq
            # g(F_x_ref, F_y_ref) - 1. (d_*_sq = (D*Fz)^2 already, so the
            # ratio (F_x_ref / (D*Fz))^2 = F_x_ref^2 / (D*Fz)^2.)
            g_ref = (
                (F_x_ref * F_x_ref) / d_long_sq
                + (F_y_ref * F_y_ref) / d_lat_sq
            ) - 1.0
            # Final row: a_x*F_x + a_y*F_y <= g_ref + 2 - (a_x*F_x_off +
            # a_y*F_y_off) — substitute F_x = F_x_coef@u + F_x_off into the
            # inequality and move offsets to the rhs.
            row = a_x * F_x_coef + a_y * F_y_coef  # u-space coefficients
            # Pad to n_decision (slack vars sit at the tail; they don't
            # participate in this row).
            full_row = np.zeros(n_decision)
            full_row[: row.shape[0]] = row
            ub = float(g_ref + 2.0 - (a_x * F_x_off + a_y * F_y_off))
            rows.append(full_row)
            ubs.append(ub)

    if not rows:
        return (
            np.zeros((0, n_decision)),
            np.zeros(0),
            np.zeros(0),
        )
    rows_arr = np.vstack(rows)
    ubs_arr = np.array(ubs, dtype=float)
    lbs_arr = np.full(len(ubs), -np.inf, dtype=float)
    return rows_arr, lbs_arr, ubs_arr


def add_ellipse_constraints(
    problem,
    stages: list[StageLinearisation],
    Phi: np.ndarray,
    g: np.ndarray,
    *,
    pc,
    fz_front_per_stage: np.ndarray | None = None,
    fz_rear_per_stage: np.ndarray | None = None,
):
    """Append per-stage ellipse rows to ``problem`` (returns a new QPProblem).

    Re-uses the propagators ``Phi, g`` from ``_build_propagators`` so the
    caller avoids re-rolling them. The function delegates row generation
    to :func:`build_ellipse_rows` and concatenates the result onto the
    QP's existing ``A`` / ``l`` / ``u`` blocks.

    Phase 5.0.4: optional per-stage Fz arrays pass-through to
    :func:`build_ellipse_rows`.
    """
    rows_arr, lbs_arr, ubs_arr = build_ellipse_rows(
        stages, Phi, g, pc=pc, n_decision=problem.n_decision,
        fz_front_per_stage=fz_front_per_stage,
        fz_rear_per_stage=fz_rear_per_stage,
    )
    if rows_arr.shape[0] == 0:
        return problem
    A_new = sp.vstack([problem.A, sp.csc_matrix(rows_arr)]).tocsc()
    l_new = np.concatenate([problem.l, lbs_arr])
    u_new = np.concatenate([problem.u, ubs_arr])
    # Import here to avoid circular import on module load.
    from .mpc_qp import QPProblem
    return QPProblem(
        P=problem.P, q=problem.q,
        A=A_new, l=l_new, u=u_new,
        n_decision=problem.n_decision,
    )


__all__ = [
    "add_ellipse_constraints",
    "build_ellipse_rows",
]
