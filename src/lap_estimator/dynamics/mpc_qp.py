"""OSQP problem setup + SQP outer loop for the Phase-5.0 MPC (spec §23.2.7).

Condensed formulation: only the per-stage rate-controls ``u_k`` are
decision variables. States are eliminated by the linear plant equation
``x_{k+1} = A_k x_k + B_k u_k + c_k`` so we end up with N*NU decision
vars (45 for N=15, NU=3) and a quadratic cost in u-space.

This module provides:

- :func:`build_qp` — assemble the condensed (P, q, A_con, l, u) tuple
  from a list of :class:`StageLinearisation` triples + the reference
  trajectory + cost weights.
- :func:`solve_sqp` — call OSQP repeatedly, re-linearising the plant
  around the rolled forward solution between iterations.

The QP is convex by construction:

- Cost: quadratic in ``e_lat``, ``e_psi``, ``v_x - v_ref``, slip-budget
  excess (soft hinge), and ``u`` magnitude/rate. After state elimination
  the Hessian is block-tridiagonal in u; we densify into a single (N*NU,
  N*NU) NumPy array which is then converted to a CSC sparse matrix for
  OSQP.
- Constraints: actuator absolute bounds (state-space → linear), actuator
  rate bounds (direct on u), slip-angle linear inequality per stage,
  speed lower bound per stage, **per-axle friction-ellipse tangent
  half-space (Phase 5.0.1)**.

Phase 5.0.1 (spec §23.2-5.0.1.4) ships the true friction-ellipse-proxy
as a per-stage per-axle linear inequality

    (F_x / (D_long * Fz))^2 + (F_y / (D_lat * Fz))^2 <= 1

linearised about the reference operating point. The pre-Phase-5.0.1
soft cost on slip-budget excess (slack-var path) is preserved as a
feasibility-restoration mechanism — see :func:`add_alpha_constraints`
and :mod:`mpc_qp_ellipse`.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np
import scipy.sparse as sp

from .mpc_model import (
    IDX_DELTA,
    IDX_E_LAT,
    IDX_E_PSI,
    IDX_OMEGA,
    IDX_THR,
    IDX_BRK,
    IDX_VX,
    IDX_VY,
    NU,
    NX,
    StageLinearisation,
    integrate_reference,
    linearise_stage,
)

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class MPCWeights:
    """Soft cost weights (spec §23.2.6).

    Default seed values from the spec table; the MPC controller pulls per-
    driver overrides from the ``control_params.mpc`` JSON sub-block.
    """

    w_lat: float = 50.0
    w_psi: float = 5.0
    w_v: float = 2.0
    w_slip: float = 200.0
    w_du: float = 0.1
    w_du2: float = 0.05
    w_term: float = 100.0
    # Phase 5.3 inner brake-aggression tune (CasADi inner only; ignored by
    # the OSQP solve_sqp path). ``w_a`` is the per-stage cost on
    # ``(a_long - a_long_ref)²`` where ``a_long_ref`` is the outer
    # planner's planned longitudinal accel; ``w_ellipse_soft`` is the
    # quadratic penalty on the friction-ellipse soft constraint
    # (previously hard-coded to 5000.0 in the CasADi inner). Both
    # defaults preserve historical CasADi-inner behaviour: w_a=0.0
    # leaves the a_long cost term off, w_ellipse_soft=5000.0 matches the
    # pre-tune hard-code.
    w_a: float = 0.0
    w_ellipse_soft: float = 5000.0


@dataclass
class MPCBounds:
    """Hard actuator + state bounds (spec §23.2.5)."""

    delta_max: float = 0.35  # rad (~20 deg)
    delta_dot_max: float = 10.0  # rad/s
    throttle_dot_max: float = 5.0  # 1/s; default unlimited-ish
    brake_dot_max: float = 5.0
    vx_min: float = 0.5  # m/s
    # Soft slip cap (rad). Set per-controller from alpha_peak * skill.
    alpha_axle_max: float = 0.12  # ~ 7 deg


@dataclass
class QPProblem:
    """Composed QP ready to hand to OSQP.

    Note: the condensed Hessian is a dense numpy array internally for
    clarity; we sparsify at hand-off. Decision variables: u_0..u_{N-1}
    stacked, shape (N*NU,). Lower/upper bound arrays match A_con's row
    layout.
    """

    P: sp.csc_matrix
    q: np.ndarray
    A: sp.csc_matrix
    l: np.ndarray
    u: np.ndarray
    n_decision: int


def _build_propagators(
    stages: list[StageLinearisation],
    x0: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute the affine map x_k = Phi_k * u_seq + g_k for k = 0..N.

    Returns
    -------
    Phi : (N+1, NX, N*NU) ndarray
        Sensitivity of state x_k to u_0..u_{N-1}.
    g : (N+1, NX) ndarray
        Free response x_k assuming u_seq = 0.
    A_prods : (N+1, NX, NX) ndarray
        Cumulative ``A_{k-1} ... A_0`` so terminal-cost ops can re-use.

    The recursion is

        x_{k+1} = A_k x_k + B_k u_k + c_k

    so the closed form is

        x_k = (prod A_j) x_0 + sum_{j<k} (prod_{m>j} A_m) (B_j u_j + c_j)
    """
    N = len(stages)
    Phi = np.zeros((N + 1, NX, N * NU))
    g = np.zeros((N + 1, NX))
    A_prods = np.zeros((N + 1, NX, NX))
    A_prods[0] = np.eye(NX)
    g[0] = x0.copy()
    # Rolling state-transition.
    for k in range(N):
        Ak = stages[k].A
        Bk = stages[k].B
        ck = stages[k].c
        # Propagate the free response.
        g[k + 1] = Ak @ g[k] + ck
        # Propagate existing sensitivities through Ak.
        Phi[k + 1] = Ak @ Phi[k]
        # Add direct sensitivity to u_k.
        Phi[k + 1, :, k * NU:(k + 1) * NU] += Bk
        A_prods[k + 1] = Ak @ A_prods[k]
    return Phi, g, A_prods


def build_qp(
    stages: list[StageLinearisation],
    x0: np.ndarray,
    *,
    v_ref_seq: np.ndarray,
    weights: MPCWeights,
    bounds: MPCBounds,
    u_prev: np.ndarray,  # last commit u_{-1} for rate-of-rate cost (NU,)
    n_ref_seq: np.ndarray | None = None,
    psi_e_ref_seq: np.ndarray | None = None,
) -> QPProblem:
    """Assemble the condensed QP.

    Cost in u-space — see module docstring. The slip-budget soft cost is
    handled via slack variables to keep the QP convex without resorting
    to a non-smooth hinge:

        max(0, |alpha| - alpha_max) -> introduce s >= 0 with
        s >= alpha - alpha_max,  s >= -(alpha + alpha_max).

    One slack per stage; the cost term is ``w_slip * s_k^2``.

    Phase 5.1 v3.4 hierarchical-MPC inner extension (spec §23.4.6.3):
    optional ``n_ref_seq`` / ``psi_e_ref_seq`` shift the running and
    terminal cross-track / heading-error penalties from centreline
    tracking (``e_lat²``, ``e_psi²``) to outer-reference tracking
    ((e_lat − n_ref)², (e_psi − ψ_e_ref)²). ``None`` (default) preserves
    bit-identical v3.2 behaviour. Algebraically the substitution is a
    pure shift in the ``b_off`` constant of each quadratic stage term —
    the Hessian rows are unchanged; only the linear-term ``q`` vector
    shifts by ``−2 · w · b_off_shift · a``.
    """
    N = len(stages)
    n_u = N * NU
    n_s = N  # one slip-slack per stage
    n_dec = n_u + n_s

    Phi, g, _ = _build_propagators(stages, x0)

    # ------------------------------------------------------------------
    # Build a dense (n_dec, n_dec) Hessian H and linear q.
    # We add per-stage contributions individually for clarity.
    # ------------------------------------------------------------------
    H = np.zeros((n_dec, n_dec))
    q = np.zeros(n_dec)

    # Selector vectors picking out the e_lat / e_psi / v_x rows from the
    # state vector. They map x_k -> the relevant scalar.
    e_lat_row = np.zeros(NX)
    e_lat_row[IDX_E_LAT] = 1.0
    e_psi_row = np.zeros(NX)
    e_psi_row[IDX_E_PSI] = 1.0
    vx_row = np.zeros(NX)
    vx_row[IDX_VX] = 1.0

    # Cost on running stages 1..N (k=0 has no decision dependence; skip).
    for k in range(1, N + 1):
        # e_lat^2 (or (e_lat - n_ref_k)^2 in HMPC inner mode).
        a = e_lat_row @ Phi[k]
        b_off = float(e_lat_row @ g[k])
        if n_ref_seq is not None:
            b_off -= float(n_ref_seq[min(k - 1, N - 1)])
        # quadratic term in u: a^T a * w
        H[:n_u, :n_u] += 2.0 * weights.w_lat * np.outer(a, a)
        q[:n_u] += 2.0 * weights.w_lat * b_off * a
        # e_psi^2 (or (e_psi - psi_e_ref_k)^2 in HMPC inner mode).
        a = e_psi_row @ Phi[k]
        b_off = float(e_psi_row @ g[k])
        if psi_e_ref_seq is not None:
            b_off -= float(psi_e_ref_seq[min(k - 1, N - 1)])
        H[:n_u, :n_u] += 2.0 * weights.w_psi * np.outer(a, a)
        q[:n_u] += 2.0 * weights.w_psi * b_off * a
        # (v_x - v_ref)^2. v_ref is at the START of stage k -> v_ref_seq[k-1]
        # (the reference for the speed we *should* be at when entering
        # stage k); fine to use v_ref_seq[k-1].
        a = vx_row @ Phi[k]
        b_off = float(vx_row @ g[k]) - float(v_ref_seq[min(k - 1, N - 1)])
        H[:n_u, :n_u] += 2.0 * weights.w_v * np.outer(a, a)
        q[:n_u] += 2.0 * weights.w_v * b_off * a

    # Terminal cost on the last stage (weights.w_term scales a sum of
    # state-error squares at horizon end).
    n_ref_terminal = (
        float(n_ref_seq[-1]) if n_ref_seq is not None else 0.0
    )
    psi_e_ref_terminal = (
        float(psi_e_ref_seq[-1]) if psi_e_ref_seq is not None else 0.0
    )
    for term_row, scale, ref_term in (
        (e_lat_row, weights.w_term, n_ref_terminal),
        (e_psi_row, weights.w_term * 0.1, psi_e_ref_terminal),
    ):
        a = term_row @ Phi[N]
        b_off = float(term_row @ g[N]) - ref_term
        H[:n_u, :n_u] += 2.0 * scale * np.outer(a, a)
        q[:n_u] += 2.0 * scale * b_off * a

    # Control magnitude + rate-of-rate cost. Both quadratic in u directly.
    R = np.diag([1.0, 1.0, 1.0])  # uniform; w_du is the overall scalar
    for k in range(N):
        # w_du * u_k^T R u_k
        H[k * NU:(k + 1) * NU, k * NU:(k + 1) * NU] += 2.0 * weights.w_du * R
        # w_du2 * (u_k - u_{k-1})^T R (u_k - u_{k-1}). For k = 0,
        # u_{-1} = u_prev (last-commit rates).
        if k == 0:
            # Cost = w_du2 * (u_0 - u_prev)^T R (u_0 - u_prev).
            H[0:NU, 0:NU] += 2.0 * weights.w_du2 * R
            q[0:NU] += -2.0 * weights.w_du2 * (R @ u_prev)
        else:
            block = 2.0 * weights.w_du2 * R
            H[k * NU:(k + 1) * NU, k * NU:(k + 1) * NU] += block
            H[(k - 1) * NU:k * NU, (k - 1) * NU:k * NU] += block
            H[k * NU:(k + 1) * NU, (k - 1) * NU:k * NU] += -block
            H[(k - 1) * NU:k * NU, k * NU:(k + 1) * NU] += -block

    # Slack costs: w_slip * s_k^2.
    for k in range(N):
        H[n_u + k, n_u + k] += 2.0 * weights.w_slip

    # ------------------------------------------------------------------
    # Constraints. Each row: l <= row(u, s) <= u.
    # ------------------------------------------------------------------
    rows_A: list[np.ndarray] = []
    rows_l: list[float] = []
    rows_u: list[float] = []

    # Rate limits on u (3 per stage).
    rate_max = np.array([bounds.delta_dot_max, bounds.throttle_dot_max,
                         bounds.brake_dot_max])
    for k in range(N):
        for j in range(NU):
            row = np.zeros(n_dec)
            row[k * NU + j] = 1.0
            rows_A.append(row)
            rows_l.append(-float(rate_max[j]))
            rows_u.append(float(rate_max[j]))

    # State-space bounds on delta, throttle, brake (each is a state).
    # delta_k+1 = e_5 @ Phi[k+1] @ u + e_5 @ g[k+1]; constrain to +- delta_max.
    delta_row = np.zeros(NX)
    delta_row[IDX_DELTA] = 1.0
    thr_row = np.zeros(NX)
    thr_row[IDX_THR] = 1.0
    brk_row = np.zeros(NX)
    brk_row[IDX_BRK] = 1.0
    vx_row_full = np.zeros(NX)
    vx_row_full[IDX_VX] = 1.0
    for k in range(1, N + 1):
        for srow, lo, hi in (
            (delta_row, -bounds.delta_max, bounds.delta_max),
            (thr_row, 0.0, 1.0),
            (brk_row, 0.0, 1.0),
            (vx_row_full, bounds.vx_min, 200.0),  # ~720 kph hard ceiling
        ):
            a = srow @ Phi[k]
            b = float(srow @ g[k])
            row = np.zeros(n_dec)
            row[:n_u] = a
            rows_A.append(row)
            rows_l.append(lo - b)
            rows_u.append(hi - b)

    # Slip-angle soft constraints are appended by :func:`add_alpha_constraints`
    # below, after this builder returns — that helper takes the front/rear
    # offsets ``(a_f, a_r)`` and the per-stage linearised denominator
    # ``v_x_lin`` directly so the alpha linearisation matches the SQP outer
    # iteration's plant centre. Here we add only the slack lower bound
    # ``s_k >= 0`` so the slack vars are well-posed in the QP.
    for k in range(N):
        row = np.zeros(n_dec)
        row[n_u + k] = 1.0
        rows_A.append(row)
        rows_l.append(0.0)
        rows_u.append(np.inf)

    A_dense = np.vstack(rows_A) if rows_A else np.zeros((0, n_dec))
    l_arr = np.array(rows_l)
    u_arr = np.array(rows_u)

    # OSQP wants symmetric Hessian and CSC sparse matrices.
    H = 0.5 * (H + H.T)
    # Small Tikhonov for numerical robustness.
    H += 1e-6 * np.eye(n_dec)

    return QPProblem(
        P=sp.csc_matrix(H),
        q=q,
        A=sp.csc_matrix(A_dense),
        l=l_arr,
        u=u_arr,
        n_decision=n_dec,
    )


def add_alpha_constraints(
    problem: QPProblem,
    stages: list[StageLinearisation],
    x0: np.ndarray,
    *,
    a_f: float,
    a_r: float,
    alpha_max: float,
) -> QPProblem:
    """Append per-stage slip-budget soft constraints to ``problem``.

    Front-axle slip-angle is linearised about the reference trajectory:

        alpha_front_k = delta_k - (v_y_k + a_f * omega_k) / v_x_lin_k

    The slack variable ``s_k`` (added by :func:`build_qp` as decision
    var ``n_u + k``) bounds the overshoot. We need two rows per stage
    (one for the upper bound, one for the lower) which add up to ``2N``
    extra rows.

    Returns a new :class:`QPProblem` with the augmented A/l/u.
    """
    N = len(stages)
    n_u = N * NU
    n_dec = problem.n_decision
    # Re-compute propagators so we can read Phi/g (same as build_qp). This
    # duplicates work; cheap enough for N=15.
    Phi, g, _ = _build_propagators(stages, x0)
    new_rows: list[np.ndarray] = []
    new_l: list[float] = []
    new_u: list[float] = []
    for k in range(1, N + 1):
        v_x_lin = max(float(stages[k - 1].x_lin[IDX_VX]), 1.0)
        # alpha_k = delta_k - (v_y_k + a_f * omega_k) / v_x_lin
        # Build row coefficients in u-space.
        coeff_delta = Phi[k, IDX_DELTA, :]
        coeff_vy = Phi[k, IDX_VY, :]
        coeff_om = Phi[k, IDX_OMEGA, :]
        a_coef = coeff_delta - (coeff_vy + a_f * coeff_om) / v_x_lin
        # Constant offset: delta_k0 - (vy_k0 + a_f * omega_k0) / v_x_lin
        b_off = (g[k, IDX_DELTA]
                 - (g[k, IDX_VY] + a_f * g[k, IDX_OMEGA]) / v_x_lin)
        # Upper bound: alpha - s_k <= alpha_max.
        row_up = np.zeros(n_dec)
        row_up[:n_u] = a_coef
        row_up[n_u + (k - 1)] = -1.0
        new_rows.append(row_up)
        new_l.append(-np.inf)
        new_u.append(float(alpha_max - b_off))
        # Lower bound: -alpha - s_k <= alpha_max  ->  alpha + s_k >= -alpha_max.
        row_lo = np.zeros(n_dec)
        row_lo[:n_u] = -a_coef
        row_lo[n_u + (k - 1)] = -1.0
        new_rows.append(row_lo)
        new_l.append(-np.inf)
        new_u.append(float(alpha_max + b_off))
        # Same for rear axle (no delta term).
        a_coef_r = -(coeff_vy - a_r * coeff_om) / v_x_lin
        b_off_r = -(g[k, IDX_VY] - a_r * g[k, IDX_OMEGA]) / v_x_lin
        row_up_r = np.zeros(n_dec)
        row_up_r[:n_u] = a_coef_r
        row_up_r[n_u + (k - 1)] = -1.0
        new_rows.append(row_up_r)
        new_l.append(-np.inf)
        new_u.append(float(alpha_max - b_off_r))
        row_lo_r = np.zeros(n_dec)
        row_lo_r[:n_u] = -a_coef_r
        row_lo_r[n_u + (k - 1)] = -1.0
        new_rows.append(row_lo_r)
        new_l.append(-np.inf)
        new_u.append(float(alpha_max + b_off_r))
    if not new_rows:
        return problem
    A_new = sp.vstack([problem.A, sp.csc_matrix(np.vstack(new_rows))])
    l_new = np.concatenate([problem.l, np.array(new_l)])
    u_new = np.concatenate([problem.u, np.array(new_u)])
    return QPProblem(
        P=problem.P, q=problem.q,
        A=A_new.tocsc(), l=l_new, u=u_new,
        n_decision=n_dec,
    )


def solve_qp(problem: QPProblem, warm_start: np.ndarray | None = None) -> tuple[np.ndarray, str, float]:
    """One-shot OSQP solve.

    Returns
    -------
    x : np.ndarray
        Decision vector (length n_decision); zeros on failure.
    status : str
        OSQP status string (``'solved'``, ``'solved_inaccurate'``,
        ``'primal_infeasible'``, ...).
    solve_time : float
        Wall-clock seconds OSQP reports.
    """
    import osqp
    solver = osqp.OSQP()
    solver.setup(
        problem.P, problem.q, problem.A, problem.l, problem.u,
        eps_abs=1e-3, eps_rel=1e-3,
        max_iter=2000, polishing=False,
        verbose=False, warm_starting=True,
    )
    if warm_start is not None and len(warm_start) == problem.n_decision:
        try:
            solver.warm_start(x=warm_start)
        except Exception:  # noqa: BLE001 — OSQP versions disagree; fall back
            pass
    res = solver.solve()
    status = str(res.info.status)
    solve_t = float(getattr(res.info, "solve_time", 0.0))
    if status in ("solved", "solved inaccurate"):
        x_out = np.asarray(res.x, dtype=float)
        return x_out, status, solve_t
    return np.zeros(problem.n_decision), status, solve_t


def solve_sqp(
    x0: np.ndarray,
    *,
    u_seq_init: np.ndarray,
    kappa_seq: np.ndarray,
    v_ref_seq: np.ndarray,
    pc,  # PlantConstants — typed in mpc_model
    ds: float,
    weights: MPCWeights,
    bounds: MPCBounds,
    alpha_max: float,
    u_prev: np.ndarray,
    sqp_max_iter: int = 3,
    enable_ellipse: bool = True,
    cimpcc_params=None,  # CiMPCCParams | None — Phase 5.0.8 overlay
    n_ref_seq: np.ndarray | None = None,
    psi_e_ref_seq: np.ndarray | None = None,
) -> tuple[np.ndarray, dict]:
    """SQP outer loop: re-linearise + re-solve up to ``sqp_max_iter`` times.

    Each iteration:
      1. Roll the nonlinear plant forward using the current u_seq.
      2. Re-linearise each stage around the rolled trajectory.
      3. Build + solve the QP (with the Phase 5.0.1 ellipse tangent
         half-space if ``enable_ellipse``).
      4. Update u_seq with the new solution.

    On infeasibility, falls back to the slip-cost-bumped problem (the
    spec §23.2.10 first-tier recovery).

    Returns the optimal ``u_seq`` (shape (N, NU)) and a stats dict.
    """
    # Local imports to avoid module-load circulars.
    from . import mpc_model
    from .mpc_qp_cimpcc import CiMPCCParams, add_cimpcc_curvature_cost
    from .mpc_qp_ellipse import add_ellipse_constraints

    # Phase 5.0.8 CiMPCC overlay: default to disabled-no-op so pre-5.0.8
    # callers (and the bit-identical baseline path) get identical QP
    # problems. Caller passes a populated CiMPCCParams to activate.
    cimpcc_active: CiMPCCParams = (
        cimpcc_params if cimpcc_params is not None
        else CiMPCCParams(enabled=False)
    )

    N = len(u_seq_init)
    u_seq = u_seq_init.copy()
    stats: dict = {
        "status_history": [],
        "solve_time_total": 0.0,
        "iters": 0,
        "infeasible_recovery": False,
        # Phase 5.0.3 (spec §23.2-5.0.3.3): post-solve diagnostics
        # consumed by the Tier 1 classifier. ``J_residual`` is the cost
        # at the final accepted iterate; ``sqp_du_inf`` is the
        # ||Δu_seq||_∞ of the last update (close to zero iff SQP
        # converged before sqp_max_iter); ``x_seq_last`` is the
        # nonlinear-rolled trajectory at the final iterate (used by
        # the post-solve ellipse check without an extra plant roll).
        "J_residual": 0.0,
        "sqp_du_inf": 0.0,
        "x_seq_last": None,
        # Phase 5.0.4 (spec §23.2-5.0.4.6 #2): per-stage Fz the final
        # accepted SQP iterate consumed. Used by the post-solve
        # nonlinear ellipse check and the Tier-1 saturation feedforward
        # so they compare against the same envelope the QP saw. ``None``
        # on the static-Fz path.
        "fz_front_per_stage_last": None,
        "fz_rear_per_stage_last": None,
    }

    weights_current = weights
    x_seq_last = None
    # Phase 5.0.4 (spec §23.2-5.0.4.5): per-stage dynamic Fz state carried
    # across SQP outer iterations. ``None`` => first iter / static-Fz path.
    fz_f_prev: np.ndarray | None = None
    fz_r_prev: np.ndarray | None = None
    for it in range(sqp_max_iter):
        # 1. Roll the plant.
        x_seq = integrate_reference(
            x0, u_seq,
            kappa_seq=kappa_seq, v_seq=v_ref_seq,
            pc=pc, ds=ds,
        )
        x_seq_last = x_seq

        # Phase 5.0.4: per-stage dynamic Fz from the rolled trajectory's
        # predicted (a_x, a_y). Damped Picard update against the prior
        # iter's profile (β = 0.4) to suppress the (a_y → Fz → Fy → a_y)
        # feedback loop. Only active when the controller built a dynamic-
        # Fz-aware PlantConstants (otherwise we keep the static Fz from
        # build_plant_constants).
        fz_f_stage = None
        fz_r_stage = None
        if bool(getattr(pc, "dynamic_fz_enabled", False)):
            a_x_seq, a_y_seq = mpc_model.axle_accelerations_from_trajectory(
                x_seq, v_ref_seq=v_ref_seq, ds=ds,
            )
            fz_f_stage, fz_r_stage = mpc_model.compute_dynamic_fz_per_stage(
                pc, a_x_seq, a_y_seq,
                Fz_front_prev=fz_f_prev, Fz_rear_prev=fz_r_prev,
                damping=0.4,
            )
            fz_f_prev = fz_f_stage
            fz_r_prev = fz_r_stage

        # 2. Linearise each stage about (x_lin, u_lin). The continuous-
        # dynamics linearisation (mpc_model.f_continuous) uses
        # ``pc.C_alpha_*`` and ``pc.Fz_*`` only for the ellipse-cap clip
        # of the affine Pacejka; that clip is intentionally conservative
        # against the static-Fz peak. The Phase 5.0.4 per-stage refresh
        # acts on the *ellipse* constraint (build_ellipse_rows) and on
        # the post-solve violation check; the linearised plant Jacobians
        # continue to use the operating-point pc values.
        stages = [
            linearise_stage(
                x_seq[k], u_seq[k],
                kappa_ref=float(kappa_seq[k]),
                v_ref=max(float(v_ref_seq[k]),
                          mpc_model.V_FLOOR),
                pc=pc, ds=ds,
            )
            for k in range(N)
        ]
        # 3. Build + solve QP.
        problem = build_qp(
            stages, x0,
            v_ref_seq=v_ref_seq,
            weights=weights_current,
            bounds=bounds,
            u_prev=u_prev,
            n_ref_seq=n_ref_seq,
            psi_e_ref_seq=psi_e_ref_seq,
        )
        # Slip-budget soft constraint (legacy α-cap; tier-1 of the
        # parent-spec recovery ladder + extra-conservative lateral cap
        # under the §23.2-5.0.1.4 composition rule).
        problem = add_alpha_constraints(
            problem, stages, x0,
            a_f=pc.a_f, a_r=pc.a_r,
            alpha_max=alpha_max,
        )
        # Phase 5.0.1 friction-ellipse-proxy hard constraint (spec
        # §23.2-5.0.1.4): per-stage per-axle tangent half-space about
        # the SQP outer loop's most recent rolled-forward (x_lin, u_lin).
        # Re-linearised each SQP iter; +4N rows = 60 at N=15.
        # Phase 5.0.4: when dynamic_fz_enabled, pass per-stage Fz arrays
        # so each row's (D · Fz)² denominator reflects the predicted
        # weight-transfer state at that stage.
        if enable_ellipse:
            # Re-roll propagators (cheap at N=15). add_ellipse_constraints
            # delegates to mpc_qp_ellipse.build_ellipse_rows.
            Phi, g, _ = _build_propagators(stages, x0)
            problem = add_ellipse_constraints(
                problem, stages, Phi, g, pc=pc,
                fz_front_per_stage=fz_f_stage,
                fz_rear_per_stage=fz_r_stage,
            )
        # Phase 5.0.8 CiMPCC curvature-velocity hinge overlay (additive,
        # appended after the ellipse so the existing ellipse constraints
        # are zero-padded by add_cimpcc_curvature_cost to match the new
        # n_decision width). No-op when params.enabled is False.
        problem = add_cimpcc_curvature_cost(
            problem, stages, x0,
            kappa_seq=kappa_seq,
            mu_lat=float(pc.D_lat_front),
            params=cimpcc_active,
        )
        # Warm start: u-prefix from prior iter, then the existing slip
        # slacks (N), then (when active) the CiMPCC slacks (N). Zero is
        # a safe seed for both — first solve will pull them into the
        # active hinge region.
        n_extra = N + (N if cimpcc_active.enabled and cimpcc_active.weight > 0.0 else 0)
        warm = np.concatenate([u_seq.flatten(), np.zeros(n_extra)])
        x_dec, status, st = solve_qp(problem, warm_start=warm)
        stats["status_history"].append(status)
        stats["solve_time_total"] += st
        stats["iters"] = it + 1
        if status not in ("solved", "solved inaccurate"):
            # Tier-1 recovery: bump w_slip 5x and re-solve once.
            # Phase 5.0.1 (spec §23.2-5.0.1.4): when the ellipse hard
            # constraint causes infeasibility, the slack vars on the
            # α-budget absorb the violation (because the ellipse is
            # geometrically tighter than the α-cap when longitudinal
            # load is present). The ellipse stays hard; the lateral
            # soft-cap relaxes to make room. Same call site as Phase 5.0.
            if not stats["infeasible_recovery"]:
                stats["infeasible_recovery"] = True
                weights_current = MPCWeights(
                    w_lat=weights.w_lat,
                    w_psi=weights.w_psi,
                    w_v=weights.w_v,
                    w_slip=weights.w_slip * 5.0,
                    w_du=weights.w_du,
                    w_du2=weights.w_du2,
                    w_term=weights.w_term,
                )
                continue
            # Tier-2 recovery: bail. Caller hands off to ghost driver.
            # Phase 5.0.3: surface the last rolled trajectory for the
            # tier-1 post-solve ellipse check / planned-direction reuse.
            stats["x_seq_last"] = x_seq_last
            stats["fz_front_per_stage_last"] = fz_f_prev
            stats["fz_rear_per_stage_last"] = fz_r_prev
            return u_seq, stats
        # Extract the u-portion of the decision vector.
        u_new = x_dec[: N * NU].reshape(N, NU)
        # Damped update — small step in u-space to keep linearisation
        # valid. Use full step for the first iteration (good for warm
        # starts); damp later iterations.
        if it == 0:
            u_seq_next = u_new
        else:
            u_seq_next = 0.5 * u_seq + 0.5 * u_new
        # Phase 5.0.3: track ||Δu_seq||_∞ to detect SQP non-convergence.
        # The classifier uses this on the last iteration; intermediate
        # values are overwritten.
        stats["sqp_du_inf"] = float(
            np.max(np.abs(u_seq_next - u_seq))
        ) if u_seq.size > 0 else 0.0
        # Cost residual: q^T x_dec + 0.5 x_dec^T P x_dec. Diagnostic
        # only (used by the soft-divergence classifier against a
        # 100-tick rolling median); cheap given the dense decision
        # vector size (NU*N + N = 60 at default).
        try:
            P_dense = problem.P.toarray()
            stats["J_residual"] = float(
                problem.q @ x_dec + 0.5 * x_dec @ P_dense @ x_dec
            )
        except Exception:  # noqa: BLE001
            stats["J_residual"] = 0.0
        u_seq = u_seq_next
    # Final nonlinear roll of the accepted iterate (post-solve trace).
    # Cheap because integrate_reference is just an Euler march; reused
    # by the Phase 5.0.3 post-solve ellipse-violation check.
    if u_seq is not None and len(u_seq) > 0:
        try:
            x_seq_last = integrate_reference(
                x0, u_seq,
                kappa_seq=kappa_seq, v_seq=v_ref_seq,
                pc=pc, ds=ds,
            )
        except Exception:  # noqa: BLE001
            pass
    stats["x_seq_last"] = x_seq_last
    # Phase 5.0.4: stash the final per-stage Fz so the controller's
    # post-solve check (and Tier 1's saturation feedforward when in
    # ellipse mode) can re-use the same envelope the QP just consumed.
    # Recompute against x_seq_last so it reflects the accepted iterate
    # — fz_f_prev / fz_r_prev are from BEFORE the last build_qp/solve,
    # which is correct for the QP's view but slightly stale for the
    # post-solve check.
    if (
        bool(getattr(pc, "dynamic_fz_enabled", False))
        and x_seq_last is not None
        and len(x_seq_last) > 1
    ):
        try:
            a_x_final, a_y_final = mpc_model.axle_accelerations_from_trajectory(
                x_seq_last, v_ref_seq=v_ref_seq, ds=ds,
            )
            fz_f_final, fz_r_final = mpc_model.compute_dynamic_fz_per_stage(
                pc, a_x_final, a_y_final,
                Fz_front_prev=fz_f_prev, Fz_rear_prev=fz_r_prev,
                damping=0.4,
            )
            stats["fz_front_per_stage_last"] = fz_f_final
            stats["fz_rear_per_stage_last"] = fz_r_final
        except Exception:  # noqa: BLE001
            stats["fz_front_per_stage_last"] = fz_f_prev
            stats["fz_rear_per_stage_last"] = fz_r_prev
    return u_seq, stats


__all__ = [
    "MPCWeights",
    "MPCBounds",
    "QPProblem",
    "build_qp",
    "add_alpha_constraints",
    "solve_qp",
    "solve_sqp",
]
