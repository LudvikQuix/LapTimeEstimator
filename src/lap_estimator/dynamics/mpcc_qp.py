"""MPCC QP build + SQP outer loop (spec §23.3.6.3).

Mirrors the shape of :mod:`mpc_qp` but with the curvilinear plant
(NX_C = 10, NU_C = 4) and the MPCC cost (contour + lag + progress reward +
input-rate / input-curvature smoothness + terminal).

Cost (spec §23.3.7.3, encoded in OSQP per §23.3.7.4):

    J = Σ_k [
        w_contour · n_k²
        + w_lag · (s_k − θ_k)²
        − w_progress · V_θ_k          (linear; enters q)
        + w_du · ‖u_k − u_{k-1}‖²
        + w_du2 · ‖u_k − 2 u_{k-1} + u_{k-2}‖²
      ]
    + w_term · (n_N² + (s_N − θ_N)²)

Constraints:

- Linear-plant dynamics (eliminated via state propagators, as in :mod:`mpc_qp`).
- Friction ellipse — re-uses :func:`mpc_qp_ellipse.build_ellipse_rows` with a
  small adapter that re-shapes the curvilinear (Phi, g) → the chassis-frame
  state layout the helper expects.
- Track edges: ``n_min_k ≤ n_k ≤ n_max_k``, one row per stage (Phase 4.1: a
  symmetric ``|n| ≤ track_half_width − safety_buffer`` per stage).
- Actuator bounds: δ_max, δ_dot_max, throttle/brake ∈ [0, 1], V_θ ∈ [0, V_θ_max].
- Slack: optional α-soft constraint reuse is deferred (the friction ellipse +
  edge constraints together cover the v3.2 α-soft envelope).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np
import scipy.sparse as sp

from .mpc_qp import QPProblem  # reuse the dataclass + solve_qp via composition
from .mpc_qp_ellipse import build_ellipse_rows
from .mpcc_model import (
    IDX_BRK,
    IDX_DELTA,
    IDX_E_PSI,
    IDX_N,
    IDX_OMEGA,
    IDX_S,
    IDX_THETA,
    IDX_THR,
    IDX_V_THETA,
    IDX_VX,
    IDX_VY,
    NU_C,
    NX_C,
    StageLinearisationC,
    integrate_reference_c,
    linearise_stage_c,
)

# v3.2 state indices needed when adapting (Phi, g) to the chassis-frame view
# the ellipse builder expects.
from .mpc_model import (
    IDX_BRK as IDX_BRK_V32,
    IDX_DELTA as IDX_DELTA_V32,
    IDX_OMEGA as IDX_OMEGA_V32,
    IDX_THR as IDX_THR_V32,
    IDX_VX as IDX_VX_V32,
    IDX_VY as IDX_VY_V32,
    NX as NX_V32,
)

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class MPCCWeights:
    """MPCC cost weights (spec §23.3.7.3 defaults from Liniger Bicycle.json).

    Build-time addition (2026-05-24): ``w_psi_e`` is **not** in the spec's
    headline weight table but is a small direct cost on the heading-error
    state ``e_psi²``. Without it, MPCC has no direct heading-tracking
    channel — the contour cost penalises ``n²`` only, which depends on
    heading via the dynamics integration over the horizon (slow). At the
    Sprint A chicane entry the chassis needs a sharp yaw change in <0.5 s
    and the implicit heading control isn't fast enough. The v3.2 MPC has
    ``w_psi = 20`` for the same reason. Default 20.0 here matches v3.2.
    """

    w_contour: float = 100.0
    w_lag: float = 1000.0
    w_progress: float = 2.0     # stored positive; cost subtracts this term
    w_du: float = 1.0
    w_du2: float = 1.0
    w_term: float = 100.0
    w_psi_e: float = 20.0


@dataclass
class MPCCBounds:
    """Hard actuator + state bounds for the MPCC QP."""

    delta_max: float = 0.35
    delta_dot_max: float = 8.0
    throttle_dot_max: float = 5.0
    brake_dot_max: float = 5.0
    v_theta_min: float = 0.0
    v_theta_max: float = 80.0      # ~1.5 · max v_ref; controller refines
    vx_min: float = 0.5
    edge_safety_buffer: float = 0.3   # m inside the track edge


def _build_propagators_c(
    stages: list[StageLinearisationC],
    x0: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Curvilinear analogue of :func:`mpc_qp._build_propagators`.

    Returns ``(Phi, g)`` with ``Phi.shape = (N+1, NX_C, N*NU_C)`` and
    ``g.shape = (N+1, NX_C)`` so each ``x_k = Phi[k] @ u_flat + g[k]``.
    """
    N = len(stages)
    Phi = np.zeros((N + 1, NX_C, N * NU_C))
    g = np.zeros((N + 1, NX_C))
    g[0] = x0.copy()
    for k in range(N):
        Ak = stages[k].A
        Bk = stages[k].B
        ck = stages[k].c
        g[k + 1] = Ak @ g[k] + ck
        Phi[k + 1] = Ak @ Phi[k]
        Phi[k + 1, :, k * NU_C:(k + 1) * NU_C] += Bk
    return Phi, g


def _phi_g_to_v32_view(
    Phi_c: np.ndarray,
    g_c: np.ndarray,
    *,
    n_u_c: int,
    n_decision: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Repack curvilinear (Phi, g) so the v3.2 ellipse builder can read it.

    The ellipse builder reads state slots ``IDX_VX_V32 = 2, IDX_VY_V32 = 3,
    IDX_OMEGA_V32 = 4, IDX_DELTA_V32 = 5, IDX_THR_V32 = 6, IDX_BRK_V32 = 7``.
    Curvilinear has the same physical-state variables at different indices:
    ``IDX_VX = 3, IDX_VY = 4, IDX_OMEGA = 5, IDX_DELTA = 6, IDX_THR = 7,
    IDX_BRK = 8``. Build a synthetic ``(Phi_v32, g_v32)`` of width
    ``n_decision`` (so the ellipse rows are zero-padded over the slack / V_θ
    columns) by copying the rows the builder cares about. Other indices are
    left zeroed — the builder doesn't touch them.
    """
    N_plus_1 = Phi_c.shape[0]
    # Output decision-vector width: pad to n_decision so the builder lays
    # rows over the full QP space including slack/V_θ columns.
    Phi_v32 = np.zeros((N_plus_1, NX_V32, n_decision))
    g_v32 = np.zeros((N_plus_1, NX_V32))
    # Only the u-prefix carries the curvilinear u-sensitivities.
    Phi_v32[:, IDX_VX_V32, :n_u_c] = Phi_c[:, IDX_VX, :]
    Phi_v32[:, IDX_VY_V32, :n_u_c] = Phi_c[:, IDX_VY, :]
    Phi_v32[:, IDX_OMEGA_V32, :n_u_c] = Phi_c[:, IDX_OMEGA, :]
    Phi_v32[:, IDX_DELTA_V32, :n_u_c] = Phi_c[:, IDX_DELTA, :]
    Phi_v32[:, IDX_THR_V32, :n_u_c] = Phi_c[:, IDX_THR, :]
    Phi_v32[:, IDX_BRK_V32, :n_u_c] = Phi_c[:, IDX_BRK, :]
    g_v32[:, IDX_VX_V32] = g_c[:, IDX_VX]
    g_v32[:, IDX_VY_V32] = g_c[:, IDX_VY]
    g_v32[:, IDX_OMEGA_V32] = g_c[:, IDX_OMEGA]
    g_v32[:, IDX_DELTA_V32] = g_c[:, IDX_DELTA]
    g_v32[:, IDX_THR_V32] = g_c[:, IDX_THR]
    g_v32[:, IDX_BRK_V32] = g_c[:, IDX_BRK]
    return Phi_v32, g_v32


def _v32_stages_view(stages_c: list[StageLinearisationC]) -> list:
    """Synthesise v3.2-shaped ``StageLinearisation`` mirrors for the ellipse builder.

    The builder reads ``stages[k].x_lin[IDX_VX_V32]``, ``IDX_VY_V32``,
    ``IDX_OMEGA_V32``, ``IDX_DELTA_V32``, ``IDX_THR_V32``, ``IDX_BRK_V32``.
    We build a thin shim with just an ``x_lin`` attribute of length NX_V32
    (8) populated from the curvilinear ``x_lin`` (10).
    """
    class _Shim:
        __slots__ = ("x_lin",)
        def __init__(self, x_v32: np.ndarray) -> None:
            self.x_lin = x_v32

    shims = []
    for stg in stages_c:
        x_v32 = np.zeros(NX_V32)
        x_v32[IDX_VX_V32] = stg.x_lin[IDX_VX]
        x_v32[IDX_VY_V32] = stg.x_lin[IDX_VY]
        x_v32[IDX_OMEGA_V32] = stg.x_lin[IDX_OMEGA]
        x_v32[IDX_DELTA_V32] = stg.x_lin[IDX_DELTA]
        x_v32[IDX_THR_V32] = stg.x_lin[IDX_THR]
        x_v32[IDX_BRK_V32] = stg.x_lin[IDX_BRK]
        shims.append(_Shim(x_v32))
    return shims


def build_mpcc_qp(
    stages: list[StageLinearisationC],
    x0: np.ndarray,
    *,
    weights: MPCCWeights,
    bounds: MPCCBounds,
    u_prev: np.ndarray,
    half_width_seq: np.ndarray,
    v_theta_max_seq: np.ndarray | None = None,
) -> QPProblem:
    """Assemble the condensed MPCC QP.

    Decision vector layout: ``[u_0, u_1, ..., u_{N-1}]`` stacked (length
    ``N * NU_C``). The curvilinear plant eliminates states; we never
    introduce slack variables here (track-edge constraints are hard).
    """
    N = len(stages)
    n_u = N * NU_C
    n_dec = n_u

    Phi, g = _build_propagators_c(stages, x0)

    # ------------------------------------------------------------------
    # Hessian + linear term.
    # ------------------------------------------------------------------
    H = np.zeros((n_dec, n_dec))
    q = np.zeros(n_dec)

    # Selectors for state slots used in the cost.
    n_row = np.zeros(NX_C)
    n_row[IDX_N] = 1.0
    s_row = np.zeros(NX_C)
    s_row[IDX_S] = 1.0
    theta_row = np.zeros(NX_C)
    theta_row[IDX_THETA] = 1.0

    # Selector for heading-error state (e_psi).
    e_psi_row = np.zeros(NX_C)
    e_psi_row[IDX_E_PSI] = 1.0

    for k in range(1, N + 1):
        # Contouring: w_contour · n_k²
        a = n_row @ Phi[k]
        b = float(n_row @ g[k])
        H += 2.0 * weights.w_contour * np.outer(a, a)
        q += 2.0 * weights.w_contour * b * a

        # Heading-error: w_psi_e · e_psi_k². Build-time addition (not in
        # spec §23.3.7.3); without it, MPCC's heading control is too slow
        # for the Sprint A chicane (see MPCCWeights docstring).
        a = e_psi_row @ Phi[k]
        b = float(e_psi_row @ g[k])
        H += 2.0 * weights.w_psi_e * np.outer(a, a)
        q += 2.0 * weights.w_psi_e * b * a

        # Lag: w_lag · (s_k − θ_k)²
        a = (s_row - theta_row) @ Phi[k]
        b = float((s_row - theta_row) @ g[k])
        H += 2.0 * weights.w_lag * np.outer(a, a)
        q += 2.0 * weights.w_lag * b * a

    # Terminal: w_term · (n_N² + (s_N − θ_N)²)
    a = n_row @ Phi[N]
    b = float(n_row @ g[N])
    H += 2.0 * weights.w_term * np.outer(a, a)
    q += 2.0 * weights.w_term * b * a
    a = (s_row - theta_row) @ Phi[N]
    b = float((s_row - theta_row) @ g[N])
    H += 2.0 * weights.w_term * np.outer(a, a)
    q += 2.0 * weights.w_term * b * a

    # Progress reward: − w_progress · V_θ_k. Linear → enters q.
    # Each u_k = (δ_dot, throttle_dot, brake_dot, V_θ); V_θ is the k-th
    # stage's 4th decision slot.
    for k in range(N):
        q[k * NU_C + IDX_V_THETA] += -float(weights.w_progress)

    # Input smoothness costs.
    # w_du · ‖u_k‖² and w_du2 · ‖u_k − u_{k-1}‖² (with u_{-1} = u_prev).
    R = np.eye(NU_C)
    for k in range(N):
        H[k * NU_C:(k + 1) * NU_C, k * NU_C:(k + 1) * NU_C] += 2.0 * weights.w_du * R
        if k == 0:
            H[0:NU_C, 0:NU_C] += 2.0 * weights.w_du2 * R
            q[0:NU_C] += -2.0 * weights.w_du2 * (R @ u_prev)
        else:
            block = 2.0 * weights.w_du2 * R
            H[k * NU_C:(k + 1) * NU_C, k * NU_C:(k + 1) * NU_C] += block
            H[(k - 1) * NU_C:k * NU_C, (k - 1) * NU_C:k * NU_C] += block
            H[k * NU_C:(k + 1) * NU_C, (k - 1) * NU_C:k * NU_C] += -block
            H[(k - 1) * NU_C:k * NU_C, k * NU_C:(k + 1) * NU_C] += -block

    # ------------------------------------------------------------------
    # Constraints.
    # ------------------------------------------------------------------
    rows_A: list[np.ndarray] = []
    rows_l: list[float] = []
    rows_u: list[float] = []

    # Rate limits on u (4 per stage). V_θ is bounded per stage by
    # ``v_theta_max(k) = min(bounds.v_theta_max, 1.1 * v_ref_max_seq[k])``
    # in the controller — passed in via ``half_width_seq`` companion path
    # would be cleaner, but for now we cap V_θ at the controller-set scalar.
    rate_max = np.array([
        bounds.delta_dot_max,
        bounds.throttle_dot_max,
        bounds.brake_dot_max,
        bounds.v_theta_max,
    ])
    rate_min = np.array([
        -bounds.delta_dot_max,
        -bounds.throttle_dot_max,
        -bounds.brake_dot_max,
        bounds.v_theta_min,
    ])
    if v_theta_max_seq is None:
        v_theta_max_seq = np.full(N, bounds.v_theta_max)
    else:
        v_theta_max_seq = np.asarray(v_theta_max_seq, dtype=float)
    for k in range(N):
        v_theta_cap_k = min(
            float(bounds.v_theta_max),
            float(v_theta_max_seq[min(k, len(v_theta_max_seq) - 1)]),
        )
        for j in range(NU_C):
            row = np.zeros(n_dec)
            row[k * NU_C + j] = 1.0
            rows_A.append(row)
            if j == 3:
                rows_l.append(float(rate_min[j]))
                rows_u.append(float(v_theta_cap_k))
            else:
                rows_l.append(float(rate_min[j]))
                rows_u.append(float(rate_max[j]))

    # State-space bounds: δ, throttle, brake, v_x at each stage.
    delta_row = np.zeros(NX_C)
    delta_row[IDX_DELTA] = 1.0
    thr_row = np.zeros(NX_C)
    thr_row[IDX_THR] = 1.0
    brk_row = np.zeros(NX_C)
    brk_row[IDX_BRK] = 1.0
    vx_row_full = np.zeros(NX_C)
    vx_row_full[IDX_VX] = 1.0
    # Track-edge: |n_k| ≤ half_width_k − safety_buffer
    n_row_full = np.zeros(NX_C)
    n_row_full[IDX_N] = 1.0
    for k in range(1, N + 1):
        for srow, lo, hi in (
            (delta_row, -bounds.delta_max, bounds.delta_max),
            (thr_row, 0.0, 1.0),
            (brk_row, 0.0, 1.0),
            (vx_row_full, bounds.vx_min, 200.0),
        ):
            a = srow @ Phi[k]
            b = float(srow @ g[k])
            row = np.zeros(n_dec)
            row[:n_u] = a
            rows_A.append(row)
            rows_l.append(lo - b)
            rows_u.append(hi - b)
        # Track-edge constraint (per-stage half-width).
        half_w = float(half_width_seq[min(k - 1, len(half_width_seq) - 1)])
        edge_lim = max(0.5, half_w - bounds.edge_safety_buffer)
        a = n_row_full @ Phi[k]
        b = float(n_row_full @ g[k])
        row = np.zeros(n_dec)
        row[:n_u] = a
        rows_A.append(row)
        rows_l.append(-edge_lim - b)
        rows_u.append(edge_lim - b)

    A_dense = np.vstack(rows_A) if rows_A else np.zeros((0, n_dec))
    l_arr = np.array(rows_l)
    u_arr = np.array(rows_u)

    # Symmetrise + Tikhonov.
    H = 0.5 * (H + H.T)
    H += 1e-6 * np.eye(n_dec)

    return QPProblem(
        P=sp.csc_matrix(H),
        q=q,
        A=sp.csc_matrix(A_dense),
        l=l_arr,
        u=u_arr,
        n_decision=n_dec,
    )


def add_mpcc_ellipse_constraints(
    problem: QPProblem,
    stages_c: list[StageLinearisationC],
    Phi_c: np.ndarray,
    g_c: np.ndarray,
    *,
    pc,
    fz_front_per_stage: np.ndarray | None = None,
    fz_rear_per_stage: np.ndarray | None = None,
) -> QPProblem:
    """Append per-axle per-stage friction-ellipse rows by reusing the v3.2 builder.

    Re-shapes the curvilinear ``(Phi_c, g_c)`` into the chassis-state view the
    v3.2 :func:`build_ellipse_rows` expects, calls the builder, then folds
    the resulting rows back onto the QP.
    """
    n_u_c = problem.n_decision  # build_mpcc_qp has no slack vars, n_dec = n_u
    Phi_v32, g_v32 = _phi_g_to_v32_view(
        Phi_c, g_c, n_u_c=n_u_c, n_decision=problem.n_decision,
    )
    stages_v32 = _v32_stages_view(stages_c)
    rows_arr, lbs_arr, ubs_arr = build_ellipse_rows(
        stages_v32, Phi_v32, g_v32,
        pc=pc,
        n_decision=problem.n_decision,
        fz_front_per_stage=fz_front_per_stage,
        fz_rear_per_stage=fz_rear_per_stage,
    )
    if rows_arr.shape[0] == 0:
        return problem
    A_new = sp.vstack([problem.A, sp.csc_matrix(rows_arr)]).tocsc()
    l_new = np.concatenate([problem.l, lbs_arr])
    u_new = np.concatenate([problem.u, ubs_arr])
    return QPProblem(
        P=problem.P, q=problem.q,
        A=A_new, l=l_new, u=u_new,
        n_decision=problem.n_decision,
    )


def _solve_qp_inner(problem: QPProblem, warm_start: np.ndarray | None) -> tuple[np.ndarray, str, float]:
    """One-shot OSQP solve. Mirrors :func:`mpc_qp.solve_qp` settings."""
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
        except Exception:  # noqa: BLE001 — OSQP versions disagree; ignore
            pass
    res = solver.solve()
    status = str(res.info.status)
    solve_t = float(getattr(res.info, "solve_time", 0.0))
    if status in ("solved", "solved inaccurate"):
        return np.asarray(res.x, dtype=float), status, solve_t
    return np.zeros(problem.n_decision), status, solve_t


def solve_sqp_mpcc(
    x0: np.ndarray,
    *,
    u_seq_init: np.ndarray,
    ref,
    v_ref_seq: np.ndarray,
    half_width_seq: np.ndarray,
    pc,
    ds_stage: float,
    weights: MPCCWeights,
    bounds: MPCCBounds,
    u_prev: np.ndarray,
    sqp_max_iter: int = 3,
    enable_ellipse: bool = True,
    v_theta_max_seq: np.ndarray | None = None,
) -> tuple[np.ndarray, dict]:
    """MPCC SQP outer loop. Returns ``(u_seq, stats)`` like :func:`mpc_qp.solve_sqp`."""
    from . import mpc_model

    N = len(u_seq_init)
    u_seq = u_seq_init.copy()
    stats: dict = {
        "status_history": [],
        "solve_time_total": 0.0,
        "iters": 0,
        "infeasible_recovery": False,
        "J_residual": 0.0,
        "sqp_du_inf": 0.0,
        "x_seq_last": None,
        "fz_front_per_stage_last": None,
        "fz_rear_per_stage_last": None,
    }

    fz_f_prev: np.ndarray | None = None
    fz_r_prev: np.ndarray | None = None
    x_seq_last: np.ndarray | None = None

    for it in range(sqp_max_iter):
        # 1. Roll the curvilinear plant.
        x_seq = integrate_reference_c(
            x0, u_seq,
            ref=ref, pc=pc,
            ds_stage=ds_stage,
            v_ref_seq=v_ref_seq,
        )
        x_seq_last = x_seq

        # 2. Per-stage dynamic Fz from the rolled (a_x, a_y) trajectory.
        fz_f_stage = None
        fz_r_stage = None
        if bool(getattr(pc, "dynamic_fz_enabled", False)):
            from .mpcc_model import axle_accelerations_from_trajectory_c
            a_x_seq, a_y_seq = axle_accelerations_from_trajectory_c(
                x_seq, v_ref_seq=v_ref_seq, u_seq=u_seq, ds_stage=ds_stage,
            )
            fz_f_stage, fz_r_stage = mpc_model.compute_dynamic_fz_per_stage(
                pc, a_x_seq, a_y_seq,
                Fz_front_prev=fz_f_prev, Fz_rear_prev=fz_r_prev,
                damping=0.4,
            )
            fz_f_prev = fz_f_stage
            fz_r_prev = fz_r_stage

        # 3. Linearise each stage about (x_seq[k], u_seq[k]).
        stages = [
            linearise_stage_c(
                x_seq[k], u_seq[k],
                ref=ref, pc=pc,
                ds_stage=ds_stage,
                v_ref_stage=float(v_ref_seq[k]),
            )
            for k in range(N)
        ]

        # 4. Build the QP, append ellipse if enabled.
        problem = build_mpcc_qp(
            stages, x0,
            weights=weights, bounds=bounds, u_prev=u_prev,
            half_width_seq=half_width_seq,
            v_theta_max_seq=v_theta_max_seq,
        )
        if enable_ellipse:
            Phi_c, g_c = _build_propagators_c(stages, x0)
            problem = add_mpcc_ellipse_constraints(
                problem, stages, Phi_c, g_c, pc=pc,
                fz_front_per_stage=fz_f_stage,
                fz_rear_per_stage=fz_r_stage,
            )

        # 5. Solve.
        warm = u_seq.flatten()
        x_dec, status, st = _solve_qp_inner(problem, warm_start=warm)
        stats["status_history"].append(status)
        stats["solve_time_total"] += st
        stats["iters"] = it + 1

        if status not in ("solved", "solved inaccurate"):
            stats["infeasible_recovery"] = True
            stats["x_seq_last"] = x_seq_last
            stats["fz_front_per_stage_last"] = fz_f_prev
            stats["fz_rear_per_stage_last"] = fz_r_prev
            return u_seq, stats

        u_new = x_dec[: N * NU_C].reshape(N, NU_C)
        if it == 0:
            u_seq_next = u_new
        else:
            u_seq_next = 0.5 * u_seq + 0.5 * u_new
        stats["sqp_du_inf"] = (
            float(np.max(np.abs(u_seq_next - u_seq)))
            if u_seq.size > 0 else 0.0
        )
        try:
            P_dense = problem.P.toarray()
            stats["J_residual"] = float(
                problem.q @ x_dec + 0.5 * x_dec @ P_dense @ x_dec
            )
        except Exception:  # noqa: BLE001
            stats["J_residual"] = 0.0
        u_seq = u_seq_next

    # Final nonlinear roll of the accepted iterate.
    if u_seq is not None and len(u_seq) > 0:
        try:
            x_seq_last = integrate_reference_c(
                x0, u_seq,
                ref=ref, pc=pc, ds_stage=ds_stage, v_ref_seq=v_ref_seq,
            )
        except Exception:  # noqa: BLE001
            pass
    stats["x_seq_last"] = x_seq_last
    stats["fz_front_per_stage_last"] = fz_f_prev
    stats["fz_rear_per_stage_last"] = fz_r_prev
    return u_seq, stats


__all__ = [
    "MPCCWeights",
    "MPCCBounds",
    "build_mpcc_qp",
    "add_mpcc_ellipse_constraints",
    "solve_sqp_mpcc",
]
