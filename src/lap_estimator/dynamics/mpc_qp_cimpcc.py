"""CiMPCC curvature-weighted velocity cost overlay (phase 5.0.8).

Implements the asymmetric soft hinge from the *Curvature-Inspired MPCC*
paper (arXiv:2502.03695):

    L_cimpcc[k] = w_kappa * max(0, v_x[k] - v_kappa_safe(kappa[k]))**2

where

    v_kappa_safe(kappa) = sqrt(D_lat * g / max(|kappa|, eps) * safety_kappa)

is the locally-safe speed given the upcoming curvature. The asymmetric
``max(0, ...)`` means **no penalty on straights or when going slow into
corners**, only when the controller carries excess speed into a turn.

The hinge is encoded with one slack variable ``s_v[k]`` per stage
(k = 1..N), exactly mirroring the slip-budget slack pattern in
:mod:`mpc_qp`. The slack carries the asymmetry; the cost is a clean
quadratic ``w_kappa * s_v[k]**2`` and the linear inequality is

    v_x[k] - s_v[k] <= v_kappa_safe[k]
    s_v[k] >= 0

Since ``v_x[k]`` is affine in the decision vector through the LTV plant
``Phi`` / ``g`` (cf. :func:`mpc_qp._build_propagators`), the row is
linear in (u, s_v). Costing only the slack (not v_x directly) means the
unconstrained optimum has ``s_v[k] = max(0, v_x[k] - v_kappa_safe[k])``,
reproducing the desired hinge.

Phase 5.0.8 (CiMPCC overlay):

- This module is additive — pre-Phase-5.0.8 callers that don't invoke
  :func:`add_cimpcc_curvature_cost` get bit-identical QP problems to
  before, regardless of how :class:`MPCWeights` evolves.
- The slack column extension means the returned :class:`QPProblem` has
  ``n_decision = old_n_decision + N``. The MPC controller doesn't need
  to know about the extra columns — only the u-portion (first N*NU
  entries) is read after :func:`mpc_qp.solve_qp` returns.

Defaults: ``w_kappa = 500`` (large vs ``w_v ≈ 0.5`` so the term actually
shapes behaviour near the boundary); ``safety_kappa = 0.95`` (1.0 would
mean ride exactly on the friction limit; 0.95 leaves 5% margin which
matches the rest of the v3.2 pipeline's ``safety_margin = 0.94`` choice).
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

import numpy as np
import scipy.sparse as sp

from .mpc_model import G, IDX_VX, NU, NX, StageLinearisation
from .mpc_qp import QPProblem, _build_propagators

log = logging.getLogger(__name__)

# Smallest curvature we'll divide by. 1e-3 1/m -> R = 1000 m -> straights.
# Below this we treat the section as a straight and skip the hinge entirely.
_KAPPA_FLOOR = 1e-3


@dataclass(frozen=True)
class CiMPCCParams:
    """CLI- / driver-JSON-configurable CiMPCC hyperparameters.

    ``weight`` ('w_kappa' in the spec / paper) scales the quadratic
    slack cost. Compare to ``MPCWeights.w_v`` which scales the symmetric
    speed-tracking cost; CiMPCC is intentionally MUCH larger because it
    only fires on the wrong side of v_kappa_safe.

    ``safety`` ('safety_kappa') multiplies v_kappa_safe down from the
    geometric limit ``sqrt(mu*g*R)``. Pass 1.0 to ride the limit exactly;
    pass <1.0 to leave margin.

    ``enabled`` mirrors the pattern of :class:`mpc_qp.MPCBounds` — a
    single source of truth for "is this overlay active". With
    ``weight = 0`` the overlay is a no-op even when ``enabled = True``,
    but the resize cost stays — keep ``enabled = False`` for true
    bit-identical baselines.
    """

    enabled: bool = False
    weight: float = 500.0
    safety: float = 0.95


def compute_v_kappa_safe(
    kappa_seq: np.ndarray,
    *,
    mu_lat: float,
    safety: float,
) -> np.ndarray:
    """Per-stage locally-safe speed envelope.

    ``v_kappa_safe[k] = sqrt(mu_lat * g / max(|kappa[k]|, eps) * safety)``.

    Returned array has the same shape as ``kappa_seq``. On straights
    (|kappa| < _KAPPA_FLOOR) the entry is +inf so the hinge can't fire.
    """
    k_abs = np.abs(np.asarray(kappa_seq, dtype=float))
    out = np.full_like(k_abs, np.inf)
    mask = k_abs >= _KAPPA_FLOOR
    if not np.any(mask):
        return out
    out[mask] = np.sqrt(mu_lat * G / k_abs[mask] * safety)
    return out


def add_cimpcc_curvature_cost(
    problem: QPProblem,
    stages: list[StageLinearisation],
    x0: np.ndarray,
    *,
    kappa_seq: np.ndarray,
    mu_lat: float,
    params: CiMPCCParams,
) -> QPProblem:
    """Append per-stage curvature-speed slacks to ``problem``.

    Parameters
    ----------
    problem
        The QP built by :func:`mpc_qp.build_qp` (already augmented with
        alpha and optionally ellipse constraints — the order doesn't
        matter; we operate purely on the decision-vector column count
        and on the row count).
    stages
        SQP outer-loop stage linearisations (used for propagator + N).
    x0
        Initial-state vector (used for propagator).
    kappa_seq
        Reference curvature per stage (length N). Stage k corresponds to
        the curvature at the START of the stage; we apply the hinge
        constraint on ``v_x`` at the END of the stage (i.e. ``Phi[k+1]``,
        matching the pattern of :func:`build_qp`'s running-cost loop).
    mu_lat
        Lateral friction peak used in v_kappa_safe. Should be ``D_lat``
        from PlantConstants (front or rear; we use front for the BMW 1M
        since it's the binding axle). The caller selects.
    params
        :class:`CiMPCCParams`. When ``params.enabled`` is False, returns
        the unchanged problem (bit-identical regression path).

    Returns
    -------
    QPProblem
        Augmented problem with ``n_decision = old + N`` (when enabled).

    Notes
    -----
    The slack columns extend the decision vector AFTER all pre-existing
    decision variables (u + slip slacks + any future extensions). OSQP
    treats them uniformly; the MPC controller only reads the u-prefix
    (first ``N * NU`` entries) and is unaffected by the trailing slacks.
    """
    if not params.enabled or params.weight <= 0.0:
        return problem
    N = len(stages)
    if N == 0:
        return problem

    n_dec_old = problem.n_decision
    n_u = N * NU
    n_dec_new = n_dec_old + N

    # ------------------------------------------------------------------
    # v_kappa_safe per stage from the reference curvature.
    # ------------------------------------------------------------------
    v_safe = compute_v_kappa_safe(
        kappa_seq, mu_lat=mu_lat, safety=params.safety,
    )

    # ------------------------------------------------------------------
    # Re-roll propagators so we can read Phi/g for the v_x state. This
    # repeats the cheap O(N * NX^2) work :func:`build_qp` did, mirroring
    # :func:`add_alpha_constraints`'s pattern.
    # ------------------------------------------------------------------
    Phi, g, _ = _build_propagators(stages, x0)
    vx_row = np.zeros(NX)
    vx_row[IDX_VX] = 1.0

    # ------------------------------------------------------------------
    # Hessian extension: w_kappa * s_v[k]**2  ->  diagonal block of 2*w
    # in the new slack rows/cols. The existing P is sparse; convert to
    # block-diagonal with the new slack block appended.
    # ------------------------------------------------------------------
    new_diag = 2.0 * params.weight * sp.eye(N, format="csc")
    P_new = sp.block_diag([problem.P, new_diag], format="csc")

    # q has zero linear term on the new slacks (they only cost
    # quadratically; their LOWER bound at 0 carries the asymmetry).
    q_new = np.concatenate([problem.q, np.zeros(N)])

    # ------------------------------------------------------------------
    # Constraints. Need to (a) extend pre-existing A to n_dec_new cols
    # by appending an N-wide zero block (pre-existing rows don't touch
    # the new slacks), and (b) append 2N new rows:
    #   row 1 (per stage):  v_x[k+1] - s_v[k] <= v_kappa_safe[k]
    #     which in matrix form is  (a · u) - s_v <= v_safe - b_off
    #     i.e. coefficients (a, -1) on (u, s_v) with bounds (-inf, RHS).
    #   row 2 (per stage):  s_v[k] >= 0
    #     i.e. coefficient (0, 1) on (u, s_v) with bounds (0, +inf).
    # For straights (v_safe = +inf) we drop the v_x row but keep the
    # non-negativity row so the slack stays well-defined.
    # ------------------------------------------------------------------
    n_rows_old = problem.A.shape[0]
    if n_rows_old > 0:
        right_pad = sp.csc_matrix((n_rows_old, N))
        A_old_ext = sp.hstack([problem.A, right_pad], format="csc")
    else:
        A_old_ext = sp.csc_matrix((0, n_dec_new))

    new_rows: list[np.ndarray] = []
    new_l: list[float] = []
    new_u: list[float] = []
    for k in range(N):
        # End-of-stage v_x state: row coefficients on u from Phi[k+1].
        # Stage index in kappa_seq / v_safe: k (start-of-stage). For the
        # end-of-stage state we use k+1 in Phi; for the local curvature
        # we use stage k (the curvature the controller is COMMITTING to
        # at this stage, not the one already passed).
        v_kappa = float(v_safe[k])
        a_coef = vx_row @ Phi[k + 1]
        b_off = float(vx_row @ g[k + 1])

        # Always add the non-negativity row so the slack is well-posed.
        row_nn = np.zeros(n_dec_new)
        row_nn[n_dec_old + k] = 1.0
        new_rows.append(row_nn)
        new_l.append(0.0)
        new_u.append(np.inf)

        # Skip the hinge row on straights — leaves the slack untouched
        # so the OSQP solver picks s_v[k] = 0 by the quadratic cost.
        if not math.isfinite(v_kappa):
            continue

        # v_x[k+1] - s_v[k] <= v_kappa_safe[k]
        # -> (a · u_decision) - s_v[k] <= v_kappa_safe[k] - b_off
        row = np.zeros(n_dec_new)
        row[:n_u] = a_coef
        row[n_dec_old + k] = -1.0
        new_rows.append(row)
        new_l.append(-np.inf)
        new_u.append(v_kappa - b_off)

    if not new_rows:
        # All-straight horizon: still extend column count for shape
        # consistency, even though no new rows fire. Empty hstack above
        # already produced A_old_ext at the right width; just return.
        return QPProblem(
            P=P_new,
            q=q_new,
            A=A_old_ext,
            l=problem.l,
            u=problem.u,
            n_decision=n_dec_new,
        )

    A_new = sp.vstack([
        A_old_ext,
        sp.csc_matrix(np.vstack(new_rows)),
    ], format="csc")
    l_new = np.concatenate([problem.l, np.array(new_l)])
    u_new = np.concatenate([problem.u, np.array(new_u)])

    return QPProblem(
        P=P_new,
        q=q_new,
        A=A_new,
        l=l_new,
        u=u_new,
        n_decision=n_dec_new,
    )


__all__ = [
    "CiMPCCParams",
    "compute_v_kappa_safe",
    "add_cimpcc_curvature_cost",
]
