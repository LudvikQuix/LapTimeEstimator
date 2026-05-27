"""HMPC inner cost-function builder (CasADi inner) — v3.7 asymmetric pedals.

Factored out of :mod:`hmpc_inner_casadi` to keep that module within the
500-line ceiling. This file owns the **per-stage cost terms** the CasADi
NLP uses; the dynamics, decision variables, parameters, and IPOPT
plumbing stay in ``hmpc_inner_casadi.py``.

The legacy (pre-v3.7) per-stage cost was:

    J += w_v · (v_x - v_ref)²
       + w_lat · (n - n_ref)²
       + w_psi · (ψ_e - ψ_e_ref)²
       + (optional w_a · (a_long - a_long_ref)²)
       + w_du · ‖U[:,k] - U[:,k-1]‖²        # combined rate
       + w_du2 · ‖U[:,k] - 2·U[:,k-1] + U[:,k-2]‖²

with the friction ellipse handled as a soft per-axle penalty (kept in
the caller). The combined rate term was the v3.6 bottleneck identified
in ``docs/architecture-v3-hmpc-chase-tomas.md``: the same scalar
``w_du`` penalises brake_dot and throttle_dot equally, which pulls both
pedals away from their natural endpoints (brake wants {0, 1}, throttle
wants smooth ramps). v3.7 splits the rate term per channel and adds an
explicit brake-bang-bang shape:

    J_rate(k) = w_du · (δ̇[k] - δ̇[k-1])²
              + w_du_brake_eff · (ḃ[k] - ḃ[k-1])²
              + w_du_throttle_eff · (ṫ[k] - ṫ[k-1])²

where ``w_du_*_eff`` falls back to ``w_du`` when the channel-specific
knob is 0.0 (back-compat — at the default zeros the rate cost equals
``w_du · sumsqr(U[:,k] - U[:,k-1])`` to within indexing).

Additional v3.7 terms (default 0.0 = off; safe to ship):

    J_brake_dw(k) = w_brake_double_well · brake[k] · (1 - brake[k])
    J_throttle_q(k) = w_throttle · throttle[k]²
    J_overlap(k) = w_brake_throttle_overlap · brake[k] · throttle[k]

The double-well term ``b · (1 - b)`` is concave on [0, 1] with a maximum
of 0.25 at b=0.5 and zeros at b=0 and b=1 — exactly the shape that pulls
the brake decision toward one extreme. IPOPT handles concave terms as
soft non-convex penalties (the iterates settle into one of the wells
based on the rest of the cost surface).

This module is **pure CasADi** — it returns symbolic expressions that
the caller adds into its master ``J`` accumulator. No NumPy arrays, no
parameters, no decision vars — the caller owns those.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import casadi as ca

if TYPE_CHECKING:
    from .mpc_qp import MPCWeights


# State indices into the inner's NX=8 plant state vector. Duplicated
# here (rather than imported from mpc_model) so this helper has no
# import surface beyond MPCWeights.
_IDX_THR = 6
_IDX_BRK = 7


def stage_rate_cost(
    weights: "MPCWeights",
    U: ca.MX,
    k: int,
    p_u_prev: ca.MX,
) -> ca.MX:
    """Per-stage rate-of-control cost — three-channel asymmetric form.

    Parameters
    ----------
    weights
        :class:`MPCWeights` carrying ``w_du`` (legacy combined),
        ``w_du_brake``, ``w_du_throttle``.
    U
        Symbolic ``(NU, N)`` decision matrix; columns are stages.
    k
        Stage index (0-based). ``k == 0`` uses ``p_u_prev`` as the
        previous-stage anchor (matches the legacy ``du0`` term).
    p_u_prev
        Symbolic length-NU parameter (the previous-tick commit), only
        consulted at ``k == 0``.

    Returns
    -------
    Symbolic scalar contribution to ``J``.
    """
    w_du = float(weights.w_du)
    # Channel-specific weights fall back to the legacy combined w_du
    # when the user hasn't set them (default 0.0 sentinel). This keeps
    # the per-stage rate-cost magnitude equal to the legacy
    # ``w_du · sumsqr(U[:,k] - U[:,k-1])`` when all three knobs are at
    # defaults.
    w_du_brk = float(weights.w_du_brake) if weights.w_du_brake > 0.0 else w_du
    w_du_thr = float(weights.w_du_throttle) if weights.w_du_throttle > 0.0 else w_du

    if k == 0:
        du = U[:, 0] - p_u_prev
    else:
        du = U[:, k] - U[:, k - 1]
    # Index layout: U[0] = δ̇, U[1] = ṫ (throttle_dot), U[2] = ḃ
    # (brake_dot). See ``mpc_model.NU = 3`` + ``f_continuous`` ordering.
    d_delta = du[0]
    d_thr = du[1]
    d_brk = du[2]
    return (
        w_du * d_delta ** 2
        + w_du_thr * d_thr ** 2
        + w_du_brk * d_brk ** 2
    )


def stage_pedal_shape_cost(
    weights: "MPCWeights",
    X: ca.MX,
    k: int,
) -> ca.MX:
    """Per-stage asymmetric pedal-shape cost (v3.7).

    Sums three optional terms — all default 0.0, so the legacy CasADi
    inner sees zero contribution when the knobs are unset:

        J = w_brake_double_well · brake · (1 - brake)
          + w_throttle · throttle²
          + w_brake_throttle_overlap · brake · throttle

    Parameters
    ----------
    weights
        :class:`MPCWeights` carrying the v3.7 knobs.
    X
        Symbolic ``(NX, N+1)`` state-trajectory matrix. We read the
        actuator-memory entries ``X[6, k]`` (throttle) and
        ``X[7, k]`` (brake) at stage k.
    k
        Stage index (0-based).

    Returns
    -------
    Symbolic scalar contribution. Returns CasADi MX(0.0) if all three
    weights are 0.0 — the caller can add it unconditionally without
    paying graph-construction cost beyond a constant.
    """
    w_dw = float(weights.w_brake_double_well)
    w_thr_abs = float(weights.w_throttle)
    w_ov = float(weights.w_brake_throttle_overlap)

    # Cheap short-circuit: no terms active.
    if w_dw == 0.0 and w_thr_abs == 0.0 and w_ov == 0.0:
        return ca.MX(0.0)

    throttle = X[_IDX_THR, k]
    brake = X[_IDX_BRK, k]

    j = ca.MX(0.0)
    if w_dw > 0.0:
        # Double-well: concave on [0, 1]; maxes at 0.25 when b=0.5,
        # zeros at b=0 and b=1. IPOPT handles concave-soft-non-convex
        # by settling into whichever well the rest of the cost surface
        # favours; the friction-ellipse soft penalty still keeps the
        # iterate inside the feasible envelope.
        j = j + w_dw * brake * (1.0 - brake)
    if w_thr_abs > 0.0:
        # Quadratic absolute-magnitude penalty on throttle. Default is
        # 0.0 (encourages WOT on straights). Lift if the optimiser
        # over-throttles into corners.
        j = j + w_thr_abs * throttle * throttle
    if w_ov > 0.0:
        # Soft complementarity: brake · throttle ∈ [0, 1]; penalty
        # discourages co-activation without a hard constraint. Trail-
        # brake-to-throttle transitions retain headroom (the penalty
        # is small at low overlap and grows linearly with each pedal).
        j = j + w_ov * brake * throttle
    return j


__all__ = ["stage_rate_cost", "stage_pedal_shape_cost"]
