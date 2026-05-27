"""DP-plan longitudinal-accel reference for the HMPC inner (Lever-3 source).

Task brief 2026-05-27 (DP a_long-ref): the HMPC inner's Lever-3
``a_long_ref`` channel can be sourced either from the **outer NLP** plan
(``ReferenceTrajectory.a_long_ref_at(s)``; short horizon, ~1 s) or from
the **whole-lap DP plan** (this module). The DP plan is the ``v3_dp``
plan source built once against the fitted Pacejka envelope (see
:mod:`longitudinal_planner`); it already knows the apex speed far
upstream of the chicane, so its ``a_long(s)`` lets the inner commit
brake earlier without enlarging the outer horizon.

The DP plan exposes ``v(s)`` (``LongitudinalPlan.speeds`` on
``LongitudinalPlan.distances``) but not ``a_long(s)``. We derive it once
from the kinematic identity along arc length:

    a_long(s) = v(s) · dv/ds

(decel negative when ``v`` is decreasing — the same sign convention the
outer NLP control ``a_long_k`` and the inner chassis-frame ``a_long_k``
use; see ``hmpc_outer`` dynamics ``v_{k+1} = v_k + a_long_k · dt_k`` and
``hmpc_inner_casadi`` ``a_long_k = (Fx − F_drag)/m``). ``dv/ds`` is taken
with NumPy central differences on the (non-uniform) plan grid.

The resulting callable :meth:`DPLongitudinalReference.a_long_at` is a
linear interpolation of the cached ``a_long_dp`` array against the plan's
``distances`` grid, clipped at the grid edges — same convention as the
outer's ``*_at`` accessors.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = ["DPLongitudinalReference"]


@dataclass(frozen=True)
class DPLongitudinalReference:
    """Cached ``a_long_dp(s)`` derived from a :class:`LongitudinalPlan`.

    Attributes
    ----------
    s_grid : np.ndarray
        The DP plan's per-point arc-length grid (m, monotonic). Equal to
        ``LongitudinalPlan.distances``.
    a_long : np.ndarray
        Per-point longitudinal accel ``v · dv/ds`` (m/s²). Decel is
        negative.
    """

    s_grid: np.ndarray
    a_long: np.ndarray

    @classmethod
    def from_plan(cls, plan) -> "DPLongitudinalReference":
        """Build from a :class:`LongitudinalPlan` (``distances`` + ``speeds``).

        Computes ``a_long_dp = v · dv/ds`` via central differences on the
        plan's own (possibly non-uniform) distance grid. ``np.gradient``
        handles the non-uniform spacing and the one-sided endpoints.
        """
        s = np.asarray(plan.distances, dtype=float)
        v = np.asarray(plan.speeds, dtype=float)
        if s.shape != v.shape or s.size < 2:
            raise ValueError(
                "DPLongitudinalReference.from_plan: plan distances/speeds "
                f"must be matching arrays of length >= 2 (got {s.shape}, {v.shape})."
            )
        dv_ds = np.gradient(v, s)
        a_long = v * dv_ds
        return cls(s_grid=s.copy(), a_long=np.asarray(a_long, dtype=float))

    def a_long_at(self, s: float) -> float:
        """Linear-interp ``a_long_dp(s)``; clipped at the grid edges."""
        return float(np.interp(
            float(np.clip(s, self.s_grid[0], self.s_grid[-1])),
            self.s_grid, self.a_long,
        ))

    def sample(self, s_seq: np.ndarray) -> np.ndarray:
        """Vectorised :meth:`a_long_at` over an arc-length sequence."""
        s_arr = np.asarray(s_seq, dtype=float)
        s_clipped = np.clip(s_arr, self.s_grid[0], self.s_grid[-1])
        return np.interp(s_clipped, self.s_grid, self.a_long)
