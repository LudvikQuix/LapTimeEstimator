"""HMPC inner tracker — full v3.2 LTV bicycle against outer reference.

Phase 5.1 v3.4 hierarchical-MPC inner layer (spec §23.4.6.3). This is
the **v3.2 tracking MPC with one change**: the reference profile fed
into the cost (`v_ref_seq`, `n_ref_seq`, `psi_e_ref_seq`) is sampled
from the outer's :class:`ReferenceTrajectory` instead of the DP plan +
centreline. Everything else — the LTV bicycle linearisation, the
friction-ellipse hard constraint, dynamic per-stage Fz, the SQP outer
loop, OSQP backend, alpha-soft slack, slip-target consumption — is
reused unchanged via :func:`mpc_qp.solve_sqp`.

The inner consumes the reference as **a function of curvilinear s**.
Between outer solves the inner re-samples the (frozen) reference at
each tick's stage s-grid, so the inner naturally reads "fresh" values
even though the outer hasn't re-solved. This is the spec §23.4.7.4
frozen-reference policy.

If the reference is infeasible for the inner (QP returns infeasible
after SQP_max_iter retries), the caller falls back to DP-plan tracking
(Tier 1) — `solve_sqp` is just called again with ``n_ref_seq = None``
and ``psi_e_ref_seq = None``, which by the v3.2 contract reverts to
centreline-tracking + DP-plan ``v_ref``.

The inner does **not** own the projection or the outer-reference
storage. The :class:`HMPCController` does. The inner is a stateless
solve-per-tick wrapper.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from time import perf_counter
from typing import TYPE_CHECKING

import numpy as np

from .mpc_model import NU, V_FLOOR
from .mpc_qp import MPCBounds, MPCWeights, solve_sqp

if TYPE_CHECKING:
    from .hmpc_outer import ReferenceTrajectory
    from .mpc_model import PlantConstants

log = logging.getLogger(__name__)


# Defaults — match v3.2 MPC numbers; spec §23.4.7.4.
DEFAULT_HORIZON_M = 30.0
DEFAULT_N_STAGES = 15
DEFAULT_TICK_HZ = 50.0
DEFAULT_SQP_MAX_ITER = 3
DEFAULT_SQP_MAX_ITER_AFTER_OUTER = 4   # spec §23.4.11 Risk 8


@dataclass(frozen=True)
class InnerSolveResult:
    """Output of one inner solve.

    Attributes
    ----------
    u_seq : np.ndarray
        ``(N_inner, NU)`` rate-control sequence. ``u_seq[0]`` is the
        first-stage commit.
    status : str
        OSQP status string (last SQP iter).
    sqp_iters : int
        Number of SQP outer iterations consumed.
    solve_time_s : float
        Wall-clock seconds.
    infeasible : bool
        True iff the SQP returned without a clean ``solved`` /
        ``solved inaccurate`` status. Caller routes to Tier-1 fallback.
    used_outer_ref : bool
        True iff this solve consumed the outer reference. False on a
        Tier-1 DP-plan retry (so diagnostics can split solve time by
        reference source).
    x_seq_last : np.ndarray | None
        Final nonlinear-rolled trajectory (passthrough from
        ``solve_sqp`` stats).
    """

    u_seq: np.ndarray
    status: str
    sqp_iters: int
    solve_time_s: float
    infeasible: bool
    used_outer_ref: bool
    x_seq_last: np.ndarray | None = None


@dataclass
class InnerTrackerConfig:
    """Tunables for :class:`InnerTracker` (spec §23.4.7.3)."""

    horizon_m: float = DEFAULT_HORIZON_M
    n_stages: int = DEFAULT_N_STAGES
    tick_hz: float = DEFAULT_TICK_HZ
    sqp_max_iter: int = DEFAULT_SQP_MAX_ITER


class InnerTracker:
    """v3.2 LTV-bicycle tracking-MPC reading the outer reference.

    Surface mirrors what :class:`MPCController` already does inside its
    ``_resolve_mpc``; we factor it into a callable so the HMPC
    controller doesn't have to duplicate the SQP plumbing.
    """

    def __init__(
        self,
        weights: MPCWeights,
        bounds: MPCBounds,
        pc: "PlantConstants",
        alpha_max: float,
        *,
        config: InnerTrackerConfig | None = None,
    ) -> None:
        self.weights = weights
        self.bounds = bounds
        self.pc = pc
        self.alpha_max = float(alpha_max)
        cfg = config if config is not None else InnerTrackerConfig()
        self.cfg = cfg
        self.ds_stage = float(cfg.horizon_m / max(cfg.n_stages, 1))
        # Diagnostic state surfaced via properties.
        self.solve_times: list[float] = []
        self.solve_count: int = 0
        self.infeasible_count: int = 0
        self.status_history: list[str] = []
        # Warm-start carry — `(N_inner, NU)` zero-init at construction.
        self._u_seq_prev: np.ndarray = np.zeros((cfg.n_stages, NU))
        # Tier-1 (DP-plan) consumption counter.
        self.tier1_count: int = 0
        # Set by the controller right after each outer solve so the
        # next inner tick can bump SQP iters one notch (Risk 8).
        self._extra_sqp_iters_next = 0

    def request_extra_sqp_iter_next(self) -> None:
        """Bump the next inner solve's SQP iter cap by one.

        Used right after an outer solve so the linearisation point
        catches up to the new reference (spec §23.4.11 Risk 8).
        """
        self._extra_sqp_iters_next = 1

    def solve(
        self,
        *,
        x0: np.ndarray,
        kappa_seq: np.ndarray,
        v_ref_seq: np.ndarray,
        n_ref_seq: np.ndarray | None,
        psi_e_ref_seq: np.ndarray | None,
        u_prev: np.ndarray,
        cimpcc_params=None,
        a_long_ref_seq: np.ndarray | None = None,  # noqa: ARG002 — CasADi-only.
    ) -> InnerSolveResult:
        """One inner solve.

        Parameters
        ----------
        x0 : (NX,) np.ndarray
            v3.2 LTV-bicycle initial state.
        kappa_seq : (N_inner,) np.ndarray
            Reference curvature at each stage's start s.
        v_ref_seq : (N_inner,) np.ndarray
            Per-stage reference speed (m/s). HMPC: read from outer
            reference. Tier-1 fallback: DP-plan speed.
        n_ref_seq, psi_e_ref_seq : (N_inner,) np.ndarray | None
            Per-stage outer cross-track / heading-error reference.
            ``None`` for Tier-1 fallback (centreline tracking — bit-
            identical to v3.2 ``--controller mpc``).
        u_prev : (NU,) np.ndarray
            Last commit (for rate-of-rate cost).
        cimpcc_params : CiMPCCParams | None
            Forwarded unchanged to ``solve_sqp``. Not used by HMPC by
            default (the outer already handles brake anticipation; no
            need for the curvature-hinge overlay).
        """
        t0 = perf_counter()
        # Reject non-finite inputs (cheap defensive guard). OSQP raises an
        # opaque OSQPException on NaN/Inf which is harder to diagnose
        # than a clean Tier-1 fallback escalation here.
        if not (
            np.all(np.isfinite(x0))
            and np.all(np.isfinite(kappa_seq))
            and np.all(np.isfinite(v_ref_seq))
            and (n_ref_seq is None or np.all(np.isfinite(n_ref_seq)))
            and (psi_e_ref_seq is None or np.all(np.isfinite(psi_e_ref_seq)))
            and np.all(np.isfinite(u_prev))
        ):
            self.solve_count += 1
            self.infeasible_count += 1
            self.status_history.append("input-nan")
            self.solve_times.append(0.0)
            return InnerSolveResult(
                u_seq=self._u_seq_prev.copy(),
                status="input-nan",
                sqp_iters=0,
                solve_time_s=0.0,
                infeasible=True,
                used_outer_ref=(n_ref_seq is not None or psi_e_ref_seq is not None),
                x_seq_last=None,
            )
        # Warm-start: shifted previous solution.
        u_seq_init = np.zeros_like(self._u_seq_prev)
        u_seq_init[:-1] = self._u_seq_prev[1:]
        u_seq_init[-1] = self._u_seq_prev[-1]
        sqp_cap = int(self.cfg.sqp_max_iter) + int(self._extra_sqp_iters_next)
        self._extra_sqp_iters_next = 0
        u_seq, stats = solve_sqp(
            x0,
            u_seq_init=u_seq_init,
            kappa_seq=kappa_seq,
            v_ref_seq=v_ref_seq,
            pc=self.pc,
            ds=self.ds_stage,
            weights=self.weights,
            bounds=self.bounds,
            alpha_max=self.alpha_max,
            u_prev=u_prev,
            sqp_max_iter=sqp_cap,
            enable_ellipse=True,
            cimpcc_params=cimpcc_params,
            n_ref_seq=n_ref_seq,
            psi_e_ref_seq=psi_e_ref_seq,
        )
        solve_t = perf_counter() - t0
        self.solve_times.append(solve_t)
        self.solve_count += 1
        status_hist = stats.get("status_history", [])
        last_status = str(status_hist[-1]) if status_hist else "no-solve"
        self.status_history.append(last_status)
        infeasible = last_status not in ("solved", "solved inaccurate")
        if infeasible:
            self.infeasible_count += 1
        else:
            # Commit warm-start carry only when the solve succeeded;
            # an infeasible solution stays linearised about the previous
            # warm start on the next tick.
            self._u_seq_prev = u_seq
        used_outer = n_ref_seq is not None or psi_e_ref_seq is not None
        if not used_outer and not infeasible:
            self.tier1_count += 1
        return InnerSolveResult(
            u_seq=u_seq,
            status=last_status,
            sqp_iters=int(stats.get("iters", 0)),
            solve_time_s=solve_t,
            infeasible=infeasible,
            used_outer_ref=used_outer,
            x_seq_last=stats.get("x_seq_last"),
        )


def first_stage_commit(
    u_seq: np.ndarray,
    *,
    actuator_delta: float,
    actuator_throttle: float,
    actuator_brake: float,
    bounds: MPCBounds,
    ds_stage: float,
    v_lin_first: float,
    tick_period: float,
) -> tuple[float, float, float]:
    """Integrate the first-stage rate-controls into committed actuator values.

    Mirrors the rate-clip + state-clip pattern used by
    :class:`MPCController._commit_clean_mpc` / :class:`MPCCController._commit_clean`.
    Returned values are the new committed ``(delta, throttle, brake)`` in
    SI / [0, 1] units, ready to feed back into the chassis.
    """
    ts0 = ds_stage / max(float(v_lin_first), V_FLOOR)
    d_delta = float(u_seq[0, 0]) * ts0
    d_thr = float(u_seq[0, 1]) * ts0
    d_brk = float(u_seq[0, 2]) * ts0
    cap_delta = bounds.delta_dot_max * tick_period
    cap_thr = bounds.throttle_dot_max * tick_period
    cap_brk = bounds.brake_dot_max * tick_period
    d_delta = float(np.clip(d_delta, -cap_delta, cap_delta))
    d_thr = float(np.clip(d_thr, -cap_thr, cap_thr))
    d_brk = float(np.clip(d_brk, -cap_brk, cap_brk))
    new_delta = float(np.clip(
        actuator_delta + d_delta,
        -bounds.delta_max, bounds.delta_max,
    ))
    new_throttle = float(np.clip(actuator_throttle + d_thr, 0.0, 1.0))
    new_brake = float(np.clip(actuator_brake + d_brk, 0.0, 1.0))
    return new_delta, new_throttle, new_brake


__all__ = [
    "DEFAULT_HORIZON_M",
    "DEFAULT_N_STAGES",
    "DEFAULT_TICK_HZ",
    "InnerSolveResult",
    "InnerTracker",
    "InnerTrackerConfig",
    "first_stage_commit",
]
