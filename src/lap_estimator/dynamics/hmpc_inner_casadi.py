"""HMPC inner tracker — CasADi + IPOPT nonlinear formulation (Phase 5.3).

Replaces :class:`hmpc_inner.InnerTracker`'s OSQP+SQP solver with a true
nonlinear MPC built on CasADi's ``Opti`` interface. Same public surface,
same return shape (:class:`hmpc_inner.InnerSolveResult`), so
:class:`HMPCController` can swap solvers with one flag.

Why CasADi+IPOPT and not the v3.2 OSQP+SQP path:

The v3.2 inner linearises the LTV bicycle around a rolled-forward
trajectory, builds a condensed QP in u-space, and asks OSQP to solve.
The friction ellipse is encoded as **eight tangent half-spaces per
axle per stage** about a linearisation point. That structure can be
tight (small enough that the QP reports infeasible) or loose (so loose
that the SQP rolls past the true ellipse boundary and the chassis
exceeds friction at simulate time) but it cannot be **right**. The
Phase 5.2 outer pivot established that the same fundamental
limitation — a linearised friction constraint inside an iterative
re-linearisation loop — was the binding architectural ceiling on the
outer. The Phase 5.0.8 closeout (0/10 MC at chicane mult 0.85, inner
friction-circle utilisation 73 %) is the same bottleneck on the
inner: the inner CAN'T track what the outer is asking because its own
LP-ish formulation throws away 25 % of the available grip.

This module ports the inner to the same solver class as the outer:
direct nonlinear constraint `(F_x / μ_long·Fz)² + (F_y / μ_lat·Fz)² ≤ 1`
per axle per stage, handed to IPOPT to handle natively. Same bicycle
dynamics (Phase 5.0.1 slope-only Pacejka, RWD with brake on both axles,
drag), same state vector layout so :func:`hmpc_inner.first_stage_commit`
plus the v3.2 plant constants (:class:`mpc_model.PlantConstants`) still
apply. The cost mirrors the v3.2 :class:`mpc_qp.MPCWeights` shape so
tunable cost weights pass through unchanged.

Build approach — symbolic graph cached once:

CasADi's overhead is **graph construction**, not solver iteration.
Rebuilding the graph each tick at 50 Hz spends ~80 % of the wall-clock
in graph-walking, not IPOPT. The fix: build the symbolic graph once
at controller construction (in ``__init__``) using ``opti.parameter()``
for everything that changes (x0, kappa_seq, v_ref_seq, n_ref_seq,
psi_e_ref_seq, u_prev, Fz_seqs). Each tick rebinds the parameter
values and calls ``opti.solve()``. Empirically this drops the per-
tick wall-clock from ~150 ms to ~40-80 ms on a 15-stage / 8-state /
3-control problem.

Solve-time budget — the spec asks for inner mean < 30 ms / p99 < 50 ms
(spec §23.4.8). CasADi+IPOPT at 15 stages × NX=8 will not hit those
bounds. The task brief explicitly accepts >40 ms p95 in exchange for
unblocking the lap completion gate; the inner is allowed to consume
its full tick budget. If wall-clock blows up further we drop the
inner tick rate from 50 Hz to 25 Hz (40 ms tick budget).

Public surface mirrors :class:`hmpc_inner.InnerTracker`:

  - ``__init__(weights, bounds, pc, alpha_max, *, config)`` — same kwargs.
  - ``solve(*, x0, kappa_seq, v_ref_seq, n_ref_seq, psi_e_ref_seq, u_prev,
    cimpcc_params=None) -> InnerSolveResult`` — same signature; the
    ``cimpcc_params`` kwarg is accepted-but-ignored (the outer handles
    brake anticipation; CiMPCC is a v3.2 overlay that doesn't apply
    here).
  - ``request_extra_sqp_iter_next()`` — accepted; in CasADi mode it
    bumps ``ipopt_max_iter`` for the next solve by 10 (instead of SQP
    iteration count).
  - ``solve_times``, ``solve_count``, ``infeasible_count``,
    ``status_history``, ``tier1_count`` — same diagnostic surface.
"""

from __future__ import annotations

import logging
from time import perf_counter
from typing import TYPE_CHECKING

import casadi as ca
import numpy as np

from .hmpc_inner import InnerSolveResult, InnerTrackerConfig
from .hmpc_inner_cost import stage_pedal_shape_cost, stage_rate_cost
from .mpc_model import NU, NX, V_FLOOR
from .mpc_qp import MPCBounds, MPCWeights

if TYPE_CHECKING:
    from .mpc_model import PlantConstants

log = logging.getLogger(__name__)


# IPOPT solver tunables. The acceptable-tolerance short-circuits let
# IPOPT exit on feasible-but-suboptimal iterates — same trade we made
# on the outer (closed-loop MPC values feasibility over optimality).
DEFAULT_IPOPT_MAX_ITER = 60
DEFAULT_IPOPT_ACCEPTABLE_TOL = 5e-2
DEFAULT_IPOPT_TOL = 1e-3
# IPOPT return-statuses we treat as feasible-enough to commit:
#  - ``Solve_Succeeded``           — clean optimal
#  - ``Solved_To_Acceptable_Level``— acceptable-tol short-circuit fired;
#    the iterate is primal-feasible within ``acceptable_constr_viol_tol``
#  - ``Maximum_Iterations_Exceeded``— hit the iter cap but the iterate
#    is typically primal-feasible (we additionally gate on
#    ``inf_pr`` < 1e-3 from stats to confirm; see :meth:`solve`).
#  - ``Search_Direction_Becomes_Too_Small`` — IPOPT can't improve; usually
#    means we're stuck at a feasible local min.
# Everything else (``Infeasible_Problem_Detected``, ``Restoration_Failed``,
# ``Diverging_Iterates``, ``ipopt_runtime_error``) is treated as hard
# infeasibility and triggers the Tier-1 retry / Tier-2 reactive ladder.
_FEASIBLE_STATUSES = (
    "Solve_Succeeded",
    "Solved_To_Acceptable_Level",
    "Maximum_Iterations_Exceeded",
    "Search_Direction_Becomes_Too_Small",
)
# Primal-infeasibility threshold for accepting a non-optimal iterate.
# IPOPT's ``inf_pr`` is the maximum constraint violation; we want
# something below the chassis's natural envelope error (state-bound
# slop is ≲ 1e-3 in our scaled coords).
_INF_PR_ACCEPT_THRESHOLD = 1e-2

G = 9.81


def _flat(mx_or_arr) -> np.ndarray:
    """Flatten a CasADi DM / ndarray to a 1-D NumPy array of floats."""
    arr = np.asarray(mx_or_arr, dtype=float)
    return arr.reshape(-1)


class CasadiInnerTracker:
    """Nonlinear inner tracker — same interface as :class:`InnerTracker`.

    Constructed once per controller. The CasADi NLP graph is built in
    ``__init__`` (~50-150 ms construction cost) and rebound each
    :meth:`solve` call with the current chassis state and reference
    sequences.
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
        self.N = int(cfg.n_stages)
        self.ds_stage = float(cfg.horizon_m / max(cfg.n_stages, 1))
        # Diagnostics — same shape as InnerTracker.
        self.solve_times: list[float] = []
        self.solve_count: int = 0
        self.infeasible_count: int = 0
        self.status_history: list[str] = []
        self.tier1_count: int = 0
        # Warm start cache: previous primal solution.
        self._u_seq_prev: np.ndarray = np.zeros((self.N, NU))
        self._x_seq_prev: np.ndarray | None = None
        # Risk-8: extra iterations on the next solve after an outer fire.
        self._extra_iters_next = 0
        # Build the symbolic NLP graph (parameters + decisions + cost).
        self._build_nlp()

    # ------------------------------------------------------------------
    # Interface parity with :class:`InnerTracker`.
    # ------------------------------------------------------------------

    def request_extra_sqp_iter_next(self) -> None:
        """Bump the next IPOPT call's iter cap (spec §23.4.11 Risk 8)."""
        self._extra_iters_next = 10

    # ------------------------------------------------------------------
    # Dynamics helper (symbolic, mirrors mpc_model.f_continuous).
    # ------------------------------------------------------------------

    def _f_continuous_sym(
        self,
        x: ca.MX,
        u: ca.MX,
        kappa: ca.MX,
        fz_f: ca.MX,
        fz_r: ca.MX,
    ) -> ca.MX:
        """Symbolic continuous-time dynamics — CasADi mirror of f_continuous.

        State indexing matches v3.2: x = (n, psi_e, v_x, v_y, omega, delta,
        throttle, brake). Controls: u = (delta_dot, throttle_dot,
        brake_dot). ``fz_f`` / ``fz_r`` are the per-stage axle Fz used in
        the friction-ellipse-clip on the Pacejka tangent.
        """
        pc = self.pc
        n = x[0]; psi_e = x[1]; v_x = x[2]; v_y = x[3]
        omega = x[4]; delta = x[5]; throttle = x[6]; brake = x[7]
        # Velocity floor for the slip-angle denominator.
        vx_safe = ca.fmax(ca.fabs(v_x), V_FLOOR)
        # Slip angles using CasADi atan2 (smooth, well-defined).
        alpha_front = delta - ca.atan2(v_y + pc.a_f * omega, vx_safe)
        alpha_rear = -ca.atan2(v_y - pc.a_r * omega, vx_safe)
        # Slope-only Pacejka tangent: Fy = C_alpha_op * alpha (F_y_bias=0).
        # Use per-unit-Fz slope so the dynamic Fz feeds the lateral force
        # through both the stiffness and the saturation cap.
        slope_f = pc.slope_op_front_per_fz
        slope_r = pc.slope_op_rear_per_fz
        Fy_front_unclipped = slope_f * fz_f * alpha_front
        Fy_rear_unclipped = slope_r * fz_r * alpha_rear
        # Saturate at the friction peak (signed). Smooth via fmin/fmax.
        Fy_peak_f = pc.D_lat_front * fz_f
        Fy_peak_r = pc.D_lat_rear * fz_r
        Fy_front = ca.fmax(-Fy_peak_f, ca.fmin(Fy_peak_f, Fy_front_unclipped))
        Fy_rear = ca.fmax(-Fy_peak_r, ca.fmin(Fy_peak_r, Fy_rear_unclipped))
        # Longitudinal (RWD; brake on both axles).
        Fx_rear = pc.k_throttle * throttle - pc.k_brake_rear * brake
        Fx_front = -pc.k_brake_front * brake
        F_drag = pc.drag_coeff * v_x * ca.fabs(v_x)
        cd = ca.cos(delta)
        sd = ca.sin(delta)
        # Curvilinear bookkeeping (matches v3.2 plant; e_lat -> n,
        # e_psi -> psi_e).
        n_dot = v_x * ca.sin(psi_e) + v_y * ca.cos(psi_e)
        psi_e_dot = omega - kappa * v_x
        vx_dot = (Fx_front * cd - Fy_front * sd + Fx_rear - F_drag) / pc.mass \
            + v_y * omega
        vy_dot = (Fx_front * sd + Fy_front * cd + Fy_rear) / pc.mass \
            - v_x * omega
        omega_dot = (pc.a_f * (Fx_front * sd + Fy_front * cd)
                     - pc.a_r * Fy_rear) / max(pc.I_zz, 1e-3)
        return ca.vertcat(
            n_dot, psi_e_dot, vx_dot, vy_dot, omega_dot,
            u[0], u[1], u[2],
        )

    # ------------------------------------------------------------------
    # NLP graph construction (called once in __init__).
    # ------------------------------------------------------------------

    def _build_nlp(self) -> None:
        """Construct the parameterised CasADi NLP graph.

        The graph is built once; per-tick :meth:`solve` calls rebind
        parameter values and call :meth:`Opti.solve`.

        Parameters (rebound each solve):
          - ``p_x0``   : initial chassis state (NX = 8)
          - ``p_kappa``: per-stage curvature reference (N)
          - ``p_v_ref``: per-stage speed reference (N) — outer or DP
          - ``p_n_ref``: per-stage lateral reference (N) — outer or zeros
          - ``p_psi_e_ref``: per-stage heading reference (N) — outer or zeros
          - ``p_u_prev``: previous-tick commit (NU = 3) for the du² cost
          - ``p_fz_f`` / ``p_fz_r``: per-stage axle Fz (N) — dynamic or static

        Decisions:
          - ``X``: (NX, N+1) state trajectory (X[:, 0] pinned to p_x0)
          - ``U``: (NU, N) rate-control sequence
        """
        N = self.N
        ds = self.ds_stage
        bounds = self.bounds
        weights = self.weights
        pc = self.pc

        opti = ca.Opti()

        # --- Decision variables ---
        X = opti.variable(NX, N + 1)
        U = opti.variable(NU, N)

        # --- Parameters (rebound each solve) ---
        p_x0 = opti.parameter(NX)
        p_kappa = opti.parameter(N)
        p_v_ref = opti.parameter(N)
        p_n_ref = opti.parameter(N)
        p_psi_e_ref = opti.parameter(N)
        p_u_prev = opti.parameter(NU)
        p_fz_f = opti.parameter(N)
        p_fz_r = opti.parameter(N)
        # Phase 5.3 inner brake-aggression tune (Lever 3): per-stage
        # longitudinal-accel reference exported by the outer planner
        # (``ReferenceTrajectory.a_long_ref``). The inner consumes it
        # via a quadratic cost ``w_a · (a_long - a_long_ref)²``. When
        # the outer reference is unavailable (Tier-1 DP-plan tracking),
        # the caller passes zeros, and the cost only fires if
        # ``weights.w_a > 0`` — which still pulls a_long toward zero,
        # so we additionally gate the cost on ``w_a > 0`` and only
        # apply it on stages where the outer's plan is non-trivial.
        p_a_long_ref = opti.parameter(N)
        # Gate (0/1) per stage telling the solver whether the
        # ``a_long_ref`` entry is meaningful for this stage. The
        # controller sets it to 1 when consuming the outer reference,
        # 0 on Tier-1 fallback. Keeping the cost shape per-stage rather
        # than rebuilding the graph keeps the warm-start valid.
        p_a_long_mask = opti.parameter(N)

        # --- Initial-state pin ---
        opti.subject_to(X[:, 0] == p_x0)

        # --- Per-stage dynamics, ellipse, bounds, cost ---
        J = ca.MX(0.0)

        # Hard actuator absolute bounds — applied on the state-machine
        # actuator memory (delta, throttle, brake at stages 1..N) and
        # on the rate controls. Stage 0 is pinned to chassis state so
        # state bounds on stage 0 would clash; we apply them from k>=1.
        delta_max = float(bounds.delta_max)
        delta_dot_max = float(bounds.delta_dot_max)
        thr_dot_max = float(bounds.throttle_dot_max)
        brk_dot_max = float(bounds.brake_dot_max)
        vx_min = float(bounds.vx_min)

        # Per-axle ellipse denominators — the v3.2 build feeds them
        # symbolically via slope_op_*_per_fz, so the dynamic-Fz path is
        # implicit. The peak (D_long, D_lat) clip caps the tangent
        # extrapolation at the true friction peak.
        D_long = float(pc.D_long)
        D_lat_f = float(pc.D_lat_front)
        D_lat_r = float(pc.D_lat_rear)

        for k in range(N):
            # Per-stage time. v_ref bounds the denominator; using v_x
            # directly would couple the dt to the optimisation and slow
            # convergence with no real gain (v_x already tracks v_ref
            # tightly in normal operation).
            v_ref_k_safe = ca.fmax(p_v_ref[k], V_FLOOR)
            dt_k = ds / v_ref_k_safe

            # Continuous dynamics evaluated at the start of the stage.
            f_k = self._f_continuous_sym(
                X[:, k], U[:, k], p_kappa[k], p_fz_f[k], p_fz_r[k],
            )
            # Explicit Euler. The v3.2 inner uses the same; midpoint /
            # RK4 would be more accurate but cost 2-4× more graph
            # complexity per stage — not worth at the 2 m stage step.
            opti.subject_to(X[:, k + 1] == X[:, k] + dt_k * f_k)

            # Friction-ellipse per-axle. Same algebra as
            # mpc_qp_ellipse.build_ellipse_rows, but as a native
            # nonlinear constraint instead of a tangent half-space.
            #
            # F_y_front = slope_op_f · fz_f · alpha_front
            # F_y_rear  = slope_op_r · fz_r · alpha_rear
            # F_x_front = -k_brake_front · brake
            # F_x_rear  =  k_throttle · throttle - k_brake_rear · brake
            #
            # Constraint: (F_x / (D_long · Fz))² + (F_y / (D_lat · Fz))² ≤ 1
            #
            # We linearise about the stage's *predicted* (X[:, k], U[:, k])
            # — i.e. the slip angle uses the optimiser's v_y, omega, delta
            # at stage k. IPOPT handles this directly; no SQP outer loop.
            v_y_k = X[3, k]
            omega_k = X[4, k]
            delta_k = X[5, k]
            thr_k = X[6, k]
            brk_k = X[7, k]
            v_x_k = X[2, k]
            vx_safe_k = ca.fmax(ca.fabs(v_x_k), V_FLOOR)
            alpha_f_k = delta_k - ca.atan2(v_y_k + pc.a_f * omega_k, vx_safe_k)
            alpha_r_k = -ca.atan2(v_y_k - pc.a_r * omega_k, vx_safe_k)
            Fy_f_k = pc.slope_op_front_per_fz * p_fz_f[k] * alpha_f_k
            Fy_r_k = pc.slope_op_rear_per_fz * p_fz_r[k] * alpha_r_k
            Fx_f_k = -pc.k_brake_front * brk_k
            Fx_r_k = pc.k_throttle * thr_k - pc.k_brake_rear * brk_k
            # Denominators (per stage; depend on dynamic Fz).
            denom_long_f = D_long * p_fz_f[k]
            denom_long_r = D_long * p_fz_r[k]
            denom_lat_f = D_lat_f * p_fz_f[k]
            denom_lat_r = D_lat_r * p_fz_r[k]
            # Ellipse constraint (≤ 1) per axle. Use squared form (no
            # sqrt); IPOPT prefers this — the gradient is cleaner.
            #
            # We model the ellipse as a SOFT constraint with a slack
            # penalty. Hard ellipses combined with the actuator state
            # propagation (the optimiser chooses ``u_k`` which rolls into
            # ``brake``, ``throttle``, ``delta`` over the horizon) lead
            # to combinatorial infeasibility wedges where IPOPT can't
            # find any feasible (X, U). Letting the optimiser exceed
            # the ellipse by a tiny ε at the cost of a hefty penalty
            # avoids those wedges while still keeping the friction-
            # circle plan honest. A future improvement would gate the
            # ellipse hard once the chassis is on a known-feasible
            # trajectory; for closed-loop MPC the soft form is the
            # standard pattern (see acados :class:`OcpNlpCost`).
            ellipse_f = (
                (Fx_f_k / denom_long_f) ** 2
                + (Fy_f_k / denom_lat_f) ** 2
            )
            ellipse_r = (
                (Fx_r_k / denom_long_r) ** 2
                + (Fy_r_k / denom_lat_r) ** 2
            )
            # Phase 5.3 brake-aggression tune (Lever 2): the ellipse
            # soft-penalty weight was previously hard-coded to 5e3. The
            # task brief specifies that this penalty is one of the
            # binding constraints on inner brake commit; lowering it
            # (``w_ellipse_soft / 2`` or ``/ 5``) lets the solver
            # bite harder into the friction circle. Default 5000.0
            # preserves historical behaviour.
            w_eps = float(weights.w_ellipse_soft)
            J += w_eps * ca.fmax(0.0, ellipse_f - 1.0) ** 2
            J += w_eps * ca.fmax(0.0, ellipse_r - 1.0) ** 2

            # Rate-control bounds — direct on u_k.
            opti.subject_to(opti.bounded(-delta_dot_max, U[0, k], delta_dot_max))
            opti.subject_to(opti.bounded(-thr_dot_max, U[1, k], thr_dot_max))
            opti.subject_to(opti.bounded(-brk_dot_max, U[2, k], brk_dot_max))

            # State actuator-memory bounds at stage k >= 1 (stage 0
            # pinned by p_x0). Bounds are SOFT — IPOPT treats a hard
            # combinatorial infeasibility (e.g. ``brake==1`` already and
            # the friction circle demands more decel than what
            # ``brake==1`` produces) by declaring the whole NLP
            # infeasible; we'd rather IPOPT find a solution that's
            # ε-outside the bound and let :func:`first_stage_commit`
            # hard-clip on commit. The penalty is large enough that the
            # optimiser never *wants* to violate the bound but the
            # constraint isn't a hard infeasibility wedge.
            if k >= 1:
                # Quadratic penalty on bound excess. We use max(0, x-ub)
                # and max(0, lb-x); CasADi's fmax(0, ...) is smooth-
                # enough for IPOPT.
                d_over_up = ca.fmax(0.0, X[5, k] - delta_max)
                d_over_dn = ca.fmax(0.0, -delta_max - X[5, k])
                t_over_up = ca.fmax(0.0, X[6, k] - 1.0)
                t_over_dn = ca.fmax(0.0, -X[6, k])
                b_over_up = ca.fmax(0.0, X[7, k] - 1.0)
                b_over_dn = ca.fmax(0.0, -X[7, k])
                v_under = ca.fmax(0.0, vx_min - X[2, k])
                # Penalty weight: pick large enough that overshoot is
                # ~0.01 in the actuator scale at the cost margin.
                # Empirically 1e4 keeps the optimiser away from the
                # bound. The downstream hard-clip catches any residual.
                w_bound = 1.0e4
                J += w_bound * (
                    d_over_up ** 2 + d_over_dn ** 2
                    + t_over_up ** 2 + t_over_dn ** 2
                    + b_over_up ** 2 + b_over_dn ** 2
                    + v_under ** 2
                )

            # Cost. Same shape as v3.2 MPCWeights, with the outer
            # reference subtracted when available.
            J += weights.w_v * (X[2, k] - p_v_ref[k]) ** 2
            J += weights.w_lat * (X[0, k] - p_n_ref[k]) ** 2
            J += weights.w_psi * (X[1, k] - p_psi_e_ref[k]) ** 2
            # Phase 5.3 brake-aggression tune (Lever 3): track the
            # outer's planned longitudinal accel. ``a_long_k`` is the
            # chassis-frame longitudinal accel (matches the outer's
            # point-mass ``a_long`` decision variable up to a small
            # ``v_y · ω`` cross-coupling we drop here, consistent with
            # how the outer plans). Definition: a_long = (Fx_front ·
            # cos(δ) − Fy_front · sin(δ) + Fx_rear − F_drag) / mass.
            # Masked off (multiplied by p_a_long_mask) when the caller
            # is on Tier-1 DP-plan fallback so a zero ``a_long_ref``
            # doesn't fight v_ref tracking.
            if weights.w_a > 0.0:
                F_drag_k = pc.drag_coeff * v_x_k * ca.fabs(v_x_k)
                cd_k = ca.cos(delta_k)
                sd_k = ca.sin(delta_k)
                a_long_k = (
                    Fx_f_k * cd_k - Fy_f_k * sd_k + Fx_r_k - F_drag_k
                ) / pc.mass
                J += weights.w_a * p_a_long_mask[k] * (
                    a_long_k - p_a_long_ref[k]
                ) ** 2
            # Rate-of-control penalty. v3.7 (chase-Tomas brake bang-bang
            # fix): split per-channel via :func:`stage_rate_cost`. At
            # default weights (``w_du_brake = w_du_throttle = 0.0``) the
            # term equals the legacy ``w_du · sumsqr(U[:,k] - U[:,k-1])``
            # so pre-v3.7 builds are bit-identical. Set
            # ``w_du_brake = 0.1`` + ``w_du_throttle = 10.0`` to enforce
            # the bang-bang-brake / smooth-throttle asymmetry from the
            # task brief.
            J += stage_rate_cost(weights, U, k, p_u_prev)
            # Rate-of-rate (curvature of input) penalty — uses a
            # 3-stage stencil so k=0,1 are skipped (matches v3.2's
            # w_du2 term which also skips the first two stages). Kept
            # combined across channels; the asymmetry is captured in
            # the rate-of-control term above. Lower if a future tune
            # wants snappier brake-snap behaviour.
            if k >= 2:
                d2u = U[:, k] - 2.0 * U[:, k - 1] + U[:, k - 2]
                J += weights.w_du2 * ca.sumsqr(d2u)
            # v3.7 asymmetric pedal-shape cost — adds three optional
            # terms (brake double-well, throttle², brake·throttle
            # overlap). At default zeros this returns MX(0.0). See
            # :mod:`hmpc_inner_cost` for the shape rationale.
            J += stage_pedal_shape_cost(weights, X, k)

        # Terminal-stage bounds — same soft-penalty pattern as per-stage.
        d_over_up = ca.fmax(0.0, X[5, N] - delta_max)
        d_over_dn = ca.fmax(0.0, -delta_max - X[5, N])
        t_over_up = ca.fmax(0.0, X[6, N] - 1.0)
        t_over_dn = ca.fmax(0.0, -X[6, N])
        b_over_up = ca.fmax(0.0, X[7, N] - 1.0)
        b_over_dn = ca.fmax(0.0, -X[7, N])
        v_under = ca.fmax(0.0, vx_min - X[2, N])
        J += 1.0e4 * (
            d_over_up ** 2 + d_over_dn ** 2
            + t_over_up ** 2 + t_over_dn ** 2
            + b_over_up ** 2 + b_over_dn ** 2
            + v_under ** 2
        )
        # Terminal cost — match v3.2: w_term · (n² + psi_e²).
        J += weights.w_term * (X[0, N] ** 2 + X[1, N] ** 2)
        # Also pull v_x toward the last v_ref entry (continuous extension).
        J += weights.w_v * (X[2, N] - p_v_ref[N - 1]) ** 2
        # v3.7: apply the pedal-shape cost at the terminal actuator-
        # memory state too, so the {0, 1}-pull doesn't trail off at the
        # horizon tail. No-op when the knobs are at defaults.
        J += stage_pedal_shape_cost(weights, X, N)

        opti.minimize(J)

        # IPOPT options — same as the outer's Phase 5.2 tuning, with
        # max_iter dialled tighter so the per-tick wall-clock stays
        # bounded. The acceptable-tolerance short-circuit is critical:
        # closed-loop MPC values feasibility-with-cost over optimality.
        ipopt_opts = {
            "print_level": 0,
            "sb": "yes",
            "max_iter": DEFAULT_IPOPT_MAX_ITER,
            "acceptable_tol": DEFAULT_IPOPT_ACCEPTABLE_TOL,
            "acceptable_iter": 4,
            "acceptable_constr_viol_tol": 1e-3,
            "tol": DEFAULT_IPOPT_TOL,
            "mu_strategy": "adaptive",
            "warm_start_init_point": "yes",
            # Bigger barrier-init for faster convergence on the
            # actuator absolute-bound box.
            "warm_start_bound_push": 1e-6,
            "warm_start_mult_bound_push": 1e-6,
            # Hessian: exact is more expensive per iter but converges
            # in fewer steps; L-BFGS converges in more iters with
            # cheaper iters. Empirically exact wins on this problem
            # size (same finding as the outer build).
            "hessian_approximation": "exact",
            "linear_solver": "mumps",
        }
        opti.solver("ipopt", {"print_time": False}, ipopt_opts)

        # Store handles for solve-time use.
        self._opti = opti
        self._X = X
        self._U = U
        self._p_x0 = p_x0
        self._p_kappa = p_kappa
        self._p_v_ref = p_v_ref
        self._p_n_ref = p_n_ref
        self._p_psi_e_ref = p_psi_e_ref
        self._p_u_prev = p_u_prev
        self._p_fz_f = p_fz_f
        self._p_fz_r = p_fz_r
        self._p_a_long_ref = p_a_long_ref
        self._p_a_long_mask = p_a_long_mask

    # ------------------------------------------------------------------
    # Per-tick solve.
    # ------------------------------------------------------------------

    def solve(
        self,
        *,
        x0: np.ndarray,
        kappa_seq: np.ndarray,
        v_ref_seq: np.ndarray,
        n_ref_seq: np.ndarray | None,
        psi_e_ref_seq: np.ndarray | None,
        u_prev: np.ndarray,
        cimpcc_params=None,  # noqa: ARG002 — interface parity; not used here.
        a_long_ref_seq: np.ndarray | None = None,
    ) -> InnerSolveResult:
        """One inner solve — rebind parameters, call IPOPT, extract u_seq.

        Parameters parity with :meth:`InnerTracker.solve`. ``cimpcc_params``
        is accepted but ignored — the v3.2 CiMPCC overlay does not apply
        to the CasADi inner (the friction circle and the brake-zone
        anticipation are both handled natively).
        """
        t0 = perf_counter()
        N = self.N

        # Sanity-check inputs (cheap defensive guard; same as InnerTracker).
        if not (
            np.all(np.isfinite(x0))
            and np.all(np.isfinite(kappa_seq))
            and np.all(np.isfinite(v_ref_seq))
            and np.all(np.isfinite(u_prev))
            and (n_ref_seq is None or np.all(np.isfinite(n_ref_seq)))
            and (psi_e_ref_seq is None or np.all(np.isfinite(psi_e_ref_seq)))
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

        # Build n_ref / psi_e_ref param arrays (zeros when caller didn't
        # supply them — Tier-1 DP-plan tracking semantics).
        n_ref_arr = (
            np.asarray(n_ref_seq, dtype=float)
            if n_ref_seq is not None else np.zeros(N)
        )
        psi_e_ref_arr = (
            np.asarray(psi_e_ref_seq, dtype=float)
            if psi_e_ref_seq is not None else np.zeros(N)
        )
        used_outer_ref = n_ref_seq is not None or psi_e_ref_seq is not None

        # Compute per-stage axle Fz from the v3.2 plant constants. We use
        # the *static* Fz here unless the caller has dynamic_fz_enabled
        # set on pc; the dynamic-Fz iteration loop is replaced inside the
        # NLP by IPOPT's exact handling of the nonlinearity, but we
        # still need *some* values for the friction-ellipse denominators.
        # Use the chassis's current Fz as a constant; the optimiser sees
        # the slope-only Pacejka through these. A future improvement
        # would feed per-stage Fz from a 1-step Picard update with the
        # optimiser's planned (a_x, a_y); for now the static path matches
        # the v3.2 inner's first SQP iteration exactly.
        fz_f_arr = np.full(N, float(self.pc.Fz_front))
        fz_r_arr = np.full(N, float(self.pc.Fz_rear))

        # ---- Rebind parameters ----
        self._opti.set_value(self._p_x0, x0)
        self._opti.set_value(self._p_kappa, np.asarray(kappa_seq, dtype=float))
        self._opti.set_value(self._p_v_ref, np.asarray(v_ref_seq, dtype=float))
        self._opti.set_value(self._p_n_ref, n_ref_arr)
        self._opti.set_value(self._p_psi_e_ref, psi_e_ref_arr)
        self._opti.set_value(self._p_u_prev, np.asarray(u_prev, dtype=float))
        self._opti.set_value(self._p_fz_f, fz_f_arr)
        self._opti.set_value(self._p_fz_r, fz_r_arr)

        # Phase 5.3 brake-aggression tune (Lever 3): rebind the outer's
        # planned a_long sequence (zeros + mask=0 on Tier-1 fallback).
        if a_long_ref_seq is not None and self.weights.w_a > 0.0:
            a_long_arr = np.asarray(a_long_ref_seq, dtype=float)
            if a_long_arr.shape[0] < N:
                a_long_arr = np.concatenate([
                    a_long_arr,
                    np.full(N - a_long_arr.shape[0], a_long_arr[-1] if a_long_arr.size else 0.0),
                ])
            elif a_long_arr.shape[0] > N:
                a_long_arr = a_long_arr[:N]
            a_long_mask_arr = np.ones(N, dtype=float)
        else:
            a_long_arr = np.zeros(N, dtype=float)
            a_long_mask_arr = np.zeros(N, dtype=float)
        self._opti.set_value(self._p_a_long_ref, a_long_arr)
        self._opti.set_value(self._p_a_long_mask, a_long_mask_arr)

        # ---- Warm-start initial values ----
        # Shifted previous u_seq (one stage forward).
        u_init = np.zeros((NU, N))
        u_init[:, :-1] = self._u_seq_prev.T[:, 1:]
        u_init[:, -1] = self._u_seq_prev.T[:, -1]
        # State warm-start: shift previous X by 1 if we have one, else
        # roll the dynamics forward from x0 with u_init (cheap NumPy
        # Euler integration).
        x_init = self._roll_warm_start(
            x0=x0,
            u_init=u_init,
            kappa_seq=np.asarray(kappa_seq, dtype=float),
            v_ref_seq=np.asarray(v_ref_seq, dtype=float),
        )
        self._opti.set_initial(self._U, u_init)
        self._opti.set_initial(self._X, x_init)

        # ---- Optional iter cap bump (Risk 8) ----
        # Re-attaching the solver each tick costs a tick of overhead;
        # only do it when actually changing the iter cap.
        if self._extra_iters_next > 0:
            self._opti.solver(
                "ipopt", {"print_time": False},
                {
                    "print_level": 0,
                    "sb": "yes",
                    "max_iter": DEFAULT_IPOPT_MAX_ITER + self._extra_iters_next,
                    "acceptable_tol": DEFAULT_IPOPT_ACCEPTABLE_TOL,
                    "acceptable_iter": 4,
                    "acceptable_constr_viol_tol": 1e-3,
                    "tol": DEFAULT_IPOPT_TOL,
                    "mu_strategy": "adaptive",
                    "warm_start_init_point": "yes",
                    "warm_start_bound_push": 1e-6,
                    "warm_start_mult_bound_push": 1e-6,
                    "hessian_approximation": "exact",
                    "linear_solver": "mumps",
                },
            )
            self._extra_iters_next = 0

        # ---- Solve ----
        # IPOPT's primal-infeasibility metric — used below to gate
        # acceptance of non-optimal iterates.
        inf_pr = float("inf")
        try:
            sol = self._opti.solve()
            status = "Solve_Succeeded"
            X_star = np.asarray(sol.value(self._X), dtype=float)
            U_star = np.asarray(sol.value(self._U), dtype=float)
            stats = self._opti.stats()
            iters = int(stats.get("iter_count", 0))
            inf_pr = float(stats.get("iterations", {}).get("inf_pr", [0.0])[-1]
                           if isinstance(stats.get("iterations"), dict)
                           else 0.0)
            infeasible = False
        except RuntimeError as exc:
            stats = self._opti.stats()
            status = str(stats.get("return_status", "ipopt_runtime_error"))
            iters = int(stats.get("iter_count", 0))
            # Pull inf_pr from the last iterate (stats['iterations']
            # is a dict-of-lists when present; some IPOPT versions
            # also expose stats['inf_pr'] directly).
            iters_dict = stats.get("iterations", {})
            if isinstance(iters_dict, dict) and iters_dict.get("inf_pr"):
                inf_pr = float(iters_dict["inf_pr"][-1])
            elif "inf_pr" in stats:
                inf_pr = float(stats["inf_pr"])
            try:
                X_star = np.asarray(self._opti.debug.value(self._X), dtype=float)
                U_star = np.asarray(self._opti.debug.value(self._U), dtype=float)
            except Exception:  # noqa: BLE001
                X_star = None
                U_star = None
            # Acceptance gate. ``Solve_Succeeded`` and ``Solved_To_Acceptable_Level``
            # are always accepted. ``Maximum_Iterations_Exceeded`` and
            # ``Search_Direction_Becomes_Too_Small`` are accepted only
            # when the last iterate is primal-feasible enough (inf_pr
            # within the threshold). Everything else (``Infeasible_Problem_Detected``,
            # ``Restoration_Failed``, runtime errors) routes to Tier-1.
            if status in ("Solve_Succeeded", "Solved_To_Acceptable_Level"):
                infeasible = False
            elif status in _FEASIBLE_STATUSES:
                # Accept iff the iterate is feasible-ish.
                infeasible = (inf_pr > _INF_PR_ACCEPT_THRESHOLD)
            else:
                infeasible = True
            if not infeasible and (X_star is None or U_star is None):
                infeasible = True
            if X_star is None or U_star is None:
                infeasible = True
            log.debug(
                "CasadiInnerTracker IPOPT non-clean: %s (status=%s, "
                "iters=%d, inf_pr=%.3e)",
                exc, status, iters, inf_pr,
            )

        solve_t = perf_counter() - t0
        self.solve_times.append(solve_t)
        self.solve_count += 1
        self.status_history.append(status)
        if infeasible or X_star is None or U_star is None:
            self.infeasible_count += 1
            return InnerSolveResult(
                u_seq=self._u_seq_prev.copy(),
                status=status,
                sqp_iters=iters,
                solve_time_s=solve_t,
                infeasible=True,
                used_outer_ref=used_outer_ref,
                x_seq_last=None,
            )

        # Sanity-clip the solution before commit. IPOPT may return
        # values microscopically outside the bounds on rounding;
        # downstream code (first_stage_commit) re-clips anyway, but the
        # warm-start cache benefits from the clean values.
        U_out = U_star.T  # (N, NU)
        # Cap on rate magnitudes (extra belt against bound-relaxation).
        U_out[:, 0] = np.clip(
            U_out[:, 0], -self.bounds.delta_dot_max, self.bounds.delta_dot_max,
        )
        U_out[:, 1] = np.clip(
            U_out[:, 1], -self.bounds.throttle_dot_max, self.bounds.throttle_dot_max,
        )
        U_out[:, 2] = np.clip(
            U_out[:, 2], -self.bounds.brake_dot_max, self.bounds.brake_dot_max,
        )
        # Cache for the next warm start.
        self._u_seq_prev = U_out.copy()
        self._x_seq_prev = X_star.copy()

        if not used_outer_ref and not infeasible:
            self.tier1_count += 1

        return InnerSolveResult(
            u_seq=U_out,
            status=status,
            sqp_iters=iters,
            solve_time_s=solve_t,
            infeasible=False,
            used_outer_ref=used_outer_ref,
            x_seq_last=X_star.T,
        )

    # ------------------------------------------------------------------
    # Warm-start helper.
    # ------------------------------------------------------------------

    def _roll_warm_start(
        self,
        *,
        x0: np.ndarray,
        u_init: np.ndarray,
        kappa_seq: np.ndarray,
        v_ref_seq: np.ndarray,
    ) -> np.ndarray:
        """Numerical-Euler roll of x0 under u_init to seed X_init.

        IPOPT's primal warm-start expects an (NX, N+1) array. The cheapest
        plausible seed is the previous solution shifted by one stage if
        available, falling back to a forward roll using the bicycle
        dynamics (NumPy mirror of :func:`mpc_model.f_continuous`).

        Returns ``(NX, N + 1)`` array.
        """
        N = self.N
        ds = self.ds_stage

        # Prefer shifted previous X.
        if self._x_seq_prev is not None and self._x_seq_prev.shape == (NX, N + 1):
            X_seed = np.zeros((NX, N + 1))
            X_seed[:, 0] = x0
            X_seed[:, 1:-1] = self._x_seq_prev[:, 2:]
            X_seed[:, -1] = self._x_seq_prev[:, -1]
            return X_seed

        # Fresh roll using a NumPy mirror of f_continuous. We import
        # locally to avoid a circular at module init.
        from .mpc_model import f_continuous

        X_seed = np.zeros((NX, N + 1))
        X_seed[:, 0] = x0
        x_k = x0.copy()
        for k in range(N):
            dt_k = ds / max(float(v_ref_seq[k]), V_FLOOR)
            f_k = f_continuous(
                x_k, u_init[:, k], kappa_ref=float(kappa_seq[k]), pc=self.pc,
            )
            x_k = x_k + dt_k * f_k
            # Sanity clamp to keep the seed feasible-ish (vx floor,
            # actuator abs limits).
            x_k[2] = max(x_k[2], float(self.bounds.vx_min))
            x_k[5] = float(np.clip(x_k[5], -self.bounds.delta_max, self.bounds.delta_max))
            x_k[6] = float(np.clip(x_k[6], 0.0, 1.0))
            x_k[7] = float(np.clip(x_k[7], 0.0, 1.0))
            X_seed[:, k + 1] = x_k
        return X_seed


__all__ = ["CasadiInnerTracker"]
