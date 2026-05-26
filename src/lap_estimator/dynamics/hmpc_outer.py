"""HMPC outer planner — CasADi + IPOPT nonlinear MPC (spec §23.4.6.2).

Phase 5.2 architectural pivot (2026-05-24): replaces the previous
OSQP + SQP outer loop with a true nonlinear MPC formulation built on
CasADi's `Opti` interface and solved by IPOPT.

Why CasADi + IPOPT, not the previous OSQP+SQP build:

The 5.1 outer loop linearised the friction circle into an 8-sided
polygon and the curvature-velocity bound into a half-space about a
linearisation point. That structure cannot represent the *coupled*
nonlinearity of "friction circle on (a_long, a_lat), with κ(s)·v² as
the centripetal demand that lives on the circle". The 5.1 outer was
primal-infeasible ~80 % of solves in a closed-loop lap (per the 5.1
architecture doc); the brake-anticipation gap closed only 38 m of the
needed 172 m.

The OUTER OCP structure (point-mass + friction circle, curvilinear
state, 500 m / 50-stage horizon at 1 Hz) is correct. The solver class
was the architectural ceiling. CasADi+IPOPT solves the nonlinear OCP
directly:

  - `pip install casadi` (Windows-friendly, no C compiler / acados
    build chain). IPOPT comes bundled.
  - Solve time 50-200 ms typical at 50 stages × 5 vars/stage. At 1 Hz
    outer cadence the budget is ~800 ms; comfortable.
  - acados would be faster (~5 ms) but its Windows install is
    non-trivial; deferred to v3.6.

The full nonlinear formulation:

States per stage ``k ∈ {0..N}``:
  ``n_k``    — lateral offset from racing-line tangent (m)
  ``ψ_e_k``  — heading error vs reference tangent (rad)
  ``v_k``    — chassis speed (m/s)

Controls per stage ``k ∈ {0..N-1}``:
  ``a_long_k``  — longitudinal accel (m/s²)
  ``a_lat_k``   — lateral accel (m/s²)

Spatial step ``ds`` is fixed; ``s`` is a *parameter* of stage index,
not a decision variable. Per-stage time ``dt_k = ds / max(v_k,
V_FLOOR)``.

Dynamics (per-stage, explicit Euler in ``s``):

    n_{k+1}   = n_k + v_k · sin(ψ_e_k) · dt_k
    ψ_e_{k+1} = ψ_e_k + (a_lat_k / v_k − κ(s_k) · v_k) · dt_k
    v_{k+1}   = v_k + a_long_k · dt_k

Constraints (per stage):

    a_long_k² + a_lat_k² ≤ (μ_circle · g)²        ← friction circle
    |n_k|   ≤ half_width(s_k) − safety_buffer
    v_k     ≥ V_FLOOR
    v_k     ≤ v_max_track

Cost (sum over stages):

    J = Σ_k  − w_progress · v_k · dt_k                  (progress)
           + w_v        · (v_k − v_ref_centerline(s_k))²  (speed pull)
           + w_n        · n_k²                            (centreline pull)
           + w_psi      · ψ_e_k²                          (heading pull)
           + w_du       · (Δa_long² + Δa_lat²)           (smoothness)
         + w_term · n_N²

Public surface:

- :class:`OuterPlanner` — same constructor / `solve()` signature as the
  5.1 build. :meth:`solve` consumes ``(s_0, n_0, ψ_e_0, v_0)`` in the
  shared curvilinear frame and returns a frozen
  :class:`ReferenceTrajectory`.
- :class:`OuterPlannerConfig` — same field names; OSQP-era SQP knobs
  (`sqp_max_iter`) are accepted for backwards compatibility but
  ignored. New `nlp_max_iter` controls IPOPT iterations.
- :class:`ReferenceTrajectory` — unchanged structure; `solver_status`
  reflects the IPOPT return string.

The inner controller (``hmpc_controller.py``) does NOT change — the
return type and stage-grid format are preserved bit-for-bit.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from time import perf_counter
from typing import TYPE_CHECKING

import casadi as ca
import numpy as np

from .mpcc_reference import ReferencePath, sample_seq

if TYPE_CHECKING:
    from .mpc_model import PlantConstants


log = logging.getLogger(__name__)


# Defaults — Phase 5.2 rebalance. Bumped horizon from 500 → 800 m so
# the outer at the start of a Sprint A lap (s=0, v=74) sees the
# chicane at s≈650; the prior 500 m horizon meant the cold-start solve
# was effectively braking-blind. CasADi+IPOPT solves 80 stages in
# ~400-600 ms (still well inside the 800 ms budget at 1 Hz cadence).
DEFAULT_HORIZON_M = 800.0
DEFAULT_N_STAGES = 80
DEFAULT_DS_OUTER = DEFAULT_HORIZON_M / DEFAULT_N_STAGES  # 10 m
DEFAULT_RATE_HZ = 1.0
DEFAULT_MU_FRAC = 0.85       # μ_circle = mu_frac · μ_inner_eff (Risk 1 buffer)
DEFAULT_W_PROGRESS = 1.0     # weight on Σ dt = Σ ds/v (time minimisation)
DEFAULT_W_N = 200.0          # centreline pull — VERY strong. The outer's
                             # job is to plan the v(s) brake profile; it
                             # should NOT invent a racing line (the inner
                             # tracks centerline by default and the
                             # n_ref ≠ 0 shape causes the inner to over-
                             # steer at the chicane). With w_n large the
                             # outer keeps n ≈ 0 and the n_ref output is
                             # a clean centerline reference.
DEFAULT_W_PSI = 50.0         # heading pull — strong, same rationale
DEFAULT_W_V = 5.0            # DP-plan speed pull. Strong anchor — the DP
                             # plan already respects the lap's friction
                             # physics; the outer's job is to enforce it
                             # against the chassis state, not to invent
                             # a different speed profile.
DEFAULT_W_DU = 0.05          # smoothness — small (don't fight the brake-
                             # zone rate of change; the inner has its own
                             # actuator-rate limits).
DEFAULT_W_TERM = 50.0        # terminal n² (centreline-finish bias)

# Curvature-aware speed cap margin. The friction circle is the binding
# constraint on cornering: ``v² · κ ≤ μ_circle · g``. In the curvilinear
# point-mass plant the link between v and the cornering demand goes
# through ``a_lat`` (a decision variable); when n=ψ_e=0 the dynamics
# absorb the bias into a_lat which is then circle-constrained. To make
# the constraint explicit (and prevent the solver from settling on
# implausibly high v_k at known-curving stages), we add a hard
# ``v_k² · |κ_k| ≤ KAPPA_V_CAP_FRAC · μ_circle · g`` constraint at every
# stage. The 0.85 multiplier reserves ~26 % of the friction budget for
# a_long so the planner can still brake while cornering.
KAPPA_V_CAP_FRAC = 0.85
KAPPA_CONSTRAINT_MIN = 5.0e-4    # rad/m; below this kappa we skip the cap
DEFAULT_SAFETY_BUFFER = 0.3  # m, inside the track edge
DEFAULT_A_LONG_MAX = 1.4 * 9.81  # ~13.7 m/s² — loose belt
DEFAULT_A_LAT_MAX = 1.6 * 9.81   # ~15.7 m/s²
V_FLOOR = 5.0                # m/s — denominator floor (also a hard lower bound)
V_MAX_DEFAULT = 95.0          # m/s — chassis hard ceiling

DEFAULT_NLP_MAX_ITER = 60     # IPOPT outer iterations
DEFAULT_PSI_E_CAP = float(np.deg2rad(35.0))  # |ψ_e| hard cap for stability

# v_ref look-ahead window for the inner. Tuned 2026-05-25 (Phase 5.3
# close-out): the previous left-shift had a sign-inversion bug in the
# accel-into-brake transition zone (e.g. s≈288, v_chassis < v_DP). The
# outer plans to accelerate up to v_DP for ~30 m before braking; a
# pure left-shift therefore pulled v_ref(s_chassis) ABOVE chassis speed
# — the inner read an ACCEL command at the very moment the outer wanted
# brake commit. Replaced with a rolling-MIN: ``v_ref_inner[k] =
# min(min(v_NLP, v_DP)[k : k+lookahead+1])``. In a brake zone this is
# bit-identical to the previous shift (the lookahead window is
# monotonically decreasing); in an accel zone it falls back to
# v_NLP[k] / v_DP[k] (no spurious accel signal); at the accel-then-brake
# boundary it picks the lower of the two (so the brake commit lands at
# the right s).
#
# Default is 3 (30 m, matches the previous hardcode for backwards-
# compatible brake-zone behaviour). The Phase 5.3 close-out empirical
# sweep shows L=20 (200 m) is needed for Sprint A first-corner
# completion at chicane_safety_mult >= 0.95; that value is shipped via
# ``driver.raw.control_params.hmpc.outer_vref_lookahead_stages`` or
# the ``HMPCController(outer_vref_lookahead_stages=...)`` kwarg, not
# as a default change (would alter behaviour for non-headline callers).
DEFAULT_VREF_LOOKAHEAD_STAGES = 3

G = 9.81


@dataclass(frozen=True)
class ReferenceTrajectory:
    """Outer planner output — frozen reference table consumed by the inner.

    Stored on a uniform ``s_outer`` grid (length ``N+1``). The inner
    reads via ``n_ref_at(s)``, ``v_ref_at(s)``, ``psi_e_ref_at(s)``
    (linear interpolation). The dataclass is immutable.
    """

    s_outer: np.ndarray            # (N+1,) — stage s grid (m)
    n_ref: np.ndarray              # (N+1,) — planned lateral offset (m)
    psi_e_ref: np.ndarray          # (N+1,) — planned heading error (rad)
    v_ref: np.ndarray              # (N+1,) — planned speed (m/s)
    a_long_ref: np.ndarray         # (N,)   — planned a_long (m/s², diag)
    a_lat_ref: np.ndarray          # (N,)   — planned a_lat (m/s², diag)
    s_horizon_end: float           # cached s_outer[-1]
    t_solve_ms: float              # wall-clock outer solve time
    solver_status: str             # IPOPT status string
    sqp_iters: int = 1             # legacy alias for IPOPT iteration count

    def n_ref_at(self, s: float) -> float:
        """Linear-interp ``n_ref(s)``; clipped at the grid edges."""
        return float(np.interp(
            float(np.clip(s, self.s_outer[0], self.s_outer[-1])),
            self.s_outer, self.n_ref,
        ))

    def v_ref_at(self, s: float) -> float:
        """Linear-interp ``v_ref(s)``; clipped at the grid edges."""
        return float(np.interp(
            float(np.clip(s, self.s_outer[0], self.s_outer[-1])),
            self.s_outer, self.v_ref,
        ))

    def psi_e_ref_at(self, s: float) -> float:
        """Linear-interp ``ψ_e_ref(s)``; clipped at the grid edges."""
        return float(np.interp(
            float(np.clip(s, self.s_outer[0], self.s_outer[-1])),
            self.s_outer, self.psi_e_ref,
        ))

    def a_long_ref_at(self, s: float) -> float:
        """Linear-interp ``a_long_ref(s)`` at the stage-midpoints.

        ``a_long_ref`` lives on the ``(N,)`` between-stages control grid;
        we anchor it at the stage MIDPOINTS for the interpolation, then
        clip at the edges. Outside the outer's horizon the result is
        clipped to the nearest endpoint — same convention as the other
        ``*_at`` accessors.
        """
        # Midpoints of the N stage intervals.
        s_mid = 0.5 * (self.s_outer[:-1] + self.s_outer[1:])
        return float(np.interp(
            float(np.clip(s, s_mid[0], s_mid[-1])),
            s_mid, self.a_long_ref,
        ))


@dataclass
class OuterPlannerConfig:
    """Tunables for :class:`OuterPlanner`. Spec §23.4.7.3 defaults."""

    horizon_m: float = DEFAULT_HORIZON_M
    n_stages: int = DEFAULT_N_STAGES
    mu_circle: float | None = None       # None → mu_frac · μ_inner_eff at build
    mu_frac: float = DEFAULT_MU_FRAC
    w_progress: float = DEFAULT_W_PROGRESS
    w_n: float = DEFAULT_W_N
    w_psi: float = DEFAULT_W_PSI
    w_v: float = DEFAULT_W_V
    w_du: float = DEFAULT_W_DU
    w_term: float = DEFAULT_W_TERM
    safety_buffer: float = DEFAULT_SAFETY_BUFFER
    a_long_max: float = DEFAULT_A_LONG_MAX
    a_lat_max: float = DEFAULT_A_LAT_MAX
    v_max_track: float = V_MAX_DEFAULT
    # Legacy OSQP-era knob (accepted for backwards compatibility with
    # callers that pass it through; ignored by the CasADi build).
    sqp_max_iter: int = 1
    nlp_max_iter: int = DEFAULT_NLP_MAX_ITER
    # v_ref rolling-MIN lookahead window in stages (ds = horizon_m /
    # n_stages, default 10 m). The inner reads
    # ``v_ref_inner[k] = min(v_arr[k : k+lookahead+1])`` — this rolls a
    # brake-signal forward without the sign-inversion of the legacy
    # left-shift.
    vref_lookahead_stages: int = DEFAULT_VREF_LOOKAHEAD_STAGES


class OuterPlannerError(RuntimeError):
    """Outer NLP returned infeasible / failed."""


# ---------------------------------------------------------------------------
# Helper: CasADi interpolant for κ(s) and half_width(s).
# ---------------------------------------------------------------------------


def _build_interpolants(ref: ReferencePath) -> tuple[ca.Function, ca.Function]:
    """Return CasADi interpolants ``kappa(s)`` and ``half_width(s)``.

    The :mod:`mpcc_reference` :class:`ReferencePath` holds dense arrays
    sampled on a uniform ``ds_ref`` grid; we wrap them as 1-D linear
    interpolants. CasADi's ``ca.interpolant`` with ``'linear'`` mode
    plays nicely with IPOPT (continuous derivatives are not required;
    IPOPT accepts the piecewise-linear gradients).
    """
    s_grid = np.asarray(ref.s_grid, dtype=float)
    kappa = np.asarray(ref.kappa_ref, dtype=float)
    half_w = np.asarray(ref.track_half_width, dtype=float)
    f_kappa = ca.interpolant("kappa_s", "linear", [s_grid], kappa)
    f_half_w = ca.interpolant("hw_s", "linear", [s_grid], half_w)
    return f_kappa, f_half_w


# ---------------------------------------------------------------------------
# OuterPlanner.
# ---------------------------------------------------------------------------


class OuterPlanner:
    """Long-horizon point-mass + friction-circle planner (CasADi + IPOPT).

    Constructed once per controller; ``solve(state)`` called every
    ``1/rate_hz`` seconds by :class:`HMPCController`. The planner is
    stateless beyond its warm-start cache (previous solution) and its
    diagnostic counters.
    """

    def __init__(
        self,
        ref: ReferencePath,
        plan,
        pc: "PlantConstants",
        *,
        config: OuterPlannerConfig | None = None,
    ) -> None:
        self.ref = ref
        self.plan = plan
        self.pc = pc
        cfg = config if config is not None else OuterPlannerConfig()
        self.cfg = cfg
        # μ_circle: mu_frac · min(D_lat, D_long). Min is the binding axle.
        if cfg.mu_circle is not None:
            self.mu_circle = float(cfg.mu_circle)
        else:
            d_lat = float(pc.D_lat_front)
            d_lng = float(pc.D_long)
            self.mu_circle = cfg.mu_frac * min(d_lat, d_lng)
        if not (0.3 <= self.mu_circle <= 2.0):
            log.warning(
                "OuterPlanner.mu_circle=%.3f outside [0.3, 2.0]; clipping.",
                self.mu_circle,
            )
            self.mu_circle = float(np.clip(self.mu_circle, 0.3, 2.0))
        self.ds_outer = float(cfg.horizon_m / max(cfg.n_stages, 1))
        # CasADi-compatible interpolants for the track.
        self._f_kappa, self._f_half_w = _build_interpolants(ref)
        # Diagnostics.
        self.solve_times: list[float] = []
        self.solve_count: int = 0
        self.infeas_count: int = 0
        self.status_history: list[str] = []
        # Warm-start cache: previous decision values + the previous
        # reference (so we can shift by Δs).
        self._x_prev: np.ndarray | None = None       # (N+1, 3): n, psi_e, v
        self._u_prev: np.ndarray | None = None       # (N, 2):   a_long, a_lat
        self._ref_prev: ReferenceTrajectory | None = None

    # ------------------------------------------------------------------
    # solve.
    # ------------------------------------------------------------------

    def solve(
        self,
        state_curvilinear: tuple[float, float, float, float],
    ) -> ReferenceTrajectory:
        """Solve one outer NLP.

        Parameters
        ----------
        state_curvilinear : tuple
            ``(s_0, n_0, ψ_e_0, v_0)`` — chassis projected into the
            shared curvilinear frame.

        Returns
        -------
        ReferenceTrajectory

        Raises
        ------
        OuterPlannerError
            If IPOPT fails to find any feasible point. Caller falls
            back to a stale reference or DP-plan tracking.
        """
        t0 = perf_counter()
        s0, n0, psi_e0, v0 = state_curvilinear
        N = int(self.cfg.n_stages)
        ds = float(self.ds_outer)
        s_total = float(self.ref.total_length)
        # Stage s grid. Clip the *upper* edge a touch inside the track
        # length so the interpolants don't grab the lap-end zero kappa
        # (the v3 simulator treats the lap end as a flat extension).
        s_grid = s0 + np.arange(N + 1, dtype=float) * ds
        s_grid = np.clip(s_grid, 0.0, max(s_total - 1e-3, 0.0))
        # κ + DP-plan v_ref + half-width at each stage (NumPy side, for
        # cost shaping and warm-start). The NLP itself queries the
        # CasADi interpolants symbolically so the gradient is exact.
        kappa_seq, v_dp_seq, half_w_seq = sample_seq(s_grid, self.ref)

        nlp_iter, status, x_star, u_star = self._solve_nlp(
            s_grid=s_grid,
            n0=float(n0), psi_e0=float(psi_e0), v0=float(v0),
            v_dp_seq=v_dp_seq,
            half_w_seq=half_w_seq,
            kappa_seq=kappa_seq,
        )
        t_solve = perf_counter() - t0
        self.solve_times.append(t_solve)
        self.solve_count += 1
        self.status_history.append(status)

        # IPOPT statuses we accept as "good enough to consume". The
        # `Solve_Succeeded` is the clean optimal. `Solved_To_Acceptable_Level`
        # hit tolerance relaxed but feasible. `Search_Direction_Becomes_Too_Small`
        # and `Restoration_Failed` we accept only when the iterate is
        # primal-feasible — IPOPT reports the last iterate either way.
        ok_statuses = {
            "Solve_Succeeded",
            "Solved_To_Acceptable_Level",
        }
        if status not in ok_statuses:
            # Even on a "warning" status, if we got a finite iterate
            # with no NaN, accept it; the inner has its own sanity
            # filter on the reference table. IPOPT's `stats()['return_status']`
            # is informative; we surface it in solver_status.
            if x_star is None or u_star is None:
                self.infeas_count += 1
                raise OuterPlannerError(
                    f"OuterPlanner infeasible (status={status}, "
                    f"mu_circle={self.mu_circle:.3f})"
                )
            if not (
                np.all(np.isfinite(x_star)) and np.all(np.isfinite(u_star))
            ):
                self.infeas_count += 1
                raise OuterPlannerError(
                    f"OuterPlanner non-finite iterate (status={status})"
                )
            # Tier-1 recovery: accept the suboptimal iterate but log.
            log.info(
                "OuterPlanner: IPOPT status=%s, accepting last iterate "
                "(may be suboptimal).", status,
            )

        n_arr = np.asarray(x_star[:, 0], dtype=float)
        psi_e_arr = np.asarray(x_star[:, 1], dtype=float)
        v_arr = np.asarray(x_star[:, 2], dtype=float)
        a_long_arr = np.asarray(u_star[:, 0], dtype=float)
        a_lat_arr = np.asarray(u_star[:, 1], dtype=float)

        # ---- v_ref rolling-MIN look-ahead, DP-capped ----
        # See ``DEFAULT_VREF_LOOKAHEAD_STAGES`` for the full rationale.
        # We build the inner-consumed v_ref from the *minimum* of the
        # NLP plan and the DP plan, then take a rolling minimum over
        # ``L = lookahead`` stages ahead:
        #
        #     v_cand[k]      = min(v_arr[k], v_DP[k])
        #     v_ref_inner[k] = min(v_cand[k : k+L+1])
        #
        # Two bugs the prior left-shift design exposed (Phase 5.3
        # close-out, 2026-05-25):
        #
        # 1. Sign inversion in accel-into-brake transitions. The NLP's
        #    initial-state pin ``v_var[0] = v_chassis`` plus the free
        #    +a_long_max at stage 0 makes the *unshifted* NLP plan rise
        #    toward v_DP for ~3 stages before any brake. A left-shift
        #    by L therefore pulls v_ref ABOVE chassis at the brake-entry
        #    tick — telling the inner to ACCELERATE just when the outer
        #    wanted brake commit. The rolling MIN never sees that bias.
        #
        # 2. DP-cap exposure. The NLP plan can briefly run above the DP
        #    plan in the under-speed-at-cruise case (chassis catching
        #    up to v_DP). Capping v_ref at the DP forbids the inner
        #    from being asked to exceed v_DP under any circumstance —
        #    the DP is the friction- and curvature-respecting baseline
        #    speed profile and is by construction a safe upper bound on
        #    the v we should track. (The OUTER plan is what gives the
        #    inner brake-anticipation; the DP cap is what prevents the
        #    inner from accel-tracking at the wrong moment.)
        #
        # In a pure brake zone the rolling MIN reduces to v_DP[k+L]
        # (DP is monotone-decreasing in the brake zone). In a pure
        # accel zone (chassis below v_DP on a straight) the MIN reduces
        # to v_DP[k] (DP is monotone-increasing, so the local stage is
        # the lowest). In the accel-into-brake transition the MIN picks
        # the brake-zone value as soon as it enters the L-stage window
        # — exactly the brake-commit signal the inner needs.
        lookahead = int(self.cfg.vref_lookahead_stages)
        n_v = len(v_arr)
        v_dp_arr = np.asarray(v_dp_seq, dtype=float)
        # Pad v_dp_arr if it's shorter than v_arr (defensive; sample_seq
        # returns N+1 values for an N-stage outer, so lengths match).
        if v_dp_arr.shape[0] < n_v:
            v_dp_arr = np.concatenate([
                v_dp_arr, np.full(n_v - v_dp_arr.shape[0], v_dp_arr[-1]),
            ])
        v_cand = np.minimum(v_arr, v_dp_arr[:n_v])
        v_arr_for_inner = np.empty_like(v_arr)
        if lookahead <= 0:
            v_arr_for_inner[:] = v_cand
        else:
            for k in range(n_v):
                upper = min(k + lookahead + 1, n_v)
                v_arr_for_inner[k] = float(np.min(v_cand[k:upper]))

        traj = ReferenceTrajectory(
            s_outer=s_grid,
            n_ref=n_arr,
            psi_e_ref=psi_e_arr,
            v_ref=v_arr_for_inner,
            a_long_ref=a_long_arr,
            a_lat_ref=a_lat_arr,
            s_horizon_end=float(s_grid[-1]),
            t_solve_ms=float(t_solve * 1000.0),
            solver_status=status,
            sqp_iters=int(nlp_iter),
        )
        # Cache for next warm-start.
        self._x_prev = x_star
        self._u_prev = u_star
        self._ref_prev = traj
        return traj

    # ------------------------------------------------------------------
    # NLP build.
    # ------------------------------------------------------------------

    def _solve_nlp(
        self,
        *,
        s_grid: np.ndarray,
        n0: float,
        psi_e0: float,
        v0: float,
        v_dp_seq: np.ndarray,
        half_w_seq: np.ndarray,
        kappa_seq: np.ndarray,
    ) -> tuple[int, str, np.ndarray | None, np.ndarray | None]:
        """Build and solve the outer NLP via CasADi/IPOPT.

        Returns ``(iter_count, status, X_star, U_star)`` where
        ``X_star`` is ``(N+1, 3)`` and ``U_star`` is ``(N, 2)``. The
        Opti variables are eliminated by IPOPT in primal-dual form.

        On hard failure ``(X_star, U_star)`` are ``None``.
        """
        N = int(self.cfg.n_stages)
        ds = float(self.ds_outer)
        mu_g = float(self.mu_circle * G)
        v_max = float(self.cfg.v_max_track)
        a_long_max = float(self.cfg.a_long_max)
        a_lat_max = float(self.cfg.a_lat_max)
        safety = float(self.cfg.safety_buffer)
        psi_cap = float(DEFAULT_PSI_E_CAP)

        opti = ca.Opti()
        # Decision variables.
        n_var = opti.variable(N + 1)
        psi_var = opti.variable(N + 1)
        v_var = opti.variable(N + 1)
        a_long = opti.variable(N)
        a_lat = opti.variable(N)

        # Initial-state pin.
        opti.subject_to(n_var[0] == n0)
        opti.subject_to(psi_var[0] == psi_e0)
        opti.subject_to(v_var[0] == max(float(v0), V_FLOOR))

        # Per-stage half-width values (precomputed scalars — using the
        # CasADi interpolant would also work but adds graph nodes
        # without changing the math, since s_k is a constant here).
        half_w_scalars = [
            max(float(half_w_seq[min(k, N)]) - safety, 0.5)
            for k in range(N + 1)
        ]
        # v_ref centerline for the soft pull (use the DP plan as the
        # speed target; the friction circle + curvature are what truly
        # cap v).
        v_ref_center = np.maximum(np.asarray(v_dp_seq, dtype=float), V_FLOOR)

        # Dynamics + per-stage constraints + cost.
        w_p = float(self.cfg.w_progress)
        w_n = float(self.cfg.w_n)
        w_psi = float(self.cfg.w_psi)
        w_v = float(self.cfg.w_v)
        w_du = float(self.cfg.w_du)
        w_term = float(self.cfg.w_term)

        J = 0.0
        for k in range(N):
            s_k = float(s_grid[k])
            # κ at s_k. Using the CasADi interpolant keeps the
            # gradient consistent (small change for the symbolic
            # build, but the interpolant returns a 1-D vector
            # `MX(1,1)`, so we extract the scalar).
            kappa_k = self._f_kappa(s_k)
            # Per-stage dt (nonlinear in v_k).
            v_safe = ca.fmax(v_var[k], V_FLOOR)
            dt_k = ds / v_safe
            # Dynamics (explicit Euler in s):
            #   n_{k+1}   = n_k + v_k · sin(ψ_e_k) · dt_k
            #             = n_k + sin(ψ_e_k) · ds        (since v·dt = ds)
            #   ψ_{k+1}   = ψ_k + (a_lat / v_k − κ · v_k) · dt
            #   v_{k+1}   = v_k + a_long_k · dt_k
            opti.subject_to(n_var[k + 1] == n_var[k]
                            + ca.sin(psi_var[k]) * ds)
            opti.subject_to(psi_var[k + 1] == psi_var[k]
                            + (a_lat[k] / v_safe - kappa_k * v_safe) * dt_k)
            opti.subject_to(v_var[k + 1] == v_var[k] + a_long[k] * dt_k)
            # Friction circle (the nonlinear constraint).
            opti.subject_to(a_long[k] ** 2 + a_lat[k] ** 2 <= mu_g ** 2)
            # Actuator caps — loose belt; friction circle is binding.
            opti.subject_to(opti.bounded(-a_long_max, a_long[k], a_long_max))
            opti.subject_to(opti.bounded(-a_lat_max, a_lat[k], a_lat_max))
            # Cost. Progress term is the per-stage TIME ``dt = ds/v_safe``;
            # minimising it rewards higher speed (time-minimisation
            # surrogate). The earlier formulation ``-w·v·dt = -w·ds`` was
            # constant per stage and so had zero gradient w.r.t. v —
            # nothing pushed the planner toward higher v on straights.
            J += w_p * dt_k
            J += w_v * (v_var[k] - float(v_ref_center[k])) ** 2
            J += w_n * n_var[k] ** 2
            J += w_psi * psi_var[k] ** 2
            if k > 0:
                J += w_du * ((a_long[k] - a_long[k - 1]) ** 2
                             + (a_lat[k] - a_lat[k - 1]) ** 2)
            # Track-edge + speed + heading bounds on stage k (state
            # bounds, not on the initial state which is pinned).
            if k >= 1:
                opti.subject_to(opti.bounded(
                    -half_w_scalars[k], n_var[k], half_w_scalars[k]))
                opti.subject_to(opti.bounded(V_FLOOR, v_var[k], v_max))
                opti.subject_to(opti.bounded(
                    -psi_cap, psi_var[k], psi_cap))
            # Cornering speed cap: ``v_k² · |κ_k| ≤ KAPPA_V_CAP_FRAC · μ·g``.
            # Explicit at every k (including k = 0 — the chassis initial
            # state may already be infeasible at the start of a tight
            # corner; the IPOPT solve will then reject the trajectory
            # and the controller falls through to Tier-1 / stale ref).
            # Sample κ from the NumPy seq (κ at the stage's *start*) so
            # the constraint is well-conditioned even when the
            # interpolant returns near-zero on near-straight sections.
            kappa_k_np = float(kappa_seq[k]) if k < len(kappa_seq) else 0.0
            if abs(kappa_k_np) >= KAPPA_CONSTRAINT_MIN:
                v_cap_corner = ((KAPPA_V_CAP_FRAC * mu_g)
                                / abs(kappa_k_np)) ** 0.5
                # ``v_k ≤ v_cap_corner`` is a simple linear bound.
                opti.subject_to(v_var[k] <= v_cap_corner)

        # Terminal stage cost + bounds.
        opti.subject_to(opti.bounded(
            -half_w_scalars[N], n_var[N], half_w_scalars[N]))
        opti.subject_to(opti.bounded(V_FLOOR, v_var[N], v_max))
        opti.subject_to(opti.bounded(-psi_cap, psi_var[N], psi_cap))
        kappa_N_np = float(kappa_seq[N]) if N < len(kappa_seq) else 0.0
        if abs(kappa_N_np) >= KAPPA_CONSTRAINT_MIN:
            v_cap_corner_N = ((KAPPA_V_CAP_FRAC * mu_g)
                              / abs(kappa_N_np)) ** 0.5
            opti.subject_to(v_var[N] <= v_cap_corner_N)
        J += w_term * n_var[N] ** 2
        J += w_v * (v_var[N] - float(v_ref_center[N])) ** 2

        opti.minimize(J)

        # Warm-start. Cold start: n=linear(n0→0), psi=linear(psi_e0→0),
        # v=v_dp_seq, a_long from finite-diff of v_dp, a_lat=v²κ
        # clipped to mu_g.
        x_init, u_init = self._build_warm_start(
            s_grid=s_grid, n0=n0, psi_e0=psi_e0, v0=v0,
            v_dp_seq=v_dp_seq, kappa_seq=kappa_seq, mu_g=mu_g,
            a_long_max=a_long_max, a_lat_max=a_lat_max,
        )
        opti.set_initial(n_var, x_init[:, 0])
        opti.set_initial(psi_var, x_init[:, 1])
        opti.set_initial(v_var, x_init[:, 2])
        opti.set_initial(a_long, u_init[:, 0])
        opti.set_initial(a_lat, u_init[:, 1])

        # Solver setup. Exact Hessian + adaptive μ converges 2-3× faster
        # than L-BFGS on this dense-block-structured OCP (build-time
        # finding 2026-05-24: L-BFGS oscillated for 60+ iterations on
        # the friction-circle + curvature-cap combo). Acceptable-tol
        # short-circuits let IPOPT exit on feasible-but-suboptimal
        # iterates — critical for closed-loop MPC where time budget >
        # optimality budget.
        ipopt_opts = {
            "print_level": 0,
            "sb": "yes",
            "max_iter": int(self.cfg.nlp_max_iter),
            "acceptable_tol": 1e-2,
            "acceptable_iter": 5,
            "acceptable_constr_viol_tol": 1e-3,
            "tol": 1e-4,
            "mu_strategy": "adaptive",
            "warm_start_init_point": "yes",
        }
        opti.solver("ipopt", {"print_time": False}, ipopt_opts)

        try:
            sol = opti.solve()
            status = "Solve_Succeeded"
            x_star = np.column_stack([
                np.asarray(sol.value(n_var)).reshape(-1),
                np.asarray(sol.value(psi_var)).reshape(-1),
                np.asarray(sol.value(v_var)).reshape(-1),
            ])
            u_star = np.column_stack([
                np.asarray(sol.value(a_long)).reshape(-1),
                np.asarray(sol.value(a_lat)).reshape(-1),
            ])
            iters = int(opti.stats().get("iter_count", 0))
            return iters, status, x_star, u_star
        except RuntimeError as exc:
            # IPOPT raised — typical for "Restoration_Failed" or
            # "Maximum_Iterations_Exceeded". Pull the debug iterate.
            stats = opti.stats()
            status = str(stats.get("return_status", "ipopt_runtime_error"))
            iters = int(stats.get("iter_count", 0))
            try:
                # `opti.debug.value(...)` returns the last iterate
                # even on failure.
                x_star = np.column_stack([
                    np.asarray(opti.debug.value(n_var)).reshape(-1),
                    np.asarray(opti.debug.value(psi_var)).reshape(-1),
                    np.asarray(opti.debug.value(v_var)).reshape(-1),
                ])
                u_star = np.column_stack([
                    np.asarray(opti.debug.value(a_long)).reshape(-1),
                    np.asarray(opti.debug.value(a_lat)).reshape(-1),
                ])
            except Exception:  # noqa: BLE001
                x_star = None
                u_star = None
            log.debug(
                "OuterPlanner IPOPT failure: %s (status=%s, iters=%d)",
                exc, status, iters,
            )
            return iters, status, x_star, u_star

    # ------------------------------------------------------------------
    # Warm-start.
    # ------------------------------------------------------------------

    def _build_warm_start(
        self,
        *,
        s_grid: np.ndarray,
        n0: float,
        psi_e0: float,
        v0: float,
        v_dp_seq: np.ndarray,
        kappa_seq: np.ndarray,
        mu_g: float,
        a_long_max: float,
        a_lat_max: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Cold or warm initial guess for the NLP.

        Cold start: n linearly decays from ``n0`` to 0, ψ_e from
        ``psi_e0`` to 0, v follows the DP plan, a_long from finite-diff
        of v_dp, a_lat from equilibrium centripetal ``v² · κ`` clipped
        to the friction circle.

        Warm start: shift the previous NLP solution by Δs (the chassis
        has moved), filling the new tail with cold-start values.
        """
        N = int(self.cfg.n_stages)
        ds = float(self.ds_outer)

        # --- Cold-start arrays ---
        v_cold = np.maximum(np.asarray(v_dp_seq, dtype=float), V_FLOOR)
        v_cold[0] = max(float(v0), V_FLOOR)
        n_cold = np.linspace(float(n0), 0.0, N + 1)
        psi_cold = np.linspace(float(psi_e0), 0.0, N + 1)
        if N >= 1:
            dv = np.diff(v_cold) / max(ds, 1e-3)
            a_long_cold = v_cold[:N] * dv[:N]
            a_long_cold = np.clip(a_long_cold, -a_long_max, a_long_max)
        else:
            a_long_cold = np.zeros(N)
        a_lat_eq = v_cold[:N] * v_cold[:N] * np.asarray(kappa_seq[:N], dtype=float)
        # Keep the warm-start strictly inside the friction circle so
        # IPOPT doesn't start on the boundary (boundary starts slow
        # convergence and inflate iter counts).
        scale = 0.9
        a_lat_cold = np.clip(a_lat_eq, -scale * mu_g, scale * mu_g)
        a_long_cold = np.clip(a_long_cold, -scale * mu_g, scale * mu_g)
        x_cold = np.column_stack([n_cold, psi_cold, v_cold])
        u_cold = np.column_stack([a_long_cold, a_lat_cold])

        if self._ref_prev is None or self._x_prev is None or self._u_prev is None:
            return x_cold, u_cold

        # --- Warm: interpolate previous solution onto the new s_grid ---
        prev = self._ref_prev
        prev_s = prev.s_outer
        # State at stage starts (length N+1).
        n_warm = np.interp(s_grid, prev_s, prev.n_ref)
        psi_warm = np.interp(s_grid, prev_s, prev.psi_e_ref)
        v_warm = np.interp(s_grid, prev_s, prev.v_ref)
        # Control values are between stages; interpolate at stage
        # midpoints.
        s_prev_mid = 0.5 * (prev_s[:-1] + prev_s[1:])
        s_new_mid = 0.5 * (s_grid[:-1] + s_grid[1:])
        a_long_warm = np.interp(s_new_mid, s_prev_mid, prev.a_long_ref)
        a_lat_warm = np.interp(s_new_mid, s_prev_mid, prev.a_lat_ref)
        # Where the new grid extends past the previous horizon, fall
        # back to cold values.
        s_end_prev = float(prev_s[-1])
        beyond_state = s_grid > s_end_prev
        beyond_ctrl = s_new_mid > s_end_prev
        if np.any(beyond_state):
            n_warm = np.where(beyond_state, n_cold, n_warm)
            psi_warm = np.where(beyond_state, psi_cold, psi_warm)
            v_warm = np.where(beyond_state, v_cold, v_warm)
        if np.any(beyond_ctrl):
            a_long_warm = np.where(beyond_ctrl, a_long_cold, a_long_warm)
            a_lat_warm = np.where(beyond_ctrl, a_lat_cold, a_lat_warm)
        # Pin the first state to the actual chassis state — IPOPT
        # constraints will enforce this hard, but a consistent initial
        # guess keeps the residual small at iteration 0.
        n_warm[0] = float(n0)
        psi_warm[0] = float(psi_e0)
        v_warm[0] = max(float(v0), V_FLOOR)
        v_warm = np.maximum(v_warm, V_FLOOR)
        x_warm = np.column_stack([n_warm, psi_warm, v_warm])
        u_warm = np.column_stack([a_long_warm, a_lat_warm])
        return x_warm, u_warm


__all__ = [
    "DEFAULT_HORIZON_M",
    "DEFAULT_N_STAGES",
    "DEFAULT_DS_OUTER",
    "DEFAULT_RATE_HZ",
    "DEFAULT_MU_FRAC",
    "OuterPlanner",
    "OuterPlannerConfig",
    "OuterPlannerError",
    "ReferenceTrajectory",
]
