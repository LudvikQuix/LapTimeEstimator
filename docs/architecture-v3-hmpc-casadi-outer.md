# Architecture — Phase 5.2 / v3.4 HMPC Outer: CasADi + IPOPT Nonlinear MPC

**Spec:** `dev-planning/lap-simulation-csv-driver/spec-section-23-4-v34-hierarchical-mpc.md` + 2026-05-24 task brief (this pivot).
**Predecessor (in-tree):** `docs/architecture-slip-model-phase5_1-v34-hierarchical-mpc.md`.
**Branch:** `feature/sc-71955/lap-simulation`.
**Status:** Outer solve is now feasible and produces correct anticipatory plans. Closed-loop completion gates **not met** — bottleneck moves to the inner tracker (the brief explicitly bars touching it).
**Cross-references:**
- `docs/architecture-slip-model-phase5_1-v34-hierarchical-mpc.md` — the OSQP+SQP outer this replaces.
- `docs/architecture-v3-session-2026-05-24.md` — session index covering the reactive baseline and v3 controller history.

---

## TL;DR

The Phase 5.1 outer loop used OSQP+SQP with a polygonal friction-circle approximation. In closed loop, **80 % of outer solves returned primal-infeasible**, so the brake-anticipation reference reached the inner only intermittently and the brake-commit gap vs the reactive baseline closed only 38 m of the needed 172 m.

Phase 5.2 replaces the OSQP+SQP solver with a **true nonlinear MPC built on CasADi + IPOPT**. Same OCP structure (point-mass + friction circle, curvilinear `(n, ψ_e, v)` state with `(a_long, a_lat)` control, 1 Hz cadence), but the friction circle is now a native nonlinear constraint solved by IPOPT directly. The public surface of `OuterPlanner` (constructor, `solve()` signature, `ReferenceTrajectory` dataclass) is unchanged, so the `HMPCController` is bit-for-bit untouched.

**Result vs Phase 5.1 (Sprint A, Tomas, BMW 1M, `--inertia-zz 2400 --chicane-safety-mult 0.85`):**

| Metric | Phase 5.1 (OSQP+SQP) | Phase 5.2 (CasADi+IPOPT) | Reactive baseline |
|---|---|---|---|
| Outer primal-infeasibility | 80 % (33 / 41 solves) | **0 % (0 / 11 solves)** | n/a |
| Outer solve mean | 6.7 ms | **310 ms** | n/a |
| Outer solve p99 | 28.2 ms | 451 ms | n/a |
| Brake first > 0.5 commit | s ≈ 391 m | **s ≈ 299 m** | s ≈ 257 m |
| Brake first > 0.9 commit | s ≈ 623 m | s ≈ 490 m | trail-brake (no 0.9 lockup) |
| MC completions (10 seeds) | 0 / 10 | 0 / 10 | 10 / 10 (2:09 median) |

The architectural pivot **succeeded on its stated mission**: every outer solve is feasible and the brake-anticipation reference flows to the inner without dropouts. The brake-commit `s = 299 m` is within 42 m of the reactive baseline `s = 257 m` (vs 134 m off with Phase 5.1).

Closed-loop completion remains 0/10 because the inner consumes ~75 % of the available friction-circle decel during the lead-in brake zone — leaving ~25 % grip on the table — and the chassis arrives at the chicane apex too fast to make the corner. The outer's planned `a_long ≈ −8.6 m/s²` (max friction-circle decel) is verified to be physically achievable; the inner just does not extract it. **This is an inner-tracker tuning problem the brief explicitly excluded from scope** ("Do NOT touch the inner OCP").

---

## File inventory

**Replaced (single file rewrite):**

| File | Lines | Role |
|---|---|---|
| `src/lap_estimator/dynamics/hmpc_outer.py` | ~620 (vs 769 before) | New CasADi `Opti` build + IPOPT solve. Same module-level public API: `OuterPlanner`, `OuterPlannerConfig`, `OuterPlannerError`, `ReferenceTrajectory`. `OuterPlannerConfig.sqp_max_iter` is preserved for caller backwards-compat but ignored; new `nlp_max_iter` controls IPOPT. |

**Edited:**

| File | Change |
|---|---|
| `requirements.txt` | Added `casadi>=3.6,<4.0`. CasADi ships IPOPT as a bundled wheel — no C compiler, no CMake, no acados build chain. Windows-friendly. |

**Untouched (regression-clean by file inventory):**

- `src/lap_estimator/dynamics/hmpc_controller.py`, `hmpc_inner.py`, `hmpc_pi_trim.py`, `hmpc_debug.py` — the hierarchical controller wiring is identical.
- `mpc_qp.py`, `mpc_qp_ellipse.py`, `mpc_model.py`, `mpc_controller_tiers.py` — v3.2 MPC unaffected.
- `mpcc_*.py` — v3.3 MPCC unaffected.
- `driver_controller.py`, `solver.py`, `longitudinal_planner.py`, `viz/*`, `web/*`.

**Smoke / acceptance scripts (`.tmp/`, scratch only):**

- `.tmp/hmpc_outer_smoke.py` — pre-existing outer-only acceptance harness; works against the new build with no edits.
- `.tmp/hmpc_casadi_closed_loop_smoke.py` — new closed-loop single-seed + 10-MC smoke; reads HMPC debug CSV to extract brake-commit s.
- `.tmp/hmpc_casadi_diag.csv` — per-tick debug trace (controller's existing `hmpc_debug_trace_path` plumbing; format unchanged).

---

## NLP formulation

The OCP follows the task brief's prescribed structure. Curvilinear point-mass plant:

```
States per stage k ∈ {0..N}:
  n_k       — lateral offset from racing-line tangent (m)
  ψ_e_k     — heading error vs path tangent (rad)
  v_k       — chassis speed (m/s)

Controls per stage k ∈ {0..N-1}:
  a_long_k  — longitudinal accel (m/s²)
  a_lat_k   — lateral accel (m/s²)

Spatial step ds = horizon_m / N_stages. Per-stage time:
  dt_k = ds / max(v_k, V_FLOOR)

Dynamics (explicit Euler in s):
  n_{k+1}   = n_k + sin(ψ_e_k) · ds                          (v·dt = ds)
  ψ_{k+1}   = ψ_k + (a_lat_k / v_k − κ(s_k) · v_k) · dt_k
  v_{k+1}   = v_k + a_long_k · dt_k

Constraints (per stage):
  a_long_k² + a_lat_k² ≤ (μ_circle · g)²    ← FRICTION CIRCLE (nonlinear)
  v_k² · |κ(s_k)| ≤ 0.85 · μ_circle · g     ← cornering-speed cap (linear in v²)
  |n_k| ≤ half_width(s_k) − safety_buffer
  V_FLOOR ≤ v_k ≤ v_max_track
  |ψ_e_k| ≤ 35°
  |a_long_k| ≤ 13.7,  |a_lat_k| ≤ 15.7      ← loose actuator belt

Cost:
  J = Σ_k  w_progress · dt_k                       (time-minimisation)
         + w_v        · (v_k − v_DP_centerline(s_k))²
         + w_n        · n_k²
         + w_psi      · ψ_e_k²
         + w_du       · (Δa_long² + Δa_lat²)
       + w_term · n_N²
```

Initial state `(n_0, ψ_e_0, v_0)` is pinned by an equality constraint to the chassis projection.

### Defaults (Phase 5.2)

| Knob | Default | Rationale |
|---|---|---|
| `horizon_m` | 800 m | The chicane on Sprint A sits at s≈650 m; the prior 500 m horizon meant the cold-start solve at s=0 was braking-blind. 800 m brings every chassis state inside the same "see the chicane" window. CasADi+IPOPT solves 80 stages in ~250 ms (warm start) — well inside the 800 ms budget at 1 Hz. |
| `n_stages` | 80 | Keeps `ds_outer = 10 m`, matching the Phase 5.1 spec amendment. |
| `mu_circle` | `0.85 · min(D_lat_front, D_long)` | Per Risk-1 spec default. With the calibrated Tomas/BMW values that's μ ≈ 0.88. |
| `w_progress` | 1.0 | Per-stage `dt_k` magnitude is ~0.1-0.5 s; `w_progress · Σ dt` is ~10-25 over a 50-stage horizon. |
| `w_n` | 200.0 | Strong centreline pull. With weaker `w_n` the planner uses lateral excursion (n=+5 to +7 m through the chicane) to dodge curvature — geometrically reasonable as a racing line but the inner doesn't follow this and the chassis ends up off-track. The outer's job is now to plan **the brake profile**; the inner tracks centreline. |
| `w_psi` | 50.0 | Mirror of `w_n` — keep heading aligned with the path. |
| `w_v` | 5.0 | DP-plan speed pull. The DP plan already respects the lap's friction physics; the outer's job is to enforce it against the chassis state. |
| `w_du` | 0.05 | Small — don't fight brake-zone rate of change; the inner has its own actuator-rate limits. |
| `w_term` | 50.0 | Terminal n² for centreline-finish bias. |
| `KAPPA_V_CAP_FRAC` | 0.85 | Cornering-speed cap reserves ~26 % of friction budget for `a_long`. |
| IPOPT `max_iter` | 60 | Empirically sufficient with the exact Hessian. |
| IPOPT `hessian_approximation` | exact (IPOPT default) | L-BFGS oscillated for 60+ iters on this problem; exact Hessian converges in 7-26 iters typically. |
| IPOPT `acceptable_tol` | 1e-2 | Lets IPOPT short-circuit on a feasible-but-suboptimal iterate. |
| IPOPT `mu_strategy` | adaptive | Robust to the bang-bang nature of the optimal brake profile. |

### Build-time decisions (the empirical retune)

The architectural pivot exposed three failure modes that the original spec defaults did not anticipate:

1. **Progress reward shape.** The Phase 5.1 spec wrote progress as `−w_progress · v_k / v_lin²` (a linearised time-min surrogate). When ported to nonlinear, the `w_progress · v · dt = w_progress · ds` formulation has *zero gradient w.r.t. v* — it's a constant per stage. The corrected form is `+w_progress · dt = +w_progress · ds / v` (a time term to be minimised). With this fix the planner actually trades brake against time.

2. **Centreline anchoring needs to be strong.** With moderate `w_n`, the outer plans a racing-line-style lateral excursion (n = +5 to +7 m through the chicane) — geometrically right for a free-driver, but the inner tracks centerline and chassis drifts off-track. `w_n = 200` is enough to pin the outer's `n_ref ≈ 0` so the inner's centerline-tracking is bit-compatible with the outer's lateral plan.

3. **The curvature-velocity bound must be explicit.** The friction circle alone is satisfied at low `a_lat`; nothing in the dynamics forces the planner to slow into corners unless the planner also realises that `v² · κ` is the centripetal demand it has to pay for. The Phase 5.1 build linearised this; Phase 5.2 enforces it as a hard linear-in-v² stage constraint with a 0.85 friction-budget reservation for `a_long`.

### v_ref look-ahead shift

`v_ref` returned to the inner is shifted left by 3 stages (30 m at `ds=10`). Rationale: the NLP's initial-state pin `v_var[0] == v_0` forces `v_ref[0] = v_chassis` by construction. The inner's `w_v · (v − v_ref)²` cost therefore reads ZERO at the chassis's current `s` at the moment of the outer fire — even when the outer plan says "brake hard from here." A 30 m look-ahead means the inner at chassis `s` reads the v-target it should be at after 30 m of travel; in the brake zone that's ~5 m/s below current, which is a substantial v-error and pushes the inner to commit hard immediately. **Only v_ref is shifted; n_ref and ψ_e_ref stay at the chassis's current `s`** — lateral tracking needs the present-position value, not a future one.

Empirical tuning of the shift:

| Shift | Brake first > 0.5 commit | Tier-2 episodes/lap | Note |
|---|---|---|---|
| 0 stages | s ≈ 335 m | ~30 | baseline (initial pin) |
| 2 stages (20 m) | s ≈ 315 m | ~30 | marginal gain |
| **3 stages (30 m)** | **s ≈ 299 m** | ~50 | shipped value |
| 5 stages (50 m) | s ≈ 276 m | ~120 | inner over-commits, instability |

---

## Solver settings — why exact Hessian, not L-BFGS

The task brief suggested `hessian_approximation: limited-memory`. Empirically, on this OCP (friction circle + curvature cap + dense block-tridiagonal state propagation), L-BFGS oscillated for 60+ iterations without converging — IPOPT exited with `Maximum_Iterations_Exceeded` even at `max_iter=100`. The constraint residual hung at 10⁻⁴ to 10⁻³ and the dual infeasibility refused to drop below 0.05.

Switching to the IPOPT default exact Hessian converged in 7-26 iterations across all test states. The exact-Hessian solve time at 80 stages is 180-260 ms (warm start) and 260 ms (cold start), well inside the 800 ms budget at 1 Hz. The brief's projected solve time was 50-200 ms; we land at the upper end.

The trade-off: exact Hessian needs CasADi to build symbolic second derivatives of the cost + constraints. That cost is paid once at first solve (the symbolic graph is constructed inside `_solve_nlp` per call, since a fresh `Opti` is built — see "Caveats" below); subsequent solves reuse the AD graph cheaply. For warm starts the actual IPOPT iteration cost is ~10-30 ms per iter at this problem size.

---

## Information flow (unchanged from Phase 5.1)

```
Track CSV + DP plan + chassis state
   │
   ▼
HMPCController._resolve()      every inner tick (50 Hz)
   │
   ├──► Outer fires?           every 1 s (or 30 m of horizon-end margin)
   │      │
   │      ▼
   │   OuterPlanner.solve()    ── NEW: CasADi Opti() + IPOPT
   │      │
   │      └─► ReferenceTrajectory  (s_outer, n_ref, ψ_e_ref, v_ref, …)
   │           │
   │           │   (v_ref has 30 m look-ahead shift; n_ref/ψ_e_ref unshifted)
   │           ▼
   │      cached as self._reference
   │
   ├──► InnerTracker.solve(    (unchanged)
   │      x0=chassis state,
   │      kappa_seq, v_ref_seq=outer.v_ref interpolated,
   │      n_ref_seq=outer.n_ref interpolated,
   │      psi_e_ref_seq=outer.psi_e_ref interpolated,
   │   )
   │
   ├──► first_stage_commit() → (steer, throttle, brake)
   │
   └──► PI trim (bounded) → Controls
```

The information flow is unchanged. Only the box "OuterPlanner.solve()" changed contents; its inputs and outputs are bit-identical to Phase 5.1.

---

## Caveats and follow-ups

1. **`Opti()` is rebuilt per solve.** The CasADi Opti object is rebuilt fresh inside `_solve_nlp` on every call. This costs ~50 ms in symbolic graph construction. A future optimisation is to pre-build the Opti once at constructor time and re-bind `(n0, psi_e0, v0)` and the per-stage `(κ_k, half_w_k, v_ref_center_k)` as `opti.parameter()` objects — that should drop solve time to ~150 ms steady-state. Deferred because the current ~300 ms mean fits comfortably in the budget.

2. **Inner-tracker friction-circle headroom.** The outer plans `a_long ≈ −8.6 m/s²` (max friction-circle decel) on the lead-in brake zone, but the actual closed-loop chassis decel is ~6.3 m/s² — leaving ~25 % friction headroom unused. This is an inner-tracker tuning issue (likely `w_v` or friction-ellipse safety margin). The task brief excludes inner-loop changes; this is the next architectural opportunity.

3. **`OuterPlannerConfig.sqp_max_iter` ignored.** Preserved as a no-op for caller backwards-compat. The new `nlp_max_iter` (default 60) controls IPOPT.

4. **`hessian_approximation`** is left at IPOPT's default (exact). The build-time finding contradicts the task brief's L-BFGS suggestion; documented above.

5. **File length** is 760 lines, above the 500-line soft cap. The natural seams are `OuterPlanner` (orchestration), `_solve_nlp` (NLP build), `_build_warm_start`. The `_solve_nlp` method is itself 130+ lines and is the bulk of the file. Splitting it into a `hmpc_outer_nlp.py` helper module is a reasonable refactor but requires moving the `_build_interpolants` helper too; deferred.

6. **`acados` is not used.** The brief explicitly defers it to v3.6 because Windows install is non-trivial. CasADi+IPOPT gets us to a working nonlinear MPC without that build chain.

---

## Acceptance summary

Per the task-brief gates:

| Gate | Target | Result |
|---|---|---|
| Outer-only smoke: feasible at chicane entry? | yes | **YES** (`status=Solve_Succeeded` from chassis states at s=0/100/200/300/500/600) |
| Single MC seed end-to-end: lap completes? | yes | NO (abort at chicane apex; brake-anticipation reaches the inner but inner under-commits decel) |
| ≥1/10 MC completion | minimum | **NOT MET** (0/10) |
| Stretch: ≥7/10 at chicane mult 0.85 | stretch | not met |
| Stretch²: lap ≤ 2:00 | stretch² | not met |
| Outer solve time mean < 500 ms | gate | **MET** (310 ms mean, 451 ms p99) |

The architectural pivot succeeded on the gate it set out to prove ("does CasADi+IPOPT solve the nonlinear outer reliably?"): every outer solve is feasible and produces a sensible plan. The closed-loop gate is now bottlenecked by the inner tracker, which the brief excluded from scope.

---

## What this unblocks

The Phase 5.1 architecture doc captured the conclusion that "controller is the binding constraint, v3.2 (MPC) is spec'd answer." Phase 5.2 raises the outer architectural ceiling — every outer plan is now physically tight to the friction circle and the curvature-speed bound. The remaining gap is **inner-tracker friction-circle exploitation**, which is the next architectural lever.

Concrete next steps (out of scope for this pivot):

1. Inner `w_v` sweep — currently 10.0; the chassis v-tracking is slow.
2. Inner friction-ellipse safety margin — currently the inner reserves grip; check whether it can be tightened.
3. Async outer fire — currently the outer blocks the inner tick when it fires (310 ms median solve). Fire the outer in a background thread and let the inner continue using the stale reference until the new one lands.
4. `acados` migration (deferred to v3.6) — would cut outer solve time to ~5-10 ms, enabling 5-10 Hz cadence and tighter coupling.

---

## Outer aggression tuning (Phase 5.3 close-out, 2026-05-25)

**Context.** Phase 5.3 shipped the CasADi nonlinear inner; standalone smoke verified `util_p85 = 0.844` (within friction envelope, no spin). But closed-loop completion remained 0/10: chassis arrived at the chicane apex too fast, with chassis decel ≈ 4 m/s² instead of the friction-circle-limited 8.6 m/s². The brief diagnosed this as a v_ref softening between the standalone outer (which planned sustained -8.61 m/s²) and the closed-loop outer.

**Root cause: sign inversion in the v_ref look-ahead shift.**

The Phase 5.2 outer ships v_ref to the inner with a hardcoded left-shift by 3 stages (`lookahead = 3`, 30 m at `ds = 10 m`). The rationale: the NLP's initial-state pin `v_var[0] = v_chassis` makes `v_ref[0] = v_chassis` by construction, so the inner sees zero v-error at the moment of the outer fire. A left-shift pulls future v values into the present so the inner reads the brake-zone target instead of the cruise target.

That rationale is correct **only in a monotone-decreasing v plan**. At the brake-entry point on Sprint A (s ≈ 260-320 m, v_chassis ≈ 65-71 m/s, v_DP ≈ 74), the NLP plan has a +a_long_max lunge for ~3 stages because:

  - The NLP cost contains `w_v · (v_k − v_DP(s_k))²` (default `w_v = 5`). Chassis below v_DP creates a downward pull toward v_DP.
  - The NLP cost has a time-minimisation term `w_progress · dt_k` rewarding higher v.
  - The friction circle permits `a_long ≤ +μg ≈ +8.61 m/s²` at stage 0.

So the *unshifted* NLP plan rises from v=70 → v=73 over the first 3 stages, then commits -8.61 m/s² sustained for the brake zone. A left-shift by 3 stages puts the lunge's *peak* at the chassis's current `s` — the inner therefore reads `v_ref = 73 > v_chassis = 70` at the brake-entry tick, i.e. an **accel** signal at the moment the outer wanted brake commit.

The inner dutifully tracked this signal and the brake committed late and soft (s ≈ 288, 0.55-0.70 sustained), giving the closed-loop chassis decel of 4-5 m/s² instead of the 8.6 m/s² the outer planned.

**Fix: rolling-MIN look-ahead, DP-capped.**

Replaced the left-shift with a rolling minimum over a configurable look-ahead window, and capped the per-stage v at the DP plan's `v_max(s)`:

```python
v_cand[k]      = min(v_NLP[k], v_DP[k])
v_ref_inner[k] = min(v_cand[k : k + L + 1])
```

Why this works:

  - In a pure brake zone, the candidate sequence is monotone-decreasing, so the rolling MIN equals `v_cand[k + L]` — bit-identical to the previous left-shift behaviour we wanted.
  - In a pure accel zone, the candidate sequence is monotone-increasing, so the rolling MIN equals `v_cand[k]` — the inner reads the local target (no spurious accel signal).
  - At the accel-into-brake transition, the rolling MIN picks the brake-zone value as soon as it enters the L-stage window — exactly the brake-anticipation signal the inner needs, without the lunge-toward-v_DP that caused the sign inversion.
  - The DP cap (`min(v_NLP, v_DP)`) is a belt-and-suspenders guarantee that no v_ref ever exceeds the friction-and-curvature-respecting v_max(s) baseline. The outer can plan above DP transiently (e.g. when chassis catches up to v_DP from below); the cap forbids the inner from being asked to exceed DP.

New tunable on `OuterPlannerConfig`:

| Knob | Default | Range tested | Effect |
|---|---|---|---|
| `vref_lookahead_stages` | 3 | 3, 6, 8, 10, 12, 15, 20 | Inner brake-anticipation horizon (in stages × `ds_outer = 10 m`). L=3 is the conservative default (preserves the spirit of the old shift); L=20 is the empirical sweet spot for Sprint A's chicane (60 m brake-anticipation window). |

Threaded through:

  - `OuterPlannerConfig.vref_lookahead_stages` — the dataclass field.
  - `HMPCController(outer_vref_lookahead_stages=...)` — the controller kwarg.
  - `driver.raw.control_params.hmpc.outer_vref_lookahead_stages` — driver-JSON override.

### Empirical sweep (Tomas / Sprint A / BMW 1M / `--inertia-zz 2400 --hmpc-inner-solver casadi`)

Single-seed completion gate (s_brake = first brake commit > 0.5; chassis state at chicane apex s ≈ 620 m measured from debug trace):

| L (stages) | chicane_mult | s_brake (m) | apex chassis v (m/s) | finished |
|---|---|---|---|---|
| 3 | 0.85 | n/a (Tier-2 storm) | abort s=637 | no |
| 8 | 0.85 | 240 | abort s=632 (apex v=37) | no |
| 12 | 0.85 | 201 | abort s=640 (apex v=33) | no |
| 15 | 0.85 | 171 | abort s=651 (apex v=25) | no |
| 15 | 0.95 | 171 | abort s=644 | no |
| 20 | 0.85 | 115 | clears chicane → aborts at s=2848 (next corner) | no |
| 20 | 0.80 | 115 | clears chicane → aborts at s=2836 (next corner) | no |
| **20** | **0.95** | **115** | **clears all corners** | **YES (148.5 s)** |

The lookahead-shift bug fix alone unlocks first-corner completion at L ≥ 20. The s=2848 abort at cm=0.85 has the same architectural shape as the chicane abort: a downstream slow corner (s ≈ 2950, v_DP_apex ≈ 13.4 m/s) that the inner under-brakes into. The cm=0.95 setting raises v_DP_apex to 14.9 m/s, which is enough headroom for the inner's soft-brake regime to survive.

### 10-MC completion gate (the headline acceptance gate)

| L | chicane_mult | completions | best lap | median lap | wall |
|---|---|---|---|---|---|
| 20 | 0.80 | 0/10 (all abort s=2836 — same downstream-corner mode) | n/a | n/a | 16 min |
| 20 | 0.85 | 0/10 (all abort s=2848) | n/a | n/a | 21 min |
| **20** | **0.95** | **10/10** | **148.50 s** | **148.50 s** | **25 min** |

**The Phase 5.3 headline gate (≥ 1/10 completion at the brief's `--chicane-safety-mult 0.85`) is NOT met — at cm=0.85 the inner's brake-conservatism caps the second corner.** The Phase 5.3 gate ON THE NEXT-LOOSER chicane multiplier (`--chicane-safety-mult 0.95`) is **MET 10/10**, with a 148.5 s lap (2:28.5). The lap is ~17 s slower than the reactive baseline (2:09 median) because the inner's soft-brake regime sacrifices ~3 m/s² of decel across every brake zone.

Three observations worth surfacing:

  1. **The MC laps are identical (148.50 s × 10).** With `consistency_sigma > 0`, the MC perturbs the inner's slip target by ±5 %, which is the only stochastic input. The HMPC stack converges to the same lap regardless — implying the slip-target jitter doesn't change the *outer's* plan (which dominates) and the inner is structurally tracking, not exploring. Whether this is a robustness virtue or a tuning ceiling is the next architectural question.

  2. **The brake-commit s shifted forward by 134 m vs Phase 5.2 baseline** (`s = 115` vs the Phase 5.2 doc's `s ≈ 299` for the first-brake-> 0.5 commit). The lookahead-shift fix is the dominant lever.

  3. **The DP-cap (`min(v_NLP, v_DP)`) is not load-bearing for this acceptance gate.** A separate ablation with `v_cand[k] = v_NLP[k]` (no DP cap) and rolling-MIN still completes at L=20 cm=0.95; the DP cap is documented as a defence-in-depth measure for transient over-DP NLP plans (which we observe at the brake-zone entry) but the headline outcome is set by the rolling-MIN.

### What's left for future work

The closed-loop completion at cm=0.95 confirms the architectural fix is correct. The remaining gap (5-6 m/s² closed-loop decel vs 8.6 m/s² planned) is **the inner-tracker's brake-commit conservatism**, not the outer's reference signal. The brief explicitly excluded inner changes from scope; documented inner levers for the next pass:

  - `w_v` bump on the inner cost (currently 10.0) — would raise the v-tracking weight against the slip/du costs that bias the inner toward soft brake.
  - `w_slip` tighten/relax on the inner (currently 200.0) — high `w_slip` penalises the slip-angle the chassis develops during hard brake-zone deceleration, biasing the inner toward easing off the pedal.
  - Inner ellipse safety margin — currently the inner uses the full `D_long · Fz` for longitudinal force; tightening this would force the inner to "save" more grip but won't help here since the chassis is already leaving 30% of the friction budget unused.
  - PI-trim pedal-bound bump — `pi_bound_pedal_abs` (default 0.10) caps the PI's contribution to ±10 % pedal authority; bumping to 0.40 was tested but did not change the outcome because the PI sums onto the inner's emit (already 0.6) and clips at 1.0.

The most architecturally clean fix (deferred): give the inner an `a_long_ref` cost term so it tracks the outer's planned a_long directly. The outer already exports `a_long_ref` (see `ReferenceTrajectory.a_long_ref`) but neither inner consumes it. That would close the loop without re-tuning weights.

---

## File inventory (Phase 5.3 close-out delta)

| File | Change |
|---|---|
| `src/lap_estimator/dynamics/hmpc_outer.py` | Added `DEFAULT_VREF_LOOKAHEAD_STAGES = 3` module constant + `OuterPlannerConfig.vref_lookahead_stages` field. Replaced left-shift with rolling-MIN over `min(v_NLP, v_DP)`. Removed the legacy "shift by 3 stages" hardcode and its comment block. |
| `src/lap_estimator/dynamics/hmpc_controller.py` | New kwargs `outer_w_v`, `outer_vref_lookahead_stages` on the constructor; both default to `None` (use `OuterPlannerConfig` default). Driver-JSON override resolves the same keys under `control_params.hmpc`. |

No changes to `hmpc_inner.py`, `hmpc_inner_casadi.py`, `mpcc_*.py`, `viz/`, or any v3.2 file (per brief constraint).
