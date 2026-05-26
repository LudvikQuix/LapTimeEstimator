# Spec §23.4 — v3.4 — Hierarchical (Two-Layer) MPC

**Parent spec:** `dev-planning/lap-simulation-csv-driver/spec-section-23-slip-model.md` (§23)
**Predecessor specs (v3.2 / v3.3 MPC line; remain in-tree, NOT replaced):**
- `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc.md` (§23.2 — tracking-MPC baseline)
- `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc-phase5_0_4.md` (dynamic per-axle Fz in plant)
- `dev-planning/lap-simulation-csv-driver/spec-section-23-3-v33-mpcc.md` (§23.3 — flat MPCC)
**Architecture (post-implementation, ArchDev to author):**
`docs/architecture-slip-model-phase5_1-v34-hierarchical-mpc.md`
**Status:** Draft (additive controller — does **not** retire `--controller mpc` or `--controller mpcc`)
**Project:** LapTimeEstimator
**Branch:** `feature/sc-71955/lap-simulation`
**Created:** 2026-05-24
**Planned with:** Buddy

---

## §23.4.1 Summary

Phase 5.0.8 closed the controller-only MPC line at **0/10 MC completions** on Sprint A. The architectural root cause is now fully characterised and independent of QP weights, ellipse linearisation, dynamic Fz, and Tier-1 logic:

> The v3.2 MPC has a **30 m / 15-stage / 50 Hz** lookahead = 0.43 s of forward visibility at 70 m/s. On Sprint A the chicane needs brake commitment from `s ≈ 257 m` to apex at `s ≈ 657 m` — a 400 m anticipation distance. The MPC cannot see the brake zone until the chassis reaches `s ≈ 429 m`. It commits the brake ≈ 170 m late, accumulates a ~24 m/s speed gap into the apex, and cannot recover within friction-ellipse limits. No QP-weight tune can fix this; the optimiser is acting on incomplete information.

Flat MPCC (§23.3) does not structurally fix this either: it shares the same horizon budget. Extending the horizon inside the existing OCP collides with the LTV bicycle's linearisation tax (Risk 4 of §23.3.11): a 200–300 m horizon at fine `ds` and full 10-state dynamics blows the per-tick solve budget by 5–10×.

**Hierarchical MPC** (canonical reference: arXiv 2003.04882 — Liniger; TUMFTM IAC stack — arXiv 2205.15979) splits the problem into two cooperating OCPs:

- **Outer loop** — long horizon (150–250 m), coarse stage step (≈ 5 m), simple point-mass + friction-circle plant, low update rate (1–5 Hz). Output: a reference trajectory `(s, v_ref(s), n_ref(s))` over the outer horizon. The outer loop sees the chicane brake zone well before the chassis arrives there.
- **Inner loop** — short horizon (20–50 m), fine stage step (≈ 2 m), full v3.2 LTV bicycle + dynamic Fz + friction ellipse, high update rate (50 Hz). Tracks the outer's reference. Reuses the existing `mpc_qp.py` / `mpc_qp_ellipse.py` / `mpc_model.py` infrastructure essentially unchanged.
- **Trimming PI** on top of the inner MPC — bounded correction on cross-track and `v_x` error, capped at ±10–15 % of inner output. Stanford DDL Pikes Peak pattern (feedforward + PI trim).

The architectural promise: the **outer plant is cheap enough** (point-mass, 3-state) that 50 coarse stages solve in 50–200 ms; the **inner stays inside the 30 ms p95 budget** it already meets; the **information flow** from outer → inner is a tabulated reference, not a tightly-coupled multi-rate solver. The brake-anticipation gap closes because the outer "knows" about the chicane two corners in advance and feeds the inner a `v_ref(s)` that already drops to corner-entry speed.

If hierarchical works, the lap-time gap to Tomas (currently +21.5 s at reactive 7/10 = 2:09.12 vs. 1:47.56) closes structurally. If it does not, the MPC family on the current physics is exhausted and reactive at 2:09 ships as v3 production.

This spec ships hierarchical MPC as **`--controller hmpc`**, alongside the existing `--controller mpc` and `--controller mpcc`. The reactive path stays the production default.

---

## §23.4.2 Goals

1. Ship `--controller hmpc` as a sibling of `--controller mpc` and `--controller mpcc`, dispatchable from `slip_simulator._make_controller`.
2. Reach **≥ 7/10 MC completions on Sprint A at chicane_safety_mult ≤ 0.85** (parity with reactive v3 baseline; no regression vs. §11.55 family).
3. **Lap-time gate:** mean best lap **≤ 2:09** (parity with reactive). Anything below 2:09 demonstrates the structural anticipation gain.
4. **Stretch lap-time gate:** mean best lap **≤ 2:00** (≈ 9 s gain over reactive; matches §23.3.9 stretch). If hit, this is the new shipping path.
5. **Stretch² gate:** ≤ 1:55 (within ~7.5 s of Tomas; the publishable outcome).
6. Solve-time gates per tick:
   - **Outer**: mean < 100 ms; p99 < 200 ms (1–5 Hz cadence, so any of these tick budgets are fine for the inner's 20 ms cadence).
   - **Inner**: mean < 30 ms; p99 < 50 ms (unchanged from §11.55-5.0.4-G).
7. Preserve `--controller mpc` and `--controller mpcc` unchanged; hierarchical is purely additive.
8. Reuse `mpc_qp.solve_sqp`, `mpc_qp_ellipse.build_ellipse_rows`, `mpc_model.f_continuous`, `mpc_model.compute_dynamic_fz_per_stage` for the inner loop. No edits to those files; only new callers.

## §23.4.3 Non-goals

- **Replace** the reactive controller as production. Reactive at 2:09 ships first; hierarchical MPC is a structural-improvement attempt on top of that.
- **Replace OSQP with acados.** OSQP first for both layers. acados is the §23.4.13 escalation path if the inner can't carry its tighter horizon at 50 Hz.
- **Replace flat MPCC.** §23.3 stays in-tree and runnable. Hierarchical is not built on top of MPCC; the inner uses the v3.2 LTV-bicycle tracking-MPC formulation (lower risk, more reuse).
- **Slip-aware longitudinal MPC channel.** Same deferral as §23.3.3. Throttle/brake commits stay with the embedded reactive sub-controller via the `_long_sub` pattern in `mpc_controller.py:531`. The inner MPC plans pedals for cost-shape reasons but commits only steering.
- **Tomas-trajectory injection as outer reference.** The outer's reference seed comes from the DP plan (`longitudinal_planner.LongitudinalPlan`) and the track centreline. Trajectory injection is a separate spec; the reference-source slot in the outer is plumbed for it.
- **LMPC, MPPI, tube-MPC robustness, learning-based residuals.** All separate research directions.
- **Multi-corner outer warm-start from `global_racetrajectory_optimization` (TUMFTM).** Documented as a future direction in §23.4.13(c); not in this spec.
- **Pacejka refit, web UI changes, new tracks.** Out of scope.

## §23.4.4 User stories / scenarios

1. **As a sim user**, I run
   ```
   python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json \
       --model slip --controller hmpc --single-lap --inertia-zz 2400 --mc 10
   ```
   and get a stint summary CSV with ≥ 7/10 completions, mean best lap ≤ 2:09, and a per-tick solve-time histogram for both layers in the diagnostics block.

2. **As a regression check**, I run the same invocation with `--controller reactive`, `--controller mpc`, `--controller mpcc`, `--controller hmpc` on the same track + driver and the printed solve-time / tier-share / completion stats compare side-by-side without code edits.

3. **As ArchDev**, I can tune the outer with `--hmpc-outer-horizon-m 200 --hmpc-outer-n-stages 40 --hmpc-outer-rate-hz 2.0` and the inner with `--hmpc-inner-horizon-m 30 --hmpc-inner-n-stages 15 --hmpc-inner-tick-hz 50.0`. No code edit required.

4. **As a diagnostic step**, when the run ends, the simulator prints:
   - Outer solve mean / p99 / count.
   - Inner solve mean / p99 / count.
   - PI trim contribution p95 (steering δ, throttle, brake).
   - Outer–inner reference staleness p95 (ticks since last fresh outer solution).
   - Outer-vs-inner `v_ref` disagreement p95 (m/s).

5. **As an A/B comparison**, I can pass `--hmpc-debug-trace` and get two CSVs at:
   - `.tmp/hmpc_outer_trace_<track>_<driver>.csv` — per outer solve, the reference trajectory it produced.
   - `.tmp/hmpc_inner_trace_<track>_<driver>.csv` — per inner tick, the inner state + reference + PI trim + final commit.

6. **As a failure-mode probe**, I can pass `--hmpc-outer-disable` and the controller falls back to a flat inner-only MPC tracking the DP plan directly (same as `--controller mpc`). Lets me isolate "is the outer adding value" without changing controllers.

## §23.4.5 Proposed design

**Two new modules + one new controller class.** The architectural shape:

```
HMPCController              (src/lap_estimator/dynamics/hmpc_controller.py)
├── OuterPlanner            (src/lap_estimator/dynamics/hmpc_outer.py)
│   └── point-mass + friction-circle OCP (5 m × 40 stages → 200 m)
├── InnerTracker            (src/lap_estimator/dynamics/hmpc_inner.py)
│   └── re-uses mpc_qp.solve_sqp + mpc_qp_ellipse + mpc_model
└── PITrim                  (inline in hmpc_controller.py)
    └── bounded correction on cross-track + v_x error
```

**Information flow (contract in §23.4.7.1):**

```
Track + DP plan + chassis state
        │
        ▼
   OuterPlanner.solve()      (fires every Δt_outer = 200–1000 ms)
        │
        ▼
   ReferenceTrajectory       (frozen-reference table, see §23.4.7.2)
        │
        ▼
   InnerTracker.solve()      (fires every Δt_inner = 20 ms)
        │
        ▼
   u_inner = (δ_dot, throttle_dot, brake_dot)
        │
        ▼
   PITrim.correct()          (computes δ-trim, throttle-trim, brake-trim)
        │
        ▼
   u_final = u_inner + clip(K_p · e + K_i · ∫e dt, ±bound)
        │
        ▼
   Chassis commit            (δ, throttle, brake) at tick rate
```

**Outer plant — point-mass + friction circle.** State `(s, n, ψ_e, v)`. Control `(a_long, a_lat)` directly on the friction circle (`a_long² + a_lat² ≤ (μ·g)²`). No bicycle dynamics, no slip, no per-axle Fz, no actuator rate limits. Stage step `ds_outer = 5 m`, `N_outer = 40` stages → 200 m horizon. Stage time `ts_outer_k = ds_outer / max(v_k, V_FLOOR)`. The outer solves a sequence of `(a_long, a_lat)` along the path and emits `(s, n_ref(s), ψ_ref(s), v_ref(s))` at the fine resampled grid the inner expects.

> **AMENDMENT 2026-05-24 (per user request):** Default outer should be **500 m / 50 stages / `ds_outer = 10 m`** (not 200 / 40 / 5). Rationale: at 70 m/s on Sprint A, 500 m = 7 s of lookahead — well past the chicane brake-anticipation gap (170 m / 2.4 s). Coarser stage step (10 m vs 5 m) keeps the OSQP problem cheap (5 vars × 50 stages = 250 vars). User's exact phrasing: "longer period, less resolution. It can has 500m." Outer cadence default also drops to **1.0 Hz** (slower justified by longer reference window). CLI flags `--hmpc-outer-horizon-m`, `--hmpc-outer-n-stages`, `--hmpc-outer-rate-hz` retain tuning role; default values change.

**Why point-mass for the outer.** Three reasons:
1. **Cheap to solve.** ≈ 5 decision variables per stage × 40 stages = 200 vars. OSQP eats this in 10–50 ms even at the 200 m horizon, leaving headroom for 1–5 Hz cadence.
2. **Information theory.** The outer's job is "what trajectory closes the brake-anticipation gap?" — a longitudinal-coupling decision. The slip / per-axle / yaw-inertia details that the inner handles do not change the brake-zone-start decision in any meaningful way (the friction circle is the binding constraint, and a circle is what the outer uses).
3. **Clean recompute.** Coarser plant → fewer linearisation-staleness risks → outer can re-solve at sparse cadence without worrying about LTV drift.

**Inner plant — full v3.2 LTV bicycle + dynamic Fz + friction ellipse.** State, control, indices unchanged from `mpc_model.py`. Re-uses `mpc_qp.solve_sqp` byte-for-byte. The only difference vs. `--controller mpc`: the **reference profile** `(v_ref_k, n_ref_k, ψ_ref_k)` consumed in the cost function is read from the outer's `ReferenceTrajectory` table, not from the DP plan + centreline. Stage step `ds_inner = 2 m`, `N_inner = 15` stages → 30 m horizon. Inner tick rate 50 Hz (= `Δt_inner = 20 ms`).

**Coordinate frame.** Both layers operate in curvilinear `(s, n, ψ_e)` along the track centreline (same projection helper as §23.3.6.1's `mpcc_reference.to_curvilinear`, factored out so both `mpcc_controller` and `hmpc_controller` can call it). Rationale: the outer's reference output and the inner's reference consumption must be in the same frame, and curvilinear `(s, n)` is the natural frame for "lookahead distance along the track". The inner's chassis dynamics still live in chassis frame internally; the `(s, n)` projection happens at the cost-function layer.

**Recompute cadence.** Outer fires at `f_outer = 2.0 Hz` (default; configurable 1–5 Hz). Inner fires at `f_inner = 50.0 Hz`. Between outer solves, the inner consumes the **last-produced outer reference** (frozen-reference policy; see §23.4.7.4 staleness handling).

**Trimming PI.** Bounded correction layered on the inner's commit:

```
e_n      = n_actual − n_ref(s_actual)                  # cross-track error
e_vx     = v_x_actual − v_ref(s_actual)                # longitudinal speed error
δ_trim   = clip(K_p_n · e_n + K_i_n · ∫e_n dt, ±0.15·δ_max)
thr_trim = clip(−K_p_vx · e_vx − K_i_vx · ∫e_vx dt, ±0.10)   # negative: brake harder if v_x too high
brk_trim =  clip(+K_p_vx · e_vx + K_i_vx · ∫e_vx dt, ±0.10)   # symmetric

u_final = u_inner_mpc + (δ_trim, thr_trim, brk_trim)
u_final = clip_to_actuator_limits(u_final)
```

The PI's contribution is hard-capped at ±15 % of `δ_max` for steering and ±10 % absolute on throttle/brake. The cap exists so that the PI **cannot dominate** the inner MPC — if the inner is committed to a trajectory, the PI nudges it; it does not override it. The integrator has anti-windup (reset when `|e| < ε_deadband`, clamp the integral when the trim is at the bound).

**Stale-outer handling (frozen-reference policy).** If the outer's last solve was `N_stale` ticks ago, the inner reads the **frozen reference** `ReferenceTrajectory` as if it had just been produced. No extrapolation. Reason: the reference is in `(s, n_ref(s), v_ref(s))` — a *function of s*, not of time — so as long as the chassis is somewhere along the s-range the outer planned over, the reference is still valid. The freshness check is "did the chassis run off the end of the outer's horizon?" not "how old is the outer solution in wall-clock time?"

Staleness bound: `s_chassis ≤ s_outer_horizon_end − 30 m` (one inner horizon's worth of safety margin). If the chassis is within 30 m of the outer's horizon end, the controller forces an outer re-solve **immediately** even if the cadence timer hasn't fired. If the outer can't complete in time (rare; only on first lap before the first outer solve completes), the inner falls back to tracking the DP plan directly — i.e., behaves as `--controller mpc`.

**Failure fallback.** If the outer's reference is **infeasible for the inner** — defined as: the inner's QP returns infeasible after 3 SQP iterations on a reference-tracking cost — the inner falls back to **tracking the DP plan from `longitudinal_planner.py`** as its reference. This is the same reference `--controller mpc` already uses, and it is known-runnable. The fallback is per-tick; the next outer solve (≤ 500 ms away) may produce a feasible reference and the inner re-engages. Per-tick fallback events are counted and reported in diagnostics.

If the inner **and** the DP-plan fallback both fail (back-to-back infeasibles) the controller falls through to **Tier-2 reactive** (same hysteresis policy as v3.2's `mpc_controller_tiers`). This is the final safety belt.

**Why this shape over alternatives.** Briefly here; §23.4.13 has full reasoning.
- Flat MPCC with longer horizon (60–100 m): collides with the LTV linearisation tax.
- Flat MPC with longer horizon: same problem.
- Tomas-trajectory injection: capped at Tomas's line; doesn't generalise; separate spec recommended.
- acados switch (same cost shape, faster solver): solves faster, still can't see beyond its horizon.
- LMPC: needs 10–30 laps to bootstrap; deferred.

---

## §23.4.6 Sub-features / work breakdown

### §23.4.6.1 Reference-path + curvilinear projection (shared)

**What:** Factor out the curvilinear-projection helper currently planned inside `mpcc_reference.py` (§23.3.6.1) into a **shared** module so both `--controller mpcc` and `--controller hmpc` consume the same code path.

Proposed: keep `mpcc_reference.py` and import from it. The helper signatures stay as in §23.3.7.8. If §23.3 has not landed yet at the time hierarchical-MPC implementation starts, ArchDev creates the helper inside a new `src/lap_estimator/dynamics/curvilinear.py` and §23.3 imports from there; coordinate at branch-merge time.

**Touchpoints:** `mpcc_reference.py` or `curvilinear.py`; no edit to `track.py` / `longitudinal_planner.py`.

**Dependencies:** none.

**Suggested owner:** ArchDev.

### §23.4.6.2 Outer planner — point-mass + friction-circle OCP

**What:** New module `src/lap_estimator/dynamics/hmpc_outer.py`. One class `OuterPlanner` with:

- `__init__(track, plan, calib, params, *, horizon_m=200.0, n_stages=40, mu_circle=None, ...)`.
- `solve(state_curvilinear: tuple[float, float, float, float]) -> ReferenceTrajectory` — given `(s_0, n_0, ψ_e_0, v_0)`, solves the OCP and returns the tabulated reference.
- `solve_times: list[float]` — per-call wall-clock.

**OCP cost (concrete, not abstract):**

Per stage `k = 0 … N_outer − 1`, plus terminal:

```
J_outer = sum_k [
            − w_progress_o · v_k · ts_outer_k                # reward distance covered
            + w_n_o        · n_k²                            # stay near centreline
            + w_du_o       · ((a_long_k − a_long_{k-1})² + (a_lat_k − a_lat_{k-1})²)
        ]
        + w_term_o · n_N²
```

where `ts_outer_k = ds_outer / max(v_k, V_FLOOR)`.

The **progress reward** structure is borrowed from MPCC's progress-reward shape (§23.3.6.3) but the outer rewards *physical distance covered per stage time* directly; there is no virtual θ — the outer is pure point-mass + position, so `s_k` itself is the progress variable and `v_k · ts_outer_k = ds_outer` is conserved (so equivalently the cost rewards `1/v_k`-minimisation = time-minimisation per stage). ArchDev resolves the equivalent algebraic form (minimise `Σ ts_outer_k = Σ ds_outer/v_k` ⇔ maximise `Σ v_k` weighted by `ds_outer²/(2·v_k³)` for the QP linearisation; or simpler: minimise terminal time `t_N = Σ ts_outer_k` directly).

**OCP constraints:**

- **Dynamics (point-mass in curvilinear frame):**
  ```
  s_{k+1}    = s_k + ds_outer
  n_{k+1}    = n_k + ds_outer · tan(ψ_e_k)              # small-angle: n_k + ds_outer · ψ_e_k
  ψ_e_{k+1}  = ψ_e_k + ds_outer · (a_lat_k / v_k² − κ_ref(s_k))
  v_{k+1}    = sqrt( v_k² + 2 · a_long_k · ds_outer )    # work-energy; or v_k + a_long_k · ts_outer_k
  ```
  Linearised once per outer solve about a roll-out from the previous outer solution (warm-start). Same explicit-Euler / midpoint-rule choice as v3.2 inner; ArchDev picks at build time.

- **Friction circle:**
  ```
  a_long_k² + a_lat_k² ≤ (μ_circle · g)²
  ```
  with `μ_circle` from `calib.mu_long_eff` or the DP plan's effective friction. Linearised as a per-stage tangent half-space, identical algebra to `mpc_qp_ellipse.build_ellipse_rows` but with axles collapsed (one constraint per stage instead of four). New helper `hmpc_outer._friction_circle_row(a_long_ref_k, a_lat_ref_k, mu_eff_k)` — local to this module.

- **Track edges:** `|n_k| ≤ track_half_width(s_k) − safety_buffer`. Same source as §23.3.6.3.

- **Actuator caps (outer-level):** `|a_long_k| ≤ a_long_max` (default 1.4 g), `|a_lat_k| ≤ a_lat_max` (default 1.6 g). These are loose enough that the friction circle is the binding constraint; they exist as a numerical safety belt only.

- **Velocity bounds:** `V_FLOOR ≤ v_k ≤ v_max_track` (default `v_max_track = 95 m/s`; from `plan.speeds.max() · 1.2`).

**OCP output — `ReferenceTrajectory`:**

```python
@dataclass(frozen=True)
class ReferenceTrajectory:
    s_outer:    np.ndarray   # (N_outer + 1,) — stage s grid
    n_ref:      np.ndarray   # (N_outer + 1,) — planned lateral offset
    psi_e_ref:  np.ndarray   # (N_outer + 1,)
    v_ref:      np.ndarray   # (N_outer + 1,) — planned speed
    a_long_ref: np.ndarray   # (N_outer,)     — planned longitudinal accel (for diag)
    a_lat_ref:  np.ndarray   # (N_outer,)     — planned lateral accel (for diag)
    s_horizon_end: float     # = s_outer[-1]; cached
    t_solve_ms:    float     # wall-clock solve time
    solver_status: str       # OSQP status string
```

The inner consumes this via interpolation on `s_outer`; helpers `n_ref_at(s)`, `v_ref_at(s)`, `psi_e_ref_at(s)` are methods on the dataclass (linear interp, no caching; called O(N_inner) times per inner solve = cheap).

**Solver:** OSQP via the same `solve_sqp` infrastructure. SQP outer iterations: 2 (one fewer than v3.2 inner; the outer plant is simpler so it converges faster). If 2 iterations are insufficient at build time, bump to 3.

**Warm start:** the previous outer solution shifted forward by `f_outer · Δt_since_last`. First-lap cold-start: use the DP plan's `(s, v)` as `v_ref`, `n_ref = 0`, `ψ_e_ref = 0`, `a_long_k` from finite-differencing `v_k`, `a_lat_k = 0`.

**Touchpoints:** new file. Imports from `mpc_qp` (sparse-matrix builder utilities only, no SQP loop reuse needed; the outer has its own simpler SQP loop), from `longitudinal_planner` (DP plan for warm-start), from `mpcc_reference` (curvilinear projection).

**Dependencies:** §23.4.6.1.

**Suggested owner:** ArchDev.

### §23.4.6.3 Inner tracker — full bicycle MPC against outer reference

**What:** New module `src/lap_estimator/dynamics/hmpc_inner.py`. One class `InnerTracker` with:

- `__init__(track, plan, calib, params, car, *, horizon_m=30.0, n_stages=15, tick_hz=50.0, ...)`.
- `solve(state, reference: ReferenceTrajectory) -> InnerSolveResult` — given the chassis state + the latest outer reference, returns the planned inner trajectory + the commit `(δ_dot, throttle_dot, brake_dot)`.
- Forwards `solve_times`, `tier_counts` properties for diagnostics-surface compatibility with `slip_simulator`.

**The inner is the v3.2 tracking-MPC with one change:** the reference profile in the cost is read from `reference.v_ref_at(s_k)`, `reference.n_ref_at(s_k)`, `reference.psi_e_ref_at(s_k)` instead of from `plan.speeds[idx_of(s_k)]` and the track centreline. Everything else — the LTV bicycle linearisation, the ellipse constraint, the dynamic Fz, the SQP outer-loop count, the OSQP backend, the actuator caps, the slip-target consumption, the Tier-1/Tier-2 fallback — is reused via `mpc_qp.solve_sqp` and `mpc_model`.

**Cost — concrete:**

The v3.2 cost (in `mpc_qp.solve_sqp` / `MPCWeights`) is:

```
J_inner_v32 = Σ_k [ w_v · (v_x_k − v_target_k)²        # speed tracking (against DP plan)
                  + w_n · n_k²                          # cross-track (against centreline)
                  + w_psi_e · ψ_e_k²                    # heading error
                  + w_du · ‖u_k − u_{k-1}‖²
                  + w_du2 · ‖u_k − 2u_{k-1} + u_{k-2}‖²
                ]
            + w_term · (n_N² + ψ_e_N²)
```

The hierarchical inner replaces `v_target_k` with `reference.v_ref_at(s_k)`, `n_k` is measured against `reference.n_ref_at(s_k)` (not centreline), and `ψ_e_k` is measured against `reference.psi_e_ref_at(s_k)` (not centreline tangent). Algebraically:

```
J_inner_hmpc = Σ_k [ w_v       · (v_x_k − v_ref_outer(s_k))²
                   + w_n       · (n_k − n_ref_outer(s_k))²
                   + w_psi_e   · (ψ_e_k − ψ_e_ref_outer(s_k))²
                   + w_du      · ‖u_k − u_{k-1}‖²
                   + w_du2     · ‖u_k − 2u_{k-1} + u_{k-2}‖²
                 ]
              + w_term · ((n_N − n_ref_outer(s_N))² + (ψ_e_N − ψ_e_ref_outer(s_N))²)
```

This is a small refactor inside `mpc_qp.solve_sqp` — currently the reference profile is passed as `v_target: np.ndarray`. Extend the signature to accept optional `n_ref: np.ndarray | None` and `psi_e_ref: np.ndarray | None`; when `None`, defaults to zeros (centreline tracking — v3.2 behaviour, no regression).

**Constraints — unchanged:**

- Friction ellipse: `mpc_qp_ellipse.build_ellipse_rows`.
- Dynamic Fz: `mpc_model.compute_dynamic_fz_per_stage`.
- Actuator caps, slip-target, α-soft slack: as in v3.2.

**Solver — unchanged:** OSQP via `solve_sqp` at 3 SQP outer iterations.

**Touchpoints:**
- New file `hmpc_inner.py` containing `InnerTracker` (thin wrapper around `solve_sqp`).
- **One edit** to `mpc_qp.solve_sqp` signature: optional `n_ref` / `psi_e_ref` kwargs (default `None`, behaves as v3.2). Marginal; the cost terms `(n_k − n_ref_k)²` collapse to `n_k²` when `n_ref` is `None`. ArchDev's call whether to add the kwargs to `solve_sqp` directly or introduce a thin `solve_sqp_with_lateral_ref` wrapper.
- No edit to `mpc_qp_ellipse.py`, `mpc_model.py`, or `mpc_controller_tiers.py`.

**Dependencies:** §23.4.6.2.

**Suggested owner:** ArchDev.

### §23.4.6.4 Hierarchical controller class + dispatch + PI trim

**What:** New `HMPCController` in `src/lap_estimator/dynamics/hmpc_controller.py`. Composes `OuterPlanner`, `InnerTracker`, the PI trim, the fallback ladder, and the staleness logic. Surface identical to `MPCController` / `MPCCController` so `_make_controller` adds one branch:

```python
if controller == "hmpc":
    return HMPCController(
        driver, track, car,
        plan=plan, calib=calib, params=params,
        rng_seed=rng_seed,
        outer_horizon_m=hmpc_outer_horizon_m,
        outer_n_stages=hmpc_outer_n_stages,
        outer_rate_hz=hmpc_outer_rate_hz,
        inner_horizon_m=hmpc_inner_horizon_m,
        inner_n_stages=hmpc_inner_n_stages,
        inner_tick_hz=hmpc_inner_tick_hz,
        pi_k_p_n=hmpc_pi_kp_n,
        pi_k_i_n=hmpc_pi_ki_n,
        pi_k_p_vx=hmpc_pi_kp_vx,
        pi_k_i_vx=hmpc_pi_ki_vx,
        pi_trim_bound_steer_frac=hmpc_pi_bound_steer,
        pi_trim_bound_pedal_abs=hmpc_pi_bound_pedal,
        outer_disable=hmpc_outer_disable,
        force_static_fz=hmpc_force_static_fz,
    )
```

**Internal control loop, per tick:**

```python
def controls(self, state, t, track=None) -> Controls:
    # 1. Project chassis to curvilinear.
    s, n, psi_e = self._project(state)

    # 2. Outer cadence + staleness check.
    if self._outer_should_fire(t, s):
        try:
            self._reference = self._outer.solve((s, n, psi_e, state.v_x))
            self._outer_solve_count += 1
        except OuterSolverError as exc:
            # Keep stale reference; log; if no prior reference exists, mark fallback.
            self._reference_stale_ticks += 1
            if self._reference is None:
                return self._fallback_to_dp_plan_inner(state, t, track)

    # 3. Inner solve against current reference.
    try:
        inner_result = self._inner.solve(state, self._reference)
        u_inner = inner_result.commit
        self._inner_consecutive_infeas = 0
    except QPInfeasibleError:
        self._inner_consecutive_infeas += 1
        if self._inner_consecutive_infeas >= 2:
            return self._fallback_to_reactive(state, t, track)
        return self._fallback_to_dp_plan_inner(state, t, track)

    # 4. PI trim.
    u_trimmed = self._pi_trim.apply(u_inner, state, self._reference, s)

    # 5. Pedal channel from _long_sub (matches v3.2 _long_sub pattern).
    pedal_commit = self._long_sub.controls(state, t, track)
    u_final = Controls(
        steer=u_trimmed.steer,
        throttle=pedal_commit.throttle + u_trimmed.throttle_trim,
        brake=pedal_commit.brake + u_trimmed.brake_trim,
    )
    u_final = clip_to_actuator_limits(u_final)
    return u_final
```

**PI trim — concrete:**

```python
@dataclass
class PITrim:
    K_p_n: float            # default 0.05  [rad steer / m cross-track]
    K_i_n: float            # default 0.01  [rad steer / m·s]
    K_p_vx: float           # default 0.02  [throttle frac / (m/s)]
    K_i_vx: float           # default 0.005 [throttle frac / (m/s)·s]
    bound_steer_frac: float # default 0.15
    bound_pedal_abs: float  # default 0.10
    _i_n: float = 0.0
    _i_vx: float = 0.0

    def apply(self, u_inner, state, ref, s_actual):
        e_n   = state.n - ref.n_ref_at(s_actual)
        e_vx  = state.v_x - ref.v_ref_at(s_actual)
        # Anti-windup integration.
        dt = 1.0 / self._tick_hz
        self._i_n  += e_n  * dt
        self._i_vx += e_vx * dt
        steer_trim  = clip(self.K_p_n  * e_n  + self.K_i_n  * self._i_n,
                           -self.bound_steer_frac * delta_max,
                           +self.bound_steer_frac * delta_max)
        # Sign: higher v_x than ref → more brake, less throttle.
        thr_trim    = clip(-self.K_p_vx * e_vx - self.K_i_vx * self._i_vx,
                           -self.bound_pedal_abs, +self.bound_pedal_abs)
        brk_trim    = clip(+self.K_p_vx * e_vx + self.K_i_vx * self._i_vx,
                           -self.bound_pedal_abs, +self.bound_pedal_abs)
        # If steer_trim hit the bound, freeze integrator (anti-windup).
        if abs(steer_trim) >= self.bound_steer_frac * delta_max - 1e-6:
            self._i_n -= e_n * dt
        if abs(thr_trim) >= self.bound_pedal_abs - 1e-6 \
                or abs(brk_trim) >= self.bound_pedal_abs - 1e-6:
            self._i_vx -= e_vx * dt
        return TrimCommit(steer=u_inner.steer + steer_trim,
                          throttle_trim=thr_trim,
                          brake_trim=brk_trim)
```

**Fallback ladder:**

| Tier | Trigger | Behaviour |
|---|---|---|
| 0 (clean HMPC) | Outer fresh + inner solved + PI within bounds | Use `u_final` from above |
| 1 (DP-plan inner) | Inner infeasible against outer reference (single tick) OR no outer solution yet | Inner re-solves with `reference = DP_plan_as_reference()`; same wrapper as `--controller mpc`; PI trim still applies but with DP plan as `v_ref` / `n_ref = 0` |
| 2 (reactive) | Inner infeasible 2 consecutive ticks against both outer ref AND DP plan ref | Fall through to embedded `DriverController.controls` (the `_long_sub`'s parent reactive instance); hysteresis-out: 5 consecutive clean Tier-0 ticks before re-engaging hierarchical |

Tier counts surfaced as `hmpc_tier_counts: dict[int, int]`.

**Touchpoints:**
- New `hmpc_controller.py`.
- `src/lap_estimator/dynamics/slip_simulator.py:_make_controller` — add `"hmpc"` branch (current line ≈ 471).
- `lap.py` — add `"hmpc"` to `--controller choices` (current line ≈ 122); add the CLI flags listed in §23.4.6.7.

**Dependencies:** §23.4.6.2, §23.4.6.3.

**Suggested owner:** ArchDev.

### §23.4.6.5 Diagnostics surface

**What:** Extend `SlipSimResult` with hierarchical-specific fields:

```python
hmpc_outer_solve_times_s: list[float]   = field(default_factory=list)
hmpc_inner_solve_times_s: list[float]   = field(default_factory=list)
hmpc_tier_counts:         dict[int, int] = field(default_factory=dict)
hmpc_outer_staleness_ticks_p95: float   = 0.0
hmpc_pi_trim_steer_p95: float           = 0.0
hmpc_pi_trim_throttle_p95: float        = 0.0
hmpc_pi_trim_brake_p95: float           = 0.0
hmpc_outer_vs_inner_vref_p95: float     = 0.0
hmpc_outer_solve_count: int             = 0
hmpc_inner_solve_count: int             = 0
```

Existing v3.2 `mpc_*` and v3.3 `mpcc_*` fields stay; an HMPC run leaves them at default. The `slip_simulator` consumer `slip_simulator.py:345-348` reads via duck-typed `ctrl.solve_times` / `ctrl.tier_counts`; HMPC exposes those as **the inner's** stats (so the existing dashboard works). The HMPC-specific properties above are read via `isinstance(ctrl, HMPCController)` guard, same pattern as §23.3.6.7.

**Debug trace CSVs** at:
- `.tmp/hmpc_outer_trace_<track>_<driver>.csv` — one row per outer solve. Columns: `(t_wall, t_sim, s_at_solve, v_at_solve, t_solve_ms, status, n_ref[0..N_outer], v_ref[0..N_outer], a_long_ref[0..N_outer-1])`.
- `.tmp/hmpc_inner_trace_<track>_<driver>.csv` — one row per inner tick. Columns: `(t, s, n, psi_e, v_x, v_y, omega, delta, throttle, brake, n_ref_outer_at_s, v_ref_outer_at_s, e_n, e_vx, steer_trim, thr_trim, brk_trim, tier, inner_solve_ms, qp_status, outer_staleness_ticks)`.

Gated on `--hmpc-debug-trace`. CSV writer lives in `src/lap_estimator/dynamics/hmpc_debug.py` (analogous to MPC's debug trace pattern from Phase 5.0.3).

**Touchpoints:** `slip_simulator.py` (extend `SlipSimResult`); new `hmpc_debug.py`.

**Dependencies:** §23.4.6.4.

**Suggested owner:** ArchDev.

### §23.4.6.6 Failure-mode harness (`--hmpc-outer-disable`)

**What:** A CLI flag that runs the controller with the outer planner disabled. Internally: the outer's `solve()` is bypassed and the reference is constructed from the DP plan + centreline once, at controller construction, and never refreshed. The inner runs against this static reference.

**Why:** isolates "is the outer adding value over a pure inner tracking the DP plan?" Without this flag, an HMPC run with a worse lap time than `--controller mpc` is ambiguous (is the outer wrong, or is the trim PI's tuning wrong?). With the flag, the comparison is cleanly inner-vs-inner.

**Touchpoints:** internal to `hmpc_controller.py`; one CLI flag in `lap.py`.

**Dependencies:** §23.4.6.4.

**Suggested owner:** ArchDev.

### §23.4.6.7 CLI plumbing

**What:** Add to `lap.py`:

- `--controller hmpc` — add to `choices`.
- Outer:
  - `--hmpc-outer-horizon-m FLOAT` (default 200.0)
  - `--hmpc-outer-n-stages INT` (default 40)
  - `--hmpc-outer-rate-hz FLOAT` (default 2.0)
  - `--hmpc-outer-mu-circle FLOAT` (default None → `calib.mu_long_eff`)
  - `--hmpc-outer-w-progress FLOAT` (default 1.0)
  - `--hmpc-outer-w-n FLOAT` (default 5.0)
  - `--hmpc-outer-w-du FLOAT` (default 0.5)
- Inner:
  - `--hmpc-inner-horizon-m FLOAT` (default 30.0)
  - `--hmpc-inner-n-stages INT` (default 15)
  - `--hmpc-inner-tick-hz FLOAT` (default 50.0)
  - Inner cost weights inherit from v3.2 `mpc` block by default; can be overridden by `--hmpc-inner-w-v`, `--hmpc-inner-w-n`, `--hmpc-inner-w-psi-e`, `--hmpc-inner-w-du`, `--hmpc-inner-w-du2`.
- PI trim:
  - `--hmpc-pi-kp-n FLOAT` (default 0.05)
  - `--hmpc-pi-ki-n FLOAT` (default 0.01)
  - `--hmpc-pi-kp-vx FLOAT` (default 0.02)
  - `--hmpc-pi-ki-vx FLOAT` (default 0.005)
  - `--hmpc-pi-bound-steer FLOAT` (default 0.15)
  - `--hmpc-pi-bound-pedal FLOAT` (default 0.10)
- Diagnostics / misc:
  - `--hmpc-outer-disable` (no-arg flag) — §23.4.6.6
  - `--hmpc-force-static-fz` (no-arg flag) — inner only; outer is always point-mass
  - `--hmpc-debug-trace` (no-arg flag)

Driver-JSON `control_params.hmpc` block mirrors the same keys with the `hmpc.` prefix stripped. Resolution priority: CLI > driver JSON > dataclass defaults.

**Touchpoints:** `lap.py` arg parser, `slip_simulator.py` arg forwarding.

**Dependencies:** §23.4.6.4.

**Suggested owner:** ArchDev.

---

## §23.4.7 Data & interface contracts

### §23.4.7.1 Information-flow contract (outer → inner)

**Producer:** `OuterPlanner.solve()` writes a `ReferenceTrajectory` (§23.4.6.2 dataclass).

**Consumer:** `InnerTracker.solve()` reads via `reference.n_ref_at(s)`, `reference.v_ref_at(s)`, `reference.psi_e_ref_at(s)` (linear interp on `reference.s_outer`).

**Invariants:**
1. `reference.s_outer[0] ≤ s_chassis_at_solve` (i.e., the reference covers the chassis's current position).
2. `reference.s_outer[-1] ≥ s_chassis_at_solve + 150 m` (the reference covers at least the inner's horizon plus headroom).
3. `reference.v_ref` is monotone-Lipschitz: `|v_ref_k+1 − v_ref_k| ≤ a_long_max · ts_outer_k`. The outer's friction-circle constraint enforces this; the inner trusts it.
4. `reference.n_ref` is bounded: `|n_ref_k| ≤ track_half_width(s_k) − safety_buffer`.
5. `reference` is **immutable**. The outer produces a new one; the inner never edits.

**Failure mode:** if any invariant is violated (e.g., outer returned an infeasible-but-completed solution), the inner detects it on the first `n_ref_at(s)` / `v_ref_at(s)` lookup and falls back to DP-plan tracking (§23.4.6.4 Tier-1).

### §23.4.7.2 `ReferenceTrajectory` table layout

See §23.4.6.2 dataclass. Stored in `HMPCController._reference: ReferenceTrajectory | None`. `None` only before the first outer solve completes.

### §23.4.7.3 Cost weights — initial values

**Outer (point-mass + friction circle):**

| Weight | Default | Range | Notes |
|---|---|---|---|
| `w_progress_o` | 1.0 | [0.5, 5.0] | Pushes `v_k` up. If outer produces a too-conservative reference, raise this. |
| `w_n_o` | 5.0 | [1, 50] | Penalises deviation from centreline (`n = 0`). Outer is not racing-line-optimal; it stays mid-track. |
| `w_du_o` | 0.5 | [0.1, 5] | Smooths `a_long`, `a_lat` between stages. |
| `w_term_o` | 5.0 | [1, 50] | Terminal `n_N²`. |

**Inner (re-uses v3.2 `MPCWeights`):**

Defaults from v3.2 Phase 5.0.4. Sweep at build time only if the inner can't track the outer's reference within `n_p95 ≤ 1.5 m` and `v_x_p95 ≤ 3 m/s`. If the inner is constraint-saturated (ellipse at the limit) and still can't track, the outer's reference is too aggressive — raise `w_du_o` or lower `w_progress_o`, don't raise inner weights.

**PI trim:** defaults in §23.4.6.4. Sweep range: `K_p_n ∈ [0.02, 0.1]`, `K_p_vx ∈ [0.01, 0.05]`. Integral gains held at 20 % of proportional.

### §23.4.7.4 Outer cadence + staleness contract

| Quantity | Default | Bound | Notes |
|---|---|---|---|
| `f_outer` | 2.0 Hz | [1, 5] Hz | Configurable via `--hmpc-outer-rate-hz`. |
| `Δt_outer` | 500 ms | — | = `1/f_outer`. |
| `f_inner` | 50.0 Hz | [10, 100] Hz | Same as v3.2 default. |
| `Δt_inner` | 20 ms | — | |
| Max staleness in ticks | 25 ticks | — | = `Δt_outer / Δt_inner` at default cadences. The inner runs against a reference up to 25 ticks old. |
| Forced re-solve trigger | `s_chassis ≥ s_horizon_end − 30 m` | hard | The outer fires even if the cadence timer hasn't elapsed. |
| Cold-start (no outer solution yet) | First outer solve runs blocking on tick 0 | — | Worst-case 200 ms delay on the first tick; recorded but not a failure. |

**Staleness policy:** frozen-reference (no extrapolation). Rationale: the reference is `(n_ref(s), v_ref(s))` as a function of `s`, not of time. As long as the chassis is in the reference's `s` range, the reference is valid. Time-staleness only matters if the chassis runs off the end of the horizon, and the forced-re-solve trigger handles that case.

### §23.4.7.5 Coordinate-frame contract

- **Outer:** curvilinear `(s, n, ψ_e, v)`. Output also curvilinear: `(s_grid, n_ref, ψ_e_ref, v_ref)`.
- **Inner:** chassis-frame state `(x, y, ψ, v_x, v_y, ω, δ, throttle, brake)` internally (unchanged from v3.2). The cost terms read curvilinear-frame reference values via the projection `s = projector(x, y)`. The inner's friction ellipse stays in chassis frame (forces are chassis-frame; coordinate basis is irrelevant — same observation as §23.3.6.3).
- **Trim PI:** consumes curvilinear errors `(e_n, e_vx)`. Cross-track `e_n` is computed via the same projection helper.

The projection helper (§23.4.6.1) is shared with `mpcc_reference.py`. One implementation, two callers.

### §23.4.7.6 Driver JSON `control_params.hmpc` schema

```json
{
  "control_params": {
    "hmpc": {
      "outer_horizon_m": 200.0,
      "outer_n_stages": 40,
      "outer_rate_hz": 2.0,
      "outer_w_progress": 1.0,
      "outer_w_n": 5.0,
      "outer_w_du": 0.5,
      "inner_horizon_m": 30.0,
      "inner_n_stages": 15,
      "inner_tick_hz": 50.0,
      "inner_w_v": null,
      "inner_w_n": null,
      "inner_w_psi_e": null,
      "inner_w_du": null,
      "inner_w_du2": null,
      "pi_kp_n": 0.05,
      "pi_ki_n": 0.01,
      "pi_kp_vx": 0.02,
      "pi_ki_vx": 0.005,
      "pi_bound_steer": 0.15,
      "pi_bound_pedal": 0.10,
      "dynamic_fz_enabled": true
    }
  }
}
```

`null` falls through to v3.2 `mpc` block (for `inner_*`) or hard-coded defaults.

### §23.4.7.7 `SlipSimResult` additions

Listed in §23.4.6.5. v3.2 `mpc_*` and v3.3 `mpcc_*` fields coexist; an HMPC run populates `hmpc_*` only.

---

## §23.4.8 Acceptance gates (§11.55-HMPC)

| Gate | Threshold | Notes |
|---|---|---|
| MC 3-lap completions, Sprint A, chicane_safety_mult ≤ 0.85 | **≥ 7 / 10** | Parity with reactive baseline (`2:09.12`). |
| Mean best lap (Sprint A, completed runs) | **≤ 2:09** | Parity gate — proves no regression. |
| Stretch mean best lap | **≤ 2:00** | The structural-win threshold. 9 s gain. |
| Stretch² mean best lap | ≤ 1:55 | Within 7.5 s of Tomas (1:47.56). Publishable. |
| Outer solve mean | < 100 ms | At 200 m horizon, 40 stages, point-mass. |
| Outer solve p99 | < 200 ms | |
| Inner solve mean | < 30 ms | Same as v3.2 Phase 5.0.4. |
| Inner solve p99 | < 50 ms | Same as v3.2. |
| Outer-staleness-ticks p95 | ≤ 25 ticks | At 2 Hz outer / 50 Hz inner. |
| PI trim p95 (steer) | ≤ 0.10 × δ_max | Within bounds; if at the bound, outer is wrong. |
| PI trim p95 (throttle/brake) | ≤ 0.08 abs | Same logic. |
| Cross-track peak | ≤ 6 m | Tier-2 trigger; matches v3.2. |
| Post-solve ellipse violation p95 (inner) | ≤ 0.05 | Reuses Phase 5.0.4 metric. |
| Tier-0 share | ≥ 85 % | A bit lower than `--controller mpc`'s 90 %; the outer-vs-inner disagreement legitimately raises Tier-1 use. |
| Tier-1 (DP-plan fallback) share | ≤ 12 % | |
| Tier-2 (reactive) share | ≤ 3 % | Hard safety belt. |
| Regression: `--controller reactive`, `mpc`, `mpcc` unchanged | Pass | HMPC is additive. |
| Outer-disabled comparison run | `--hmpc-outer-disable` matches `--controller mpc` headline lap | Sanity check (§23.4.6.6). |

---

## §23.4.9 Failure modes and fallback policy

### §23.4.9.1 Outer infeasible

**Trigger:** OSQP returns `infeasible` after 2 SQP iterations on the outer.

**Action:** keep the last-good `ReferenceTrajectory`; increment `outer_infeas_count` diagnostic; do not invalidate the inner. The cadence timer continues; next outer fire is `Δt_outer` later. If 3 consecutive outer solves fail, log a warning and force the inner into Tier-1 (DP-plan fallback) on the next tick.

### §23.4.9.2 Inner infeasible against outer reference

**Trigger:** inner's `solve_sqp` returns infeasible after 3 SQP iterations OR `tier_counts` shows Tier-2 fall-through.

**Action:** retry the same inner solve with the DP plan as the reference (Tier-1). If the retry succeeds, commit; record this tick as Tier-1. If the retry also fails, fall through to reactive (Tier-2). Two consecutive Tier-2 ticks abort the lap, same as v3.2.

### §23.4.9.3 Outer–inner disagreement

**Symptom:** the PI trim's `e_vx` or `e_n` saturates at the bound for > 0.5 s.

**Diagnosis (build-time):** likely cause is the outer's friction circle being looser than the inner's friction ellipse — outer plans more grip than inner can deliver. Resolve by setting `μ_circle = 0.85 · μ_inner_ellipse_avg` (a 15 % conservative buffer).

**Action at runtime:** none beyond logging — the PI bound enforces that the trim can't override the inner. The inner is the source of truth for what the chassis can actually do.

### §23.4.9.4 Chassis runs off the outer's horizon

**Trigger:** `s_chassis > reference.s_horizon_end − 5 m` (5 m margin in addition to the 30 m forced-re-solve trigger).

**Action:** if this happens, the forced-re-solve has not fired in time — outer is stuck. Inner immediately falls back to Tier-1 (DP-plan tracking). Diagnostic counter `outer_overrun_count` increments.

### §23.4.9.5 Cold start

**Trigger:** very first tick of a stint; `_reference is None`.

**Action:** inner runs in Tier-1 mode (DP-plan tracking) for ticks 0..K until the first outer solve completes (typically K ≤ 10 ticks at 50 Hz tick / 200 ms outer solve). After first outer completes, switch to Tier-0.

### §23.4.9.6 Tier-2 (reactive) fallback semantics

Same `_long_sub` parent instance pattern as v3.2 (`mpc_controller.py:531`). The reactive's state (Stanley integrator, brake-rate history) is **always** ticked alongside the HMPC, even when in Tier-0, so the reactive is "warm" the moment it's needed. This matches v3.2's existing pattern.

---

## §23.4.10 Integration touchpoints (concrete file list)

**New files:**
- `src/lap_estimator/dynamics/hmpc_outer.py` (~ 350 lines)
- `src/lap_estimator/dynamics/hmpc_inner.py` (~ 150 lines; thin wrapper)
- `src/lap_estimator/dynamics/hmpc_controller.py` (~ 350 lines)
- `src/lap_estimator/dynamics/hmpc_debug.py` (~ 100 lines)

**Edited files:**
- `src/lap_estimator/dynamics/slip_simulator.py` — add `"hmpc"` branch in `_make_controller` (~ line 471); extend `SlipSimResult` (~ line 60–80).
- `src/lap_estimator/dynamics/mpc_qp.py` — extend `solve_sqp` signature with optional `n_ref` / `psi_e_ref` kwargs (§23.4.6.3). Behaviour preserved when args are `None`.
- `lap.py` — add `"hmpc"` to `--controller choices` (~ line 122); add the CLI flags listed in §23.4.6.7 (~ inside the controller-flags block).
- `src/lap_estimator/dynamics/mpcc_reference.py` (or new `curvilinear.py`) — factor out the projection helper to be shared with HMPC (§23.4.6.1).

**Untouched (explicitly):**
- `src/lap_estimator/dynamics/mpc_controller.py` — no edits; `--controller mpc` unchanged.
- `src/lap_estimator/dynamics/mpc_controller_tiers.py` — no edits.
- `src/lap_estimator/dynamics/mpc_qp_ellipse.py` — no edits (Tier-1 logic re-used through the inner unchanged).
- `src/lap_estimator/dynamics/mpc_model.py` — no edits.
- `src/lap_estimator/dynamics/mpcc_controller.py` / `mpcc_qp.py` / `mpcc_model.py` (when they land from §23.3) — no edits; MPCC unaffected.
- `src/lap_estimator/dynamics/longitudinal_planner.py` — no edits.
- `src/lap_estimator/dynamics/solver.py` — no edits (ground-truth plant unchanged).
- `tests/` — out of scope per user golden rules; manual QA only.

---

## §23.4.11 Risks and mitigations

**Risk 1 — Outer planner is too coarse to be useful.** Point-mass + friction circle ignores load transfer and tyre coupling; the outer might emit a `v_ref` that's structurally unreachable by the inner. **Mitigation:** (a) `μ_circle` defaults to `0.85 · calib.mu_long_eff` — a 15 % conservative buffer; (b) when the PI trim saturates at the bound for > 0.5 s in a run, ArchDev reduces `μ_circle` further (sweep 0.85 → 0.75 → 0.65 if needed); (c) §23.4.6.6 `--hmpc-outer-disable` provides a clean A/B baseline. If the outer is strictly worse than the inner-only path at every `μ_circle`, the architecture is wrong and we ship reactive.

**Risk 2 — Outer fires too slowly; inner consumes stale references through major corners.** At 2 Hz outer / chassis at 70 m/s, the chassis covers 35 m between outer solves. **Mitigation:** the forced-re-solve trigger (§23.4.7.4) fires when `s_chassis ≥ s_horizon_end − 30 m`; at horizon 200 m and chassis at 70 m/s this means re-solves are typically ~2.4 s apart in straights but tighter into corners. If the forced-re-solve fires more than 50 % of the time, the cadence is too slow — raise to 3 Hz or 5 Hz at the cost of outer solve-time gates.

**Risk 3 — Inner cost ill-conditioned because outer's `v_ref` is itself wiggly.** A noisy `v_ref(s)` from successive outer solves can make the inner's tracking cost chatter the steering. **Mitigation:** `w_du_o` enforces stage-to-stage smoothness in the outer's `(a_long, a_lat)`; combined with the curvature-of-input penalty already in the inner (`w_du2`), this is enough in v3.2 / §23.3 by analogy. If the inner's commit chatters between successive outer solves, low-pass-filter the outer's `v_ref` and `n_ref` with τ = 100 ms before serving them to the inner — one helper inside `hmpc_controller`.

**Risk 4 — Outer's friction-circle linearisation falls apart at sharp curvature.** Same algebra as `mpc_qp_ellipse.build_ellipse_rows`; same risk profile as §23.3.11 Risk 1. **Mitigation:** clamp `κ_ref(s)` to `[−0.2, 0.2]` rad/m at reference-build time, same as §23.3.6.1. Outer never sees apex radii tighter than 5 m.

**Risk 5 — PI trim mis-tuned, oscillates around the reference.** **Mitigation:** start at the §23.4.6.4 defaults (`K_p_n = 0.05`, `K_i_n = 0.01`); if the trim p95 hits the bound, the cause is more likely outer-reference infeasibility than PI tuning — diagnose with `--hmpc-outer-disable`. If the trim chatters but never saturates, tune `K_p_n` down by 2× and re-test.

**Risk 6 — `_long_sub` reactive sub-controller fights the PI trim.** Both modify throttle/brake. **Mitigation:** the PI trim adds to `_long_sub`'s output, then clips to actuator limits. If `_long_sub` is already at full throttle and PI says "add 0.1 throttle", the clip absorbs it; same on brake. The PI's role is small corrections, not authority transfer. If diagnostics show `_long_sub` and PI trim consistently disagreeing on sign (one wants throttle, the other brake), the outer's `v_ref` is significantly wrong — investigate the outer.

**Risk 7 — Hierarchical wallclock budget exceeded.** Outer 100 ms + inner 30 ms = 130 ms per "burst" if both fire on the same tick. **Mitigation:** the outer runs asynchronously of the inner — formally, in the current synchronous codebase the outer's solve happens on a tick where the inner also runs. Schedule the outer to fire on *odd* inner ticks only (avoiding the worst-case stack of "outer + inner + everything else"). If outer + inner together still bust the wallclock per the sim loop's expected tick duration, drop outer cadence from 2 Hz → 1 Hz; the staleness bound (§23.4.7.4) handles up to 50 ticks (= 1 Hz) without architectural change.

**Risk 8 — Pacejka operating-point linearisation diverges in the inner when the outer's `v_ref` jumps.** The inner's LTV-bicycle warm-start is the previous inner solve; if the outer just produced a brand-new reference that's far from the inner's previous predicted trajectory, the linearisation point is far off. **Mitigation:** v3.2 already handles this via the SQP outer-iteration count (default 3). For HMPC, raise the inner's SQP cap to 4 for the **first inner solve after each outer solve** only; subsequent inner solves stay at 3.

**Risk 9 — Outer–inner reference frame mismatch at the start/finish line.** `s` wraps; `ψ_ref(s)` is periodic. Same risk as §23.3.11 Risk 7. **Mitigation:** identical — project `s` modulo `track_length`; treat `(s_chassis − reference.s_outer[0])` as signed-shortest-path distance. The helper lives in §23.4.6.1's shared module.

---

## §23.4.12 Build-time tuning protocol

A concrete sequence for ArchDev so the build doesn't turn into an open-ended sweep:

**Step 1 — Outer-only validation (no inner, no PI).**
Run the outer planner standalone on Sprint A, dump the `ReferenceTrajectory` for the chicane region, plot `v_ref(s)` against the DP plan's `v(s)` and Tomas's real `v(s)` from `tracks_csv/ks_nurburgring/layout_sprint_a__tomas_full_sim_trace_slip.csv`. Acceptance: outer's `v_ref` is anticipatory — drops below the DP plan's `v(s)` *before* the DP plan does at the chicane entry. If not, the outer is broken; debug before going further.

**Step 2 — Inner-only validation (`--hmpc-outer-disable`).**
Run with the outer disabled. The inner should match `--controller mpc` headline numbers (0/10 completion, ~30 ms p95 solve). If the inner regresses, the cost-function refactor in §23.4.6.3 (adding `n_ref`, `psi_e_ref` to `solve_sqp`) introduced a bug — fix before going further.

**Step 3 — Hierarchical first run (defaults + `μ_circle = 0.85 · μ_inner`).**
Run `--controller hmpc` with all defaults on Sprint A. Three outcomes:
- (a) ≥ 7/10 completion AND lap ≤ 2:09 → success, proceed to lap-time tuning (Step 5).
- (b) Improved completion but lap > 2:09 → outer is too conservative; raise `w_progress_o`, lower `w_n_o`, re-run.
- (c) No improvement or regression → diagnose with traces; check PI trim saturation, outer infeas count, outer-vs-inner disagreement p95. Likely Risk 1 or Risk 3.

**Step 4 — Outer cadence sweep.**
Sweep `--hmpc-outer-rate-hz` ∈ {1, 2, 3, 5}. Pick the rate that minimises lap time while keeping outer solve p99 < 200 ms.

**Step 5 — Lap-time push.**
Raise `--hmpc-outer-w-progress` from 1.0 → 2.0 → 5.0 with completion held ≥ 7/10. The progress weight is the "be more aggressive" knob; the friction circle bounds aggressiveness.

**Step 6 — Stretch (only if Step 5 lands ≤ 2:05).**
Tighten `μ_circle = 0.95 · μ_inner` (less conservative). Re-run completion gate. If stable, accept; if completion drops below 7/10, revert.

ArchDev records the sweep in `docs/architecture-slip-model-phase5_1-v34-hierarchical-mpc.md`.

---

## §23.4.13 Alternatives considered

**(a) Flat MPCC with longer horizon (§23.3, scaled up to 100 m).** Spec'd in §23.3.10 open question 1 and §23.3.11 Risk 4. Bottleneck is the LTV bicycle's linearisation tax at horizon ≥ 50 m: OSQP solve times grow superlinearly with horizon length for the curvilinear-frame dynamics, and 100 m horizon at 25 fine stages is already at the OSQP-acados crossover. Rejected as the structural fix; hierarchical splits the long-horizon problem onto a cheaper plant. Flat MPCC stays in-tree for the 50 m regime where it adds value over v3.2 tracking-MPC.

**(b) acados switch on the existing v3.2 cost.** Solves the same OCP faster. Doesn't extend the lookahead — same horizon, same cost shape, same blind-to-chicane problem. Rejected as the structural fix; acados is the §23.3.12 escalation for solver-tax-limited regimes.

**(c) Pre-computed offline trajectory (TUMFTM `global_racetrajectory_optimization`).** IPOPT solves the full minimum-time trajectory offline, capping at the chassis's ellipse limits with Pacejka. Could be used as the outer's warm-start, or even replace the outer entirely with a once-per-lap lookup. **Strong** option, but: (i) it requires the same physics ingredients we're already shipping (Pacejka, dynamic Fz, friction ellipse) — recomputing offline doesn't unlock anything new at runtime; (ii) it assumes the chassis can track a pre-baked plan, which is exactly what the inner does anyway; (iii) it doesn't react to inner-vs-outer disagreement (e.g. tyre wear / fuel load / driver-input lag). Recommended **follow-up** spec: use it to seed `OuterPlanner`'s first-lap warm-start. Not in this spec.

**(d) Tomas-trajectory injection as the outer reference.** Cheap; capped at Tomas's line; same caveat as §23.3.13(e). Doesn't generalise across tracks; not a structural fix. Recommended as a separate small spec (the reference-source slot is already pluggable).

**(e) LMPC (Rosolia & Borrelli).** Best ceiling under the model. Needs 10–30 laps of bootstrap. Our MC runs are 1–3 lap; LMPC's terminal-set buildout doesn't fit cleanly. Defer.

**(f) MPPI (Williams et al.) as the outer.** Sampling-based; doesn't linearise; naturally handles long horizons. Step-change architecture; requires GPU or aggressive CPU vectorisation to hit our solve budgets. Defer; recommended fallback if hierarchical at `μ_circle ≤ 0.65 · μ_inner` still fails to deliver structural completion improvement.

**(g) Tube MPC + offline planner (TUMFTM IAC stack).** Highest-fidelity reference. The "tube" is robustness against modelling error; we're not modelling-error-bound yet (we're horizon-bound). Rejected as overkill for our current bottleneck. Re-evaluate if hierarchical works and tube-style robustness is the next gap.

---

## §23.4.14 References

- Liniger Hierarchical MPCC (2020): https://arxiv.org/abs/2003.04882 — canonical academic reference for the architecture; +20 % vs prior SOTA on 1:43 RC.
- TUMFTM IAC (2022): https://arxiv.org/abs/2205.15979 — Indianapolis Autonomous Challenge 270 km/h; outer planner + Tube-MPC inner stack.
- TUMFTM `mod_vehicle_dynamics_control` (BSD): https://github.com/TUMFTM/mod_vehicle_dynamics_control — production-grade hierarchical implementation.
- TUMFTM `global_racetrajectory_optimization` (BSD): https://github.com/TUMFTM/global_racetrajectory_optimization — offline IPOPT planner with full Pacejka; candidate warm-start source for the outer.
- AMZ Driverless lateral/longitudinal split — academic reference for two-layer separation of concerns.
- Stanford DDL Pikes Peak (Funke et al., 2016) — feedforward + PI trim pattern that this spec's PI layer follows.
- Velenis & Tsiotras 2007 — trail-braking as minimum-time solution under friction-ellipse constraints (continues to motivate the outer's progress-reward shape).
- Existing v3 entry points referenced throughout:
  - `src/lap_estimator/dynamics/mpc_controller.py`
  - `src/lap_estimator/dynamics/mpc_qp.py`
  - `src/lap_estimator/dynamics/mpc_qp_ellipse.py`
  - `src/lap_estimator/dynamics/mpc_model.py`
  - `src/lap_estimator/dynamics/mpc_controller_tiers.py`
  - `src/lap_estimator/dynamics/mpcc_reference.py` (when §23.3 lands)
  - `src/lap_estimator/dynamics/longitudinal_planner.py`
  - `src/lap_estimator/dynamics/slip_simulator.py:_make_controller` (line ~471)
  - `lap.py --controller` (line ~122)
- v3 session artefacts:
  - `docs/architecture-v3-session-2026-05-24.md` (cross-ref this spec from here once written)
  - `docs/architecture-slip-model-phase5_0_8-v32-first-class-longitudinal.md` (the 0/10 result that motivated this spec)
  - Sibling specs `spec-section-23-2-*.md` and `spec-section-23-3-v33-mpcc.md`

---

## §23.4.15 Cross-references to update after this spec lands

1. `docs/architecture-v3-session-2026-05-24.md` — add a row in "What shipped" pointing at this spec; add a row in "Known remaining gaps" stating that hierarchical MPC is the next architectural lever after Phase 5.0.8.
2. `docs/AI_CONTEXT.md` — extend the v3 controller listing to include `--controller hmpc`.
3. `dev-planning/lap-simulation-csv-driver/spec-section-23-slip-model.md` — add a §23.4 subsection link.
4. `dev-planning/lap-simulation-csv-driver/open-points.md` — append the §23.4.11 open risks (1, 6, 7) as tracked items.
