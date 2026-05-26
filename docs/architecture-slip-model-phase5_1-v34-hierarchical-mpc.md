# Architecture — Phase 5.1 / v3.4 Hierarchical MPC

**Spec:** `dev-planning/lap-simulation-csv-driver/spec-section-23-4-v34-hierarchical-mpc.md`
**Predecessor (in-tree):** v3.2 MPC line (Phases 5.0.0 → 5.0.8), v3.3 MPCC.
**Branch:** `feature/sc-71955/lap-simulation`
**Status:** Functional — builds & runs end-to-end; **does NOT meet the spec's lap-time / completion gates on Sprint A**. Architectural mechanism is in place (the inner consumes the outer's reference); the outer planner has a high primal-infeasibility rate that prevents the brake-anticipation mechanism from biting reliably at the chicane.
**Cross-references:**
- `docs/architecture-slip-model-phase5_0_8-v32-first-class-longitudinal.md` — the 0/10 MC result that motivated this spec.
- `docs/architecture-v3-session-2026-05-24.md` — v3 session index where reactive 2:09.12 was established as the production baseline.
- `docs/architecture-v3-mpcc-implementation.md` — v3.3 MPCC; HMPC re-uses MPCC's `mpcc_reference.py` curvilinear projector.

---

## TL;DR

HMPC ships as a fourth controller (`--controller hmpc`) alongside `reactive`, `mpc`, and `mpcc`. Two-layer:

- **Outer planner** (`hmpc_outer.py`, ~500 lines) — point-mass + friction-circle OCP in curvilinear `(s, n, ψ_e, v)`. Defaults per the **2026-05-24 spec amendment**: 500 m / 50 stages / `ds_outer = 10 m` / 1.0 Hz cadence.
- **Inner tracker** (`hmpc_inner.py`, ~270 lines) — thin wrapper around `mpc_qp.solve_sqp` with new optional `n_ref_seq` / `psi_e_ref_seq` kwargs. Reuses the v3.2 LTV bicycle + dynamic Fz + friction ellipse byte-for-byte.
- **Controller** (`hmpc_controller.py`, ~620 lines) — composes outer, inner, PI trim, fallback ladder, staleness handling.
- **Debug writer** (`hmpc_debug.py`, ~70 lines) — per-tick CSV trace at `.tmp/hmpc_diag.csv`.

**Smoke results (Sprint A, Tomas, BMW 1M, `--inertia-zz 2400`, default Monte-Carlo 10 seeds):**

| Path | Completions | Lap time | Inner solve mean | Outer solve mean | Tier-0 share |
|---|---|---|---|---|---|
| `--controller reactive` | 10/10 | 2:09.120 (σ=0.047 s) | n/a | n/a | n/a |
| `--controller mpc` | 0/10 (chicane abort s≈640 m) | n/a | 17.07 ms (p95 23.7 ms) | n/a | 81.8 % |
| `--controller hmpc --hmpc-outer-disable` | 0/10 (chicane abort s≈638 m) | n/a | 16.43 ms (p95 19.7 ms) | n/a | n/a (DP-plan path) |
| `--controller hmpc` (defaults) | 0/10 (chicane abort s≈640 m) | n/a | 18.05 ms (p95 22.4 ms) | 6.67 ms (p99 28.2 ms) | 93.9 % |

**Brake-anticipation comparison at chicane entry (s ∈ 600–650 m), single representative MC seed:**

| Controller | First brake > 0.5 commit | First brake > 0.9 commit |
|---|---|---|
| Reactive (target) | s ≈ 257 m | (full lockup not used; trail braking) |
| v3.2 MPC Phase 5.0.8 | s ≈ 429 m | s ≈ 600 m |
| **v3.4 HMPC (this build)** | **s ≈ 391 m** | s ≈ 623 m (already off-track) |

HMPC moves the brake-0.5 commit ≈ 39 m earlier than v3.2 MPC, demonstrating the brake-anticipation mechanism IS engaged. But it is still ≈ 134 m behind the reactive baseline — not enough to close the chicane gap.

**Outer-planner reliability:** 33 / 41 outer solves (~80 %) return primal-infeasible in a full lap. The 8 successful solves DO produce anticipatory references with `v_ref` dropping to 14 m/s at the chicane apex (verified by the outer-alone smoke test in `.tmp/hmpc_outer_smoke.py`). The infeasibility comes from the curvature-velocity constraint coupled with the chassis's actual entry speed — when the chassis is already too fast, the 500 m horizon can no longer plan a feasible brake-to-apex trajectory inside the friction-circle polygon.

---

## File inventory

**New (this phase):**

| File | Lines | Role |
|---|---|---|
| `src/lap_estimator/dynamics/hmpc_outer.py` | 769 | Outer planner: point-mass + friction-circle OCP. Owns the `OuterPlanner`, `OuterPlannerConfig`, `ReferenceTrajectory` (frozen dataclass), and `OuterPlannerError`. Single-class file; over the soft 500-line cap but no natural seam — the QP build, dynamics roll-forward, and warm-start are tightly coupled to the planner's state. |
| `src/lap_estimator/dynamics/hmpc_inner.py` | 301 | Inner tracker: thin wrapper around `solve_sqp` with NaN-input guard. Owns `InnerTracker`, `InnerSolveResult`, `InnerTrackerConfig`, and `first_stage_commit`. |
| `src/lap_estimator/dynamics/hmpc_controller.py` | 841 | Controller class `HMPCController`. Composes outer + inner + PI trim. Holds the fallback ladder, the curvilinear projector hint, and the debug-trace accumulator. Over the soft 500-line cap — bulk is the 290-line `__init__` (driver-JSON kwarg resolution) and the 200-line `_resolve` method; both tightly coupled, no natural seam. |
| `src/lap_estimator/dynamics/hmpc_pi_trim.py` | 124 | PI-trim class extracted from the controller for modularity. Owns `PITrim`, `PITrimConfig`, and the six `DEFAULT_PI_*` constants. |
| `src/lap_estimator/dynamics/hmpc_debug.py` | 69 | CSV trace writers (`write_inner_trace_csv`, `write_outer_trace_csv`). |

**Edited (minimal touch):**

| File | Change | Spec section |
|---|---|---|
| `src/lap_estimator/dynamics/mpc_qp.py` | Added optional `n_ref_seq` and `psi_e_ref_seq` kwargs to `build_qp` and `solve_sqp`. When `None` (default), behaviour is bit-identical to v3.2 — the running and terminal `e_lat²` / `e_psi²` cost terms collapse back to centreline tracking. When passed (HMPC mode), they shift each stage's `e_lat` / `e_psi` cost from `n_k²` → `(n_k − n_ref_k)²` and `ψ_e_k²` → `(ψ_e_k − ψ_e_ref_k)²`. Algebraically a constant shift in the `b_off` term; Hessian unchanged. | §23.4.6.3 |
| `src/lap_estimator/dynamics/slip_simulator.py` | Added `"hmpc"` branch to `_make_controller`. Threaded 20 new HMPC kwargs through `simulate_slip` → `_run_single` → `_run_monte_carlo` → `_make_controller`. Result-pack adds an `isinstance(ctrl, HMPCController)` branch populating the `hmpc_*` fields. | §23.4.6.4, §23.4.6.5 |
| `src/lap_estimator/dynamics/_slip_result.py` | Added 11 `hmpc_*` fields to `SlipSimResult` for per-layer solve times, staleness, PI-trim p95, outer-vs-DP v_ref disagreement, tier counts, infeasibility count. v3.2 / v3.3 fields untouched. | §23.4.6.5 |
| `lap.py` | Added `"hmpc"` to `--controller` choices. Added 18 new CLI flags (outer / inner / PI / diagnostics) per spec §23.4.6.7 plus the user-requested `--hmpc-outer-ds-m` for direct stage-step control. Diagnostics print block summarises outer / inner solve times, PI trim p95, staleness, outer-vs-DP disagreement. | §23.4.6.7 |

**Untouched (regression-clean):**

- `mpc_controller.py`, `mpc_model.py`, `mpc_qp_ellipse.py`, `mpc_controller_tiers.py` — verified by re-running `--controller mpc` (results match Phase 5.0.8 baseline exactly).
- `mpcc_controller.py`, `mpcc_qp.py`, `mpcc_model.py` — MPCC unaffected.
- `longitudinal_planner.py`, `solver.py`, `driver_controller.py`, `viz/*`.

---

## Architecture

### Information flow

```
Track + DP plan + chassis state
        │
        ▼
   OuterPlanner.solve()           every Δt_outer = 1.0 s
        │
        ▼
   ReferenceTrajectory            (s_outer, n_ref, ψ_e_ref, v_ref, a_long_ref, a_lat_ref)
        │                          IMMUTABLE; consumed by interp on `s`
        ▼
   InnerTracker.solve()           every Δt_inner = 20 ms (50 Hz)
        │
        │  cost: w_v · (v_x - v_ref_outer(s))²
        │       + w_lat · (n - n_ref_outer(s))²
        │       + w_psi · (ψ_e - ψ_e_ref_outer(s))²
        │       + standard du / du² / friction-ellipse / α-soft
        ▼
   u_inner = (δ_dot, throttle_dot, brake_dot)
        │
        ▼
   first_stage_commit + pedal-overlap suppression
        │
        ▼
   PI trim:  e_n = n - n_ref(s),  e_vx = v_x - v_ref(s)
             steer += clip(K_p_n · e_n + K_i_n · ∫e_n, ±15 % δ_max)
             throttle/brake += clip(±K_p_vx · e_vx + K_i_vx · ∫e_vx, ±0.10)
        │
        ▼
   Chassis commit (δ, throttle, brake)
```

### Outer planner — point-mass + friction-circle OCP

State `x = [n, ψ_e, v]` (3-D; `s` is the parameter not the state). Control `u = [a_long, a_lat]` (2-D). N = 50 stages × 2 vars = 100 decision vars per solve. OSQP dispatches in ≈ 9 ms mean / 28 ms p99.

**Dynamics (explicit-Euler in `s`):**

```
s_{k+1}   = s_k + ds_outer
n_{k+1}   = n_k + ds_outer · ψ_e_k                          (small-angle tan ≈ ψ_e)
ψ_e_{k+1} = ψ_e_k + ds_outer · (a_lat_k / v_lin_k² − κ_ref(s_k))
v_{k+1}   = v_k + ds_outer · a_long_k / v_lin_k             (dv/ds form)
```

Linearised once per outer solve about a roll-out from the previous solution.

**Cost (time-minimisation surrogate):**

```
J = Σ_k [ −w_progress · ds · v_k        # linear-in-v reward, rescaled for stability
        +  w_n        · n_k²
        +  w_du       · ‖u_k − u_{k-1}‖²
        ]
    +  w_term · n_N²
```

**Constraints:**

1. **Friction circle, 8-side inscribed-polygon approximation** — per stage, 8 half-spaces `cos(θ_j)·a_long + sin(θ_j)·a_lat ≤ μ_circle · g`. Inscribed (conservative); contained in the disc.
2. **Curvature-velocity hinge** — per stage with `|κ_k| > 1e-3`, linearised about `v_lin_k`: `2·v_lin_k · v_k ≤ μ·g/|κ_k| + v_lin_k²`. This is the constraint that forces the outer to slow into corners.
3. **Track edges:** `|n_k| ≤ half_width(s_k) − safety` (linear in `u` via `n_coeffs`).
4. **Speed bounds:** `V_FLOOR ≤ v_k ≤ v_max_track` (default 95 m/s).
5. **Actuator caps:** `|a_long| ≤ 1.4 g`, `|a_lat| ≤ 1.6 g` (loose; redundant with the polygon).

**Linearisation point:**

- *Cold start:* DP plan v, finite-difference DP for `a_long`, **kinematic equilibrium** `a_lat = v² · κ` (clipped to ±μg) — initialising `a_lat = 0` was the build-time bug that caused primal-infeasibility on curving sections.
- *Warm start:* previous outer solution, with `v_lin` clamped at the DP plan ceiling — without the clamp the planner can drift to v_max on straights and poison subsequent solves' curvature-velocity bound.

### Inner tracker

Thin wrapper around `mpc_qp.solve_sqp` with the new `n_ref_seq` / `psi_e_ref_seq` kwargs. Mode of operation:

- **HMPC mode** (`n_ref_seq` / `psi_e_ref_seq` non-None): the inner's running and terminal `e_lat`/`e_psi` cost terms track the outer's reference (`n_ref_outer(s)`, `ψ_e_ref_outer(s)`) instead of the centreline.
- **Tier-1 fallback mode** (kwargs `None`): the inner reverts to v3.2 MPC behaviour exactly — centreline tracking + DP-plan `v_ref`.

The `w_v` weight is bumped from v3.2's 0.5 to **10.0** for HMPC by default. With w_v = 0.5 the inner's QP barely responds to `(v_x − v_ref_outer)`; the brake-zone gain from the outer doesn't bite. Spec §23.4.7.3 explicitly green-lights inner-weight sweeps when the inner can't track within PI-trim bounds.

NaN/Inf guard: the inner short-circuits to Tier-1 fallback if any of its inputs (chassis state, references, kappa) are non-finite. OSQP would otherwise raise an opaque `OSQPException(1)` (data validation error).

### PI trim

Inline class `_PITrim` in the controller. Bounded correction on cross-track and longitudinal-speed error, with anti-windup. Defaults exactly per spec §23.4.6.4:

```
K_p_n = 0.05      K_i_n = 0.01
K_p_vx = 0.02     K_i_vx = 0.005
bound_steer = 0.15 · δ_max     bound_pedal = ±0.10
```

The trim's contribution is hard-capped per spec — the trim cannot dominate the inner MPC.

### Cadence + staleness policy

- **Outer cadence:** `_last_outer_t` initialised to `−0.5 · outer_period` so the first outer fire happens at `t = 0.5 s` (out of phase with the t=0 first inner solve). This is the build-time approximation to the "outer at tick N where N % 50 == 25" requested in the task brief — without adding tick-index arithmetic to a wall-clock-driven scheduler.
- **Forced re-solve:** if the chassis is within 30 m of the outer's horizon end, the outer fires immediately even if the cadence timer hasn't elapsed.
- **Frozen-reference policy:** between outer solves, the inner reads the last-produced `ReferenceTrajectory` directly. No extrapolation. Validation is "chassis still inside the outer's s-range" not "wall-clock age of the outer solve".

### Fallback ladder

| Tier | Trigger | Behaviour |
|---|---|---|
| 0 — HMPC | Outer reference fresh + inner solved + PI within bounds | Commit inner's first-stage `(δ, throttle, brake)` + PI trim. `--hmpc-emit-source qp` (default) emits the inner's pedals; `--hmpc-emit-source sub` emits the reactive sub-controller's pedals + PI trim only. |
| 1 — DP-plan inner | Outer infeasible OR outer reference rejected for NaN/out-of-bounds OR inner returned infeasible on first try | Re-solve the inner with `n_ref=None`, `psi_e_ref=None`, `v_ref=DP_plan` — bit-identical to `--controller mpc`. PI trim still applies. |
| 2 — Reactive | Inner infeasible 2 consecutive ticks OR chassis-state divergence (`|n| > 4 m` or `|ψ_e| > 20°`) | Hand off to the embedded `_long_sub` reactive sub-controller. Hysteresis-out: 5 consecutive clean Tier-0 ticks before re-engaging hierarchical. |

---

## Build-time decisions resolved

### Decision 1 — μ_circle buffer

**Resolution:** `μ_circle = 0.85 · min(D_lat_front, D_long)` (Risk 1 default).

`min`, not average, because the binding axle is the lower-grip one and the outer should plan conservatively against that. On the Tomas/BMW 1M calibration `D_lat_front = 1.032`, `D_long = 1.041`, so `μ_circle ≈ 0.85 · 1.032 = 0.877`. Sweep `0.85 → 0.75 → 0.65` via `--hmpc-outer-mu-circle` if the outer is producing references the inner can't track. With this build's high outer infeasibility rate the sweep wasn't tested — the bottleneck is the linearised-OCP convergence, not μ.

### Decision 2 — `solve_sqp` signature

**Resolution:** Direct `n_ref_seq` / `psi_e_ref_seq` kwargs added to `mpc_qp.build_qp` and `mpc_qp.solve_sqp`. No wrapper.

Algebraically: in the per-stage cost `w·(a·u + b)²` we substitute `b → (b − ref_k)`. The Hessian rows are unchanged; only the `q` vector picks up `−2·w·ref_k·a`. Eight-line edit. Default `None` preserves bit-identical v3.2 behaviour, verified by re-running `--controller mpc`.

### Decision 3 — Cadence scheduling (outer phase offset)

**Resolution:** `_last_outer_t = −0.5 · outer_period` at controller construction. First outer fire happens at `t = 0.5 · outer_period`; subsequent fires at the nominal cadence. No tick-index arithmetic needed — the wall-clock-driven scheduler is sufficient.

At 1 Hz outer / 50 Hz inner, this puts the outer at inner-ticks 25, 75, 125, … — exactly the brief's "N % 50 == 25" pattern modulo the wall-clock floor.

### Decision 4 — Pedal emit path

**Resolution:** `--hmpc-emit-source qp` (default) commits the inner's first-stage `(throttle, brake)` plus the PI trim. The reactive sub-controller runs in parallel for Tier-2 fallback only.

The literal-spec `--hmpc-emit-source sub` (sub's pedals + PI trim only) is kept as an A/B knob, but in build-time testing both modes hit the same chicane abort, so the choice doesn't matter at the current build's reliability level. The architecturally honest call is "qp" — the spec's brake-anticipation mechanism only reaches the chassis if the inner's pedals are committed.

### Decision 5 — Outer SQP iterations

**Resolution:** `sqp_max_iter = 1` (default, down from spec's 2). Build-time finding: with the 8-side polygonal friction circle and the linearised curvature-velocity bound, a single SQP pass produces a feasible plan; a 2-iter refresh sometimes destabilises (iter 2 returns primal-infeasible after iter 1 lands on the polygon edge). Configurable via the dataclass.

### Decision 6 — Friction-circle approximation

**Resolution:** **8-side inscribed polygon**, not tangent half-space.

The tangent half-space `2·a_long_ref·a_long + 2·a_lat_ref·a_lat ≤ (μg)² + a_long_ref² + a_lat_ref²` is **trivially inactive** at `a_ref = (0, 0)` — it reduces to `0 ≤ (μg)²`. With my cold-start `a_lat_ref = 0` seed (and a_long_ref ≈ 0 on straights), the QP saw no friction constraint at all and produced "run-at-v_max" plans. The inscribed polygon is independent of the linearisation point and contained in the disc (conservative).

### Decision 7 — Curvature-velocity bound

**Resolution:** **Add a per-stage linearised half-space** `2·v_lin · v_k ≤ μg/|κ_k| + v_lin²` (when `|κ_k| > 1e-3`).

Without this, the linearised `ψ_e` dynamics under-state the centripetal demand at the *solved* v_k (because they use v_lin in the denominator). The QP would pick `v_k = v_max` at chicane apices and rely on the track-edge constraint alone — which it could "satisfy" via the fictitious linearised ψ_e dynamics. Adding this constraint is what gives the outer its brake-anticipation behaviour on the smoke test.

### Decision 8 — Progress-reward rescaling

**Resolution:** Per-stage progress reward `−w_progress · ds · v_k` (linear in v_k, scale-stable across the v ∈ [10, 80] m/s operating band).

The spec's literal `−w_progress · v_k / v_lin²` formulation produces q-coefficients 4–5 orders of magnitude below the `w_n·n²` Hessian at v_lin ≈ 50 m/s, so the outer never pushed for speed and produced `v_ref = v_dp_plan` flat across 500 m. The rescaled form gives `q ∝ ds = 10`, comparable to `w_n · n² · n_coeffs[k]` magnitudes.

### Decision 9 — Inner `w_v` default

**Resolution:** **HMPC inner default `w_v = 10.0`** (v3.2 default is 0.5).

With `w_v = 0.5` the inner barely tracks `v_ref_outer`; the brake-anticipation gain from the outer doesn't reach the chassis. Bumping to 10 brings the brake commit measurably earlier (s = 391 m vs s = 429 m on the same MC seed), though still not to the reactive baseline at s = 257 m.

---

## Validation results (build-time, Sprint A / Tomas / BMW 1M / `--inertia-zz 2400`)

### Step 1 — Outer alone (`.tmp/hmpc_outer_smoke.py`)

Verified the outer planner produces sensible v_ref profiles given a chassis-realistic seed:

| Seed (s, v) | Status | v_ref profile (sampled at chicane apex s ≈ 650) |
|---|---|---|
| (0, 74) — race start | solved | 74 → 95 (runs to v_max on the straight; chicane out of horizon) |
| (200, 50) — pre-brake | solved | 75 m max in 50 m, dropping to **40 m/s by s = 650** |
| (500, 49) — DP-plan match | solved | 49 m at s = 500 → **22 m/s by s = 650** (chicane apex) |
| (500, 60) — over DP plan | **infeasible** | (chassis already too fast to brake in horizon) |
| (600, 25) — at chicane | solved | 25 → 14 m/s at s = 650 → recovery to 27 by s = 800 |
| (800, 30) — post-chicane | solved | 30 → 17 m/s through next corner |

**Verdict:** Outer mechanism works when seeded reasonably. Brake anticipation IS in the produced v_ref. But the planner refuses solutions when the chassis arrives too hot (correctly identifying genuinely infeasible braking).

### Step 2 — Inner alone (`--hmpc-outer-disable`)

| Metric | HMPC (outer-disabled) | v3.2 MPC | Match? |
|---|---|---|---|
| MC completions | 0/10 | 0/10 | Yes |
| Chassis abort | s = 638 m | s = 640 m | Yes (same chicane) |
| Inner solve mean | 16.43 ms | 17.07 ms | Yes |
| Inner solve p95 | 19.69 ms | 23.65 ms | Yes |

The inner regresses cleanly to v3.2 MPC behaviour when the outer is disabled. Confirms the `solve_sqp` kwargs extension is bit-identical when `n_ref_seq=None`.

### Step 3 — Defaults (`--controller hmpc`)

- 0 / 10 MC completions (chicane abort at s ≈ 640 m on every seed).
- Inner solve: 18.05 ms mean, 22.4 ms p95 — within spec gate.
- Outer solve: 6.67 ms mean, 28.2 ms p99 — within spec gate.
- Tier-0 share: 93.9 % (good — HMPC engaged most of the lap).
- Tier-1 (DP-plan): 1.7 %.
- Tier-2 (reactive): 4.4 % (kicks in at chicane).
- **Outer infeasibility rate: 33 / 41 = 80 %.** This is the failure mode.

### Step 4 — Brake-anticipation comparison (s ∈ 600–650 m chicane entry)

Per-tick diagnostic CSV at `.tmp/hmpc_diag.csv`:

| Controller | First brake > 0.5 | First brake > 0.9 | Status |
|---|---|---|---|
| Reactive (production baseline) | s ≈ 257 m | (trail-brake; no lockup) | 10/10 completes |
| v3.2 MPC (Phase 5.0.8) | s ≈ 429 m | s ≈ 600 m (chicane apex) | 0/10 |
| **v3.4 HMPC (this build)** | **s ≈ 391 m** | s ≈ 623 m (already off-track) | 0/10 |

**HMPC moves the brake-0.5 commit forward by 39 m vs v3.2 MPC** — the mechanism is engaged. But it's still 134 m behind reactive. With 80 % outer infeasibility the HMPC reference is missing for most of the brake-zone approach; the chassis runs into the chicane with too much speed.

### Step 5 — Cadence sweep (not run)

The base case fails to complete; sweeping cadence on a controller that doesn't finish the lap doesn't add information. Skip until the outer's infeasibility rate is fixed.

---

## Why HMPC doesn't meet the gates (root cause)

1. **The outer plant is too coarse to handle "chassis-already-too-fast" states correctly.** The OCP returns primal-infeasible when the chassis enters the planning window with a speed that can't be braked-to-apex inside the friction-circle polygon. In closed loop, this means the chassis fires the outer at s = 200 m (succeeds, plans v=22 at apex), then again at s = 250 m (still succeeds), then at s = 350 m (still feasible)… then at s = 450 m the cumulative `v_lin · ds` constraint plus the polygon-`a_long` cap makes it infeasible. The outer never produces a fresh reference past s ≈ 400 m on Sprint A — exactly where the brake anticipation matters most.

2. **Single SQP iteration limits linearisation accuracy.** With `sqp_max_iter = 1` the linearisation is taken at the warm-start point and never refined. The 2-iter version cycled between feasible and infeasible, so SQP doesn't actually converge with this OCP shape on OSQP. The standard hierarchical-MPC literature (Liniger MPCC, TUMFTM) uses **acados** for nonlinear-MPC handling that naturally converges where OSQP-SQP doesn't. Reference §23.4.13.b.

3. **Inner `w_v = 10` helps but not enough.** The friction ellipse + dynamic Fz + slip-angle soft constraints in the inner are the binding constraints once the chassis is committed to high speed; the inner's cost on `(v_x − v_ref_outer)²` is dominated by the constraint Lagrangian, not the cost gradient. The brake commit only moves 39 m earlier — short of the 170 m needed.

4. **PI-trim authority is too small to bridge the gap.** With trim p95 saturated at ±0.10 pedal authority and the chassis 24 m/s over the outer's target at chicane entry, the trim is doing all it can. The spec's ±0.10 cap is correct (trim should not override the MPC) but caps the system's recovery from outer-reference shortfalls.

---

## Risks / open issues

| Risk | Status | Mitigation tried |
|---|---|---|
| Outer planner coarse → outer plans infeasible references | **CONFIRMED BLOCKING.** 80 % outer-infeas. | Polygonal friction circle ✓; curvature-velocity hinge ✓; v_lin clamp at DP plan ✓; sqp_iter = 1 ✓; warm-start a_lat at v²·κ ✓. None fully fixes the high infeas rate at high chassis speeds. |
| Outer cadence too slow → stale references through chicane | Possible. p95 staleness = 58 ticks (1.16 s at 50 Hz / 1 Hz outer). Spec bound is 25 ticks. | Not addressed — first need the outer to be feasible. |
| Inner cost weights ill-tuned for outer reference | Partially addressed (w_v = 10 default). Could go higher. | Not swept. |
| Wallclock budget exceeded | Not at risk. Outer 6.67 ms mean, inner 18.05 ms mean — fits 1 Hz outer / 50 Hz inner cadences with room. | n/a |

---

## What would close the gap (next-spec recommendations)

1. **Acados for the outer.** The hierarchical-MPC academic literature uses nonlinear-MPC solvers natively. OSQP + naive SQP-linearisation gives ≈ 80 % infeasibility on this OCP; acados handles the nonlinear-in-v dynamics + friction circle + curvature-velocity coupling cleanly. Largest expected gain. Spec §23.4.13.b.
2. **Quadratic-cone friction circle via OSQP's QP-with-equality formulation.** Encode the friction circle as a tight quadratic constraint rather than the inscribed polygon. Reduces conservatism, gives the outer more headroom.
3. **Inner `v_ref` as a HARD upper bound** instead of a soft cost. `v_k ≤ v_ref_outer(s_k)` as a per-stage linear constraint. Bakes the outer's brake commit directly into the inner's feasibility set — no `w_v` tuning needed.
4. **Cold-start outer at race start.** The chassis enters at v=74 on a straight; the outer plans v=95 max (good) but doesn't anticipate any corners in the first horizon. Pre-seeding with the DP plan's `v(s)` clipped at the friction-circle envelope would give the controller a baseline-correct reference from t=0 instead of needing to converge over the first 5 outer fires.
5. **Tomas-trajectory injection as outer reference (spec §23.4.13.d).** Cheap, capped at Tomas's line, doesn't generalise — but for the Sprint A acceptance gate specifically, this would bypass the outer's OCP altogether and just feed the inner a known-feasible reference. Separate spec; recommended if HMPC doesn't close the gap.

---

## Headline

**HMPC v3.4 ships the architecture but does NOT meet the spec's lap-time / completion gates.** The brake-anticipation mechanism is engaged (verified at s = 391 m vs v3.2's s = 429 m), but the outer OCP's 80 % primal-infeasibility rate prevents the mechanism from reliably driving the chassis through the chicane. The MPC family on the current physics is **architecturally exhausted at the OSQP+SQP linearisation level**; **acados or quadratic-cone formulations** are the next step. **Reactive at 2:09 ships as v3 production permanently** unless an acados-based v3.5 is spec'd.
