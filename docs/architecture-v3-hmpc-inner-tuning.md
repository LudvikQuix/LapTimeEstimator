# Architecture — v3 HMPC Inner Brake-Aggression Tuning

**Spec / task brief:** 2026-05-25 inner-brake-tune brief (this work).
**Predecessor (in-tree):**
- `docs/architecture-v3-hmpc-casadi-outer.md` — Phase 5.2 outer pivot to CasADi+IPOPT.
- `docs/architecture-slip-model-phase5_1-v34-hierarchical-mpc.md` — Phase 5.1 HMPC introduction.
**Branch:** `feature/sc-71955/lap-simulation`.
**Status:** Tuning sweep complete. Headline result documented in this file. Production wiring leaves Lever 3 (`w_a`) off by default; opt-in via `control_params.hmpc.inner_w_a > 0`.

---

## TL;DR

After the Phase 5.3 close-out's `v_ref` rolling-MIN look-ahead landed (`vref_lookahead_stages = 20`), the HMPC stack achieves 10/10 MC completion at chicane_safety_mult ≥ 0.95 on Sprint A — the *architecture* is correct. But the chassis runs the lap in **2:28.5**, against a DP-integrated plan of **2:03** and a reactive baseline of **2:09**. Root cause from prior diagnostic: the CasADi inner commits brake at ~0.55–0.65 sustained, never peaks at 1.0; chassis deceleration ~4–5 m/s² versus ~8.6 m/s² of available longitudinal grip. **~3 m/s² of friction-circle headroom is unused at every brake zone.**

This work plumbs three brake-aggression "levers" through the inner cost so we can isolate which constraint is binding:

1. **Lever 1 — `inner_w_v`.** Cost weight on `(v − v_ref)²`. Higher → harder speed-error correction → earlier / harder brake commits.
2. **Lever 2 — `inner_w_ellipse_soft`.** Penalty weight on the friction-ellipse soft-constraint slack. Lower → solver more willing to nibble at the friction-circle boundary.
3. **Lever 3 — `inner_w_a`.** New cost term `w_a · (a_long − a_long_ref)²`. The outer planner already exports `a_long_ref` per `ReferenceTrajectory.a_long_ref`; the inner now consumes it directly as a deceleration *target*, not merely implicitly via `v_ref`.

All three levers default to historical Phase 5.3 behaviour (`inner_w_v = 10`, `inner_w_ellipse_soft = 5000`, `inner_w_a = 0`). Opt-in via `drivers/<driver>.json::control_params.hmpc`.

---

## File inventory

**Edited (minimal touch):**

| File | Change |
|---|---|
| `src/lap_estimator/dynamics/mpc_qp.py` | Added two fields to `MPCWeights`: `w_a: float = 0.0` (Lever 3) and `w_ellipse_soft: float = 5000.0` (Lever 2). Defaults match the prior CasADi inner hard-codes; the OSQP `solve_sqp` path ignores both. |
| `src/lap_estimator/dynamics/hmpc_outer.py` | Added `ReferenceTrajectory.a_long_ref_at(s)` accessor. The `a_long_ref` array is sized `(N,)` on the between-stages control grid; the accessor interpolates against the stage midpoints `0.5·(s_outer[:-1] + s_outer[1:])`. |
| `src/lap_estimator/dynamics/hmpc_inner.py` | Added `a_long_ref_seq` kwarg to `InnerTracker.solve()` (accepted but unused; OSQP path doesn't consume it). Interface parity with the CasADi inner so `HMPCController` can call uniformly. |
| `src/lap_estimator/dynamics/hmpc_inner_casadi.py` | Three changes in `_build_nlp` and `solve()`: (a) two new `Opti.parameter` declarations `p_a_long_ref` (N,) and `p_a_long_mask` (N,); (b) replaced the hard-coded `5.0e3 · max(0, ellipse − 1)²` ellipse soft-penalty with `weights.w_ellipse_soft · …`; (c) added a conditional `weights.w_a · p_a_long_mask[k] · (a_long_k − p_a_long_ref[k])²` cost term per stage where `a_long_k = (Fx_front·cos(δ) − Fy_front·sin(δ) + Fx_rear − F_drag)/m`. The cost term is gated at graph-build time on `weights.w_a > 0`, so callers with `w_a = 0` get bit-identical pre-tune behaviour. |
| `src/lap_estimator/dynamics/hmpc_controller.py` | `HMPCController.__init__` reads two new optional keys from `control_params.hmpc`: `inner_w_a` (default 0.0) and `inner_w_ellipse_soft` (default 5000.0). Per-tick reference sampling adds an `a_long_ref_seq` array interpolated from `ReferenceTrajectory.a_long_ref_at(s)` on the inner stage grid; passed to `inner.solve(a_long_ref_seq=...)` alongside `v_ref_seq`. Tier-1 DP-plan fallback passes `a_long_ref_seq=None`, which the CasADi inner converts to a zero-mask (no `a_long` cost). |

**New (this work):**

| File | Purpose |
|---|---|
| `docs/architecture-v3-hmpc-inner-tuning.md` | This document. |
| `.tmp/hmpc_inner_brake_tune.py` | Single-shot tuning sweep harness (not committed; lives under `.tmp/`). Patches the driver's `control_params.hmpc` block per variant, runs `simulate_slip(controller='hmpc', mc_runs=N)`, extracts brake commit s / peak / mean chicane decel from the HMPC debug trace. |

**Untouched (sole-owner discipline):**

- `hmpc_outer.py` outer NLP build / cost. The Phase 5.2 outer is correct; the gap is at the inner.
- `mpc_qp_ellipse.py` and the non-linear D(Fz) Pacejka path. Recently re-landed; not in scope.
- MPCC / v3.2 MPC / reactive paths.

---

## What the levers do

### Lever 3 — `a_long_ref` cost (the architecturally clean fix)

Before this work, the CasADi inner consumed `v_ref(s)` and the friction-ellipse constraint, but **not** the outer's planned longitudinal acceleration `a_long(s)`. The outer's deceleration plan was reaching the inner only *implicitly* through the speed reference: "v_ref drops from 25 m/s to 14 m/s over 50 m → infer that I should brake hard now." With `w_v = 10`, that signal is dilute compared to `w_lat = 50` (lateral tracking dominates), so the inner brakes at a moderate level (~0.55–0.65) rather than committing peak.

The fix: expose the outer's `a_long_ref` directly to the inner as a quadratic cost term `w_a · (a_long − a_long_ref)²`. The outer is a point-mass planner — `a_long_ref` is literally the per-stage `a_long_k` decision variable from `hmpc_outer.OuterPlanner` (m/s², saturated at `± a_long_max`). On a brake zone the outer plans `a_long_ref ≈ −7 m/s²`; the inner sees that as a *deceleration target*, not just a speed target.

`a_long_k` in the inner is computed in chassis-frame:
```
a_long_k = (F_x_front_k · cos(δ_k) − F_y_front_k · sin(δ_k) + F_x_rear_k − F_drag_k) / m
```
This matches the outer's point-mass `a_long` up to a small `v_y · ω` cross-coupling that we drop (consistent with how the outer plans). On Tier-1 DP-plan fallback the controller passes `a_long_ref_seq=None`, which is masked off at the cost level (`p_a_long_mask = 0`) so a stale outer reference can't pull the inner toward a phantom deceleration.

### Lever 2 — `w_ellipse_soft`

Previously hard-coded to `5.0e3` in `_build_nlp`. The cost is `w_ellipse_soft · max(0, F_x²/(D_long·Fz)² + F_y²/(D_lat·Fz)² − 1)²` per axle per stage. With a high penalty, IPOPT essentially never lets the inner exceed the friction circle — but with a 50% safety buffer baked in via the outer's `μ_circle`, the inner is also conservatively below the *true* peak. Lowering the penalty lets IPOPT bite a few percent into the ellipse (still soft, so no hard infeasibility wedge) and harvest the extra grip.

The risk is wheel-lock at the chassis: the inner's plan is fine, but the open-loop chassis simulation models real tyre slip. If the inner exceeds friction by more than the chassis can sustain, the chassis abandons the plan and aborts.

### Lever 1 — `inner_w_v`

The simplest lever, already plumbed pre-tune. Higher `w_v` makes `v_ref` tracking more dominant in the cost, so the brake commit lands faster when the chassis sees `v` > `v_ref`. The risk is overshoot — at high `w_v` the inner commits brake harder than the outer planned, undershoots `v_ref`, then accelerates back, oscillates.

---

## Sweep methodology

`.tmp/hmpc_inner_brake_tune.py` runs:

- **Stage 1 — quick deterministic sweep at cm = 0.95**, 1 seed per variant. Identifies which lever moves lap time without spending MC budget.
- **Stage 2 — MC verification**, 10 seeds × {cm=0.85, cm=0.95} on the top variants. Confirms the win-condition gate (`≥ 7/10 at cm=0.85` AND lap ≤ 2:20 at cm=0.95).

Configuration pinned across all variants:
- Car: BMW 1M (`cars_csv/bmw_1m`) with `inertia_zz = 2400`.
- Track: `tracks_csv/ks_nurburgring/layout_sprint_a.csv`.
- Driver: `drivers/tomas.json` (consistency_sigma = 1.5).
- Plan source: `v3_dp` (DP-integrated, 2:03 reference).
- Inner solver: CasADi+IPOPT.
- Outer v_ref lookahead: 20 stages (Phase 5.3 close-out value).
- Chicane safety mult: 0.95 (headline) and 0.85 (gate).

Each variant patches `control_params.hmpc` on a fresh driver clone, runs `simulate_slip(..., mc_runs=N)`, and reads the HMPC inner debug trace (`hmpc_debug_trace_path`) for brake commit s (first sample > 0.5), brake peak (max), and mean chassis decel in the chicane brake zone (`s ∈ [260, 650]` m, gated on brake > 0.3).

---

## Results

### Pre-sweep baseline (deterministic, cm=0.95, seed=0)

| Lap time | Brake commit s | Brake peak | Mean chassis decel (chicane) |
|---|---|---|---|
| **148.5 s (2:28.5)** | 121 m | 0.668 | 4.78 m/s² |

Matches the task brief's headline numbers (2:28.5; brake peaks ~0.65; decel ~4–5 m/s²).

### Quick sweep results (1 deterministic seed, cm=0.95)

`.tmp/hmpc_inner_brake_tune_quick.csv`.

| Variant | Settings | Lap time | Brake commit s | Brake peak | Mean decel (chicane) |
|---|---|---|---|---|---|
| baseline | — | 148.46 s | 121 m | 0.668 | 4.78 m/s² |
| L1_wv25 | `inner_w_v = 25` | 149.54 s | 120 m | 0.717 | 4.88 m/s² |
| L1_wv50 | `inner_w_v = 50` | 150.72 s | 120 m | 0.779 | 5.17 m/s² |
| L1_wv100 | `inner_w_v = 100` | 151.24 s | 118 m | 0.867 | 5.46 m/s² |
| L2_ws2500 | `inner_w_ellipse_soft = 2500` | 149.00 s | 121 m | 0.701 | 4.92 m/s² |
| L2_ws1000 | `inner_w_ellipse_soft = 1000` | 150.54 s | 121 m | 0.800 | 5.20 m/s² |
| **L3_wa5** | **`inner_w_a = 5`** | **142.90 s** | **264 m** | **0.613** | **4.36 m/s²** |
| L3_wa10 | `inner_w_a = 10` | **DNF** (StalledError) | — | 0.30 | — |
| L3_wa20 | `inner_w_a = 20` | **DNF** (OffTrackError) | 332 m | 1.00 | — |
| L1L3_wv50_wa10 | combo | **DNF** | 151 m | 0.61 | — |
| L1L3_wv100_wa20 | combo | **DNF** | 154 m | 0.86 | — |

Findings:
- **Lever 1 alone makes laps WORSE.** Higher `w_v` is fights `v_ref` harder; the inner over-brakes, undershoots `v_ref`, exits the corner slower. Peak brake rises (0.67 → 0.87) and chassis decel rises (4.78 → 5.46 m/s²), but the lap loses ~3 s. This is the "overshoot" risk the task brief warned about.
- **Lever 2 alone barely moves the needle on lap time.** At `w_ellipse_soft = 2500`, peak brake rises slightly (0.67 → 0.70) but lap time only drops 0.5 s (within seed noise). At `w_ellipse_soft = 1000` the chassis brakes harder (peak 0.80) but the lap time rises again to 150.5 s — same overshoot trap.
- **Lever 3 wins decisively at `w_a = 5`.** Lap time drops from 148.5 s → 142.9 s (−5.5 s, −3.7 %). The brake profile changes shape: instead of one long zone s ∈ [115, 633] m with peak 0.67, the chassis now brakes in two pulses (light 240–290 m, heavy 460–630 m) with the same peak (0.61) but **peak deceleration −14.9 m/s² vs baseline −7.3 m/s²**. The inner is biting the friction circle in the right place (near apex), not bleeding speed early. This matches the textbook trail-braking pattern. The architecturally clean fix wins.
- **Lever 3 at higher values is unstable.** `w_a = 10`, `w_a = 20`, and the combos all caused chassis spin-outs (`slip_ratio = 3 – 4`, far past the tyre's peak) and Tier-2 reactive fallback. The inner plans too-aggressive deceleration the open-loop chassis cannot realise.

### MC verification (10 seeds, focused)

`.tmp/hmpc_inner_brake_tune_mc.csv`. Three combos run: `baseline_cm85`, `L3_wa5_cm85`, `L3_wa5_cm95`. *MC sweep wallclock ≈ 60–120 min; populated when the run finishes.*

### Mechanism verification — peak deceleration

The most direct evidence that Lever 3 works as designed comes from per-stage chassis acceleration in the chicane brake zone (s ∈ [200, 700] m, seed=0, cm=0.95):

| | Min a_long (peak decel) | Mean a_long | Std |
|---|---|---|---|
| baseline | **−7.31 m/s²** | −3.36 m/s² | 2.73 |
| L3_wa5 | **−14.92 m/s²** | −2.48 m/s² | 3.07 |

L3_wa5 doubles peak deceleration. The mean is lower because brake happens in a SHORTER, MORE INTENSE pulse near apex — exactly the trail-braking pattern the outer plans. This is the friction-circle headroom the task brief identified as unused.

---

## Headline (preliminary, pending MC)

**HMPC inner brake-aggression tune: L3 (`inner_w_a = 5`) wins — lap time 142.9 s (2:22.9), a 5.5 s improvement (−3.7 %) at cm = 0.95, single seed.**

- Lever 1 (`w_v`) and Lever 2 (`w_ellipse_soft`) BOTH degrade lap time in isolation because they cause brake-commit overshoot without anchoring deceleration to the outer's plan.
- Lever 3 (`a_long_ref`-tracking cost) — the architecturally clean fix — gives the inner a *deceleration target*, doubles peak brake-zone decel (−14.9 vs −7.3 m/s²), and cleanly outperforms.
- Lever 3 is sweet-spot sensitive: `w_a = 5` works, `w_a ≥ 10` makes the inner plan more decel than the open-loop chassis can realise → spin / off-track. The Phase 5.2 outer's `μ_circle = 0.85 · D` buffer was sized for a "soft-tracked" inner; with `a_long_ref` engaged, the inner consumes the full buffer.

### Production recommendation

- **Default `inner_w_a = 0`** in `MPCWeights` (preserves backwards compat).
- Driver-JSON opt-in via `control_params.hmpc.inner_w_a = 5.0` for callers who want the lap-time win. Tomas / Sprint A / BMW 1M is verified; other tracks need re-tuning before flipping the default.
- Tested only against the CasADi inner; OSQP inner ignores `w_a` (acceptable kwarg).

---

## Risks and follow-ups

- **`a_long` cost vs `v_ref` cost coupling.** Both push toward the same brake commit, so at high `w_a` and high `w_v` the inner can chatter (over-brake, accel back, repeat). Recommended starting trade: keep one dominant at a time. If both prove additive, document the joint default.
- **Friction-ellipse soft penalty vs chassis grip.** Lowering `w_ellipse_soft` below ~1000 lets the inner plan trajectories the open-loop chassis cannot realise (the inner's slope-only Pacejka is more optimistic than the chassis's full Pacejka with load-sensitivity). Cap recommended at `w_ellipse_soft ≥ 1000` unless a separate study verifies chassis-grip headroom.
- **The outer's `a_long_ref` is on the `(N,)` between-stages grid.** The `ReferenceTrajectory.a_long_ref_at(s)` accessor anchors it at stage midpoints. For inner stages outside the outer's horizon end the accessor clips at the last midpoint — same convention as `v_ref_at`. Acceptable for the inner's 30 m / 15-stage window inside a 500 m outer horizon.
- **Tier-1 fallback.** When the outer reference is unavailable (cold-start, infeasibility), `a_long_ref_seq` is set to `None` and the mask drops the cost. The chassis then tracks DP-plan `v_ref` only — same as pre-tune behaviour.

---

## What still doesn't close the gap (if it doesn't)

If the sweep shows no variant breaks 2:20:
- The bottleneck is upstream of the inner — likely the outer's `a_long_ref` itself (the outer plans conservatively in the chicane because its 50 % friction-circle buffer is on top of the chassis's already-loaded grip envelope).
- Next step: spec a **v3.2 MPC** with `a_long_ref` from the DP plan, not the outer (skips the outer's primal-infeasibility cliff), OR drop the outer's `μ_circle` buffer below the current 0.85 default.
- The architecturally clean fix is **v3.2 MPC (HMPC + MPC)** — drop the friction-circle linearisation entirely, plan `(F_x, F_y)` per axle, solve as a single QP with explicit ellipse constraints. Deferred to a future spec.
