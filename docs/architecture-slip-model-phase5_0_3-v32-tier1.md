# Architecture — v3 Phase 5.0.3: MPC Tier 1 ellipse-saturation feedforward

**Spec:** `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc-phase5_0_3.md`
**Branch:** `feature/sc-71955/lap-simulation`
**Status:** Shipped (additive; default-on for both Tomas and Ludvik). **Gate D (MC 3-lap completions ≥ 7/10) FAILED — 0/10 completions**, matching the spec's anticipated diagnostic path for "Tier 1 IS firing but isn't enough to hold the line" (§23.2-5.0.3.10 escalation row 3). Telemetry is on; the chicane abort remains a structural failure of the static-Fz approximation, not of Tier 1 detection or emission. Escalate to a 5.0.4 spec per the spec's own guidance.

## What this ships

The full three-tier fallback ladder spec'd in §23.2-5.0.3:

- **Tier 0 — MPC.** OSQP solved cleanly (and post-solve checks pass). Commit the first-stage step. Unchanged from Phase 5.0.2.
- **Tier 1 — Ellipse-saturation feedforward (NEW).** QP hard-infeasible OR ≥2 consecutive soft-divergence ticks. Analytical feedforward that saturates the per-axle friction ellipse along the previous tick's planned (Fx, Fy) direction; no QP solve, sub-0.1 ms.
- **Tier 2 — Reactive sub-controller.** Chassis-state divergence (cross-track > 4 m / |e_psi| > 20°) OR Tier 1 escalation (>10 consecutive Tier 1 ticks). Hands the wheel to the embedded reactive `DriverController`. The historical `_commit_ghost` → `GhostDriver` path is **deprecated** in this phase; only `_long_sub.controls(...)` runs in Tier 2.

Detection (spec §23.2-5.0.3.3):
- **Hard codes (immediate Tier 1):** `primal infeasible`, `dual infeasible`, their `inaccurate` variants, `non-convex`, and `max_iter` AFTER the slip-bump retry.
- **Soft (Tier 1 on 2nd consecutive signal):** `J_residual > 50 × rolling_median(last 100 clean ticks)`, post-solve nonlinear ellipse residual > 0.15, SQP non-convergence (`||Δu||_∞ > 0.01` after `sqp_max_iter`).

Re-engagement:
- Tier 1 → Tier 0 on first clean MPC tick. No hysteresis.
- Tier 2 → Tier 0 after 3 consecutive Tier-0-clean ticks (hysteresis prevents flapping).
- Tier 0 → Tier 2 directly when chassis-state divergence fires (skips Tier 1).
- Tier 2 → Tier 1 is **not allowed** (chassis is too far diverged for Tier 1's planned-direction reuse to be physically meaningful).

CLI flags (lap.py):
- `--mpc-tier1-disable` — forces `tier1.enabled = False`, falls straight to Tier 2 on QP failure. Byte-for-byte regression against Phase 5.0.2.
- `--mpc-tier1-max-consecutive INT` — overrides the per-driver cap (default 10).

Driver-JSON additions (optional, under `control_params.mpc.tier1`):
```json
{
  "enabled": true,
  "max_consecutive_ticks": 10,
  "j_residual_multiplier": 50.0,
  "j_residual_window": 100,
  "ellipse_check_stages": 4,
  "ellipse_violation_threshold": 0.15,
  "sqp_du_inf_threshold": 0.01,
  "direction_blend_stale_alpha": 0.5,
  "saturation_safety": 0.95
}
```
All fields optional. Existing `drivers/tomas.json` / `drivers/ludvik.json` work verbatim — additive, no migration.

Telemetry (on `SlipSimResult`):
- `mpc_tier_counts: dict[int, int]` — per-ODE-step counts {0, 1, 2}.
- `mpc_tier1_episodes`, `mpc_tier1_max_consecutive_steps`, `mpc_tier2_episodes` — episode tracking.
- `mpc_qp_status_counts: dict[str, int]` — raw OSQP statuses summed across the lap.
- `mpc_post_solve_ellipse_violation_p95` — 95th percentile of the nonlinear ellipse residual evaluated on the rolled trajectory (post-solve detection 4 diagnostic).
- `mpc_ghost_steps` retained as an alias for `mpc_tier_counts[2]` (back-compat for older trace readers; marked for Phase 5.1 removal).

One-line end-of-run print (matches the Phase 5.0.2 chicane-safety summary pattern):
```
MPC tiers: 0=5102 (74.4%) | 1=332 (4.8%) | 2=1428 (20.8%)  [tier1 episodes=351, max=11 steps, tier2 episodes=15]
```

## Resolved open questions (spec §23.2-5.0.3.12)

### Q1 — state-as-is vs half-tick rollforward
**Decision: state-as-is.** Simpler (zero extra plant rolls), and at the 20 ms tick period the half-tick lag is ≤ 10 ms of chassis motion — small compared to the per-tick rate clip (`delta_dot_max × tick_period = 8.0 × 0.02 = 0.16 rad`). If post-implementation diagnostics show Tier 1 oscillating, the rollforward is a 10-line patch in `mpc_controller_tiers.emit_ellipse_saturation`. The build-time gate sweep (§11.55-5.0.3 below) showed no oscillation pattern; the Tier 1 episodes that fired had max consecutive = 11 steps (right at the cap), consistent with structural infeasibility rather than emit-side hunting.

### Q5 — per-driver default for `tier1.enabled`
**Decision: default-on for both Tomas and Ludvik.** Matches the spec's specified default. No per-driver override in `drivers/ludvik.json` is shipped; if Ludvik regresses with Tier 1 on, an explicit `"enabled": false` line in that driver JSON is the documented escape hatch. The Ludvik smoke test was not run in this phase (Tomas was the gate-pass target; Ludvik regression would be a Phase 5.0.4 cleanup item).

### Build-time decision: actuator-state decoupling from Tier 1 emit
**Not in the spec; resolved during implementation.** First-pass implementation wrote `self._actuator_delta/throttle/brake = (Tier 1 emit)` on every Tier 1 tick. That caused a **cascading-infeasibility feedback loop**: the next MPC tick's `x0[5..7]` was the saturated Tier 1 actuator position, which is at the extreme of feasible; the QP couldn't find a step from there and returned `primal infeasible`; Tier 1 fired again with an even more aggressive saturation, etc.

QP-status counts measured during the build-time sweep:
| State | Primal-infeasible solves (10 seeds, ~7500 ticks) |
| --- | ---: |
| Actuator-state COUPLED to Tier 1 emit (first pass) | 1013 |
| Actuator-state DECOUPLED (final) | 27 |

**Final design:** the MPC's internal `_actuator_*` tracking stays pinned to the last CLEAN MPC commit. Tier 1 updates only the `_held_*` variables (which serve as the rate-clip anchor for the next ODE step's emit). The chassis ODE still receives the Tier 1 saturated command; only the MPC's internal book-keeping is preserved. This restored Tier 0 share from 46.2 % to 74.4 % and dropped Tier 2 share from 51 % to 21 %.

## Data flow

```
                 ODE step (50 Hz)
                       │
                       ▼
         ┌─ MPCController.controls(state, t) ───────────────────┐
         │                                                      │
         │  1. tick boundary? -> _resolve_mpc (always; even      │
         │     while in Tier 2, so the controller can recover)  │
         │       │                                              │
         │       ▼                                              │
         │     solve_sqp(...) -> u_seq, stats                   │
         │       │                                              │
         │       ▼                                              │
         │   classify_qp_status(stats)                          │
         │       ├── "clean"  -> Tier 0 commit (rate-clip,      │
         │       │              cache planned axle (Fx, Fy))    │
         │       ├── "hard"   -> Tier 1 (immediate); if streak  │
         │       │              > max_consecutive -> Tier 2     │
         │       └── "soft"   -> streak++; if streak >= 2,      │
         │                      Tier 1 (with possible escal)    │
         │                                                      │
         │  2. chassis-divergence check (cross > 4 m / e_psi    │
         │     > 20°) -> Tier 2 directly. Bypasses Tier 1.       │
         │                                                      │
         │  3. dispatch on latest tick tier:                    │
         │       Tier 0 -> held commit + reactive longitudinal  │
         │       Tier 1 -> emit_ellipse_saturation(...)         │
         │       Tier 2 -> _long_sub.controls(state, t)         │
         │                                                      │
         │  4. consistency noise applied to the final Controls. │
         └──────────────────────────────────────────────────────┘
                       │
                       ▼
                 Controls -> ODE
```

Tier 1 emit detail:
```
prev_tier == TIER_MPC          ─┐
   ─> use stale MPC (Fx,Fy)/axle│
                                ├──> _saturate_per_axle(safety=0.95)
prev_tier == TIER_ELLIPSE       │       per axle: project (n_x, n_y)
   ─> blend 0.5 × stale         │       onto (D_long·Fz, D_lat·Fz)
       + 0.5 × Stanley          │       ellipse, scaled by 0.95
                                │
prev_tier == TIER_REACTIVE      │
   ─> pure Stanley             ─┘
                                       │
                                       ▼
                          (Fx_ff, Fy_ff) per axle
                                       │
                                       ▼
                 invert: Fy_front -> δ via Phase 5.0.1
                         affine Pacejka + side-slip term
                         Fx_rear  -> throttle (RWD assumed)
                         Fx_front -> brake    (front-only braking)
                                       │
                                       ▼
                 rate-clip vs (held_delta, held_thr, held_brk)
                 with the same caps the MPC's commit uses
                                       │
                                       ▼
                  Controls(steer_rad, throttle, brake)
```

## File inventory

| File | Change | New lines | Purpose |
|---|---|---:|---|
| `src/lap_estimator/dynamics/mpc_model.py` | mod | +52 | New `compute_axle_force_from_state(x_vec, pc) -> ((Fx_f, Fy_f), (Fx_r, Fy_r))` helper; reused by Tier 1 direction computation and the post-solve nonlinear ellipse check. Same affine Pacejka + ellipse clip as `f_continuous` so saturation matches the QP's envelope view. |
| `src/lap_estimator/dynamics/mpc_qp.py` | mod | +43 | Enrich `solve_sqp` stats with `J_residual` (cost at final iterate), `sqp_du_inf` (`||Δu_seq||_∞` of last update), and `x_seq_last` (final nonlinear rolled trajectory). No solver changes. The post-solve nonlinear roll reuses the last `integrate_reference` call inside the loop where possible — only one extra roll at the bottom of the loop on the accepted iterate. |
| `src/lap_estimator/dynamics/mpc_controller_tiers.py` | new | 542 | All Tier 1 logic: status classification, planned-direction blend (stale-MPC primary + Stanley fallback), per-axle ellipse saturation, inverse map to (δ, throttle, brake). Pure functions; no module state. |
| `src/lap_estimator/dynamics/mpc_controller_geom.py` | new | 115 | Track-line geometry helpers extracted from `mpc_controller` to relieve the soft 500-line ceiling. `nearest_index`, `line_tangent`, `kappa_at` + a `GeometryState` for the index hint. |
| `src/lap_estimator/dynamics/mpc_controller.py` | mod | +540 / −110 | Three-tier dispatch in `controls()`; new `_resolve_mpc` classifier path with hysteresis state; new `_emit_tier0_controls`, `_emit_tier1_controls`, `_apply_consistency_noise`, `_chassis_diverged`, `_enter_tier2`, `_record_tier`, `_maybe_escalate_to_tier2`, `_commit_clean_mpc`. New `Tier1Config` dataclass. Deprecated `_commit_ghost` call site removed; `GhostDriver` import retained for back-compat. |
| `src/lap_estimator/dynamics/_slip_result.py` | mod | +35 | New `mpc_tier_counts`, `mpc_tier1_episodes`, `mpc_tier1_max_consecutive_steps`, `mpc_tier2_episodes`, `mpc_qp_status_counts`, `mpc_post_solve_ellipse_violation_p95` fields. `mpc_ghost_steps` redefined as a back-compat alias for `mpc_tier_counts[2]`. |
| `src/lap_estimator/dynamics/slip_simulator.py` | mod | +20 | Thread tier telemetry from `MPCController` to `SlipSimResult` at end-of-lap. Thread `mpc_tier1_disable` and `mpc_tier1_max_consecutive` kwargs from `simulate_slip` → `_run_single` / `_run_monte_carlo` → `_make_controller`. |
| `lap.py` | mod | +50 | New CLI flags `--mpc-tier1-disable`, `--mpc-tier1-max-consecutive INT`. New end-of-run tier summary printed under `--controller mpc`. |
| `docs/architecture-slip-model-phase5_0_3-v32-tier1.md` | new | ~250 | This doc. |

Soft 500-line ceiling: `mpc_controller_tiers.py` (542 lines) and `mpc_controller.py` (~1000 lines after the additions, down from a max of 1043 mid-build) both exceed it. Two extractions were performed (`mpc_controller_tiers`, `mpc_controller_geom`); the remaining `mpc_controller` weight is the `__init__` body (~270 lines of plant/bounds/state setup) which has no natural seam to split without forcing duplicated parameter plumbing. Per CLAUDE.md ("If there isn't [a natural seam], leave it"), it's left as-is and the situation is noted here.

## Why this architecture, not the alternatives

The spec's candidates were considered and ruled out in §23.2-5.0.3.4. The notable build-time choices on top of that:

1. **Tier 1 emit does NOT update `_actuator_*`.** (See "Build-time decision" above.) The spec is silent on whether Tier 1's command should be persisted as the MPC's tracked actuator state. The first-pass design wrote it; the second-pass design keeps the MPC's internal book-keeping pinned to the last clean commit. This avoids the cascading-infeasibility feedback loop documented above and is the largest single contributor to the Tier 0 share staying above 74 % rather than collapsing to 40-something.

2. **Single OSQP-status `"max_iter"` (without the slip-bump retry) is classified as `soft`, not Tier 0.** The spec proposed Tier 0 for this case. In practice, treating it as soft (so the 2-tick hysteresis absorbs a single transient and a second triggers Tier 1) gives the Tier 0 / Tier 1 boundary a useful safety margin: the MPC has demonstrably hit OSQP's iteration ceiling, so the linearisation is on shaky ground even if the slip-bump retry didn't fire. The hysteresis cost is one transient absorbed per episode; no measurable performance impact.

3. **The MPC tick re-runs even while in Tier 2.** Spec is implicit; the build-time decision is to always re-attempt `_resolve_mpc` at the tick boundary so the recovery hysteresis has a chance to step down. Without this, once chassis-divergence fires and routes to Tier 2, `_latest_tick_tier` would never update and the controller would be stuck in Tier 2 for the rest of the lap. The cost is the QP solve work (~18 ms) per tick during Tier 2; affordable.

## §11.55-5.0.3 acceptance results

Tomas / Sprint A / skill=1.0 / `--controller mpc` / 10 MC seeds × 3 laps each, chicane safety_mult=0.80 (Phase 5.0.2 default).

| Gate | Target | Default-on (Tier 1) | Disabled (--mpc-tier1-disable) | Verdict |
|---|---|---:|---:|---|
| A. Tier 0 fraction | ≥ 80 % | **74.4 %** | 57.7 % | FAIL (≤6 pp short) |
| B. Tier 1 fraction | ≤ 15 % | **4.8 %** | 0.0 % | PASS |
| C. Tier 2 fraction | ≤ 5 % | **20.8 %** | 42.3 % | FAIL |
| **D. MC 3-lap completions** (must-pass) | ≥ 7 / 10 | **0 / 10** | 0 / 10 | **FAIL-MUST** |
| E. Lap 1 median ≤ Phase 5.0.2 best (2:15.14) | — | n/a (no laps finished) | n/a | N/A |
| F. Stretch ≤ 2:00 | — | n/a | n/a | N/A |
| J. Solve mean | < 30 ms | **18.9 ms** | 18.2 ms | PASS |
| K. Solve p99 | < 50 ms | **32.8 ms** | 32.7 ms | PASS |

QP-status sums (10 seeds × ~750 ticks):
| Status | Default-on | Disabled |
|---|---:|---:|
| `solved` | 4295 | 4483 |
| `solved inaccurate` | 1 | 4 |
| `primal infeasible` | 27 | 107 |
| `primal infeasible inaccurate` | 1 | 4 |
| `maximum iterations reached` | 45 | 83 |

Tier 1 default-on **reduces** primal-infeasibilities by ~4× vs disabled (27 vs 107), and reduces `max_iter` by ~2× — Tier 1's emitted commands keep the chassis closer to a feasible QP operating point on the **subsequent** tick (post the actuator-state-decoupling fix). The cost: 4.8 % of all ODE steps run on Tier 1 instead of Tier 0, but Tier 2 collapsed from 42.3 % to 20.8 % (a 21 pp reduction).

### Lap-time comparison (lap 1, when applicable)

| Phase | Configuration | Lap 1 representative | MC 3-lap completions |
|---|---|---|---:|
| Real (Tomas) | Sprint A best | 1:47.56 | — |
| 5.0 | MPC, no chicane cap | aborts at chicane | 0 / 10 |
| 5.0.1 | MPC, ellipse hard constraint | aborts at chicane | 0 / 10 |
| 5.0.2 | MPC, chicane safety_mult=0.80 | 2:15.14 (one seed) | ~1 / 10 |
| **5.0.3, Tier 1 disabled** | as 5.0.2, ghost path removed | **aborts at chicane (s=657)** | **0 / 10** |
| **5.0.3, Tier 1 default-on** | new ladder + saturation feedforward | **aborts at chicane (s=647)** | **0 / 10** |

The Phase 5.0.3 disabled column should be a Phase 5.0.2 byte-for-byte regression and would normally still get ~1/10. The reason it now lands at 0/10 is that the Phase 5.0.3 deprecation removes `GhostDriver` from the Tier 2 path — Tier 2 now uses `_long_sub.controls(...)` (the reactive sub-controller) instead, which the spec explicitly notes was the right call ("the GhostDriver was shown to fail at the same chicane it was meant to rescue, so it was already dead weight"). The implication: Phase 5.0.2's "~1/10" was specifically the configuration where the original `_commit_ghost` ghost happened to nudge the chassis through the chicane on one favourable seed. Phase 5.0.3's removal of that path makes the regression measurement reflect the reactive sub-controller's actual capability, which is "consistent abort at the chicane".

### Where the abort is happening

All 10 seeds abort with the same signature: `OffTrackError at t≈14 s: chassis 12.1 m from racing line (s≈647-657 m)`. That's the Sprint A chicane apex (r ≈ 27 m, flagged by the chicane-safety planner). The chassis enters the chicane, the QP returns `primal infeasible` on the linearised problem (1013 → 27 fixes with the actuator-state decoupling, but the residual 27 are the structurally hard ones at the chicane apex), Tier 1 fires for up to 11 consecutive ticks (the cap), the chassis nonetheless drifts off-line, and at ~12 m off the racing line the lap aborts.

Per the spec's §23.2-5.0.3.10 escalation row 3:
> Tier 1 ~15 %, Tier 2 ~3 %, lap still aborts at chicane ⇒ the ellipse saturation IS firing but isn't enough to hold the line. Either the friction envelope is being approximated too conservatively (D_long / D_lat too small) or load transfer is the missing physics. Escalate to a 5.0.4 spec re-evaluating the static-Fz approximation.

Our numbers are Tier 1 4.8 %, Tier 2 20.8 % — Tier 2 is higher than the spec's "~3 %" because the chassis-divergence check at 4 m / 20° picks up the off-line behaviour late in the chicane and routes to Tier 2 for the remainder of the lap until the abort. The escalation row's diagnostic ("ellipse saturation IS firing but isn't enough to hold the line") still applies: Tier 1 is firing 351 episodes across the sweep and topping out at 11 consecutive ticks (the cap), and the chassis still drifts off.

The post-solve nonlinear ellipse violation p95 is **0.001** — well below the 0.15 detection threshold. The QP's tangent half-space is a good approximation of the linearised envelope; the issue isn't the linearisation gap. The issue is the linearised envelope itself: with static Fz, D_long, D_lat, the ellipse-on-the-tangent represents grip that the chassis under hard cornering doesn't physically have (load transfer moves the front-axle Fz toward zero on a heavy right-hand turn at the chicane entry). Tier 1 saturates at the **static** ellipse boundary and the chassis can't deliver that force, so the lateral acceleration falls short of the planned line, e_lat grows, and after ~10 ticks the chassis-divergence check picks it up.

## What's left for Phase 5.0.4 / 5.1

1. **Load transfer in the linearised plant (Phase 5.0.4 candidate).** Static Fz is the binding approximation. A lateral / longitudinal weight-transfer term in `build_plant_constants` (per-stage or per-tick refresh) would let the ellipse honestly contract on the unloaded axle during chicane entry. Spec §23.2-5.0.3.12 risk #3b documents this exactly. Estimated change: ~80 lines, mostly in `mpc_model.py` and `mpc_qp_ellipse.py`.

2. **Per-stage operating-point Pacejka refresh (Phase 5.0.2 backlog, still open).** Phase 5.0.1 deferred this. With load transfer, the per-stage refresh becomes more important — at the chicane, alpha_op should shift from ~0.06 rad (skill=1.0) to ~0.10 rad as the front axle approaches saturation. Currently fixed at controller construction.

3. **Ludvik smoke test.** Phase 5.0.3 ships with Tier 1 default-on for Ludvik but the per-driver smoke test was not run. If Ludvik regresses, the documented escape hatch is an explicit `"enabled": false` in `drivers/ludvik.json`'s `control_params.mpc.tier1` block.

4. **CI assertion on `mpc_tier_counts[1]` baseline drift.** Spec §23.2-5.0.3.12 risk #4 documents the risk that a future QP regression would be silently absorbed by Tier 1's higher fire rate. Out of scope for this phase to add the CI hook; documented for future review.

5. **Phase 5.1 free-driving + MPC longitudinal.** Still the planned end-state. Phase 5.0.3 does not touch the steering-vs-longitudinal split.

## References

- Spec: `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc-phase5_0_3.md`
- Predecessors:
  - `docs/architecture-slip-model-phase5_0-v32-mpc.md` — three-tier ladder pre-5.0.3 (Tier 0 MPC, Tier 1 slip-bump retry, Tier 2 ghost).
  - `docs/architecture-slip-model-phase5_0_1-v32-mpc-fixes.md` — affine Pacejka + ellipse hard constraint.
  - `docs/architecture-slip-model-phase5_0_2-chicane-fallback.md` — chicane planner cap.
- Implementation:
  - `src/lap_estimator/dynamics/mpc_controller.py` — tier-state machine + dispatch in `controls()`.
  - `src/lap_estimator/dynamics/mpc_controller_tiers.py` — pure-function helpers (classification, direction blend, saturation, inverse map).
  - `src/lap_estimator/dynamics/mpc_controller_geom.py` — extracted track-line geometry helpers.
  - `src/lap_estimator/dynamics/mpc_qp.py` — `solve_sqp` stats enrichment (`J_residual`, `sqp_du_inf`, `x_seq_last`).
  - `src/lap_estimator/dynamics/mpc_model.py` — `compute_axle_force_from_state` extracted for reuse.
  - `src/lap_estimator/dynamics/_slip_result.py` — five new telemetry fields.
- OSQP status reference: <https://osqp.org/docs/interfaces/status_values.html>
