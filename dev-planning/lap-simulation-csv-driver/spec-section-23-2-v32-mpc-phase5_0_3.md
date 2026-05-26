# Spec §23.2 — v3.2 Phase 5.0.3 — QP-divergence detection + ellipse-saturation fallback

**Parent spec:** `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc.md` (§23.2)
**Predecessor specs:**
- `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc-phase5_0_1.md` (Phase 5.0.1 — operating-point Pacejka + ellipse hard constraint)
- Phase 5.0.2 spec brief lives in-conversation; architecture in `docs/architecture-slip-model-phase5_0_2-chicane-fallback.md`
**Architecture (post-implementation, ArchDev to author):** `docs/architecture-slip-model-phase5_0_3-v32-mpc-ellipse-saturation.md`
**Status:** Draft (additive; controller-only patch, does not replace Phase 5.0.1 / 5.0.2)
**Project:** LapTimeEstimator
**Branch:** `feature/sc-71955/lap-simulation`
**Created:** 2026-05-22
**Planned with:** Buddy

---

## §23.2-5.0.3.1 Why Phase 5.0.3 exists

Phase 5.0.2 (chicane planner-side `v_max` cap) closed the named Sprint A chicane abort for the reactive controller and delivered the first MPC lap-completion at the chicane: 2:15.14 (vs Phase 5.0 best 2:23.70, Phase 5.0.1 best ~2:29.6). Real Tomas lap is 1:47.56. The headline numbers from Phase 5.0.2 (`architecture-slip-model-phase5_0_2-chicane-fallback.md`):

| Configuration | Lap 1 (representative) | Abort point | MC 3-lap completions |
|---|---|---|---:|
| Reactive, no cap | aborts | s ≈ 658 m (chicane apex) | 0 / 10 |
| Reactive, default cap (0.80) | aborts at downstream corner | s ≈ 1087 m (lateral-tracking bug) | 0 / 10 |
| MPC, cap 0.80, 1 lap | **2:15.14** | finished | **~1 / 10** |
| MPC, cap 0.80, 3 laps | aborts on lap 2 | s ≈ 599–660 m (`infeasible QP after slip bump`) | 0 / 10 |
| MPC, cap 0.60, 3 laps | aborts on lap 2 | s ≈ 599 m | 0 / 10 |

The diagnosis is structural: the OSQP inner QP returns infeasible at the chicane regardless of the planner-side cap. Verified across `safety_mult ∈ [0.60, 1.0]` and `ramp_segments ∈ [5, 15]`. The friction-ellipse-proxy hard constraint and the slip-budget soft constraint cannot both be satisfied at the chicane linearisation operating point — the QP returns `OSQP_PRIMAL_INFEASIBLE`, the tier-1 `w_slip × 5` re-solve fails for the same structural reason, and the existing tier-2 fallback (`_commit_ghost` in `mpc_controller.py:570`) hands off to the embedded `GhostDriver`, which has its own chicane lateral-tracking failure. Net: when the QP fails, the controller falls off the line and aborts a few hundred ms later.

Phase 5.0.3 inserts a **new tier between MPC and the existing ghost-fallback**: an ellipse-saturation feedforward action that emits the largest possible (Fx, Fy) the friction envelope admits, directed along the planned input direction. The intuition: when the QP says "I can't simultaneously honour the slip soft-constraint AND the ellipse hard-constraint", the right behaviour is *not* "abdicate to a Stanley tracker"; it is "deliver the maximum-grip control the tyre can produce in the direction the previous solution wanted to go." That is a feedforward computation, not an optimisation. It costs <0.1 ms.

Phase 5.0.2 already proved the chicane is *physically* survivable (MPC completed a full lap once). The QP infeasibility is a numerical/structural artefact of the linearisation, not a tyre-envelope reality. The ellipse-saturation tier replaces the bail-to-ghost path during these transient infeasibilities, restoring continuity in steering and pedals while the next MPC tick re-linearises and (usually) recovers.

---

## §23.2-5.0.3.2 Scope

In scope:

- §23.2-5.0.3.3 — QP-divergence detection (status codes + numerical-residual soft check).
- §23.2-5.0.3.4 — Ellipse-saturation feedforward fallback action (Tier 1).
- §23.2-5.0.3.5 — Three-tier ladder definition (Tier 0 MPC, Tier 1 ellipse-saturation, Tier 2 reactive/ghost).
- §23.2-5.0.3.6 — Re-engagement rules between tiers.
- §23.2-5.0.3.7 — Telemetry surfaces on `_slip_result.SlipSimulationResult`.
- §23.2-5.0.3.8 — Driver-JSON shape additions (`control_params.mpc.tier1`).
- §23.2-5.0.3.9 — CLI flags.
- §23.2-5.0.3.10 — Acceptance gates (§11.55-5.0.3) with concrete numbers.
- §23.2-5.0.3.11 — Implementation surface (files & rough sizing).
- §23.2-5.0.3.12 — Risks and open questions.

Out of scope:

- Re-tuning Phase 5.0.1's slope-only-vs-affine Pacejka linearisation. Deferred to a hypothetical 5.0.4 if this phase misses.
- Per-stage operating-point linearisation refresh (still the Phase 5.0.2 backlog item from §23.2-5.0.1.3 alternative row 3).
- The s≈1090 m reactive-controller lateral-tracking failure (separate root cause; see Phase 5.0.2 arch doc "Future work" item 2).
- Free-driving / racing-line selection. Still Phase 5.1+.
- Web UI exposure of new tier counters. Phase 5.2+.
- Pure-MPC longitudinal. Still consumed via the reactive sub-controller per Phase 5.0.1 §23.2-5.0.1.2.

---

## §23.2-5.0.3.3 QP-divergence detection

### Detection conditions (per MPC tick)

After `mpc_qp.solve_sqp(...)` returns, evaluate the following classifier on the returned `stats` dict. The order matters: hard failures first, soft-failures next, success last.

**Hard infeasibility** (Tier 1 triggers immediately):

1. The last entry of `stats["status_history"]` is in the set:
   - `"primal infeasible"` (OSQP code `OSQP_PRIMAL_INFEASIBLE`).
   - `"dual infeasible"` (OSQP code `OSQP_DUAL_INFEASIBLE`).
   - `"primal infeasible inaccurate"`.
   - `"dual infeasible inaccurate"`.
   - `"non-convex"` (OSQP code `OSQP_NON_CVX`; should not occur with a convex problem, but guard for it).

2. The QP returned `"max_iter"` without `"solved"` *and* the tier-1 `w_slip × 5` re-solve in `solve_sqp` was attempted (`stats["infeasible_recovery"] is True`) and still did not produce a `"solved"` status on the second try.

   - Rationale: a single `"max_iter"` without recovery is OSQP saying "I needed more time"; we treat it as soft (see below). `max_iter` after a slip-bump retry indicates the problem is structurally hard, not just slow.

**Soft divergence** (Tier 1 triggers on the *next* consecutive occurrence — see §23.2-5.0.3.6 hysteresis):

3. The last entry of `stats["status_history"]` is `"solved inaccurate"` and the SQP outer loop's terminal cost-function residual exceeds:
   - `J_residual > J_residual_threshold = 50.0 × J_residual_baseline`
   - where `J_residual_baseline` is the median residual over the last 100 ticks' `"solved"` calls (initialised to 1.0 for the first 100 ticks; the threshold is effectively wide-open during warm-up).

4. The QP returned `"solved"` cleanly, but a post-solve plant-rollout check shows constraint-violation residuals that the QP didn't catch:
   - Re-evaluate the per-stage ellipse residual `g_k_a(F_x, F_y) = (F_x / (D_long·Fz))² + (F_y / (D_lat·Fz))² − 1` after rolling the planned u_seq through the *nonlinear* plant for k = 0..3 (first 4 stages — close enough to the commit horizon to matter, cheap enough to compute).
   - If `max_k_a g_k_a > 0.15` (15 % above the ellipse), classify as soft divergence.
   - Rationale: the QP can return "solved" on the *linearised* problem while the *true* tangent-half-space approximation is well outside the real ellipse; this is the §23.2-5.0.1.9 risk #2 manifesting at runtime.

5. `stats["status_history"]` is `"solved"` and the SQP outer loop hit `sqp_max_iter` (3) without u_seq change between iters dropping below `||Δu_seq||_∞ < 0.01` — i.e. the SQP did not converge. This is "the controller is chasing its own tail" and matches the Phase 5.0.1 §23.2-5.0.1.9 risk #2 detection criterion.

### Numerical thresholds (proposed)

| Threshold | Default | Source / rationale | JSON override |
|---|---:|---|---|
| `J_residual_multiplier` | `50.0` | A factor of 50× over the 100-tick rolling median is well above natural cost variance (typically 2–5× across ticks at chicane entry). | `control_params.mpc.tier1.j_residual_multiplier` |
| `J_residual_window` | `100` ticks | At 50 Hz, 2 s of history; long enough to stabilise the median through a corner, short enough to adapt across lap halves. | `control_params.mpc.tier1.j_residual_window` |
| `ellipse_post_solve_check_k_max` | `4` stages | Covers the first 4 × 4 m = 16 m of the horizon (the part that gets committed). Cheap: ~0.05 ms. | `control_params.mpc.tier1.ellipse_check_stages` |
| `ellipse_violation_threshold` | `0.15` (15 % above) | The tangent-half-space approximation error is bounded by ~10 % at typical operating points (Phase 5.0.1 §23.2-5.0.1.4); 15 % gives margin for the post-solve nonlinear evaluation. | `control_params.mpc.tier1.ellipse_violation_threshold` |
| `sqp_unconverged_du_inf_threshold` | `0.01` | Below this `||Δu_seq||_∞`, the SQP is converged. Above it after `sqp_max_iter`, the SQP is chasing its tail. | `control_params.mpc.tier1.sqp_du_inf_threshold` |

### What does not trigger Tier 1

- A `"solved"` with no SQP convergence flag and no post-solve violation: this is the nominal happy path. Tier 0.
- A `"max_iter"` on the first solve where the tier-1 `w_slip × 5` retry then returns `"solved"` or `"solved inaccurate"`: Tier 0 (the existing slip-bump retry inside `solve_sqp` absorbed the issue).
- The simulator's separate cross-track > 4 m / heading-error > 20° check in `mpc_controller.controls()` lines 382–383. That check stays — it is a "chassis-state divergence" detector independent of the QP and routes directly to Tier 2 (see §23.2-5.0.3.5). Tier 1's detection is purely QP-driven.

### Why post-solve nonlinear checks are cheap

The MPC already rolls the plant forward inside `solve_sqp` (the `integrate_reference` call at the top of each SQP iter). Reusing the *last* rolled trajectory `x_seq` for the post-solve evaluation costs zero extra plant rolls; only the per-axle force evaluation at the first 4 stages adds work. That is 8 Pacejka evaluations + 8 ellipse-residual scalars per tick — well under 0.1 ms.

---

## §23.2-5.0.3.4 Tier 1 fallback — ellipse-saturation feedforward

### What the fallback emits

A `Controls(steer_rad, throttle, brake)` tuple computed analytically (no QP solve), targeting per-axle forces (Fx_axle_ff, Fy_axle_ff) that saturate the friction envelope along the **planned direction**.

### Planned-direction definition (resolved decision)

Three candidates were weighed:

| Candidate | What it captures | Risk | Decision |
|---|---|---|---|
| (a) Previous-tick's MPC `u_seq[0]` integrated to a per-axle (Fx, Fy) | Reflects the latest *intent* of the QP, including any cross-track correction it was building | If the previous tick is the *first* infeasibility, the direction is the planned direction the QP couldn't satisfy — which is the right thing to push toward; the saturation just delivers more grip in that direction | considered |
| (b) DP plan's reference velocity tangent + cross-track-error correction (Stanley-style feedforward) | Pure plan-tracking signal, ignores chassis state | Drops any roll-into / roll-out the MPC was building; can pull the car into the line in a way that fights chassis momentum at corner entry | rejected — Stanley-equivalent, doesn't help over ghost |
| (c) Fixed 50/50 longitudinal/lateral blend at the friction limit | Direction-agnostic; always points "max effort somewhere" | Wrong on entries (need brake-heavy), wrong on exits (need throttle-heavy) | rejected — wrong physics |
| **(d) Hybrid: last MPC solution as primary; if the last MPC tick was itself Tier 1, blend toward (b) at α = 0.5** | Reuses (a) when available; degrades gracefully toward (b) when the previous tick was also a fallback (which means we don't have a fresh MPC intent) | Slight extra logic | **chosen** |

The **resolved direction** at tick `t`:

```
if last_tier == 0:               # last tick was a clean MPC solve
    n_planned_axle = unit_dir(F_x_axle_planned_last, F_y_axle_planned_last)
elif last_tier == 1:             # last tick was already a fallback
    # Blend: 0.5 × stale MPC direction + 0.5 × Stanley-style direction
    n_planned_axle = unit_dir(
        0.5 * (F_x_axle_planned_last, F_y_axle_planned_last)
      + 0.5 * (F_x_stanley, F_y_stanley)
    )
else:                            # last tick was Tier 2 (ghost)
    # No useful MPC history — go full Stanley feedforward.
    n_planned_axle = unit_dir(F_x_stanley, F_y_stanley)
```

`F_x_axle_planned_last`, `F_y_axle_planned_last` are computed by substituting the previous tick's first-stage planned state `(v_x, v_y, omega_yaw, delta, throttle, brake)` into the same affine `F_x / F_y` expressions that the QP used in `mpc_qp.py:_build_ellipse_constraints`. Per-axle: front and rear independent.

`F_x_stanley`, `F_y_stanley` are computed from the reactive sub-controller's preview tracker: take the (steer, throttle, brake) it would emit at the current state, run it through the linearised plant for one stage, read off (Fx, Fy) per axle.

### Saturation computation

For each axle a ∈ {front, rear}, given unit direction `n_a = (n_x_a, n_y_a)`:

```
# Ellipse-norm: weight by per-axle traction limits
denom_a = sqrt( (n_x_a / D_long_a)² + (n_y_a / D_lat_a)² ) / Fz_a

F_x_ff_a = n_x_a / denom_a                # absolute force, N
F_y_ff_a = n_y_a / denom_a
```

`Fz_a` is the static normal load per axle from `PlantConstants` (the MPC's existing static-Fz approximation; load transfer is not modelled here — same simplification the QP uses, so the saturation matches the QP's view).

`D_long_a`, `D_lat_a` are the Pacejka envelope coefficients exposed by `build_plant_constants` per Phase 5.0.1 §23.2-5.0.1.4 ("`mpc_model.py::build_plant_constants(...)` — gains `D_long_axle`, `D_lat_axle` as exposed constants").

This places `(F_x_ff_a, F_y_ff_a)` exactly on the ellipse `(F_x / D_long_a·Fz_a)² + (F_y / D_lat_a·Fz_a)² = 1` along the direction `n_a`. **Important**: we saturate to the ellipse, not 5 % below it — the slip-budget soft cap in §23.2-5.0.1.4 already provides skill-modulated headroom against peak. Saturating below the ellipse would be doubly conservative; saturating above would risk wheel-spin (see §23.2-5.0.3.12 risk #3).

### Converting (Fx, Fy) per axle back to (steer, throttle, brake)

Inverse-map through the same affine relations the QP uses:

1. **Lateral force → steering angle.** The Phase 5.0.1 affine Pacejka `F_y_axle = F_y_bias_axle − C'_op_axle · α_axle` inverts to:
   ```
   α_axle = (F_y_bias_axle − F_y_ff_axle) / C'_op_axle
   ```
   The bicycle slip-angle expressions (small-angle):
   ```
   α_front = δ − atan2(v_y + a_f · ω_yaw, v_x)
   α_rear  =     − atan2(v_y − a_r · ω_yaw, v_x)
   ```
   The rear gives nothing to solve for (no steering input on rear axle); use the **front** to back out `δ`:
   ```
   δ_ff = α_front_target + atan2(v_y + a_f · ω_yaw, v_x)
   ```
   Clip to `[-delta_max, delta_max]` from `MPCBounds`.

2. **Longitudinal force → throttle / brake.** The MPC's `F_x_axle` expression is affine in `(throttle, brake)`:
   ```
   F_x_front = k_brake_front · (−brake)
   F_x_rear  = k_throttle · throttle  +  k_brake_rear · (−brake)
   ```
   (BMW 1M is RWD; specific coefficients live in `mpc_model.py::build_plant_constants`.)

   For target `(F_x_front_ff, F_x_rear_ff)`, solve the 2×2 system in `(throttle, brake)`. If the solution requires both > 0 (impossible: can't brake and throttle simultaneously), prefer the dominant axle: if `F_x_rear_ff > 0`, set `brake = 0` and solve for throttle; if `F_x_rear_ff < 0`, set `throttle = 0` and solve for brake. Clip both to `[0, 1]`.

3. **Standing-start guard.** Same as Phase 5.0 (`mpc_controller.py:560`): if `v_x < 1.0 m/s` and `v_ref > 2.0 m/s`, force `throttle = 1.0`, `brake = 0.0`. Tier 1 inherits this behaviour.

### Time-smoothing — ramp from last commit to the saturation point

A snap from the last MPC commit to the saturation point would cause steering chatter on transient Tier-1 firings. The Tier-1 emitter applies a single-tick rate clip identical to the MPC's:

```
delta_step_max = delta_dot_max · tick_period     # 8.0 × (1/50) = 0.16 rad/tick
δ_emitted = clip(δ_ff, δ_last − delta_step_max, δ_last + delta_step_max)
# Same for throttle, brake.
```

This is **exactly** the rate cap already enforced in `mpc_controller.py:539–544` (`cap_delta`, `cap_thr`, `cap_brk`). No new logic; same physical rate limit. The smoothness comes for free.

### What changes about the QP after Tier 1 fires

Nothing. The next tick re-runs `solve_sqp` from scratch on the updated chassis state. The warm-start `u_seq_init` uses the *previous successful* MPC solve's u_seq (i.e. the one from before the Tier 1 episode), shifted forward by however many ticks Tier 1 ran. If the Tier 1 episode lasted > N stages, the warm-start is the all-zero default. This is no worse than a first-tick cold-start, which the MPC handles fine.

---

## §23.2-5.0.3.5 Fallback ladder — three tiers

The full controller behaviour per ODE step:

| Tier | When | Action | Where in code |
|---|---|---|---|
| **Tier 0 — MPC** | OSQP solved cleanly per §23.2-5.0.3.3 | Use MPC's first-stage commit (existing path) | `mpc_controller.controls()` falls through to the post-`_resolve_mpc` happy path |
| **Tier 1 — Ellipse-saturation feedforward** | QP hard-infeasible OR (soft-divergence ≥ 2 consecutive ticks) | Emit `(δ_ff, throttle_ff, brake_ff)` per §23.2-5.0.3.4 | New method `MPCController._emit_ellipse_saturation(state, t)` |
| **Tier 2 — Reactive sub-controller** | Tier 1 fired for N_TIER1_CONSECUTIVE_MAX ticks straight without an MPC recovery OR chassis-state divergence (cross-track > 4 m / |e_psi| > 20°) | Hand to `self._long_sub.controls(state, t)` (existing `_long_sub`) | The existing reactive-fallback at `mpc_controller.py:382` |

The chassis-state divergence check in `mpc_controller.controls()` lines 371–393 routes to **Tier 2 directly**, bypassing Tier 1. Rationale: if the chassis is already far enough off the line that the linearisation is untrustworthy, the ellipse-saturation feedforward is computing forces against a stale operating point, and the reactive Stanley tracker has at least a fighting chance to recover the line.

The existing `_commit_ghost` path (`mpc_controller.py:570`, which delegates to `GhostDriver` rather than the reactive sub-controller) is **deprecated** in this phase. Tier 2 is the reactive sub-controller (`_long_sub`). The `GhostDriver` import is kept for back-compat but the call site is removed. Rationale: in Phase 5.0.2 testing the `GhostDriver` was shown to fail at the same chicane it was meant to rescue, so it was already dead weight; the reactive sub-controller has the actual driver-tuned tracking logic.

### Tier escalation logic

State variable on `MPCController`:

```python
self._tier_history: list[int] = []     # one entry per ODE step (not per MPC tick)
self._tier1_consecutive: int = 0
self._tier1_total: int = 0
self._tier_counts = {0: 0, 1: 0, 2: 0}
```

Per ODE step:

1. If chassis-divergence check fires → Tier 2. Reset `_tier1_consecutive = 0`.
2. Else if a fresh MPC tick happened this step:
   - If QP detection (§23.2-5.0.3.3) says "infeasible" → Tier 1. Increment `_tier1_consecutive`.
   - If `_tier1_consecutive > N_TIER1_CONSECUTIVE_MAX` → escalate to Tier 2. Reset to 0.
   - Else if QP solved cleanly → Tier 0. Reset `_tier1_consecutive = 0`.
3. Else (between MPC ticks, hold path): Tier remains the same as the most recent MPC tick.

`N_TIER1_CONSECUTIVE_MAX = 10` (proposed default). At 50 Hz that's 200 ms of saturation before giving up; covers a typical chicane transient (Phase 5.0.2 measured chicane abort duration ≤ 100 ms from first infeasibility to off-track). Override via `control_params.mpc.tier1.max_consecutive_ticks`.

---

## §23.2-5.0.3.6 Re-engagement rules

### Tier 1 → Tier 0

Every MPC tick re-attempts the solve from scratch. The first tick that returns Tier-0-clean per §23.2-5.0.3.3 immediately switches back. `_tier1_consecutive` resets to 0. No hysteresis on re-entry — the QP either solved cleanly or it didn't; if it did, we trust it.

### Tier 2 → Tier 0

Same as the existing Phase 5.0 behaviour: when the chassis-divergence check returns false (cross-track ≤ 4 m AND |e_psi| ≤ 20°) and the next MPC tick produces a Tier-0-clean solve, switch back. Hysteresis prevents flapping: require **3 consecutive Tier-0 ticks** before declaring Tier 2 over. Hysteresis applies only to the Tier 2 → Tier 0 transition (not Tier 1 → Tier 0; Tier 1 doesn't need hysteresis because it doesn't have a separate "chassis is bad" signal).

### Tier 2 → Tier 1

Not allowed. Once Tier 2 takes over due to chassis-state divergence, the only exit is back to Tier 0 (with the 3-tick hysteresis). Rationale: Tier 1's feedforward uses the chassis state as input; if the chassis is diverged, Tier 1 is operating on bad data. Better to let the reactive sub-controller recover the line, then re-engage MPC.

### Tier 0 → Tier 2 directly (skipping Tier 1)

Allowed when the chassis-divergence check fires. The cross-track > 4 m / |e_psi| > 20° check is **independent** of the QP status; even if the QP is solving cleanly, a diverged chassis means the linearisation is stale and the next QP solve will produce nonsense.

### Soft-divergence hysteresis (Tier 0 → Tier 1)

Per §23.2-5.0.3.3, soft-divergence conditions (3, 4, 5) trigger Tier 1 only on **two consecutive ticks** of soft signal. A single transient soft signal is absorbed (treated as Tier 0). Two in a row escalates to Tier 1. State: `self._soft_divergence_streak: int = 0`.

| State | Action |
|---|---|
| Soft signal this tick, streak < 1 | Stay Tier 0; increment streak |
| Soft signal this tick, streak ≥ 1 | Switch to Tier 1; streak resets after 1 clean tick |
| No soft signal | Decrement streak (clamp at 0) |

Hard-infeasibility signals trigger Tier 1 immediately (no hysteresis on the way in).

---

## §23.2-5.0.3.7 Telemetry

Additions to `_slip_result.SlipSimulationResult` (existing dataclass at `src/lap_estimator/dynamics/_slip_result.py:71–97`):

```python
# Phase 5.0.3 (v3.2 MPC, spec §23.2-5.0.3.7): fallback-tier diagnostics.
mpc_tier_counts: dict[int, int] = field(default_factory=lambda: {0: 0, 1: 0, 2: 0})
"""Per-ODE-step tier counts. Tier 0 = clean MPC, Tier 1 = ellipse-saturation
feedforward, Tier 2 = reactive sub-controller. Used for §11.55-5.0.3 gates."""

mpc_tier1_episodes: int = 0
"""Number of distinct Tier-1 episodes (contiguous runs). Diagnostic; not gated."""

mpc_tier1_max_consecutive_steps: int = 0
"""Longest Tier-1 episode in steps. Diagnostic; not gated."""

mpc_tier2_episodes: int = 0
"""Number of distinct Tier-2 episodes (contiguous runs)."""

mpc_qp_status_counts: dict[str, int] = field(default_factory=dict)
"""Raw OSQP status string → count. Diagnostic; helps post-mortem classify
which infeasibility class (primal vs dual vs max_iter) dominates per lap."""

mpc_post_solve_ellipse_violation_p95: float = 0.0
"""95th percentile of max-stage post-solve ellipse residual across all clean
Tier-0 ticks. >0.15 ⇒ the tangent half-space approximation is biting and
the post-solve soft check (§23.2-5.0.3.3 detection 4) is firing often.
Diagnostic; informs whether to tighten the ellipse linearisation in 5.0.4."""
```

The existing `mpc_ghost_steps: int = 0` field stays but is **redefined** as a synonym for `mpc_tier_counts[2]`. It is filled in for back-compat; new code reads `mpc_tier_counts[2]` directly.

The existing `mpc_solve_times_s: list[float]` stays unchanged.

### Printed simulation summary (one new line)

`slip_simulator._print_summary` (or equivalent) gains one line at end-of-run:

```
MPC tiers: 0=21456 (95.2%) | 1=987 (4.4%) | 2=87 (0.4%)  [tier1 episodes=14, max=8 steps]
```

Matches the existing one-line summaries (e.g. `Chicane-safety: 316 segments flagged...`). The %ages are computed against total ODE steps in the lap. If a tier is zero, omit it from the print (`MPC tiers: 0=21456 (95.6%) | 2=987 (4.4%)`).

---

## §23.2-5.0.3.8 Driver-JSON shape additions

A new optional sub-block under `control_params.mpc`:

```json
{
  "control_params": {
    "mpc": {
      "horizon_m": 30.0,
      "n_stages": 15,
      "tick_hz": 50.0,
      "sqp_max_iter": 3,
      "w_lat": 50.0,
      "w_psi": 20.0,
      "w_v": 0.5,
      "w_slip": 200.0,
      "w_du": 1.0,
      "w_du2": 1.0,
      "w_term": 100.0,
      "tier1": {
        "enabled": true,
        "max_consecutive_ticks": 10,
        "j_residual_multiplier": 50.0,
        "j_residual_window": 100,
        "ellipse_check_stages": 4,
        "ellipse_violation_threshold": 0.15,
        "sqp_du_inf_threshold": 0.01,
        "direction_blend_stale_alpha": 0.5
      }
    }
  }
}
```

All fields under `tier1` are optional. If `tier1` is omitted entirely or `tier1.enabled` is `true`, Tier 1 is **default-on**. To disable, set `"enabled": false` in driver JSON OR pass the CLI flag (see §23.2-5.0.3.9).

Existing driver JSONs (`drivers/tomas.json`, `drivers/ludvik.json`) work verbatim — additive, no migration required. The defaults are baked into the code; the JSON block only overrides them.

Resolution precedence (matches Phase 5.0.2 chicane block):

```
CLI flag > driver JSON control_params.mpc.tier1.<field> > code default
```

---

## §23.2-5.0.3.9 CLI flags

Two flags on `lap.py` (and via `--help`):

| Flag | Type | Default | Effect |
|---|---|---|---|
| `--mpc-tier1-disable` | bool flag | unset | When set, forces `tier1.enabled = False` regardless of JSON. Useful for A/B regression measurement against Phase 5.0.2 byte-for-byte. |
| `--mpc-tier1-max-consecutive INT` | int | None (use JSON / default 10) | Overrides the per-driver `tier1.max_consecutive_ticks`. |

No flag for the thresholds (j_residual_multiplier, ellipse_violation_threshold, etc.) — those are buildtime tuning, not user-facing knobs. If a thresholdspike needs experiment, the user edits driver JSON.

Tier 1 is **default-on**. The CLI carries no positive `--mpc-tier1-enable` flag; presence is implicit. This matches the Phase 5.0.2 chicane-cap default-on pattern.

---

## §23.2-5.0.3.10 Acceptance gates — §11.55-5.0.3

Tomas on Sprint A, `--model slip --controller mpc --skill-pct 1.0`, defaults from Phase 5.0.1 (`safety_margin=0.94`, `LAP_OFFTRACK_ABORT_M=12`) + Phase 5.0.2 (`chicane safety_mult=0.80`, `radius_thresh=60`) + Phase 5.0.3 (Tier 1 default-on). All gates measured on 10-seed MC unless noted.

### Tier-distribution gates

| Gate | Target | Phase 5.0.2 measured | Rationale |
|---|---:|---:|---|
| **A. Tier 0 (clean MPC) fraction** | **≥ 80 % of ticks** | unmeasured (no tier instrumentation) | The QP should still be the primary controller; Tier 1 is a stop-gap, not the main path. <80 % ⇒ structural problem deeper than this phase fixes |
| **B. Tier 1 (ellipse-sat) fraction** | **≤ 15 % of ticks** | n/a | Bounds the "we're saturating instead of optimising" share. Mostly chicane + corner-entry transients. |
| **C. Tier 2 (reactive) fraction** | **≤ 5 % of ticks** | ~0.4 % on the one completed lap (Phase 5.0.2 used existing tier-2-only fallback) | The reactive sub-controller is the last-resort safety net. >5 % ⇒ chassis-state divergence is firing often; Tier 1 isn't catching enough. |

### Completion gate (must-pass)

| Gate | Target | Phase 5.0.2 measured | Rationale |
|---|---:|---:|---|
| **D. MC 3-lap completions** | **≥ 7 / 10 seeds** | ~1 / 10 | Inherited from Phase 5.0.1 gate B; still the headline gate. This is the actual problem we're solving. |

### Lap-time gates (no-regression)

| Gate | Target | Phase 5.0.2 measured | Rationale |
|---|---:|---:|---|
| **E. Lap 1 (median seed)** | **≤ Phase 5.0.2 best (2:15.14)** | 2:15.14 | No regression. Tier 1 should not slow the controller on laps where MPC was already completing. |
| **F. Lap 1 (median seed, stretch)** | **≤ 2:00** | n/a | Stretch goal. A Tier-1 episode at the chicane should saturate efficiently rather than over-conservatively, recovering some of the planner-cap loss. Not blocking. |

### Quality gates (should-pass)

| Gate | Target | Phase 5.0.2 measured | Rationale |
|---|---:|---:|---|
| **G. Cross-track p95** | **≤ 6 m** | unmeasured (laps aborted) | Inherits Phase 5.0.1 gate D. |
| **H. util_p85 (honest)** | **≤ 1.15** | unmeasured (laps aborted) | Inherits Phase 5.0.1 gate C. |
| **I. Skill monotonicity** | skill=0.5 ≥ 3 s slower than skill=1.0 | unmeasured | Inherited preserved. |

### Solve-time gates (observability)

| Gate | Target | Phase 5.0.2 measured | Rationale |
|---|---:|---:|---|
| **J. MPC solve time mean** | **< 30 ms** | ~20 ms | Unchanged from Phase 5.0.1 gate G. Tier 1 adds zero QP work (it's a pure computation), so this should not regress. |
| **K. MPC solve time p99** | **< 50 ms** | ~30 ms p95 | Unchanged. |

### Must-pass / should-pass split

- **Must-pass:** D (the headline). Without ≥ 7/10 completions, the phase doesn't ship.
- **Should-pass:** A, B, C, E, G. Failure of any one triggers a post-mortem but doesn't block shipping if D holds and no other should-pass gate fails.
- **Stretch:** F.
- **Observability:** H, I, J, K. Document and proceed.

### When 5.0.3 doesn't pass

- **D fails (< 7/10).** Triage by tier distribution:
  - Tier 1 < 5 % and Tier 2 > 10 % ⇒ Tier 1 is not catching the infeasibility class the chicane produces. Likely cause: the QP isn't reporting status codes Tier 1 listens to. Add the status code(s) seen in `mpc_qp_status_counts` to the detection set.
  - Tier 1 > 30 % ⇒ Tier 1 is firing too often AND not catching the chassis enough to recover. Likely cause: planned-direction is wrong (§23.2-5.0.3.12 risk #1). Re-spec the direction selection in 5.0.4.
  - Tier 1 ~15 %, Tier 2 ~3 %, lap still aborts at chicane ⇒ the ellipse saturation IS firing but isn't enough to hold the line. Either the friction envelope is being approximated too conservatively (D_long / D_lat too small) or load transfer is the missing physics. Escalate to a 5.0.4 spec re-evaluating the static-Fz approximation.
- **E fails (> 2:15.14).** Tier 1 is over-conservative and is firing in cases where the QP would have produced a faster solution. Detune `direction_blend_stale_alpha` toward 1.0 (always trust the stale MPC direction) and re-measure.

---

## §23.2-5.0.3.11 Implementation surface — files & rough sizing

| File | Action | Lines (est) | Purpose |
|---|---|---:|---|
| `src/lap_estimator/dynamics/mpc_controller.py` | modify | +180 / −30 | Add `_emit_ellipse_saturation`, `_classify_qp_status`, `_compute_post_solve_residual`; rewire `controls()` to dispatch on tier; replace `_commit_ghost` call site with `_long_sub.controls(...)` (Tier 2); add tier counters / streak state; load `tier1` JSON block in `__init__` |
| `src/lap_estimator/dynamics/mpc_qp.py` | modify | +20 / −0 | Expose richer stats: `J_residual` (post-solve cost), `sqp_du_inf` (last SQP step ∞-norm). No solver changes. |
| `src/lap_estimator/dynamics/mpc_model.py` | modify | +30 / −0 | Helper `compute_axle_force_from_state(x, u, pc)` returning `(F_x_a, F_y_a)` per axle. Currently this logic is inline in `mpc_qp_ellipse._build_ellipse_constraints`; extract for reuse by the Tier 1 feedforward direction computation. |
| `src/lap_estimator/dynamics/_slip_result.py` | modify | +25 / −0 | New `mpc_tier_counts`, `mpc_tier1_episodes`, `mpc_tier1_max_consecutive_steps`, `mpc_tier2_episodes`, `mpc_qp_status_counts`, `mpc_post_solve_ellipse_violation_p95` fields. Back-compat alias on `mpc_ghost_steps`. |
| `src/lap_estimator/dynamics/slip_simulator.py` | modify | +10 / −0 | Thread tier counters from `MPCController` to `SlipSimulationResult` at end-of-lap; print one-line tier summary. |
| `lap.py` | modify | +5 / −0 | CLI flags `--mpc-tier1-disable`, `--mpc-tier1-max-consecutive`. |
| `docs/architecture-slip-model-phase5_0_3-v32-mpc-ellipse-saturation.md` | new (ArchDev to author post-implementation) | ~250 | Architecture mirror of this spec; what actually shipped + first-lap diagnostic plots. |

No file is forecast to cross the 500-line soft cap as a result of these changes. `mpc_controller.py` (currently ~670 lines after Phase 5.0.1) crosses the cap with this addition (~820 lines). **Mitigation:** extract the new methods into a sibling `mpc_controller_tiers.py` module (~250 lines) exposing pure functions:

```
classify_qp_status(stats, history_state) -> Tier
compute_planned_direction(prev_tier, prev_u_seq, prev_x_seq, stanley_cmd) -> (n_front, n_rear)
emit_ellipse_saturation(state, n_front, n_rear, pc, last_commit) -> Controls
```

The main `MPCController` stays slim, calling out to these helpers. This matches the "many small focused files" rule from the user's global CLAUDE.md.

No new dependencies. `osqp`, `scipy.sparse`, `numpy` are already on board.

No driver JSON migrations. No track-CSV changes. No lake schema changes. No Web UI changes.

---

## §23.2-5.0.3.12 Risks and open questions

### Risks

1. **Planned-direction choice is wrong → Tier 1 pulls the car the wrong way.** Per §23.2-5.0.3.4, the default direction is the previous-tick's MPC solution. If the chicane infeasibility happens because the previous tick was *itself* wrong (e.g. the QP's first SQP iter produced a u_seq that aimed too aggressively into the apex, then later iters couldn't reconcile with the ellipse), the saturation is honouring an already-bad intent. The fallback to the stale-blended direction (§23.2-5.0.3.4 candidate (d)) is the mitigation but assumes the Stanley-style direction is at least *physically reasonable*. **Detection:** if Tier 1 fires AND cross-track grows during the Tier 1 episode (rather than stabilising or shrinking), the direction is wrong; abort and let chassis-divergence escalate to Tier 2. Adds a passive monitor on `cross-track delta during Tier 1` per-episode; bail to Tier 2 if `Δ cross-track > 1.0 m` over the Tier 1 window. **Acceptance impact:** §11.55-5.0.3 gate G (cross-track ≤ 6 m) catches the failure mode.

2. **Saturation discontinuity → steering chatter.** Snap from a moderate MPC steering command (say 0.05 rad) to a saturation command (say 0.15 rad along the planned axle's lateral component) within one tick would be a 60 deg/s rate. The single-tick rate clip (§23.2-5.0.3.4 "Time-smoothing") caps it at `delta_dot_max · tick_period = 8.0 × 0.02 = 0.16 rad`, which is precisely one MPC tick worth of motion — so the worst-case rate is by construction identical to what the MPC itself can command. **No new chatter risk**, but: when Tier 1 fires for sustained ticks (5–10 in a chicane), the cumulative steering motion can still be 0.5–1.5 rad of steer, larger than the MPC's per-tick smooth ramp. Detection: monitor `mpc_tier1_max_consecutive_steps` per lap; > 20 implies sustained saturation and the chassis was held off-line for ≥ 0.4 s. Mitigation: reduce `N_TIER1_CONSECUTIVE_MAX` from 10 to 6 if observed.

3. **The friction-ellipse linearisation is itself approximate; saturating an approximate ellipse risks (a) under-saturation (conservative — OK, just slower) or (b) over-saturation (NOT OK — wheel-spin / loss of control).** The Phase 5.0.1 tangent-half-space approximation is a *first-order* expansion about the operating point; the true ellipse is a constant-radius constraint and the tangent over-approximates outside the operating region (under-conservative). For Tier 1 we are evaluating the ellipse at the *current* operating point per axle, then placing the saturated force exactly on the linearised boundary. **Two sub-risks:**
   - **(a) Under-saturation.** If D_long, D_lat are reported tight (e.g. wet conditions, cold tyres — not currently modelled but planned), the Tier 1 ellipse is below physical grip and Tier 1 produces less force than physically available. **Acceptable**: the simulation just runs a bit slower; no instability.
   - **(b) Over-saturation (the real risk).** If the static-Fz approximation overstates load on the unloaded axle in a corner (load transfer not modelled — Phase 5.0.1 risk acknowledged), the ellipse for the unloaded axle is *bigger* than physical; Tier 1 commands a brake that the unloaded front cannot deliver, the actual longitudinal force saturates lower, the lateral side spills over and the chassis rotates uncontrollably. **Detection:** post-Tier-1-tick check on `Fy_observed / (D_lat · Fz_actual_load_transfer_estimate)` from the ODE plant; if > 1.0, log a `WheelSpinRisk` event. Mitigation: bake a **0.95 safety scalar** into the Tier 1 ellipse RHS (saturate at 0.95× the linearised limit, not 1.0×). **Resolved decision:** ship with `tier1_saturation_safety = 0.95` as the default (a new field under `control_params.mpc.tier1`); revisit if Phase 5.0.3 gates need the extra 5 %.

4. **Tier 1 hides QP regressions.** If a future change to `mpc_qp.py` introduces a subtle bug that makes the QP infeasible more often, Tier 1 absorbs the symptom and the lap still completes — but `mpc_tier_counts[1]` rises. **Mitigation:** the §11.55-5.0.3 gate B (Tier 1 ≤ 15 %) catches drift; CI should add a post-run assertion that compares `mpc_tier_counts[1]` against a stored baseline + ε. Out of scope for this spec to define the CI hook; documented for the user's future review.

5. **Tier 1's direction blend at α=0.5 is heuristic.** The 0.5 weight between stale MPC and Stanley is picked because it's the cleanest split; no measurement justifies 0.5 over 0.3 or 0.7. If Tier 1 fires for 2+ consecutive ticks (i.e. last_tier was already Tier 1), the chosen blend determines whether we lean into the MPC's last *good* intent or the planner's geometric target. **Open question (resolve during build):** does 0.5 work in practice? If post-implementation diagnostics show the blend pulls the car off the plan during chicane saturation, raise to 0.7 (lean stale MPC) or drop to 0.3 (lean Stanley). The driver-JSON field `direction_blend_stale_alpha` exists precisely so this is tunable per driver without code changes.

6. **Post-solve nonlinear ellipse check is itself susceptible to the same load-transfer approximation as the QP.** §23.2-5.0.3.3 detection 4 evaluates `g_k_a = (Fx/D_long Fz)² + (Fy/D_lat Fz)² − 1` using the *static* Fz from `PlantConstants`. If the real grip is materially different (e.g. lateral load transfer in a hard corner moves Fz by 30 %), the check is comparing planned forces against the wrong envelope. The check classifies "ellipse violated" or "ellipse honoured" with that systematic bias. **Mitigation:** keep the threshold loose (0.15, not 0.05); the check is for catastrophic mis-linearisations, not for fine-grained ellipse honouring. Acceptable as a coarse soft-divergence signal.

### Open questions (resolve during build)

- **Q1.** Should the Tier 1 feedforward use the `state` at the ODE step (where it is called) or roll forward by half a tick to predict the chassis position at the end of the tick? Phase 5.0.3 default: use `state` as-is. If post-build diagnostics show a half-tick lag is causing oscillation, add the half-tick rollout (cheap, ~10 lines).
- **Q2.** When the `j_residual_baseline` median is being initialised (first 100 ticks), should the threshold be effectively wide-open (current proposal), or should it use a conservative absolute floor (e.g. `J_residual_threshold = max(50.0 × median, 1000.0)` — never trigger soft below 1000)? Phase 5.0.3 default: wide-open during warm-up. The risk is a false positive in the first 100 ticks of a lap; acceptable because Tier 1 is benign at low cost.
- **Q3.** Should `_long_sub.controls(...)` (the Tier 2 path) also see a flag that says "you are now the active controller, not a sub-controller"? Currently the reactive sub-controller's preview / softener / consistency-noise behaviour is the same in both roles (MPC-driven longitudinal and Tier 2 lateral+longitudinal). Phase 5.0.3 default: no flag — the reactive sub-controller's behaviour is identical in both roles. Acceptable trade-off; no measurable disadvantage in Phase 5.0.2 testing.
- **Q4.** Does the back-compat `mpc_ghost_steps` field stay forever, or do we deprecate-and-remove in a future phase? Phase 5.0.3 default: stay as an alias for `mpc_tier_counts[2]`. Mark with a comment for future removal in Phase 5.1.
- **Q5.** Should the per-driver default for `tier1.enabled` differ between Tomas and Ludvik? Phase 5.0.3 default: same (default-on) for both. If Ludvik regresses with Tier 1 on, set explicitly in `drivers/ludvik.json`. (Same pattern as Phase 5.0.1 softener kill-switch.)

---

## §23.2-5.0.3.13 Decisions block update — proposed §23.M.31

Append after the §23.M.29 entry (Phase 5.0.1) and §23.M.30 entry (Phase 5.0.2 chicane safety) in the parent spec's decisions block:

> **31. (v3.2 Phase 5.0.3) MPC QP-divergence detection + ellipse-saturation feedforward fallback.** Phase 5.0.2 closed the chicane abort for the reactive controller and produced the first MPC lap-completion (2:15.14 / Sprint A), but MC 3-lap completion held at ~1/10 because the OSQP inner QP returns infeasible at the chicane regardless of planner-side `v_max` cap. Phase 5.0.3 adds a third tier between MPC (Tier 0) and the existing reactive-fallback (Tier 2): an analytical feedforward (Tier 1) that emits `(δ, throttle, brake)` saturating the per-axle friction ellipse along the previous MPC tick's planned input direction. Detection: hard OSQP infeasibility codes (`primal_infeasible`, `dual_infeasible`, their inaccurate variants, `non_convex`, `max_iter` after slip-bump retry) trigger Tier 1 immediately; soft-divergence (cost-residual > 50× rolling median, post-solve ellipse violation > 15 %, SQP non-convergence after `sqp_max_iter`) triggers Tier 1 on two consecutive signal ticks. Tier 1 saturates `(F_x_axle_ff, F_y_axle_ff)` on the per-axle Pacejka envelope `(F_x/D_long·Fz)² + (F_y/D_lat·Fz)² = 1` along the planned direction (stale-MPC primary, 0.5× Stanley blend when MPC history is stale, full Stanley when prior tier was Tier 2). Inverse-map to (δ, throttle, brake) via the Phase 5.0.1 affine Pacejka relations. Same single-tick rate clip as MPC so saturation cannot exceed `delta_dot_max · tick_period` per tick. Re-engagement: every MPC tick re-attempts solve from scratch; Tier 0 returns on the first clean solve; 10-consecutive-tick cap escalates to Tier 2 (the reactive sub-controller); the existing `_commit_ghost` ghost-driver path is deprecated. Telemetry: new `mpc_tier_counts`, `mpc_tier1_episodes`, `mpc_qp_status_counts`, `mpc_post_solve_ellipse_violation_p95` on `SlipSimulationResult`; one-line summary at end-of-lap. JSON: optional `control_params.mpc.tier1` block, default-on, all fields override-able. CLI: `--mpc-tier1-disable`, `--mpc-tier1-max-consecutive INT`. Acceptance: new §11.55-5.0.3 — must-pass D (MC 3-lap ≥ 7/10), tier-distribution A/B/C (≥80/≤15/≤5 %), no lap-time regression vs 2:15.14, stretch ≤ 2:00. No new deps; ~270 lines added across `mpc_controller.py`, `mpc_qp.py`, `mpc_model.py`, `_slip_result.py`, `slip_simulator.py`, `lap.py`, with `mpc_controller_tiers.py` extracted to keep the soft 500-line cap. (§23.2-5.0.3.1–§23.2-5.0.3.12.)

---

## §23.2-5.0.3.14 Forward pointer (update in predecessor specs)

Add to `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc-phase5_0_1.md` at the bottom of §23.2-5.0.1.10:

> **Next phase:** §23.2-5.0.3 (`spec-section-23-2-v32-mpc-phase5_0_3.md`) addresses the residual chicane QP-infeasibility (Phase 5.0.2 planner-side cap closed reactive's chicane abort but MPC's QP still goes infeasible at the chicane; Phase 5.0.3 adds an ellipse-saturation feedforward tier between MPC and the reactive sub-controller).

The Phase 5.0.2 architecture doc (`docs/architecture-slip-model-phase5_0_2-chicane-fallback.md`) "Future work" section already references "MPC structural fixes (operating-point Pacejka, ellipse hard constraint)"; the new pointer is to this 5.0.3 spec which is the controller-side complement to the planner-side 5.0.2 cap.

### Next phase (post-5.0.3)

> **Next phase:** §23.2-5.0.4 (`spec-section-23-2-v32-mpc-phase5_0_4.md`) addresses the residual chicane MC 3-lap failure that 5.0.3 did not close. Tier-1 saturation eliminated QP infeasibility (1013 → 27 per 3-lap MC) and drove post-solve ellipse violation p95 to 0.001, but the lap still aborts at s ≈ 647 m. Root cause: the MPC plant uses **static Fz**; the truth-model ODE already uses 4-wheel dynamic Fz (`solver._weight_transfer`), and the static approximation overstates grip on the unloaded axle during chicane transitions. Phase 5.0.4 adds per-stage per-axle dynamic Fz to the MPC plant + ellipse + Tier-1 saturation; gates target ≥ 7/10 MC completions on Sprint A.

---

## §23.2-5.0.3.15 References

- `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc.md` — parent §23.2 spec; this file extends the tier-fallback ladder defined in §23.2.10.
- `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc-phase5_0_1.md` — Phase 5.0.1: operating-point Pacejka + ellipse hard constraint. §23.2-5.0.1.4 defines the ellipse linearisation that Tier 1 saturates against; §23.2-5.0.1.9 risk #2 anticipated the soft-divergence path that Phase 5.0.3 instruments.
- `docs/architecture-slip-model-phase5_0-v32-mpc.md` — Phase 5.0 architecture; the three-tier fallback ladder lives here (Phase 5.0.3 inserts a new Tier 1 between the existing Tier 0 MPC and Tier 2 ghost).
- `docs/architecture-slip-model-phase5_0_1-v32-mpc-fixes.md` — Phase 5.0.1 architecture (ArchDev's algebra-error correction on the ellipse RHS).
- `docs/architecture-slip-model-phase5_0_2-chicane-fallback.md` — Phase 5.0.2 chicane planner cap; confirms QP infeasibility is invariant across `safety_mult` and motivates Phase 5.0.3.
- `src/lap_estimator/dynamics/mpc_controller.py` — main integration point. The existing `_commit_ghost` (line 570) and divergence-fallback (lines 371–393) are the surfaces replaced/extended by Phase 5.0.3.
- `src/lap_estimator/dynamics/mpc_qp.py` — `solve_sqp` returns `stats["status_history"]` (line 484); Phase 5.0.3 reads from this. Phase 5.0.3 adds `stats["J_residual"]` and `stats["sqp_du_inf"]` to the stats payload.
- `src/lap_estimator/dynamics/mpc_qp_ellipse.py` — Phase 5.0.1 ellipse constraint helper; the `compute_axle_force_from_state` helper in `mpc_model.py` is extracted from logic currently inline here.
- `src/lap_estimator/dynamics/_slip_result.py` — telemetry dataclass; gains 5 new fields per §23.2-5.0.3.7.
- OSQP documentation, `Status codes`: <https://osqp.org/docs/interfaces/status_values.html> — the canonical mapping from OSQP integer codes to the string keys used in `stats["status_history"]`.
- Borrelli, Bemporad, Morari, *Predictive Control for Linear and Hybrid Systems*, Ch. 14 ("Reference governors and feasibility recovery") — academic precedent for emit-fallback-on-infeasibility patterns in MPC.
- Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed., Ch. 4 — friction-ellipse formulation and the per-axle (D_long, D_lat) coefficients Tier 1 saturates against.
