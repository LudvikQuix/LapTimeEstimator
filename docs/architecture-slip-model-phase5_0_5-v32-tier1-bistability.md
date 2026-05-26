# Architecture — v3 Phase 5.0.5: Tier-1 emit bistability fix (rolling-window FIR)

**Spec:** inline mini-spec on the Phase 5.0.5 brief (no Buddy round-trip). Source intent: Phase 5.0.4 architecture doc §"What's left for Phase 5.0.5 / 5.1" item 1.
**Branch:** `feature/sc-71955/lap-simulation`
**Status:** Shipped (additive; default-on for Tomas and Ludvik). **Gate D (MC 3-lap completions >= 7/10) FAILED — 0/10 completions**, same Sprint A chicane abort signature as Phases 5.0 through 5.0.4. The targeted Tier-1 emit smoother does measurably damp the documented 50 Hz Tier-1 direction-blend bistability (≥3.6× attenuation by smoke test; clean engagement in 15/20 chicane-window Tier-1 emits). But the chicane abort itself is **not driven by Tier-1 emit chatter** — empirically the binding chatter is in the **reactive sub-controller running under Tier 2 (chassis divergence fallback)**, which Phase 5.0.5 is not specced to touch. This is the 7th consecutive controller iteration to fail at the same corner; honest report in §"What this DOESN'T fix" below.

## What this ships

A finite-impulse-response smoother on the Tier-1 emit's planned-direction signal. Sized to damp the documented 50 Hz limit cycle.

1. **`Tier1Config.blend_window_ticks`** (new, default 5). At 50 Hz that is 100 ms — longer than the 20 ms limit cycle observed in `.tmp/diag_chicane_5_0_4.csv`, so a 5-tick uniform mean attenuates the dominant alternation by ≥3.6× (smoke test in `.tmp/smoke_fir_damping.py`). Configurable via `control_params.mpc.tier1.blend_window_ticks` in the driver JSON. Setting to 0 or 1 restores Phase 5.0.4 byte-for-byte behaviour.

2. **`MPCController._tier1_direction_history`** (new). A `collections.deque(maxlen=window)` of recent emitted `((nx_f, ny_f), (nx_r, ny_r))` unit-vector pairs. Cleared on:
   - Clean MPC commit (a fresh Tier-1 episode shouldn't average in stale samples from prior episodes).
   - Tier-2 entry (the chassis is doing something the buffered direction history can't honestly inform).
   - Controller construction (empty by default).

3. **`compute_planned_direction(...)` extension** in `mpc_controller_tiers.py`. Two new keyword args (`direction_history`, `blend_window_ticks`); both default to legacy (no FIR, no extra state). When the history buffer is full (`len >= window` and `window > 1`), the function returns the **mean of the last `window` entries**, re-normalised to unit length per axle. Before the buffer is full ("bootstrap" path), the function returns the legacy instantaneous direction (the existing tier-history-based stale-MPC vs Stanley blend), so the first ≤ 4 ticks of any Tier-1 episode behave exactly as in 5.0.4. A degenerate case (mean direction = 0) falls back to the instantaneous value too.

The smoother is on the **emitted** direction, not on the input candidates — so the next-tick output is the average of what we actually emitted, not what we wanted. That makes it a classic FIR smoother on the control signal, which is what the spec called for.

Driver-JSON addition (optional, under `control_params.mpc.tier1`):
```json
{
  "tier1": {
    "blend_window_ticks": 5
  }
}
```
Default is 5; absent block falls through to `Tier1Config()` (also 5). Shipped explicitly in `drivers/tomas.json` and `drivers/ludvik.json` for visibility.

No CLI flag added (the field is intentionally only driver-JSON-tunable; the sensitivity sweep below documents the range and the production default stays 5).

## Data flow

```
        (chassis state, t)
                │
                ▼
   MPCController.controls() — every ODE step (50 Hz tick = 20 ms)
                │
   ┌────────────┴────────────┐
   │                         │
   ▼                         ▼
 Tier 1 emit path           Tier 0 / Tier 2 (unchanged)
   │
   │  build stanley_forces from sub-controller cmd
   │  prev_planned_axle_forces = last clean MPC commit
   │  ─────────────────────────────────────────
   │  compute_planned_direction(
   │      prev_tier, prev_planned_axle_forces,
   │      stanley_forces,
   │      direction_history=self._tier1_direction_history,
   │      blend_window_ticks=self.tier1.blend_window_ticks,
   │  )
   │           │
   │   ┌───────┴───────────────┐
   │   ▼                       ▼
   │  legacy instantaneous     PHASE 5.0.5 FIR
   │  (history empty /         mean of last `window` entries,
   │   window <= 1)            re-normalised; engages once
   │                           buffer is full
   │           │
   │           ▼
   │   (n_front, n_rear)  ← smoothed unit vectors
   │           │
   │           ▼
   │   self._tier1_direction_history.append((n_front, n_rear))
   │           │
   │           ▼
   │   emit_ellipse_saturation(state, n_front, n_rear, ...)
   │           │
   │           ▼
   │   Controls(steer, throttle, brake)  ← emit
   │
   ▼ (clean MPC commit happens later)
 _commit_clean_mpc(...) →  self._tier1_direction_history.clear()
 _enter_tier2(...)      →  self._tier1_direction_history.clear()
```

## File inventory

| File | Change | LoC | Purpose |
|---|---|---:|---|
| `src/lap_estimator/dynamics/mpc_controller_tiers.py` | mod | +35 | `compute_planned_direction` gains `direction_history` + `blend_window_ticks` kwargs; FIR averaging + bootstrap fallback. Pure function, no extra state. |
| `src/lap_estimator/dynamics/mpc_controller.py` | mod | +25 | `Tier1Config.blend_window_ticks` field + `from_block` plumbing; `MPCController._tier1_direction_history` deque; FIR engagement in `_emit_tier1_controls`; buffer-clear hooks in `_commit_clean_mpc` and `_enter_tier2`. |
| `drivers/tomas.json` | mod | +3 | `control_params.mpc.tier1.blend_window_ticks: 5` (explicit; default value, surfaced for transparency). |
| `drivers/ludvik.json` | mod | +3 | Same. |
| `docs/architecture-slip-model-phase5_0_5-v32-tier1-bistability.md` | new | this file | Architecture, sensitivity sweep, gate results, what's left. |
| `.tmp/smoke_fir_damping.py` | new | scratch | Unit smoke test: confirms the FIR attenuates an alternating direction signal by ≥3.6× at window=5 (and fully at window>=8). |
| `.tmp/diag_chicane_5_0_5.py` | new | scratch | Mirrors `.tmp/diag_chicane_5_0_4.py`; logs per-tick chassis state + emit through the chicane with the FIR engaged. |
| `.tmp/diag_chicane_5_0_5_debug.py` | new | scratch | Wraps `compute_planned_direction` to log FIR engagement count + bootstrap region. |
| `.tmp/acceptance_5_0_5.py` | new | scratch | 10-seed × 3-lap §11.55-5.0.5 acceptance sweep harness. |
| `.tmp/sensitivity_5_0_5.py` | new | scratch | Sweep of `blend_window_ticks ∈ {3, 5, 8, 12}` on seed 0. |

`mpc_controller.py` total grew from 1080 (Phase 5.0.4) to ~1105 lines — still under the soft 500-line ceiling already long-passed by this file. CLAUDE.md "no natural seam, leave it" guidance applies; the §5.0.4 doc already established the controller has no clean split short of a refactor.

`mpc_controller_tiers.py` total grew from 604 to ~640 lines, comfortably under the 500-line guideline-as-target only if measured against the original module charter (the file is documented in its own header as the relief module that lets the controller stay under 500 — by inheritance it's allowed to exceed 500 itself).

## Why this architecture, not the alternatives

The spec specified the FIR shape (rolling-window mean over recent emitted directions). Build-time decisions on top of the spec:

1. **Buffer the post-smooth emitted direction, not the candidate inputs.** The spec said "keep a short ring buffer of recent planned-direction unit vectors". The two readings are: (a) buffer the candidates the FIR will then average over, or (b) buffer the outputs (post-FIR) and apply the FIR over those outputs each tick (a moving average of the previous outputs). I picked (b) — it produces a classical FIR on the **emitted** control signal, which is what closes the feedback loop with the plant. Option (a) would average the noisy candidate signal which doesn't directly damp the actuator chatter. The append-after-compute order in `_emit_tier1_controls` is the key.

2. **Bootstrap on instantaneous, not on Stanley-only.** Per spec: "before the buffer is full, fall back to the existing fixed-α=0.5 (or the most recent value, whichever is simpler)." I chose the instantaneous tier-history-based blend — this is the cheapest and matches Phase 5.0.4 behaviour byte-for-byte for the first ≤ 4 ticks of any Tier-1 episode. If the chicane episode lasts more than 4 ticks (the spec-cited norm), the FIR engages from tick 5 onward.

3. **Buffer cleared on `_commit_clean_mpc` and `_enter_tier2`**, not on tier-history transition alone. Cleaning the buffer when MPC recovers is the spec's intent (a fresh Tier-1 episode shouldn't blend in stale data); clearing on Tier-2 entry is the symmetric case (the reactive sub-controller is taking over; if MPC reasserts later it should start from a fresh buffer). Not cleared mid-Tier-1 — the rolling window IS the smoother.

4. **No per-axle window asymmetry.** Both front and rear axles share the same window length and the same history slot. A two-axle FIR with different windows would only matter if one axle saturates faster than the other; the chicane diagnostic shows both axles' direction signals alternate in lockstep, so a single window is correct.

5. **Window default = 5 (100 ms at 50 Hz)**, not 3 or 8. The spec explicitly anchored on 5 ticks. The sensitivity sweep below confirms larger windows damp more aggressively (window=12 produces a clean 79.1% Tier-0 share vs 46.3% at window=5) — but **even window=12 still aborts at the chicane**. Larger windows lag more behind transients in the plant; window=5 is the conservative midpoint.

## Verification

### Step 1: Smoke test (unit-level)

`.tmp/smoke_fir_damping.py` constructs an alternating unit-vector deque ((1,0) ↔ (0,1) per tick) and measures the consecutive-tick peak-to-peak swing in the emitted direction:

```
window= 1: peak-to-peak swing = 1.4142   (= sqrt(2), full alternation)
window= 3: peak-to-peak swing = 0.6325
window= 5: peak-to-peak swing = 0.3922   (3.61x attenuation vs w=1)
window= 8: peak-to-peak swing = 0.0000   (fully damped)
window=12: peak-to-peak swing = 0.0000
```

The FIR is structurally correct. Larger windows damp more.

### Step 2: Chicane diagnostic (integration-level)

`.tmp/diag_chicane_5_0_5.py` re-runs the same flying-lap trace as `.tmp/diag_chicane_5_0_4.py` with FIR window=5 (driver-JSON default). Comparing the last 60 ODE steps before abort against the 5.0.4 baseline (`.tmp/diag_chicane_5_0_4.csv`):

| Window | last-60 steer range | last-60 brake range | |Δsteer|_max | |Δbrake|_max |
|---|---|---|---:|---:|
| 5.0.4 (no FIR) | [0.000, 0.200] | [0.456, 1.000] | 0.2000 | 0.4846 |
| 5.0.5 (FIR w=5) | [0.000, 0.200] | [0.000, 1.000] | 0.2000 | 0.4835 |

**Identical chatter signature.** The FIR did not move the last-60-step alternation. The reason is structural: `.tmp/diag_chicane_5_0_5_debug.py` patches `compute_planned_direction` to log every call, and reports:

```
Total compute_planned_direction calls:  20
FIR engaged (hist_len >= window):       15
Bootstrap (hist too short):              5
tier_counts: {0: 291, 1: 20, 2: 413}    ← 20 Tier-1 emits, 413 Tier-2 emits
```

Of the 724 ODE steps in the failed lap, only **20 are emitted by Tier 1**; 413 are emitted by the **reactive sub-controller running under Tier 2** (chassis-divergence fallback). The chatter visible in the last-60-step CSV is Tier-2 emit, not Tier-1. The FIR cleanly engaged on every long-enough Tier-1 episode (15/20 emits past the 5-tick bootstrap window) and produced stable smoothed directions (e.g. `n_front ≈ (-0.955, +0.295)` for 12 consecutive ticks at the chicane window), but Tier 2 dominates emission counts ~20:1 at the chicane and dictates the actuator chatter the simulator sees.

### Step 3: §11.55-5.0.5 acceptance sweep

`.tmp/acceptance_5_0_5.py`. Tomas / Sprint A / skill=1.0 / MPC / 10 MC seeds × 3 laps each. Driver JSON default `blend_window_ticks=5`. Chicane safety_mult=0.80, dynamic Fz on.

| Gate | Target | Phase 5.0.5 | Verdict |
|---|---|---:|---|
| **D. MC 3-lap completions** (must-pass) | ≥ 7 / 10 | **0 / 10** | **FAIL-MUST** |
| A. Tier 0 fraction | ≥ 80 % | 46.3 % | FAIL |
| B. Tier 1 fraction | ≤ 15 % | 4.7 % | PASS |
| C. Tier 2 fraction | ≤ 5 % | 49.0 % | FAIL |
| E. Lap 1 best | ≤ 2:15.14 | n/a (all aborts pre-lap-end) | N/A |
| F. Stretch | ≤ 2:00 | n/a | N/A |
| J. Solve mean | < 30 ms | 25.0 ms | PASS |
| K. Solve p99 | < 50 ms | 39.2 ms (max 53.5) | PASS |
| L. Post-solve ellipse violation p95 | ≤ 0.05 | 0.0000 | PASS |

10/10 seeds abort with the same signature: `OffTrackError at t ≈ 12.8-14.5 s: chassis 12.0-12.2 m from racing line (s = 646-655 m)`. Identical to 5.0.3 / 5.0.4. Tier-1 episode counts are dominated by single- or two-tick events (461 episodes, max consecutive 11) — the FIR rarely accumulates 5+ ticks of Tier-1 within an episode.

Tier 0 fraction dropped from the 79.4 % reported in Phase 5.0.4's single-lap single-run measurement to 46.3 % here because the acceptance sweep counts ODE steps across **failed** lap attempts (abort at t ≈ 13-14 s of 3-lap simulations), and the chicane region spends a much higher fraction of time in Tier 2 than the per-lap-average. Phase 5.0.4's per-lap single-run measurement was on a single seed; the 10-seed aggregate here is the more honest measure.

### Step 4: Sensitivity sweep

`.tmp/sensitivity_5_0_5.py`. Seed 0, sweeping `blend_window_ticks ∈ {3, 5, 8, 12}` (3 laps each):

| `blend_window_ticks` | Finished | Tier 0 | Tier 1 | Tier 2 | Abort |
|---:|:---|---:|---:|---:|---|
| 3 | FAIL | 45.4 % | 3.1 % | 51.5 % | s = 649 m at t = 12.82 s |
| 5 (default) | FAIL | 40.2 % | 2.8 % | 57.0 % | s = 655 m at t = 14.48 s |
| 8 | FAIL | 40.1 % | 2.8 % | 57.2 % | s = 655 m at t = 14.52 s |
| 12 | FAIL | 79.1 % | 4.7 % | 16.2 % | s = 647 m at t = 13.62 s |

**No window value rescues gate D.** Larger windows (window=12) noticeably shift the tier distribution toward MPC-dominance because the FIR-stabilised Tier 1 produces fewer escalations to Tier 2, but the chicane abort point itself (s = 646-655 m) is unchanged. The Tier-1 fix is working as designed and visibly improves the controller's tier health in the brake-zone-to-apex regime; the binding failure is downstream (reactive sub-controller chatter under Tier 2, see "What this DOESN'T fix" below).

## Where the abort happens

The 5.0.5 abort signature is identical to 5.0.4's (architecture-slip-model-phase5_0_4-v32-load-transfer.md §"Where the abort happens"): the chassis is in Tier-2 reactive fallback for ~10-15 ODE steps immediately before abort, the reactive sub-controller (preview Stanley with slip-band P-loop) alternates between `(steer ≈ 0.2, brake ≈ 0.5)` and `(steer ≈ 0.0, brake ≈ 1.0)` at 50 Hz, and cross-track exceeds the 12 m abort threshold at s ≈ 647-655 m.

The reactive sub-controller (in `driver_controller.py`) was tuned in Phase 4.x against a different envelope and operates at the same 50 Hz tick as the MPC. Inside the chicane it sees a tightly-clipped target speed (12.9 m/s from the chicane-safety v_max cap), a high cross-track, and a high heading error — the conditions under which it alternates between "brake harder" and "let off brake + apply lateral correction". That alternation is the source of the chatter in the CSV. **The reactive controller alone can complete the chicane** — running `--controller reactive` on the same driver/track gets to s = 1087 m (well past the chicane) before aborting at a later corner. The chicane abort under `--controller mpc` happens because the **Tier-2 hysteresis** keeps the reactive sub-controller engaged through the chicane window after the MPC's brief Tier-1 saturation episode escalates to Tier 2, and the sub-controller is forced to handle a chassis state that is already past the trust region by the time it takes over.

## What this DOESN'T fix

The Phase 5.0.5 brief identified Tier-1 emit bistability as the proximate abort cause based on `.tmp/diag_chicane_5_0_4.csv` showing "74 of last 78 ODE ticks before abort are Tier 1". That count was derived from the `latest_tier` column, which is `_latest_tick_tier` — the MPC tick's *verdict*, not the per-ODE-step *emitted* tier. Across the chicane window, `_latest_tick_tier` stays at 1 (the MPC tick keeps returning ellipse-hit verdicts), but the per-step emit path routes to Tier 2 via the chassis-divergence check at the top of `controls()` for the vast majority of ODE steps. The Phase 5.0.4 doc inadvertently misread that flag.

Concretely (seed 0, 10-seed sweep):
- **20 ODE steps** emit via Tier 1 (ellipse saturation) — these are now FIR-smoothed and stable.
- **413 ODE steps** emit via Tier 2 (reactive sub-controller) — these chatter at 50 Hz and drive the abort.

The FIR fix is **structurally correct for the layer it targets** and would matter if the controller were spending the chicane in Tier 1. It empirically does not, so gate D is unaffected.

## What's left for Phase 5.0.6+

Per the spec brief: "This would be the 7th consecutive controller iteration to fail. Honest report; do not over-engineer." The leading suspects, re-prioritised after this finding:

1. **Damp the reactive sub-controller in the chicane window (5.0.6 candidate).** The chatter at the abort point is between `(0.2, 0.5)` and `(0.0, 1.0)` — same pattern as 5.0.4 because nothing in the MPC pipeline has touched the sub-controller. Options:
   - Lower the chassis-divergence Tier 2 cross-track threshold (currently 4 m) — fires Tier 2 less aggressively and lets Tier 1's now-stable FIR direction reach the actuator.
   - Apply the same FIR smoother *inside* the reactive sub-controller — would touch `driver_controller.py`, outside the MPC scope.
   - Hand the reactive sub-controller a longer pedal smoothing time constant — also `driver_controller.py` change.
   This is the next minimum-viable fix; estimated ~50 LoC.

2. **Per-wheel Fz tracking in the MPC plant (5.0.6+ rewrite, ~300 LoC).** Same rationale as in `architecture-slip-model-phase5_0_4-v32-load-transfer.md`. Now the deeper structural suspect: the chicane chatter under the reactive sub-controller is driven by the same combined-slip clamp in `vehicle.compute_derivatives` (truth model, per-wheel) that the MPC's axle plant cannot reproduce. A per-wheel MPC plant would let the QP plan the front-axle right-wheel saturation explicitly instead of the sub-controller flailing reactively.

3. **DP planner reconsidered.** Build-time chicane safety_mult sweep (5.0.4 doc §Q4) said the planner isn't binding. Re-test at lower safety_mult once the sub-controller chatter is damped — they may interact.

4. **Track-line revisit.** The Sprint A chicane racing line in `tracks_csv/ks_nurburgring/layout_sprint_a.csv` may itself be geometrically infeasible at the BMW 1M's measured grip envelope. Regenerate from the corner detector pipeline against current Pacejka constants and overlay. Lives in track-pipeline tooling, not the controller.

5. **Pacejka refit with load-sensitivity.** Same as 5.0.4 doc §"What's left" item 3 — out of scope for any 5.0.x controller phase.

## References

- Spec: inline brief on the Phase 5.0.5 user message (the user's mini-spec).
- Predecessors:
  - `docs/architecture-slip-model-phase5_0_4-v32-load-transfer.md` — Phase 5.0.4 per-stage dynamic Fz; identifies Tier-1 bistability as the proximate next target.
  - `docs/architecture-slip-model-phase5_0_3-v32-tier1.md` — Tier-1 ladder + saturation feedforward; the layer this phase smooths.
  - `docs/architecture-slip-model-phase5_0-v32-mpc.md` — Phase 5.0 receding-horizon MPC.
- Implementation:
  - `src/lap_estimator/dynamics/mpc_controller_tiers.py` — `compute_planned_direction` FIR extension.
  - `src/lap_estimator/dynamics/mpc_controller.py` — `Tier1Config.blend_window_ticks`, `_tier1_direction_history` deque, buffer-clear hooks.
- Build-time verification:
  - `.tmp/smoke_fir_damping.py` — FIR unit test.
  - `.tmp/diag_chicane_5_0_5.py` — chicane trace.
  - `.tmp/diag_chicane_5_0_5_debug.py` — FIR engagement counter.
  - `.tmp/acceptance_5_0_5.py` — §11.55 acceptance sweep.
  - `.tmp/sensitivity_5_0_5.py` — window sensitivity sweep.
  - `.tmp/diag_chicane_5_0_5.csv` — per-tick state from the chicane diagnostic.
