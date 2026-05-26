# Architecture — v3.1 Phase 4.3: slip-aware steering softener

**Spec:** `dev-planning/lap-simulation-csv-driver/spec-section-23-10-v31-controller.md` §23.10.12
**Branch:** `feature/sc-71955/lap-simulation`
**Status:** Code shipped, **acceptance gate §11.55 + §11.55.E FAIL.** Lap does not complete with the spec-mandated cross-track abort threshold (8 m). With the legacy 50 m threshold for comparison, the lap still does not complete; max cross-track error reaches ~38 m mid-lap. Per spec §23.10.12.8 fail action, the cheap-trick attempt has been honestly executed and has not delivered. **Recommendation: escalate to v3.2 (full MPC / free-driving).**

## What this ships

A multiplicative scalar gain applied to commanded steering δ inside `DriverController.controls()`, between the Stanley raw-δ computation and the `max_steer_rad` clip. The gain is 1.0 when the previous-step measured front α is below `engage · α_peak_front`, decays linearly to 0.0 at `full · α_peak_front`, and saturates at 0.0 above. Defaults: `engage = 0.85`, `full = 1.05`. Two optional fields are added to the driver-JSON `control_params` block; absent fields default sensibly, so every driver JSON on disk loads unchanged.

The cross-track-error abort threshold in `solver.py` is tightened from 50 m to 8 m as spec §23.10.12.6 mandates — this is the diagnostic guard that makes the §11.55.E gate observable.

## Why this architecture

Phase 4.2 (longitudinal DP planner) brought Tomas-Sprint-A to 2:04.14, +16.6 s over real (1:47.56). The plan is feasible against the measured Pacejka envelope; the bottleneck moved to the controller, which over-steers chasing cross-track error and pushes front α to ~19° — well past `α_peak ≈ 6.9°`. On the falling side of the Pacejka curve, lateral force collapses and the tyre cannot turn the car. The principled fix is a receding-horizon optimiser (v3.2 MPC). The Phase 4.3 spec asks for one cheap controller-side scalar gain first, on the hypothesis that *clamping* steering as α approaches peak will hold the tyre near the peak and trade a small amount of cross-track error for lap time.

### Decisions

1. **Multiplicative gain on δ, not on κ or a_y.** Spec §23.10.12.1 places the softener after Stanley, before clip + rate-limit + noise. Multiplying δ preserves sign and only attenuates magnitude.
2. **Previous-step measured α as the input.** Spec §23.10.12.1 reads the controller's own slip-band-modulator approximation of `alpha_front_avg = steer_cmd - body_slip_front`, cached from the prior call. No new state plumbed through the ODE.
3. **Hard sentinel when no Pacejka block.** `_alpha_peak_front_rad = -1.0` when the driver JSON lacks a measured fit; the softener block short-circuits and behaviour matches Phase 4.2 verbatim. Back-compat for hand-authored drivers without measured calibration.
4. **Same α-peak clamp as the slip-target derivation.** `apf_clamped = clip(α_peak_front_deg, 4°, 10°)` per spec §23.10.9 risk #2.
5. **Schema validation at load time, not at runtime.** `ControlParams.from_driver` rejects `engage ∉ [0.5, 1.5)` or `full ∉ (engage, 1.5]` with a clear ValueError; no silent-fallback.
6. **Cross-track abort tightened, not the simulator's geometric model.** Spec §23.10.12.6 calls for tightening the abort threshold (50 m → 8 m) so cross-track drift becomes observable, not silently absorbed. §11.55.E adds a 4 m soft cap so the gate fails before the abort fires.

## Data flow

```
DriverController.controls(state, t)
   │
   ├── Stanley raw δ_cmd  ─── (line 236)
   │
   ├── ── if α_peak_front_rad > 0 ─────────────────────────────────────┐
   │      alpha_norm = |last_alpha_front_avg| / α_peak_front_rad        │
   │      soften = ramp(alpha_norm, engage, full)  in [0.0, 1.0]        │
   │      δ_cmd *= soften                                                │
   │      ────────────────────────────────────────────────────────────  │
   │                                                                    │
   ├── max_steer_rad clip  ─── (was line 243)                           │
   ├── steering rate-limit                                              │
   │                                                                    │
   │  (slip-band throttle modulators — independent denominator,         │
   │   spec §23.10.12.5 — touch throttle / brake only)                  │
   │                                                                    │
   ├── ghost-fallback check                                             │
   ├── consistency noise on δ                                           │
   │                                                                    │
   └── cache alpha_front_avg → self._last_alpha_front_avg  ◄────────────┘
                                          (read on next call's softener block)
```

## Files modified

| File | Lines | Reason |
|---|---|---|
| `src/lap_estimator/dynamics/_control_params.py` | +20 | New fields `steering_softener_engage` / `steering_softener_full`; load-time validation; propagate in `with_slip_target`. |
| `src/lap_estimator/dynamics/driver_controller.py` | +30 (init) +20 (controls) | Resolve `α_peak_front_rad` at __init__; new softener block between lines 236 and 243; cache `_last_alpha_front_avg` at end of `controls()` and on ghost-fallback exit. |
| `src/lap_estimator/dynamics/solver.py` | +6 / −1 | Off-track abort threshold tightened 50 m → 8 m (squared 2500 → 64); abort message rounding upgraded to 1 decimal. |

No driver JSON on disk was modified. No CLI surface change.

## Acceptance gate result (§11.55 + §11.55.E)

| Gate | Required | Measured | Pass |
|---|---|---|---|
| Lap completes | yes | NO — `OffTrackError` at t=12.88 s, s=647 m, cross-track=8.1 m | **FAIL** |
| ±3 s of real (1:47.56) | 1:44.5–1:50.5 | n/a (lap did not complete) | **FAIL** |
| `util_p85 ≤ 1.05` | yes | 0.164 (softener clamped δ → no lateral demand reached the tyre at all in the first 12 s) | n/a |
| Ghost-fallback ≤ 20/lap | yes | 0 (lap aborted before any fallback fired) | n/a |
| MC σ ≤ 0.8 s | yes | 0.000 s (every run aborted near the same s) | n/a |
| **§11.55.E** max cross-track ≤ 4 m | yes | 7.85 m at t=12.86 s (just before abort); with legacy 50 m threshold for diagnostic: 38.07 m at t=15.64 s | **FAIL** |

### α_peak used

`alpha_peak_front_deg = 6.925°` (from `drivers/tomas.json` measured Pacejka block via `Driver._alpha_peak_front_deg()`).
`alpha_peak_front_rad = 0.12086` (post-clamp `max(4°, min(10°, 6.925°)) = 6.925°`).

### Cross-track abort threshold

| Before | After |
|---|---|
| `d2_off > 2500.0` (50 m chassis distance from racing line) | `d2_off > 64.0` (8 m) |

### Diagnostic: with legacy 50 m threshold (softener on)

Lap still does not complete. Aborts at t=16.68 s, s=672 m with cross-track 50.2 m. Max cross-track in the trace before that abort: 38.07 m at t=15.64 s. `util_p85 = 1.58`, meaning the softener fires hard (front α reaches well past peak), but the controller has nothing else to steer with — it pulls δ to zero, the car drifts, and the next corner is unmakeable.

## Why this fails (post-mortem)

The softener treats steering as a single scalar input: when α exceeds peak, suppress δ. But Stanley δ has two jobs — *steer toward the apex* (target geometry) and *keep the car on the line* (cross-track error). Suppressing δ scales both equally, so when α climbs because the car is *under-radius* (entry too fast), softening δ makes the car go even more under-radius. The cross-track error compounds; the next corner is approached with worse geometry; α climbs more; softener kicks harder; lap diverges.

A principled fix needs to decouple "target line" from "current line" — i.e. plan a feasible *trajectory* and track it. That is what receding-horizon MPC does. Spec §23.10.7 / §23.10.12.8 recognise this and route the failure here.

## Integration with neighbouring features

- **Phase 4.1 (preview longitudinal):** untouched. Throttle / brake loop is the same.
- **Phase 4.2 (DP planner):** untouched. The plan is consumed verbatim. Both `--plan-source v2` and `--plan-source v3_dp` fail identically with the softener active — confirming this is a controller bug, not a plan bug.
- **Slip-band throttle modulators (UNDER / OVER / HARD_CAP):** untouched. Different denominator (`slip_target` vs `α_peak`), different actuator. Confirmed independence per spec §23.10.12.5.
- **Ghost-fallback:** untouched. Did not fire in any FAIL run because the off-track abort fired first.

## Recommendation

Per spec §23.10.12.8: **escalate to v3.2 (full MPC).** Phase 4.3 has been honestly attempted and the failure mode is precisely the one §23.10.12.6 risk #1 anticipated. Further controller-side scalar gains will not save it — the controller needs a horizon.

The Phase 4.3 code stays on disk so:
- The kill-switch is available (`steering_softener_engage = 1.5` in driver JSON disables the softener entirely; cross-track abort can be restored to 50 m if a temporary unblock is needed for unrelated debugging).
- v3.2 MPC work can rebase off a controller layer that already has α_peak plumbed in.

The 8 m cross-track abort threshold should stay tightened — it correctly surfaces the off-track-drift failure mode that the legacy 50 m bound was hiding. v3.2 MPC must produce trajectories that pass `§11.55.E ≤ 4 m`.
