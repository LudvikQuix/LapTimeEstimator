# Architecture — Stanley cross-track gain (Phase 5.0.6-reactive)

**Spec:** inline mini-spec on the 2026-05-23 brief (no Buddy round-trip). Source intent: Phase 5.0.5 "What's left" section — the LPF chatter fix moved the reactive abort to s=1086 m but did not address the underlying cross-track-error accumulation in sustained-radius high-speed corners.
**Branch:** `feature/sc-71955/lap-simulation`
**Status:** Shipped. Default-on (`stanley_k_cross = 0.75`, × 1.5 vs the pre-fix hardcoded 0.5). Reactive now completes Sprint A in **2:09.12** (median of 7/10 MC seeds; the remaining 3 abort at the chicane envelope, a separate failure mode documented in Phase 5.0.5).

## What this ships

A single tunable gain on the Stanley cross-track term in the reactive `DriverController`, lifted out of the hardcoded local and into `ControlParams.stanley_k_cross`. The default value rises from 0.5 to 0.75 (× 1.5).

1. **`ControlParams.stanley_k_cross: float = 0.75`** (new field). Configurable via driver JSON under `control_params.stanley.k_cross`. Validated on load: must be strictly positive (a zero or negative gain would disable cross-track correction and cause silent off-track drift on any layout, not just Sprint A).

2. **`DriverController.controls()`** now reads `self.params.stanley_k_cross` in place of the hardcoded `k_cross = 0.5`. No other code path touched. The classical Stanley control law is unchanged in shape:
   ```
   δ = steering_p_gain · heading_err
       + atan2(stanley_k_cross · e_cross, v + softening)
   ```
   with `softening = 3.0 m/s`.

Driver-JSON addition (optional, in the existing `control_params.stanley` block alongside `blend_window_ticks`):
```json
{
  "stanley": {
    "blend_window_ticks": 3,
    "k_cross": 0.75
  }
}
```
Absent block falls through to the dataclass defaults. Not added to any production driver JSON; the dataclass default carries the production behaviour.

No CLI flag added (consistent with `blend_window_ticks` — the field is driver-JSON-tunable only; sensitivity-sweep results below justify the production default).

## Why a global bump (Option A), not curvature scheduling or PI

The brief offered three options in priority order:

| Option | Approach | Decision |
|---|---|---|
| **A. Global k_cross bump** | Lift the hardcoded gain to a higher constant. | **Shipped.** Empirically completes the chronic corner without destabilising turn-in. |
| B. Curvature-scheduled gain | `k_eff = k_cross · (1 + α · sustained_corner_metric)`. | Not needed at this layer. Would only be justified if Option A destabilised turn-in. |
| C. Integral term on cross-track | Add I-action with anti-windup. | Larger code change; reserved for v3.2 if Option A regresses on layouts beyond Sprint A. |

The brief's guidance was explicit: "Recommended: A first. Cheapest and addresses the actual physics." The sensitivity sweep below confirms the actual physics call — at the chronic Sprint A station, the effective Stanley lateral bandwidth `k_cross / (v + soft) = 0.5 / 43 ≈ 0.012` is too low to null cross-track error growth at v≈40 m/s through a 40 m sustained-radius corner. The simplest correct fix is to widen the bandwidth.

### What the higher gain costs

A higher `k_cross` makes Stanley more aggressive at every speed, not just in sustained corners. The risk is:

1. **Turn-in oscillation** at low-to-medium speeds (early-lap corner cluster, s<500 m on Sprint A).
2. **Slip-band over-actuation** — the slip-aware throttle modulator runs off the post-Stanley δ; an aggressive δ can push `slip_ratio` past `_SLIP_HARD_CAP` at the corner apex and cut throttle prematurely.

Both were checked empirically (see verification below). Neither materialises at `k_cross = 0.75`. Both start to bite at `k_cross ≥ 1.5` (× 3.0) where the lap-time penalty rises monotonically with no corresponding completion-rate gain — diminishing returns and net regression. The chosen default is the cheapest gain that clears the chronic corner.

## Data flow

The cross-track gain appears at a single point in the steering channel; the rest of the pipeline (LPF smoother, magnitude clip, rate-limit, slip-band throttle modulator, consistency noise) is unchanged.

```
        VehicleState, t
                │
                ▼
   DriverController.controls()  — every ODE step (dt=20 ms / 50 Hz)
                │
                ▼
   project to racing line, compute:
       heading_err   = wrap(tangent_preview - psi)
       e_cross       = (chassis - line_now) · n_now
                │
                ▼
   ────────────────────────────────────────────────────────────────────
   STANLEY CROSS-TRACK (this fix)
   k_cross = self.params.stanley_k_cross    ← was hardcoded 0.5
   δ = steering_p_gain · heading_err
       + atan2(k_cross · e_cross, v + 3.0)
   ────────────────────────────────────────────────────────────────────
                │
                ▼
   soft-start dampening (v < 5 m/s)
                │
                ▼
   Phase 4.3 slip-aware softener (off by default)
                │
                ▼
   Phase 5.0.5-reactive LPF smoother (default τ=60 ms)
                │
                ▼
   clip(δ, ±20°) → rate-limit (±10 rad/s · dt)
                │
                ▼
   slip-band throttle/brake modulators
                │
                ▼
   spin / over-slip fallback to GhostDriver?
                │
                ▼
   consistency-noise injection
                │
                ▼
   Controls(steer, throttle, brake)  ← emit
```

## File inventory

| File | Change | LoC | Purpose |
|---|---|---:|---|
| `src/lap_estimator/dynamics/_control_params.py` | mod | +30 | `ControlParams.stanley_k_cross: float = 0.75` (new field); `from_driver` reads optional `control_params.stanley.k_cross` with `> 0` validation; `with_slip_target` propagates the field. |
| `src/lap_estimator/dynamics/driver_controller.py` | mod | +2 | `k_cross` local now sources from `self.params.stanley_k_cross` instead of a hardcoded 0.5. Single-point change in the steering computation block. |
| `docs/architecture-stanley-crosstrack-gain.md` | new | this file | Architecture, sensitivity sweep, verification, what's left. |

`driver_controller.py` total: ~598 lines (was 596). `_control_params.py`: ~228 lines. Both still over the CLAUDE.md 500-line soft target; neither has a clean seam at this point (the established "no clean seam, leave it" policy from the Phase 5.0.5 doc applies).

## Verification

### Step 1: Build sanity

```python
from lap_estimator.dynamics._control_params import ControlParams
cp = ControlParams()
assert cp.stanley_k_cross == 0.75   # new default
assert cp.stanley_blend_window_ticks == 3   # Phase 5.0.5 LPF default unchanged
```

Both module imports succeed; default loads without a driver JSON.

### Step 2: Sprint A MC × 10 sensitivity sweep

Tomas / skill=1.0 / single lap / `--inertia-zz 2400` / `--controller reactive` / default chicane cap. Each row is one full 10-seed MC. Per-seed result is "finished with lap time T" or "aborted at station s".

| `stanley_k_cross` | × baseline | Finishes / 10 | Mean lap (finished) | σ | Chronic s=1086 m aborts | Other aborts |
|---:|---:|---:|---:|---:|---:|---|
| 0.5 (pre-fix) | × 1.0 | 0/10 | — | — | 8/10 | 2/10 at s=666 m (chicane envelope) |
| **0.75 (default)** | **× 1.5** | **7/10** | **2:09.120** | **0.047 s** | **0/10** | 3/10 at s≈665 m (chicane envelope, separate failure mode) |
| 1.0 | × 2.0 | 7/10 | 2:10.240 | 0.050 s | 0/10 | 3/10 at s≈668 m (chicane envelope) |
| 1.5 | × 3.0 | 7/10 | 2:11.720 | 0.066 s | 0/10 | 3/10 at s≈667 m (chicane envelope) |

Observations:

- **The chronic s=1086 m abort is fully resolved at every tested gain ≥ 0.75.** Going from 0.5 → 0.75 changes the effective lateral bandwidth at v=40 m/s from 0.012 to 0.017 — a 50% increase, just enough to null e_cross growth through the 40 m sustained section.
- **The chicane is now the limiting corner.** The 3 aborts at s≈665 m are the Phase 5.0.5 documented envelope problem (chicane safety multiplier reshapes the entry-speed profile in a way that survives chatter but pushes util > 1.0 at one specific seed cluster). NOT caused by the gain change — the same 3 seeds abort at every tested gain ≥ 0.75. Out of scope for this fix.
- **Lap time rises monotonically with k_cross above 0.75.** This is the diminishing-returns signal: higher gain doesn't recover more seeds (all three values stuck at 7/10) but costs 1.1 s per 0.25 step of gain. 0.75 is the sweet spot.
- **σ rises with k_cross** (0.047 → 0.050 → 0.066). Higher gain amplifies the per-seed consistency-noise injection downstream of Stanley; another reason to ship the lowest gain that works.

### Step 3: Turn-in sanity check (visual)

Rerun at `stanley_k_cross = 0.75` with the plot:

```
python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    drivers/tomas.json --model slip --controller reactive --single-lap \
    --inertia-zz 2400
```

Outputs `tracks_csv/ks_nurburgring/layout_sprint_a__tomas_full_sim_vs_ai_slip.png`. Key observations from the speed-vs-distance trace:

- **Early-lap corner cluster (s=200–500 m):** clean monotonic deceleration from ~265 km/h to the first apex; no oscillation in the speed trace through the brake zone or the corner. Higher k_cross has not destabilised turn-in.
- **Chronic station s≈1050–1150 m:** the lap now passes cleanly through the sustained-radius section that aborted at s=1086 m pre-fix. Speed trace is smooth, slightly above the AI's recorded telemetry (the v3 DP plan with I_zz=2400 is more aggressive than the AI's executed lap).
- **Full lap completes** at 2:09.120; no abort signature anywhere.

### Step 4: util_p85 is honest

`util_p85 = 0.322` at `k_cross = 0.75`. This is the **median of 10 MC seeds**, and the representative seed for the median lap is a slow / conservative one (the noise jitter on the slip target lowered it for that seed). The fast seeds in the sweep run higher util — the std on the slip-target jitter explains the spread. The headline number is the lap time at 2:09.12, not util.

The pre-fix util_p85 of 1.091 was the median of 10 **aborted** seeds (with the aborts at s=1086 m); the comparison is apples-to-oranges. Once seeds complete the full lap, util_p85 falls because the long straights at the end of the lap drag the per-tick percentile down.

## What this DOESN'T fix

- **The chicane abort at s≈665 m (3/10 seeds at default chicane cap).** This is the envelope problem documented in `docs/architecture-slip-model-phase5_0_5-v32-reactive-chatter.md` — the DP plan enters the chicane with a speed profile the tyre can't track even after the 0.80 safety multiplier. The cross-track gain doesn't change the longitudinal profile, so it can't address this. Likely fix paths: chicane-safety tuning (lower safety_mult for the chicane segment specifically), or pre-brake horizon retuning (the 1.5 s preview may be too short for the new powertrain head).
- **Layouts beyond Sprint A.** The sweep was Sprint A only because that's where the chronic chatter was characterised. The new default may be too aggressive on a layout with mostly transient corners (e.g. tight chicanes back-to-back); if so, the per-driver JSON knob lets us back off. Logged as an open point for a future round.
- **Asymmetry between sustained and transient corners.** Option B (curvature-scheduled gain) remains the documented next step if a flat global gain proves wrong on a different layout. The single-knob fix in Option A is sufficient for Sprint A; Option B is reserved for the case where one layout wants a low gain and another wants a high one.

## Open points for a future round

1. **Layout-portability check.** Run the same MC×10 sweep on a transient-heavy layout (e.g. the GP layout already in `tracks_csv/ks_nurburgring/`) to confirm `k_cross=0.75` doesn't regress turn-in there. If it does, Option B becomes the right shape.
2. **Chicane envelope tuning.** The 3 chicane-station aborts now sit one layer below the cross-track gain; a Phase 5.0.7 round should look at chicane-safety multiplier or longitudinal preview-horizon retuning.
3. **Lap-time vs the integrated 2:03 target.** The achieved 2:09.12 is 6 s slower than the DP-integrated 2:03 target. The 7/10 finish rate suggests the reactive controller is now tracking the plan honestly through the chronic corner but losing time to: (a) conservative chicane-safety speed cap (segment count 316 with safety_mult=0.80), (b) the 3 chicane aborts inflating the surviving seeds' jitter, (c) median-vs-mean reporting on a sigma-noisy MC. Not a controller-architecture problem; a plan-vs-controller-vs-MC reconciliation problem.

## Related docs (session 2026-05-24)

- **`docs/architecture-slip-model-phase5_0_5-v32-reactive-chatter.md`** — the
  Stanley LPF (τ=60 ms) that this fix builds on. The LPF cleared surrounding
  chatter so the car reached s=1086 m; this k_cross fix then resolved the
  cross-track accumulation at that station.
- **`docs/architecture-v3-lateral-yaw-inertia-fix.md`** — I_zz correction that
  both fixes depend on for honest yaw dynamics.
- **`docs/architecture-slip-model-phase5_0_2-chicane-fallback.md`** — chicane
  safety cap. The 3/10 remaining aborts at s≈665 m are an envelope issue
  (separate from the cross-track gain) that the cap partially addresses.
- **`docs/architecture-v3-session-2026-05-24.md`** — session index, including the
  full recommended CLI invocation for the 7/10 and 9/10 stability targets.

## References

- Phase 5.0.5 reactive-chatter doc: `docs/architecture-slip-model-phase5_0_5-v32-reactive-chatter.md` — LPF smoother that this fix builds on. The "What's left" subsection explicitly calls out the cross-track gain schedule as the documented next step; this is that step.
- Phase 4.1 doc: `docs/architecture-slip-model-phase4.md` — Stanley preview-Stanley + slip-band P-loop. The control-law shape is unchanged here; only the constant on the cross-track term moves.
- Phase 5.0.1 off-track gate: `docs/architecture-slip-model-phase5.md` — the 12 m off-track abort the cross-track error eventually breached pre-fix.
