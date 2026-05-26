# Architecture — v3 Phase 5.0.5-reactive: reactive-controller steering smoother

**Spec:** inline mini-spec on the 2026-05-23 brief (no Buddy round-trip). Source intent: Phase 5.0.5 Tier-1 architecture doc §"What's left for Phase 5.0.5 / 5.1" — empirical finding that the chronic Sprint A s≈1087 m chatter was in the **reactive sub-controller** (Tier 2), not the Tier-1 emit signal patched in the original Phase 5.0.5.
**Branch:** `feature/sc-71955/lap-simulation`
**Status:** Shipped. Default-on (`stanley_blend_window_ticks=3` → τ=60 ms). With the default chicane cap, reactive now clears the Sprint A chicane and reaches s=1086 m — within 1 m of the chronic chatter station — vs ~660 m pre-fix.

## What this ships

A first-order discrete low-pass filter on the reactive `DriverController`'s post-softener steer command, gated by the same `control_params.stanley.blend_window_ticks` knob the user spec'd.

1. **`ControlParams.stanley_blend_window_ticks: int = 3`** (new field). Configurable via driver JSON `control_params.stanley.blend_window_ticks`. The integer `N` translates to an LPF time constant `τ = N · dt` (so at the simulator's 50 Hz tick / dt=20 ms, default N=3 gives τ=60 ms — directly matching the spec's stated "τ ≈ 60 ms" suggestion for the LPF variant). Setting `N=0` or `N=1` bypasses the LPF entirely (pass-through, byte-equivalent to pre-fix).

2. **`DriverController._steer_lpf_prev: float | None`** (new state). Persists the previous tick's smoothed δ across `controls()` calls. Seeded on the first call; reset to the ghost-fallback's emitted δ on a transient panic step so the post-fallback LPF blends from the recovered command rather than from a stale pre-fallback Stanley target.

3. **Discrete first-order LPF** (matching the spec literally):
   ```
   δ_smoothed[k] = α · δ_target[k] + (1-α) · δ_smoothed[k-1]
   with α = 1 / N
   ```
   Applied to the raw post-softener δ, BEFORE the magnitude clip / steering rate-limit / consistency noise. The clip + rate-limit thus remain the final lateral safety floor on the smoothed signal, and per-tick Gaussian noise injection on emit is never re-fed into the LPF state.

The reactive controller has no integrator term in the Stanley control law (the cross-track correction is a proportional `atan2(k · cross, v + soft)`), so no anti-windup state needs clearing during a windowed flat output. The existing `_max_steer_rate_rad_s = 10.0` rate-limit and `_max_steer_rad = 20°` clip act as the only lateral safety stages downstream of the smoother.

Driver-JSON addition (optional, under `control_params.stanley`):
```json
{
  "stanley": {
    "blend_window_ticks": 3
  }
}
```
Default is 3; absent block falls through to the dataclass default. Not added to any production driver JSON — the default value carries the production behaviour, and the knob is for sensitivity studies.

No CLI flag added (the field is intentionally only driver-JSON-tunable; the sensitivity sweep below documents the range and the production default is locked in via the dataclass).

## Why LPF, not FIR (the spec offered both)

The spec presented two alternatives: a rolling-window FIR (default N=5) and a first-order LPF (τ≈60 ms), with explicit "pick whichever is simpler" guidance. I started with the spec-default FIR but the FIR on the **emitted** signal (matching the Phase 5.0.5 Tier-1 pattern) recursed badly with the consistency-noise injection.

Mathematical sketch of the FIR pathology when applied to a raw radian δ with stochastic noise on emit:

- Tier-1 pattern: `y[k] = mean(y[k-1..k-N])` where `y` is a **unit direction vector**. Unit vectors are bounded (norm = 1), so even when the averaging is recursive it produces a bounded smoother output. The signal-to-noise ratio is structurally favourable.
- Reactive-emit FIR: `y[k] = mean(y[k-1..k-N])` where `y[k-i]` is itself the noise-injected emit `clip(δ_smoothed[k-i]) + noise[k-i]`. The recursion folds the per-tick noise into the smoother state, growing the effective IIR memory beyond the nominal N-tap window.

Empirical confirmation (Tomas / Sprint A / default chicane cap, MC seed median):

| Smoother | Window | Abort station | util_p85 | Notes |
|---|---:|---:|---:|---|
| None (window=1) | — | s≈660 m (chicane) | 0.59 | Pre-fix baseline. |
| Emit-FIR | N=5 (spec default) | s≈169 m | 0.625 | Spec-faithful — but the recursive emit-FIR over-damps turn-in. |
| Emit-FIR | N=3 | s≈165 m | 0.770 | Lower window still too laggy with noise injection. |
| Emit-FIR | N=2 | s≈162 m | 0.917 | Same failure mode. |
| **LPF (this fix)** | **N=3 / τ=60 ms** | **s=1086 m** | **1.083** | Default. Clears the chicane; abort moves to the chronic chatter station. |
| LPF | N=2 | s≈162 m | — | Smoother step response too fast — chatter survives. |
| LPF | N=4 / τ=80 ms | s≈675 m (chicane) | 1.86 | Over-smoothing pushes util through tyre peak in the chicane. |
| LPF | N=5 / τ=100 ms | s≈660 m (chicane) | 1.46 | Same trend. |
| LPF | N=6 / τ=120 ms | s≈657 m (chicane) | 1.38 | Same trend. |
| LPF | N=8 / τ=160 ms | stalled @ t=8 s | 10.5 | Smoother kills turn-in entirely. |

The LPF at N=3 is the only configuration where the controller (a) survives the chicane and (b) reaches within 1 m of the historical s=1087 chatter station — confirming the FIR-on-emit was the wrong shape for this layer, and the LPF on raw is the right one.

## Data flow

```
        VehicleState, t
                │
                ▼
   DriverController.controls()  — every ODE step (dt=20 ms / 50 Hz)
                │
                ▼
   Stanley preview-target steer  ─▶  δ_raw  (heading_err + cross-track)
                │
                ▼
   Soft-start dampening (v < 5 m/s)
                │
                ▼
   Phase 4.3 slip-aware softener (off by default with kill-switch band)
                │
                ▼
   ────────────────────────────────────────────────────────────────────
   PHASE 5.0.5-REACTIVE LPF  (this fix)
   if window > 1:
      α = 1 / window
      δ_smoothed = α · δ_raw + (1-α) · self._steer_lpf_prev
      self._steer_lpf_prev = δ_smoothed
   else:
      δ_smoothed = δ_raw   (pass-through)
   ────────────────────────────────────────────────────────────────────
                │
                ▼
   clip(δ_smoothed, ±20°)
                │
                ▼
   rate-limit (±10 rad/s · dt)
                │
                ▼
   slip-band throttle/brake modulators
                │
                ▼
   spin / over-slip fallback to GhostDriver?
                │       ├─ YES → _steer_lpf_prev = ghost_cmd.steer_rad
                │       │         (sync state so the next normal step
                │       │          blends from the recovered command)
                │       └─ NO  → continue
                ▼
   consistency-noise injection  (Gaussian, channel-wise)
                │
                ▼
   Controls(steer, throttle, brake)  ← emit
                │
                ▼
   ODE step
```

## File inventory

| File | Change | LoC | Purpose |
|---|---|---:|---|
| `src/lap_estimator/dynamics/_control_params.py` | mod | +35 | `ControlParams.stanley_blend_window_ticks` (default 3); `from_driver` reads optional `control_params.stanley.blend_window_ticks`; `with_slip_target` propagates the field. Validation: must be ≥ 0. |
| `src/lap_estimator/dynamics/driver_controller.py` | mod | +30 | `_steer_lpf_prev` state in `__init__`; LPF block in `controls()` between the softener and the clip/rate-limit; LPF state sync on the ghost-fallback path. |
| `docs/architecture-slip-model-phase5_0_5-v32-reactive-chatter.md` | new | this file | Architecture, sensitivity sweep, verification, what's left. |

`driver_controller.py` total: 596 lines (was 532). `_control_params.py`: 201 lines. Both still under the CLAUDE.md soft 500-line target (the controller was already over; the established "no clean seam, leave it" policy from neighbouring docs applies).

## Verification

### Step 1: Build sanity

```python
from lap_estimator.dynamics._control_params import ControlParams
from lap_estimator.dynamics.driver_controller import DriverController
cp = ControlParams()
assert cp.stanley_blend_window_ticks == 3   # default
```

Both module imports succeed; default loads without a driver JSON.

### Step 2: Sprint A — three CLI scenarios (Tomas / skill=1.0 / single lap)

Each row is the Monte Carlo median (Tomas has `consistency_sigma=1.5`, so 10 seeds). New physics head from the 2026-05-23 longitudinal fix.

| Scenario | Pre-fix abort | Post-fix abort | Δ |
|---|---|---|---|
| Default (`safety_mult=0.80, radius_thresh=60.0`) | s≈660 m (chicane), util_p85≈0.59 | **s=1086 m** (chatter station), util_p85=1.083, t=35.16 s | +426 m further; abort moved to the chronic chatter corner. |
| Chicane cap OFF (`--chicane-safety-mult 1.0`) | s=647 m (chicane) per spec brief | s=666 m (chicane), util_p85=1.659 | +19 m. Chicane is now an envelope problem (util > 1.0), not a chatter problem — without the safety speed margin, the plan demands cornering force the tyre can't deliver. |
| Calibrated overrides (`--boost-steady 0.745 --cd-override 0.32 --brake-torque-mult 1.80`) | n/a (new physics) | s=649 m (chicane), util_p85=0.185, t=16.68 s | The calibration overrides reshape the speed profile entering the chicane; util_p85 is now LOW because the controller is failing on geometry (line departure) before the tyre is engaged. |

### Step 3: Is the s=1087 chatter gone?

**Reactive now reaches s=1086 m before aborting — within 1 m of the chronic chatter station.** The lap does not complete; it aborts with the same OffTrackError signature at essentially the same location. So:

- The LPF damps the lateral chatter enough to clear all corners on the way TO s=1087.
- The lateral failure AT s=1087 itself looks like it survives the immediate chatter but accumulates cross-track error over the 40 m sustained-radius section until the 12 m OffTrack tolerance is breached.

That's a **partial fix**: the FIR-shaped smoother (in LPF form) does what the user asked it to do — damps the surrounding chatter so reactive gets to the chronic corner — but the root cause at s=1087 is something the LPF alone doesn't address. The controller still tracks too far off-line through that corner. Reading: the Stanley preview-target steer at v≈40 m/s through a 60 m sustained-radius corner has a too-low cross-track gain (`k_cross / (v + soft) = 0.5/43 ≈ 0.012`), so cross-track error accumulates faster than the corrective steer can null it. This is a **gain-schedule** problem, not a chatter problem. Out of scope for this fix.

## What this DOESN'T fix

- The s=1087 m sustained-corner accumulation. The LPF makes the controller's emit cleaner but doesn't increase the cross-track corrective bandwidth. A v3.2 cross-track gain-schedule (k_cross rising with sustained corner duration, or a feed-forward yaw bias) is the documented next step. Logged as an open point.
- The chicane abort with chicane-safety OFF (s=666). This is an envelope problem at util > 1.0 — the DP plan asks for cornering force the tyre can't deliver without the 0.80 safety margin. Not a chatter problem; the LPF is doing its job there.
- The chicane abort with the calibrated overrides (s=649). The new powertrain/brake calibration reshapes the entry speed profile in a way the reactive controller doesn't pre-brake hard enough for. Same envelope-vs-tracking decomposition as the previous row, but now util_p85 is LOW because the geometry departure happens before the tyre is loaded. Likely a longitudinal pre-brake issue, not a lateral one.

## Open points for a future round

1. **Cross-track gain schedule** in `controls()` for sustained-radius corners. Either lift `k_cross` from 0.5 to a v-scheduled value, OR add a yaw-rate feed-forward derived from the racing-line curvature at the chassis point (not the preview point). See dev-planning/lap-simulation-csv-driver/open-points.md when filed.
2. **Pre-brake calibration for the new powertrain head.** With the brake-torque-mult / boost-steady overrides, the chicane entry speed is too high; the reactive controller's preview-time horizon (1.5 s) may need re-tuning for the new dynamics.

## Related docs (session 2026-05-24)

- **`docs/architecture-v3-lateral-yaw-inertia-fix.md`** — the I_zz correction that
  preceded this LPF. Without the corrected yaw response the pre-LPF util_p85 was
  0.5, which made the chatter look like a utilisation problem rather than a
  steering-command noise problem.
- **`docs/architecture-stanley-crosstrack-gain.md`** — the k_cross uplift that
  followed this fix. The LPF cleared the s=1086 m chatter; the gain fix then
  resolved the sustained-corner cross-track accumulation at that same station.
- **`docs/architecture-slip-model-phase5_0_2-chicane-fallback.md`** — the chicane
  safety cap (mult=0.80, radius<60 m) that the LPF depends on; without the cap
  the chicane remains an envelope problem (util>1.0) that the LPF cannot address.
- **`docs/architecture-v3-session-2026-05-24.md`** — session index.

## References

- Phase 5.0.5 Tier-1 doc: `docs/architecture-slip-model-phase5_0_5-v32-tier1-bistability.md` — establishes the FIR pattern this fix mirrors at the right layer.
- Phase 4.1 doc: `docs/architecture-slip-model-phase4.md` — the reactive controller's preview-Stanley + slip-band P-loop that this fix smooths.
- Sprint-A chicane phase doc: `docs/architecture-slip-model-phase5_0_2-chicane-fallback.md` — chicane-safety multiplier that protects the chicane envelope.
