# Reactive Adaptive (Per-Corner) Preview Braking

**Status:** Draft
**Project:** LapTimeEstimator
**Branch:** feature/sc-71955/lap-simulation
**Created:** 2026-05-27
**Planned with:** Buddy

## 1. Summary

The reactive controller's longitudinal brake channel uses a **uniform** preview
horizon: `brake_lookahead_m = max(30, v_preview · preview_time_s)` with a single
`preview_time_s`. On the high-grip plant (`drivers/tomas_highgrip.json`,
D×1.28 = Tomas's measured ~1.5 g envelope), the lap now finishes 10/10 at
**111.92 s** with `preview_time_s = 2.3`, but that is **+1.36 s over the
≤ 110.56 s target** (Tomas himself: 107.56 s). The residual gap is *structural*,
not a tuning miss: a uniform preview long enough for the tight s≈627 m hairpin
(needs ≈2.3 s) is far too long everywhere else, so the car pre-brakes on every
faster corner and bleeds time lap-wide. Sweeps confirm lap time is set
**entirely** by `preview_time_s`; `chicane_safety_mult` and `brake_p_gain` do
nothing on the finishers (brake already saturates to 1.0 in the braking zone).

This spec makes the preview **adaptive (per-corner)**: short by default, long
only where a genuine large speed drop is imminent. The goal is to beat 111.92 s
and pass ≤ 110.56 s on the high-grip plant while leaving the stock reactive
baseline (`drivers/tomas.json`, ~2:09 / 10-of-10) untouched.

## 2. Goals

- Replace the uniform brake-lookahead horizon with a per-corner adaptive one.
- High-grip ideal-line reactive: finish 10/10 **and** lap ≤ 110.56 s (stretch:
  approach the 110.88 s ideal ceiling / Tomas 107.56 s).
- Stock reactive baseline preserved bit-for-bit by default (opt-in switch).
- Parameter-light, tunable via `control_params` exactly like the existing
  `preview_time_s` / `brake_p_gain`.
- Stay within the ~500-line ceiling; the change is confined to one file's
  reactive brake path plus the `ControlParams` dataclass.

## 3. Non-goals

- No change to the tyre model, the friction ellipse, or the grip-envelope fix.
- No change to HMPC / MPC / MPCC controllers (they have their own preview), the
  ideal-line CSV bypass, or the v2 point-mass model.
- No planner / DP change (no brake-distance feasibility cap on plan v_max).
- No new brake rate-limit, no change to `brake_p_gain` magnitude behaviour.
- Not adding a radius/curvature array to the planner output (see §9 — the
  recommended formulation does not need one).

## 4. User stories / scenarios

1. **High-grip hairpin (s≈627 m):** car approaches at ~45 m/s; the plan drops to
   ~15 m/s at the apex. The adaptive horizon expands to the physical braking
   distance for that drop, so braking begins early enough to clear the hairpin
   on-line — 10/10, as the uniform 2.3 s preview already achieves.
2. **Fast sweeper / straight (everywhere else):** little or no upcoming speed
   drop, so the adaptive horizon collapses toward the 30 m floor. The car no
   longer pre-brakes; it carries speed it previously bled, recovering the
   ≈1.36 s lap-wide deficit.
3. **Stock plant, switch off (default):** with `tomas.json` the adaptive flag is
   absent/false, so the controller runs the existing uniform path verbatim and
   still finishes ~2:09 / 10-of-10.
4. **Stock plant, switch on (A/B):** an analyst sets the flag on `tomas.json` to
   confirm adaptive reduces to ≈ uniform behaviour there (no regression / mild
   improvement), validating the safety claim.

## 5. Proposed design

**Recommended: Formulation 1 — physics-based braking-distance preview.**

Compute the brake lookahead as the distance physically needed to decelerate from
the current speed to the minimum upcoming plan speed at the plant's available
longitudinal deceleration:

```
lookahead_m = max(min_lookahead_m, safety_factor · (v² − v_min_ahead²) / (2 · a_brake_avail))
```

This auto-adapts per corner: the term `(v² − v_min_ahead²)` is large only when a
large speed drop is imminent (the hairpin), and ≈0 on straights / fast corners,
where the horizon falls back to the floor. It is principled (the same v²/2a
relation the planner uses), parameter-light (one decel constant + one safety
factor + a floor), and needs no curvature source.

**Chicken-and-egg note (important):** `v_min_ahead` is itself the output of
`_min_speed_within(idx, lookahead_m)`, but `lookahead_m` depends on
`v_min_ahead`. Resolve with a **two-pass evaluation per tick** inside the
controller:

1. **Seed pass:** compute a seed `v_min_ahead` using a generous fixed seed
   horizon (the legacy uniform window `max(30, v_preview · preview_time_s)`, or a
   dedicated `seed_lookahead_m`). This guarantees the real downstream slow point
   is in view.
2. **Physics pass:** compute `lookahead_m` from the seed `v_min_ahead`, then
   recompute `v_target_min = _min_speed_within(idx, lookahead_m)` over that
   tightened horizon and feed it to the existing brake P-controller unchanged.

One fixed-point iteration is sufficient and stable: the seed horizon is an upper
bound, so the physics horizon only ever shrinks toward the true braking distance,
never missing the slow point. Clamp the physics horizon to the seed horizon as an
upper bound to keep the search bounded and prevent absurd horizons.

Everything downstream of `v_target_min` (the brake P-term at lines 417-418, the
throttle gating, the slip-band logic) is **unchanged**. The only edit is how
`brake_lookahead_m` / `v_target_min` are derived (lines 391-409).

### `a_brake_avail` source

Use a **driver-JSON-supplied constant**, defaulting to a conservative fixed value,
not a live plant query (the controller does not have the friction-ellipse solver
in hand and a live estimate would couple the two passes). The high-grip envelope
fix note records a measured **chicane decel ≈ 13 m/s²** (`tomas_highgrip.json`
`grip_envelope_fix`), so the high-grip driver opts in with a value anchored
there (start `a_brake_avail ≈ 11–12 m/s²`, swept). The stock plant, if ever
opted in, would use a lower value (~9–10 m/s²). Exposed as
`control_params.preview.a_brake_avail_ms2`.

### Fallback to today's behaviour

When `control_params.preview.mode` is absent or `"uniform"` (the default), the
controller computes `brake_lookahead_m` exactly as today and skips both passes —
**zero behavioural change**, guaranteeing the stock baseline. `"adaptive"`
selects Formulation 1. This is the regression-safety switch (§ Regression safety).

## 6. Sub-features / work breakdown

1. **`ControlParams` preview sub-block** *(ArchDev)*
   - Add a nested `preview` block read in `ControlParams.from_driver`
     (`src/lap_estimator/dynamics/_control_params.py`), mirroring how the
     `stanley` / `mpc` sub-blocks are parsed (line 142 onward):
     - `mode: str = "uniform"` — `"uniform"` | `"adaptive"`.
     - `a_brake_avail_ms2: float = 10.0` — plant decel used in v²/2a.
     - `safety_factor: float = 1.15` — margin on the computed distance.
     - `min_lookahead_m: float = 30.0` — the existing 30 m floor, now named.
     - `seed_lookahead_m: float | None = None` — when None, reuse the legacy
       uniform window `max(30, v_preview · preview_time_s)` as the seed.
   - Validate ranges (positive decel, `safety_factor ≥ 1.0`, floor ≥ 0); raise
     with a `control_params.preview.*` message consistent with existing
     validators. Keep `preview_time_s` / `brake_p_gain` defaults intact.
   - Touchpoints: `_control_params.py` dataclass fields + `from_driver`.
   - Depends on: none.

2. **Adaptive brake-lookahead in the reactive path** *(ArchDev)*
   - In `driver_controller.py` lines ~391-409, branch on `self.params.preview`
     mode:
     - `uniform` → existing code path unchanged.
     - `adaptive` → seed pass + physics pass as in §5, producing
       `brake_lookahead_m` and `v_target_min`.
   - Reuse the existing `_min_speed_within(idx, lookahead_m)` helper (line 612)
     for both passes — no new geometry needed.
   - Optionally factor the two-pass math into a small private helper
     (`_adaptive_brake_lookahead(idx, v, v_preview)`) to keep the main
     `step()` readable and under the 500-line ceiling.
   - Depends on: sub-feature 1.

3. **High-grip driver opt-in config** *(ArchDev)*
   - Add the `control_params.preview` block to `drivers/tomas_highgrip.json`
     with `mode: "adaptive"` and the anchored `a_brake_avail_ms2`. Leave
     `drivers/tomas.json` with no preview block (stays uniform).
   - Note: the high-grip driver currently has **no** `preview_time_s` /
     `brake_p_gain` override (defaults 1.5 / 0.6); the winning uniform result
     used `preview_time_s = 2.3`. For adaptive, `preview_time_s` only governs the
     seed horizon — verify the seed still surfaces the hairpin slow point at the
     default 1.5 s, or set a `seed_lookahead_m` explicitly.
   - Depends on: sub-features 1–2.

4. **Tuning sweep + acceptance verification** *(ArchDev, backgrounded)*
   - Sweep `a_brake_avail_ms2` (≈9–13) × `safety_factor` (≈1.05–1.30) on the
     high-grip ideal-line, 10-seed MC, to find the fastest finisher; confirm
     ≤ 110.56 s and 10/10. Harness goes in `.tmp/` (scratch).
   - Run the stock-baseline regression check (both default-off and forced-on).
   - Depends on: sub-features 1–3.

## 7. Data & interface contracts

New driver-JSON block (read by `ControlParams.from_driver`):

```jsonc
"control_params": {
  "preview": {
    "mode": "adaptive",          // "uniform" (default) | "adaptive"
    "a_brake_avail_ms2": 11.5,   // plant decel for v²/2a; high-grip ~11–12
    "safety_factor": 1.15,       // >= 1.0
    "min_lookahead_m": 30.0,     // floor (existing behaviour)
    "seed_lookahead_m": null     // null => reuse uniform window as seed
  }
}
```

Controller-internal contract (per tick, `adaptive` mode):

```
v_min_seed   = _min_speed_within(idx, seed_lookahead_m_or_uniform)
phys_la      = safety_factor * (v**2 - v_min_seed**2) / (2 * a_brake_avail_ms2)
lookahead_m  = clip(phys_la, min_lookahead_m, seed_horizon)   # floor & upper-bound
v_target_min = _min_speed_within(idx, lookahead_m)
# brake = clip(brake_p_gain * max(v_target_min - v, 0), 0, 1)   # UNCHANGED
```

No changes to topics, file formats, or the trace schema. `preview_time_s` and
`brake_p_gain` semantics are preserved (`preview_time_s` now also feeds the seed
horizon when `seed_lookahead_m` is null).

All `lap.py` invocations in this spec use **Bash / plain `python`** (PowerShell is
blocked):

```
python lap.py cars_csv/bmw_1m \
  tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv \
  drivers/tomas_highgrip.json \
  --model slip --controller reactive --single-lap --no-plot --no-telemetry \
  --inertia-zz 2400
```

Stock regression run swaps in `drivers/tomas.json` and (for the on/off A/B) a
scratch clone of `tomas.json` carrying `preview.mode = "adaptive"`.

## 8. Risks, constraints, and open questions

- **Seed-horizon adequacy (R1):** if the seed horizon is shorter than the true
  braking distance the physics pass can't lengthen past it (it only shrinks).
  Mitigation: seed = the legacy uniform window, which already finishes at
  `preview_time_s = 2.3`; default the high-grip seed to ≥ that, or set
  `seed_lookahead_m` to a generous fixed value (e.g. the worst-case
  v²/2a for the hairpin, ~90–100 m). **Open question:** is the 1.5 s default
  seed enough, or must the high-grip driver keep `preview_time_s = 2.3` (or set
  `seed_lookahead_m` explicitly)?
- **a_brake_avail mismatch (R2):** too high → brakes too late, hairpin abort
  returns; too low → over-conservative, gap doesn't close. Mitigated by the
  sweep (sub-feature 4) and anchored to the measured ~13 m/s² chicane decel.
- **Friction-ellipse coupling (R3):** during heavy braking *while* turning, the
  available longitudinal decel drops (lateral demand eats the ellipse). A single
  constant `a_brake_avail` ignores this. Mitigation: `safety_factor` absorbs it;
  if it proves insufficient at the hairpin, consider a corner-aware derate as a
  follow-up (out of scope here).
- **Two-pass stability (R4):** confirm a single fixed-point iteration converges
  (it does, since the seed is a monotone upper bound). Do not loop.
- **Regression exactness (R5):** the `uniform` branch must be byte-identical to
  today's code path. Mitigation: gate at the top of the branch so no adaptive
  math executes when `mode != "adaptive"`; verify stock lap time/σ unchanged.
- **Open question:** should `min_lookahead_m` stay at 30 m for high-grip, or
  does the higher entry speed warrant a larger floor? Resolve in the sweep.

## 9. Alternatives considered

- **Formulation 2 — curvature-scheduled preview** (`preview_time_s = f(upcoming
  radius/curvature)`, short for large radius, long for tight corners):
  conceptually simple but **not recommended**. (a) It needs a curvature source:
  the controller currently holds no radius array — curvature would have to be
  derived on the fly from `_xs/_ys` (second differences, noise-prone) or the
  planner output extended to carry radius (a contract change this spec's
  non-goals forbid). (b) It needs a hand-tuned radius→time mapping/threshold,
  which is more parameters and less principled than v²/2a. (c) It schedules on
  geometry, not on the actual speed drop — a tight but slow-throughout corner
  (no big delta-v) would still trigger a long preview unnecessarily, whereas
  Formulation 1 keys directly on the quantity that matters (the speed drop). It
  remains a fallback if the physics formulation underperforms.
- **Uniform `preview_time_s` (status quo):** the lever already exhausted —
  finishes at 111.92 s, structurally +1.36 s over target. Rejected as the
  binding limitation this spec exists to remove.
- **Switch to HMPC/MPC inner with explicit preview braking:** decouples
  corner-entry braking from a global horizon and is the "proper" long-term fix,
  but is a far larger change and out of scope for closing this gap on the
  reactive controller. Tracked as a future option.
- **Planner brake-distance feasibility cap on plan v_max:** a planner change
  (non-goal); also caps pace rather than recovering it.

## 10. References

- `docs/architecture-v3-reactive-highgrip-ideal-line.md` — diagnosis + the
  brake-authority addendum (the 111.92 s / `preview_time_s = 2.3` result; brake
  magnitude is a red herring; `pedal_press_rate_per_s` inert in reactive).
- `docs/architecture-v3-tyre-grip-envelope-fix.md` — the D×1.28 grip fix baked
  into `drivers/tomas_highgrip.json`.
- `src/lap_estimator/dynamics/driver_controller.py:385-418` — the reactive
  preview + brake path to modify; `:612-617` — `_min_speed_within`.
- `src/lap_estimator/dynamics/_control_params.py:54-244` — `ControlParams`
  dataclass + `from_driver` (pattern for the new `preview` sub-block).
- `drivers/tomas_highgrip.json` — high-grip opt-in carrier; `grip_envelope_fix`
  note records the ~13 m/s² chicane decel anchor for `a_brake_avail`.
- `drivers/tomas.json` — stock reactive baseline (~2:09 / 10-of-10).
- MEMORY: ideal-line ceiling 110.88 s; Tomas 107.56 s; target ≤ 110.56 s.
```