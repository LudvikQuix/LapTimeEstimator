# Spec §23.10 — v3.1 controller upgrade (close the Phase 4 gap)

**Parent spec:** `dev-planning/lap-simulation-csv-driver/spec-section-23-slip-model.md`
**Status:** Draft (additive; lives as §23.10 inside the v3 section)
**Project:** LapTimeEstimator
**Branch:** feature/sc-71955/lap-simulation
**Created:** 2026-05-18
**Planned with:** Buddy

This section is the post-mortem and remediation plan for **Phase 4 of §23.9**.
Phase 4 shipped a reactive Stanley-steer + P-controller longitudinal loop and
did not pass its acceptance gate (§11.55, ±3 s vs real). This file scopes the
controller redesign that will. It is **strictly a controller change** —
Pacejka calibration, ODE solver, weight transfer, tyre-state plumbing, and
the line itself are all unchanged. v3 still tracks an externally-given racing
line; free-driving / line selection remains the v3.2 backlog.

Cross-references: §23.6 (ODE solver), §23.7 (Pacejka fit), §23.8 (controller),
§23.9 (phasing), §11.55 (headline gate). The Decisions item §27 from §23.12
is amended at the bottom of this file (§23.10.10).

---

## §23.10.1 Why Phase 4 failed — measured root cause

The Phase 4 acceptance gate (§11.55) requires Tomas on Sprint A to come in
within ±3 s of his real lake average (1:47.56). Latest measurements
(2026-05-18):

| Configuration                                         | Lap time | Δ vs real |
|---|---|---|
| Real (Tomas lake average, Sprint A)                   | 1:47.56  | —         |
| v2 point-mass (current best)                          | 1:48.05  | +0.49 s   |
| v3 current (Phase 4 controller, measured Pacejka)     | 2:09.46  | +21.9 s   |
| v3 with diagnostic D_lat=1.28 override (max plausible)| 2:00.92  | +13.4 s   |
| v3 with D_lat=1.28 **and** D_long=1.25 overrides      | 2:00.98  | +13.4 s   |

Key observations:

1. **Pacejka isn't the bottleneck.** Doubling down on D_long (1.25 override)
   delivered **zero** lap-time improvement on top of the D_lat override. The
   longitudinal envelope is large enough already.
2. **The controller has lateral grip slack and doesn't use it.** With
   D_lat=1.28 override, observed `util_p85 = 1.12` — i.e. the *Pacejka tyre*
   can deliver 12% more lateral force than the controller asks for, on the
   85th percentile of cornering effort. The reactive P-loop simply doesn't
   command enough lateral acceleration.
3. **The `target_scale` heuristic compounds the loss.**
   `slip_simulator._make_controller` derives `target_scale ∈ [0.70, 1.00]`
   from the fitted `D_per_Fz` (current value ~0.85 for Tomas+Semislicks).
   That's a ~7-8 s tax baked in before the controller even starts.
4. **Ghost-fallbacks cluster at one corner exit every MC run.** At
   `t ≈ 20.9 s`, slip-ratio multiplier ≈ 3.0 (front α = 12° vs target 4.4°),
   the controller hands off to GhostDriver. Same corner, every seed.
5. **The slip target is conservative.** `derived_slip_target_deg = 6.0·skill
   + 1.0·(1-skill)` gives `slip_target = 4.4°` for Tomas (skill=0.68). The
   fitted Pacejka has α_peak ≈ 6.3° on the front axle. The controller is
   under-using the tyre by ~30% of the slip envelope on purpose.

**Root cause synthesis.** A reactive P-controller cannot honestly track v2's
friction-circle-optimal speed plan. v2 is a 3-pass distance-marched optimizer
with *perfect lookahead* — it knows the corner speed before it arrives. v3
RK4 + reactive P knows the corner only after it has measured slip climbing,
by which time it is already past the brake point. Compounded by a
conservative slip target and a `target_scale` band-aid, the controller leaves
~13-22 s on the table even with honest tyre physics.

---

## §23.10.2 Goals (v3.1)

- Close the Phase 4 gap: Tomas on Sprint A `--model slip --skill-pct 1.0` ≤
  **±3 s** vs real lake average. **Same acceptance threshold as §11.55**, now
  realistically reachable.
- Track an externally-given racing line (v2 plan or recorded line) with
  high fidelity — the controller no longer *under-utilises* the tyre when
  given a good plan.
- Preserve every byte of the Phase 2 Pacejka calibration. **No refit
  required.** A v3.1 controller running against the existing
  `pacejka_calibration` block in `drivers/tomas.json` (fit on 2026-05-15)
  must Just Work.
- Remove the `target_scale` heuristic from `_make_controller`. Once the
  controller honestly tracks the v2 plan, scale should always be 1.0 (with
  a small documented safety margin for transients — §23.10.6).
- Derive `slip_target_lat_deg` from the **fitted Pacejka α_peak**, not from
  the v3.0 hand-coded `6.0·skill + 1.0` formula.

## §23.10.3 Non-goals (still v3.2 or out of scope)

- **Free driving / line selection.** v3.1 still consumes an external speed
  plan (v2 plan or recorded). Full MPC that chooses the line is v3.2.
- **Aero refinement, differential modelling, pit-stop compound change,
  driver fatigue** — unchanged from §23.2.
- **New driver-fit pipeline.** v3.1 reads the Pacejka block written by
  Phase 2; it does not re-fit anything. The `--fit-pacejka` code path is
  untouched.
- **Lake schema, track CSV, setup JSON** — all unchanged.
- **Stint mode behaviour** — unchanged (per-lap state passthrough still
  works; only the per-lap controller is replaced).

---

## §23.10.4 Approach decision — preview longitudinal + DP plan, in that order

Four design dimensions were considered:

1. **Lookahead longitudinal P-controller** — replace reactive throttle/brake
   with a 1-2 s preview that pre-brakes before corners.
2. **Single-shot longitudinal DP** — solve minimum-time longitudinal pass
   over the lap given the precomputed line and Pacejka envelope.
3. **Full MPC** (free-driving) — out of scope, v3.2.
4. **Remove `target_scale` heuristic** — band-aid for poor tracking.
5. **Slip-target from α_peak** — controller is conservative today.

**Recommendation: Path 1 + Path 5 first (Phase 4.1), then Path 2 if needed
(Phase 4.2). Drop `target_scale` (Path 4) in Phase 4.1. Path 3 remains v3.2.**

Rationale:

- Phase 4.1 (preview + α_peak-derived slip target + drop `target_scale`) is
  small (≤1 day ArchDev), reuses the existing v2 speed plan as the target,
  and is the single biggest expected improvement. Targets the diagnostic
  ceiling of ~2:00.9 → ~1:49-50.
- If Phase 4.1 doesn't close to ±3 s, Phase 4.2 (longitudinal DP over the
  pre-fitted Pacejka envelope) is the principled fix: compute the
  minimum-time speed-vs-distance for *this* tyre, then have the controller
  track *that* plan instead of the v2 plan. This handles the case where v2's
  plan is itself slightly off for the measured Pacejka envelope.
- Phase 4.3 (full MPC) defers cleanly to v3.2 — it is the same code path as
  free-driving, scoping it together is the right factoring.

The phasing is sequential because **we want to know whether the simpler fix
suffices** before building DP. If Phase 4.1 hits ±3 s, Phase 4.2 ships as
v3.2 with free-driving instead of an interim release.

---

## §23.10.5 Phase 4.1 — preview longitudinal + α_peak slip target

### 23.10.5.1 New controller behaviour

Replace `driver_controller.DriverController.controls()` longitudinal half
(lines 216-313 of the current implementation). Steering channel and rate
limits are **unchanged** (Stanley with preview tangent works — the bug is
strictly longitudinal).

The new throttle/brake loop:

1. **Preview window for target speed.** Currently
   `_min_speed_within(idx, brake_lookahead_m)` already does a v²-scaled
   lookahead. **Keep it**, but extend the lookahead horizon:
   - Current: `max(20.0, v² / (2·5.0))` — assumes 5 m/s² braking, which is
     conservative given the tyre.
   - New: `max(30.0, v_preview · t_preview)` where
     - `t_preview = control_params.preview_time_s` (new field, default 1.5 s).
     - `v_preview = max(v, target_v_local)` so the lookahead grows in fast
       sections even before the controller has built speed.
2. **Pre-brake bias.** Compute the *minimum* target speed in the preview
   window (call it `v_target_min`) and the *current line* target speed
   (`v_target_now`). When `v_target_min < v` (i.e. a slow corner is upcoming),
   the controller starts braking now even if `v_target_now > v`:
   - `e_v_effective = v_target_min - v` (vs current `v_target_local - v`).
   - This is the single biggest change. Today the controller only brakes
     when `v > v_target_local`; tomorrow it brakes whenever
     `v > v_target_min` over the preview window.
3. **Throttle ramp from skill / pedal-rate (already measured by §1.2 driver
   fitting).** Plug `driver.profile.dynamic.throttle_ramp_*` (already in JSON
   from v1.2) into a throttle rate limit. Today the P-controller has no rate
   limit on throttle; v1.2-measured pedal slew should govern it.
4. **Drop the `target_scale` band-aid.** In `_make_controller`:
   - Delete the `target_scale = sqrt(D / mu_v2)` block.
   - Hard-code `target_scale = 1.0` for v3.1.
   - Document the deletion with a comment pointing here.
5. **Drop the in-controller `slip_ratio > 0.85` feedforward clamp.** Lines
   270-291 of current `driver_controller.py` clamp `v_target` toward `v` once
   slip is high. With proper pre-braking, the controller will not be deep
   into slip in the first place. The slip-band throttle modulators
   (UNDER/OVER/HARD_CAP at lines 293-304) **stay** as the post-hoc
   protection layer, but the speed-target clamp goes.

### 23.10.5.2 α_peak-derived slip target (skill mapping)

Replace the v3.0 hand-coded skill→slip-target formula. New mapping lives in
`driver.derived_slip_target_deg()` and reads the Pacejka block:

```
if pacejka_calibration is present and source.measured == True:
    alpha_peak_front_rad = argmax(pacejka_fy(alpha, Fz_median, coeffs_front_lat))
                                 over alpha in [0, 15°]
    alpha_peak_front_deg = degrees(alpha_peak_front_rad)
    slip_target_deg = alpha_peak_front_deg * (0.5 + 0.5 * skill_pct)
        # skill=1.0 → 100% of peak  (e.g. 6.3° for Tomas)
        # skill=0.5 → 75% of peak   (e.g. 4.7°)
        # skill=0.0 → 50% of peak   (e.g. 3.15°)
else:
    # fallback for drivers without measured pacejka_calibration
    slip_target_deg = 6.0 * skill_pct + 1.0 * (1.0 - skill_pct)
        # unchanged v3.0 formula
```

Rationale:
- The original formula was a guess at α_peak (6°) — now we *know* α_peak
  from the fit.
- Linear interpolation between 50% and 100% of peak keeps low-skill drivers
  in the linear regime, matching the v3.0 intent.
- Tomas (skill=0.68): old slip_target=4.4°, new slip_target ≈ 5.4° (assuming
  α_peak=6.3°). The controller will load the tyre ~22% harder on average.

### 23.10.5.3 Concrete interface changes

**`control_params` block (driver JSON, §23.5.2) — new optional field:**

```jsonc
{
  "control_params": {
    "preview_distance_m": 18.0,
    "preview_time_s": 1.5,           // NEW — Phase 4.1 longitudinal preview horizon
    "steering_p_gain": 1.2,
    "throttle_p_gain": 0.5,
    "brake_p_gain": 0.6,
    "throttle_rate_limit_pct_s": 250, // NEW — optional; defaults from driver.profile.dynamic.throttle_ramp_pct_s
    "slip_target_lat_deg": 6.0,
    "consistency_noise_std_steer_deg": 0.3,
    "consistency_noise_std_throttle_pct": 1.5,
    "consistency_noise_std_brake_pct": 1.5,
    "measured": false
  }
}
```

- `preview_time_s`: defaults to 1.5 s if absent. Range [0.5, 3.0]. Drivers
  fit before this spec ships have no field → default applies, no migration
  needed.
- `throttle_rate_limit_pct_s`: defaults to whatever the v1.2 driver fit
  measured in `profile.dynamic.throttle_ramp_pct_s` (already on disk for
  Tomas). Falls back to 1000 (effectively unlimited) if both are absent.

**`pacejka_calibration.source` — new computed-once field (optional):**

```jsonc
{
  "pacejka_calibration": {
    "source": { ..., "alpha_peak_front_deg": 6.3, "alpha_peak_rear_deg": 6.0 }
  }
}
```

- Computed at end of Stage E (cross-validation) in `pacejka_fit.py`. The
  controller reads it if present; otherwise it recomputes once on first
  use and caches on the `Driver` object. **No refit required** — the field
  is derivable from the existing `(B, C, D, E)`. Phase 4.1 implementation
  augments `fit_driver.py` to write it on next fit; existing JSONs without
  the field still work via the lazy-compute path.

**CLI (`lap.py`) — no surface change.** `--model slip` continues to dispatch
to `simulate_slip`; the new controller is transparent.

**Python interface (`DriverController.__init__`) — no surface change.**
`target_speed_scale` kwarg is **kept** (so external callers still compile)
but `_make_controller` now always passes `1.0`. The kwarg is marked
`DeprecatedKwarg` in the v3.1 docstring; removal is a v3.2 ABI cleanup.

### 23.10.5.4 Phase 4.1 acceptance gate (replaces §11.55)

A revised §11.55 is the headline:

> **§11.55 (Phase 4.1, headline):** Tomas on Sprint A `--model slip
> --skill-pct 1.0` produces a lap time within **±3 s** of his real lake
> average (1:47.56 → target 1:44.5–1:50.5). `util_p85 ≤ 1.05` honestly
> computed (no clamping). Ghost-fallback step count ≤ **20** per lap
> (down from the current ~100s at one corner). MC-σ (10 runs at skill=1.0)
> ≤ 0.8 s — proving the controller is consistent under noise.

Plus two supporting checks:

- **§11.55.A (Phase 4.1, supporting):** `target_scale` is hard-coded 1.0 in
  `_make_controller`. Inspecting the generated trace CSV, `target_speed[i]`
  equals `v2_plan.speeds[i_nearest]` to within numerical precision.
- **§11.55.B (Phase 4.1, supporting):** With `--skill-pct 0.5`, lap time is
  ≥3 s slower than skill=1.0 (preserves §11.56). And the measured average
  front α stays within `0.5·α_peak ± 1°` (proving the new slip target works).

**Fail action.** If Phase 4.1 misses ±3 s but lands inside ±5 s, ship 4.1 and
move to Phase 4.2. If it misses ±5 s, post-mortem and re-plan before 4.2.

---

## §23.10.6 Phase 4.2 — single-shot longitudinal DP plan (only if 4.1 misses)

### 23.10.6.1 Trigger

Phase 4.2 ships only if Phase 4.1 closes to within ±5 s but not ±3 s. The
hypothesis: the v2 plan itself is slightly off for the measured Pacejka
envelope (v2's `mu` is a scalar; v3's Pacejka has more shape than a circle).
A DP pass against the actual Pacejka envelope will give the controller a
target plan it can hit *and* that is minimum-time for the modeled tyre.

### 23.10.6.2 Algorithm

Single forward-backward dynamic programming pass over the racing line:

1. **Discretise.** Use the existing track CSV ideal-line distance samples
   (typically 5 m resolution, ~700 points for Sprint A).
2. **Backward pass — max corner speed.** At each line point i, compute the
   max speed the tyre can hold given line curvature κ_i and the Pacejka
   lateral envelope at that point (use `D_lat · Fz_static` as the ceiling;
   Phase 4.2 doesn't model weight transfer in the planner — that stays in
   the simulator). `v_max[i] = sqrt(D_lat · g / |κ_i|)` clamped to car top
   speed.
3. **Backward pass — brake-feasibility.** Walk i from N-1 down to 0. At each
   step, `v_brake_feasible[i] = sqrt(v_max[i+1]² + 2·a_brake·ds)`, where
   `a_brake` is the combined-slip-aware longitudinal deceleration available
   given the current lateral demand. `v_plan[i] = min(v_max[i],
   v_brake_feasible[i])`.
4. **Forward pass — throttle-feasibility.** Walk i from 0 up to N-1. At each
   step, `v_throttle_feasible[i+1] = sqrt(v_plan[i]² + 2·a_throttle·ds)`
   where `a_throttle` is the combined-slip-aware longitudinal acceleration
   available. `v_plan[i+1] = min(v_plan[i+1], v_throttle_feasible[i+1])`.
5. **Output.** A `(distances, speeds)` array shaped exactly like the existing
   `v2_plan` so the controller's lookup machinery doesn't change. The
   controller's `target_speed_ds`/`target_speeds` ctor kwargs receive this
   instead.

This is **the same algorithm v2 uses** (`simulator.py`'s 3-pass) except
applied to the Pacejka envelope instead of the v2 grip circle. ~150 lines.

### 23.10.6.3 Module placement

New file: `src/lap_estimator/dynamics/longitudinal_planner.py`.

Public surface:
```
def plan_longitudinal(
    car, track, calib, *, safety_margin: float = 0.97
) -> LongitudinalPlan
```

`LongitudinalPlan` is a dataclass with `.distances: np.ndarray`,
`.speeds: np.ndarray` — same shape as the v2 plan. The `safety_margin`
parameter (default 0.97) is the documented small headroom the controller
needs to absorb transient overshoot — this is the *honest* analog of the old
`target_scale` band-aid: 3% safety, configurable, justified.

`_make_controller` in `slip_simulator.py` gains a new flag:
- `plan_source = "v2" | "v3_dp"`, default `"v3_dp"` once Phase 4.2 ships.
- v2 plan path stays for regression / debugging via
  `--plan-source v2` CLI flag.

### 23.10.6.4 CLI surface (Phase 4.2 only)

New flag: `--plan-source {v2, v3_dp}`, default `v3_dp` post-4.2.
Compatibility: `--model point-mass` ignores this flag (always v2 plan).
Phase 4.1 ships without this flag; flag lands with 4.2.

### 23.10.6.5 Acceptance gate (Phase 4.2)

Same headline §11.55 (±3 s). Plus:

- **§11.55.C (Phase 4.2, supporting):** `--plan-source v2` still produces
  the Phase 4.1 result (regression).
- **§11.55.D (Phase 4.2, supporting):** `--plan-source v3_dp` plan v.s. v2
  plan: when overlaid in the validation PNG, the v3_dp plan brakes
  *earlier* and apex-speeds are within ±2 m/s of v2 (sanity — same physics,
  similar plan).

---

## §23.10.7 v3.2 backlog — full MPC / free-driving (now scoped)

Full v3.2 spec lives at
[`spec-section-23-2-v32-mpc.md`](spec-section-23-2-v32-mpc.md) (Phase 5.0
line-following MPC; Phase 5.1 free-driving as stretch).

---

## §23.10.8 Migration & back-compat

- **Pacejka calibration.** The existing `pacejka_calibration` block in
  `drivers/tomas.json` (fit 2026-05-15) is consumed verbatim. The new
  `source.alpha_peak_front_deg` / `_rear_deg` fields are computed lazily on
  load if absent, cached on the `Driver` object. **No refit required.**
- **Driver JSON `control_params`.** The two new fields (`preview_time_s`,
  `throttle_rate_limit_pct_s`) default sensibly when absent. Existing JSONs
  without `control_params` at all still get the v3.0 defaults plus the new
  defaults — net result: a v3.0 JSON runs under v3.1.
- **`target_speed_scale` kwarg on `DriverController`.** Retained for ABI
  compat; documented `DeprecatedKwarg`; `_make_controller` always passes
  1.0. Removal scheduled for v3.2.
- **v2 path.** `--model point-mass` is untouched. Byte-equivalent to today.
- **Phase 2 fit pipeline.** Untouched. The `--fit-pacejka` flag in
  `fit_driver.py` is augmented to emit `alpha_peak_*` into the
  calibration block on next fit; existing JSONs simply lazy-compute on
  first run.
- **Web UI (§22 + §23.4.2).** No surface changes for Phase 4.1. The Sim tab
  dropdown still says "Slip-based dynamics (v3 — slower, honest physics)".
  Phase 4.2 adds an optional "Plan source: v2 / v3 DP" radio under the
  Sim tab (collapsed by default). v3.2 will rework the controller picker.

---

## §23.10.9 Risks, open questions

### Risks

1. **The v2 plan itself might be too aggressive** for the measured Pacejka
   envelope (v2's grip circle is `mu_v2 ≈ 1.20`; fitted D_lat ≈ 1.03). If so,
   Phase 4.1 will brake on time but exit corners under-grip — symptom is
   lap-time floor at ~1:51-52 even with perfect controller. Mitigation:
   Phase 4.2 ships. Detection: §11.55.D-style overlay comparing fitted
   D_lat-derived `v_max[i]` against v2's `speed_ms[i]`.
2. **α_peak from a fitted curve is sensitive to E.** Some fits land
   `E ≈ -2.0` (the lower bound) with α_peak at the curve's monotonic edge.
   Mitigation: clamp α_peak between 4° and 10° before feeding the skill
   mapping; warn if clamped.
3. **Throttle rate limit interacting with the existing slip-band
   modulator** at line 293-304 of `driver_controller.py`. If both fire
   simultaneously (rate-limit says +0.05, slip-band says ×0.85), behaviour
   is order-dependent. Mitigation: apply rate limit *first*, then slip-band
   modulators on the rate-limited output.
4. **Ghost-fallback at corner exit might not vanish.** If pre-braking helps
   entry but the corner-exit slip excursion is a tyre-load issue (low Fz on
   inside-rear during throttle-up), Phase 4.1 won't fix it. Mitigation:
   the slip-band modulators stay as the safety layer; the ghost fallback
   stays for `> 3·slip_target` and yaw `> 4 rad/s`. Acceptance gate caps
   fallback step count at 20/lap to force the issue if it persists.

### Open questions (to resolve during build, not before)

- **Should `preview_time_s` be skill-modulated?** A high-skill driver looks
  further ahead. v3.1 keeps it constant per driver and revisits if §11.55.A
  shows a residual. Default 1.5 s applies to all drivers.
- **Should the DP planner (Phase 4.2) include weight transfer?** v3.1
  proposal is "no" — the planner uses static Fz, the simulator handles the
  transients. If Phase 4.2 misses ±3 s, revisit by adding a single
  quasi-static weight-transfer pass to the DP backward sweep.
- **Skill = 0 driver under α_peak mapping.** 50% of α_peak ≈ 3.15° for
  Tomas. Is that "novice" enough? Probably yes; revisit if a skill=0 lap
  comes in unrealistically fast on Sprint A.

---

## §23.10.10 Amended Decisions block (replaces v3.0 §27)

Replace the v3.0 §27 in the main `spec.md` Decisions block with:

> **27. (v3) Slip-based dynamics model — parallel track, opt-in.** (v3.0
> wording kept verbatim through "minimum-time controllers (MPC / single-step
> DP) are v3.1.") **v3.1 specifics:** the v3.0 reactive P-controller did not
> pass §11.55 (Tomas Sprint A skill=1.0 came in at 2:09.46 vs real 1:47.56,
> Δ +21.9 s, even after diagnostic Pacejka overrides). The v3.1 controller
> upgrade (§23.10) ships in phases: **Phase 4.1** — preview longitudinal
> P-controller with `preview_time_s` lookahead, throttle rate-limit from the
> v1.2-measured pedal slew, α_peak-derived slip target (replaces the v3.0
> `6.0·skill + 1.0` formula), and removal of the `target_scale` band-aid in
> `_make_controller` (forced to 1.0); **Phase 4.2** — single-shot
> longitudinal DP planner against the fitted Pacejka envelope, swapping the
> v2 plan as controller target (only if 4.1 misses ±3 s); **Phase 4.3
> (next-attempted, before v3.2)** — slip-aware steering softener (§23.10.12),
> a cheap controller-side scalar gain that softens commanded δ as front α
> approaches α_peak, accepting larger cross-track error in exchange for
> keeping the tyre near peak. Phase 4.3 is the last cheap trick attempted
> before escalating to v3.2 (full MPC / free-driving). v3.1 still consumes
> an externally-given racing line; free-driving / line selection is v3.2.
> v3.1 does **not** invalidate the Phase 2 Pacejka fit — the
> `pacejka_calibration` block on disk is consumed verbatim and augmented
> with lazily-computed `alpha_peak_front_deg` / `alpha_peak_rear_deg`
> fields. `control_params` gains optional `preview_time_s`,
> `throttle_rate_limit_pct_s`, `steering_softener_engage`, and
> `steering_softener_full` fields; absent fields default sensibly.
> Phase 4.1, 4.2, and 4.3 share the same revised headline gate §11.55
> (±3 s of real, `util_p85 ≤ 1.05`, ghost-fallback steps ≤ 20/lap, MC-σ
> ≤ 0.8 s); Phase 4.3 adds supporting check §11.55.E (max cross-track
> error ≤ 4 m). (§23.10.1–§23.10.12, §11.55, §11.55.A–E.)

---

## §23.10.12 Phase 4.3 — slip-aware steering softener (cheap trick before v3.2)

**Trigger.** Phase 4.1 + 4.2 shipped 2026-05-18. Measured lap = 2:04.14
(Δ +16.6 s vs real 1:47.56). Acceptance gate §11.55 misses (±3 s). ArchDev's
post-4.2 diagnosis: the controller is the binding constraint, not the plan
or the tyre. `util_p85 = 2.74` means front α reaches ~19° (vs α_peak ≈ 6.9°
and `slip_target = 6.9°`). Lowering DP `safety_margin` from 0.97 → 0.88 only
floors lap at 2:06.6 with `util = 1.10` — controller-limited regardless of
plan headroom. Root mechanism: Stanley preview-target steering chases
cross-track error, so the actual radius is tighter than κ_plan → demanded α
exceeds α_peak → tyre force collapses on the falling side of the Pacejka
curve → ghost-fallback. The principled fix is v3.2 full MPC (§23.10.7).
Phase 4.3 is **one cheap controller-side trick attempted first** before that
escalation.

### §23.10.12.1 Mechanism

After Stanley computes raw δ_cmd, *before* steering rate-limit and *before*
consistency-noise injection, apply a multiplicative softener gain `soften`
that is 1.0 when the front axle is below `softener_engage · α_peak`, decays
linearly to 0.0 at `softener_full · α_peak`, and saturates at 0.0 above:

```
alpha_front_meas = 0.5 * (alpha_FL_prev + alpha_FR_prev)     # measured last step
alpha_norm       = abs(alpha_front_meas) / alpha_peak_front_rad
if alpha_norm <= softener_engage:
    soften = 1.0
elif alpha_norm >= softener_full:
    soften = 0.0
else:
    soften = (softener_full - alpha_norm) / (softener_full - softener_engage)
delta_cmd *= soften
```

Notes:
- **Measurement source.** `alpha_FL_prev` / `alpha_FR_prev` come from the
  previous ODE step's tyre state already plumbed into the controller for
  the slip-band throttle modulators. No new state.
- **Blend curve.** Linear, not smoothstep. Linear is one parameter simpler
  and the corner-of-engagement is well outside the linear regime anyway —
  smoothstep buys nothing measurable.
- **Sign convention.** `soften` is applied to the post-Stanley δ before any
  clip or rate-limit; this preserves sign and only attenuates magnitude.
- **Hysteresis.** None. The 0.85 → 1.05 range is wide enough that
  instantaneous-α flicker rarely crosses the band twice per step at 10 ms
  control rate; if MC-σ shows steering chatter, revisit with a 50 ms LP on
  `alpha_norm`.

### §23.10.12.2 Where to apply (driver_controller.py)

Insertion point: `DriverController.controls()`, immediately after the
Stanley computation (line 236 — assignment of `steer_cmd`), **before** the
`max_steer_rad` clip (line 243) and **before** the steering rate-limit block
(lines 249-254). Concretely, the softener block lives between current
lines 236 and 243. The downstream consistency-noise block (lines 387-409,
specifically the steer_noise application around lines 393-405) remains
untouched — noise is applied on the softened, rate-limited δ.

Order of operations after Phase 4.3:
1. Stanley raw δ_cmd (line 236)
2. **NEW: softener gain** (between 236 and 243)
3. `max_steer_rad` clip (line 243)
4. Steering rate-limit (lines 249-254)
5. Consistency noise (lines 393-405)

### §23.10.12.3 Defaults and justification

Recommended defaults: `steering_softener_engage = 0.85`,
`steering_softener_full = 1.05`.

Justification:
- **`engage = 0.85`** keeps the softener inactive across the entire useful
  cornering range of skill ≤ 1.0 drivers. At skill=1.0, `slip_target =
  α_peak`, so the driver intentionally cruises at α/α_peak ≈ 1.0; the
  softener only fires once *measured* α has exceeded the target by ~15%
  (i.e. when Stanley is over-steering past plan). Lower-skill drivers
  (skill < 1.0) have `slip_target < α_peak` and will hit `engage = 0.85`
  later in absolute α terms — they get all of the plan's headroom before
  the softener intervenes. No regression for low-skill drivers.
- **`full = 1.05`** gives a small (5%) overshoot tolerance before steering
  is fully suppressed. Hard zero at exactly α_peak would mean the
  controller cannot recover from any α excursion that crosses peak (which
  *will* happen on every transient). 5% over peak is a comfortable
  Pacejka-falling-side region where reducing δ unloads the tyre back below
  peak within one or two ODE steps.
- **Width = 0.20** (1.05 − 0.85) gives a graceful linear ramp; narrower
  bands create stiff toggling that interacts badly with steering
  rate-limit.

### §23.10.12.4 `control_params` schema additions

```jsonc
{
  "control_params": {
    "...": "...",
    "steering_softener_engage": 0.85,  // NEW — α_norm where softener begins (defaults 0.85)
    "steering_softener_full":   1.05   // NEW — α_norm where softener saturates at 0 (defaults 1.05)
  }
}
```

- Both fields optional. Absent → defaults apply.
- Validation: `0.5 ≤ engage < full ≤ 1.5`. Out-of-range values reject at
  driver-load time with a clear message.
- A new field `steering_softener_engage = 1.5` effectively disables the
  softener (it never engages); document this as the kill-switch.
- Schema is additive — every driver JSON on disk today (Tomas, Ludvík, all
  archived stints) loads under v3.1 + Phase 4.3 unchanged.

### §23.10.12.5 Interaction with existing slip-band throttle modulators

The current UNDER/OVER/HARD_CAP throttle band (lines 293-304 of
`driver_controller.py`) fires on
`slip_ratio = abs(alpha_front_avg) / slip_target_rad`.
The new softener fires on
`alpha_norm   = abs(alpha_front_avg) / alpha_peak_front_rad`.

**These are different denominators.** With the v3.1 α_peak-derived skill
mapping:
- `slip_target = α_peak · (0.5 + 0.5 · skill_pct)`.
- At skill = 1.0, `slip_target = α_peak` → `slip_ratio = alpha_norm` exactly.
- At skill < 1.0, `slip_target < α_peak` → `slip_ratio > alpha_norm`. The
  throttle band fires *earlier* (in absolute α terms) than the softener,
  which is the correct ordering: throttle is the cheaper actuator to back
  off, steering is the last lever.

**Decision: keep them independent.** Slip-band modulates throttle; softener
modulates steering. Different physics (long vs lat force), different
actuators, different decision logic. Unifying them onto one denominator
would either over-aggressively soften steering on low-skill drivers
(if unified on `slip_target`) or under-aggressively cut throttle on
high-skill drivers (if unified on `α_peak`). Independent is correct.

Order in `controls()`: softener fires inside the steering block (between
lines 236 and 243), well before the slip-band throttle block at lines
293-304. No ordering coupling.

### §23.10.12.6 Risks and mitigations

1. **Cross-track drift → OffTrackError aborts.** Suppressing δ near α_peak
   means the car accepts larger cross-track error. If error exceeds the
   simulator's current track-edge tolerance (~50 m, effectively unlimited
   on Sprint A), nothing aborts; if it exceeds a *tighter* future bound,
   the run aborts.
   - **Mitigation A.** Add a soft cap: if cross-track error exceeds 8 m
     for more than 0.5 s, log a warning trace event but do not abort.
     Tighten to a hard abort only if §11.55.E shows error reliably > 4 m.
   - **Mitigation B.** The softener does not zero δ until α_norm > 1.05;
     transient over-α is rare and short. Sustained off-track requires
     sustained over-α, which is itself the failure case Phase 4.3 is meant
     to expose.
2. **Steering chatter under noise.** With consistency noise on δ applied
   *after* softening, the rate-limit (line 249) absorbs the chatter. If
   MC-σ deteriorates beyond 0.8 s, low-pass `alpha_norm` over 50 ms before
   the softener fires.
3. **Slow corners (κ small, v low) cause α to climb late.** The softener
   is symmetric in α, so it works the same at any speed; the only
   concern is if very low-speed manoeuvres (pit lane, T1 hairpin) trigger
   premature engagement. Spot-check the trace at the slowest two corners
   of Sprint A — if `α_norm` crosses 0.85 there, raise `engage` to 0.90.
4. **Interaction with ghost-fallback.** Today the ghost driver triggers at
   `slip_ratio > 3.0` or yaw > 4 rad/s. With the softener active,
   `slip_ratio` should rarely cross 3.0 → ghost-fallback events should
   drop. If they don't, the softener is too weak; widen the band
   (`engage = 0.80`, `full = 1.10`).

### §23.10.12.7 Acceptance gate

Headline §11.55 (Phase 4.3) — same as §23.10.5.4:
- ±3 s of real lake average (1:44.5–1:50.5).
- `util_p85 ≤ 1.05` honestly computed.
- Ghost-fallback step count ≤ 20 per lap.
- MC-σ (10 runs at skill=1.0) ≤ 0.8 s.

Plus one new supporting check:

- **§11.55.E (Phase 4.3, supporting):** Max absolute cross-track error
  reported in the per-step trace CSV is ≤ **4 m** across the lap. This
  proves the softener is keeping the tyre near peak *without* drifting the
  car off the geometry. If util_p85 closes but cross-track error blows
  through 4 m, the softener is trading lap-time for off-track distance —
  not acceptable.

### §23.10.12.8 Fail action

- **If lap lands inside ±5 s but outside ±3 s, and §11.55.E passes:** ship
  Phase 4.3, call it the best cheap-trick result, move v3.2 (full MPC) to
  the top of the backlog.
- **If lap lands outside ±5 s OR §11.55.E fails (cross-track > 4 m) OR new
  abort modes appear (OffTrackError, NaN states, ghost-fallback count
  spikes):** post-mortem, do not ship Phase 4.3, escalate to v3.2 MPC
  immediately. The cheap trick has been honestly attempted and has not
  delivered; further controller-side scalar gains will not save it.

### §23.10.12.9 Scope discipline

Phase 4.3 is **purely a controller-side scalar gain**. It does not:
- modify the plan (`v2_plan` or `v3_dp` plan are both consumed unchanged);
- modify the Pacejka calibration or refit anything;
- modify the ODE solver, weight transfer, or tyre-state plumbing;
- modify the ghost-fallback policy or thresholds;
- modify the slip-band throttle modulators;
- modify any driver JSON beyond the two new optional `control_params`
  fields;
- introduce any new CLI flags.

If implementation discovers a need for any of the above, stop and update
the spec before writing code.

---

## §23.10.11 References

- `dev-planning/lap-simulation-csv-driver/spec-section-23-slip-model.md` —
  parent v3 section; §23.8 is the v3.0 controller this section replaces.
- `src/lap_estimator/dynamics/driver_controller.py` — current Phase 4
  reactive P-controller (lines 250-330 are the throttle/brake/slip-clamp
  logic this section rewrites). Phase 4.3 softener insertion point:
  lines 236–243 (between Stanley δ computation and `max_steer_rad` clip).
- `src/lap_estimator/dynamics/slip_simulator.py` — `_make_controller`
  `target_scale` heuristic (lines 306-360) deleted in Phase 4.1;
  `_run_single` / `_flying_lap_initial_state` unchanged.
- `src/lap_estimator/dynamics/pacejka.py` — α_peak derivation calls
  `pacejka_fy` over a fine α grid.
- `src/lap_estimator/simulator.py` — v2 3-pass distance-marched optimizer;
  Phase 4.2's DP planner is structurally identical, applied to the Pacejka
  envelope.
- `drivers/tomas.json` — Phase 2 Pacejka calibration (fit 2026-05-15);
  consumed verbatim by v3.1.
- §1.2 driver profile spec — `profile.dynamic.throttle_ramp_pct_s` is the
  default for the new `throttle_rate_limit_pct_s` field.
- `memory/feedback_log_at_higher_rate.md` — pedal tau is input slew, not
  reflex; relevant to the throttle-rate-limit interpretation.
