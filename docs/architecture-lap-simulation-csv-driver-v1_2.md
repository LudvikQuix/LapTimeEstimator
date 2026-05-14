# Architecture — Lap Simulation CSV Driver, v1.2 (profile dynamics + lap selection)

## What this code does

`fit_driver.py` now measures **five dynamic driver-profile signals** from real
AC telemetry instead of taking hand-defaults: `driver_tau_s`, `trail_brake_m`,
`throttle_ramp_m` (sim-consumed), plus `pedal_press_rate_per_s` and
`steering_aggression_deg_per_s` (statistics only). The fitter takes a list of
telemetry CSVs (or a glob), keeps only the **5–10 newest** by mtime, pools
their cornering samples for `skill_pct` / `consistency_sigma`, and writes a
`profile.dynamic` block alongside the existing `source` provenance. The
synthetic-telemetry default cadence is raised from 100 ms to 10 ms so the
driver-lag low-pass produces visibly smooth traces (α ≈ 0.077 instead of
0.455).

`Driver.load` reads `profile.dynamic.<field>` first, falls back to top-level,
then to the hand-default — pre-v1.2 driver JSONs (no `profile` block) keep
working unchanged.

## Why this architecture

1. **Single-file profile-dynamics module.** All five measurements share the
   per-lap merged frame, the corner-apex detector, and the candidate-pooling
   logic. Co-locating them in `profile_dynamics.py` keeps the algorithm
   close to its acceptance criteria (spec §13.11) and avoids cross-module
   plumbing. The dataclass + dict-of-bools layout makes the JSON shape
   one-to-one with the in-memory struct.
2. **Median + clamp + threshold-of-pool fallback.** Every measured field
   uses the same recipe: collect candidates → median → clamp to plausible
   range → fall back to hand-default if the pool is under-supplied. The
   `measured.<field>` boolean records which path was taken so downstream
   readers can tell measured-true from fallback.
3. **Lap selection as a CLI-layer concern.** The `--newest N` rule operates
   on file paths + mtimes, with no algorithmic knowledge of telemetry. It
   lives in `fit_driver.py` (the CLI) so the library's `fit_driver()` is
   re-callable from notebooks / tests without a filesystem dependency.
4. **v1.1 + v1.2 in one ship.** The brief targets v1.2 features, but the
   library was still on the v1 single-lap fit. The library upgrade
   (`driver_fit.fit_driver()` now takes a list of merged frames, pools
   `util` across laps, and surfaces `n_laps` / `n_finished_laps` /
   `pooled_sample_count` / `real_lap_times_s`) had to land alongside §13.11
   to satisfy the spec output schema (§13.5 step 8).
5. **Backward compatibility via field precedence.** Top-level
   `driver_tau_s` / `trail_brake_m` / `throttle_ramp_m` are kept in the
   v1.2 output (mirrored from `profile.dynamic` or the hand-default).
   `Driver.load` always consults `profile.dynamic` first, top-level
   second, default third. v1.1 JSONs (top-level only) hit the second
   tier and produce identical sim output to today.

## Data flow

```
[user CLI: positional CSVs | --laps-glob]
        │
        ▼
fit_driver.py
   ├─ glob expansion (if --laps-glob)
   ├─ lap selection (sort by mtime desc, lex tiebreak, take :N)
   │                                          ─── §13.12 ───
   ├─ for each selected path:
   │     read_ac_log(path)                # telemetry.py
   │     ── opportunistic steerAngle + any numeric extras
   │     filter_to_lap (if sim 'lap' column present)
   │     _trim_outlap (drop pre-start prefix on standing-start laps)
   │     merge_with_track(telem, track)
   │
   ▼
driver_fit.fit_driver(car, merged_frames)  # driver_fit.py (rewritten)
   ├─ per-lap util pool:
   │     lat_g_obs = v² / R / g  (R < 500 m only)
   │     lat_g_max = car.tyre_grip_lateral(v) * (1 + downforce/(m·g))
   │     util = clip(obs/max, 0, 1.2)
   │     ── ncp_span >= 0.9  →  finished lap; capture real_lap_time_s
   ├─ pool across laps; percentile(85) → skill_pct, stdev → sigma
   ├─ measure_dynamics(merged_frames)        # profile_dynamics.py (new)
   │     ├─ Step A: hysteresis-armed leading-edge detection on gas+brake
   │     │           → driver_tau_s (median, clamp [0.02, 0.50])
   │     │           → pedal_press_rate_per_s (median slope)
   │     │           fallback when pool <10 edges
   │     ├─ Step B: smoothed-speed minima → corner apex indices
   │     ├─ Step C: per apex, last brake>=0.80 → first brake<=0.10
   │     │           → trail_brake_m (median, clamp [5, 150])
   │     │           fallback 30.0 when pool <5 candidates
   │     ├─ Step D: per apex, first gas>=0.40 → first gas>=0.95
   │     │           → throttle_ramp_m (median, clamp [5, 200])
   │     │           fallback 40.0 when pool <5 candidates
   │     └─ Step E: |d(steerAngle)/dt| 95th percentile (rad-vs-deg auto-
   │                detected); None when no steerAngle column.
   ▼
FitResult
   ├─ skill_pct, consistency_sigma, util_*, sample counts
   ├─ n_laps, n_finished_laps, real_lap_times_s, real_lap_time_s
   └─ profile: ProfileDynamics
   ▼
fit_driver.py
   ├─ build payload with profile.dynamic block + top-level mirror
   ├─ source.lap_selection (rule, candidates_considered, selected_*)
   ├─ source.fit_version = "2"
   ├─ optional validation sim (two-lap, report lap-2 time + delta verdict)
   └─ write JSON
```

For the sim pipeline (`lap.py`), the only changes are:
- `--telemetry-dt-ms` default 10 (was 100).
- `Driver.load` reads `profile.dynamic.*` first when present.

## File inventory

| File | Status | Why |
|---|---|---|
| `src/lap_estimator/profile_dynamics.py` | NEW (~370 lines) | Owns the five-field measurement algorithm. Single entry point `measure_dynamics(merged_frames) -> ProfileDynamics`. |
| `src/lap_estimator/telemetry.py` | MODIFIED | `read_ac_log` now passes through `steerAngle` and any other numeric extras as float arrays. `merge_with_track` preserves them. Required-column set is unchanged. |
| `src/lap_estimator/driver_fit.py` | REWRITTEN | Signature: `fit_driver(car, merged_frames)` accepts a list of per-lap merged frames (single-dict legacy callers wrapped). Pools `util` across laps, tracks `n_finished_laps` via `ncp_span >= 0.9`, calls `profile_dynamics.measure_dynamics`. New fields on `FitResult`. |
| `src/lap_estimator/driver.py` | MODIFIED | `Driver.load` resolves the three sim-consumed fields with v1.2 precedence (`profile.dynamic.<field>` > top-level > default). Loads two new statistic fields onto the dataclass (`pedal_press_rate_per_s`, `steering_aggression_deg_per_s`) but does not wire them into the simulator. Validation rejects `profile.dynamic.driver_tau_s` outside [0, 1] and negative `trail_brake_m`/`throttle_ramp_m`. |
| `fit_driver.py` | REWRITTEN | Positional reordered to `<car> <track> <output> [<csv>...]` per spec §13.3. Variadic positionals + `--laps-glob` (mutually exclusive) + `--newest N` (default 10, min 2). Out-lap trim. Profile-dynamics summary line. JSON output with `profile.dynamic`, `source.lap_selection`, `fit_version="2"`. |
| `lap.py` | MODIFIED | `--telemetry-dt-ms` default 10. Help text updated. |
| `src/lap_estimator/sim_telemetry.py` | MODIFIED | `write_synthetic_log` kwarg default `telemetry_dt_ms=10`. |
| `README.md` | MODIFIED | Documents `profile.dynamic`, `--newest`, the 10 ms default and how to revert (`--telemetry-dt-ms 100`). |

## Integration points

- **`Driver.load` → `simulator.simulate`.** The sim consumes
  `driver.driver_tau_s` / `trail_brake_m` / `throttle_ramp_m` via the same
  `sim_telemetry.write_synthetic_log` path as v1.1. The v1.2 statistics
  fields are loaded onto the dataclass but **not** passed into the
  simulator (spec §13.2 non-goal: "v1.3 picks them up").
- **`fit_driver.py` → validation sim.** Reuses `simulator.simulate(...,
  two_lap=True)` and reports lap-2 time. The validation block in the JSON
  (`sim_lap_time_s`, `delta_s`) lives under `source`.
- **`telemetry.merge_with_track` pass-through.** Any numeric column that
  appears in the input CSV (including `steerAngle`) is forwarded onto the
  merged frame. `profile_dynamics` reads it opportunistically; absent
  columns yield `None` + `measured=false` for the corresponding field.
- **Lap-selection mtime ordering.** The CLI sorts candidates by `-mtime`
  (newest first), lex tiebreak on path. The first `N` are kept. The same
  newest-first list goes into `source.lap_selection.selected_sources`
  and `source.telemetry_csvs`.

## Why some measured values land at clamp boundaries

The Tomas BMW M1 sample CSVs at 50 Hz show very fast pedal action:
gas/brake transitions typically span 1–3 samples (20–60 ms). The clamp
[0.02, 0.50] on `driver_tau_s` therefore produces measured medians close
to 0.03 s, which is below the brief's "expected" [0.05, 0.30]. This is a
**dataset property, not a bug** — slower drivers / 100 Hz telemetry would
land in the higher range. Similarly, `trail_brake_m` lands at 5.5 m
(clamp floor) because Tomas's braking is largely bang-bang in these laps;
only ~65 of 5409 samples sit in the 0.1–0.95 brake range per lap.
`measured=true` is set in both cases — the measurement succeeded; it just
happens to peg the clamp. Downstream consumers can read the
`sample_counts` block to see how many candidates fed each measurement.

## Forward-compatibility hooks (recorded for v1.3)

- The statistic fields are already on the `Driver` dataclass; wiring them
  into a slew-rate-limited filter (replacing the now-removed 1st-order
  low-pass) is purely a `sim_telemetry.py` change.
- `profile.dynamic.by_corner_type` is reserved (spec §13.13) — keep new
  per-corner overrides under `profile.dynamic.<scope>` to avoid touching
  the top-level schema.

## v1.2.1 — IIR low-pass removed

Removed IIR low-pass on emitted gas/brake (was cosmetic; racing has no
classical reaction time, only motor execution ~20–50 ms which is sub-sample
at 50 Hz). `driver_tau_s` retained as statistic only.

Touchpoints:
- `src/lap_estimator/sim_telemetry.py`: deleted Layer 3 block and the
  `_iir_lowpass` helper. Pipeline is now two layers (limit-label rule →
  corner-shape heuristic) and the heuristic's output is written to disk
  directly. The function signature is unchanged; `driver.driver_tau_s` is
  simply no longer read.
- `src/lap_estimator/driver.py`: `driver_tau_s` stays on the dataclass and
  in `Driver.load` for back-compat / statistics; a comment marks it as
  unused by the sim.
- `src/lap_estimator/profile_dynamics.py`: unchanged — still measures
  `driver_tau_s` as a driver characteristic.
- README: removed the "1st-order driver-lag low-pass" claim from the v1.1
  blurb; updated `driver_tau_s` field semantics to "statistic only, not
  consumed by sim".
- No CLI changes (`--telemetry-dt-ms` default stays at 10).

Spec references: §14.1 (rationale), §14.3 (two-layer pipeline), §14.10
items 11 / 13 (acceptance criteria), §11.26 (invariance under
`driver_tau_s`), Decisions item 22 (supersedes item 15).
