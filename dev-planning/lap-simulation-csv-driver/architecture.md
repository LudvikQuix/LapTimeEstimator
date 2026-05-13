# Architecture — lap-simulation-csv-driver

## What it does

LapTimeEstimator now has five entry points and a small core library:

- **`lap.py`** — runs a sim for `(car, track, driver)`, emits a trace CSV / overlay PNG / synthetic AC-schema telemetry CSV; optional `--validate-against <real.csv>` cross-compares vs a real lap.
- **`fit_driver.py`** — derives a driver YAML (`skill_pct`, `consistency_sigma`) from a real AC telemetry lap on a known car/track and runs a validation sim.
- **`prep/prep_car.py`** — wraps `decode_acd.py` to populate `cars_csv/<car>/data/`.
- **`prep/prep_track.py`** — wraps `decode_track.py` AI-line parser; emits the rich per-point CSV.
- **`analysis/corner_analysis.py`** — telemetry-free corner classification; reads a track CSV + `tracks_config.json`; emits `<stem>_corners.json` + two PNGs.

## File layout

```
src/lap_estimator/      library (importable)
  car.py                AC car physics (unchanged from pre-feature)
  track.py              Track (segments + new from_csv); to_points / to_ai_reference
  driver.py             Driver dataclass; _DriverScaledCar composition wrapper
  driver_fit.py         fit_driver(car, merged) -> FitResult
  simulator.py          3-pass simulate / simulate_monte_carlo / print_report
  telemetry.py          read_ac_log + merge_with_track
  sim_telemetry.py      write_synthetic_log (AC-schema CSV)
  report.py             trace CSV + comparison PNG + plot_speed_overlay helper
  validate.py           validate_lap + write_bins_csv

prep/                   AC raw-content preparation (standalone; no lap_estimator dep)
  decode_acd.py
  decode_track.py
  prep_car.py
  prep_track.py

analysis/
  corner_analysis.py    standalone classifier (reads tracks_config.json)

lap.py                  sim CLI (root)
fit_driver.py           driver-fit CLI (root)
_bootstrap.py           tiny sys.path shim; imported by root CLIs
drivers/                example YAMLs (pro, amateur) + fitted YAMLs
samples/aclog/          one committed AC log for reproducible acceptance
```

## Module responsibilities

- `Car` (untouched): parses `engine.ini`, `tyres.ini`, `aero.ini`, etc. into the physics model. Public surface is `tyre_grip_lateral / longitudinal`, `max_cornering_speed`, `max_braking_decel`, `max_accel`, `max_traction_force`, `downforce`, `top_speed`.
- `Track`: dual-backed. Built-ins / JSON tracks use a `segments` list. CSV tracks store column arrays in `csv_data`. `to_points(ds)` returns `(distances, radii)` on a uniform grid; for CSV tracks it interpolates curvature (`1/R`) to avoid spikes at sample boundaries. `to_ai_reference(ds)` returns the AI speed trace for overlays.
- `Driver`: dataclass loaded from YAML. `wrap(car, rng, noise)` returns `_DriverScaledCar`, a thin composition wrapper that overrides grip-related methods to apply `skill_pct` (and optional per-point Gaussian noise sampled from `grip_sigma`).
- `simulator.simulate(car, track, driver, ds, rng, noise)` runs the 3-pass core; produces `SimResult` with per-point `times`, `ai_speeds`, and `limit_label` (`corner|accel|brake`). `simulate_monte_carlo` runs N noisy laps and returns mean/std plus a representative deterministic run.
- `telemetry.read_ac_log` parses an AC telemetry CSV (sorted by timestamp; the user-supplied lap-2 log was unsorted at the lap wrap). `merge_with_track` interpolates track radius / gradient onto each telemetry sample.
- `driver_fit.fit_driver(car, merged)` computes `lat_g_obs / lat_g_max` per cornering sample, sets `skill_pct = clip(p85(util), 0.05, 1.0)` and `consistency_sigma = clip(stdev(util)/0.03, 0, 1.5)`.
- `sim_telemetry.write_synthetic_log` time-resamples a `SimResult` at `dt_ms` and reconstructs `gas`/`brake` from the binding label (accel → 1/0, brake → 0/1, corner → drag+roll required / max traction).
- `validate.validate_lap` resamples a real lap onto the sim distance grid and computes per-bin (or per-corner-span) deltas; verdict thresholds GOOD / LOOSE / BAD per spec §15.5.
- `report.write_trace_csv` / `plot_speed_overlay` / `write_comparison_plot` own all output I/O. `build_output_stem` standardises file naming (`<csv_stem>__<driver>` next to the input track CSV; cwd for built-ins).

## Data flow — `lap.py`

```
[car_dir]  -> Car
[track]    -> Track.from_csv | Track.from_json | BUILTIN_TRACKS[<name>]
[driver]   -> Driver.load
                       |
            v
        simulate(car, track, driver, ds) -> SimResult
                       |
            +----------+--------------------+--------------------+
            v                               v                    v
   report.write_trace_csv         report.write_comparison_plot   sim_telemetry.write_synthetic_log
            |                               |                    |
   <stem>_sim_trace.csv             <stem>_sim_vs_ai.png    <stem>_sim_telemetry.csv

   if --validate-against:
        validate.validate_lap(...) -> ValidationResult -> stdout + bins.csv + overlay.png
```

## Data flow — `fit_driver.py`

```
[car_dir]   -> Car
[track_csv] -> Track.from_csv
[aclog.csv] -> telemetry.read_ac_log -> telemetry.merge_with_track(_, track)
                      |
                      v
               driver_fit.fit_driver(car, merged) -> FitResult
                      |
                      v
               write YAML (with `source` block); optional validation sim
```

## Decisions / deviations from the spec

1. **CSV-track curvature interpolation uses curvature `1/R`** (spec §8 risk), then clipped back to `[1.0, 2000.0]` m. Straights end up at 2000 m, which `Car.max_cornering_speed` treats as "no lateral limit".
2. **`grip_sigma` (driver consistency)** is `clip(consistency_sigma * 0.03, 0, 0.1)`, matching spec §13.6 inverse. Applied as a multiplicative perturbation to grip per query.
3. **`telemetry.read_ac_log` sorts rows by timestamp.** The user's lap-2 sample CSV had the wrap-around (lap-end then lap-start) at the front of the file; without sorting `lap_time_seconds` returned a near-zero value. Sort is idempotent if the file is already monotonic.
4. **`merge_with_track` normalises `distanceTraveled` to start at 0** before interpolating radius/gradient. AC's `distanceTraveled` is session-cumulative.
5. **`prep_track.py` emits geometry-only columns and zero-fills widths.** The existing AI-line parser (`prep/decode_track.py`) reads position + cumulative distance only. The committed `tracks_csv/ks_nurburgring/layout_*.csv` files were populated by a prior pipeline with speed + width data; until `decode_track.py` is extended to parse the AI line's detail block (speed channel, `side_l`, `side_r`), `prep_track.py` will not exactly match those reference CSVs. Documented in README. Acceptance criterion §11.11 is not fully covered — pending an AI-line parser extension. Reference CSVs are kept in place and treated as canonical inputs to `lap.py` / `corner_analysis.py`.
6. **Corner-analysis output: only true corners (R < 400 m) land in `corners[]`.** Straights are omitted in v1 per spec §17.4; schema reserves `direction: straight` for v2.
7. **Validation `_corner_spans` uses 500 m as the corner/straight cutoff** and emits both `corner` and `straight` spans (with `kind` column) when `--per-corner` is set; the spec only mandated corner spans, this is a strict superset.
8. **No `pyproject.toml` / editable install.** `_bootstrap.py` injects `<repo>/src` on `sys.path` for the root CLIs. Each prep/analysis script handles its own sibling imports (single-line shim). Acceptable per spec §6.8.
9. **Built-in tracks (`monza`, `spa`, ...) are retained for `--all-tracks` and legacy invocations**; the third positional driver YAML argument is required on `lap.py` (spec §10).

## Smoke test results (Nurburgring sprint_a, BMW 1M, ds=2.0)

- Built-in Monza: legacy invocation works, lap time 2:17.395 (Pro).
- CSV sprint_a + Pro (skill 0.97, sigma 0.1): 1:47.394 ± 0.006 (N=20).
- CSV sprint_a + Amateur (skill 0.82, sigma 0.6): 1:53.140 ± 0.031 (N=20). Slower as expected.
- `fit_driver.py` on the bundled sample lap: real 1:51.860, sim 1:46.439, delta −5.421 s (−4.85%). Verdict: LOOSE. 16% of cornering samples have util > 1.0 — the AC car model under-grips relative to actual driving, so the 85th-percentile utilisation saturates at 1.0 (skill ceiling). This is the v1 limitation noted in spec §8 risks; the fit is functionally correct.
- `--validate-against` flow: end-to-end ok, verdict LOOSE, bins CSV + overlay PNG produced.
- Loop closure (sim telemetry → re-fit): recovered `skill_pct` 0.9703 vs original 0.97 (0.03% error), sim lap 1:47.389 vs original 1:47.300 (Δ = 0.089 s). Passes spec §14.8.
- `corner_analysis.py` on sprint_a: 7 corners detected, JSON + corner-map PNG + speed-vs-position PNG written next to the CSV.
- `--telemetry-dt-ms` cadence test: 50 ms → 2149 rows; 200 ms → 538 rows (~4× ratio).
- Driver YAML validation: missing `skill_pct` and `skill_pct > 1.0` both raise `ValueError` with clear messages.

## What was NOT done

- `prep/decode_track.py` was not extended to parse the AC AI line's detail block (speed/side_l/side_r). `prep_track.py` therefore emits zero-filled widths and blank speed columns. The committed `tracks_csv/ks_nurburgring/layout_*.csv` reference files are unaffected and remain canonical inputs.
- No `pyproject.toml` / editable install — `_bootstrap.py` shim is used instead (per spec §6.8).
- Did not delete or modify the user's `Corner_Analysis/` scratch folder (per spec §13.8).
- Did not write tests — that is Tester's job, and the user said to skip Tester by default.

---

## v1.1 addendum (driver JSON migration, layered telemetry, two-lap default)

### Summary of v1.1 changes

Four bundled changes layered on the shipped v1 plumbing (spec §13/§14.3/§20):

- **A. YAML → JSON driver config.** Loader now uses stdlib `json` only. PyYAML dropped. Every existing `drivers/*.yaml` rewritten as `.json` and deleted. No fallback.
- **B. Driver-lag 1st-order low-pass** on emitted gas/brake (`driver_tau_s`, default 0.12 s).
- **C. Trail-brake + throttle-ramp corner-shape heuristic** (`trail_brake_m`/30 m, `throttle_ramp_m`/40 m defaults). Applied BEFORE the low-pass.
- **D. Two-lap "tiled" simulation, default on.** Single 3-pass over a tiled segment list; output split into lap 1 (standing) / lap 2 (flying). `--single-lap` opts out.

### Layered telemetry pipeline (sim_telemetry.py, §14.3)

```
sim limit-label sequence (per-point: corner|accel|brake)
   |
   v
[Layer 1] limit-label rule  ----------------------------------+
   accel  -> gas=1, brake=0                                    |
   brake  -> gas=0, brake=1                                    |
   corner -> gas = (drag+rr) / max_traction, brake=0           |
   |                                                           |
   v                                                           |
[Layer 2] corner-shape heuristic                               |
   - trail-brake taper:  brake region's last `trail_brake_m`   |
     metres replaced by linear ramp 1.0 -> 0.0 (in distance,   |
     applied on the continuous distance axis -- works across   |
     the lap-1/lap-2 boundary just like any other transition). |
   - throttle ramp-up:   corner-region exit's first             |
     `throttle_ramp_m` metres replaced by linear ramp           |
     corner_exit_gas -> 1.0.                                    |
   - either field == 0.0 disables that leg (no-op).            |
   |                                                           |
   v                                                           |
[Layer 3] driver-lag 1st-order IIR low-pass                    |
   y[n] = y[n-1] + alpha * (x[n] - y[n-1])                     |
   alpha = dt / (driver_tau_s + dt)                            |
   y[0] = x[0]; applied independently to gas and brake.        |
   - driver_tau_s == 0.0 bypasses (Layer 2 output passes       |
     through -> v1 bang-bang when trail=ramp=0 too).           |
   |                                                           |
   v                                                           |
CSV write (AC schema + trailing `lap` column)
```

**Order is load-bearing.** Inverting heuristic and low-pass would smear the
limit-label transitions before the heuristic can read them — taper/ramp
distances would be applied to already-smoothed edges, producing
double-smoothed shapes with the wrong total length.

Layers 2 and 3 operate on the **continuous distance axis** (lap 1 [0..L] then
lap 2 [L..2L]) so cross-boundary transitions are handled identically to any
mid-lap transition.

### Two-lap tiled simulation (simulator.py, §20)

```
track.to_points(ds) -> (distances_lap, radii_lap)
       |
       v
   tile: distances_full = [distances_lap, distances_lap + L + ds_step]
         radii_full     = [radii_lap, radii_lap]
         lap_id_full    = [1...1, 2...2]
       |
       v
   3-pass solver over the WHOLE tiled grid (single pass):
     pass 1: v_corner[i]
     pass 2: v_forward[i] -- continuous across the lap-1/lap-2 boundary,
                             so lap 2's initial speed is naturally
                             lap 1's end-of-lap forward-pass speed.
     pass 3: v_brake[i] (backward) -- runs across the whole grid.
       |
       v
   integrate time -> times[] (monotonic across both laps)
       |
       v
   split:
     - distances_out: per-lap-relative (resets at lap-2 start)
     - lap1_time = times[n_lap - 1]
     - lap2_time = times[-1] - times[n_lap - 1]
     - SimResult.lap_time = lap2_time (the headline flying lap)
     - SimResult.two_lap  = True
```

Monte-Carlo (`simulate_monte_carlo`) calls `simulate(..., two_lap=True)` N
times and takes mean/std of **lap 2 only**. Lap 1 is computed once
deterministically (standing-start variance is dominated by launch traction,
not driver consistency).

### `lap` column propagation

- **Telemetry CSV (`sim_telemetry.write_synthetic_log`)**: header is
  `timestamp_ms,gas,brake,distanceTraveled,speedKmh,normalizedCarPosition,lap`.
  `timestamp_ms` is strictly monotonic across the boundary;
  `distanceTraveled` and `normalizedCarPosition` reset to 0 at lap-2 start.
- **Trace CSV (`report.write_trace_csv`)**: header is
  `lap,distance_m,sim_speed_ms,sim_speed_kmh,ai_speed_kmh,time_s`.
- **`--single-lap` mode**: lap column is present but constant `1`.

### `--validate-against` lap-2 targeting

`validate.validate_lap(..., target_lap=2)` extracts the lap-2 slice of
`sim_result` (mask on `lap_id`), re-zeroes its time axis to start at 0, and
compares against the real telemetry exactly as in v1. With `--single-lap`,
`lap.py` passes `target_lap=1` and prints a warning ("comparing real flying
lap against sim standing lap").

### `fit_driver.py` lap selection

When the input telemetry CSV has a `lap` column (sim-emitted via the round
trip `lap.py` -> `fit_driver.py`), the fitter calls `telemetry.filter_to_lap`
(default lap 2) before merging with the track. Real AC logs have no `lap`
column and are unaffected. CLI: `--lap {1,2}` override.

The validation sim inside `fit_driver.py` is two-lap; the reported
`sim_lap_time_s` and the `source.delta_s` are lap-2 (flying) values.

### File inventory (v1.1)

**Modified:**
- `src/lap_estimator/driver.py` — JSON loader; new fields with defaults; PyYAML dep removed.
- `src/lap_estimator/simulator.py` — tiled two-lap mode; `lap_id`/`lap1_time`/`lap2_time` on `SimResult`; two-lap-aware `print_report`.
- `src/lap_estimator/sim_telemetry.py` — 3-layer pipeline; `lap` column; cross-boundary distance handling; takes `driver` arg.
- `src/lap_estimator/report.py` — `lap` column on trace CSV; lap-2 slice on comparison plot.
- `src/lap_estimator/validate.py` — `target_lap=2` default; lap-slicing helper.
- `src/lap_estimator/telemetry.py` — optional `lap` column read; `filter_to_lap` helper.
- `lap.py` — `--single-lap` flag; passes driver into `write_synthetic_log`; validation lap-2 targeting; two-lap-aware plot label.
- `fit_driver.py` — JSON write (no YAML); `--lap` flag; two-lap validation; lap-2 sim_lap_time_s.
- `README.md` — driver JSON examples; two-lap default documented; PyYAML reference removed.

**Migrated (YAML → JSON, contents preserved + new fields appended):**
- `drivers/pro.yaml` → `drivers/pro.json`
- `drivers/amateur.yaml` → `drivers/amateur.json` (amateur defaults bumped: tau=0.20, trail=45, ramp=55 per spec §6.2)
- `drivers/tomas_nurburgring_sprint.yaml` → `drivers/tomas_nurburgring_sprint.json`
- `drivers/ludvik_nurburgring_sprint.yaml` → `drivers/ludvik_nurburgring_sprint.json`
- `drivers/tomas_lap{2,3,4,5}.yaml` → `drivers/tomas_lap{2,3,4,5}.json`

**Deleted:** all `drivers/*.yaml`.

### v1.1 smoke test (Nurburgring sprint_a, BMW 1M, ds=2.0)

- `tomas_nurburgring_sprint.json`: lap 1 1:48.281 (matches v1 lap-time exactly), lap 2 1:45.111 +/- 0.087 (N=20). lap_2 <= lap_1 + sigma satisfied.
- `--validate-against` (Tomas Lap5): real 1:47.560, sim lap 2 1:44.706, delta -2.854 s (-2.65%), verdict GOOD.
- `pro.json` regression-safety with tau=0 trail=0 ramp=0: gas/brake transitions are abrupt 1.0->0.0 (no IIR smoothing); legacy bang-bang restored.
- Loop closure (Pro -> sim telemetry -> re-fit, lap 2): skill_pct recovered 0.9710 vs 0.97 (0.10% error); sim lap delta 0.030 s.
- Corner-exit stuck-at-extreme run (4 s window after brake-zero transitions): 0 samples (spec §11.16 requires <= 5). The 138-sample stuck-at-1.0 runs on long straights are expected (long-straight gas saturation is realistic, not a defect).
- Trail-brake taper visible on lap-2 brake region (sample window dist=998..1056m): brake decays from peak 0.897 -> 0.101 over ~30 m, consistent with the configured `trail_brake_m=30.0` modulo low-pass.

### Decisions / deviations from the v1.1 spec

1. **Continuous-distance axis for Layer 2.** The trail-brake / throttle-ramp heuristics walk the *continuous* (cross-lap) distance grid so that taper distances are measured correctly even when a transition straddles the lap-1/lap-2 boundary. Per-lap-relative distances are only used for the `distanceTraveled` and `normalizedCarPosition` output columns.
2. **`SimResult.lap_time` is the primary lap.** In two-lap mode this is lap 2 (flying); in single-lap mode it's lap 1. `lap1_time` / `lap2_time` are exposed separately for callers that need both.
3. **The `Driver` `__init__` accepts all v1.1 fields** as keyword args. `fit_driver.py` constructs a temporary in-memory `Driver` for its validation pass; the v1.1 defaults are pinned via `DEFAULT_*` module constants imported from `driver.py`.
4. **`sim_telemetry.write_synthetic_log` now requires a `driver` arg.** Callers that don't have a Driver object (currently none in the repo) can pass `None` for bang-bang behaviour; the function tolerates that.

### What was NOT done (v1.1)

- Did not extend `fit_driver.py` to fit `driver_tau_s` / `trail_brake_m` / `throttle_ramp_m` from telemetry — spec §13.9 defers this to v2 (requires a real driver-input model).
- Did not touch `prep/` or `analysis/` modules — v1.1 is sim-layer only.
- Did not commit (per user instruction).

