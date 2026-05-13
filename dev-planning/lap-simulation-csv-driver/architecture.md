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
