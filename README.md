# LapTimeEstimator

Point-mass lap time simulator using Assetto Corsa car physics + AI line data,
driven by a per-driver JSON (skill + consistency + input-shape parameters).

v1.1: drivers ship as JSON (no YAML), and every `lap.py` run simulates two
laps by default (lap 1 standing, lap 2 flying). Emitted gas/brake traces
include a trail-brake taper, throttle ramp, and a 1st-order driver-lag
low-pass so the synthetic telemetry looks human-plausible.

## Requirements

- Python 3.10+
- NumPy
- Matplotlib (optional, for plots)

No PyYAML dependency: driver configs are JSON (`drivers/*.json`).

## Input Data

Raw AC content is not committed. Two staging folders are gitignored:

- **`cars_in/<car>/`** -- copy a car folder from `<AC>/content/cars/<car>/`. Must contain `data.acd`.
- **`tracks_in/<track>/`** -- copy a track folder from `<AC>/content/tracks/<track>/`. Must contain `ai/fast_lane.ai` (per layout).

The prep step converts these into the tracked `cars_csv/` and `tracks_csv/` artefacts.

## Quick Start

```bash
# 1. Prep a car (one-time)
python prep/prep_car.py cars_in/bmw_1m

# 2. Prep a track (one-time)
python prep/prep_track.py tracks_in/ks_nurburgring

# 3. (Optional) Generate corner notation
python analysis/corner_analysis.py tracks_csv/ks_nurburgring/layout_sprint_a.csv

# 4. Fit a driver from a real AC lap
python fit_driver.py \
    cars_csv/bmw_1m \
    tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv \
    drivers/ludvik_nurburgring_sprint.json

# 5. Run a two-lap sim with that driver (lap 1 standing + lap 2 flying)
python lap.py cars_csv/bmw_1m \
    tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    drivers/pro.json

# 6. Validate sim against a real lap (real lap vs sim lap 2)
python lap.py cars_csv/bmw_1m \
    tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    drivers/ludvik_nurburgring_sprint.json \
    --validate-against samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv
```

## CLIs

### `lap.py` -- simulator

```
python lap.py <car_dir> <track> <driver_json> [options]
```

- `<track>`: CSV path, JSON path, or built-in name (`monza|spa|nurburgring|brands_hatch`).
- `--ds 2.0` -- distance step (m).
- `--all-tracks` -- run all built-ins.
- `--single-lap` -- disable v1.1 two-lap default; emit lap 1 only (legacy v1).
- `--no-plot` -- skip PNG generation.
- `--no-telemetry` -- skip synthetic telemetry CSV.
- `--telemetry-dt-ms 100` -- cadence for synthetic telemetry CSV.
- `--validate-against <real.csv>` -- additionally compare sim **lap 2** vs a real AC lap.
- `--bin-m 100` / `--per-corner` -- validation delta-table mode.

Outputs (next to track CSV, or in cwd for built-ins):
- `<stem>__<driver>_sim_trace.csv` -- per-point trace with `lap` column.
- `<stem>__<driver>_sim_vs_ai.png` -- speed overlay (sim lap 2 vs AI reference).
- `<stem>__<driver>_sim_telemetry.csv` -- AC-schema synthetic telemetry with trailing `lap` column.
- `<stem>__<driver>_validation_bins.csv` and `..._validation_overlay.png` -- when `--validate-against` is set.

### `fit_driver.py` -- derive a driver JSON from AC telemetry

```
python fit_driver.py <car_dir> <track_csv> <ac_telemetry.csv> <output.json> \
                     [--name <driver_name>] [--no-validate] [--no-plot] [--lap {1,2}]
```

Computes `skill_pct` (85th percentile of per-point lateral-G utilisation) and
`consistency_sigma` (scaled stdev of utilisation). Writes a JSON with a `source`
block and (by default) runs a two-lap validation sim, reporting **sim lap 2** as
the comparison point. When the input telemetry has a `lap` column (sim-emitted),
picks the chosen lap (default `--lap 2`).

### `prep/prep_car.py` -- decrypt an AC car

```
python prep/prep_car.py cars_in/<car>
```

Writes `cars_csv/<car>/data/*.ini` and `*.lut`.

### `prep/prep_track.py` -- parse AC AI lines into rich per-point CSV

```
python prep/prep_track.py tracks_in/<track> [--ds 1.0] [--layouts all|<l1>,<l2>]
```

Writes `tracks_csv/<track>/layout_<L>.csv` for each layout. Output columns:
`index, distance_m, segment_length_m, x, y, z, elevation_m, gradient_pct,
radius_m, speed_ms, speed_kmh, width_left_m, width_right_m, width_total_m`.

NOTE: the AI-line parser currently extracts position + cumulative distance only.
`speed_ms` and width columns are emitted blank / zeroed; for the committed
Nurburgring CSVs they were populated by an earlier (manual) pipeline. The
preferred long-term path is to extend `prep/decode_track.py` to read the AC AI
line's detail block (speed, sides). Treat the committed `tracks_csv/` files as
reference.

### `analysis/corner_analysis.py` -- corner classification

```
python analysis/corner_analysis.py <track_csv> [--config tracks_config.json] \
                                   [--no-plot] [--no-json]
```

Telemetry-free. Reads the track CSV + `tracks_config.json` thresholds, writes:
- `<stem>_corners.json` -- corner notation (schema in spec §17.4)
- `<stem>_corner_map.png`, `<stem>_speed_vs_position.png`

## Driver JSON schema (v1.1)

```json
{
  "name": "Pro",
  "skill_pct": 0.97,
  "consistency_sigma": 0.1,
  "driver_tau_s": 0.12,
  "trail_brake_m": 30.0,
  "throttle_ramp_m": 40.0,
  "source": {
    "telemetry_csv": "...",
    "track_csv": "...",
    "car_data_dir": "...",
    "real_lap_time_s": 102.135,
    "sim_lap_time_s": 103.402,
    "delta_s": 1.267,
    "fitted_at": "2026-05-13T14:22:01Z",
    "fit_version": "1"
  }
}
```

Field semantics:
- `name` (required, string) -- used in output filenames.
- `skill_pct` (required, float in (0, 1]) -- multiplier on tyre grip.
- `consistency_sigma` (optional, default 0.0) -- seconds; >0 triggers a 20-run
  Monte-Carlo on **lap 2 only**. Lap 1 (standing start) is always deterministic.
- `driver_tau_s` (optional, default 0.12) -- seconds; 1st-order low-pass time
  constant applied to gas/brake on the synthetic telemetry. `0` disables.
- `trail_brake_m` (optional, default 30.0) -- metres; linear taper of `brake`
  from 1.0 to 0.0 over the last `trail_brake_m` of every braking-bound region
  immediately before a corner. `0` disables.
- `throttle_ramp_m` (optional, default 40.0) -- metres; linear ramp of `gas`
  from the corner-region partial throttle to 1.0 over the first `throttle_ramp_m`
  of every corner-exit acceleration. `0` disables.
- `source` (optional) -- provenance block populated by `fit_driver.py`.

`consistency_sigma > 0` triggers a 20-run Monte-Carlo; the reported lap-2 time
becomes `<mean> +/- <stdev> (N=20)`. The trace CSV / plot / synthetic-telemetry
still come from the deterministic skill-only run.

## Two-lap output (v1.1 default)

Every `lap.py` run simulates two laps in a single 3-pass over a tiled segment
list:
- **Lap 1 (standing)**: starts from rest. Deterministic; no MC noise applied.
- **Lap 2 (flying)**: starts at lap 1's end-of-lap speed. MC noise applies here.

Output artefacts include both laps; the trace CSV and synthetic telemetry CSV
carry a trailing `lap` column with values `1` or `2`. `timestamp_ms` is
monotonic across the boundary; `distanceTraveled` and `normalizedCarPosition`
reset at the start of lap 2 (per-lap-relative, matching AC's behaviour).

The comparison plot shows lap 2 vs AI by default. `--validate-against` targets
sim lap 2 against the real lap. Use `--single-lap` to opt out (legacy v1
behaviour; produces lap 1 only with the `lap` column constant `1`).

## Track CSV format

Per-point columns: `distance_m, radius_m, speed_ms` are required; the rest are
loaded when present. See spec for the full column reference.

## Layout

```
LapTimeEstimator/
├── lap.py                       # sim CLI
├── fit_driver.py                # telemetry -> driver JSON CLI
├── _bootstrap.py                # adds src/ to sys.path for scripts
├── src/lap_estimator/           # core library
│   ├── car.py
│   ├── track.py
│   ├── driver.py
│   ├── driver_fit.py
│   ├── simulator.py             # two-lap tiled sim
│   ├── telemetry.py             # AC log parser (+ optional `lap` column)
│   ├── sim_telemetry.py         # 3-layer pipeline: limit-label -> heuristic -> low-pass
│   ├── report.py                # trace CSV + plot
│   └── validate.py              # real vs sim lap 2
├── prep/                        # AC-content preparation
│   ├── decode_acd.py
│   ├── decode_track.py
│   ├── prep_car.py
│   └── prep_track.py
├── analysis/
│   └── corner_analysis.py
├── drivers/                     # tracked driver JSONs (v1.1: JSON only)
├── cars_csv/<car>/data/         # tracked decrypted car data
├── tracks_csv/<track>/          # tracked per-point track CSVs + outputs
├── samples/aclog/               # tracked sample AC telemetry
├── cars_in/, tracks_in/         # GITIGNORED user-supplied raw AC content
└── tracks_config.json           # corner-classification thresholds + colours
```
