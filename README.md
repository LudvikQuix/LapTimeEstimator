# LapTimeEstimator

Point-mass lap time simulator using Assetto Corsa car physics + AI line data,
driven by a per-driver YAML (skill + consistency).

## Requirements

- Python 3.10+
- NumPy
- PyYAML (`pip install pyyaml`)
- Matplotlib (optional, for plots)

## Input Data

Raw AC content is not committed. Two staging folders are gitignored:

- **`cars_in/<car>/`** — copy a car folder from `<AC>/content/cars/<car>/`. Must contain `data.acd`.
- **`tracks_in/<track>/`** — copy a track folder from `<AC>/content/tracks/<track>/`. Must contain `ai/fast_lane.ai` (per layout).

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
    drivers/ludvik_nurburgring_sprint.yaml

# 5. Run a sim with that driver
python lap.py cars_csv/bmw_1m \
    tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    drivers/pro.yaml

# 6. Validate sim against a real lap (cross-track or same-track)
python lap.py cars_csv/bmw_1m \
    tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    drivers/ludvik_nurburgring_sprint.yaml \
    --validate-against samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv
```

## CLIs

### `lap.py` — simulator

```
python lap.py <car_dir> <track> <driver_yaml> [options]
```

- `<track>`: CSV path, JSON path, or built-in name (`monza|spa|nurburgring|brands_hatch`).
- `--ds 2.0` — distance step (m).
- `--all-tracks` — run all built-ins.
- `--no-plot` — skip PNG generation.
- `--no-telemetry` — skip synthetic telemetry CSV.
- `--telemetry-dt-ms 100` — cadence for synthetic telemetry CSV.
- `--validate-against <real.csv>` — additionally compare sim vs a real AC lap.
- `--bin-m 100` / `--per-corner` — validation delta-table mode.

Outputs (next to track CSV, or in cwd for built-ins):
- `<stem>__<driver>_sim_trace.csv` — per-point trace.
- `<stem>__<driver>_sim_vs_ai.png` — speed overlay.
- `<stem>__<driver>_sim_telemetry.csv` — AC-schema synthetic telemetry.
- `<stem>__<driver>_validation_bins.csv` and `..._validation_overlay.png` — when `--validate-against` is set.

### `fit_driver.py` — derive a driver YAML from AC telemetry

```
python fit_driver.py <car_dir> <track_csv> <ac_telemetry.csv> <output.yaml> \
                     [--name <driver_name>] [--no-validate] [--no-plot]
```

Computes `skill_pct` (85th percentile of per-point lateral-G utilisation) and
`consistency_sigma` (scaled stdev of utilisation). Writes a YAML with a `source`
block and, by default, runs a validation sim and prints the real-vs-sim delta.

### `prep/prep_car.py` — decrypt an AC car

```
python prep/prep_car.py cars_in/<car>
```

Writes `cars_csv/<car>/data/*.ini` and `*.lut`.

### `prep/prep_track.py` — parse AC AI lines into rich per-point CSV

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

### `analysis/corner_analysis.py` — corner classification

```
python analysis/corner_analysis.py <track_csv> [--config tracks_config.json] \
                                   [--no-plot] [--no-json]
```

Telemetry-free. Reads the track CSV + `tracks_config.json` thresholds, writes:
- `<stem>_corners.json` — corner notation (schema in spec §17.4)
- `<stem>_corner_map.png`, `<stem>_speed_vs_position.png`

## Driver YAML schema

```yaml
name: Pro                # used in output filenames
skill_pct: 0.97          # required, in (0, 1]
consistency_sigma: 0.1   # optional, seconds (0 = single deterministic run)
source:                  # optional, populated by fit_driver.py
  telemetry_csv: ...
  track_csv: ...
  car_data_dir: ...
  real_lap_time_s: 102.135
  sim_lap_time_s: 103.402
  delta_s: 1.267
  fitted_at: 2026-05-13T14:22:01Z
  fit_version: "1"
```

`consistency_sigma > 0` triggers a 20-run Monte-Carlo; the reported lap time
becomes `<mean> ± <stdev> (N=20)`. The trace CSV / plot / synthetic-telemetry
still come from the deterministic skill-only run.

## Track CSV format

Per-point columns: `distance_m, radius_m, speed_ms` are required; the rest are
loaded when present. See spec for the full column reference.

## Layout

```
LapTimeEstimator/
├── lap.py                       # sim CLI
├── fit_driver.py                # telemetry -> driver YAML CLI
├── _bootstrap.py                # adds src/ to sys.path for scripts
├── src/lap_estimator/           # core library
│   ├── car.py
│   ├── track.py
│   ├── driver.py
│   ├── driver_fit.py
│   ├── simulator.py
│   ├── telemetry.py
│   ├── sim_telemetry.py
│   ├── report.py
│   └── validate.py
├── prep/                        # AC-content preparation
│   ├── decode_acd.py
│   ├── decode_track.py
│   ├── prep_car.py
│   └── prep_track.py
├── analysis/
│   └── corner_analysis.py
├── drivers/                     # tracked driver YAMLs
├── cars_csv/<car>/data/         # tracked decrypted car data
├── tracks_csv/<track>/          # tracked per-point track CSVs + outputs
├── samples/aclog/               # tracked sample AC telemetry
├── cars_in/, tracks_in/         # GITIGNORED user-supplied raw AC content
└── tracks_config.json           # corner-classification thresholds + colours
```
