# LapTimeEstimator

Point-mass lap time simulator using Assetto Corsa car physics and AI-line track
data, driven by a per-driver JSON (skill, consistency, and input-shape parameters).

**Current state: v2 / v1.3.** Multi-lap stint simulation with per-wheel tyre
state (temperature, wear, pressure) evolving lap-to-lap. Asymmetric pressure
model: grip penalty above PRESSURE_IDEAL only; drag penalty below PRESSURE_IDEAL
only. Multi-compound support — `--compound Semislicks` selects a different set of
grip / wear / thermal curves from `tyres.ini`. Inverse-PSI solver recommends cold
setup pressures to hit a target wear at a given lap.

**v3 slip path (shipped, research-quality):** `--model slip --controller reactive`
runs the Pacejka envelope + RK4 ODE + reactive preview-Stanley controller with
the Phase 5.0.2 chicane safety cap. v2 point-mass remains the default. The v3
**MPC** stack (`--controller mpc`) is **experimental / parked** — 7 consecutive
controller iterations failed at the Sprint A chicane and the binding constraint
is the reactive sub-controller running under Tier 2, not anything the MPC layer
controls. See `docs/architecture-v3-shipping-state.md` for the shipping decision,
known limitations per mode, and resurrection criteria.

**v3 status as of 2026-05-24:** First complete v3 lap on Sprint A. A physics audit
(6-term longitudinal fix, twin-turbo curve, I_zz correction, Stanley LPF + k_cross
uplift) turned 0/10 MC completions into 7/10. Current best lap: **2:09.12** (7/10
stable, `--inertia-zz 2400`). With 9/10 stability target: **2:12.64** at
`--chicane-safety-mult 0.75`. Recommended reactive invocation:

```bash
python lap.py \
    cars_csv/bmw_1m \
    tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    drivers/tomas.json \
    --model slip --controller reactive --single-lap \
    --inertia-zz 2400
```

See `docs/architecture-v3-session-2026-05-24.md` for the session index, full
reading order, and open gaps.

v1.2: `fit_driver.py` measures dynamic driver-profile signals from telemetry
(`trail_brake_m`, `throttle_ramp_m`, `driver_tau_s`, `pedal_press_rate_per_s`,
`steering_aggression_deg_per_s`). `driver_tau_s` is a recorded statistic; it is
**not consumed** by the simulator (v1.2.1 removed the IIR low-pass — see spec
§14.3). Synthetic-telemetry default cadence is 10 ms (pass `--telemetry-dt-ms 100`
to revert).

## Requirements

- Python 3.10+
- NumPy
- Matplotlib (optional, for plots)

No PyYAML dependency: all configs are JSON.

## Input data

Raw AC content is not committed. Two staging folders are gitignored:

- **`cars_in/<car>/`** — copy a car folder from `<AC>/content/cars/<car>/`. Must
  contain `data.acd`.
- **`tracks_in/<track>/`** — copy a track folder from `<AC>/content/tracks/<track>/`.
  Must contain `ai/fast_lane.ai` per layout.

The prep step converts these into the tracked `cars_csv/` and `tracks_csv/`
artefacts.

## Quick start

```bash
# 1. Prep a car (one-time)
python prep/prep_car.py cars_in/bmw_1m

# 2. Prep a track (one-time)
python prep/prep_track.py tracks_in/ks_nurburgring

# 3. (Optional) Generate corner notation
python analysis/corner_analysis.py tracks_csv/ks_nurburgring/layout_sprint_a.csv

# 4. Fit a driver from real AC laps (requires >=2 laps)
python fit_driver.py \
    cars_csv/bmw_1m \
    tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    drivers/tomas.json \
    samples/aclog/Tomas_Lap1.csv \
    samples/aclog/Tomas_Lap2.csv \
    samples/aclog/Tomas_Lap3.csv

# 5. Run a 3-lap stint on Semislicks at 26 psi
python lap.py \
    cars_csv/bmw_1m \
    tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    drivers/tomas_full.json \
    --laps 3 --compound Semislicks \
    --pressure FL=26,FR=26,RL=26,RR=26 \
    --ambient-temp-c 26

# 6. Validate sim against a real lap (sim lap 2 vs real)
python lap.py \
    cars_csv/bmw_1m \
    tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    drivers/tomas.json \
    --validate-against samples/aclog/Tomas_Lap2.csv

# 7. Solve for cold PSI that gives 50% wear at lap 12
python lap.py \
    cars_csv/bmw_1m \
    tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    drivers/tomas_full.json \
    --solve-pressure-for-wear 0.50 --at-lap 12 \
    --compound Semislicks --ambient-temp-c 26

# 8. (Research) Run the v3 slip model with the reactive controller.
#    Note: aborts at the Sprint A chicane (s~1087 m) — see
#    docs/architecture-v3-shipping-state.md for known limitations.
python lap.py \
    cars_csv/bmw_1m \
    tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    drivers/tomas.json \
    --model slip --controller reactive --single-lap
```

## CLIs

### `lap.py` — simulator

```
python lap.py <car_dir> <track> <driver_json> [options]
```

**Positional arguments:**

| Argument | Description |
|---|---|
| `<car_dir>` | Path to `cars_csv/<car>` (or directly to the `data/` subfolder). |
| `<track>` | Track CSV path, JSON path, or built-in name (`monza`, `spa`, `nurburgring`, `brands_hatch`). |
| `<driver_json>` | Path to a driver JSON (see schema below). |

**Stint and tyre flags:**

| Flag | Default | Description |
|---|---|---|
| `--laps N` | `2` | Number of laps to simulate (1–50). `--laps 1` = single-lap legacy; `--laps 2` = v1.1 two-lap back-compat; `--laps N` for N≥3 enables full stint mode with per-wheel tyre state. |
| `--single-lap` | — | Alias for `--laps 1`. Mutually exclusive with `--laps`. |
| `--compound <name>` | car default | Active tyre compound. Case-insensitive; trailing parenthetical stripped (e.g. `"Semislicks (SM)"` resolves to `Semislicks`). Unknown name prints available compounds and exits. |
| `--setup <path>` | — | Path to a setup JSON (`setups/<car>_*.json`). Provides cold pressures, ambient temp, and optional compound. |
| `--pressure FL=...,FR=...,RL=...,RR=...` | compound PRESSURE_STATIC | Per-wheel cold-pressure override (psi). Partial specification allowed. |
| `--ambient-temp-c <T>` | `25.0` | Ambient temperature in °C. Used as the initial tyre temperature and the ideal-gas reference. |

**Inverse-PSI solver flags:**

| Flag | Description |
|---|---|
| `--solve-pressure-for-wear <pct>` | Bisect cold PSI to achieve `<pct>` wear (0.0–1.0) at `--at-lap`. |
| `--at-lap <N>` | Target lap for the wear solver (required with `--solve-pressure-for-wear`). |
| `--target-wheel <wheel>` | Aggregator wheel for the solver: `max` (default), `min`, `avg`, `FL`, `FR`, `RL`, `RR`. |
| `--uniform-pressure` | Solve a single PSI applied to all four wheels instead of per-wheel. |

**Output and misc flags:**

| Flag | Default | Description |
|---|---|---|
| `--ds <m>` | `2.0` | Distance step for the 3-pass solver (metres). Smaller = more accurate, slower. |
| `--all-tracks` | — | Run on all built-in tracks (ignores positional `<track>`). |
| `--no-plot` | — | Skip PNG generation. |
| `--no-telemetry` | — | Skip synthetic telemetry CSV. |
| `--telemetry-dt-ms <ms>` | `10` | Cadence for synthetic telemetry CSV. Pass `100` to restore v1.1 behaviour. |
| `--validate-against <real.csv>` | — | Compare sim lap 2 against a real AC lap. |
| `--bin-m <m>` | `100` | Bin width for `--validate-against` delta table. |
| `--per-corner` | — | Per-corner delta table instead of per-bin. |

**v3 slip-model flags (research path):**

| Flag | Default | Description |
|---|---|---|
| `--model {point-mass,slip}` | `point-mass` | Simulator model. `point-mass` is the v2 default. `slip` runs the v3 Pacejka + RK4 ODE + driver-controller stack. CSV-backed tracks only. |
| `--controller {reactive,mpc}` | `reactive` | Slip-model controller. `reactive` is the shipped v3 path (preview-Stanley + slip-band P-loop). `mpc` is experimental / parked. Ignored when `--model point-mass`. |
| `--plan-source {v2,v3_dp}` | `v3_dp` | Target speed plan source for the slip controller. `v3_dp` is the forward-backward DP against the fitted Pacejka envelope. `v2` is the regression path (legacy point-mass plan). Ignored when `--model point-mass`. |
| `--dp-safety-margin <m>` | `0.94` | DP planner safety margin (spec §23.2-5.0.1.5). Only with `--model slip --plan-source v3_dp`. |
| `--chicane-safety-mult <m>` | `0.80` | Conservative multiplier on `v_corner` for tight-radius segments (spec §23.2-5.0.2). 5-segment ramp-in / ramp-out around each flagged cluster. Pass `1.0` to disable. Only with `--model slip --plan-source v3_dp`. |
| `--chicane-radius-thresh <m>` | `60.0` | Segments with `radius_m` below this are flagged "tight" and receive the chicane safety multiplier. Only with `--model slip --plan-source v3_dp`. |

The four `--mpc-*` and `--static-fz` flags (see `python lap.py --help`) drive
the parked MPC controller; they have no effect with `--controller reactive`.

**Outputs** (written next to the track CSV, or in cwd for built-ins):

| File | Description |
|---|---|
| `<stem>__<driver>_sim_trace.csv` | Per-point trace with `lap` column. |
| `<stem>__<driver>_stint_summary.csv` | Per-lap wear/temp/pressure/lap-time block (stint mode). |
| `<stem>__<driver>_sim_vs_ai.png` | Speed overlay (sim lap 2 vs AI reference). |
| `<stem>__<driver>_sim_telemetry.csv` | AC-schema synthetic telemetry. 12 per-wheel state columns appended in stint mode. |
| `<stem>__<driver>_validation_bins.csv` | Delta table vs real lap (`--validate-against`). |
| `<stem>__<driver>_validation_overlay.png` | Speed overlay vs real lap. |

---

### `fit_driver.py` — derive a driver JSON from AC telemetry

```
python fit_driver.py <car_dir> <track_csv> <output.json> \
                     [<lap1.csv> <lap2.csv> ...] \
                     [--laps-glob "<pattern>"] [--newest 10] \
                     [--ds 2.0] [--name <n>] [--no-validate] [--no-plot] [--lap {1,2}]
```

- Requires **≥2 laps** after lap selection.
- `--laps-glob` is mutually exclusive with positional CSVs.
- `--newest N` (default 10, minimum 2) keeps the freshest N laps by file mtime.

The output JSON includes:

- `skill_pct` — 85th percentile of pooled lateral-G utilisation across all laps.
- `consistency_sigma` — scaled stdev of the utilisation pool (seconds).
- `profile.dynamic` — measured `driver_tau_s`, `trail_brake_m`, `throttle_ramp_m`,
  `pedal_press_rate_per_s`, `steering_aggression_deg_per_s` plus per-field
  `measured` booleans and `sample_counts`.
- `tyre_calibration` — four thermal/wear knobs (`k_friction`, `h`, `C_thermal`,
  `k_wear`). Fitted when input telemetry contains per-wheel state channels
  (`tyreTempFL/...`, `tyreWearFL/...`, `wheelsPressureFL/...`); otherwise written
  with `measured: false` and hand-defaults.

By default runs a two-lap validation sim and reports sim lap 2 vs the mean real
lap time.

---

### `prep/prep_car.py` — decrypt an AC car

```
python prep/prep_car.py cars_in/<car>
```

Writes `cars_csv/<car>/data/*.ini` and `*.lut`.

---

### `prep/prep_track.py` — parse AC AI lines into per-point CSV

```
python prep/prep_track.py tracks_in/<track> [--ds 1.0] [--layouts all|<l1>,<l2>]
```

Writes `tracks_csv/<track>/layout_<L>.csv`. Columns: `index, distance_m,
segment_length_m, x, y, z, elevation_m, gradient_pct, radius_m, speed_ms,
speed_kmh, width_left_m, width_right_m, width_total_m`.

Note: `speed_ms` and width columns are zeroed unless the committed
`tracks_csv/` reference files are used. Extending `prep/decode_track.py` to read
the AC AI detail block is a known backlog item.

---

### `analysis/corner_analysis.py` — corner classification

```
python analysis/corner_analysis.py <track_csv> [--config tracks_config.json] \
                                   [--no-plot] [--no-json]
```

Telemetry-free. Reads the track CSV and `tracks_config.json` corner thresholds.

Writes:
- `<stem>_corners.json` — corner notation (spec §17.4)
- `<stem>_corner_map.png`, `<stem>_speed_vs_position.png`

`tracks_config.json` controls the radius thresholds used to classify corners as
hairpin (≤60 m radius), tight (≤150 m), sweeper (≤400 m), or straight.

---

## Driver JSON schema (v2)

```json
{
  "name": "tomas_full",
  "skill_pct": 1.0,
  "consistency_sigma": 1.5,
  "driver_tau_s": 0.0344,
  "trail_brake_m": 5.51,
  "throttle_ramp_m": 29.01,
  "profile": {
    "dynamic": {
      "driver_tau_s": 0.0344,
      "trail_brake_m": 5.51,
      "throttle_ramp_m": 29.01,
      "pedal_press_rate_per_s": 11.6167,
      "steering_aggression_deg_per_s": 20.0,
      "measured": {
        "driver_tau_s": true,
        "trail_brake_m": true,
        "throttle_ramp_m": true,
        "pedal_press_rate_per_s": true,
        "steering_aggression_deg_per_s": true
      },
      "sample_counts": {
        "pedal_leading_edges": 47,
        "brake_taper_segments": 46,
        "throttle_ramp_segments": 46,
        "steering_samples": 27569,
        "steering_unit_detected": "rad"
      }
    }
  },
  "tyre_calibration": {
    "k_friction": 0.184785,
    "h": 112.5514,
    "C_thermal": 44221.83,
    "k_wear": 6.018992601918139e-08,
    "measured": true,
    "source": {
      "telemetry_csvs": ["samples/aclog/tomas_nurburgring_bmw_1m_Lap5_full.csv"],
      "compound": "Semislicks",
      "fit_rmse_temp_C": 5.4687,
      "fit_rmse_wear_pct": 0.0812,
      "fit_rmse_pressure_psi": 1.0031,
      "fitted_at": "2026-05-14T12:18:00Z"
    }
  },
  "source": {
    "telemetry_csvs": ["samples/aclog/tomas_nurburgring_bmw_1m_Lap5_full.csv"],
    "track_csv": "tracks_csv/ks_nurburgring/layout_sprint_a.csv",
    "car_data_dir": "cars_csv/bmw_1m",
    "n_laps": 5,
    "n_finished_laps": 5,
    "real_lap_times_s": [107.56, 108.479, 108.16, 109.52, 117.56],
    "real_lap_time_s": 110.256,
    "pooled_sample_count": 17101,
    "sim_lap_time_s": 102.886,
    "delta_s": -7.37,
    "lap_selection": {
      "rule": "newest-5-to-10",
      "candidates_considered": 5,
      "selected_count": 5,
      "selected_sources": ["samples/aclog/tomas_nurburgring_bmw_1m_Lap5_full.csv"]
    },
    "fitted_at": "2026-05-14T12:18:00Z",
    "fit_version": "2"
  }
}
```

**Field reference:**

| Field | Type | Description |
|---|---|---|
| `name` | string | Used in output filenames. |
| `skill_pct` | float (0, 1] | Multiplier on tyre grip; 85th-percentile lateral-G utilisation measured from telemetry. |
| `consistency_sigma` | float ≥0 | Seconds; >0 triggers 20-run Monte-Carlo on lap 2. |
| `driver_tau_s` | float | Median pedal-press time-to-50% (s). **Statistic only — not consumed by the sim** (v1.2.1). |
| `trail_brake_m` | float | Metres over which brake tapers from 1.0 to 0.0 at corner entry. `0` disables. |
| `throttle_ramp_m` | float | Metres over which throttle ramps to 1.0 at corner exit. `0` disables. |
| `profile.dynamic` | object | Measured driver-profile signals (v1.2). Per-field `measured` booleans flag telemetry-measured vs hand-default. |
| `tyre_calibration` | object | Four thermal/wear knobs. `measured: true` when fit from per-wheel state channels in telemetry. `tyre_calibration.source.compound` records which compound the fit used. |
| `source` | object | Provenance. `fit_version: "2"` when `profile.dynamic` is populated. |

Pre-v1.2 driver JSONs (no `profile` block) continue to load and produce identical
sim output. `profile.dynamic.<field>` wins over the top-level mirror when both
are present.

---

## Setup JSON schema

```json
{
  "car": "bmw_1m",
  "name": "default",
  "pressures_psi": {"FL": 42.0, "FR": 42.0, "RL": 43.0, "RR": 43.0},
  "ambient_temp_C": 25.0,
  "compound": "Street"
}
```

`pressures_psi` is optional. When omitted, the sim falls back to the active
compound's `PRESSURE_STATIC` from `tyres.ini`. `compound` is also optional; when
present it is the level-2 fallback if `--compound` is not passed on the CLI.

---

## Two-lap output (v1.1 default)

Every `lap.py` run simulates at minimum two laps:

- **Lap 1 (standing):** starts from rest. Always deterministic.
- **Lap 2 (flying):** starts at lap 1's end-of-lap speed. Monte-Carlo noise (if
  `consistency_sigma > 0`) applies here only.

The trace CSV and synthetic telemetry carry a trailing `lap` column. The comparison
plot and `--validate-against` both target lap 2. Use `--single-lap` (or `--laps 1`)
to get lap 1 only.

---

## Asymmetric pressure model (v1.3)

Below `PRESSURE_IDEAL`: `f_pressure_grip(p) = 1.0` (no grip penalty; the small
real-world contact-patch bonus is ignored in v1.3). Rolling-resistance drag
increases via `f_pressure_drag(p)`.

Above `PRESSURE_IDEAL`: grip drops via a one-sided quadratic; drag decreases
slightly.

`combined_grip_envelope(state, model)` returns `(mu_x_scale, mu_y_scale, drag_scale)`.
`drag_scale` multiplies total drag force in the 3-pass solver. At `PRESSURE_IDEAL`
both scales are 1.0 — no change to the baseline lap time.

See `docs/architecture-lap-simulation-stint-v1_3-asymmetric-pressure.md` and spec
§21.3 for the curve definitions and acceptance-test results.

---

## Multi-compound support (v2)

`car.py` parses every compound declared in `tyres.ini`. For the BMW M1:
- **Compound 0 — Street:** `PRESSURE_STATIC` 35/35 psi, `PRESSURE_IDEAL` 42/43 psi.
- **Compound 1 — Semislicks:** `PRESSURE_STATIC` 28/28 psi, `PRESSURE_IDEAL` 33/34 psi.

The car's `[COMPOUND_DEFAULT].INDEX` selects which compound is active when neither
`--compound` nor a setup JSON's `compound` field is specified.

`tyre_calibration` in the driver JSON is **car- and compound-coupled**. A calibration
fit against Tomas's Semislicks laps should not be used when simulating on Street
tyres. See the "Common pitfalls" section below.

---

## Common pitfalls

**Wrong compound selected by default.** If you omit `--compound` and the car's
default is Semislicks (BMW M1: `[COMPOUND_DEFAULT].INDEX = 1`), you get Semislicks
even when you intend Street. Check the `Compound: ...` line in stdout.

**Passing hot-pressure targets as cold setup.** `--pressure` takes **cold** values.
The sim heats tyres from the ambient starting temperature; hot pressures are
reported in the per-lap block and `stint_summary.csv` but are not what you pass in.

**`tyre_calibration` from a different car.** The four knobs (`k_friction`, `h`,
`C_thermal`, `k_wear`) were fit against a specific car's tyre physics. Using
Tomas's BMW M1 / Semislicks calibration to simulate a different car or compound
will produce inaccurate per-wheel state. Run `fit_driver.py` against telemetry
from the target car to get matched calibration.

---

## Folder layout

```
LapTimeEstimator/
├── lap.py                        # sim CLI
├── fit_driver.py                 # telemetry -> driver JSON CLI
├── _bootstrap.py                 # adds src/ to sys.path for scripts
├── src/lap_estimator/            # core library
│   ├── car.py                    # Car + Compound dataclasses, tyres.ini parsing
│   ├── track.py                  # Track.from_csv, built-in tracks
│   ├── driver.py                 # Driver.load, tyre_calibration field
│   ├── driver_fit.py             # fit_driver(), fit_tyre_calibration()
│   ├── profile_dynamics.py       # measure_dynamics() — five driver-profile signals
│   ├── simulator.py              # simulate(), simulate_stint(), StintResult
│   ├── tyre_state.py             # TyreState, combined_grip_envelope (3-tuple), GripScaledCar
│   ├── setup.py                  # Setup.load, resolve_compound, resolve_setup
│   ├── solve_setup.py            # inverse-PSI solver
│   ├── telemetry.py              # read_ac_log, merge_with_track
│   ├── sim_telemetry.py          # write_synthetic_log (10 ms default)
│   ├── report.py                 # write_trace_csv, write_stint_summary_csv, plots
│   └── validate.py               # validate_lap, write_bins_csv
├── prep/                         # AC-content preparation
│   ├── decode_acd.py
│   ├── decode_track.py
│   ├── prep_car.py
│   └── prep_track.py
├── analysis/
│   └── corner_analysis.py
├── drivers/                      # tracked driver JSONs
├── setups/                       # cold-pressure + compound setup JSONs
│   └── bmw_1m_default.json
├── cars_csv/<car>/data/          # tracked decrypted car data
├── tracks_csv/<track>/           # tracked per-point track CSVs + outputs
├── samples/aclog/                # tracked sample AC telemetry
├── cars_in/, tracks_in/          # GITIGNORED raw AC content
└── tracks_config.json            # corner-classification thresholds + colours
```
