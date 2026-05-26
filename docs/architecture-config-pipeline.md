# Architecture: configuration pipeline

Spec: `dev-planning/lap-simulation-csv-driver/spec.md` §7.10, §21.7, §21.11,
§6.14, §13.14. Relevant prior docs:
`architecture-lap-simulation-stint-v2.md` (per-wheel state),
`architecture-lap-simulation-stint-v1_3-asymmetric-pressure.md` (asymmetric
pressure model).

---

## Overview

Every `lap.py` or `fit_driver.py` invocation draws from five distinct config
sources. The five sources have a fixed resolution order for the three fields that
have multiple potential origins — compound, pressures, and ambient temperature.
This document describes what each source contains, where it lives in code, the
resolution rules, and the data flow from raw files to the first call to
`simulate_stint`.

---

## The five config sources

### 1. Car config

**Location:** `cars_csv/<car>/data/` — a directory of decrypted AC ini and lut
files. Never edited by hand; produced by `prep/prep_car.py` from
`cars_in/<car>/data.acd`.

**What it provides:**
- Engine torque curve (`power.lut`)
- Drivetrain ratios and type (RWD/FWD/AWD) (`drivetrain.ini`)
- Aero coefficients and LUT refs (`aero.ini`)
- Brake torque and balance (`brakes.ini`)
- Suspension geometry including wheelbase, CG location, and track width (`suspensions.ini`)
- **All tyre compounds** defined in `tyres.ini`: per-axle grip coefficients
  (`DY0/DX0`), speed sensitivity, `PRESSURE_IDEAL`, `PRESSURE_STATIC`,
  `PRESSURE_D_GAIN`, wear-curve LUT paths, thermal-performance LUT paths.
- `[COMPOUND_DEFAULT] INDEX` — which compound is active when nothing overrides it.

**Parsed by:** `src/lap_estimator/car.py`

Key entry point: `Car(data_dir)` (or `Car.from_dir(data_dir)`). The constructor
calls `_parse_compounds(ini)` which scans `tyres.ini` section names for the
families `FRONT`, `REAR`, `THERMAL_FRONT`, `THERMAL_REAR`, groups by numeric
suffix (un-suffixed = index 0, `_1` = index 1, …), and builds one frozen
`Compound` dataclass per index. Parsing raises a clear error at load time if any
of the four families is missing for a given index.

`car.compounds` is a `list[Compound]`. `car.default_compound_index` reflects
`[COMPOUND_DEFAULT].INDEX` or 0. `car.default_compound` is the resolved
`Compound` at that index.

`car.find_compound(query)` matches case-insensitively against `Compound.name` and
`Compound.short_name`, stripping any trailing parenthetical — so
`"Semislicks (SM)"` (the string AC's telemetry puts in `tyreCompound`) resolves
to the `"Semislicks"` compound.

**Legacy back-compat:** `car.tyre_dy0_f` and similar direct attributes are pinned
to compound 0 regardless of `[COMPOUND_DEFAULT].INDEX`. These feed the v1.2.1
single-lap simulation path which must not change (spec §11.30, §11.31).
Active-compound baselines for stint mode flow through `GripScaledCar(compound=...)`
rather than mutating the `Car` object.

---

### 2. Track config

**Location:** `tracks_csv/<track>/layout_*.csv` (geometry) and `tracks_config.json`
(corner-classification thresholds, corner-label minimum length, plot colours).

**What it provides:**
- Per-point geometry: `distance_m`, `radius_m`, `gradient_pct`, `x/y/z`,
  `speed_ms`, widths.
- Corner classification thresholds used by `analysis/corner_analysis.py` (not
  by the lap simulator directly).

**Parsed by:** `src/lap_estimator/track.py` (`Track.from_csv`) for the lap
simulator. `analysis/corner_analysis.py` additionally reads `tracks_config.json`
directly.

`Track.from_csv` is the only track source that interacts with the sim. The
`tracks_config.json` file is **not** loaded during a `lap.py` run — it is
consumed only by the standalone corner-analysis tool.

---

### 3. Driver config

**Location:** `drivers/<name>.json` — hand-authored or produced by `fit_driver.py`.

**What it provides:**
- `skill_pct`: grip-multiplier applied uniformly across all sim points.
- `consistency_sigma`: Monte-Carlo lap-time noise (lap 2 only).
- `trail_brake_m`, `throttle_ramp_m`: corner-entry and corner-exit shaping for
  synthetic telemetry emission.
- `driver_tau_s`: measured pedal-press statistic (loaded, not consumed by sim).
- `profile.dynamic`: measured driver-dynamics signals with `measured` booleans
  and `sample_counts` (v1.2).
- `tyre_calibration`: four thermal/wear knobs (`k_friction`, `h`, `C_thermal`,
  `k_wear`) plus `measured` flag and provenance. Fit from per-wheel state
  channels when available; otherwise hand-defaults with `measured: false`.

**Parsed by:** `src/lap_estimator/driver.py` (`Driver.load`).

Field-resolution precedence inside `Driver.load` (spec §13.11):
1. `profile.dynamic.<field>` (v1.2 measured value)
2. Top-level `<field>` (v1.1 mirror or hand-authored)
3. Hard-coded default (`driver_tau_s=0.12`, `trail_brake_m=30.0`,
   `throttle_ramp_m=40.0`)

Pre-v1.2 JSONs (no `profile` block) hit tier 2 and produce identical sim output.

`driver.get_tyre_calibration()` returns a `TyreCalibration` dataclass. When
`measured=false`, the four knobs carry hand-defaults and the per-wheel state
evolution still runs but produces a physically approximate result.

---

### 4. Setup config

**Location:** `setups/<car>_<scenario>.json` (optional; omitting it falls back to
the active compound's `PRESSURE_STATIC`).

**What it provides:**
- `pressures_psi` (optional dict `{FL, FR, RL, RR}`): cold pressures in psi.
  When the field is absent, `resolve_setup` fills from the active compound's
  `PRESSURE_STATIC`.
- `ambient_temp_C` (default 25.0): ambient temperature used as initial tyre
  temperature and ideal-gas reference.
- `compound` (optional string): level-2 fallback in compound resolution.

Example — `setups/bmw_1m_default.json`:
```json
{
  "car": "bmw_1m",
  "name": "default",
  "pressures_psi": {"FL": 42.0, "FR": 42.0, "RL": 43.0, "RR": 43.0},
  "ambient_temp_C": 25.0,
  "compound": "Street"
}
```

**Parsed by:** `src/lap_estimator/setup.py` (`Setup.load`, `Setup.default_for_car`).

`Setup.default_for_car(car, compound)` reads the **active compound's
`PRESSURE_STATIC`** per axle (not `PRESSURE_IDEAL`). `PRESSURE_STATIC` is what a
driver dials into the tyre app before going out; `PRESSURE_IDEAL` is the hot
target for peak grip after warm-up. For BMW M1 Street: 35/35 psi. For Semislicks:
28/28 psi.

---

### 5. CLI flags

**Source:** `lap.py` or `fit_driver.py` argparse.

| Flag | Overrides |
|---|---|
| `--compound <name>` | Setup-JSON `compound`, car default (level 1 in compound resolution) |
| `--pressure FL=...,FR=...,RL=...,RR=...` | Setup-JSON `pressures_psi` and compound `PRESSURE_STATIC` (level 1 in pressure resolution) |
| `--ambient-temp-c <T>` | Setup-JSON `ambient_temp_C` and the 25.0 °C default |

Partial `--pressure` specification (e.g. `FL=26,FR=26` without RL/RR) is allowed;
unspecified wheels retain their setup-file or compound-default values.

---

## Resolution rules

### Compound

```
1. --compound <name>        (lap.py CLI)           → source: "cli"
2. setup JSON `compound`                           → source: "setup"
3. telemetry's most-common tyreCompound            → source: "telemetry"
                                                      (fit_driver.py only; unknown
                                                       name warns + falls through)
4. car.compounds[car.default_compound_index]       → source: "car-default"
```

Implemented in `setup.resolve_compound(car, *, cli_name, setup, telemetry_name)`.
Levels 1 and 2 raise `ValueError` listing available compound names when the query
does not match. Level 3 is best-effort (caller responsibility to warn on miss).

`lap.py` performs a **two-phase resolution**: it first loads the setup JSON (if
`--setup` was given) to peek its `compound` field, then calls `resolve_compound`
once with both `cli_name` and `setup` populated. This avoids loading the setup
JSON twice and keeps the resolved compound available for the subsequent
`resolve_setup` call.

The resolved compound is logged on stdout:
```
Compound: Semislicks (idx 1) | source: cli
```

### Pressures

```
1. --pressure FL=...,FR=...,RL=...,RR=...          (CLI per-wheel override)
2. setup JSON pressures_psi                        (Setup.load)
3. compound.pressure_static_front / _rear          (Setup.default_for_car)
```

Implemented in `setup.resolve_setup(car_or_data_dir, setup_path, pressure_str, ambient)`.

`Setup.with_overrides(overrides, ambient_override)` merges CLI per-wheel values
on top of whatever base setup was loaded, clamps to `[20.0, 50.0]` psi, and
sets `source = "<prior_source>+cli"` when CLI wins.

The resolved setup is logged on stdout:
```
Setup: tyres.ini+cli | FL=26.0 FR=26.0 RL=26.0 RR=26.0 | ambient=26.0°C | compound=Semislicks
```

### Ambient temperature

```
1. --ambient-temp-c <T>                            (CLI)
2. setup JSON ambient_temp_C                       (Setup.load)
3. DEFAULT_AMBIENT_TEMP_C = 25.0                   (setup.py constant)
```

Ambient temp becomes `TyreState.T_amb` and the ideal-gas reference temperature
`T_cold_K = ambient + 273.15` used in the per-segment pressure update.

---

## Data flow

```
[Preparation — one-time]

cars_in/<car>/data.acd  →  prep_car.py     →  cars_csv/<car>/data/*.ini + *.lut
tracks_in/<track>/      →  prep_track.py   →  tracks_csv/<track>/layout_*.csv

[Fitting — once per driver / car+track combo]

cars_csv/<car>/         ─┐
tracks_csv/<track>/     ─┤
AC telemetry CSVs       ─┘ → fit_driver.py → drivers/<name>.json
                              (resolve compound from tyreCompound,
                               fit skill_pct / consistency_sigma,
                               fit tyre_calibration when per-wheel
                               state channels present)

[Simulation — every lap.py run]

cars_csv/<car>/data/    → Car(data_dir)
                             └── compounds[0..N]
                             └── default_compound_index

tracks_csv/<track>/     → Track.from_csv(layout_*.csv)

drivers/<name>.json     → Driver.load(path)
                             └── skill_pct, consistency_sigma
                             └── trail_brake_m, throttle_ramp_m
                             └── tyre_calibration (k_friction, h, C_thermal, k_wear)

setups/<file>.json      → Setup.load(path)   ─┐
--compound / --pressure                        ├─→ resolve_compound(car, cli, setup)
--ambient-temp-c                               │       → Compound
                                               └─→ resolve_setup(car, setup_path,
                                                     pressure_str, ambient,
                                                     compound=compound)
                                                       → Setup (validated, source tagged)

  (Compound, Setup, Car, Track, Driver)
            │
            ▼
    TyreState.from_setup(setup, car, compound)
      initial: T_core = ambient, wear = 100 %, pressure_psi = cold

            │
            ▼
    simulate_stint(car, track, driver,
                   n_laps=N, setup=setup,
                   calibration=calibration,
                   compound=compound, ds=ds)

        for lap k in 1..N:
            (mu_x, mu_y, drag_scale) = combined_grip_envelope(state, tyre_model)
            inner = _DragScaledCar(car, drag_scale)  [if drag_scale != 1.0]
            scaled = GripScaledCar(inner, mu_x, mu_y, compound=compound)
            lap_result = simulate(scaled, track, driver, ds=ds, two_lap=False)
            update_per_segment(state, lap_result, car, calibration, tyre_model)
        END

            │
            ▼
    StintResult
      .lap_times_s
      .tyre_state_history[0..N]
      .per_lap_sim_results[0..N-1]
      .per_point_states

            │
            ├──→ stdout per-lap block  (print_per_lap_block)
            ├──→ <stem>_stint_summary.csv
            ├──→ <stem>_sim_trace.csv
            ├──→ <stem>_sim_telemetry.csv  (+12 per-wheel state cols in stint mode)
            └──→ <stem>_sim_vs_ai.png
```

---

## Where each function lives

| Responsibility | Module | Entry point |
|---|---|---|
| Car + compound parsing | `src/lap_estimator/car.py` | `Car(data_dir)`, `Car.from_dir`, `Car.find_compound`, `Compound` dataclass |
| Track loading | `src/lap_estimator/track.py` | `Track.from_csv(path)` |
| Driver loading | `src/lap_estimator/driver.py` | `Driver.load(path)`, `driver.get_tyre_calibration()` |
| Setup loading | `src/lap_estimator/setup.py` | `Setup.load(path)`, `Setup.default_for_car(car, compound)` |
| Compound resolution | `src/lap_estimator/setup.py` | `resolve_compound(car, *, cli_name, setup, telemetry_name)` |
| Full setup resolution | `src/lap_estimator/setup.py` | `resolve_setup(car, setup_path, pressure_str, ambient, *, compound)` |
| Pressure string parsing | `src/lap_estimator/setup.py` | `parse_pressure_str("FL=26,FR=26,RL=26,RR=26")` |
| Tyre-state init | `src/lap_estimator/tyre_state.py` | `TyreState.from_setup(setup, car, compound)` |
| Grip envelope + drag scale | `src/lap_estimator/tyre_state.py` | `combined_grip_envelope(state, model)` → `(mu_x, mu_y, drag_scale)` |
| Compound-aware tyre model | `src/lap_estimator/tyre_state.py` | `build_car_tyre_model(car, compound)` → `CarTyreModel` |
| Drag scaling wrapper | `src/lap_estimator/simulator.py` | `_DragScaledCar(car, drag_scale)` |
| Grip + compound scaling | `src/lap_estimator/tyre_state.py` | `GripScaledCar(car, mu_x, mu_y, compound=...)` |
| Multi-lap stint loop | `src/lap_estimator/simulator.py` | `simulate_stint(car, track, driver, *, n_laps, setup, calibration, compound, ds)` |
| Inverse-PSI solver | `src/lap_estimator/solve_setup.py` | `solve_pressure_for_wear(car, track, driver, *, target_wear, target_lap, ...)` |
| CLI orchestration (sim) | `lap.py` | `main()`, `_run_one_track(...)`, `_run_solver(...)` |
| Compound detection from telemetry | `src/lap_estimator/driver_fit.py` | `_resolve_compound_from_frames(merged_frames, car)` |

---

## Worked example: one `lap.py` invocation

```
python lap.py cars_csv/bmw_1m \
    tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    drivers/tomas_full.json \
    --compound Semislicks --pressure FL=26,FR=26,RL=26,RR=26 \
    --ambient-temp-c 26 --laps 3
```

**Step 1 — argparse.** Three positional paths plus flags resolved. `n_laps = 3`.

**Step 2 — Car loading.** `Car("cars_csv/bmw_1m/data")` parses `tyres.ini` and
builds `car.compounds = [Street (idx 0), Semislicks (idx 1)]`.
`car.default_compound_index = 1` (BMW M1 `[COMPOUND_DEFAULT] INDEX=1`).
Legacy `car.tyre_dy0_f` etc. are pinned to compound 0 (Street) for v1.2.1
back-compat.

**Step 3 — Track loading.** `Track.from_csv("tracks_csv/ks_nurburgring/layout_sprint_a.csv")`
builds the per-point geometry array.

**Step 4 — Driver loading.** `Driver.load("drivers/tomas_full.json")` reads
`skill_pct=1.0`, `consistency_sigma=1.5`, `trail_brake_m=5.51`,
`throttle_ramp_m=29.01` (from `profile.dynamic` — tier 1), and
`tyre_calibration` with `measured=true` (k_friction/h/C_thermal/k_wear fit from
Tomas's Semislicks laps).

**Step 5 — Compound resolution.** `resolve_compound(car, cli_name="Semislicks", setup=None)`:
level 1 matches → returns `(Semislicks compound, "cli")`. Stdout: `Compound: Semislicks (idx 1) | source: cli`.

**Step 6 — Setup resolution.** `resolve_setup(car, None, "FL=26,FR=26,RL=26,RR=26", 26.0, compound=semislicks_compound)`:
no setup file → base pressures from `compound.pressure_static_front/rear` = 28 psi
→ CLI override applies all four wheels to 26 psi → ambient overridden to 26.0 °C.
Result: `Setup(pressures_psi={FL:26, FR:26, RL:26, RR:26}, ambient_temp_C=26.0, source="tyres.ini+cli")`.

**Step 7 — Stint engagement.** `n_laps=3` and `calibration.measured=True` both
satisfy the `use_stint` predicate in `_run_one_track`.

**Step 8 — `simulate_stint` call.** `TyreState.from_setup(setup, car, semislicks_compound)`
initialises: `T_core = 26.0 °C` per wheel, `wear_pct = 100.0`, `pressure_psi = 26.0`.
`build_car_tyre_model(car, semislicks_compound)` loads the Semislicks wear
(`semislicks_front.lut`, `semislicks_rear.lut`) and thermal LUTs
(`[THERMAL_FRONT_1]`, `[THERMAL_REAR_1]`).

For each of the 3 laps:
1. `combined_grip_envelope(state, tyre_model)` → `(mu_x_scale, mu_y_scale, drag_scale)`.
   On lap 1 at cold=26 psi: Semislicks `PRESSURE_IDEAL_FRONT=33`, so pressure is below
   ideal; `f_pressure_grip=1.0` (no penalty); `f_pressure_drag > 1.0` (rolling resistance
   increase); `drag_scale > 1.0`.
2. `inner = _DragScaledCar(car, drag_scale)` since `drag_scale != 1.0`.
3. `scaled = GripScaledCar(inner, mu_x_scale, mu_y_scale, compound=semislicks_compound)` —
   swaps in the Semislicks `DY0/DX0/speed_sensitivity` baseline.
4. `simulate(scaled, track, driver, ...)` runs the 3-pass solver.
5. `update_per_segment(state, ...)` evolves temperature, wear, and pressure for
   every segment of that lap.

**Step 9 — Outputs.** `StintResult` written to:
- `tracks_csv/ks_nurburgring/layout_sprint_a__tomas_full_sim_trace.csv`
- `tracks_csv/ks_nurburgring/layout_sprint_a__tomas_full_stint_summary.csv`
- `tracks_csv/ks_nurburgring/layout_sprint_a__tomas_full_sim_telemetry.csv` (+12 per-wheel state cols)
- `tracks_csv/ks_nurburgring/layout_sprint_a__tomas_full_sim_vs_ai.png`

---

## Common pitfalls

**Forgetting `--compound` when running with a Semislicks-fit driver.** The BMW M1
`[COMPOUND_DEFAULT].INDEX = 1` means the car default is Semislicks. If you omit
`--compound`, the sim still selects Semislicks (via level 4). This is typically
correct but surprises users who expect Street. Always check the `Compound: ...`
stdout line.

**Passing hot pressures as `--pressure`.** `--pressure` takes **cold** values.
The sim heats tyres from ambient. Hot pressures appear in the per-lap summary and
`stint_summary.csv`, not on the CLI.

**Mixing calibration from one car with another.** `tyre_calibration` knobs
(`k_friction`, `h`, `C_thermal`, `k_wear`) were fit against a specific car's
`tyres.ini` LUTs and a specific compound. Using Tomas's BMW M1 / Semislicks
calibration while simulating a different car will produce incorrect per-wheel
state. Re-run `fit_driver.py` against telemetry from the target car.

**Using `PRESSURE_IDEAL` for cold setup.** `Setup.default_for_car` reads
`PRESSURE_STATIC` (the cold-dial-in value), not `PRESSURE_IDEAL` (the hot target).
For BMW M1 Street, `PRESSURE_IDEAL=42` but `PRESSURE_STATIC=35`. If you manually
author a setup JSON and copy `PRESSURE_IDEAL` as the cold value, the sim will start
with over-pressure and apply the grip-reduction quadratic from lap 1.

**Level-2 compound override from a stale setup JSON.** If your setup JSON has
`"compound": "Street"` but you pass `--compound Semislicks` on the CLI, the CLI
wins (level 1). But if you pass only `--setup` and forget `--compound`, the setup
JSON's compound field controls. Verify the `Compound: ...` stdout line.
