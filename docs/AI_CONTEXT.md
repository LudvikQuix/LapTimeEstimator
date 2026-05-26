# LapTimeEstimator — AI Context Document

This document provides the knowledge an AI assistant needs to work with this
project effectively. It is kept up to date as features ship.

**Current state: v2 / v1.3** (multi-compound tyres.ini parsing, per-wheel tyre
state, asymmetric pressure model, inverse-PSI solver). See spec
`dev-planning/lap-simulation-csv-driver/spec.md` for full §-numbered requirements.

**v3 slip path:** also shipped (research-quality) as
`--model slip --controller reactive` (Pacejka envelope + RK4 ODE + reactive
preview-Stanley controller + Phase 5.0.2 chicane safety cap). The v3 **MPC**
stack (`--controller mpc`, files `src/lap_estimator/dynamics/mpc_*.py`) is
**EXPERIMENTAL / PARKED** after 7 consecutive controller iterations
(Phase 4.2 → Phase 5.0.5) failed §11.55 at the Sprint A chicane. See
`docs/architecture-v3-shipping-state.md` for the shipping decision, known
limitations per mode, and what would resurrect the MPC work.

---

## Project purpose

Point-mass lap time estimator that uses Assetto Corsa (AC) car physics and track
geometry to simulate lap times. The 3-pass solver (corner speeds → forward
acceleration → backward braking) is retained from v1 through v2. Per-wheel tyre
state modulates a scalar `(mu_x_scale, mu_y_scale, drag_scale)` envelope applied
between laps; per-wheel force dynamics stay in v3 (backlog, spec §19).

The headline workflow: **fit a driver from real AC telemetry** (`fit_driver.py`)
then **predict their lap time on a track they have never run** (`lap.py`).
Post-drive validation (`--validate-against`) closes the loop.

---

## Repository layout

```
LapTimeEstimator/
├── lap.py                        # sim CLI (v2: --laps, --compound, --pressure, etc.)
├── fit_driver.py                 # telemetry -> driver JSON CLI
├── _bootstrap.py                 # puts src/ on sys.path
├── src/lap_estimator/
│   ├── car.py                    # Car + Compound dataclasses; multi-compound parsing
│   ├── track.py                  # Track.from_csv; BUILTIN_TRACKS
│   ├── driver.py                 # Driver.load; TyreCalibration dataclass
│   ├── driver_fit.py             # fit_driver(car, merged_frames, track); fit_tyre_calibration
│   ├── profile_dynamics.py       # measure_dynamics() — five driver-profile signals
│   ├── simulator.py              # simulate(); simulate_stint(); StintResult; _DragScaledCar
│   ├── tyre_state.py             # TyreState; combined_grip_envelope (3-tuple); GripScaledCar
│   ├── setup.py                  # Setup; resolve_compound; resolve_setup
│   ├── solve_setup.py            # solve_pressure_for_wear (bisection)
│   ├── telemetry.py              # read_ac_log; merge_with_track; PER_WHEEL_STATE_CHANNELS
│   ├── sim_telemetry.py          # write_synthetic_log (default 10 ms)
│   ├── report.py                 # write_trace_csv; write_stint_summary_csv; plots
│   └── validate.py               # validate_lap; write_bins_csv
├── prep/                         # AC-content preparation (decode_acd, decode_track, prep_car, prep_track)
├── analysis/corner_analysis.py   # telemetry-free corner classifier
├── drivers/                      # tracked driver JSONs (v2 schema)
├── setups/                       # cold-pressure + compound setup JSONs
├── cars_csv/<car>/data/          # tracked decrypted AC ini/lut files
├── tracks_csv/<track>/           # per-point track CSVs + sim outputs
├── samples/aclog/                # tracked sample AC telemetry CSVs
├── cars_in/, tracks_in/          # GITIGNORED raw AC content
└── tracks_config.json            # corner-classification thresholds + colours
```

---

## Key AC data files (after decryption)

### Car physics files

| File | Purpose | Key fields |
|---|---|---|
| `car.ini` | Basic car info | `TOTALMASS`, `STEER_LOCK`, `FUEL`, `MAX_FUEL` |
| `engine.ini` | Engine | `LIMITER`, `INERTIA`, turbo `MAX_BOOST`/`WASTEGATE` |
| `power.lut` | Torque curve | `RPM\|torque_Nm` per line |
| `drivetrain.ini` | Gearbox + diff | `TYPE` (RWD/FWD/AWD), gear ratios, `FINAL`, lock % |
| `tyres.ini` | Tyre model (multi-compound) | `DY0`/`DX0` per compound, `PRESSURE_IDEAL`, `PRESSURE_STATIC`, `PRESSURE_D_GAIN`, wear/thermal LUT paths |
| `aero.ini` | Aerodynamics | Wing `CHORD`/`SPAN`/`CD_GAIN`/`CL_GAIN`, LUT refs |
| `brakes.ini` | Brakes | `MAX_TORQUE`, `FRONT_SHARE` |
| `suspensions.ini` | Suspension geometry | `WHEELBASE`, `CG_LOCATION`, `TRACK` |

### LUT format

Pipe-delimited: `input_value|output_value`

Examples:
- `power.lut`: `RPM|torque_Nm`
- `tcurve_street.lut`: `temperature_C|grip_multiplier` (thermal performance)
- `street_front.lut`: `wear_km|grip_multiplier` (wear curve)
- `wing_body_AOA_CD.lut`: `angle_degrees|Cd`

### Track CSV format

| Column | Type | Description |
|---|---|---|
| `index` | int | Point index (0-based) |
| `distance_m` | float | Cumulative distance from start (m) |
| `segment_length_m` | float | Distance to next point (m) |
| `x`, `y`, `z` | float | World coordinates (m); y = elevation |
| `elevation_m` | float | Elevation above reference (same as y) |
| `gradient_pct` | float | Road gradient (positive = uphill) |
| `radius_m` | float | Corner radius (m); ~2000 = straight |
| `speed_ms` | float | AI reference speed (m/s) |
| `speed_kmh` | float | AI reference speed (km/h) |
| `width_left_m`, `width_right_m`, `width_total_m` | float | Track widths |

Two CSV variants per layout: `layout_*.csv` (centerline) and `*_ideal_line.csv`
(AI racing line). The simulator uses the ideal line when available.

---

## Physics model

### 3-pass solver

1. **Corner pass:** maximum speed at each point from `sqrt(mu_y * (m*g + Fdw(v)) * R / m)`.
2. **Forward pass:** limit speed by traction capability from the previous point.
3. **Backward pass:** limit speed by braking capability into the next point.
4. **Time integration:** `dt = ds / v_avg` per segment.

### Engine / drivetrain

- Torque from `power.lut` at current RPM; turbo multiplied by `(1 + wastegate_boost)`.
- `wheel_torque = engine_torque × gear_ratio × final_ratio`.
- Optimal gear selected for maximum wheel torque at current speed.

### Tyres (per compound)

- Lateral grip: `mu_y = DY0 / (1 + SPEED_SENSITIVITY × v)`
- Longitudinal grip: `mu_x = DX0 / (1 + SPEED_SENSITIVITY × v)`
- Active compound selected at sim entry; `GripScaledCar(car, mu_x_scale, mu_y_scale, compound=...)` wraps the baseline grip.

### Aerodynamics

- Drag: `F_drag = 0.5 × rho × Cd × A × v²`
- Downforce: `F_down = 0.5 × rho × |Cl| × A × v²`
- Frontal area approximated from body wing `CHORD × SPAN`.

---

## Per-wheel tyre state (v2)

Per-wheel state struct: `{T_core[w], wear_pct[w], pressure_psi[w]}` for
`w ∈ {FL, FR, RL, RR}`.

**Thermal Euler step (per segment, per wheel):**
```
dE[w]      = Fz[w] × (|a_long| × long_share[w] + |a_lat|) × dt
P_heat[w]  = k_friction × dE[w] / dt
dT/dt[w]   = (P_heat[w] - h × (T[w] - T_amb)) / C_thermal
```

**Wear update:**
```
penalty      = 1 / max(f_temp(T[w]), 0.3)
dwear/dt[w]  = k_wear × dE[w]/dt × penalty   (pct/s)
wear[w]     -= dwear/dt[w] × dt              (clamped [0, 100])
```

**Pressure update (ideal gas, isovolumetric):**
```
p[w] = p_cold[w] × (T[w] + 273.15) / T_cold_K
```

**Scalar reduction (between laps):**
```
g[w]       = f_temp(T[w]) × f_wear(wear[w]) × f_pressure_grip(p[w])
g_combined = 0.5 × (min(g_FL, g_FR) + min(g_RL, g_RR))
g_combined = max(g_combined, 0.30)    # numerical floor
(mu_x_scale, mu_y_scale) = (g_combined, g_combined)
```

---

## Asymmetric pressure model (v1.3)

Two compound-attached functions replace the symmetric v2 quadratic:

**`f_pressure_grip(model, p, *, front)`**
- Returns `1.0` for `p ≤ PRESSURE_IDEAL` (no grip penalty for under-pressure).
- One-sided quadratic penalty for `p > PRESSURE_IDEAL`; clamp `[0.30, 1.0]`.

**`f_pressure_drag(model, p, *, front)`**
- Linear penalty `1 + k_drag × (IDEAL − p) / IDEAL` for `p < PRESSURE_IDEAL`
  (under-pressure → rolling resistance).
- Small linear benefit `1 − k_drag_reduction × (p − IDEAL) / IDEAL` for
  `p > PRESSURE_IDEAL`; clamp `[0.85, 1.50]`.

`k_drag = 0.5`, `k_drag_reduction = 0.1` are hand-defaults on the `Compound`
dataclass in v1.3 (same value for all compounds; telemetry fit deferred to v1.4,
spec §21.10).

`combined_grip_envelope(state, model)` returns `(mu_x_scale, mu_y_scale, drag_scale)`
where `drag_scale = mean(d_FL, d_FR, d_RL, d_RR)`. `drag_scale != 1.0` wraps the
inner car with `_DragScaledCar` before the 3-pass solve.

`f_pressure` is retained as a back-compat alias for `f_pressure_grip` in
`driver_fit.py`'s `lat_g_max` computation.

---

## Multi-compound support (v2, §21.11)

`Car.from_dir(path)` (or `Car(data_dir)`) parses all compounds from `tyres.ini`.
Section-naming convention:
- Un-suffixed `[FRONT]`, `[REAR]`, `[THERMAL_FRONT]`, `[THERMAL_REAR]` = compound 0.
- `[FRONT_1]`, `[REAR_1]`, `[THERMAL_FRONT_1]`, `[THERMAL_REAR_1]` = compound 1.
- Etc. All four families must be present per index or the parser raises at load time.

`[COMPOUND_DEFAULT] INDEX=N` honours a non-zero default compound. BMW M1 has
`INDEX=1` (Semislicks is the default).

`Car.find_compound(query)` matches case-insensitively against `name` and
`short_name`, stripping any trailing parenthetical (so `"Semislicks (SM)"` resolves
to `"Semislicks"`).

Compound resolution precedence (spec §21.11):
1. `--compound <name>` CLI flag (levels 1+2 raise on unknown name).
2. Setup-JSON `compound` field.
3. Telemetry's most-common `tyreCompound` (fitter only; unknown name warns + falls
   through).
4. `car.default_compound`.

Legacy direct attributes (`car.tyre_dy0_f`, `car.tyre_speed_sens_f`, etc.) are
pinned to compound 0 (un-suffixed sections) regardless of `[COMPOUND_DEFAULT].INDEX`
to preserve the v1.2.1 single-lap back-compat path.

---

## Configuration sources and precedence

Five config sources feed every sim run. See
`docs/architecture-config-pipeline.md` for the full data-flow description.

| Source | What it provides | Loaded by |
|---|---|---|
| `cars_csv/<car>/data/*.ini` + `*.lut` | Physics, all compounds | `car.py` `Car` |
| `tracks_csv/<track>/layout_*.csv` + `tracks_config.json` | Geometry + corner thresholds | `track.py` `Track`, `analysis/corner_analysis.py` |
| `drivers/<name>.json` | `skill_pct`, `consistency_sigma`, dynamic profile, `tyre_calibration` | `driver.py` `Driver.load` |
| `setups/<car>_*.json` | Cold pressures, ambient temp, optional compound selection | `setup.py` `Setup.load` |
| CLI flags | `--compound`, `--pressure`, `--ambient-temp-c` — override everything | `lap.py`, `fit_driver.py` |

**Precedence rules:**
- Compound: `--compound` > setup-JSON `compound` > telemetry `tyreCompound` (fitter) > `car.default_compound`.
- Pressures: `--pressure` > setup-JSON `pressures_psi` > compound `PRESSURE_STATIC` (not `PRESSURE_IDEAL`).
- Ambient temp: `--ambient-temp-c` > setup-JSON `ambient_temp_C` > 25.0 °C.
- Driver smoothing: `profile.dynamic.<field>` > top-level mirror field > hand-default.

---

## Driver JSON data formats

### Top-level fields

| Field | Consumed by sim? | Description |
|---|---|---|
| `skill_pct` | Yes | Grip multiplier; also gates skill-percentile step in driver_fit |
| `consistency_sigma` | Yes | MC noise on lap 2 |
| `driver_tau_s` | **No** (statistic only, v1.2.1) | Measured pedal press speed |
| `trail_brake_m` | Yes | Metres of trail-brake taper at corner entry |
| `throttle_ramp_m` | Yes | Metres of throttle ramp at corner exit |

### `profile.dynamic` block (v1.2)

Five measured fields, per-field `measured` booleans, and `sample_counts`.
`profile.dynamic.<field>` wins over top-level when both are present (`Driver.load`
precedence). `driver_tau_s`, `pedal_press_rate_per_s`, `steering_aggression_deg_per_s`
are statistics only — not consumed by the v1.2 simulator.

### `tyre_calibration` block (v2)

| Field | Description |
|---|---|
| `k_friction` | Scales heat input from slip energy (W·m⁻²) |
| `h` | Convective cooling coefficient (W·m⁻²·K⁻¹) |
| `C_thermal` | Thermal mass of the contact patch (J·K⁻¹) |
| `k_wear` | Wear rate constant (pct·s⁻¹ per W·m⁻²) |
| `measured` | `true` when fit from per-wheel state channels in telemetry |
| `source.compound` | Compound the calibration was fit against |

Calibration is car-and-compound-coupled. Using a BMW M1 / Semislicks calibration
to simulate a different car is incorrect.

---

## Synthetic telemetry columns

The sim emits AC-schema telemetry. Required columns (always present):

`timestamp_ms, distanceTraveled, normalizedCarPosition, speedKmh, gas, brake,
steerAngle, gear, rpms, accG_lat, accG_lon, lap`

Per-wheel state columns appended in stint mode (12 columns, spec §7.12):

`tempFL, tempFR, tempRL, tempRR, wearFL, wearFR, wearRL, wearRR,
pressureFL, pressureFR, pressureRL, pressureRR`

Real AC telemetry uses `tyreTempFL`, `tyreWearFL`, `wheelsPressureFL` etc.
Sim-emitted uses the shorter `tempFL`/`wearFL`/`pressureFL` names — the two
naming conventions are intentionally different.

---

## Inverse-PSI solver (v2, spec §21.5)

`solve_pressure_for_wear(car, track, driver, *, target_wear, target_lap, ...)`:

1. **Seed scan:** try PSI = {22, 27, 32, 37, 42, 47}; run `simulate_stint` for each.
2. **Bracket:** find adjacent (lo, hi) pair straddling the target wear.
3. **Bisection:** ≤12 iterations per wheel; converge when `|observed − target| ≤ 0.01`
   or `|psi_hi − psi_lo| ≤ 0.5`.
4. **Greedy per-wheel:** each wheel solved independently (coordinate descent).
   Use `--uniform-pressure` for a single PSI across all four.
5. **Verification stint:** re-run with the recommended PSI and print a per-lap
   wear/temp/pressure table.

---

## Common tasks

- **Add a new car:** run `prep/prep_car.py cars_in/<car>` to decrypt; output
  lands in `cars_csv/<car>/data/`.
- **Add a new track:** run `prep/prep_track.py tracks_in/<track>`.
- **Fit a driver:** `fit_driver.py` against ≥2 real AC telemetry CSVs on a
  specific car+track.
- **Compare compounds:** run `lap.py` twice with `--compound Street` and
  `--compound Semislicks`.
- **Tune accuracy:** smaller `--ds` (e.g. 1.0) increases solver resolution.
- **Extend physics:** the v3 backlog (spec §19) describes slip-based Pacejka
  physics if you need oversteer/understeer fidelity.

---

## v3 slip-based simulator (shipped, research-quality)

- **Shipped path:** `--model slip --controller reactive` — Pacejka Magic Formula,
  friction ellipse, RK4 ODE, reactive preview-Stanley + slip-band P-loop driver
  controller, Phase 5.0.2 chicane safety cap in the DP planner. CSV-backed tracks
  only.
- **Parked path:** `--controller mpc` — receding-horizon MPC with OSQP inner +
  SQP outer loop, Tier 1/2/0 dispatch, dynamic per-axle Fz refresh, Tier-1 emit
  FIR smoother. 7 phases of work; binding chatter sits in the Tier-2 reactive
  sub-controller, not in any layer Phases 5.0–5.0.5 specced to touch. Do not
  use in production.
- **Status as of 2026-05-24:** A physics audit (6-term longitudinal fix,
  twin-turbo curve, I_zz from `[BASIC].INERTIA`, Stanley LPF τ=60 ms, k_cross
  lifted to 0.75) changed the reactive Sprint A result from 0/10 to **7/10 MC
  completions at 2:09.12** (`--inertia-zz 2400`, default chicane cap). 9/10
  stability: 2:12.64 at `--chicane-safety-mult 0.75`. Remaining 3/10 aborts are
  a chicane-envelope issue, not a chatter issue. See
  `docs/architecture-v3-session-2026-05-24.md` for the full session record.
- **Recommended invocation (reactive):**
  ```
  python lap.py cars_csv/bmw_1m \
      tracks_csv/ks_nurburgring/layout_sprint_a.csv \
      drivers/tomas.json \
      --model slip --controller reactive --single-lap \
      --inertia-zz 2400
  ```
- **Known remaining limitation:** 3/10 chicane-envelope aborts at s≈665 m. The
  chicane safety cap (mult=0.80) is already in place; the aborts are a combined-
  slip envelope problem that requires an ellipse-aware FF controller (the
  user-spec'd next direction). v2 point-mass completes all four Nurburgring
  layouts. See `docs/architecture-v3-shipping-state.md` for the full smoke-test
  record and the resurrection criteria for the MPC work.

## Future directions (backlog, not scheduled)

- **v1.4 drag calibration** — fit `k_drag` / `k_drag_reduction` per compound
  from telemetry pressure sweeps. `Compound` dataclass already carries both
  fields (hand-defaults in v1.3); v1.4 replaces the defaults with measured values.
  Spec §21.10.

- **v3.x MPC resurrection** — three prerequisites listed in
  `docs/architecture-v3-shipping-state.md` §"What would resurrect the MPC work":
  per-wheel Fz plant, Pacejka refit weighted toward high-curvature samples,
  alternative validation track without a Sprint-A-style chicane.
