# LapTimeEstimator - AI Context Document

This document provides the knowledge an AI assistant needs to work with this project effectively.

## Project Purpose

Point-mass lap time estimator that uses Assetto Corsa (AC) car physics data and track geometry to simulate lap times. The simulation uses a 3-pass approach: cornering speed limits, forward acceleration pass, backward braking pass.

## Project Structure

```
LapTimeEstimator/
├── decode_acd.py        # Decrypt AC .acd car archives
├── decode_track.py      # Parse AC fast_lane.ai track files into segments
├── car.py               # Car physics model (parses AC ini/lut files)
├── track.py             # Track model + 4 built-in tracks
├── simulator.py         # 3-pass point-mass lap time simulator
├── main.py              # CLI entry point
├── cars/                # Car data (one subfolder per car)
│   └── <car_name>/
│       └── data/        # Decrypted .ini and .lut files
├── tracks/              # Track data (one subfolder per track)
│   └── <track_name>/
│       ├── layout_*.csv           # Track centerline data
│       └── *_ideal_line.csv       # AI racing line data
└── docs/
```

## Data Pipeline

### Step 1: Decrypt Car Data

AC stores car physics in encrypted `.acd` archives. Decrypt with:

```
python decode_acd.py <data.acd> <car_folder_name> [output_dir]
```

- The car folder name (e.g. `bmw_1m`) is the encryption key seed.
- The key is derived via an 8-part numeric formula producing a string like `"67-169-131-162-142-140-73-110"`.
- Each content byte is stored as a 4-byte little-endian int32. Decryption: `plain[i] = (int32[i] & 0xFF - key_char[i % key_len]) & 0xFF`.

### Step 2: Decode Track Data

AC track AI lines (`fast_lane.ai`) contain XYZ racing line points. Decode with:

```
python decode_track.py <fast_lane.ai> [output.json]
```

Outputs a JSON with segments (`length` + `radius`). This can also be done from pre-processed CSV files in `tracks/`.

### Step 3: Run Simulation

```
python main.py <car_data_dir> <track_name_or_json> [--ds 2.0] [--all-tracks]
```

## Key AC Data Files (After Decryption)

### Car Physics Files

| File | Purpose | Key Fields |
|---|---|---|
| `car.ini` | Basic car info | `TOTALMASS`, `STEER_LOCK`, `FUEL`, `MAX_FUEL` |
| `engine.ini` | Engine model | `LIMITER`, `INERTIA`, turbo sections (`MAX_BOOST`, `WASTEGATE`) |
| `power.lut` | Torque curve | Format: `RPM\|torque_Nm` per line |
| `drivetrain.ini` | Gearbox + diff | `TYPE` (RWD/FWD/AWD), `GEAR_1..N`, `FINAL`, `POWER`/`COAST` lock |
| `tyres.ini` | Tyre model | `DY0`/`DX0` (grip coefficients), `RADIUS`, `SPEED_SENSITIVITY`, `WIDTH` |
| `aero.ini` | Aerodynamics | Wing sections with `CHORD`, `SPAN`, `CD_GAIN`, `CL_GAIN`, LUT references |
| `brakes.ini` | Brake system | `MAX_TORQUE`, `FRONT_SHARE` |
| `suspensions.ini` | Suspension geometry | `WHEELBASE`, `CG_LOCATION`, `TRACK`, spring/damper rates |

### LUT File Format

Lookup tables use pipe-delimited format:
```
input_value|output_value
```

Examples:
- `power.lut`: `RPM|torque_Nm` — engine torque curve
- `wing_body_AOA_CD.lut`: `angle_degrees|Cd` — drag coefficient vs angle of attack
- `tcurve_street.lut`: `temperature|grip_multiplier` — thermal performance curve
- `street_front.lut`: `wear_km|grip_multiplier` — tyre wear curve

### Track CSV Format

Pre-processed track data in CSV with these columns:

| Column | Type | Description |
|---|---|---|
| `index` | int | Point index (0-based) |
| `distance_m` | float | Cumulative distance from start in meters |
| `segment_length_m` | float | Distance to next point in meters |
| `x` | float | World X coordinate (meters) |
| `y` | float | World Y coordinate (meters, elevation) |
| `z` | float | World Z coordinate (meters) |
| `elevation_m` | float | Elevation above reference (same as y) |
| `gradient_pct` | float | Road gradient in percent (positive = uphill) |
| `radius_m` | float | Corner radius in meters (large values ~2000 = straight) |
| `speed_ms` | float | AI reference speed in m/s |
| `speed_kmh` | float | AI reference speed in km/h |
| `width_left_m` | float | Track width to the left of the line in meters |
| `width_right_m` | float | Track width to the right of the line in meters |
| `width_total_m` | float | Total track width in meters |

Two CSV variants exist per layout:
- `layout_*.csv` — track centerline
- `*_ideal_line.csv` — AI optimal racing line (typically faster speeds, different positioning)

### Track JSON Format (decode_track.py output)

```json
{
  "name": "Track Name",
  "total_length": 5793.0,
  "segments": [
    {"length": 800.0, "radius": 0},
    {"length": 120.0, "radius": 85.0},
    {"length": 50.0, "radius": -60.0}
  ]
}
```

- `radius = 0` means straight
- `radius > 0` means right turn
- `radius < 0` means left turn
- `|radius|` is the corner radius in meters

## Physics Model Summary

### Engine
- Torque interpolated from `power.lut` at current RPM
- Turbo: `effective_torque = base_torque * (1 + wastegate_boost)`
- RPM limited by `LIMITER` value

### Drivetrain
- `wheel_torque = engine_torque * gear_ratio * final_ratio`
- `vehicle_speed = wheel_rps * tyre_radius`
- Optimal gear selected for maximum wheel torque at current speed

### Tyres
- Lateral grip: `mu_y = DY0 / (1 + SPEED_SENSITIVITY * speed)`
- Longitudinal grip: `mu_x = DX0 / (1 + SPEED_SENSITIVITY * speed)`
- Speed sensitivity reduces grip at higher speeds

### Aerodynamics
- Drag: `F_drag = 0.5 * rho * Cd * A * v^2`
- Downforce: `F_down = 0.5 * rho * |Cl| * A * v^2`
- Frontal area approximated from body wing `CHORD * SPAN`
- Cd/Cl from LUT files at 0 degrees angle of attack

### Cornering
- Max corner speed solved iteratively: `v = sqrt(mu * (m*g + downforce(v)) * R / m)`

### Simulation (3-pass)
1. **Corner pass**: max speed at each point from radius + grip
2. **Forward pass**: limit speed by acceleration capability from previous point
3. **Backward pass**: limit speed by braking capability into next point
4. **Time integration**: `dt = ds / v_avg` for each segment

## Common Tasks for AI

- **Add a new car**: decrypt its `.acd`, place data in `cars/<name>/data/`
- **Add a new track**: decode `fast_lane.ai` or place CSV in `tracks/<name>/`
- **Compare cars**: run same track with different car dirs
- **Tune accuracy**: adjust `--ds` (smaller = more accurate), improve tyre model, add elevation effects
- **Extend physics**: add traction control, ABS, weight transfer, tyre thermal model

## Future directions

Longer-horizon research backlog is captured in the lap-simulation spec, §19. Two items, neither scheduled:

- **Slip-based simulator (v3)** — Pacejka Magic Formula tyre model, friction ellipse, time-domain ODE integration over `(x, y, ψ, v_x, v_y, ω_yaw, ω_wheel×4)`, driver-as-control-loop (preview + PID + α_target). Replaces the point-mass for drift / oversteer / understeer fidelity; lives alongside it. Multi-week build. AC shared-memory channels (`localVelocity_*`, `localAngularVel_*`, `wheelSlipFL/FR/RL/RR`, `wheelLoadFL/...`, `wheelAngularSpeedFL/...`, `tyreContactHeading*`) give us ground-truth `(α, κ, Fz)` per wheel for fitting Pacejka coefficients against AC.
- **Tyre-state model (v2)** — per-wheel `(T_core, wear_km, P)` integrated over the lap; grip = product of thermal LUT × wear LUT × pressure curve. All three curves already exist in AC's `tyres.ini` (`PERFORMANCE_CURVE`, `WEAR_CURVE`, `PRESSURE_IDEAL` / `PRESSURE_D_GAIN`) and are currently ignored by `car.py`. Fits inside the existing point-mass sim — one week of evening work, can ship independently of v3.

See `dev-planning/lap-simulation-csv-driver/spec.md` §19 for AC signal coverage, conversion strategy from `tyres.ini` to Pacejka, effort estimates, and the v1 → v2 → v3 sequencing recommendation.
