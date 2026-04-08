# LapTimeEstimator

Point-mass lap time simulator using Assetto Corsa car physics data.

## Requirements

- Python 3.8+
- NumPy (`pip install numpy`)
- Optional: Matplotlib for track plotting (`pip install matplotlib`)

## Input Data

This project uses car and track data from **Assetto Corsa**. The raw game data is not included in the repository and must be provided by the user.

- **`cars_in/`** — Place Assetto Corsa car folders here (e.g. `cars_in/bmw_1m/`). Each folder should contain the car's `data.acd` file along with its assets. These are found in `<Assetto Corsa install>/content/cars/`.
- **`tracks_in/`** — Place Assetto Corsa track folders here (e.g. `tracks_in/ks_nurburgring/`). Each folder should contain the track's `ai/fast_lane.ai` file. These are found in `<Assetto Corsa install>/content/tracks/`.

## Quick Start

```bash
# 1. Copy car/track data from Assetto Corsa into the input folders
#    cp -r "<AC install>/content/cars/bmw_1m" cars_in/
#    cp -r "<AC install>/content/tracks/ks_nurburgring" tracks_in/

# 2. Decrypt a car's data.acd file
python decode_acd.py cars_in/bmw_1m/data.acd bmw_1m cars_csv/bmw_1m/data/

# 3. Run a lap simulation with a built-in track
python main.py cars_csv/bmw_1m monza

# 4. Run on all built-in tracks
python main.py cars_csv/bmw_1m monza --all-tracks
```

## Scripts

### decode_acd.py - Decrypt Car Data

Extracts physics files from Assetto Corsa's encrypted `.acd` archives.

```
python decode_acd.py <data.acd> <car_folder_name> [output_dir]
```

| Argument | Description |
|---|---|
| `data.acd` | Path to the encrypted archive |
| `car_folder_name` | Exact name of the car's folder (e.g. `bmw_1m`) - used as decryption key |
| `output_dir` | Where to write files (default: `data/` next to the .acd) |

**Output:** Individual `.ini` and `.lut` files (see [Data Formats](#car-data-ini-files) below).

### decode_track.py - Parse Track AI Lines

Converts AC's binary `fast_lane.ai` racing line files into JSON track segments.

```
python decode_track.py <fast_lane.ai> [output.json] [--plot]
```

| Argument | Description |
|---|---|
| `fast_lane.ai` | Path to the AI line file (found in `<track>/ai/fast_lane.ai`) |
| `output.json` | Output path (default: same name with `.json` extension) |
| `--plot` | Save a top-down track map image (requires matplotlib) |

**Output:** JSON with segments array (see [Track JSON Format](#track-json-format) below).

### main.py - Run Simulation

```
python main.py <car_data> <track> [--ds 2.0] [--all-tracks]
```

| Argument | Description |
|---|---|
| `car_data` | Path to car's data directory (containing .ini/.lut files) |
| `track` | Built-in name (`monza`, `spa`, `nurburgring`, `brands_hatch`) or path to track JSON |
| `--ds` | Distance step in meters (default: 2.0, smaller = more accurate) |
| `--all-tracks` | Run simulation on all 4 built-in tracks |

**Output:** Lap time, speed trace per segment, performance stats (0-100, 0-200, top speed).

---

## Data Formats

### Car Data (INI files)

After decrypting a `.acd`, you get standard INI files with AC-specific physics parameters. Key files:

| File | What it contains |
|---|---|
| `car.ini` | Mass, fuel capacity, steering lock |
| `engine.ini` | Rev limiter, turbo boost, engine inertia |
| `power.lut` | Engine torque curve (RPM vs Nm) |
| `drivetrain.ini` | Gear ratios, final drive, differential settings |
| `tyres.ini` | Grip coefficients, tyre dimensions, speed sensitivity, compounds |
| `aero.ini` | Aero surfaces with drag/lift lookup tables |
| `brakes.ini` | Max brake torque, front/rear bias |
| `suspensions.ini` | Wheelbase, CG position, spring/damper rates |

### LUT Files (Lookup Tables)

Pipe-delimited, one entry per line:

```
input_value|output_value
```

**power.lut** - Engine torque curve:
```
0|100          # RPM | Torque (Nm)
500|119
1000|146
...
7000|159
8000|0         # cutoff
```

**wing_body_AOA_CD.lut** - Drag coefficient vs angle of attack:
```
-10|1          # angle (degrees) | Cd
0|0.34
10|0.35
30|1
```

### Track CSV Format

Pre-processed track data with one row per point along the racing line:

```csv
index,distance_m,segment_length_m,x,y,z,elevation_m,gradient_pct,radius_m,speed_ms,speed_kmh,width_left_m,width_right_m,width_total_m
0,0.00,1.544,-4.878,63.954,-763.340,63.954,0.00,2000.0,26.14,94.1,2.63,10.81,13.44
1,1.54,1.543,-4.870,63.946,-761.797,63.946,-0.52,2000.0,26.45,95.2,2.63,10.80,13.44
```

| Column | Unit | Description |
|---|---|---|
| `index` | - | Sequential point index |
| `distance_m` | m | Cumulative distance from start/finish |
| `segment_length_m` | m | Distance to next point |
| `x`, `y`, `z` | m | 3D world coordinates (y = elevation) |
| `elevation_m` | m | Height above reference datum |
| `gradient_pct` | % | Road slope (positive = uphill) |
| `radius_m` | m | Corner radius (large values like 2000 = effectively straight) |
| `speed_ms` | m/s | AI reference speed at this point |
| `speed_kmh` | km/h | Same speed in km/h |
| `width_left_m` | m | Track width left of the line |
| `width_right_m` | m | Track width right of the line |
| `width_total_m` | m | Total track width |

Two variants per layout:
- **`layout_*.csv`** - follows the track centerline
- **`*_ideal_line.csv`** - follows the AI's optimal racing line (different positioning, typically higher speeds)

### Track JSON Format

Output of `decode_track.py`, input for `main.py`:

```json
{
  "name": "Monza",
  "total_length": 5793.0,
  "segments": [
    {"length": 800.0, "radius": 0},
    {"length": 120.0, "radius": 85.0},
    {"length": 50.0, "radius": -60.0}
  ]
}
```

| Field | Description |
|---|---|
| `length` | Segment length in meters |
| `radius` | Corner radius: `0` = straight, `> 0` = right turn, `< 0` = left turn |

---

## Directory Layout

```
LapTimeEstimator/
├── decode_acd.py         # Car data decryptor
├── decode_track.py       # Track AI line parser
├── car.py                # Car physics model
├── track.py              # Track model + built-in tracks
├── simulator.py          # Lap time simulation engine
├── main.py               # CLI
├── cars_in/              # Raw Assetto Corsa car data (not tracked, user-provided)
├── tracks_in/            # Raw Assetto Corsa track data (not tracked, user-provided)
├── cars_csv/
│   └── bmw_1m/
│       └── data/         # Decrypted .ini and .lut files
├── tracks_csv/
│   └── ks_nurburgring/   # Pre-processed track CSVs + images
│       ├── layout_gp_a.csv
│       ├── layout_gp_a_ideal_line.csv
│       └── ...
└── docs/
    └── AI_CONTEXT.md     # Detailed reference for AI assistants
```

## Example Output

```
Loaded: Car(mass=1592kg, RWD, 6spd, turbo=46%, Cd=0.340, grip_y=1.280/1.284)

============================================================
  LAP TIME ESTIMATION
============================================================
  Car:    Car(mass=1592kg, RWD, 6spd, turbo=46%, Cd=0.340)
  Track:  Track('Monza', 5270m, 11 corners)
------------------------------------------------------------
  Lap Time:     2:17.401
  Max Speed:    212.8 km/h
  Min Speed:    54.0 km/h
  Avg Speed:    138.0 km/h
------------------------------------------------------------
  Performance Summary:
  Top speed (top gear):  261.0 km/h
  0-100 km/h:            5.4 s
  0-200 km/h:            19.9 s
============================================================
```
