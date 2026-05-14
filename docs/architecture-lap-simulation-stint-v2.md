# Architecture: per-wheel tyre state & multi-lap stint sim (v2)

## What the code does

v2 extends the LapTimeEstimator with **per-wheel tyre state (temperature, wear,
pressure) evolving across a multi-lap stint**, plus an **inverse-PSI solver**
that recommends cold setup pressures to hit a target wear at a target lap. Two
new user use cases (spec §21.1):

1. "How many laps until tyres reach 80%?" → run `lap.py ... --laps N` and read
   the per-lap wear block.
2. "What cold PSI gets 50% wear at lap 12?" → run `lap.py ... --solve-pressure-for-wear 0.50 --at-lap 12`.

Architecturally this is **option 2.5** (spec §21.1 / Decisions item 23). The
existing 3-pass `simulator.py` and its single-scalar grip envelope are retained
**unchanged**. Per-wheel state is computed **between laps** and reduced to a
scalar `(mu_x_scale, mu_y_scale)` multiplier that scales `tyre_grip_lateral`
and `tyre_grip_longitudinal` for the next lap's solver pass. Per-wheel forces,
yaw dynamics, Pacejka, and slip-based control-loop drivers stay in v3 (§19).

## Why this architecture

- **Minimal change to a tested solver.** `simulate(...)` was already validated
  against real AC telemetry (v1.1 + v1.2.1 acceptance criteria). Wrapping it in
  a per-lap loop preserves the entire backward-compat surface (§11.30, §11.31).
- **Inverse solver requires forward sims, not a custom ODE.** A user-facing
  "give me PSI for 50% wear" is structurally a search over cold-pressure
  space; we re-use `simulate_stint` as the inner evaluator and bisect.
- **NumPy-only optimization.** The four calibration knobs are fit via
  coordinate-descent in log10 space (no SciPy dependency). 4-knob problems
  with smooth losses converge in 3-4 outer sweeps; matches the brief's
  flexibility carve-out and keeps the dep set lean.
- **Single-grip envelope reduction is a documented simplification** (spec
  §21.3 step 5). `0.5 · (min(g_FL, g_FR) + min(g_RL, g_RR))` loses
  oversteer/understeer balance information — that lives in v3.

## Data flow

```
                                     +-----------------------------------+
                                     |  Setup (cold psi, ambient temp)   |
                                     |  -- setups/*.json / --pressure /  |
                                     |     tyres.ini PRESSURE_IDEAL       |
                                     +-----------------+-----------------+
                                                       |
                                                       v
                                     +-----------------------------------+
   driver JSON ─────────────────────►|     TyreState.from_setup(...)     |
   tyre_calibration block            |  (initial: temp=ambient, wear=100, |
   k_friction / h / C_thermal /      |   pressure_psi=cold)               |
   k_wear / measured                 +-----------------+-----------------+
                                                       |
                                                       v
                                     +-----------------------------------+
                                     |   for lap k in 1..n_laps:         |
   car (Car) ──────────────────────► |     g_x, g_y = combined_envelope( |
   track (Track CSV) ──────────────► |          state, car_tyre_model)   |
                                     |     scaled_car = GripScaledCar(   |
                                     |          car, g_x, g_y)           |
                                     |     lap_result = simulate(        |
                                     |          scaled_car, track,       |
                                     |          driver, two_lap=False)   |
                                     |     for each segment in lap:      |
                                     |        update_per_segment(state,  |
                                     |          seg, car, calib, model)  |
                                     |   END                              |
                                     +-----------------+-----------------+
                                                       |
                                                       v
                                     +-----------------------------------+
                                     |             StintResult            |
                                     |   .lap_times_s, .tyre_state_       |
                                     |    history, .per_lap_sim_results,  |
                                     |    .per_point_states               |
                                     +-----------------+-----------------+
                                                       |
                  +------------------+----------------+----------------+
                  |                  |                |                |
                  v                  v                v                v
            stdout (per-lap)   stint_summary.csv  sim_trace.csv   sim_telemetry.csv
            (§11.33)           (§7.11)            (existing)      (§7.12, 19 cols)
```

Per-segment state evolution (called once per simulator segment within a lap):

```
SegmentInfo(distance, segment_length, v_ms, radius, radius_sign, binding_label)
        │
        ├── slip-energy attribution (load transfer + driven-axle split)
        │     k_load = 0.30, k_long = 0.20  (hand-coded per spec §21.3)
        │     dE[w] = Fz[w] · (|a_long|·long_share[w] + |a_lat|) · dt
        │
        ├── thermal Euler step (per wheel)
        │     P_heat[w]   = k_friction · dE[w] / dt
        │     dT/dt[w]    = (P_heat[w] - h · (T[w] - T_amb)) / C_thermal
        │     T[w] += dT/dt[w] · dt
        │
        ├── wear update (per wheel)
        │     penalty = 1 / max(f_temp(T[w]), 0.3)
        │     dwear/dt[w] = k_wear · dE[w]/dt · penalty   (pct/s)
        │     wear[w] -= dwear/dt[w] · dt    (clamped [0, 100])
        │
        └── pressure update (per wheel, ideal-gas, isovolumetric)
              p[w] = p_cold[w] · (T[w] + 273.15) / T_cold_K
```

Scalar reduction (called once per lap, before the next lap's 3-pass):

```
g[w] = f_temp(T[w]) · f_wear(wear[w]) · f_pressure(p[w])
g_combined = 0.5 · (min(g_FL, g_FR) + min(g_RL, g_RR))
g_combined = max(g_combined, 0.30)   # numerical floor (see "Deviations")
(mu_x_scale, mu_y_scale) = (g_combined, g_combined)
```

Inverse-PSI solver (`solve_setup.solve_pressure_for_wear`):

```
1. seed scan:  for psi in {22, 27, 32, 37, 42, 47}: simulate_stint(...) → observed_wear
2. bracket:    find adjacent (lo, hi) straddling target
3. bisection:  per-wheel (or uniform), <=12 iters, refine bracket each step
4. converge:   |obs - target| <= 0.01 OR |psi_hi - psi_lo| <= 0.5
5. output:     4 PSI + verification stint history
```

## File inventory

### New files

| Path | Purpose | Spec section |
|------|---------|--------------|
| `src/lap_estimator/tyre_state.py` | Per-wheel state struct, per-segment update, grip envelope reduction, signed-curvature helper, GripScaledCar wrapper | §6.11, §21.3 |
| `src/lap_estimator/setup.py` | Setup dataclass; precedence `--pressure` > `--setup` > `tyres.ini` | §6.12, §7.10, §21.7 |
| `src/lap_estimator/solve_setup.py` | Inverse-PSI solver (greedy per-wheel coordinate-descent bisection) | §6.13, §21.5 |
| `setups/bmw_1m_default.json` | Default-pressure setup for the BMW 1M (PRESSURE_IDEAL from tyres.ini) | §7.10 |
| `docs/architecture-lap-simulation-stint-v2.md` | This document | — |

### Modified files

| Path | Change | Spec section |
|------|--------|--------------|
| `src/lap_estimator/simulator.py` | + `StintResult` dataclass, + `simulate_stint(...)`, back-compat fast path for `n_laps == 2 && !measured` | §6.4, §11.30, §11.31, §21.4 |
| `src/lap_estimator/sim_telemetry.py` | + 12 trailing per-wheel state columns when given a StintResult; + back-compat stint writer | §7.12, §14.6, §14.12 item 14 |
| `src/lap_estimator/report.py` | + `write_stint_summary_csv`, + `print_per_lap_block` | §6.5, §7.11, §11.33 |
| `src/lap_estimator/driver.py` | + `tyre_calibration` dataclass field, + `get_tyre_calibration()` | §6.1, §7.2 |
| `src/lap_estimator/driver_fit.py` | + `fit_tyre_calibration(...)` (NumPy-only coordinate-descent), folded into `fit_driver(...)` with `measured=False` fallback | §13.14 |
| `src/lap_estimator/telemetry.py` | + `PER_WHEEL_STATE_CHANNELS`, + `has_per_wheel_state(...)` helper | §21.6 |
| `lap.py` | + `--laps`, `--setup`, `--pressure`, `--ambient-temp-c`, `--solve-pressure-for-wear`, `--at-lap`, `--target-wheel`, `--uniform-pressure`. `--single-lap` aliased to `--laps 1` | §6.6, §21.4 / §21.5 |
| `fit_driver.py` | Threads `track` into `fit_driver` and writes `tyre_calibration` block to output JSON | §13.14, §7.2 |

### Untouched (per spec §21.8)

`track.py`, `prep/*`, `analysis/*`, `validate.py`, `profile_dynamics.py`.

### v2 compound-aware extension (2026-05-14, §6.14, §21.11)

| Path | Change | Spec section |
|------|--------|--------------|
| `src/lap_estimator/car.py` | + `Compound` dataclass, + `_parse_compounds(...)` multi-compound parsing, + `Car.from_dir`, + `Car.find_compound`, + `Car.default_compound`, `Car.default_compound_index`. Legacy `tyre_dy0_f` etc. pinned to compound 0 for v1.2.1 back-compat. | §6.14, §21.11 |
| `src/lap_estimator/tyre_state.py` | `build_car_tyre_model(car, compound=None)` keyed off the active compound's LUT paths. `GripScaledCar(...)` gains `compound=` kwarg that swaps the baseline `dy0/dx0/speed_sens` on the wrapper. | §21.11 |
| `src/lap_estimator/setup.py` | `Setup.pressures_psi` is now optional (per §7.10). `Setup.default_for_car(car, compound)` reads `PRESSURE_STATIC` per axle (not `PRESSURE_IDEAL`). New `resolve_compound(...)` helper implementing the four-level precedence. `resolve_setup(...)` takes `compound` kwarg. | §7.10, §21.7, §21.11 |
| `src/lap_estimator/simulator.py` | `simulate_stint(..., compound=None)` and `StintResult.compound`. Back-compat fast path additionally gated on `compound.index == car.default_compound_index`. | §21.4, §21.11 |
| `src/lap_estimator/solve_setup.py` | `solve_pressure_for_wear(..., compound=None)` threads through every internal `_run_stint`. | §21.11 |
| `src/lap_estimator/driver_fit.py` | + `_resolve_compound_from_frames(...)` runs before calibration; threads compound into `fit_tyre_calibration(...)` and the per-sample `lat_g_max` envelope. `FitResult` gains `.compound_name`/`.compound_source`. | §13.14, §21.11 |
| `src/lap_estimator/telemetry.py` | + `OPTIONAL_STRING_COLUMNS = ("tyreCompound",)` passthrough as numpy object array. | §21.6, §21.11 |
| `lap.py` | + `--compound <name>` argparse flag. Two-phase compound peek (setup-load → resolve via `resolve_compound`). Stint mode engaged when user supplies `--setup`/`--compound`/`--pressure`/`--ambient-temp-c` (so the compound override actually affects lap-1 grip). `_run_solver` and `_run_one_track` accept/thread `compound`. | §11.35, §21.11 |
| `fit_driver.py` | Writes `tyre_calibration.source.compound` from `fit.compound_name`. Prints `Tyre compound: <name> (source: ...)` on stdout. | §13.14, §11.36 |

## Key decisions and trade-offs

- **NumPy-only Nelder-Mead replacement.** Brief flexibility carve-out: instead
  of adding SciPy, we use coordinate descent over log10(knobs) with a 5-point
  line search per axis and 8 outer sweeps max. Trade-off: convergence may be
  slower than Nelder-Mead on highly coupled 4D landscapes, but for our smooth
  weighted-RMSE loss the difference is ~0.5s of wall time; acceptable.
- **Numerical floor `g_combined >= 0.30`** (in `combined_grip_envelope`). NOT
  in the spec. Necessary because uncalibrated defaults drive temperatures into
  the LUT's tail very quickly and a sub-0.30 grip envelope produces lap times
  of several minutes that are pathological for the QA workflow. Matches the
  `f_pressure` lower clamp; documented under "Deviations" below.
- **Back-compat fast path for `n_laps == 2 && !measured`.** When the user
  hasn't fit calibration knobs, the v1.1 two-lap-default behaviour is
  preserved bit-for-bit by delegating to `simulate(..., two_lap=True)`. The 12
  state columns are emitted but constant (spec §11.31, §14.12 item 14).
- **State emission via per-point arrays.** `simulate_stint` captures
  per-segment state via an `on_segment` callback into pre-allocated NumPy
  arrays. This avoids re-computing state during telemetry emission and keeps
  the writer a straight nearest-neighbour interp from the sim grid.
- **Signed-radius inference from x,y.** Track CSVs only store unsigned radius.
  We compute the sign at the start of each stint via 3-point cross-product on
  the x,y centerline (`derive_radius_sign`). Cached per stint (not per lap).
- **Greedy per-wheel coordinate-descent bisection.** Spec §21.5 step 4. The
  alternative (joint 4D bisection) is exponentially harder and assumes
  non-monotonic coupling; the greedy form converges in practice on the
  monotonic-wear-vs-PSI region we care about.

## Compound-aware tyres.ini parsing (v2 extension, 2026-05-14)

Spec §21.11 / Decisions item 24. Motivated by the live bug where Tomas's
Semislicks lap at 26 psi was scored against Street's `PRESSURE_IDEAL=42`,
collapsing `f_pressure` to its 0.30 floor and producing 2:56+ lap times for
a real 1:48 lap.

### What

`Car` now holds `compounds: list[Compound]` and `default_compound_index`. Each
`Compound` (frozen dataclass) carries per-axle `pressure_ideal`,
`pressure_static`, `pressure_d_gain`, `dy0/dy1/dx0/dx1/dy_ref/dx_ref`,
`speed_sens`, plus the LUT filenames for wear and thermal performance curves.
`Car.from_dir(...)` is added as a classmethod alias and `Car.find_compound(q)`
matches case-insensitively against `name`/`short_name`, stripping any trailing
parenthetical (so AC's telemetry `"Semislicks (SM)"` resolves to `"Semislicks"`).

Section discovery in `_parse_compounds`: scan section names by family
(`THERMAL_FRONT` checked before `FRONT` to avoid ambiguity with
`THERMAL_FRONT_1`), group by suffix index (`_n`, un-suffixed = 0), and require
all four families to be present per index — otherwise raise a clear
parse-time error. `[COMPOUND_DEFAULT].INDEX` is honoured if present, else 0.

### Compound resolution precedence (§21.11 / §7.10)

```
1. --compound <name> on lap.py                 -> source: cli
2. setup-JSON `compound` field                 -> source: setup
3. telemetry's most-common `tyreCompound`      -> source: telemetry  (fitter only)
4. car.compounds[car.default_compound_index]   -> source: car-default
```

Levels 1 and 2 raise on unknown name (listing available compounds). Level 3
warns and falls through to level 4 on miss. The resolved compound is logged
on stdout as `Compound: <name> (idx <i>) | source: <tag>`.

### Per-compound function plumbing

- `tyre_state.build_car_tyre_model(car, compound)` now reads wear-curve and
  thermal-performance LUTs off the active `Compound` rather than the
  hard-coded un-suffixed sections. The same `Car` instance backs sims for any
  compound; only the per-stint `CarTyreModel` differs.
- `tyre_state.GripScaledCar(car, mu_x, mu_y, compound=...)` swaps in the
  active compound's `dy0/dx0/speed_sens` as the baseline for grip queries
  (the previous-lap reduction multiplies on top). When `compound` is None the
  wrapper falls back to `car.tyre_dy0_f` etc. for v1.2.1 back-compat.
- `simulator.simulate_stint(...)` gains a `compound` kwarg; threads it
  through every lap and through `update_per_segment` via the `CarTyreModel`.
- `solve_setup.solve_pressure_for_wear(..., compound=...)` threads to every
  internal `simulate_stint` call (seed scan, bisection, verification).
- `driver_fit.fit_driver(...)` resolves the compound from the merged frames'
  most-common `tyreCompound` value (via `_resolve_compound_from_frames`)
  before the calibration step. The resolved compound feeds both the
  calibration fit AND the `lat_g_max` envelope used in the skill-percentile
  step. `FitResult` now exposes `.compound_name` / `.compound_source` and
  `fit_driver.py` writes `tyre_calibration.source.compound` into the driver
  JSON.

### Cold-pressure default change

`Setup.default_for_car(car, compound)` now reads the **active compound's
`PRESSURE_STATIC`** per axle (not `PRESSURE_IDEAL`). `PRESSURE_STATIC` is the
cold-pressure target a driver dials in pre-session; `PRESSURE_IDEAL` is the
hot-grip-peak target after warm-up. Earlier v2 drafts had this inverted —
corrected here. For BMW M1 Street the default becomes 35/35 psi; for
Semislicks 28/28 psi.

### Stint-mode engagement broadened

`lap.py`'s `use_stint` predicate now ORs in `user_supplied_state_overrides`
(any of `--setup`, `--compound`, `--pressure`, `--ambient-temp-c`). Without
this, `--laps 1 --compound Semislicks --pressure 26` would silently fall
through to the v1.2.1 MC/single-lap path that ignores tyre state, defeating
the compound override.

### Legacy attributes pinned to compound 0

`Car.tyre_dy0_f` and friends are pinned to compound 0 (un-suffixed
sections) regardless of `[COMPOUND_DEFAULT].INDEX` for v1.2.1 back-compat:
the legacy single-lap MC path uses these attributes and must continue to
match the pre-v2 baseline (§11.30, §11.31). Active-compound baselines for
stint mode flow through `GripScaledCar(compound=...)` rather than mutating
the `Car` instance.

## How v2 integrates with neighboring features

- **fit_driver.py** now optionally calibrates the four tyre-state knobs
  before the skill-percentile pass. If per-wheel state channels (`tyreTempFL`,
  `tyreWearFL`, `wheelsPressureFL`, etc.) are present in every input frame,
  the fitter runs `fit_tyre_calibration(...)` and writes the result under
  `tyre_calibration` in the driver JSON. The calibrated grip envelope is
  then folded into the per-sample `lat_g_max` so `skill_pct` does not absorb
  tyre-state confounds (spec §13.14, §21.6 / §13 risks).
- **Telemetry consumers** (real or sim-emitted) gain 12 trailing columns in
  stint mode. Real AC schema uses `tyreTempFL` / `tyreWearFL` /
  `wheelsPressureFL`; sim-emission uses the spec §7.12 short names
  `tempFL` / `wearFL` / `pressureFL`. The two conventions are deliberately
  different to keep AC-schema fidelity vs. sim-output compactness separate.
- **Validation flow (`--validate-against`)** is untouched — it compares
  lap-2 sim speed against real lap speed. Stint mode does not engage the
  validator; users still pass `--laps 2` and `--validate-against` together
  for the validation use case.
