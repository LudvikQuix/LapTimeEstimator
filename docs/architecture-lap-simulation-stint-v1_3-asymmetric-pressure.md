# Architecture: v1.3 asymmetric pressure model

Spec: `dev-planning/lap-simulation-csv-driver/spec.md` §21.3, §21.11,
§11.37–§11.39, Decisions item 25.

## What the code does

v1.3 splits the pre-v1.3 symmetric `f_pressure(p)` quadratic into two
compound-attached functions:

- `f_pressure_grip(p)` — cornering-grip multiplier. Penalty active **only
  above** `PRESSURE_IDEAL` (over-pressure → smaller contact patch → less
  grip). Below IDEAL it returns `1.0` (no penalty; the small real-world bonus
  from a larger contact patch is ignored in v1.3 — see spec §3 non-goals).
- `f_pressure_drag(p)` — straight-line drag multiplier. Penalty active **only
  below** `PRESSURE_IDEAL` (under-pressure → sidewall flex + rolling
  resistance → more drag). Above IDEAL a small linear benefit (less rolling
  resistance from a stiffer, smaller patch). Final clamp `[0.85, 1.50]`.

`combined_grip_envelope(state, compound)` now returns a **3-tuple**:

```python
(mu_x_scale, mu_y_scale, drag_scale) = combined_grip_envelope(state, model)
```

`mu_x_scale` / `mu_y_scale` continue to scale `tyre_dy0_*` / `tyre_dx0_*` as
in v2 (unchanged). `drag_scale` multiplies the **total drag force** (aero +
rolling resistance) applied by the 3-pass solver during the forward and
backward velocity passes.

`drag_scale == 1.0` at IDEAL pressure preserves `§11.30` single-lap regression
byte-for-byte (the wrapper short-circuits when `drag_scale == 1.0`).

## Why this architecture

- **Physical correctness.** The pre-v1.3 symmetric quadratic
  `1 − PRESSURE_D_GAIN · (p − IDEAL)²` double-counted the low-PSI loss in the
  grip term where it belonged in the drag term. The fix is the split above:
  grip-only penalty above IDEAL, drag-only penalty below IDEAL.
- **Minimal solver change.** Wrapping the inner `car` with a thin
  `_DragScaledCar` proxy that scales `drag_force(v)` and
  `rolling_resistance(v)` is the smallest possible mutation: no edits to the
  3-pass solver loop, no per-segment force recomputation. The wrapper is a
  no-op when `drag_scale == 1.0` — preserves v2 byte-compat at IDEAL.
- **Backward compatibility.** The asymmetric grip side preserves the
  over-pressure regression (44 psi still tanks lap time via the existing
  quadratic). `driver_fit.py` continues to use `f_pressure` (now an alias for
  `f_pressure_grip`) — its `lat_g_max` envelope still only cares about
  cornering grip; drag is a straight-line quantity and is not part of
  `lat_g_max`.
- **Compound-attached constants, compound-agnostic defaults (v1.3).**
  `k_drag = 0.5` and `k_drag_reduction = 0.10` live on the `Compound`
  dataclass (so v1.4 can fit per-compound values from telemetry) but in v1.3
  both Street and Semislicks share the hand-defaults. Spec §21.10 documents
  the v1.4 telemetry-fit candidate.

## Data flow

```
                  +---------------------------------------------+
                  |  TyreState (per-wheel pressure_psi[w] etc.) |
                  +-----------------------+---------------------+
                                          |
                                          v
+--------------------------+   +----------+-----------+   +--------------------+
|  Compound                |   |  combined_grip_      |   |  CarTyreModel      |
|  - PRESSURE_IDEAL        +-->|  envelope(state,     +<--+  - k_drag          |
|  - k_drag = 0.5          |   |    model)            |   |  - k_drag_red      |
|  - k_drag_reduction= 0.1 |   |                      |   |  - p_ideal         |
+--------------------------+   |  g[w] = f_temp · f_  |   +--------------------+
                               |        wear · f_p_   |
                               |        grip(p[w])    |
                               |  d[w] = f_pressure_  |
                               |        drag(p[w])    |
                               +----------+-----------+
                                          |
                                          v
                  (mu_x_scale, mu_y_scale, drag_scale)
                                          |
                                          v
+--------------------------+   +----------+-----------+
|  car (Car)               +-->|  simulator.simulate( |
|                          |   |    car, ...,         |
+--------------------------+   |    drag_scale=...)   |
                               |                      |
                               |  if drag_scale != 1: |
                               |    car = _DragScaled |
                               |          Car(car,    |
                               |          drag_scale) |
                               |                      |
                               |  scaled = driver.    |
                               |         wrap(car)    |
                               |                      |
                               |  _three_pass(scaled, |
                               |    distances,        |
                               |    radii, ...)       |
                               +----------+-----------+
                                          |
                                          v
                                     SimResult
```

For multi-lap `simulate_stint`, the per-lap evolution is:

```
lap k:
  (mu_x, mu_y, drag_scale) = combined_grip_envelope(state, tyre_model)
  inner = _DragScaledCar(car, drag_scale) if drag_scale != 1 else car
  scaled = GripScaledCar(inner, mu_x, mu_y, compound=compound)
  lap_result = simulate(scaled, track, driver, drag_scale=1.0)   # already wrapped
  update_segments_in_place(state, ..., lap_result, ...)
```

The `_DragScaledCar` wrapper goes on the **innermost** car so every downstream
wrapper (`DriverScaledCar`, `GripScaledCar`) reads the scaled drag via
`self._car.drag_force(...)` and `self._car.rolling_resistance(...)`. Wrapping
the outer layer would not work — `GripScaledCar.max_braking_decel` accesses
`self._car.drag_force(...)` directly (not through `self`), so the wrapper
must sit beneath it.

## File inventory

### Modified

- `src/lap_estimator/car.py`
  - `Compound` dataclass: added two fields with hand-defaults
    `k_drag: float = 0.5` and `k_drag_reduction: float = 0.10`. Not parsed
    from `tyres.ini` (AC has no source). Compound-agnostic in v1.3.

- `src/lap_estimator/tyre_state.py`
  - Module docstring updated to describe the 3-tuple return.
  - Added `DRAG_CLAMP_LO = 0.85`, `DRAG_CLAMP_HI = 1.50`.
  - `CarTyreModel`: added `k_drag: float`, `k_drag_reduction: float`.
  - `build_car_tyre_model`: sources `k_drag` / `k_drag_reduction` from the
    active `Compound`.
  - `f_pressure(...)` removed and replaced by:
    - `f_pressure_grip(model, p, *, front)` — one-sided quadratic above
      IDEAL, clamp `[0.30, 1.0]`; returns `1.0` below IDEAL.
    - `f_pressure_drag(model, p, *, front)` — linear penalty
      `1 + k_drag · (IDEAL − p) / IDEAL` below IDEAL; small linear benefit
      `1 − k_drag_reduction · (p − IDEAL) / IDEAL` above IDEAL; clamp
      `[0.85, 1.50]`.
    - `f_pressure(...)` retained as a **back-compat alias** for
      `f_pressure_grip(...)` so `driver_fit.py`'s `lat_g_max` calculation
      keeps working without churn. Drag does not enter `lat_g_max` (spec
      §13.14, v1.3 note).
  - `combined_grip_envelope`: returns `(mu_x_scale, mu_y_scale, drag_scale)`
    where `drag_scale = mean(d_FL, d_FR, d_RL, d_RR)`.

- `src/lap_estimator/simulator.py`
  - New `_DragScaledCar` wrapper class. Multiplies `drag_force(v)` and
    `rolling_resistance(v)` by `drag_scale`; `__getattr__` delegates all
    other attributes to the inner car.
  - `simulate(...)` gained kwarg-only `drag_scale: float = 1.0`. When
    `drag_scale != 1.0` the inner car is wrapped with `_DragScaledCar`
    BEFORE the driver wrap, so `DriverScaledCar._car.drag_force(...)` reads
    through the scaler.
  - `simulate_stint(...)` lap loop unpacks the 3-tuple from
    `combined_grip_envelope` and applies `_DragScaledCar` at the innermost
    layer (before `GripScaledCar`), then passes `drag_scale=1.0` into
    `simulate(...)` to avoid double-wrapping.

### Untouched

- `src/lap_estimator/driver_fit.py` — uses `f_pressure` (now an alias) for
  its cornering-grip envelope. Drag is a straight-line quantity and is
  intentionally not folded into `lat_g_max` (spec §13.14, v1.3 note).
- CLI (`lap.py`), driver JSON schema, setup JSON schema, `setups/*` — no
  changes (per Buddy brief).
- `tyres.ini` parsing — `k_drag` / `k_drag_reduction` are NOT parsed.

## Solver-wiring decision: natural seam choice

The Buddy brief asked which seam was picked for `drag_scale` plumbing. The
3-pass solver consumes drag exclusively via `max_accel(v)` (`traction − drag
− rr`) and `max_braking_decel(v)` (`(grip_force + drag) / mass`). All three
existing car-like classes (`Car`, `DriverScaledCar`, `GripScaledCar`)
ultimately delegate the drag computation to `inner.drag_force(v)` and
`inner.rolling_resistance(v)`.

**Chosen seam:** wrap the innermost `car` with `_DragScaledCar`, which scales
both `drag_force(v)` and `rolling_resistance(v)` at the source. This:

- Lands the multiplier in **one place** (the wrapper) — spec §21.3 "Solver
  wiring" requirement.
- Covers aero drag + rolling resistance jointly — both are pressure-sensitive
  in reality and the spec's `f_pressure_drag` is described as the total
  straight-line drag multiplier.
- Requires **no edits** to `_three_pass`, `simulate`'s inner loop, or any
  existing `max_accel` / `max_braking_decel` implementation.
- Is a no-op at `drag_scale == 1.0` (the wrapper isn't applied), preserving
  §11.30 byte-equivalence.

The alternative — refactoring `Car.drag_force` to accept a scale parameter
and plumbing it through three wrapper classes — would touch ~9 callsites
across `car.py`, `driver.py`, and `tyre_state.py` for the same effect.

## Verification: sensitivity sweep

`python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv
 drivers/tomas_full.json --laps 3 --compound Semislicks
 --pressure FL=$p,FR=$p,RL=$p,RR=$p --ambient-temp-c 26`

| Cold psi | Pre-v1.3 lap 3 | Post-v1.3 lap 3 | Δ vs IDEAL (post) |
|---:|---:|---:|---:|
| 22 | 2:19.592 | **1:47.276** | −0.04 s |
| 26 | 1:53.455 | **1:47.177** | −0.14 s |
| 30 | 1:47.558 | **1:47.077** | −0.24 s |
| 33 (IDEAL) | 1:47.326 | **1:47.320** | 0 |
| 36 | 1:50.231 | **1:50.212** | +2.89 s |
| 40 | 2:02.770 | **2:02.737** | +15.42 s |
| 44 | 2:51.327 | **2:51.252** | +63.93 s |

Acceptance:

- §11.30 single-lap regression: pre-v1.3 `1:49.376` vs post-v1.3 `1:49.376`
  → **byte-equivalent**.
- §11.37 (low-PSI within ±2 s of IDEAL): `lap_time(26) − lap_time(33)` =
  `−0.14 s` ∈ `[−1, +2]`. Pre-v1.3 was `+6.13 s`. **PASS.**
- §11.38 (over-pressure penalty preserved at 44 psi): `lap_time(44) −
  lap_time(33)` = `+63.93 s` ≥ 30 s. **PASS.**
- §11.39 (asymmetric curve): `(lap_time(22) − lap_time(33))` = `−0.04 s` <
  `(lap_time(44) − lap_time(33))` = `+63.93 s`. Low side strictly flatter.
  **PASS.**

## Integration with neighbouring features

- **v2 per-wheel tyre state (§21.3, `architecture-lap-simulation-stint-v2.md`).**
  v1.3 is a strict extension of v2's `combined_grip_envelope`: same per-wheel
  state evolution (temp / wear / pressure), same scalar reduction shape, only
  the pressure side of the grip multiplier and the new drag scalar change.
- **v2 compound-aware parsing (§21.11).** v1.3 uses the active compound's
  `PRESSURE_IDEAL` (per-axle) and the new `k_drag` / `k_drag_reduction`
  fields. Both compounds carry the same hand-defaults in v1.3.
- **`driver_fit.py` skill-percentile fit (§13.14).** Asymmetric grip removes
  the under-pressure penalty from `f_pressure_grip`, which slightly tightens
  measured `skill_pct` across pre-v1.3 / v1.3 fits of the same telemetry —
  spec §13 notes this as a correction (not a regression). Drag does not enter
  `lat_g_max` so the fit shape is otherwise stable.
- **v1.4 backlog (§21.10).** Per-compound `k_drag` / `k_drag_reduction`
  calibration from telemetry pressure sweeps. The dataclass already carries
  per-compound fields; v1.4 will replace the hand-defaults with measured
  values.
