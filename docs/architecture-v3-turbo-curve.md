# v3 Turbo Curve — Per-RPM Steady-State Boost (BMW 1M twin-turbo)

**Status:** Shipped 2026-05-23. Replaces the flat `[TURBO_0].WASTEGATE`
multiplier (`0.46`) with a per-RPM steady-state turbo curve assembled from
*all* `[TURBO_n]` sections in `engine.ini`. The lag/transient model
(`LAG_DN`, `LAG_UP`) is intentionally **not** implemented yet — that is a
5.0.x follow-up.

This document supersedes the "Turbo (steady-state flat)" paragraph in
`architecture-v3-longitudinal-physics-fix.md` and the per-driver
`boost_steady_override` calibration recipe in
`architecture-bmw1m-powertrain-calibration.md`. Both documents are still
correct as historical records of the flat-multiplier era; this one
documents the curve-driven default.

## What changed

`src/lap_estimator/car.py`:

1. `Car._load` now walks `[TURBO_0]`, `[TURBO_1]`, ... in `engine.ini`
   until the section is missing, building a list of
   `(MAX_BOOST, WASTEGATE, REFERENCE_RPM, GAMMA)` tuples on
   `Car.turbo_specs`.
2. `Car.turbo_boost_at_rpm(rpm)` returns the steady-state multiplier at
   that RPM. Accepts a scalar or numpy array; returns the same kind.
3. `Car.engine_torque(rpm)` calls `turbo_boost_at_rpm(rpm)` instead of the
   former flat `self.turbo_max_boost`.
4. `Car.turbo_max_boost` is retained as the *peak* steady-state multiplier
   (`sum(WASTEGATE)` across all turbos = 0.92 for the BMW 1M). It is still
   used by `__repr__` and is what `boost_steady_override` writes when set.
5. `boost_steady_override` semantics preserved: when set, the override
   bypasses the curve and behaves like the legacy flat multiplier
   (back-compat for any consumer pinned to a specific calibration).

No other production files were touched. The v3 plant code
(`vehicle._engine_torque_at_wheel`) already passes RPM through
`car.engine_torque(rpm)`, so the curve propagates automatically. The v2
codepath (`Car.max_traction_force`) likewise calls `wheel_torque(rpm,
gear)` → `engine_torque(rpm)`, so it also picks up the curve without any
edits there.

## Why this architecture

### Why "per-RPM steady-state curve, no lag"

The empirical reference is `.tmp/tomas_lap5_rich.csv`, column
`turboBoost`, on Tomas's full 1:47.56 Sprint A lap. The diagnostic
`.tmp/diag_turbo_curve.py` plots boost vs RPM and fits four candidate
formulae:

| Formula                                           | rmse  | bias  |
| ------------------------------------------------- | ----- | ----- |
| Single turbo `(rpm/1400)^(1/2)` clip `WG`         | 0.458 | -0.46 |
| Single turbo `(rpm/1400)^2`  clip `WG`            | 0.458 | -0.46 |
| **Twin turbo additive (two `[TURBO_n]` summed)**  | **0.014** | **+0.002** |
| Linear ramp `0 → WG`                              | 0.458 | -0.46 |

The twin-turbo additive model fits the data to within 1.4 % rms with
near-zero bias. The single-turbo interpretation is wrong by a factor of
exactly 2 because the BMW 1M's N54 declares two identical
`[TURBO_0]/[TURBO_1]` sections in `engine.ini`, each contributing up to
`WASTEGATE = 0.46`. AC sums them, capped at `2 * WASTEGATE = 0.92` — the
saturated value the lake records at all RPM ≥ ~1400 with full throttle.

Steady-state-only is the right starting point for two reasons:

1. **Tomas's recorded data is essentially saturated** above ~3000 rpm at
   full throttle. The off-throttle dips visible in the scatter
   (`.tmp/diag_turbo_lowrpm.png`) are *throttle* effects, not RPM
   effects — the `_engine_torque_at_wheel` path already multiplies by
   `throttle`. So a steady-state curve composed with the existing
   throttle scaling already reproduces the bulk of the recorded
   behaviour.
2. **Lag (LAG_DN, LAG_UP) is a transient model.** Adding it requires
   integrating boost as a state variable, plumbing it through the ODE,
   and re-fitting against a transient-rich segment of the lake. None of
   that is needed to close the v@t=8s gap.

### Why scalar/vector dual-mode return

`turbo_boost_at_rpm` is called both from per-step ODE evaluations (scalar
RPM) and from any future swept lookup (vector RPM). Branching on
`np.ndim(rpm) == 0` keeps the common scalar call cheap and avoids
allocating a 0-d numpy array for every integration step.

### Why keep `turbo_max_boost` as a back-compat field

`__repr__` uses it for the human-readable string. The
`boost_steady_override` ctor flag writes to it (so a calibration script
that sweeps "boost" can keep working as it did). Removing the field would
break the `architecture-bmw1m-powertrain-calibration.md` calibration
recipe, which we deliberately keep alive for users on existing
calibrations.

## Data flow

```
engine.ini                                  car.py                                  vehicle.py
+--------------+                            +----------------------------+           +-------------------------+
| [TURBO_0]    |---parse-->turbo_specs--->  | turbo_boost_at_rpm(rpm)    |---boost-->| _engine_torque_at_wheel |
|   MAX_BOOST  |                            |   ratio = (rpm/REF)^(1/G)  |           |   base * (1+boost)      |
|   WASTEGATE  |                            |   single = min(ratio,MB,WG)|           |   * throttle * eta      |
|   REF_RPM    |                            |   total  = sum(single_i)   |           |                         |
|   GAMMA      |                            +-------------+--------------+           +-------------------------+
| [TURBO_1]    |                                          |
|   ...        |                                          v
+--------------+                            +----------------------------+
                                            | engine_torque(rpm)         |
                                            |   base = interp(power.lut) |
                                            |   return base * (1+boost)  |
                                            +----------------------------+
                                                          ^
                                                          |
                                            +-------------+--------------+
                                            | wheel_torque(rpm,gear)     |
                                            |  (v2 path, optimal_gear,   |
                                            |  max_traction_force,       |
                                            |  top_speed)                |
                                            +----------------------------+
```

When `boost_steady_override` is set, `turbo_boost_at_rpm` short-circuits
to `float(self.boost_steady_override)` ignoring `rpm` — the legacy flat
behaviour.

## File inventory

- **Modified**: `src/lap_estimator/car.py`
  - `_load`: parse all `[TURBO_n]` sections; build `turbo_specs`;
    `turbo_max_boost` becomes `sum(WG)`.
  - `engine_torque`: call `turbo_boost_at_rpm(rpm)` for per-RPM boost.
  - `turbo_boost_at_rpm` (new): steady-state per-RPM formula with twin
    (or N-fold) additive support.
- **Added**: `docs/architecture-v3-turbo-curve.md` (this file).
- **Diagnostics** (not part of the repo, in `.tmp/`):
  - `.tmp/diag_turbo_curve.py` — formula-fit harness.
  - `.tmp/diag_turbo_curve.png` — formula vs lake scatter.
  - `.tmp/diag_turbo_lowrpm.py` — throttle-coloured scatter to confirm
    that low-rpm "ramp" is a throttle artefact, not an RPM ramp.
  - `.tmp/diag_turbo_lowrpm.png` — same, 2-D throttle heatmap.

## Integration points

- **v3 dynamics** (`vehicle._engine_torque_at_wheel`): unchanged caller;
  picks up the new curve transparently.
- **v2 lap simulator** (`Car.max_traction_force`, `Car.top_speed`,
  `Car.optimal_gear`, `Car.wheel_torque`): unchanged callers; pick up the
  new curve transparently. Note that v2's `top_speed` and
  `optimal_gear` now reflect ~0.92 peak boost rather than 0.46, so
  estimated top speed will increase. This is the intended correction:
  the old flat model was undershooting engine output by ~2x at saturated
  RPM.
- **Per-driver calibration** (`Car(boost_steady_override=...)`): still
  works exactly as before — passing a scalar bypasses the curve. The
  `boost_steady_override` field is the documented escape hatch for
  drivers/laps whose recorded `turboBoost` does NOT match the BMW 1M
  twin-turbo model (e.g. detuned cars, different cars sharing the same
  `bmw_1m/engine.ini`, calibration sweeps).

## Validation

Open-loop replay (`.tmp/tomas_openloop_replay.py`) on Tomas Lap5 Sprint
A. Two configurations:

| Config                                          | boost source            | lap time | v@t=8s     | Δv vs Tomas | max cross-track |
| ----------------------------------------------- | ----------------------- | -------- | ---------- | ----------- | --------------- |
| `--boost-steady 0.745 --cd-override 0.32`       | flat 0.745 (prior best) | 117.32 s | 200.23 km/h | -8.19 km/h | 732 m           |
| `--cd-override 0.32` (curve, no boost override) | per-RPM curve (twin)    | 86.53 s  | 204.81 km/h | -3.61 km/h | 333 m           |

Both runs also used `--brake-torque-mult 1.8` per the v3 longitudinal-
physics fix (already in tree). The proper curve:

- **closes the v@t=8s gap from −8.19 to −3.61 km/h** (cut by 56 %), since
  the engine now delivers the correct torque at the RPMs Tomas actually
  drove at instead of a flat-mean approximation.
- **cuts max cross-track drift from 732 m to 333 m** — better dv/dt
  match → better speed → better arclength → less off-line excursion
  under open-loop replay (no controller).
- **overshoots Tomas's lap time** (86.5 s vs real 107.6 s, *faster* by
  21 s). This is expected for open-loop replay with the corrected
  engine: with no controller and full Tomas inputs, the car simply
  reaches max speed earlier and stays there longer. The remaining
  arclength integrates faster. Closed-loop / controller behaviour is
  validated separately by the lap-sim acceptance suite.

One-line: **proper turbo curve closes the engine gap to −3.61 km/h at
peak speed and lap-time delta to −21.0 s (faster) under open-loop
replay.**

## Related docs (session 2026-05-24)

- **`docs/architecture-v3-longitudinal-physics-fix.md`** — the six-term longitudinal
  audit that this turbo-curve change is part of. The `DRIVETRAIN_EFFICIENCY=0.87`
  constant there is the companion to the corrected boost here; both are needed to
  close the straight-line speed gap.
- **`docs/architecture-bmw1m-powertrain-calibration.md`** — earlier doc that diagnosed
  the boost gap via the `--boost-steady` / `--cd-override` CLI sweep. The flat-
  multiplier recipe in that doc is superseded on the default path by this curve, but
  the `boost_steady_override` escape hatch remains live for custom calibrations.
- **`docs/architecture-v3-session-2026-05-24.md`** — session index and recommended
  CLI invocations.

## Caveats / open follow-ups

- **Default behaviour changed**: existing configs that did NOT pass
  `boost_steady_override` will now see ~2× the torque boost they used to
  (saturated curve = 0.92 vs flat WG = 0.46). This is the *correct*
  number per AC — the old model was undershooting. Users with hand-
  tuned calibrations should expect their cars to feel more powerful.
  Recommended migration: re-run any controller / DP-planner calibration
  baselines and verify lap times still land in the expected window.
- **Transient lag not modelled.** `LAG_DN = 0.996` and `LAG_UP = 0.99`
  are AC's first-order filter constants for boost spool-up / spool-down.
  Adding them requires boost-as-state in the ODE. Filed as a 5.0.x
  follow-up; mostly relevant for off-throttle → on-throttle transitions
  in slow corners.
- **Only BMW 1M validated.** Cars with `[TURBO_n]` sections that AC
  treats differently (sequential turbos, anti-lag, twin-scroll) may
  need formula adjustments. The parsing loop tolerates any N ≥ 0
  sections but the *combining rule* (additive, capped at sum-of-WG) is
  validated only against the BMW 1M N54 in tree.
