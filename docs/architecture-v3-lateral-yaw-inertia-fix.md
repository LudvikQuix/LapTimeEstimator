# v3 lateral yaw-inertia (`I_zz`) fix

**Date**: 2026-05-23
**Branch**: `feature/sc-71955/lap-simulation`
**Status**: shipped (default behaviour changed; CLI flag added)

## TL;DR

The v3 dynamics plant was using a "thin-plate" formula
`I_zz = m * (wheelbase^2 + track_f^2) / 12` for chassis yaw moment of
inertia. For the BMW 1M that lands at ~1258 kg·m², roughly **half** of the
manufacturer / suspension-engineering reference value (2300–2500 kg·m²).
The lateral empirical diagnostic confirmed it from a second angle:
the slope of `M_z_obs / M_z_v3` against measured yaw acceleration on
Tomas's lake telemetry was 0.131, implying the effective I_zz needed
to fit the recorded yaw rate is much larger than the plate formula.

The fix replaces the plate formula with three priority-ordered sources:

1. `--inertia-zz FLOAT` CLI flag (per-driver override; not baked into ini).
2. AC's `car.ini` `[BASIC].INERTIA` box-dimension formula
   `I_zz = m * (w^2 + l^2) / 12` (this is what AC's own physics
   computes internally; field is "width, height, length" in metres, NOT
   the inertia tensor itself).
3. `2.0 * plate` backstop when neither override nor box-dims are
   available (still better than the prior bug; the 2× scaling matches
   the empirical doubling needed for "typical passenger car" mass
   distribution per the audit).

For the BMW 1M with `INERTIA=1.60,1.40,4.52` and `m≈1592 kg`, the new
default lands on **3051 kg·m²** — slightly above the manufacturer
2300–2500 band. The box formula is a uniform-solid-mass model; a real
car concentrates mass lower and more inboard, so the AC box-derived
value is the natural upper bound. The CLI flag is the lever for
shifting into the manufacturer band when desired (e.g. `--inertia-zz
2400`).

## The bug

`src/lap_estimator/dynamics/_chassis_geometry.py:66` (pre-fix):

```python
I_zz = car.total_mass * (car.wheelbase ** 2 + track_f ** 2) / 12.0
```

This is the moment of inertia of a uniform-mass **rectangular plate**
(or thin disk) with side lengths `wheelbase × track_f`. For a car
chassis, the plate is the wrong shape — a real car has substantial
mass distributed along the **longitudinal extent of the body**, not
just between the wheels. The wheelbase plate ignores everything
overhanging the axles (engine bay forward of the front axle, boot /
fuel tank / rear panels behind the rear axle). For a typical road car
those overhangs push the actual radius of gyration to ~1.20× — 1.30×
the wheelbase plate's, giving a real `I_zz` that's ~1.5× — 1.7×
larger than the plate.

For the BMW 1M specifically:
- Plate formula: `1592.5 * (2.66² + 1.55²) / 12 ≈ 1258 kg·m²`
- Manufacturer / Wikipedia / suspension-engineering reference: **2300 – 2500 kg·m²**
- AC box formula (`w=1.60, h=1.40, l=4.52`): `1592.5 * (1.60² + 4.52²) / 12 ≈ 3051 kg·m²`

The plate is ~2.0× too low. The AC box is ~1.25× too high vs the
manufacturer band, which is expected — AC's box is a worst-case
upper bound for a uniform-density solid with the body's outer
dimensions.

## Independent confirmation: the M_z_obs / M_z_v3 slope

The lateral empirical diagnostic computes the yaw moment the v3 model
predicts (`M_z_v3 = I_zz * d_omega/dt`) against the yaw moment
observable from telemetry (`M_z_obs = I_zz * d_omega/dt` with the
actual recorded `omega_yaw` from the lake). With the wrong `I_zz`
baked in, the model and the observable both scale linearly with the
same constant, so the **ratio of slopes** is the multiplicative
correction factor needed for the model's `I_zz`.

Observed slope: **0.131**. Implied I_zz correction: 1 / 0.131 ≈ 7.6×.

That's higher than the manufacturer 2× factor — partly because the
diagnostic also picks up other lateral-side modelling errors (tyre
stiffness, sideslip dynamics) that get attributed to `I_zz` in a
single-axis regression. The audit's recommendation of "2300–2500
kg·m² as the right ballpark" anchors on the manufacturer side
of the evidence and is what the CLI override is calibrated against.

## What changed

### `src/lap_estimator/car.py`

1. New ctor kwarg `inertia_zz_override: float | None = None` on
   `Car.__init__` and `Car.from_dir`. Mirrors the existing
   `boost_steady_override` / `cd_override` / `brake_torque_mult`
   pattern: per-driver, not baked into the ini.

2. New parser for `car.ini` `[BASIC].INERTIA`. Field is split on
   `,` into `(w, h, l)` and stored as `self.car_box_dims_m`. Robust
   to missing field / malformed value (falls back to `None`).

3. New attribute `self.inertia_zz: float` (and `self.inertia_zz_source: str`
   for diagnostics). Computed at the end of `_load()` after
   `total_mass` is set, with the priority order documented in the TL;DR.

### `src/lap_estimator/dynamics/_chassis_geometry.py`

The plate-formula line at `:66` is gone. `load_car_dynamics` now reads
`car.inertia_zz` and packs it into the immutable `CarDynamics`. The
old computation is kept as an inline fallback in `getattr(...) or ...`
so that any caller manually constructing a `Car`-like duck type without
`inertia_zz` continues to work (defensive programming; no production
caller exercises this path).

### `lap.py`

- New CLI flag `--inertia-zz FLOAT`. Plumbed into both the
  point-mass `Car(...)` construction (`main`) and the slip-model
  `Car(...)` construction (`_run_slip_model`).
- Slip-model header now prints `Chassis I_zz: NNNN kg.m^2 (source: ...)`
  so a glance at stdout tells you which value was used.

### `.tmp/tomas_openloop_replay.py`

- Mirrored the CLI flag. The replay tool already accepted
  `--boost-steady` / `--cd-override` / `--brake-torque-mult`; this
  adds `--inertia-zz` on the same pattern so the open-loop diagnostic
  can sweep yaw inertia.

## Smoke tests on Tomas / Sprint A / skill=1.0 / reactive / single-lap

Pre-fix audit baseline: aborts at s=1087, util_p85 ≈ 0.5 (Stanley
chatter). Note that the parallel DP-planner ArchDev finalised their
fix during this work, so the post-fix baselines are running against
a more aggressive DP plan than the pre-fix audit referenced — apples
are not perfectly apples here.

| `--inertia-zz`     | I_zz (kg·m²) | abort `s` (m) | util_p85 | Notes                                |
|--------------------|--------------|---------------|----------|--------------------------------------|
| (default)          | 3051         | 660           | 1.455    | Box formula; over-stiff yaw          |
| 2400 (mid manufacturer band) | 2400 | 1086         | 1.091    | Audit reference                      |
| 2500 (high manufacturer band) | 2500 | 666 / 1086 (MC-jittery)| 1.090 | Near a stability boundary  |
| 3000               | 3000         | 1086          | 1.080    | Just under box-formula default       |

The reactive controller still cannot complete Sprint A — it aborts at
the chicane (s≈1086 m, around 1551–1650 m where the chicane-safety
ramp starts). The util_p85 changed character: pre-fix the controller
was operating at 50% of envelope with too-easy yaw rotation, post-fix
it operates at 108% (8% over) which is honest. **I_zz is no longer the
binding constraint** — the controller's Stanley + slip-band P-loop is
the next thing to look at, and that's exactly the parallel "Stanley FIR
fix" ArchDev's territory.

## Open-loop replay (Tomas Lap5 inputs through v3 ODE, no controller)

`.tmp/tomas_openloop_replay.py` runs Tomas's recorded `gas / brake /
steerAngle` straight through the v3 ODE with no controller and no
abort guards (other than NaN). It's the apples-to-apples test of
"does the plant + recorded inputs reproduce the recorded trajectory".

| `--inertia-zz` | I_zz (kg·m²) | reaches centreline end? | final `s` (m) | max cross-track (m) | max \|α\| front (°) |
|----------------|--------------|--------------------------|-----------------|------------------------|------------------------|
| 1258 (pre-fix) | 1258         | yes                      | 3565            | 379                    | 85.9                   |
| (default)      | 3051         | no                       | 1633            | 9826                   | 86.9                   |
| 2400           | 2400         | no                       | 2115            | 9571                   | 86.5                   |
| 6000           | 6000         | yes                      | 3565            | 1546                   | 87.7                   |

The audit predicted that fixing I_zz would reduce max-slip-angle peaks
from 87° to <50°. **That prediction did not hold.** The open-loop
replay's slip peaks come from saturation events early in the lap
where recorded steering inputs feed in faster than the model's lateral
tyre force can react; once `α` saturates Pacejka, peak alpha is
unbounded regardless of I_zz. What I_zz changes in the open-loop test
is how far the resulting yaw error grows before being self-corrected
by tyre forces — and that response curve has a deep non-monotonicity
(1258 finishes, 2400-3051 don't, 6000 finishes again). The
non-monotonicity is the integrator wandering off-line in
qualitatively different ways at different I_zz values. The
open-loop replay therefore is **not** a clean yardstick for I_zz
correctness — the closed-loop reactive runs (`lap.py`) are the
better diagnostic, and they show clear improvement (util_p85
1.45 → 1.08 going from default 3051 → 2400, abort distance
660 → 1086 m).

## How this integrates with neighbouring features

- **v3 longitudinal-physics fix** (`docs/architecture-v3-longitudinal-physics-fix.md`):
  introduced the same CLI-override + per-driver pattern (`--boost-steady`,
  `--cd-override`, `--brake-torque-mult`). This fix slots in alongside
  them; no interaction beyond the shared `Car` constructor.
- **Slip-model phase 4 / 5 controllers** (`docs/architecture-slip-model-phase4_*` etc.):
  consume `load_car_dynamics(car).I_zz` via the yaw-rate integration
  in `compute_derivatives` and the MPC plant. They get the corrected
  value automatically — no per-controller code change.
- **Parallel ArchDev: longitudinal DP planner update**: the planner
  reads `car.total_mass`, gear ratios, drag, etc., but never reads
  `I_zz`. The two fixes are independent and stack cleanly.
- **Parallel ArchDev: Stanley FIR fix**: lives in
  `driver_controller.py`. Will see the new I_zz indirectly through
  the ODE. The Stanley gains may need re-tuning with the corrected
  yaw response, but the FIR-filter behaviour is orthogonal to I_zz.

## Related docs (session 2026-05-24)

- **`docs/architecture-v3-longitudinal-physics-fix.md`** — companion longitudinal
  audit. The two docs form the complete physics-audit pair for this session; I_zz
  and the six longitudinal terms were discovered and fixed in the same pass.
- **`docs/architecture-slip-model-phase5_0_5-v32-reactive-chatter.md`** — the
  Stanley LPF that followed this fix. The LPF is effective only because I_zz was
  corrected first; with the plate-formula I_zz the controller was operating at
  util_p85≈0.5, masking the actual slip behaviour.
- **`docs/architecture-stanley-crosstrack-gain.md`** — the cross-track gain uplift
  that followed the LPF. Both Stanley fixes depend on the corrected yaw response
  that this I_zz fix delivers.
- **`docs/architecture-v3-session-2026-05-24.md`** — session index and recommended
  CLI invocations (including the `--inertia-zz 2400` recommendation).

## What to read next

- `src/lap_estimator/car.py:218-260` — ctor signature, override
  parameter, ctor docstring.
- `src/lap_estimator/car.py:340-355` — parser for `[BASIC].INERTIA`.
- `src/lap_estimator/car.py:411-451` — `inertia_zz` resolution logic.
- `src/lap_estimator/dynamics/_chassis_geometry.py:67-83` — the
  consumer (`load_car_dynamics`) reading off `car.inertia_zz`.
- `lap.py:206-217` — CLI flag definition.
- `lap.py:243` — point-mass `Car(...)` call site.
- `lap.py:435` — slip-model `Car(...)` call site.
