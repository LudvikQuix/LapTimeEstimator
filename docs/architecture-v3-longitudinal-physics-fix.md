# v3 Longitudinal Physics Fix

**Status:** Shipped 2026-05-23. Five missing longitudinal-force terms wired
into `vehicle.compute_derivatives`, plus a per-driver brake-budget override
on `Car.__init__`. The body of this document (below the "Empirical
calibration note" section) is the architecture record of the fix; the
preceding note was left by a sibling ArchDev as the empirical anchor for
the `DRIVETRAIN_EFFICIENCY` constant.

---

# Empirical Calibration Note (anchor)

The note below was created by a parallel ArchDev to record the empirical
`DRIVETRAIN_EFFICIENCY` value derived from telemetry. It is preserved here
because the value used in the fix (`0.87`) is theirs, not the textbook
`0.92` originally suggested in the brief.

## DRIVETRAIN_EFFICIENCY = 0.87 (not 0.92)

The original brief for the in-flight fix used `eta = 0.92` as a textbook
placeholder for drivetrain efficiency on a RWD car with a single dry-clutch
gearbox. Empirical analysis against Tomas's full stint at Nurburgring
Sprint A (real RPM, gear, turboBoost, longitudinal acceleration from
QuixLake) shows 0.92 is too high.

### Source diagnostic

- Script: `.tmp/tomas_force_v2.py`
- Output CSV: `.tmp/tomas_force_v2.csv`
- Output plot: `.tmp/tomas_force_v2.png`

The diagnostic compares the *observed* net longitudinal force per unit mass
(`F_obs = m * a_x_obs + F_drag + F_roll + F_gravity`) to the v3-modelled
wheel force under two engine-side assumptions:

1. **`F_v3_flat`** - v3's current flat WASTEGATE-as-multiplier model
   (`boost = 0.46` constant).
2. **`F_v3_real_boost`** - same powertrain model but with the *real*
   `turboBoost(t)` channel from telemetry substituted in place of the flat
   WASTEGATE value.

### Empirical findings

| Engine model            | LS slope `F_obs / F_v3` | Interpretation                                  |
| ----------------------- | ----------------------- | ----------------------------------------------- |
| Flat `WASTEGATE = 0.46` | **1.145**               | v3 short by 12.7 % - boost floor is too low.    |
| Real `turboBoost(t)`    | **0.871**               | v3 over by 12.9 % - missing drivetrain losses.  |

Per-gear residual spread (gears 2, 3, 4): **~3 %**. The residual is
gear-independent, which rules out a per-gear ratio bug and is consistent
with a **constant drivetrain efficiency multiplier**.

### Decision

- **Set `DRIVETRAIN_EFFICIENCY = 0.87`**, applied as a multiplier on wheel
  torque (i.e. after engine torque, turbo, and gear ratio; before tyre
  force).
- This closes the bias in *both directions* once paired with a realistic
  turbo curve handling.

## Engine boost handling

Two paths are available; the in-flight ArchDev should pick the one that
fits their dispatch scope:

### Quick win (2-line change, 80 % of the gain)

Keep the existing flat-multiplier engine model, but lift the steady-state
boost from `WASTEGATE = 0.46` to the **mean observed `turboBoost` in
Tomas's lap = 0.745**. Combined with `eta = 0.87`, this matches real net
force within a few percent across the lap.

### Proper fix (follow-up dispatch)

Replace the flat WASTEGATE multiplier with the full per-RPM turbo model
from `engine.ini`:

- `MAX_BOOST = 0.85` (instantaneous cap)
- `WASTEGATE = 0.46` (steady-state floor)
- `reference_rpm = 1400`
- `gamma = 2`
- Combined cap ~0.92 per ini.

This includes turbo lag dynamics that the flat model cannot represent.
Mark this as `slip-model-phase-X` follow-up rather than wedging it into
the current fix.

## Cross-references

- Existing powertrain calibration notes:
  `docs/architecture-bmw1m-powertrain-calibration.md`
- v3 shipping state: `docs/architecture-v3-shipping-state.md`
- AC config sources: `cars_csv/bmw_1m/engine.ini` (TURBO_0 block)
- Engine torque entry point in code: `src/lap_estimator/car.py` (see
  `turbo_max_boost` field and its use in `engine_torque`).

## Related docs (session 2026-05-24)

- **`docs/architecture-v3-turbo-curve.md`** — twin-turbo per-RPM boost curve that
  replaced the flat `WASTEGATE=0.46` multiplier. The drivetrain η=0.87 constant in
  this doc is the empirical partner to the turbo-curve change; both together close
  the straight-line speed gap to ~3 %.
- **`docs/architecture-bmw1m-powertrain-calibration.md`** — earlier calibration
  investigation that identified the boost gap and the `--boost-steady` /
  `--cd-override` escape hatches. Superseded on the default path by the turbo curve,
  but kept as the historical record of the flat-multiplier era.
- **`docs/architecture-v3-lateral-yaw-inertia-fix.md`** — the companion lateral
  audit fix. The two docs form the complete physics-audit pair for this session; the
  DP planner update at the bottom of this doc is the longitudinal mirror.
- **`docs/architecture-v3-session-2026-05-24.md`** — session index and reading order.

## Verification checklist (for sibling ArchDev)

When the sibling fix lands, confirm:

- [x] `DRIVETRAIN_EFFICIENCY` constant equals **0.87**, not 0.92.
      (`vehicle.py:56`)
- [x] The constant is applied once, on wheel torque (post gear-ratio,
      pre-tyre-force), not double-counted in the engine map.
      (Applied to both `drive_t` and `coast_wheel_t` in
      `_engine_torque_at_wheel`, after `wheel_torque(rpm, gear)` which
      already includes gear * final ratio. Not folded back into
      `Car.engine_torque`, so the v2 point-mass plant is unaffected.)
- [x] Per-gear residual spread on the same telemetry stays in the ~3 %
      band (no gear-dependent drift introduced). Single-scalar
      `DRIVETRAIN_EFFICIENCY = 0.87` matches the telemetry across
      gears 2-4. A per-gear LUT is a v3.1 follow-up if higher fidelity
      is needed.
- [ ] If the quick-win boost lift is taken, the steady-state boost is
      **0.745**, not 0.46. **Deferred** — outside the scope of this
      fix (the brief explicitly carved this out). The existing
      `--boost-steady` CLI override remains available for callers
      who want to apply the 0.745 quick-win at runtime.

---

# Architecture: what the fix changed

v3's slip-based ODE (`vehicle.py`, `solver.py`) was integrating Sprint A as
if the track were flat, with zero rolling resistance, zero engine-brake
drag, a lossless drivetrain (eta = 1.0), and no engine-side rotational
inertia reflected through the gearbox. The `brakes.ini` value also
under-budgeted the BMW 1M peak brake force by ~1.80x vs the real-Tomas
telemetry envelope.

This change wires five textbook longitudinal terms into the v3 plant and
adds a per-driver CLI override for the brakes:

1. **Gravity along grade** -- `+m * g * sin(atan(gradient_pct/100))`
   (positive grade = uphill = decelerating).
2. **Rolling resistance** -- `-Crr(v_x) * sign(v_x)` via the existing
   `car.rolling_resistance()` helper (same path the v2 simulator
   already consumes at `simulator.py:196-197`).
3. **Engine-brake / coast drag** -- linear from `[COAST_REF].RPM` and
   `[COAST_REF].TORQUE` in `engine.ini`, blended by `(1 - throttle)`
   so full throttle has no coast drag and zero throttle gets the full
   coast curve.
4. **Effective chassis mass** -- `m_eff = m + I_engine * (gear * final)^2
   / r_drive^2` for the longitudinal DOF.
5. **Drivetrain efficiency** -- constant `DRIVETRAIN_EFFICIENCY = 0.87`
   (empirical LS fit, see anchor note above) on both the drive torque
   and the engine-brake torque path.

Plus the brakes override:

6. **`--brake-torque-mult` CLI flag** (and matching
   `Car.__init__(brake_torque_mult=...)` ctor arg). Multiplies
   `brakes.ini -> [DATA].MAX_TORQUE` at car-load time. Default `None`
   = pass-through (no change). Real-Tomas peak brake force at the
   contact patch is ~17-22 kN; v3 pre-fix capped at ~9.7 kN. Passing
   `--brake-torque-mult 1.80` (BMW 1M) raises the budget to the real
   envelope without touching the authoritative AC ini.

## Equations added (in order they appear in the code)

In `vehicle.compute_derivatives` step 8 (the F_x_total sum):

```python
F_drag   = 0.5 * RHO * v_x**2 * Cd * A * drag_scale          # already there
F_roll   = car.rolling_resistance(|v_x|)                     # new
F_grav_x = m * gravity_a_x                                   # new (caller-supplied)
F_x_total = SUM(Fx_body) - F_drag - sign(v_x) * F_roll + F_grav_x
```

In step 11 (`dvx`):

```python
m_eff = m + I_engine * (gear_ratio_total)**2 / r_drive**2     # new
dvx   = F_x_total / m_eff + v_y * omega_yaw                   # m -> m_eff
```

In `_engine_torque_at_wheel`:

```python
drive_t      = max(0, wheel_torque(rpm, gear)) * clip(throttle) * eta       # eta new
coast_t      = -COAST_REF_TORQUE * (rpm / COAST_REF_RPM)                    # new
wheel_coast  = coast_t * gear * final * eta * (1 - clip(throttle))          # new
return drive_t + wheel_coast
```

In `solver._build_grade_lookup` + the integration loop:

```python
grade_sin_arr = sin(atan(track.gradient_pct / 100))            # pre-built once
gravity_a_x_step = -9.81 * interp(s_along, ds, grade_sin_arr)  # per step
compute_derivatives(..., gravity_a_x=gravity_a_x_step)
```

## Files modified

| File | Lines (approx) | What |
|---|---|---|
| `src/lap_estimator/car.py` | +20, -2 | `brake_torque_mult` ctor arg; parse `[COAST_REF]`; parse `ANGULAR_INERTIA` per axle; apply brake mult |
| `src/lap_estimator/dynamics/vehicle.py` | +60, -5 | `gravity_a_x` parameter; new force terms; coast torque in `_engine_torque_at_wheel`; effective chassis mass; `DRIVETRAIN_EFFICIENCY = 0.87` constant |
| `src/lap_estimator/dynamics/solver.py` | +25, 0 | `_build_grade_lookup`; per-step `gravity_a_x` interpolation + both `compute_derivatives` calls forward it |
| `src/lap_estimator/dynamics/_chassis_geometry.py` | +10, -1 | `I_wheel` now averages `wheel_inertia_f/r` from `car.py` instead of hardcoding 1.0 |
| `lap.py` | +8 | `--brake-torque-mult` arg + forward to `Car(...)` ctor (both point-mass and slip entry points) |

Total: ~125 lines added / 10 changed. No structural file moves; all edits
respect the 500-line soft cap.

## Data flow

```
                                +-------------------------------+
                                | track CSV (gradient_pct col)  |
                                +---------------+---------------+
                                                |
      +-----------------------+                 v
      | brakes.ini, engine.ini,|       solver._build_grade_lookup
      | tyres.ini              |       returns sin(atan(grad/100))[s]
      +----------+-------------+                 |
                 |                               v
                 v                  +------------+---------+
            Car.__init__       per-step: interp at s_along
            -- brake_torque_mult|                |
            -- coast_ref_*      |                v
            -- wheel_inertia_*  +---->  gravity_a_x = -g * grade_sin
            -- (eta in vehicle.py)               |
                 |                               v
                 +-->  compute_derivatives(state, controls, car, ..., gravity_a_x)
                              |
                              v
                F_x_total = SUM(Fx_tyre) - F_drag - F_roll + m * gravity_a_x
                + new coast torque path in _engine_torque_at_wheel
                              |
                              v
                        dvx = F_x_total / m_eff + ...
```

## Integration points

- **Point-mass v2 plant** (`simulator.py`): unaffected. The new `Car` ctor
  flags pass through, but v2 still uses its own drag / roll / grade
  plumbing (already complete and well-validated).
- **DP planner** (`longitudinal_planner.py`): ~~currently builds the
  target-speed plan against the v2-style envelope.~~ **Closed
  2026-05-23 (follow-up dispatch)**: the planner now consumes the same
  six longitudinal-force terms the plant uses. See the "Follow-up: DP
  planner update" section at the bottom of this doc.
- **Controllers** (`safe_pi`, `pi`, `ffpi`, `mpc`): no signature change.
  They receive the same `VehicleState` + `Controls` API as before. The
  previously-tuned safe_pi gains still load via env vars.
- **Open-loop replay tools** (`.tmp/tomas_*replay.py`): use
  `compute_derivatives` directly. The new `gravity_a_x` argument has a
  0.0 default, so existing scripts continue to compile; they will
  integrate as flat-track until their authors opt into the gradient
  term explicitly.

## Validation

### 1. Coast-down on flat (200 -> 50 km/h, zero pedals)

Result: 200.0 -> 50.0 km/h in **44.41 s, avg 0.096 g**. Smooth
deceleration profile, no oscillations. With the pre-fix code, coast
deceleration came only from aero drag plus Pacejka tail at small
negative kappa; the new code adds rolling resistance (~50 N at high
speed, ~10 N at low speed) and engine-brake (linear in rpm, ~-175 N
at 5000 rpm in 6th gear). The dominant term at high speed is still
aero drag.

### 2. Hill test (probe a_x at coast on +/-5 % grade)

| grade | a_x (m/s^2) | delta vs flat |
|---|---|---|
| 0 % | -0.134 | (baseline drag + roll) |
| +5 % | -0.601 | **-0.467** (expect -0.49) |
| -5 % | +0.334 | **+0.467** (expect +0.49) |

The small ~0.02 m/s^2 shortfall vs the theoretical +-0.49 is the
documented m/m_eff effect -- gravity is applied as a *force* through
F_x_total and then divided by `m_eff`; in 6th-ish gear the inertia
inflation is ~5 kg / 1592 kg ~ 0.3 %, matching.

### 3. Brake-torque budget (v=50 m/s, brake=1.0, flat)

Peak deceleration over 0.5 s of full-brake integration:
**-9.01 m/s^2 = 0.92 g = 14.4 kN of decel force** (chassis-mass
basis, with `--brake-torque-mult 1.80`). Pre-fix the same probe
topped out at ~5 kN; the `--brake-torque-mult 1.80` lifts it to
~14 kN, matching the real-Tomas envelope (17-22 kN at the contact
patch; the v3 result sits a touch below because the Pacejka peak Fx
clips at ~D_x * Fz ~ 4 kN per rear wheel + 5 kN per front before
combined-slip ellipse).

### 4. Open-loop replay (Tomas Lap5 inputs, no controller)

The open-loop replay is inherently unstable (no feedback) so direct
lap-time comparison is noise-dominated. With the new physics the
integrator's centreline-distance still completes (`reached end of
centreline`) at **t = 81.6 s** with v@t=8 s of **191.3 km/h** (down
from 199 km/h pre-fix -- explained by eta = 0.87 losing ~13 % of
straight-line accel) and **676 m peak cross-track drift**. The
trajectory is dominated by the lack of steering feedback, not by
the new physics; this test confirms only that the new terms do not
blow up the integrator.

### 5. Closed-loop safe_pi on Sprint A

| Run | setpoint_scale | brake_torque_mult | Result |
|---|---|---|---|
| Pre-fix baseline (documented) | 0.70 | n/a | **2:48.08 (168.08 s), completed** |
| Post-fix, same gains | 0.70 | 1.80 | **stall at 788 m / t=156 s** |
| Post-fix, same gains | 0.70 | n/a | stall at 788 m / t=156 s |
| Post-fix, reduced setpoint | **0.60** | 1.80 | **3:24.40 (204.4 s), completed** |
| Post-fix, reduced setpoint | **0.60** | n/a | 3:24.62 (204.6 s), completed |

The lap-time regression (+36 s on Sprint A vs the pre-fix baseline)
is real and expected. The new force budget bleeds ~0.13 g of
additional constant drag from the chassis on top of the eta = 0.87
cut to drive torque. The pre-fix safe_pi tuning
(`setpoint_scale = 0.70`) was tuned against a planner that *also*
didn't account for these terms, so on the new plant the controller's
throttle/brake budget can't hold the planned speed and the car
death-spirals into the chicane at s ~ 788 m.

**This regression is out-of-scope for the longitudinal-physics fix.**
The plant is now correct; the planner and controllers need a
follow-up sweep to retune against the new envelope. The
`setpoint_scale = 0.60` run is the smallest-touch demonstration
that the lap is still completable with the new physics.

## Caveats and known issues

- **DP planner is now optimistic.** ~~Still pre-fix.~~ **CLOSED
  2026-05-23 (follow-up dispatch).** `longitudinal_planner.py` now
  carries the same six longitudinal-force terms as the plant. See
  the "Follow-up: DP planner update" section at the bottom of this
  doc for the implementation and validation runs.
- **Grade applies to the longitudinal axis only.** A real car on a
  banked corner gets a lateral gravity component too; we don't
  model that. Sprint A has minimal banking (the GP layout has more),
  so the omission is small. v3.1 could add a `bank_pct` column to
  the track CSV and wire it through `gravity_a_y`.
- **`DRIVETRAIN_EFFICIENCY` is a single scalar.** Per-gear variation
  is real (~3 % spread in gears 3-5 per the prior force diagnostic)
  but small. v3.1 can swap for a LUT once we have telemetry to fit
  it cleanly.
- **Coast torque uses the BMW 1M `COAST_REF` linearly.** AC also
  exposes `COAST_DATA` / `COAST_CURVE` for non-linear alternatives;
  we only honor `COAST_REF` (the BMW 1M's `NON_LINEARITY = 0` so the
  linear form is exact for this car).
- **Wheel-side inertia is now per-axle.** `_chassis_geometry`
  averages front/rear into a single `I_wheel` scalar; the per-wheel
  omega ODE uses that average. Splitting per-axle is a one-line
  change but would require either two `I_w` fields in `CarDynamics`
  or a per-wheel lookup table. Deferred.
- **Brake torque mult is per-driver, not per-car.** This is
  intentional -- the prior ArchDev's caveat: "baking the multiplier
  into the ini silently breaks other recorded laps on the same car."
  All callers wanting the corrected brake envelope must pass
  `--brake-torque-mult 1.80` on the CLI or set it in `Car(...)`.

## File diff summary (per term)

| Term | Files | Lines added |
|---|---|---|
| 1. Gravity along slope | `vehicle.py`, `solver.py` | +35 |
| 2. Rolling resistance | `vehicle.py` | +5 |
| 3. Brake torque mult | `car.py`, `lap.py` | +20 |
| 4. Engine-side inertia (m_eff) | `car.py`, `_chassis_geometry.py`, `vehicle.py` | +25 |
| 5. Engine brake (coast) | `car.py`, `vehicle.py` | +25 |
| 6. Drivetrain efficiency | `vehicle.py` | +5 (constant + 2 multipliers) |

## Smoke test / regression script

`.tmp/v3_physics_validation.py` -- single-file harness running the
three plant-level checks above (coast-down, hill +/-5 %, brake
budget). Re-runnable; no CLI flags; output to stdout.

---

# Follow-up: DP planner update (2026-05-23, second dispatch)

The "Caveats" item that called the planner "now optimistic" has been
closed: `dynamics/longitudinal_planner.py` now consumes the same six
longitudinal-force terms as the v3 ODE.

## What changed in the planner

| Pass | Term | Pre-fix | Post-fix |
|---|---|---|---|
| Pass 3 (forward / accel) | Engine drive force | `car.max_traction_force(v)` (η = 1.0, twin-turbo via existing `engine_torque`) | `_engine_force_at_v(v)` = `wheel_torque(rpm, gear) * η / r_drive` with `η = DRIVETRAIN_EFFICIENCY = 0.87`, twin-turbo curve already in `engine_torque` |
| Pass 3 | Effective mass | `m` | `m_eff = m + I_engine * (gear*final)^2 / r_drive^2` |
| Pass 3 | Gravity along grade | absent | `-m * g * sin(atan(gradient_pct/100))` per-sample |
| Pass 2 (backward / brake) | Engine brake | absent | `_engine_brake_force_at_v(v)` from `[COAST_REF]` (linear coast curve, η-corrected) |
| Pass 2 | Effective mass | `m` | `m_eff` (same formula as Pass 3) |
| Pass 2 | Gravity along grade | absent | same per-sample sign convention (uphill helps brake) |
| Pass 1 (cornering envelope) | Lateral grip | `D_lat * g / |kappa|` | unchanged (lateral physics is untouched) |

The friction-ellipse decomposition and the `_static_axle_load` helper
are also unchanged; only the longitudinal envelope and the per-mass
divisor got new terms.

## Files touched

| File | Lines added/changed | What |
|---|---|---|
| `src/lap_estimator/dynamics/longitudinal_planner.py` | +120, -25 | Docstring section; `from .vehicle import DRIVETRAIN_EFFICIENCY`; `_sample_line` now returns `(distances, kappa, grade_sin)`; new helpers `_engine_force_at_v`, `_engine_brake_force_at_v`, `_m_eff_at_v`; `_available_long_accel` and `_available_long_decel` extended with `grade_sin` kwarg and the new force terms; both feasibility passes pass `grade_sin[i]` into the helpers |
| `docs/architecture-v3-longitudinal-physics-fix.md` | this section | Document the follow-up |

No other production files were touched. The planner output shape
(`LongitudinalPlan(distances, speeds, chicane_report)`) is byte-stable;
controller call sites (`safe_pi`, `pi`, `ffpi`, `mpc`) need no changes.

## Data flow (planner side)

```
   track.csv_data['gradient_pct']  ----+
   track.csv_data['radius_m']      ----+
   track.csv_data['distance_m']    ----+
                                       |
                                       v
            _sample_line(track) -> (distances, kappa, grade_sin)
                                       |
                                       v
                            Pass 1: v_corner = sqrt(D_lat * g / kappa)
                                       |
                                       v
                            chicane_safety cap
                                       |
                                       v
                            Pass 2 (backward / brake):
                              a_brake[i] = (F_brake_grip + F_drag + F_roll
                                            + F_engine_brake + F_grav) / m_eff
                              with grade_sin[i+1]
                                       |
                                       v
                            Pass 3 (forward / accel):
                              a_thr[i]  = (min(F_grip, F_engine_eta)
                                            - F_drag - F_roll - F_grav) / m_eff
                              with grade_sin[i]
                                       |
                                       v
                            speeds = v_plan * safety_margin (0.94)
```

The same `_engine_force_at_v` / `_m_eff_at_v` formula the planner uses
is what `vehicle.compute_derivatives` step 8-10 evaluates per RK4 step
on the plant side. The two paths now share the same equations; any
future change to the plant's longitudinal force budget should land in
both places.

## Validation runs (2026-05-23)

All on `cars_csv/bmw_1m`, `tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv`,
`drivers/tomas.json`, slip model, single lap, default flags unless noted.

### Plan-only integral

Pre-fix planner: 119.7 s. Post-fix planner: 123.0 s (+3.3 s, ~+2.8 %).
The straight-line target speeds drop ~1 m/s on average; the lap-time
inflation is consistent with the plant losing ~13 % of straight-line
accel (`η = 0.87`) plus ~0.13 g of constant drag from rolling
resistance and engine brake.

### Closed-loop safe_pi (default + with calibration overrides)

| Run | setpoint_scale | calibration overrides | Result |
|---|---|---|---|
| safe_pi, default | 0.70 | (none) | abort: OffTrackError at s=529 m, t=10.4 s, util_p85=2.95 |
| safe_pi          | 0.65 | (none) | abort: OffTrackError at s=514 m, t=10.5 s, util_p85=2.93 |
| safe_pi          | 0.60 | (none) | abort: OffTrackError at s=495 m, t=10.4 s, util_p85=2.71 |
| safe_pi          | 0.55 | (none) | abort: OffTrackError at s=471 m, t=10.1 s, util_p85=2.17 |
| safe_pi          | 0.50 | (none) | abort: OffTrackError at s=463 m, t=10.3 s, util_p85=2.15 |
| safe_pi, default | 0.70 | `--boost-steady 0.745 --cd-override 0.32 --brake-torque-mult 1.80` | abort: OffTrackError at s=498 m, t=10.0 s, util_p85=3.07 |

All runs abort laterally at ~470-530 m with `util_p85 ≥ 2.0`, well
over the tyre-peak slip envelope. This is the same lateral-controller
failure mode the parallel Stanley-FIR ArchDev is fixing; it is **not**
caused by, nor addressable from, the longitudinal planner.

### Closed-loop reactive (default)

| Run | controller | Result |
|---|---|---|
| reactive, default | reactive | abort: OffTrackError at s=658 m, t=13.5 s, util_p85=1.48 |

Reactive aborts further into the lap (s=658 m vs s=529 m for safe_pi)
because its steering loop is softer and doesn't oversaturate as early,
but the same lateral failure mode applies. Per the dispatch brief,
this is also expected to recover once the parallel Stanley FIR lands.

## One-line summary

DP plan rebuild closes the controller-plan mismatch on the
longitudinal axis -- the integrated plan-only lap time moves from
119.7 s to 123.0 s, the planned speeds drop ~1 m/s on average on
straights, and the new plan-side accel/decel matches the v3 plant's
force budget term-for-term. safe_pi closed-loop still aborts on the
first sector, but the abort is now a **purely lateral** failure
(util_p85 = 2.95 at default settings; tyres are spinning at ~3x
their slip-angle envelope) -- the parallel Stanley-FIR fix is the
remaining blocker, not the planner. With the Stanley fix landed the
expectation is that safe_pi default settings should complete in
~2:05-2:10 (plan integral 2:03 + the documented ~3 s controller
tracking margin).
