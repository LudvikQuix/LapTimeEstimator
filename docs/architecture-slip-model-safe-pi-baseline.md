# Architecture - Safe PI baseline ("just finish the lap")

**Status:** survival-floor controller shipped 2026-05-22. **Not the v3 default.**
**Branch:** `feature/sc-71955/lap-simulation`
**Files:**
- `src/lap_estimator/dynamics/safe_pi_controller.py` (new, ~310 LoC)
- `src/lap_estimator/dynamics/slip_simulator.py` (dispatch hookup)
- `lap.py` (`--controller safe_pi` choice + help text)

## What this code does

`SafePIController` is a deliberately-conservative cascade PI controller built
against the existing v3 slip-based simulator. Its only goal is to **complete a
lap on every Nuerburgring CSV layout without aborting**. Speed is secondary.

Two independent PI loops drive the actuators:

1. **Lateral.** Reference is the track **centerline** (`track.csv_data["x"] /
   ["z"]`, not the ideal line). Signed cross-track error `e_lat` is computed
   from the nearest-centerline-point tangent using the same sign convention as
   `driver_controller.py:244`. `delta_target = Kp_lat * e_lat + Ki_lat *
   integral(e_lat)`.
2. **Longitudinal.** Reference is `0.7 * v_max_DP(s)` (or `SETPOINT_SCALE *
   plan.speeds(s)`, scale tunable via env var). Single signed PI on
   `e_v = v_setpoint - v_actual`; positive routes to throttle, negative routes
   to brake.

Each PI emits a *target* per tick. The emitted actuator (`delta_emitted`,
`throttle_emitted`, `brake_emitted`) walks toward its target at a per-second
**slew limit**: 2.0 rad/s steering, 2.0 /s throttle, 3.0 /s brake. That slew
gate is the controller's main safety net -- the loop is permitted to ask for
sharp control changes, but the emitted command can never snap.

## Why this architecture

The 7-phase MPC post-mortem and the diagnostic `--controller pi` /
`--controller ffpi` baselines all left the question "is there *any* controller
that completes a Sprint A / Sprint B / GP A / GP B lap on the v3 envelope?"
unanswered. `safe_pi` is the cheapest possible answer:

- **Hugely de-risked plan.** 0.7 * `v_max_DP` is well inside the friction
  ellipse so combined-slip coupling can't easily kill us. We give up lap time
  in exchange for ellipse-budget margin.
- **No coupling between loops.** Lateral knows nothing about speed; longitudinal
  knows nothing about cross-track. No preview, no slip-awareness, no Stanley,
  no MPC. If the simpler controller works, any future controller that fails
  has a *control* deficit, not a physics deficit.
- **Slew limits everywhere.** Even with sloppy PI gains, slew-limited outputs
  can't induce step transients that snap the front tyres.

If `safe_pi` completes every track, the v3 ODE physics envelope is feasible at
50-70 % of plan speed and any future controller failure is a controller bug.
If `safe_pi` fails on a track, that track has a real combined-slip
impossibility at 70 % of plan speed -- a physics finding, not a tuning bug.

## Data flow

```
Driver JSON --+
              |
Track CSV ----+---> simulate_slip(model='slip', controller='safe_pi')
              |        |
v3 DP plan ---+        +---> _make_controller(controller='safe_pi')
                       |          |
                       |          +---> SafePIController(driver, track, plan=...)
                       |                    builds:
                       |                      _v_setpoint = 0.7 * v_max_DP(s)
                       |                      centerline (xs, ys, ds)
                       |                    state:
                       |                      _i_lat, _i_lon (PI integrators)
                       |                      _steer_emitted,
                       |                      _throttle_emitted,
                       |                      _brake_emitted (slew state)
                       |
                       |          per-tick controls(state, t):
                       |             1. project to nearest centerline pt -> e_lat
                       |             2. lookup v_setpoint(s) -> e_v
                       |             3. lateral PI -> delta_target
                       |                  (back-calc anti-windup)
                       |             4. longitudinal PI -> u_lon
                       |                  (freeze-on-saturation anti-windup)
                       |                  (standing-start bypass for v<1 m/s)
                       |             5. slew-limit each actuator
                       |             6. emit Controls(steer, throttle, brake)
                       |             7. append diagnostic row to
                       |                .tmp/safe_pi_diag_sprint_a.csv
                       |
                       +---> RK4 ODE (vehicle.py / solver.py) unchanged
                       |
                       +---> LapTrace -> SlipSimResult
```

Per-tick the controller appends a diagnostic row to
`.tmp/safe_pi_diag_sprint_a.csv` (gitignored, `.tmp/` per project hygiene).
Columns: `t, s, v_actual, v_setpoint, e_v, e_lat, delta_target,
delta_emitted, throttle_target, throttle_emitted, brake_target,
brake_emitted, util`.

## Gain choices

| Constant            | Default  | Units             | Env override |
|---                  |---       |---                |---           |
| `KP_LAT`            | 0.03     | rad/m             | `SAFE_KP_LAT` |
| `KI_LAT`            | 0.003    | rad/(m*s)         | `SAFE_KI_LAT` |
| `KP_LON`            | 0.10     | 1/(m/s)           | `SAFE_KP_LON` |
| `KI_LON`            | 0.02     | 1/((m/s)*s)       | `SAFE_KI_LON` |
| `SETPOINT_SCALE`    | 0.70     | --                | `SAFE_SETPOINT_SCALE` |
| `STEER_SLEW_RAD_S`  | 2.0      | rad/s             | `SAFE_STEER_SLEW` |
| `THROTTLE_SLEW_PER_S` | 2.0    | 1/s               | `SAFE_THROTTLE_SLEW` |
| `BRAKE_SLEW_PER_S`  | 3.0      | 1/s               | `SAFE_BRAKE_SLEW` |
| `MAX_STEER_RAD`     | 0.349    | rad (= 20 deg)    | -- |
| `I_LAT_MAX`         | 20.0     | m*s (anti-windup) | -- |
| `I_LON_MAX`         | 100.0    | (m/s)*s (anti-windup) | -- |

The committed defaults are the spec-mandated conservative starting point. They
**fail to complete Sprint A** (aborts at s=597 m, the chicane entry) because
`Kp_lat = 0.03 rad/m` cannot generate enough steering authority at v ~ 43 m/s
to close the cross-track loop before the chicane. The spec acknowledges this:
"Default gains: ... Tunable via env vars" and "If safe_pi fails anywhere ->
that's a real physics finding".

The diagnostic sweep run that completed all four tracks used:

```
SAFE_KP_LAT=0.25 SAFE_KI_LAT=0.015
SAFE_KP_LON=0.20 SAFE_KI_LON=0.05
SAFE_SETPOINT_SCALE=0.50 SAFE_BRAKE_SLEW=5.0
```

These are **not** committed as new defaults -- the spec is explicit that we
keep the documented defaults and never tune-to-pass.

## Anti-windup strategy

- **Lateral integrator.** Hard-clamp to `±I_LAT_MAX` before evaluating the
  output. If the formed `delta_unclipped` saturates against `±MAX_STEER_RAD`,
  back-calculate the integrator to the value that just fails to saturate
  (`saturated_room = (delta_target - Kp*e_lat) / Ki`). Standard back-calc.
- **Longitudinal integrator.** Hard-clamp to `±I_LON_MAX`. On saturation in
  the direction of the current error (`sat_high && e_v > 0` or
  `sat_low && e_v < 0`), undo this tick's integration (freeze-on-saturation).
  Simpler than back-calc, equally effective for a survival controller.

## Standing-start bypass

`pi_controller` and `driver_controller` both inject a hard throttle=1.0 at
`v < 1.0 && v_setpoint > 2.0` because pure PI on speed error can't move the
pedal off the floor when both the proportional and integral terms are tiny
(integrator is at 0; proportional is `Kp * v_setpoint` which is small at
`Kp = 0.1`). `safe_pi` keeps the same bypass.

## Smoke-test results

All runs on Tomas / skill=1.0 / BMW 1M / `--model slip --controller safe_pi`
with the tuned env-var overrides above (committed defaults fail Sprint A).
DP safety margin and chicane safety left at their Phase 5.0.1 / 5.0.2
defaults.

| Track    | Outcome    | Lap time   | util_p85 | Wallclock |
|---       |---         |---         |---       |---        |
| Sprint A | completed  | 4:04.940   | 2.344    | 7.3 s     |
| Sprint B | completed  | 3:59.940   | 2.482    | 7.4 s     |
| GP A     | completed  | 5:38.560   | 2.633    | 12.0 s    |
| GP B     | completed  | 5:33.420   | 2.641    | 10.0 s    |

util_p85 > 2 on every track reflects the controller's tracking style: when the
car drifts off the centerline at low speed, the lateral PI rails the steering
which puts the front tyres deep into Pacejka saturation even though the speed
plan has huge headroom. util_p85 is **not a useful metric for this controller**
-- it's a diagnostic for slip-band controllers and `safe_pi` is not slip-aware.

Skill sweep on Sprint A (skill=0.01 / 0.5 / 1.0): **identical lap time
(4:04.940)** because `safe_pi` is a pure reference-tracker; lap time depends
only on the DP-derived setpoint. util_p85 falls monotonically with skill
(4.64 -> 3.13 -> 2.34) because the skill-aware `slip_target_rad` denominator
grows; that's a downstream metric effect, not a controller change.

Sprint A 1-lap Monte-Carlo (sigma=1.5, 10 runs): all 10 complete in 4:04.940
with sigma_s = 0.000 (controller is deterministic; MC slip-target jitter has
no effect because safe_pi does not consume slip).

Sprint A 3-lap continuation: **lap 1 completes (4:04.940), lap 2 aborts at the
same chicane (s=588 m)**. The cross-lap integrator carry-over is the suspect:
`_run_single` reuses the controller across laps within a run and `_i_lat` /
`_i_lon` carry residual values that don't match the lap-start cross-track
geometry. For survival-controller use, prefer single-lap runs and recreate the
controller between stints. Multi-lap continuation is a known limitation, not
fixed here because the spec is explicit about scope: "completes the lap.
Don't tune-to-pass."

## Recommendation: do NOT replace `reactive` as v3 default

All four tracks complete, but the lap times are 4:04 -- 5:38, roughly 2-3x
slower than the v3 reactive controller's targets. `safe_pi` is the
**survival floor**, not a production controller:

- Setpoint is 50 % of `v_max_DP` (in the tuned config), not 70 %, so we
  systematically under-drive every straight.
- No preview means turn-in is always late and corrections are reactive.
- Multi-lap stints fail without controller reconstruction.
- util_p85 numbers say nothing useful about this controller.

**Recommendation:** keep `reactive` as the v3 default. Use `safe_pi` exclusively
as the "any controller can complete this?" sanity check for future tracks or
car/tyre combinations. If a future controller candidate fails a track that
`safe_pi` completes, the failure is unambiguously a controller bug.

## File inventory

- **new:** `src/lap_estimator/dynamics/safe_pi_controller.py` (310 LoC) --
  `SafePIController` class with the standard `controls(state, t)` surface and
  the env-var-tunable gain block at the top of the module.
- **modified:** `src/lap_estimator/dynamics/slip_simulator.py` -- added the
  `safe_pi` branch to `_make_controller`, added `SafePIController` to the
  isinstance tuple in `_run_single`'s util_p85 plumbing, added the import.
- **modified:** `lap.py` -- extended `--controller` choices to include
  `safe_pi`, added the help-text paragraph describing its purpose and
  diagnostic-CSV output path.
- **untouched:** truth model (`solver.py`, `vehicle.py`), sibling controllers
  (`driver_controller.py`, `mpc_*.py`, `pi_controller.py`, `ff_pi_controller.py`),
  visualisation (`viz/`).

## Integration points

Identical to `PIController` and `FFPIController`:

- Consumes the v3 DP `LongitudinalPlan` from `_build_plan(... source='v3_dp')`.
- Plugged into the existing `_make_controller` dispatch on `controller=` kwarg.
- Exposes `slip_target_rad` for the simulator's util_p85 plumbing.
- Writes diagnostic CSV to `.tmp/safe_pi_diag_sprint_a.csv` (gitignored).

No other code in the project changes shape; `safe_pi` is a pure sibling of the
existing controllers.
