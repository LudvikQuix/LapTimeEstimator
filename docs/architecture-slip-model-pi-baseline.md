# Architecture - PI baseline controller (physics-envelope smoke test)

**Status:** diagnostic instrument shipped 2026-05-22. **Not a production controller.**
**Branch:** `feature/sc-71955/lap-simulation`
**Files:**
- `src/lap_estimator/dynamics/pi_controller.py` (new, ~290 LoC)
- `src/lap_estimator/dynamics/slip_simulator.py` (dispatch hookup)
- `lap.py` (`--controller pi` choice + help text)

## What this code does

`PIController` is a deliberately-dumb cascade PI baseline that runs against the
existing v3 slip-based simulator pipeline. It exists to disprove (or fail to
disprove) the working theory that the 7 consecutive MPC iterations failing at
the Sprint A chicane are MPC-side bugs rather than physics-envelope
infeasibility. The control law has no preview, no slip-awareness, no Stanley
softening, no MPC, no fallback ladder. Two decoupled PI loops:

1. **Lateral.** `steer = clip(Kp_lat * e_lat + Ki_lat * integral_e_lat, ±20°)`,
   where `e_lat` is the signed perpendicular distance from chassis to the
   nearest ideal-line point (same cross-track definition the existing
   `DriverController` uses at `driver_controller.py:244`).
2. **Longitudinal.** `e_lon = v_target(s) - v_actual`; positive routes to
   throttle, negative routes to brake. Integrator on both legs with
   anti-windup clamp.

The controller consumes the v3 DP `LongitudinalPlan` exactly as the MPC does
(same `plan.distances` / `plan.speeds`), so any speed-target delta between PI
and MPC is purely a tracking-fidelity question.

## Why this architecture

The 7-phase MPC post-mortem (`docs/architecture-slip-model-phase5_0_*`) left
us unable to distinguish two hypotheses:

- **H1.** The MPC is over-engineered / buggy; a simpler controller against the
  same physics envelope would complete the lap.
- **H2.** The physics envelope is fundamentally infeasible at the planned
  speeds for this car/track; no controller can complete the lap without a
  Pacejka refit or a different track.

A reference controller "so simple it can't possibly be the bug" is the cheap
discriminator. We deliberately strip every feature the MPC has -- preview,
slip-awareness, anti-saturation logic, the three-tier fallback ladder -- so
that any failure mode the PI exhibits is **only** physics + plan +
proportional/integral feedback. That gives us a clean experimental cut.

## Data flow

```
Driver JSON --+
              |
Track CSV ----+---> simulate_slip(model='slip', controller='pi')
              |        |
v3 DP plan ---+        |---> _make_controller(controller='pi')
                       |          |
                       |          +---> PIController(driver, track, plan=...)
                       |                     ^
                       |                     |  per-tick:
                       |                     +--- controls(state, t) ->
                       |                            e_lat = perp dist
                       |                            e_lon = v_target - v_x
                       |                            steer = Kp_lat * e_lat + Ki_lat * I_lat
                       |                            (throttle | brake) from e_lon
                       |
                       +---> RK4 ODE (vehicle.py / solver.py) unchanged
                       |
                       +---> LapTrace -> SlipSimResult
```

Per-tick the controller also appends a diagnostic row to
`.tmp/pi_diag_sprint_a.csv` (gitignored). Columns:
`t, s, x, y, v, v_target, e_lat, e_lon, steer, throttle, brake, i_lat, i_lon`.
That CSV is the offline tuning target -- gains can be searched against the
recorded trace without re-running the sim.

## Gain choices

Documented as module-level constants with env-var overrides
(`PI_KP_LAT`, `PI_KI_LAT`, `PI_KP_THROTTLE`, `PI_KP_BRAKE`, `PI_KI_LON`):

| Constant       | Default  | Units             |
|---             |---       |---                |
| `KP_LAT`       | 0.05     | rad/m             |
| `KI_LAT`       | 0.01     | rad/(m*s)         |
| `KP_THROTTLE`  | 0.1      | 1/(m/s)           |
| `KP_BRAKE`     | 0.2      | 1/(m/s)           |
| `KI_LON`       | 0.05     | 1/((m/s)*s)       |
| `MAX_STEER_RAD`| 0.349    | rad (= 20 deg)    |
| `MAX_STEER_RATE_RAD_S` | 10.0 | rad/s         |
| `I_LAT_MAX`    | 5.0      | m*s (anti-windup) |
| `I_LON_MAX`    | 50.0     | (m/s)*s (anti-windup) |

The spec defaults oscillate at high speed (the law has no `1/v` softening so
`Kp_lat * e_lat` produces a large steer command at v=65 m/s for any
meaningful e_lat). The diagnostic sweep below used `PI_KP_LAT=0.003`,
`PI_KI_LAT=0.001` to push the failure point as late as possible without
high-speed instability. These are **not** committed as new defaults -- the
spec is explicit that we keep the default gains documented and not
tune-to-pass.

## Diagnostic-run results

Tomas / Sprint A / `--model slip --controller pi`. All runs aborted with
`OffTrackError` (chassis > 12 m from line); no run completed a lap.
Lap-time column shows the abort distance `s` and abort time `t`.

### Default gains (Kp_lat=0.05, Ki_lat=0.01)

| Chicane mult | Abort s | Abort t | util_p85 | Notes                              |
|---           |---      |---      |---       |---                                 |
| 0.80         | 574 m   | 9.36 s  | 3.07     | High-speed cross-track oscillation |

### Tuned gains (Kp_lat=0.003, Ki_lat=0.001)

| Chicane mult | Skill | Abort s | Abort t  | util_p85 | Notes                                  |
|---           |---    |---      |---       |---       |---                                     |
| 0.80         | 1.0   | 638 m   | 10.60 s  | 0.138    | Chicane apex; tyre nowhere near peak   |
| 0.60         | 1.0   | 638 m   | 10.66 s  | 0.135    | Identical failure point                |
| 1.00         | 1.0   | 638 m   | 10.56 s  | 0.140    | Identical failure point                |
| 0.40         | 1.0   | 638 m   | 10.72 s  | 0.132    | Identical                              |
| 0.30         | 1.0   | 638 m   | 10.74 s  | 0.131    | Identical -- chicane-cap not the issue |
| 0.80         | 0.01  | 638 m   | 10.60 s  | 0.274    | slip_target lower so util_p85 higher   |
| 0.80         | 0.50  | 638 m   | 10.60 s  | 0.184    | Same failure mode                      |

At the abort point: `e_lat = 11.9 m`, `max |steer_rate| = 0.07 rad/s` (far
below the 10 rad/s limit -- the controller is **not** trying hard enough; it
literally can't, the gain is too low to overcome the chicane geometry).

## Verdict on the physics envelope

**Inconclusive (PI is too dumb to chicane-turn even when not tyre-limited).**

The PI controller fails at the same point (s = 638 m, the Sprint A chicane
apex) regardless of approach speed (`--chicane-safety-mult` from 0.30 to
1.0). At the abort, `util_p85 = 0.13` -- the tyre is at 13% of its slip
envelope. This rules out a "tyre saturating" failure mode and rules out the
chicane-safety cap as the cause. But it does **not** prove the physics
envelope is infeasible: a pure PI without `1/v` softening or preview cannot
turn fast enough through a chicane regardless of grip available, because
`Kp_lat * e_lat` produces destabilising steer at high speed even when the
controller can in principle "see" the corner.

To make the PI baseline conclusive, the controller would need at minimum
**either** a `1/v`-softened steer law (Stanley-style) **or** a preview
target (look-ahead point projection). Adding either of those changes makes
the controller no longer "pure PI", which is the entire point of this
diagnostic. So the test is inconclusive **by design** -- the same simplicity
that makes PI a useful reference also makes it unable to discriminate
between "no preview" and "no grip" failure modes at a sharp chicane.

**What this tells us about the MPC.** The MPC at the same chicane reaches
slip-saturation around s = 647 m -- 9 m past the PI's failure. The PI's
failure mode (no preview -> over-steer at speed) is **strictly worse** than
the MPC's, which is internally consistent: the MPC's anticipatory control is
doing real work, just not enough. We cannot conclude the physics is
feasible. We can conclude that the MPC's failure is **not because the
physics envelope is wildly wrong** -- the PI's util_p85 = 0.13 says the
fitted Pacejka has plenty of headroom in cornering force at the chicane
speeds the plan asks for.

## File inventory

- **`src/lap_estimator/dynamics/pi_controller.py`** (new) -- `PIController`
  class + module-level gain constants + env-var override helper.
- **`src/lap_estimator/dynamics/slip_simulator.py`** (modified) --
  `_make_controller` now dispatches on `controller='pi'`; the slip-target
  publish guard in `_run_single` (line ~328) admits `PIController`.
- **`lap.py`** (modified) -- `--controller` argparse choice extended to
  `('reactive', 'mpc', 'pi')`; help text updated.

## Integration with neighbouring features

- **Plan source.** PI consumes the v3 DP `LongitudinalPlan` via
  `plan.distances` and `plan.speeds`. Same plan the MPC uses. Honors
  `--plan-source` and `--chicane-safety-mult`/`--chicane-radius-thresh`
  exactly as before.
- **Util_p85.** `_run_single` reads `ctrl.slip_target_rad` to compute the
  spec-§11.55 util_p85 metric; PI publishes the same field so the
  diagnostic line works.
- **Reactive / MPC controllers untouched.** PI lives next to them, not
  through them. No edits to `driver_controller.py` or `mpc_controller.py`.
- **Diagnostic CSV.** `.tmp/pi_diag_sprint_a.csv` is gitignored
  (`.gitignore:215`). Hard-coded path in `_make_controller` -- override by
  editing `slip_simulator.py:_make_controller` if you want a different
  location.

## Future work

- The natural next step is **NOT** to keep tuning PI. Tuning PI to pass the
  chicane defeats the purpose (a controller that passes only because we hand-
  tuned it isn't a clean reference baseline). The clean next step is one of:
  - **PI + 1/v softening** (a "Stanley-without-preview" controller) -- still
    simple, would tell us if the failure is preview-specific.
  - **Pacejka refit on Tomas's flying-lap data** (skip the chicane-laden
    Sprint A) -- if util_p85 stays at 0.13 with the refit, the bottleneck is
    confirmed control-side.
  - **Try Sprint B** (no equivalent chicane) -- if PI completes Sprint B at
    any gain, the chicane is a specific control-geometry obstacle, not a
    fundamental physics gap.
