# Architecture — FF+PI baseline (`--controller ffpi`)

## What it does

`FFPIController` is a feedforward-dominant + bounded-PI controller for the v3
slip-based simulator. It precomputes three feedforward arrays from the track
geometry and the DP plan, looks them up at each tick by arclength, and lets a
small PI trim correction handle the residual cross-track + speed error. PI
delta is clipped to ±30 % of FF magnitude (with a small absolute floor) so the
PI can never fight the plan beyond a bounded envelope.

This controller is a *diagnostic baseline*: the goal is to prove (more
decisively than the deliberately-dumb cascade PI) whether the physics envelope
can be exploited by a competent-but-simple controller.

## Why this architecture

Classic racing-control pattern. The FF channel does the bulk of the work
because the DP plan is already a feasibility-respecting reference; the PI only
needs to absorb model error, the drag-comp residual, and small steady-state
cross-track drift. By bounding PI to a fraction of FF, the controller can
never exceed the plan's envelope by more than 30 % — which makes the
diagnostic clean: if it fails, the failure is *not* a runaway PI; it's a
genuine FF or envelope issue.

Trade-offs:

* **Self-contained.** No coupling to slip / Pacejka / friction-ellipse beyond
  what's already baked into the DP plan. PI doesn't see slip.
* **Precomputed FF tables (1D in arclength).** ~700 samples on Sprint A, cheap
  ``np.interp``-style lookup per tick (~50 Hz).
* **Bounded PI.** Anti-windup integrators clamp on saturation. PI delta is
  clipped to ``max(0.3 * |FF|, floor)``.
* **No state-dependent FF refresh.** The throttle FF inverse uses the
  *planned* speed at each sample, not the actual state speed -- which means
  when the truth state drifts off plan, FF doesn't compensate; PI does (within
  its envelope). This is by design for diagnostic clarity.

## Components

### `FFPIController` (`src/lap_estimator/dynamics/ff_pi_controller.py`)

Public API mirrors `DriverController` / `MPCController` / `PIController`:

* `__init__(driver, track, car, *, plan, rng_seed, slip_target_rad_override,
  log_csv_path)` -- precomputes the three FF arrays and opens the diagnostic
  CSV.
* `.controls(state, t, track=None) -> Controls` -- one tick of control:
  cross-track + speed error → bounded PI trim added to FF lookup.
* `.slip_target_rad` -- carried for `util_p85` plumbing only; the controller
  does NOT modulate on slip.
* `.close()` -- flush diagnostic CSV.

### `_compute_steer_ff(L)`

Geometric Ackermann FF: ``delta_FF(s) = arctan(L * kappa_signed(s))``.

* `L` = wheelbase, read from `car.wheelbase` (BMW 1M ≈ 2.66 m).
* `kappa_signed` = derivative of the unwrapped centreline tangent
  `psi_line = arctan2(dz, dx)`. The track CSV's ``radius_m`` magnitude is
  used as a cap (so genuinely sharp corners aren't over-driven by
  tangent-noise on the unwrap).
* Sign convention: positive `kappa_signed` matches the cross-track PI's
  "positive cross => positive corrective steer" so FF and PI reinforce
  rather than fight.
* Lightly smoothed (5-sample moving average on `psi_line`) to suppress
  per-sample CSV noise.

### `_compute_a_x_plan()`

`a_x_plan(s) = v_target(s) * dv_target/ds`. The DP plan is the source of
truth; we don't add any safety margin (the plan already includes
`safety_margin=0.94`).

### `_compute_lon_ff(a_x_plan)`

Physics-aware inverse model (a deliberate enhancement over the spec literal
`a_x / 0.4g` because that mapping under-commands the throttle at high speed --
the BMW 1M's full-throttle net accel is only ~0.05 g at 65 m/s):

* `F_req = m * a_x_plan + F_drag(v_plan) + F_rolling(v_plan)`.
* `throttle_FF = clip(F_req / F_traction_max(v_plan), 0, 1)` when
  `a_x_plan >= 0`, else 0.
* `brake_FF = clip(-a_x_plan / 1.1g, 0, 1)` when `a_x_plan < 0`, else 0.
* Drag is NOT discounted from `brake_FF` (drag aids braking; the PI absorbs
  the small residual).

### PI feedback loops

**Lateral (`Kp_lat=0.05, Ki_lat=0.01`):**

* `e_lat` = signed perpendicular cross-track to nearest line point
  (positive => right of line). Same convention as `DriverController` and
  `PIController`.
* `d_delta_raw = Kp_lat * e_lat + Ki_lat * integral`.
* Clipped to ``+- max(0.3 * |delta_FF(s)|, 0.02 rad)``.
* Integrator anti-windup bleeds back to whatever value would just-fail-to-
  saturate.
* Damped at standing start (`v < 5 m/s`).

**Longitudinal (`Kp_lon=0.20, Ki_lon=0.05`):**

* `e_lon = v_target(s) - v_actual`.
* `d_u_raw = Kp_lon * e_lon + Ki_lon * integral`.
* Clipped to ``+- max(0.3 * max(|throttle_FF|, |brake_FF|), 0.05)``.
* Crossover-aware: `net = throttle_FF - brake_FF + d_u`. Positive net =>
  throttle; negative net => brake. PI can cancel + cross the actuator
  boundary while still bounded.

**Hard limits**: `MAX_STEER_RAD = 20 deg`, `MAX_STEER_RATE_RAD_S = 10 rad/s`.

### Wiring

* `slip_simulator._make_controller`: new branch `if controller == "ffpi"`.
  CSV diagnostic always goes to ``.tmp/ffpi_diag_sprint_a.csv``.
* `slip_simulator._run_single`: `slip_target_rad` extraction now covers
  `FFPIController` for `util_p85`.
* `lap.py --controller`: choices extended to `{reactive, mpc, pi, ffpi}`.

### Env-var gain overrides

All four gains overridable for offline tuning:
* `FFPI_KP_LAT`, `FFPI_KI_LAT`, `FFPI_KP_LON`, `FFPI_KI_LON`.

## Data flow

```
Track CSV (xs, zs, distance, radius)  ----+
                                           |
                                           v
DP plan (v_target(s))  ----------> _compute_steer_ff (per-sample)  ---+
                                   _compute_a_x_plan                  |
                                   _compute_lon_ff (per-sample)  -----+
                                                                      v
VehicleState (x, y, v_x)  --->  controls()  --> nearest_index(s)
                                            --> e_lat = signed cross
                                            --> e_lon = v_target - v_x
                                            --> lookup delta_FF / t_FF / b_FF
                                            --> PI integrators + bounds
                                            --> Controls(steer, throttle, brake)
                                                + diagnostic CSV row
```

## File inventory

* `src/lap_estimator/dynamics/ff_pi_controller.py` -- new (~350 LoC). Class
  `FFPIController` + module-level constants + helpers.
* `src/lap_estimator/dynamics/slip_simulator.py` -- modified: import + add
  `ffpi` branch to `_make_controller`, extend `isinstance` check in
  `_run_single`.
* `lap.py` -- modified: extend `--controller` choices and help text.

## Integration with neighbouring features

* **DP planner** (`longitudinal_planner.plan_longitudinal`): consumed
  unchanged. FF reads `plan.distances` / `plan.speeds` for `v_target` and
  derives `a_x_plan` from those.
* **Truth model** (`solver.simulate_slip_lap`): consumed unchanged. The
  controller is a black box from the simulator's perspective; it returns
  `Controls(steer_rad, throttle, brake)` per tick.
* **Pacejka calibration / slip target**: carried but unused. The `util_p85`
  metric is still meaningful as a diagnostic of how much slip the FF+PI's
  commands generated, even though the controller never reads slip.
* **Chicane-safety config**: consumed via the DP plan (the cap is applied
  in `plan_longitudinal`). FF+PI is sensitive to the cap only insofar as
  it changes `v_target`.

## Diagnostic CSV

Per-tick rows at `.tmp/ffpi_diag_sprint_a.csv` (always emitted; deterministic
single-file path per the dumb-PI convention):

```
t, s, v, v_target, e_lat, e_lon,
delta_ff, d_delta_pi, delta,
throttle_ff, brake_ff, d_u_pi, throttle, brake
```

## Smoke-test results (Tomas, skill=1.0, BMW 1M, default chicane mult 0.80)

| Track     | Outcome | s_abort | util_p85 | Notes                              |
| --------- | ------- | ------- | -------- | ---------------------------------- |
| Sprint A  | ABORT   | 301 m   |   6.19   | Pre-chicane straight, util massively over peak |
| Sprint B  | ABORT   | 458 m   |  11.16   | Pre-chicane straight                |
| GP A      | ABORT   | 281 m   |   2.69   | Pre-chicane straight                |
| GP B      | ABORT   | 339 m   |   5.39   | Pre-chicane straight                |

**Skill sweep, Sprint A:**

| Skill | Outcome | s_abort | util_p85 |
| ----- | ------- | ------- | -------- |
| 0.01  | ABORT   | 301 m   | 12.25    |
| 0.50  | ABORT   | 301 m   |  8.25    |
| 1.00  | ABORT   | 301 m   |  6.19    |

Lower skill => more slip-target noise => higher util. Monotonic, expected.

**Chicane-mult sweep, Sprint A:** all four (0.30 / 0.60 / 0.80 / 1.00) abort
at exactly s=301 m with identical util. FF+PI dies *before* the chicane, so
the chicane cap is irrelevant in this regime.

**Pure-FF reference (PI gains all zero):** Sprint A reaches s=628 m (the
chicane mouth) with util_p85=0.06 -- the FF alone is conservative within the
plan, but has no cross-track correction so it slowly drifts off-line and dies
at the first sharp corner.

## Verdict

**FF+PI does not complete Sprint A.** The PI destabilises the controller
because the throttle FF at cruise (~0.85 on the high-speed straight) leaves
no friction-circle headroom for the lateral PI trim: the PI's small steer
correction (within the ±0.02 rad floor) combined with cruise throttle
triggers rear-wheel slip; speed bleeds; PI sees a growing speed deficit;
throttle saturates at 1.0; rear spins harder. Util_p85 of 6-11 across all
tracks confirms massive over-peak slip well before any corner.

This is a clean diagnostic: the FF channel alone is well-behaved
(util=0.06 to chicane mouth), so the *physics envelope* is fine in the
straight-and-gentle-curve regime, **and** the previous PI's util=0.13 floor
was a false negative (the dumb PI was too dumb to even reach the envelope).
With a properly-loaded FF baseline, the friction-ellipse coupling between
the throttle and lateral channels is the binding constraint, NOT the
chicane geometry. A future v3 controller needs to either:

* Reduce the FF throttle preset on cornering-adjacent segments
  (friction-ellipse-aware FF), OR
* Couple the PI bound to a slip-headroom estimate so the lateral trim
  can't trigger rear oversteer.

The MPC's combined-slip-aware QP was attempting (a) and (b) jointly; the
data here suggests that level of coupling is the right minimum, not over-
engineering.

## Caveats

* The spec's literal throttle FF inverse `a_x_plan / 0.4g` was kept as
  `A_X_MAX_DRIVE = 0.4g` but the actual mapping was changed to physics-aware
  `F_req / F_traction_max(v_plan)` because the literal mapping commands
  ~0 throttle on cruise (the BMW 1M only delivers ~0.05 g net accel at
  65 m/s). The literal mapping is impossible to make work without
  a separate drag-comp PI which would defeat the bounded-PI envelope. This
  is a documented deviation; see `_compute_lon_ff` docstring.
* `delta_FF` uses the centreline tangent's derivative (with the CSV
  `radius_m` as a magnitude cap), not the CSV `radius_m` directly. The CSV
  uses `radius=2000 m` as a "straight" sentinel which masks real gentle
  curvature; without tangent-derived FF, the controller drifts off-line on
  the 0..540 m "straight" of Sprint A.
* Diagnostic CSV file path is hard-coded to `.tmp/ffpi_diag_sprint_a.csv`
  for parity with the dumb-PI baseline; the same file is overwritten per
  run regardless of which track was simulated.
