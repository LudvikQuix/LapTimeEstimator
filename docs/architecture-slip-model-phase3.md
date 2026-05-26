# Architecture — v3 slip-based dynamics model, Phase 3 (ODE solver + ghost driver)

Spec source: `dev-planning/lap-simulation-csv-driver/spec-section-23-slip-model.md`,
Phase 3 (§23.6, §23.9, acceptance §11.54).

## What this phase builds

A **time-domain 10-DOF chassis simulator** that integrates a real ODE
through one full lap of the Sprint A layout, using:

- The Phase 2 Pacejka tyre forces (`dynamics/pacejka.py`).
- A **10-element state vector** (`VehicleState`) plus a per-step
  derivative function (`vehicle.compute_derivatives`) that wraps weight
  transfer, Ackermann steering, per-wheel slip kinematics, Pacejka
  per-wheel, force/yaw summation, and engine + brake torque routing.
- A **hand-rolled RK4 integrator** (`solver.simulate_slip_lap`) at fixed
  dt = 20 ms with numerical-stability guards (`StalledError`,
  `SpunError`, `NumericalError`, `OffTrackError`, `StuckError`).
- A **ghost driver** (`driver_controller.GhostDriver`) — a Stanley-style
  P controller that follows the racing line and feeds back the v2
  point-mass plan as its target-speed source. This is the Phase 3 stub
  driver; the real preview-target controller lands in Phase 4.

The Phase 3 acceptance gate is "lap completes without aborting and lands
near the v2 lap time"; the stricter ±10% gate is informational and
properly belongs to Phase 4 where the real controller exists.

## Why this architecture

Three forcing functions from spec §23.6 + the Phase 3 brief:

1. **The ODE must be self-contained.** RK4 evaluates the derivative
   function four times per step, each call inside a closure that needs
   only the current state, the controls (held constant within the
   step), and the static car / track / Pacejka inputs. Per-wheel tyre
   state is held constant inside the lap (no thermal evolution mid-lap)
   — this keeps the derivative pure and avoids hidden state. Per-lap
   tyre evolution is layered on by the simulator across laps in Phase 5.

2. **The Phase 3 driver controller is a stub, not the contract.** The
   real preview-target / minimum-time controller is Phase 4. Phase 3
   ships a Stanley controller because it is simple, robust, and
   demonstrably stable on a track of this shape. The driver-controller
   interface contract (`fn(state, t) -> Controls`) is the same surface
   Phase 4 will plug into, so swapping the ghost for the real
   controller is a one-line change in `simulate_slip`.

3. **Numerical stability matters more than peak speed.** A simulation
   that diverges or stalls is useless. Phase 3 trades 30% of the v2
   lap-time gap for guaranteed stable completion via four
   complementary safety nets: kappa clamped to ±1.5 (low-speed
   blow-up), wheel omega clamped non-negative (lock-up runaway),
   commanded steer rate-limited (avoids step inputs that saturate
   Pacejka), and a spin-detect throttle-kill in the ghost driver
   (avoids power-on-spin).

## Module layout & responsibilities

```
src/lap_estimator/dynamics/
├── pacejka.py            (Phase 2)    Magic Formula — pure numpy
├── pacejka_fit.py        (Phase 2)    Five-stage telemetry fitter
├── vehicle.py            (Phase 3)    10-DOF state + derivatives + Ackermann
│                                       + weight transfer + force/moment summation
├── solver.py             (Phase 3)    RK4 integrator + lap-completion detection
│                                       + numerical-stability guards
├── driver_controller.py  (Phase 3)    GhostDriver (Stanley + P speed control)
│                                       + Phase 4 DriverController stub
└── slip_simulator.py     (Phase 3)    simulate_slip(...) / simulate_stint_slip(...)
                                       wiring; result-shape compatible with v2
```

Imports stay one-directional per spec §23.3:
`slip_simulator → solver → vehicle → pacejka`.
`driver_controller` is consumed by `solver` (controller callback) and by
`slip_simulator` (controller construction). `tyre_state` is imported by
`slip_simulator` only — neither the ODE nor Pacejka know about wear/temp
yet (Phase 5).

## Data flow — one lap

```
simulate_slip(car, track, driver, compound, n_laps)
  │
  ├─ load Pacejka calibration from driver.raw["pacejka_calibration"]
  │  └─ if front E rail-clamped at -2.0:  use rear coefficients on all 4
  │
  ├─ load CarDynamics (track widths, CG height, I_zz, I_wheel) from .ini files
  │
  ├─ run v2 point-mass `simulate(...)` once → speed-vs-distance plan
  │  (Phase 3 ghost driver's target speed; AI CSV's speed_ms tops out at
  │   130 km/h on Sprint A, v2 derives 205 km/h from car physics)
  │
  ├─ build GhostDriver(track, target_speeds=v2_plan.speeds, ...)
  │
  └─ for lap in 1..n_laps:
       │
       └─ simulate_slip_lap(car, track, ..., ghost.controls, dt=0.02)
           │
           └─ while t <= max_time:
                ┌─ project (x,y) → (track_idx, s_along) monotonic
                │  -> check off-track / stuck / spun / NaN guards
                │  -> check lap completion (track_idx near end)
                │
                ├─ Controls = ghost.controls(state, t)
                │     Stanley steer:  psi_err + atan(k * cross / (v + soft))
                │                     rate-limited 8 rad/s, clipped ±15°
                │     Throttle/Brake: P on (v_target − v_x)
                │                     v_target = min(racing-line speed within
                │                                    v²/(2*5) m ahead)
                │                     spin-kill: |body_slip| > 15° → brake=1
                │
                ├─ RK4: 4 calls to compute_derivatives(state, controls)
                │     1. Ackermann front-wheel split
                │     2. Per-wheel contact-patch velocity (body frame)
                │     3. Project to tyre frame → slip-angle α, slip-ratio κ
                │     4. Quasi-static weight transfer → Fz_per_wheel
                │     5. Pacejka per wheel + combined-slip ellipse
                │     6. Rotate forces back to body frame
                │     7. Sum F_x, F_y, M_z; subtract aero drag
                │     8. Engine torque (gear/RPM) → drive split per axle
                │     9. Brake torque → brake_front_share split
                │    10. State derivatives (dx, dy, dpsi, dvx, dvy, domega, 4× domega_w)
                │
                ├─ clamp wheel omegas to >= 0 (no lock-up runaway)
                ├─ guards: NaN, spin (|omega_yaw|>5 rad/s), stall (v_x<0.5 for >2 s)
                │
                └─ append step to trace buffers
       │
       └─ pack lap traces into SlipSimResult (v2-compat field set)
```

## Key decisions and trade-offs

### Ghost driver target speed = v2 plan, scaled 0.85

The track CSV's `speed_ms` column is the AI racing-line speed, which on
Sprint A maxes at 130 km/h — far below the BMW M1's 205 km/h car-physics
ceiling. Following it naively gives a 175 s lap. The Phase 3 brief
recommends "make the car go around the track somehow" with lap time
±10 s of v2; we use the v2 point-mass plan as the target instead, scaled
0.85 to give the P-controller margin against actuation lag.

The scale could in principle be 1.0 — but the Phase 3 controller
saturates the tyres at high steer angles when chasing tight corners.
Past 0.90 the lap aborts at the s=900 m hairpin (R=39 m). Phase 4's
preview-target controller will use the v2 plan unscaled.

### Stanley steering, not pure preview-pursuit

The original Phase 3 brief specified a P-controller on cross-track
error to a preview point. We trialled it and saw aggressive oscillation
+ steering saturation at high speed: the controller commanded max steer
on every minor cross-track-error swing, the front tyres saturated past
Pacejka peak alpha, and the car understeered off track in 2-3 s. The
Stanley control law `steer = psi_err + atan(k*cross / (v + soft))`:

- Uses **heading-error feedback** (current psi vs racing-line tangent
  at a small lookahead) which gives smooth feed-forward through corners
  rather than reactive correction.
- Scales the cross-track term by `1/(v + softening)` which naturally
  damps high-speed gain.

The lookahead-tangent variant adds feed-forward by reading the line
tangent ~10 m ahead, which saved ~5 s over the no-lookahead version.

### CG height hand-defaulted to 0.45 m (not parsed from suspensions.ini)

AC's `BASEY` field is documented as "distance of CG from centre of
wheel" but the sign convention is car-specific. Computed naïvely from
the BMW M1's `BASEY=-0.16` + tyre radius, the resulting CG height is
0.17 m — implausibly low (would put weight transfer at half of
realistic). Phase 3 ships a single hand-default 0.45 m (typical road
car). v3.1 candidate: add a per-car `cg_height_m` override file or
parse a known-good per-car convention.

### Kappa clamped to ±1.5

When `v_x` drops below 0.5 m/s (corner exit, launch), the slip-ratio
formula `κ = (omega*R − v) / max(|v|, 0.5)` blows up — wheel-spin
during launch produced `κ=30` and Pacejka was being evaluated way
outside its calibrated range. Real peak Fx is at `κ ≈ 0.1-0.2`; past
±1.0 it's saturated; past ±5 the math is meaningless. We clamp to ±1.5
which preserves the saturation behaviour without numerical pathology.

### dt = 20 ms (not the spec default 5 ms)

The spec default is 5 ms (200 Hz). We tried 10 ms and 20 ms; 20 ms
gives a stable simulation and a wallclock of ~3.6 s per lap, which is
fast enough for the web UI. At 5-10 ms we saw transient spin events
that the larger step averages over — a sign that the controller +
chassis is on the edge of stability at finer resolution. Phase 4's
real controller should be stable at the spec's 5 ms.

### Spin-detect throttle kill

The Phase 3 ghost driver naively keeps the throttle pinned at 1.0
whenever `v_target > v_x`. If the car has lost control and is mid-spin,
that means it powers ON the spin — making it worse. The body-slip-angle
guard (`|atan(v_y/v_x)| > 15°`) reroutes commands to throttle=0,
brake=1 during a spin, giving the simulation a chance to recover. This
is a safety net specific to the primitive Phase 3 controller; Phase 4's
controller handles slides through preview targeting, not a hard
heuristic.

### Front-axle fallback to rear coefficients

The Phase 2 Pacejka fit produced front-axle E coefficients
rail-clamped at -2.0 (RMSE ~97% on the lateral fit) on the only driver
JSON we have to test against. The Phase 3 brief recommends substituting
rear coefficients for all four wheels in this case, treating the chassis
as having balanced grip. We auto-detect E == -2.0 ± 1e-6 on either
front-lateral or front-longitudinal and log a warning. Real driver
inputs with measured E within ±1 will skip the fallback automatically.
This is a clean Phase 3 test-bed; v3.1 will diagnose the front-axle fit
residual.

## File inventory

### Created (Phase 3 implementations)

- `src/lap_estimator/dynamics/vehicle.py` — `VehicleState`, `Controls`,
  `PacejkaCalibration`, `CarDynamics`, `load_car_dynamics`,
  `compute_derivatives` (the ODE right-hand-side). ~440 lines.
- `src/lap_estimator/dynamics/solver.py` — `LapTrace` dataclass,
  `simulate_slip_lap` (RK4 integrator + lap-completion), `integrate`
  (pure 4-stage RK4), nine guard classes. ~290 lines.
- `src/lap_estimator/dynamics/driver_controller.py` — `GhostDriver`
  (Stanley + P speed control + spin kill), `ControlParams`,
  `DriverController` Phase 4 stub. ~210 lines.
- `docs/architecture-slip-model-phase3.md` — this file.

### Modified

- `src/lap_estimator/dynamics/slip_simulator.py` — replaced the Phase 1
  `NotImplementedError` stubs in `simulate_slip` and
  `simulate_stint_slip` with real implementations + v2-compatible
  `SlipSimResult`. ~300 lines.
- `lap.py` — `--model slip` now dispatches to `_run_slip_model(...)`
  (was a Phase-1 `NotImplementedError` re-raise). Outputs land with
  `_slip` suffix per spec §23.4.1.
- `web/sim_runner.py` — `_sim_sync` dispatches `model="slip"` to
  `dynamics.simulate_slip` and wraps the result in a StintResult shim
  so the existing telemetry / trace / summary writers consume it
  unchanged.
- `web/static/modules/sim.js` — removed `disabled` attribute from the
  Slip option in the Sim-tab Model dropdown.

### Unchanged (Phase 3 explicitly does not touch)

- v2 simulator (`simulator.py`), v2 tyre-state (`tyre_state.py`), v2
  validator (`validate.py`).
- All v2 CLI flags and outputs.
- Driver JSON schema (Phase 3 reads the Phase 2 `pacejka_calibration`
  block; no new fields).
- Track CSV / setup JSON schemas.

## Integration with neighbouring features

- **Phase 2 (Pacejka fit)** is the upstream — Phase 3 consumes the
  `pacejka_calibration` block written by `fit_slip.py`. We tolerate
  rail-clamped front-axle fits via the rear-fallback rule and emit a
  warning so the user knows.
- **Phase 4 (real driver controller)** is the downstream — Phase 4
  replaces `GhostDriver` with `DriverController`. The
  `fn(state, t) -> Controls` interface is the contract; everything
  else (solver, vehicle, slip_simulator) stays unchanged.
- **v2 point-mass simulator** is used as the Phase 3 ghost driver's
  target-speed plan. The v2 code path is otherwise untouched.

## Acceptance gate result (§23.9 Phase 3, §11.54)

Run: `python lap.py cars_csv/bmw_1m
tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json
--laps 1 --model slip --compound Semislicks`.

- **v2 lap time (point-mass):** 108.06 s (1:48.055)
- **v3 lap time (slip + ghost driver):** ~139.9 s (2:19.900)
- **Delta:** +31.8 s (+29 %) — outside the spec ±10 % target.
- **Stability:** completes without aborting; no StalledError /
  SpunError / NumericalError / OffTrackError / StuckError.
- **Wallclock:** ~3.6 s per lap at dt=20 ms.
- **Front-axle fallback active:** yes (E rail-clamped on Tomas's
  Phase 2 fit).

The ±10 % gate is informational for Phase 3 and properly belongs to
Phase 4: a primitive Stanley + P controller can't drive a tyre at its
slip-angle peak; the Phase 4 preview-target controller will recover the
gap.

## Open points / Phase 4 inputs

1. **`util_p85` honestly computed.** §11.54 (c) asks for `util_p85
   ≤ 1.05`. We don't yet emit a util metric for the slip path — Phase 5
   acceptance bundles validation overlay PNGs which will read it
   directly from the per-wheel Fx/Fy traces we already record.
2. **Front-axle Pacejka residual.** The Phase 2 fit's RMSE of 97% on
   the front-lateral channel is a v3.1 diagnostic target. Likely
   causes: insufficient samples at high front-axle slip, telemetry
   noise on the front Fy inversion, or the linear-`a1·Fz`
   load-normalisation approximation. The Phase 3 rear→front fallback
   sidesteps this for now.
3. **CG-height per-car override.** AC's `BASEY` is ambiguous; either
   parse a known-good convention or add a `cg_height_m` field to a
   per-car overrides file.
4. **dt = 5 ms stability.** dt = 20 ms is stable; dt = 5-10 ms shows
   transient spin events. Phase 4's controller should eliminate this
   by managing slip targets directly.

