# BMW 1M powertrain calibration overrides

Date: 2026-05-23. Owner: ArchDev.

## What this change does

Adds two optional Car-construction overrides that let callers swap the BMW 1M
steady-state turbo boost and body drag coefficient without editing
`cars_csv/bmw_1m/*.ini`. The overrides surface as `--boost-steady FLOAT` and
`--cd-override FLOAT` on `lap.py`, and as the same flags on the open-loop
replay diagnostic at `.tmp/tomas_openloop_replay.py`.

The overrides exist because the v3 open-loop replay under-predicts Tomas's
recorded speed on Sprint A's opening straight by ~9 km/h at t=8 s. The
diagnostic at `.tmp/v3_powertrain_diagnostic.csv` traced this to a torque
shortfall on wide-open-throttle 3rd-gear pulls. The hypothesis is that the
ini's `[TURBO_0].WASTEGATE = 0.46` is too low a steady-state boost for the
torque-multiplier model the simulator uses (`T = T_base * (1 + boost)`); the
ini's `MAX_BOOST = 0.85` is the *peak* before the wastegate opens. The
overrides let us probe where the truth lies between those two without
forking the car data.

## Why two scalars and not a richer model

The simulator's powertrain is intentionally simple:

- `Car.engine_torque(rpm) = lerp(power.lut, rpm) * (1 + turbo_max_boost)`
- `Car.drag_force(v)     = 0.5 * rho * aero_cd * frontal_area * v**2`

Both `turbo_max_boost` and `aero_cd` are scalars, populated once at load
time. The AC LUTs add no behaviour we sample (`aero_cd` is read at AOA=0
only; the wastegate ramp-up dynamics are absent from our flat multiplier).
Replacing the scalar at load time is therefore lossless against the model
we have. Adding a richer turbo model (boost vs RPM, lag, etc.) is a separate
spec.

## Files changed

- `src/lap_estimator/car.py`
  - `Car.__init__` now accepts keyword-only `boost_steady_override` and
    `cd_override`, both defaulting to `None`.
  - `_load` applies the boost override after reading `WASTEGATE` and the
    Cd override before falling through to `_calc_aero_cd`.
  - `Car.from_dir` forwards both kwargs.
- `lap.py`
  - Adds `--boost-steady` and `--cd-override` CLI flags, threaded to both
    the point-mass and slip-model `Car(...)` constructions.
- `.tmp/tomas_openloop_replay.py`
  - Argparse with the same two flags plus `--tag` for output naming.
  - Appends a one-row summary to `.tmp/powertrain_calibration_sweep.csv`.
  - Reports `v @ t=8s` against Tomas's recorded log.

## Calibration sweep result (2026-05-23)

Open-loop replay of Tomas's Lap5 inputs through v3 dynamics on Sprint A.
Reference: Tomas v(t=8s) = 208.42 km/h; lap 107.56 s.

| tag             | boost | Cd    | v(t=8s) | dv vs Tomas | lap (s) | xtrk (m) |
|-----------------|-------|-------|---------|-------------|---------|----------|
| baseline        | 0.46  | 0.340 | 199.27  | -9.15       | 142.2*  | 2391     |
| boost055_cd032  | 0.55  | 0.320 | 203.02  | -5.40       |  88.9*  |  243     |
| boost060_cd032  | 0.60  | 0.320 | 204.55  | -3.87       |  DNF    | 9988     |
| boost085_cd032  | 0.85  | 0.320 | 212.08  | +3.66       |  DNF    | 4349     |

`*` Open-loop lap times are meaningless. Without feedback the car wanders
off the line; "lap time" here is only "time to project the closest point on
the centreline through the finish marker", which can be faster than reality
when the trajectory cuts geometry.

### Headline findings

1. **Source identified: `WASTEGATE`**. `car.py` line 236 reads
   `[TURBO_0].WASTEGATE = 0.46`, never `MAX_BOOST`. The 0.85 in the ini was
   never in the loop.
2. The straight-line speed scales monotonically with boost as expected.
   Between boost=0.60 and 0.85 (with Cd held at 0.32) v(t=8s) gains
   ~7.5 km/h, i.e. ~30 km/h per unit boost in this regime.
3. **`MAX_BOOST=0.85` over-predicts by only +3.7 km/h.** The engine LUT is
   NOT short; the v3 plant is short on *boost*, not on torque-curve shape.
4. **The right steady-state boost target sits at ~0.70-0.75 with Cd=0.32**,
   not 0.55. The recommended `boost_steady=0.55` only closes ~40 % of the
   gap (203 vs 208). `boost_steady=0.60` closes ~58 %.
5. Cd=0.32 vs the stock 0.34 is plausible (BMW spec is 0.34 publicly; the
   AC stock LUT matches). The sweep does not isolate Cd's individual
   contribution; that needs a separate boost-held probe if anyone wants to
   bisect the two knobs.

### Lap-time gap-closure claim

ArchDev's earlier "1-2 s of lap-time gap closure" estimate cannot be
verified by this test alone. The open-loop replay cannot speak to lap time
because cross-track drift dominates. To pin down lap-time impact we need a
closed-loop slip-model lap (`lap.py --model slip ... --boost-steady X
--cd-override 0.32`) and read the controller-stabilised lap time. That's a
follow-up.

## Recommendation

- **Do NOT bake a new default into the ini-loader.** The override should
  stay per-invocation. Reason: there is no AC-side ground truth for what
  steady-state boost belongs in the torque-multiplier model; 0.55, 0.70,
  and 0.85 are all defensible interpretations of the ini, and the right
  value is driver/lap dependent (telemetry-fit, not data-sheet).
- **Treat boost_steady as a per-driver calibrated parameter**, in the same
  league as the Pacejka calibration we already store in `drivers/*.json`.
  Either:
  - add it to the driver JSON schema (preferred — keeps everything
    co-located with the lap that calibrated it), or
  - add it to the setup JSON schema (less ideal — boost isn't a setup
    choice, it's a model-vs-reality patch).
- **Re-run a closed-loop slip lap on Sprint A with boost_steady=0.70,
  Cd=0.32** before committing a default. If that lap closes 1-2 s of the
  v3 vs Tomas gap, ship it as the BMW 1M's tuned profile. If it does not,
  the gap is elsewhere (controller, DP plan, Pacejka envelope) and the
  powertrain calibration shouldn't masquerade as the fix.

## How to use

```
# closed-loop slip lap with the recommended calibration
python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    drivers/tomas.json --model slip --controller mpc \
    --boost-steady 0.70 --cd-override 0.32

# open-loop diagnostic with same overrides
python .tmp/tomas_openloop_replay.py --tag boost070_cd032 \
    --boost-steady 0.70 --cd-override 0.32
```

Outputs land at `.tmp/tomas_openloop_replay_<tag>.csv|.png` and the running
sweep summary at `.tmp/powertrain_calibration_sweep.csv`.

## Integration points

- `Car` is the single entry point. Both the point-mass simulator
  (`simulator.simulate`) and the slip-model dispatch
  (`dynamics.vehicle._engine_torque_at_wheel`) consume the overridden
  `turbo_max_boost` / `aero_cd` automatically; no other layer needs to
  know.
- Driver JSON, setup JSON, and the Pacejka calibration files are
  untouched. The override does not mutate any tyre/grip parameter.
- `viz/` is not touched.
