# Architecture — v3 slip-based dynamics model, Phase 1 (skeleton + dispatcher)

Spec source: `dev-planning/lap-simulation-csv-driver/spec-section-23-slip-model.md`,
Phase 1 (§23.9 Phase 1, acceptance §11.50–§11.51).

## What this phase builds

A **parallel-track v3 simulator scaffolding** that the CLI and the web UI can
already dispatch to, but whose physics is unimplemented. Every public entry
point in the new `src/lap_estimator/dynamics/` package raises
`NotImplementedError("v3 phase 1: dynamics module skeleton in place")`. The
v2 point-mass model (`src/lap_estimator/simulator.py`) is **byte-for-byte
unchanged** — Phase 1's regression-safety acceptance gate (§11.50) verifies
that `python lap.py ... --model point-mass` produces output identical to
omitting the flag.

Phase 1 deliberately ships dead-on-arrival code so Phases 2–5 can land
incrementally without churning the public surface. The dataclass shapes
(`VehicleState`, `Controls`, `ControlParams`, `PacejkaCoeffs`,
`SlipSimResult`) and the function signatures (`simulate_slip`,
`simulate_stint_slip`, `integrate_lap`, `compute_derivatives`,
`pacejka_fy/fx`, `combined_friction_ellipse`, `fit_pacejka_from_lake`,
`DriverController.step`) are part of the Phase 2+ contract and will not
change.

## Why this architecture

Three forcing functions from spec §23.1–§23.3:

1. **Parallel-track posture.** v2 stays the default; v3 is opt-in. A
   sibling package (`dynamics/`) keeps imports one-directional (v3 imports
   v2 utilities such as `Car`, `Track`, `Driver`, `tyre_state`, never the
   reverse). `simulator.py` gains one header line and nothing else.
2. **Dispatcher first, physics later.** The CLI and web UI need a
   selector wired now so downstream UX work (FrontEndEsthetic, Buddy's
   Phase 5 polish, NitpickerCustomer's exploration runs) doesn't block on
   the ODE integrator. Phase 1 ships the dispatcher and a clean
   single-line error from the slip branch — no Python traceback —
   exiting code 2 to keep CI behaviour explicit.
3. **Soft 500-line file ceiling.** Spec §23.3 partitions the package
   along clean seams (Pacejka math vs vehicle state vs solver vs
   controller vs fitter vs top-level orchestration). Every Phase 1 file
   is well under the ceiling. If `vehicle.py` or `pacejka_fit.py` grow
   past the ceiling during Phase 2/3 build-out, the suggested splits are
   recorded in §23.3 and in the file-level docstrings.

## Module layout

```
src/lap_estimator/dynamics/
  __init__.py            re-exports simulate_slip, simulate_stint_slip, SlipSimResult
  pacejka.py             Magic Formula tyre force (Fy, Fx, combined-slip ellipse)
  vehicle.py             VehicleState (10 DOF), Controls, compute_derivatives
  solver.py              RK4 + LSODA integrate_lap; StalledError / SpunError / NumericalError
  driver_controller.py   ControlParams, DriverController (preview-target P-controllers)
  pacejka_fit.py         fit_pacejka_from_lake (5-stage algorithm, §23.7.2)
  slip_simulator.py      simulate_slip, simulate_stint_slip, SlipSimResult dataclass
```

Imports stay one-directional per spec §23.3:

```
slip_simulator -> solver -> vehicle -> pacejka
                     |
                     +-> driver_controller -> vehicle
slip_simulator -> tyre_state (v2 module, reused unchanged)
```

`pacejka_fit` is a leaf (called from `fit_driver.py` in Phase 2; standalone
otherwise).

## CLI dispatch (`lap.py`)

The argparse surface gains one flag:

```
--model {point-mass, slip}   default: point-mass
```

`point-mass` is a no-op — control falls through to the existing
`simulate_stint` / `simulate` code path unchanged. `slip` is intercepted
**before** any of the v2 code runs:

```python
if args.model == "slip":
    from lap_estimator.dynamics import simulate_slip
    try:
        simulate_slip(None, None, None)
    except NotImplementedError as e:
        print(f"slip model not yet implemented - {e}. "
              "Pass --model point-mass to use the working kinematic model.",
              file=sys.stderr)
        sys.exit(2)
    return
```

This shape is deliberate: catching `NotImplementedError` from the entry
point itself (rather than re-checking a string flag downstream) means
Phases 3+ replace `raise NotImplementedError(...)` with a real call,
and `lap.py` just keeps working — the catch becomes harmless dead code
(`NotImplementedError` will no longer fire) until removal at Phase 3
merge. Phase 3 will rewrite this block to thread `car`/`track`/`driver`
arguments through; the wire format above is intentionally minimal.

The ASCII hyphen in the error string (rather than the em-dash the spec
shows) sidesteps cp1252/UTF-8 console encoding mojibake on Windows
terminals — spec §11.51 explicitly allows "or similar clean message".

## Web UI dispatch (`web/static/modules/sim.js`, `web/sim_routes.py`)

The Sim tab gains a `<select id="sim-model">` next to the Compound dropdown
with two options:

- `point-mass` (default, `selected`).
- `slip` with `disabled` + `title="Coming soon — Phase 1 scaffolding only."`.

The selected value is plumbed into the `/api/sim` POST body as field
`model`. `web/sim_runner._sim_sync` reads the field, validates it is one
of `{"point-mass", "slip"}` (rejecting unknown values with a 400 via
`HTTPException`), and otherwise ignores it. Phase 1 therefore always runs
point-mass; Phase 5 will branch on this field to dispatch to
`simulate_slip` / `simulate_stint_slip`.

The slip option is `disabled` in HTML so the UI is visibly aware that v3
exists, but the user cannot accidentally select it before the physics
lands. This matches the spec §23.9 Phase 1 acceptance condition.

## Files created

- `src/lap_estimator/dynamics/__init__.py`
- `src/lap_estimator/dynamics/pacejka.py`
- `src/lap_estimator/dynamics/vehicle.py`
- `src/lap_estimator/dynamics/solver.py`
- `src/lap_estimator/dynamics/driver_controller.py`
- `src/lap_estimator/dynamics/pacejka_fit.py`
- `src/lap_estimator/dynamics/slip_simulator.py`
- `docs/architecture-slip-model-phase1.md` (this file)

## Files modified

- `lap.py` — adds `--model` argparse flag + slip-branch dispatcher with
  clean-error wrapping (5 lines of new code + 6 lines of help text).
- `src/lap_estimator/simulator.py` — one new docstring header line
  pointing at the v3 sibling package. No behavioural change.
- `web/static/modules/sim.js` — `<select id="sim-model">` in the form
  grid; `model` plumbed into the POST body.
- `web/sim_runner.py` — reads + validates the `model` field; ignores it
  for Phase 1 (always runs point-mass).

## Integration with neighbouring features

- **v2 point-mass simulator (`docs/architecture-lap-simulation-stint-v2.md`).**
  Untouched. Phase 1 is strictly additive at the dispatcher layer.
  `simulator.py`'s docstring picks up one header line; all functions,
  classes, side effects, and output filenames stay identical.
- **v1.3 asymmetric-pressure stint (`docs/architecture-lap-simulation-stint-v1_3-asymmetric-pressure.md`).**
  Phase 3+ will reuse `tyre_state.py` (the per-wheel state machinery
  that doc describes) verbatim, feeding it Pacejka-derived per-wheel
  slip energy instead of the v2 `v^2/R` proxy. Phase 1 has no
  interaction.
- **Web UI (`docs/architecture-web-ui.md`).** Phase 1 adds one form
  control on the Sim tab; the rest of the UI architecture is unchanged.
  Phase 5 will wire the Driver-tab "Slip model parameters" panel and
  enable the slip option in the dropdown.
- **Config pipeline (`docs/architecture-config-pipeline.md`).** No
  changes in Phase 1. Phase 2 will add the `pacejka_calibration` block
  to driver JSON (sibling of `tyre_calibration`); Phase 1 doesn't read
  or write driver JSON.

## Acceptance gates verified

- **§11.50 (point-mass byte-identical).** `python lap.py cars_csv/bmw_1m
  tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json
  --laps 2 --no-plot` produces identical stdout and identical
  `*_sim_trace.csv`, `*_sim_telemetry.csv`, `*_stint_summary.csv`
  regardless of whether `--model point-mass` is passed. Verified via
  `diff -q` on all four artefacts.
- **§11.51 (slip clean error).** `python lap.py ... --model slip` exits
  with code 2 and prints to stderr:
  `slip model not yet implemented - v3 phase 1: dynamics module
  skeleton in place. Pass --model point-mass to use the working
  kinematic model.` No Python traceback.

## What Phase 2 will do

Phase 2 implements `pacejka.py` (the three force functions) and
`pacejka_fit.py` (the five-stage fit algorithm), and wires
`--fit-pacejka` into `fit_driver.py`. The Phase 2 acceptance gate
(§11.52, §11.53) is a calibration-block writer that passes Stage E
cross-validation on Tomas's five lake laps. None of Phase 1's signatures
need to change; only the `NotImplementedError` bodies are filled in.
