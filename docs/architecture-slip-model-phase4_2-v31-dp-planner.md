# Architecture — v3.1 Phase 4.2: longitudinal DP planner

**Spec:** `dev-planning/lap-simulation-csv-driver/spec-section-23-10-v31-controller.md` §23.10.6
**Branch:** `feature/sc-71955/lap-simulation`
**Status:** Code shipped. Acceptance gate §11.55 **NOT MET** — lap still ~17 s slow vs real. Root cause: reactive P-controller cannot track even an honest plan. Spec §23.10.7 v3.2 (full MPC) is the planned remediation.

## What this ships

A single new module — `src/lap_estimator/dynamics/longitudinal_planner.py` — that builds a minimum-time speed-vs-distance reference plan against the *measured* Pacejka envelope, replacing the v2 friction-circle plan that Phase 4.1 was forced to chase. The slip-sim entry point gains a `plan_source` kwarg with values `v3_dp` (default) and `v2` (regression), surfaced as the `--plan-source` CLI flag on `lap.py`. The controller's lookup machinery is untouched — both plans produce a `(distances, speeds)` pair of identical shape.

## Why this architecture

Phase 4.1 (preview-longitudinal P-controller + α_peak slip target) reached the controller's limit. It cannot track the v2 friction-circle plan (`mu_v2 ≈ 1.20`) because the *measured* Pacejka tyre on Tomas+BMW has `D_lat ≈ 1.03` — the v2 plan is geometrically infeasible. Every skill=1.0 lap aborts with `OffTrackError` at s ≈ 890 m. The honest fix is to build a plan against the *actual* tyre envelope; the controller can then chase a feasible reference. This is the v2 3-pass algorithm applied to the Pacejka grip surface instead of v2's scalar `mu * g`.

### Decisions

1. **Static Fz, no weight transfer in the planner** (spec §23.10.6.2). The simulator handles transients; the plan is a reference. Adding WT would couple plan to controller in ways that delay convergence.
2. **Front-axle peaks drive the envelope.** Tomas's pooled fit gives front=rear so this is a no-op for him; for asymmetric fits the *binding* (lower) axle's `D_lat` is the correct ceiling. Code uses `calib.front.lateral.D` and `calib.front.longitudinal.D` directly.
3. **Friction ellipse with the fitted exponent.** `(a_x / (D_long · g))^n + (a_y / (D_lat · g))^n ≤ 1` with `n = calib.ellipse_exponent` (2.0 for the Tomas fit). This couples lateral demand to available longitudinal headroom at every point.
4. **Sample the existing CSV ideal-line grid.** ~700 (Sprint A reports 2325) samples at the CSV's native resolution. Re-using the controller's interp grid keeps lookup cheap and removes a possible re-sampling artefact.
5. **Documented `safety_margin = 0.97` default** — the *honest* analog of the deleted `target_scale` band-aid. 3% headroom for transient overshoot, configurable, justified.

## Data flow

```
                   ┌────────────────────────┐
                   │  track CSV ideal line  │
                   │  (distance_m, radius)  │
                   └───────────┬────────────┘
                               │
                               ▼
   pacejka_calibration ──► plan_longitudinal(car, track, calib)
   (D_lat, D_long, ε)       │
                            │  Pass 1: v_max[i] = sqrt(D_lat·g/|κ|)
                            │  Pass 2: backward brake-feasibility
                            │           (a_brake limited by ellipse @ v_max[i+1])
                            │  Pass 3: forward throttle-feasibility
                            │           (a_thr limited by ellipse @ v_plan[i])
                            │  Final:  speeds *= safety_margin
                            ▼
                  LongitudinalPlan(distances, speeds)
                            │
                            ▼
   slip_simulator._build_plan(source="v3_dp")
                            │
                            ▼
   DriverController(target_speed_ds=plan.distances,
                    target_speeds=plan.speeds)
```

The dispatcher `_build_plan` in `slip_simulator.py` routes on the `plan_source` arg: `v3_dp` calls `_build_dp_plan` (new helper), `v2` calls the existing `_build_target_speed_plan` (legacy v2 point-mass). Anything else raises.

## File inventory

| File | Action | Lines | Why |
|---|---|---|---|
| `src/lap_estimator/dynamics/longitudinal_planner.py` | new | ~210 | The planner module. Public surface: `plan_longitudinal()` returning `LongitudinalPlan`. |
| `src/lap_estimator/dynamics/slip_simulator.py` | modified | + `plan_source` kwarg, `_build_plan` / `_build_dp_plan` helpers, `v2_plan` -> `plan` rename throughout | Wires the DP plan into the controller dispatch. |
| `lap.py` | modified | + `--plan-source` CLI flag (default `v3_dp`) | User-facing toggle. `--model point-mass` ignores it. |

The longitudinal_planner module is well under the 500-line soft cap and self-contained — no imports from `slip_simulator` or `solver`, only `pacejka` types via `vehicle`.

## Integration with neighbouring features

- **Phase 4.1 (`architecture-slip-model-phase4_1-v31-controller.md`):** Phase 4.1's preview-longitudinal controller, α_peak slip target, and dropped `target_scale` are all preserved. The only change downstream of `_make_controller` is which `(distances, speeds)` pair is fed in.
- **v2 simulator (`simulator.py`):** Untouched. `--model point-mass` is byte-equivalent.
- **Pacejka fit pipeline (`pacejka_fit.py`):** Untouched. Existing `pacejka_calibration` blocks on driver JSONs work verbatim.
- **Tyre-state plumbing:** Plan uses **static Fz only** (spec defers WT to potential 4.3). The simulator's per-step WT is unaffected.
- **Web UI (§22):** Not modified. Spec §23.10.8 reserves an optional "Plan source" radio for a later UI pass.

## Acceptance gate — measured result

**Phase 4.2 does NOT pass §11.55.** Single-lap deterministic abort at `s≈936 m`. MC at default safety_margin=0.97: only 3/10 finish, median lap **2:04.14 = +16.6 s** vs real (1:47.56). Outside the ±5 s ship-anyway band by ~11 s.

| Safety margin | Finished (10 MC runs) | Median lap (s) | util_p85 | MC σ (s) |
|---|---:|---:|---:|---:|
| 0.97 (spec default) | 3 | 124.14 | 2.74 | 1.03 |
| 0.96 | 10 | 125.14 | 2.80 | 0.73 |
| 0.94 | 10 | 124.44 | 2.78 | 0.52 |
| 0.92 | 10 | 124.16 | 2.63 | 0.30 |
| 0.90 | 10 | 124.68 | 1.66 | 0.18 |
| 0.88 | 10 | 126.60 | 1.10 | 0.04 |

Tightening the safety margin to 0.88 brings `util_p85` inside the 1.05 gate and σ to 0.04 s — but **lap time floors at ~124 s regardless**. The plan is no longer the bottleneck. The reactive P-controller leaves ~17 s on the table even with a perfectly feasible reference plan.

Supporting checks DID pass:
- **§11.55.D — v3_dp brakes earlier than v2 at the chicane.** Confirmed: v3_dp begins braking ~75 m earlier than v2 approaching the chicane (s=514 m vs s≈590 m); apex speeds within 1.82 m/s of v2 (v3_dp=15.68, v2=17.50). ✓
- **§11.55.C — `--plan-source v2` still aborts.** Confirmed regression: deterministic abort at `OffTrackError s=892 m`, same failure mode as pre-4.2. ✓

## Why it didn't close the gap (root cause)

Spec §23.10.1 nailed the root cause prospectively:

> A reactive P-controller cannot honestly track v2's friction-circle-optimal speed plan. v2 is a 3-pass distance-marched optimizer with *perfect lookahead* — it knows the corner speed before it arrives. v3 RK4 + reactive P knows the corner only after it has measured slip climbing, by which time it is already past the brake point.

Phase 4.2 swapped the *plan* the controller chases, not the *controller*. The honest Pacejka plan reduces apex demand from `(mu_v2=1.20 · g)` to `(D_lat=1.03 · g)` — apex speeds drop 5-10 m/s in the chicane — but the same reactive controller still over- and under-shoots the new reference. At s=936 m (a medium-speed sweeper, r≈100-200 m) the controller commands too much steering against under-tracked body slip, blows up alpha_front_avg to ~18° (util_p85=2.74), and either aborts or grinds out a 124 s lap.

The spec's planned remediation for this is **v3.2 (full MPC, §23.10.7)** — replace the receding-horizon P-controller with a model-predictive controller that jointly optimises line and speed over a short horizon. Phase 4.2 was the right principled move to unblock the plan; it confirms the controller, not the plan, is now the limiting factor.

## How to reproduce the headline result

```
python lap.py cars_csv/bmw_1m \
  tracks_csv/ks_nurburgring/layout_sprint_a.csv \
  drivers/tomas.json \
  --model slip --single-lap
# Default plan source is v3_dp.
# Wallclock ~30 s (10 MC runs at Tomas's sigma=1.5).
```

Add `--plan-source v2` to reproduce the §11.55.C regression abort. `--model point-mass` ignores the flag and runs the unchanged v2 simulator (lap 1:48.05).
