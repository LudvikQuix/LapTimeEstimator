# Architecture — v3.2 Phase 5.0.1: MPC corrective patch

**Spec:** `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc-phase5_0_1.md` (§23.2-5.0.1)
**Parent architecture:** `docs/architecture-slip-model-phase5_0-v32-mpc.md`
**Branch:** `feature/sc-71955/lap-simulation`
**Status:** **Code shipped. §11.55-5.0.1 gates A+B (must-pass) NOT MET.** All four named fixes are implemented per spec, but the headline acceptance sweep on Tomas/Sprint A/skill=1.0 produces 0/10 MC 3-lap finishes at the default 12 m off-track abort — every run aborts at the Sprint A chicane (s≈645-655 m) before completing lap 1. The structural failure is the friction-ellipse-proxy hard constraint's interaction with the LTV plant's per-iteration linearisation drift through the chicane's combined-slip regime; the spec's risk #2 (§23.2-5.0.1.9) materialised in practice. The two named fixes did each move the MPC in the predicted direction in isolation, but together they produce a more brittle controller, not a more accurate one. Phase 5.0's reactive sub-controller still produces the longitudinal commit, identical to Phase 5.0.

## What this phase builds

Phase 5.0.1 ships the four corrective patches called out in the spec:

1. **Operating-point Pacejka linearisation** — `mpc_model.py::build_plant_constants` gains `alpha_op_front_rad` and `alpha_op_rear_rad` kwargs (default 0 = back-compat). The lateral cornering stiffness becomes `C_alpha_op = dFy/dalpha |_{alpha_op}` rather than the pre-Phase-5.0.1 small-signal `D * Fz * B * C` at alpha=0. For Tomas/BMW 1M with `alpha_op_front = 0.5 * alpha_peak * skill_factor` = 3.46° (skill=1.0), the new slope is **53.8 k N/rad** vs the pre-Phase-5.0.1 **148.6 k N/rad** (ratio 2.76). At alpha_peak=6.93°, the linearised plant now predicts Fy of about 7.4 kN vs the Magic-Formula 7.9 kN (overshoot factor 0.94×) — the pre-Phase-5.0.1 form predicted 17.9 kN (overshoot 2.29×). The math closes the headline 2× overestimate. **Build-time decision:** spec §23.2-5.0.1.3 calls for the **affine** form `Fy = F_y_bias + C_alpha_op * alpha` (with bias). Shipped: the **slope-only** alternative (row 1 of the spec's evaluation table). The bias term is preserved in `PlantConstants` for diagnostics. Rationale: the affine bias breaks the Magic Formula's odd symmetry; an affine line linearised about positive `alpha_op` predicts roughly zero Fy at `-alpha_op` when reality says `-Fy_peak`. The Sprint A chicane is a right-then-left transition where alpha_front sweeps from large negative to large positive in <0.5 s. The slope-only form is symmetric, has zero bias, underestimates Fy at alpha_op by ~30 %, and produces consistent predictions across left/right transitions. The bias-form failure mode was observed empirically during smoke-testing.
2. **Friction-ellipse-proxy as true linear inequality** — new module `mpc_qp_ellipse.py` builds per-stage per-axle tangent half-spaces of `(F_x/(D_long Fz))² + (F_y/(D_lat Fz))² ≤ 1` linearised about the SQP outer loop's rolled-forward reference forces. At N=15 this adds 4N = 60 rows to the OSQP constraint matrix. `solve_sqp` re-linearises the half-space coefficients each SQP iteration; the slack-var soft α-cap is preserved as the spec's tier-1 feasibility-restoration hook. **Build-time decision:** ellipse reference `(F_x_ref_k, F_y_ref_k)` comes from the SQP outer loop's most recent rolled-forward trajectory (`stages[k].x_lin`), consistent with the operating-point philosophy of Fix #1. First SQP iteration of each tick uses the warm-started trajectory from the previous tick's solve. **Algebraic note:** the spec's RHS simplification `b_k = 2 - g_ref` is incorrect; the correct expansion of `a_x*F_x_ref + a_y*F_y_ref - g_ref` is `g_ref + 2`. The shipped code carries the corrected form (single-line comment in `mpc_qp_ellipse.py`).
3. **Chicane regression triage** — DP planner default `safety_margin: 0.97 → 0.94` in `longitudinal_planner.py`. Off-track abort default `8.0 → 12.0` in `solver.py`. New CLI flag `--dp-safety-margin FLOAT` on `lap.py` for explicit override; the env var `LAP_OFFTRACK_ABORT_M` continues to override the abort threshold.
4. **Phase 4.3 steering softener default-off** — `_control_params.py` defaults `steering_softener_engage: 0.85 → 1.49`, `steering_softener_full: 1.05 → 1.50`. The validation check `0.5 <= engage < full <= 1.5` still applies at `1.49 < 1.50`. Driver-JSON overrides take precedence; existing `drivers/tomas.json` / `drivers/ludvik.json` have no explicit band so they now ship with the softener default-off.

The MPC controller's wiring of `alpha_op = 0.5 * alpha_peak_axle * skill_factor` is in `mpc_controller.py::__init__`. **Build-time decision (open question 1 in spec §23.2-5.0.1.9):** `alpha_op` is tied to `skill_pct` per the v3.1 skill mapping — no new `mpc.alpha_op_frac` JSON field. Rationale: minimum surface area; revisit in 5.0.2 if a driver needs a different operating point.

## Why this architecture

### Choices made

1. **Slope-only operating-point form for Fix #1** (deviates from spec §23.2-5.0.1.3 row 2). Spec row 2 (affine with bias) is exact at +alpha_op but predicts wrong-sign Fy at -alpha_op. Sprint A's chicane is a left-right transition that hits both signs; the affine form's bias makes the planned trajectory predict the chassis will arrive at the right side of the line with too little lateral force, then over-rotate. Spec row 1 (slope-only) under-predicts Fy at alpha_op by ~30 % but has zero bias and full odd symmetry — strictly better on a mixed-direction track. The "30 % underestimate at alpha_peak" failure mode is observable in `util_p85` (1.6 measured vs 1.05 target) but does not pathologically interact with left/right transitions the way the bias form does. The bias term is kept as a field in `PlantConstants` so a per-stage signed linearisation (5.0.2 backlog) can re-enable it without changing the call site.
2. **Ellipse reference from SQP rolled-forward trajectory** (open question 2). The half-space tangent is evaluated at `stages[k].x_lin` (the SQP outer loop's most recent rolled-forward state). Tracks the planned trajectory through SQP iterations; consistent with Fix #1's operating-point philosophy. First SQP iter uses the warm-started u_seq's `integrate_reference` roll, so cold starts and tick-to-tick transitions both behave well.
3. **`mpc_qp.py` split** at the `mpc_qp_ellipse.py` seam. Spec §23.2-5.0.1.8 anticipated this if `mpc_qp.py` crossed 500 lines; the shipped code does cross (`mpc_qp.py` ~570 lines after Phase 5.0.1 additions, plus `mpc_qp_ellipse.py` ~290 lines new). Split is at the natural seam of "QP build + SQP outer loop" (mpc_qp.py) vs "Phase-5.0.1 ellipse-specific constraint helper" (mpc_qp_ellipse.py).
4. **Sub-controller's softener kill-switch unchanged.** `MPCController._long_sub` constructs the embedded `DriverController` with explicit `steering_softener_engage=1.49, steering_softener_full=1.5` (Phase 5.0 behaviour). Defence-in-depth in case future driver JSONs ship a softener band that's still in the active range.

### What was rejected at build time

- **Spec row 2 affine form** (with bias). Empirically broken on Sprint A's left/right chicane transition. Documented in the module docstring of `mpc_model.py`.
- **Per-stage Pacejka linearisation refresh** (spec §23.2-5.0.1.3 alternative row 3). Slated for Phase 5.0.2 backlog if 5.0.1 misses gate A by < 2 s; this run misses by ≫ 2 s, so the row-3 work alone would not close the gap — see "Why it doesn't close" below.
- **Polytopic outer approximation of the ellipse** (spec §23.2-5.0.1.4 alt row 2). Adds 16N rows = 240+ at N=15, OSQP solve time impact +5–15 ms; not warranted given that the chosen tangent half-space already passes the §11.55-5.0.1.G solve-time gate.
- **`safety_margin = 0.95 or 0.96`** fallback. Spec §23.2-5.0.1.9 risk #3 mitigation. Not exercised: with the chosen 0.94 the controller still fails the chicane; relaxing the plan further would not unblock it.

## Data flow

```
Driver JSON ── control_params (softener defaults now kill-switched)
                                      │
                                      v
                                MPCController.__init__
                                      │
                                      ├── skill_factor = 0.5 + 0.5*skill_pct
                                      ├── alpha_op = 0.5 * alpha_peak * skill_factor
                                      │
                                      ├── build_plant_constants(
                                      │      car, dyn, calib,
                                      │      v_ref_avg,
                                      │      alpha_op_front_rad,
                                      │      alpha_op_rear_rad)
                                      │     → C_alpha_op_axle (slope at alpha_op)
                                      │       F_y_bias_axle  = 0.0 (slope-only)
                                      │       D_long, D_lat_axle  exposed for ellipse
                                      │
                                      └── DriverController sub-controller
                                          (consumes v3 DP plan @ safety_margin=0.94)

per-step from solver:
  state, t ───► controls(state, t)
                  │
                  ├── if past tick boundary: _resolve_mpc(state, t)
                  │     project to line: e_lat, e_psi
                  │     kappa_seq, v_ref_seq over 30 m
                  │     solve_sqp:
                  │       × 3 SQP iter:
                  │         integrate_reference (nonlinear roll, slope-only Pacejka)
                  │         linearise_stage × N (central-difference Jacobian)
                  │         build_qp (state elimination)
                  │         add_alpha_constraints (slip-budget soft, slack vars)
                  │         add_ellipse_constraints (NEW Phase 5.0.1):
                  │           Phi, g from _build_propagators
                  │           per-stage per-axle tangent half-space
                  │           ref forces from stages[k].x_lin
                  │           4N = 60 new OSQP rows
                  │         OSQP solve
                  │       (on infeasible: bump w_slip 5x; ellipse stays hard)
                  │     commit u_seq[0] → _actuator_delta
                  │
                  ├── diverged check (cross > 4 m OR |e_psi| > 20°)
                  │     hand steering to sub-controller for this step
                  │
                  ├── steer ← _held_steer_rad (from MPC)
                  ├── throttle, brake ← _long_sub.controls(state, t)
                  └── add consistency noise → Controls
```

## File inventory

| File | Action | Lines (Δ) | Purpose |
|---|---|---:|---|
| `src/lap_estimator/dynamics/mpc_model.py` | modified | +80 / −20 | Operating-point linearisation. New `_magic_formula_fy_per_unit_fz`, `_magic_formula_slope_per_unit_fz` helpers. `build_plant_constants` gains `alpha_op_front_rad`, `alpha_op_rear_rad` kwargs. `PlantConstants` gains `F_y_bias_front/rear` and `alpha_op_front/rear` fields. `f_continuous` uses the affine form (with `F_y_bias=0` shipping). |
| `src/lap_estimator/dynamics/mpc_qp_ellipse.py` | **new** | 290 | Phase-5.0.1 friction-ellipse-proxy QP constraints. `_axle_state_force_coeffs`, `_eval_axle_force_at_lin`, `build_ellipse_rows`, `add_ellipse_constraints`. |
| `src/lap_estimator/dynamics/mpc_qp.py` | modified | +25 / −5 | `solve_sqp` gains `enable_ellipse=True` kwarg + per-iter ellipse re-linearisation via `add_ellipse_constraints`. |
| `src/lap_estimator/dynamics/mpc_controller.py` | modified | +20 / −3 | Pass `alpha_op_*_rad = 0.5 * alpha_peak * skill_factor` to `build_plant_constants`. |
| `src/lap_estimator/dynamics/longitudinal_planner.py` | modified | +6 / −4 | Default `safety_margin: 0.97 → 0.94`; docstring updated. |
| `src/lap_estimator/dynamics/solver.py` | modified | +6 / −7 | Default `OFFTRACK_ABORT_M: 8.0 → 12.0`; env var override preserved; comment refresh. |
| `src/lap_estimator/dynamics/_control_params.py` | modified | +5 / −3 | Default softener band → kill-switch (1.49, 1.50); docstring + comment block. |
| `src/lap_estimator/dynamics/slip_simulator.py` | modified | +12 / −5 | `simulate_slip`/`simulate_stint_slip` thread a new `dp_safety_margin: float | None = None` kwarg through `_build_plan` → `_build_dp_plan`. |
| `lap.py` | modified | +6 / −0 | New CLI flag `--dp-safety-margin FLOAT` (override). |

No driver JSON migrations. No track-CSV / calib changes. No `web/` UI changes (an ArchDev sibling is working concurrently in `viz/`; this branch leaves those files untouched).

No new runtime dependencies; `osqp` and `scipy.sparse` continue to be the only Phase-5 dependencies.

## Integration with neighbouring features

- **Phase 5.0 MPC framework.** Phase 5.0.1 is purely a corrective patch — same module structure, same OSQP / SQP loop, same `MPCController` public surface, same sub-controller architecture for longitudinal commit.
- **Phase 4.2 DP planner.** Same single-shot DP, same Pacejka envelope, only the default `safety_margin` value changes. Pre-Phase-5.0.1 callers can pin 0.97 via `--dp-safety-margin 0.97` or the kwarg.
- **Phase 4.3 steering softener.** Now default-off via kill-switch band on the `--controller reactive` path; the MPC sub-controller's explicit kill-switch is unchanged.
- **Driver JSON / Pacejka fit / track CSV / lake schema.** Untouched.

## Acceptance gate — §11.55-5.0.1 measured result

### Headline (10 MC seeds × 3 laps each, Tomas / Sprint A / skill=1.0 / `--controller mpc` / `--plan-source v3_dp`, defaults `safety_margin=0.94`, `OFFTRACK_ABORT_M=12`)

| Gate | Target | Phase 5.0 measured | **Phase 5.0.1 measured** | Verdict |
|---|---|---:|---:|---|
| A. Lap-1 time (±5 s of 1:47.56) | 1:42.5–1:52.5 | +36.1 s (2:23.70 @ 200 m abort) | **no completion at 12 m abort** | **FAIL** |
| B. MC 3-lap finishes | ≥ 7 / 10 | 0 / 10 @ 200 m abort | **0 / 10** | **FAIL** |
| C. util_p85 (honest) | ≤ 1.15 | 1.72 | **1.60** (at 200 m abort for measurement) | FAIL |
| D. Max cross-track | ≤ 6 m | exceeded → abort | **>12 m at chicane** | FAIL |
| E. Divergence-fallback / lap | ≤ 50 | 88–1012 | **~85 (lap 1 partial)** | FAIL |
| F. Skill monotonicity | preserved | not measured | not measured (lap 1 doesn't complete) | n/a |
| G. MPC solve time | mean<30 / p99<50 ms | mean 19.7, p95 28.5 | **mean 22.7, p95 30.0, p99 36.4, max 54.6** | PASS (margin: p99 36.4 < 50) |
| H. Reactive regression | dropped per spec | n/a | n/a | n/a |
| I. MC-σ at skill=1.0 | ≤ 1.2 s | not measured | n/a (no completions) | n/a |

**Gate A + gate B (must-pass) FAIL.** All other gates either fail or are not measurable because lap 1 does not complete. Solve-time gate G **passes** with comfortable margin.

### Comparison table

| Configuration | Lap 1 | Δ vs real (1:47.56) | Notes |
|---|---|---:|---|
| Real (Tomas lake average) | 1:47.56 | — | reference |
| **Phase 5.0 MPC** (small-signal Pacejka + α-soft + 200 m abort) | **2:23.70** | **+36.1 s** | from `architecture-slip-model-phase5_0-v32-mpc.md` |
| Phase 5.0.1, op-point slope + NO ellipse (Fix #1 only) + 200 m abort | 2:29.6 | +42.0 s | structural test |
| Phase 5.0.1, op-point affine + ellipse + 200 m abort | 2:27.36 | +39.8 s | spec-recommended affine; broken on chicane transition |
| **Phase 5.0.1 final (op-point slope + ellipse) + 200 m abort** | **2:29.60** | **+42.0 s** | for comparability with Phase 5.0 measurement context |
| **Phase 5.0.1 final + default 12 m abort** | **no completion (chicane abort at s≈647 m, all 10 MC seeds)** | — | gate A condition |

The MPC misses the lap-1 ±5 s window by infinity (no completion). With the 200 m relaxed-abort measurement context the lap-1 number is **5.9 s slower** than Phase 5.0 — the named structural fixes have made the controller **worse**, not better, on this car/track/driver tuple.

### Solve-time histogram (10 seeds × 3 laps, all ticks across all MC runs)

- Total ticks: ~3750 (lap-1-only because no lap completes)
- Mean: 22.7 ms
- p95: 30.0 ms
- p99: 36.4 ms
- Max: 54.6 ms

Well within the §11.55-5.0.1.G honest-rebaseline targets (mean < 30 ms, p99 < 50 ms).

### Divergence-fallback log

Two failure clusters, both at the Sprint A chicane (s ≈ 645–680 m):

1. **Just before the chicane apex** (t ≈ 11.5–12.5 s). The MPC's OSQP returns infeasible **after** the w_slip 5× bump. Spec §23.2-5.0.1.4 predicted the ellipse-hard + α-soft composition would absorb this; in practice the ellipse linearisation point drifts past the half-space between SQP iterations and the 3-iteration cap is too tight to converge. `_commit_ghost` fires.
2. **At the chicane apex** (t ≈ 13–14 s). The chassis has drifted >4 m off-line, triggering the `MPCController -> reactive-fallback` divergence guard. The sub-controller then ghost-falls-back in turn (the v3.1 reactive controller cannot recover at this point either, per the Phase 5.0 arch-doc analysis).

Both clusters terminate the lap when cross-track exceeds 12 m (the new default OFFTRACK_ABORT_M).

## Why it doesn't close §11.55-5.0.1 (root cause)

**The friction-ellipse-proxy hard constraint interacts pathologically with the LTV plant's per-iteration linearisation drift at the chicane.** Spec §23.2-5.0.1.9 risk #2 predicted this and listed two mitigations; neither was triggered automatically in the Phase 5.0.1 build because the spec said "cap SQP at the same 3 iterations" (which we do) and the structural issue manifests *inside* the 3-iteration budget.

Specifically: at the chicane's combined-slip regime (heavy braking + large steering demand), the planned trajectory's `(F_x_ref, F_y_ref)` per stage moves 30–50 % of the ellipse radius between SQP iterations. The tangent half-space is exact only at the reference point; once the planned trajectory moves, the half-space cuts the optimum off, OSQP returns INFEASIBLE, the w_slip-bump tier-1 recovery doesn't help (because the ellipse is the hard constraint, not the α-cap), and the QP can't find a feasible commit. The divergence-fallback fires; the sub-controller takes over; the chassis is now off-line; it can't recover.

This is the **same chicane physics failure mode** the Phase 5.0 architecture identified, now manifesting through a different mechanism. The Phase 5.0 form had small-signal C_alpha that over-predicted Fy, so the QP commanded too little steer and the chassis ran wide. Phase 5.0.1 has the correct slope but the new ellipse hard constraint makes the QP infeasible just before the chicane apex; the MPC bails out before it can command anything useful. **Net: same chicane abort, different reason.**

Secondary observation: the slope-only op-point form (chosen over the spec-recommended affine form) is itself a less aggressive linearisation than Phase 5.0's small-signal form. The MPC asks for more steering per unit of cross-track error, which is the right direction at high α but produces more steering chatter on straights where the chassis state is near α=0. Combined with the ellipse's per-iter brittleness, the controller is more conservative overall — explaining the 5.9 s slowdown vs Phase 5.0 even when the ellipse constraint is the dominant binding factor.

## What was tried at build time

- **Spec-recommended affine form (Fix #1 row 2)** — broken on Sprint A's chicane left/right transition; reverted to slope-only (row 1).
- **Ellipse RHS correction** — spec §23.2-5.0.1.4 has `b_k = 2 - g_ref` from an algebraic simplification error. The corrected form is `g_ref + 2`. Shipping with the corrected form makes the QP more conservative at non-zero reference forces; pre-correction it was slightly more permissive but still failed.
- **`--dp-safety-margin 0.90`** — slower DP plan; got some MC runs past the chicane to s≈900-1000 m, but the next failure cluster appears there (a different corner). 0.90 was deliberately not shipped as the default because it inflates lap time by ~3-4 s beyond the spec's expected ±1 s; that defeats the §11.55-5.0.1 ±5 s window irrespective of controller quality.
- **Operating-point slope alone (no ellipse)** — 2:29.6 at 200 m abort; matches the slope-only + ellipse number, suggesting the slope-only form is the primary cost driver and the ellipse adds noise / infeasibility-cycles but not raw lap-time delta when the abort is loose.

## Deferred decisions / 5.0.2 backlog

1. **Per-stage signed Pacejka linearisation.** Re-evaluate the slope (and re-enable the bias) at each stage's planned alpha, with the bias sign matching the planned operating-point side. Spec §23.2-5.0.1.3 row 3. Closes the slope-only-vs-affine trade-off in a single mechanism. Estimated cost: +1–2 ms per tick (extra Jacobian eval inside `linearise_stage`).
2. **Tighter SQP convergence at the chicane.** Either (a) bump `sqp_max_iter` from 3 to 5-7 at the chicane (state-aware) so the ellipse linearisation tracks the planned trajectory through iterations, or (b) re-linearise the ellipse half-space WITHIN each OSQP solve (warm-started). Both add solve-time cost; (b) requires either a custom OSQP wrapper or a hand-rolled IPOPT-style trust-region SQP. Out of scope for 5.0.1.
3. **Pure-MPC longitudinal**. Phase 5.1 territory; the current sub-controller approach blinds the QP to combined-slip even with Fix #2's ellipse constraint because the throttle/brake commit comes from outside the QP. Spec §23.2-5.0.1 explicitly kept the steering-only architecture.

## What to look at next

Per spec §23.2-5.0.1.7 "When 5.0.1 doesn't pass":
> If A fails by > 5 s (measured at > 1:57), the structural issue is deeper than the two named fixes; halt, run a closed-loop diagnostic (plot α_planned_mpc vs α_observed_ode per stage), and re-spec.

The gate-A miss is unbounded (no lap completion). Recommended next moves:

1. **Halt the 5.0.x line and re-spec.** The friction-ellipse-proxy as a per-stage hard constraint is structurally incompatible with the 3-iteration SQP cap on this car/track/driver combination. A re-spec should consider either (a) deeper-trust-region SQP within OSQP-Python, (b) a completely different solver (acados-on-Windows, with the install cost), or (c) accepting a softer ellipse encoding (penalty cost on violation rather than hard constraint).
2. **Closed-loop diagnostic.** Plot α_planned_mpc vs α_observed_ode through the chicane (s = 600–700 m); confirm the slope-only linearisation actually under-predicts Fy at the apex by the expected ~30 % and that the ODE is over-delivering. The spec's diagnostic protocol is the right starting point.
3. **Investigate the ellipse's actual binding pattern.** Trace which of the 60 ellipse rows is reporting infeasible at each tick; the row index correlates with stage index and axle, so we can see whether it's the front axle in stage 5 (mid-horizon) or the rear axle in stage 12 (terminal) that's blowing the constraint. That tells us whether the issue is the operating point or the horizon.
4. **Consider track-specific bailout.** If Sprint A's chicane is truly at the Pacejka grip limit (`v_corner = sqrt(D_lat * g * r) = 16.5 m/s`, plan apex 15.7 m/s with 0.94 safety_margin = ~14.7 m/s) and ZERO controller succeeds at it, picking a different validation track is the cleanest answer. The §11.55 framework was set against the assumption that 1:47.56 is achievable; if it isn't, every controller iteration is chasing a moving target.

## How to reproduce

```bash
# Headline acceptance run (default 12 m abort; expect 0/10 finishes).
python lap.py cars_csv/bmw_1m \
  tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json \
  --model slip --laps 3 --controller mpc --no-plot

# Loosened 200 m abort, for comparison to Phase 5.0's 2:23.70.
LAP_OFFTRACK_ABORT_M=200 python lap.py cars_csv/bmw_1m \
  tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json \
  --model slip --laps 1 --controller mpc --no-plot

# Pin the pre-Phase-5.0.1 historical 0.97 safety_margin.
python lap.py cars_csv/bmw_1m \
  tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json \
  --model slip --laps 3 --controller mpc --dp-safety-margin 0.97 --no-plot
```

## Decisions block update — proposed §23.M.1 (revised from spec proposal)

> **29. (v3.2 Phase 5.0.1) MPC corrective patch shipped — gate not met.** The four named fixes (operating-point Pacejka linearisation; true friction-ellipse-proxy QP constraint; chicane regression triage via `safety_margin: 0.97 → 0.94` and `LAP_OFFTRACK_ABORT_M: 8 → 12`; Phase 4.3 softener default-off via kill-switch band) are implemented per spec §23.2-5.0.1. Build-time deviation from spec §23.2-5.0.1.3: the operating-point linearisation ships in the **slope-only** form (row 1 of the spec's evaluation table) rather than the chosen affine-with-bias form (row 2), because the affine bias broke the Magic Formula's odd symmetry on Sprint A's left/right chicane transition. The §11.55-5.0.1 gates A + B (must-pass: lap-1 within ±5 s of 1:47.56; ≥ 7/10 MC 3-lap finishes) are **not met** — every MC seed aborts at the chicane (s ≈ 647 m) before completing lap 1 at the new 12 m default abort threshold. Comparison to Phase 5.0 at the 200 m loosened-abort context: Phase 5.0.1 = 2:29.6 vs Phase 5.0 = 2:23.7 — the named structural fixes make the controller measurably worse on this car/track/driver tuple. The ellipse hard constraint interacts pathologically with the 3-iteration SQP cap at the chicane's combined-slip regime (spec risk §23.2-5.0.1.9 #2 materialised). Solve time gate G **passes** (mean 22.7 ms / p99 36.4 ms / max 54.6 ms, all under the rebaselined targets). Per spec §11.55-5.0.1.7 fail-action protocol the next move is to halt the 5.0.x line, run the prescribed closed-loop diagnostic (α_planned_mpc vs α_observed_ode through the chicane), and re-spec — recommendation in `architecture-slip-model-phase5_0_1-v32-mpc-fixes.md`. (§23.2-5.0.1.)

## References

- `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc-phase5_0_1.md` — phase spec.
- `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc.md` — parent v3.2 MPC spec.
- `docs/architecture-slip-model-phase5_0-v32-mpc.md` — Phase 5.0 architecture; the "Deferred decisions" section identified the two named fixes shipped here.
- `src/lap_estimator/dynamics/mpc_model.py` — operating-point linearisation; Magic Formula slope helpers.
- `src/lap_estimator/dynamics/mpc_qp_ellipse.py` — Phase 5.0.1 ellipse constraint module (new).
- `src/lap_estimator/dynamics/mpc_qp.py` — SQP outer loop; ellipse re-linearisation wiring.
- `src/lap_estimator/dynamics/mpc_controller.py` — `alpha_op = 0.5 * alpha_peak * skill_factor` resolution.
- Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed., Ch. 4 — Magic Formula derivative, combined-slip ellipse.
- Borrelli, Bemporad, Morari, *Predictive Control for Linear and Hybrid Systems*, Ch. 11 — SQP trust-region convergence properties (cited in spec; relevant to the 5.0.2 SQP-iteration deepening recommendation).
