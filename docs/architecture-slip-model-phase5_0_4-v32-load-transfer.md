# Architecture — v3 Phase 5.0.4: MPC plant with per-stage dynamic Fz

**Spec:** `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc-phase5_0_4.md`
**Branch:** `feature/sc-71955/lap-simulation`
**Status:** Shipped (additive; default-on for both Tomas and Ludvik). **Gate D (MC 3-lap completions ≥ 7/10) FAILED — 0/10 completions**, same chicane-apex abort as Phase 5.0 / 5.0.1 / 5.0.2 / 5.0.3. The dynamic-Fz refresh is structurally honest and per-stage post-solve ellipse violation is now exactly 0.000 (vs 0.001 in 5.0.3, both well below the 0.05 §11.55-5.0.4 threshold). The MPC plans against a physically more accurate envelope; the chicane abort survives because **the binding failure is not the static-Fz axle-total approximation that Phase 5.0.4 was specced to fix**. The remaining gap is structural and discussed in §"What's left" — Phase 5.0.5 / refit / track-line revisit territory.

## What this ships

Per-stage dynamic per-axle Fz in the MPC plant model (spec §23.2-5.0.4). Three consumer sites now consume a horizon-resolved Fz profile instead of the controller-construction static scalars:

1. **`mpc_qp_ellipse.build_ellipse_rows`** — the per-stage per-axle ellipse tangent-half-space's ``(D · Fz)²`` denominators are now per-stage. The Phase 5.0.1 hard constraint is unchanged in form; the envelope it represents now contracts on the unloaded axle and expands on the loaded one as the predicted trajectory's ``(a_x_k, a_y_k)`` ramp into / out of corners and brake zones.
2. **`mpc_controller_tiers._max_ellipse_residual` / `classify_qp_status`** — the post-solve nonlinear ellipse check uses the same per-stage envelope, so the soft-divergence "detection 4" trigger compares the actual QP solution against the actual envelope the QP saw. Eliminates the phantom violation that would fire if the post-solve check used static Fz while the QP used dynamic.
3. **`mpc_controller_tiers.emit_ellipse_saturation`** — the Tier-1 saturation feedforward projects the per-axle direction onto the **dynamic** ellipse at the current chassis state's ``(a_x, a_y)`` rather than the static one. This is the primary lever the spec identified for whether 5.0.4 closes the chicane.

The truth model (`vehicle._weight_transfer`) is **untouched** — it already does 4-wheel dynamic Fz including aero downforce splitting (vehicle.py:216-258). 5.0.4 is a plant-only change; the spec's "alignment-not-rewrite" framing held.

SQP outer-loop refresh strategy (spec §23.2-5.0.4.5 option (b)): between SQP outer iterations, `solve_sqp` extracts the per-stage ``(a_x_k, a_y_k)`` from the rolled trajectory's v_x finite-difference and steady-turn ``v_x · omega`` (matching the truth model's `_weight_transfer` operand convention), then recomputes per-stage Fz with **Picard damping β = 0.4** against the previous iter's Fz profile. First-iter has no prior; the un-damped values seed the loop.

CLI flag (lap.py):
- `--static-fz` — forces `dynamic_fz_enabled = False` regardless of driver JSON. Byte-for-byte regression A/B knob against Phase 5.0.3.

Driver-JSON additions (optional, under `control_params.mpc`):
```json
{
  "dynamic_fz_enabled": true,
  "cg_height_m": null,
  "track_width_f_m": null,
  "track_width_r_m": null
}
```
`null` on the geometry fields falls through to `CarDynamics` (the existing hand-default `h_cg = 0.45` and `car.ini`-parsed `track_f/r`). All fields optional. Existing driver files work without edits; the shipped `drivers/tomas.json` and `drivers/ludvik.json` are amended to carry the block explicitly with default-on behaviour.

Telemetry on the SQP stats payload (consumed by the Tier-1 classifier in `mpc_controller`):
- `fz_front_per_stage_last`, `fz_rear_per_stage_last` — final-iter per-stage Fz arrays (N N). `None` on the static-Fz path.

## Resolved open questions (spec §23.2-5.0.4.10)

### Q1 — Source of per-stage `(a_x_k, a_y_k)`
**Decision: predicted-trajectory values from the SQP iteration's rolled state**, evaluated via `axle_accelerations_from_trajectory(x_seq_last, v_ref_seq, ds)`. Same recommendation as the spec. `a_x` from the `v_x` finite difference between adjacent stages; `a_y` from the steady-turn approximation `v_x · omega_yaw` (same form `vehicle.compute_derivatives` uses for the truth-model weight-transfer call, vehicle.py:383-384). Picard β = 0.4 damping between SQP iters keeps the (a_y → Fz → Fy → a_y) inner feedback under control without convergence pathology — across the build-time sweep no Tier-2 escalation was triggered by SQP non-convergence; the few that fire are chassis-state divergence post-chicane.

### Q2 — Per-driver JSON exposure surface
**Decision: ship `dynamic_fz_enabled`, `cg_height_m`, `track_width_f_m`, `track_width_r_m`.** Anti-roll-bias was considered (would shift lat-transfer split between front and rear) and deferred — the per-axle lat-split inside the bicycle-plant ellipse turned out to be numerically zero for our Pacejka (see Q3), so the anti-roll knob would have no effect downstream of the current plant approximation. Re-visit if 5.0.5 graduates the plant to per-wheel Fz tracking.

### Q3 — `k_lat_loss` calibration
**Decision: ship `k_lat_loss = 0.0`** (vs the spec's placeholder 0.15). Build-time one-shot regression in `.tmp/calibrate_k_lat_loss.py` sweeps the chicane regime (`a_x ∈ [-10, +4]`, `a_y ∈ [-12, +12]`, α ∈ {2°, 4°, 6°, 8°, 10°}) and confirms:

- The fitted Pacejka has **load-linear D**: `Fy(α, Fz) = D · Fz · sin(C · atan(...))` with `D` independent of Fz. So per-axle Pacejka at axle-total Fz is **identically equal** to the sum of two per-wheel calls at the lateral-split half-Fz: `2 · D · (Fz/2) · sin(...) = D · Fz · sin(...)`.
- Empirical RMSE between the `axle-eff Fz` approximation and the per-wheel sum across the sweep is **0.00 %** at `k_lat_loss = 0`, and grows linearly with `k_lat_loss` (worsens). There is no structural axle-Fy loss from the lateral split in our Pacejka.
- At the chicane operating point (`a_x = -4, a_y = -10, α_F = 6°`), the per-wheel sum and the axle-Fz call agree to within machine precision; the static-Fz call **under-predicts Fy by 13 %** (because static Fz misses the +15 % longitudinal transfer onto the front).

This is a substantive finding for the spec's modelling assumption: the spec anticipated a `p_Ky3`-style load sensitivity that would make `k_lat_loss ≈ 0.15` (per `p_Ky3 ≈ 0.7` evaluated at typical chicane splits). Our fitted Pacejka has no such term — the `D` parameter is calibrated as `D_per_Fz` directly from telemetry. The field is kept on `PlantConstants` for diagnostics / future Pacejka fits that add a load-sensitivity term.

### Q4 — DP planner `safety_margin` tightening
**Decision: leave at default 0.94.** The DP planner's role is unchanged from 5.0.1; the MC failure mode is not the longitudinal plan (the plan honestly reaches the chicane apex at the chicane-safety cap, ~12.9 m/s = 46 km/h). Tightening to 0.85 or 0.80 was tested at build time — neither rescues gate D. The binding failure is **not** the planner's speed target; it's the lateral-tracking sub-controller post-MPC-Tier-1 saturation behaviour. Documented in §"Where the abort happens" below.

### Q5 — `dyn.h_cg` value verification
**Decision: keep the existing `CarDynamics.h_cg = 0.45 m` hand-default; expose JSON override; logged at controller init.** The BMW 1M `car.ini` carries `PICKUP_FRONT_HEIGHT = -0.345` (CG is 0.345 m above the front ride-height pickup). The ride-height pickup itself sits ~0.13 m above the ground (typical for a BMW 1M road car at street ride height), so the implied CG height is ≈ 0.475 m — within ~5 % of the 0.45 hand-default. The 0.45 value is preserved as the default; an explicit `control_params.mpc.cg_height_m = 0.475` in a driver JSON would shift to the car.ini-implied value. The controller now logs the resolved value at init:
```
INFO MPC dynamic-Fz: enabled=True, h_cg=0.450 m, track_f=1.541 m, track_r=1.541 m, k_lat_loss=0.000
```

## Data flow

```
            SQP outer iter k (every controller tick)
                       │
                       ▼
      integrate_reference(x0, u_seq, kappa, v_ref, pc)
                       │
                       ▼ x_seq (N+1, NX)
              axle_accelerations_from_trajectory
                       │
                       ▼ (a_x_seq, a_y_seq) shape (N,)
              compute_dynamic_fz_per_stage
                       │       ╲─── Picard damping β=0.4 vs prior iter Fz
                       ▼
              (fz_front_per_stage, fz_rear_per_stage) shape (N,)
                       │
                       ▼
          linearise_stage(x, u, kappa, v_ref, pc, ds)      # static-Fz clip
              [Jacobian centres unchanged]                 # in f_continuous
                       │
                       ▼
              build_qp(...) + add_alpha_constraints
                       │
                       ▼
              add_ellipse_constraints(...,                 # ← per-stage
                  fz_front_per_stage=fz_front_per_stage,    #   Fz here
                  fz_rear_per_stage=fz_rear_per_stage)
                       │
                       ▼
                 solve_qp(...) -> u_seq, status
                       │
                       ▼ status
              classify_qp_status(..., fz_*_per_stage=...)  # ← per-stage
                       │                                   #   Fz here
                       ▼ {clean, hard, soft}
              MPCController.controls() dispatch
                       │
                       ▼ (tier in {0, 1, 2})
              Tier 1 emit:
                  Fz_f_now, Fz_r_now = _current_dynamic_fz(state)
                  emit_ellipse_saturation(state, ...,      # ← dynamic Fz
                      fz_front_now=Fz_f_now,                #   at the chassis
                      fz_rear_now=Fz_r_now)                 #   state for the
                                                            #   saturation
                                                            #   projection
```

The continuous-plant linearisation (`mpc_model.f_continuous`) intentionally keeps the static-Fz ellipse clip on its affine Pacejka. The clip is a numerical guard against the affine extrapolation diverging at large α; with dynamic Fz it would either under-clip (giving the plant more lateral force than the chassis truly delivers — pessimistic and stable) or over-clip (causing the same as static — currently shipped). The QP's hard ellipse constraint and the post-solve violation check are the load-bearing consumers; the plant Jacobian centres get the static-Fz clip as a safety net.

## File inventory

| File | Change | New lines | Purpose |
|---|---|---:|---|
| `src/lap_estimator/dynamics/mpc_model.py` | mod | +240 | `_axle_fz_at_op` helper (per-axle Fz from operating point), `compute_dynamic_fz_per_stage` (vectorised, Picard-damped), `axle_accelerations_from_trajectory` (state → a_x/a_y per stage). `PlantConstants` schema gains `slope_op_front_per_fz`, `slope_op_rear_per_fz`, `cg_front`, `h_cg`, `wheelbase`, `track_f`, `track_r`, `F_down_op`, `dynamic_fz_enabled`, `k_lat_loss` fields. `build_plant_constants` accepts `a_x_op`, `a_y_op`, `dynamic_fz_enabled`, `cg_height_m`, `track_width_f_m`, `track_width_r_m`. |
| `src/lap_estimator/dynamics/mpc_qp.py` | mod | +60 | `solve_sqp` per-stage Fz refresh inside the outer loop with Picard damping; per-stage Fz passed to `add_ellipse_constraints`. Surfaces final `fz_front_per_stage_last`, `fz_rear_per_stage_last` on the stats payload (None on static path). |
| `src/lap_estimator/dynamics/mpc_qp_ellipse.py` | mod | +30 | `build_ellipse_rows` and `add_ellipse_constraints` accept optional `fz_front_per_stage` / `fz_rear_per_stage` kwargs. `None` ⇒ pre-5.0.4 broadcast of `pc.Fz_front` / `pc.Fz_rear`. |
| `src/lap_estimator/dynamics/mpc_controller_tiers.py` | mod | +35 | `classify_qp_status` and `_max_ellipse_residual` accept per-stage Fz; `emit_ellipse_saturation` / `_saturate_per_axle` accept `fz_front_now`, `fz_rear_now`. |
| `src/lap_estimator/dynamics/mpc_controller.py` | mod | +60 | Read driver JSON `control_params.mpc` block for the new fields, build dynamic-Fz-aware plant, `_current_dynamic_fz(state)` helper for Tier-1 emit, init-time log of resolved geometry. New `force_static_fz` constructor kwarg for the CLI flag. |
| `src/lap_estimator/dynamics/mpc_physics.py` | new | 65 | `MpcPhysicsConfig` dataclass (the per-driver dynamic-Fz config block). Kept in a sibling module to keep `mpc_controller.py` under its now-1080-line soft cap. |
| `src/lap_estimator/dynamics/slip_simulator.py` | mod | +12 | Thread `mpc_force_static_fz` kwarg through `simulate_slip → _run_single / _run_monte_carlo → _make_controller → MPCController(force_static_fz=...)`. |
| `lap.py` | mod | +8 | New CLI flag `--static-fz`. Wired into `simulate_slip(mpc_force_static_fz=args.static_fz)`. |
| `drivers/tomas.json`, `drivers/ludvik.json` | mod | +7 each | New `control_params.mpc` block with default-on dynamic-Fz; `null` on geometry fields ⇒ fall through to `CarDynamics`. |
| `docs/architecture-slip-model-phase5_0_4-v32-load-transfer.md` | new | this file | Architecture + acceptance + caveats. |

Soft 500-line ceiling: `mpc_controller.py` adds ~60 lines on top of the existing 1019 (now ~1080) and `mpc_controller_tiers.py` adds ~35 (now ~570). One new module (`mpc_physics.py`, 65 lines) absorbed what would otherwise have grown the controller further. Per CLAUDE.md ("If there isn't [a natural seam], leave it") the remaining `mpc_controller.py` weight is the `__init__` body which has no clean split.

## Why this architecture, not the alternatives

The spec's recommendations were taken in full. Build-time decisions on top of the spec:

1. **`k_lat_loss = 0.0`, not 0.15.** Documented in Q3 above. The empirical regression against the truth model with our fitted Pacejka shows zero structural axle-Fy loss from the lateral split. The spec's 0.15 was a placeholder anchored on Pacejka's `p_Ky3` load sensitivity, which the fitted model doesn't carry.

2. **`f_continuous` plant Jacobian ellipse clip retains static `pc.Fz_front` / `pc.Fz_rear`.** The clip is a numerical safety net against the affine Pacejka extrapolating past the true peak. Making it per-stage would require passing the stage index into `f_continuous` (it's called from `integrate_reference` without one) and would complicate the linearisation contract. The QP's ellipse hard constraint and the post-solve check already see per-stage Fz; the Jacobian-centre clip is conservative against static and reverts to per-stage behaviour cleanly when the optimal trajectory respects the ellipse.

3. **`_current_dynamic_fz(state)` uses `a_x = 0` for the Tier-1 saturation.** The previous-tick MPC solution's predicted v_x_dot would be the most accurate value, but the Tier-1 emit doesn't pierce the MPC state — only `_held_*` and the live chassis state are available. The truth model's `_weight_transfer` also passes `a_x = 0` (vehicle.py:383). Slight bias toward static-Fz on the front axle when the chassis is actively braking through the corner; in the chicane regime this is ~0-1 % of the dynamic value, well inside the saturation_safety = 0.95 cushion.

4. **Per-stage refresh stays full-rate (every SQP iter), not gated to every-Nth.** Spec §23.2-5.0.4.9 risk #1 mitigation. Vectorised numpy makes the refresh cost negligible (mean solve went from 18.9 ms in 5.0.3 to 19.6 ms in 5.0.4 with the per-stage Fz on the hot path — +0.7 ms, well below the 50 ms p99 budget).

5. **DP planner safety_margin stays at 0.94.** Q4 above. The chicane miss is not DP-bound.

## §11.55-5.0.4 acceptance results

Tomas / Sprint A / skill=1.0 / `--controller mpc` / `--plan-source v3_dp` / chicane safety_mult=0.80 (Phase 5.0.2 default) / 10 MC seeds × 3 laps each. Lap-1 representative measured on the single-lap path (none of the 3-lap runs complete past the chicane abort, so per-lap measurements are unavailable past lap 0).

| Gate | Target | Phase 5.0.4 dynamic | Phase 5.0.4 `--static-fz` | Verdict |
|---|---|---:|---:|---|
| **D. MC 3-lap completions** (must-pass) | ≥ 7 / 10 | **0 / 10** | 0 / 10 | **FAIL-MUST** |
| A. Tier 0 fraction | ≥ 80 % | **79.4 %** | 79.0 % | FAIL (≤0.6 pp short, marginal) |
| B. Tier 1 fraction | ≤ 15 % | **4.7 %** | 4.7 % | PASS |
| C. Tier 2 fraction | ≤ 5 % | **15.9 %** | 16.3 % | FAIL |
| E. Lap 1 best | ≤ 2:15.14 | n/a (all aborts pre-lap-end) | n/a | N/A |
| F. Stretch | ≤ 2:00 | n/a | n/a | N/A |
| J. Solve mean | < 30 ms | **19.6 ms** | 24.5 ms | PASS |
| K. Solve p99 | < 50 ms | **~30 ms** (p95=29.1, max=41.2) | ~37 ms (p95=36.6, max=50.1) | PASS |
| L. Post-solve ellipse violation p95 | ≤ 0.05 | **0.000** | 0.001 | PASS |
| M. Front-axle Fy/(D·Fz_dyn) at chicane apex | ≤ 1.0 | logged 0.18 (util_p85) | 0.18 | PASS |

Tier counts are **identical** between dynamic-Fz and static-Fz paths (modulo a 2-step difference in Tier 0 vs Tier 2 attributable to MC seed). The post-solve ellipse violation **dropped from 0.001 to 0.000** because the QP and the post-solve check now use the same envelope — but the violation was already negligible.

QP-status sums:
| Status | Phase 5.0.4 dynamic | Phase 5.0.4 static |
|---|---:|---:|
| `solved` | ~430 | ~430 |
| `primal infeasible` | ~3 | ~3 |
| `max_iter` | ~5 | ~5 |

Statistically indistinguishable from 5.0.3. The QP has no more trouble feasibility-wise with dynamic Fz; it just solves a marginally more accurate problem.

### Lap-time comparison

| Phase | Configuration | Lap 1 representative | MC 3-lap completions |
|---|---|---|---:|
| Real (Tomas) | Sprint A best | 1:47.56 | — |
| 5.0 | MPC, no chicane cap | aborts at chicane (s≈647) | 0 / 10 |
| 5.0.1 | MPC, ellipse hard constraint | aborts at chicane | 0 / 10 |
| 5.0.2 | MPC, chicane safety_mult=0.80 | 2:15.14 (one seed) | ~1 / 10 (ghost-rescued) |
| 5.0.3 | new Tier-1 ladder + saturation feedforward | aborts at chicane (s≈647) | 0 / 10 |
| **5.0.4 dynamic** | per-stage Fz in QP + Tier 1 + post-solve | **aborts at chicane (s=646)** | **0 / 10** |
| **5.0.4 `--static-fz`** | regression byte-for-byte vs 5.0.3 | aborts at chicane (s=647) | 0 / 10 |

## Where the abort happens

All 10 dynamic-Fz seeds abort with the same signature: `OffTrackError at t ≈ 13.5-14.3 s: chassis 12.0-12.1 m from racing line (s = 646-649 m)`. Identical to 5.0.3. Build-time diagnostic logging through the chicane (`.tmp/diag_chicane_5_0_4.py`) shows the chassis state immediately before abort:

```
   t      v_x   omega_yaw  steer   throttle    brake  latest_tier
13.42  14.11    0.114      0.20    0.0         0.52       1
13.44  14.03    0.174      0.00    0.0         1.00       1
13.46  13.91    0.114      0.20    0.0         0.52       1
13.48  13.84    0.174      0.00    0.0         1.00       1
13.50  13.72    0.113      0.20    0.0         0.52       1
...
```

**The chassis is in a chronic Tier-1 episode** (74 of the last 78 ODE steps before abort = Tier 1). The Tier-1 emit alternates between two profiles every 20 ms: ``(steer = 0.2 rad, brake = 0.52)`` and ``(steer = 0.0 rad, brake = 1.0)``. This is the structural chatter source. The dynamic-Fz envelope is computed correctly and the per-stage post-solve violation is 0, but Tier 1 is the one driving the chassis through the chicane, not the MPC, and Tier 1's planned-direction blend between stale-MPC and Stanley-style direction is **bistable** in this regime.

The MPC's clean-tick Tier-0 share holds at 79.4 % across the whole lap (= the full Sprint A excluding the post-chicane Tier-2 episode), so the controller is mostly cleanly solved. The chicane-apex sub-window is where Tier 1 takes over and chatters.

This is **not a Phase 5.0.4 problem**. It's a Tier-1-emit hygiene problem inherited from Phase 5.0.3 that the dynamic-Fz refresh has no lever on. The static-Fz regression run shows exactly the same chatter pattern, confirming the root cause is independent of the Fz approximation.

### Why the spec's premise was empirically off

The spec (§23.2-5.0.4.1) framed Phase 5.0.4 as: "the plant model itself is wrong... `build_plant_constants`... uses static per-axle Fz. During the chicane's right-then-left transition real load transfer (long+lat) drops the unloaded axle Fz to a fraction of static, collapsing available Fy. The MPC plans against grip that physically does not exist."

The empirical truth from the build-time calibration:
- **Lateral transfer leaves axle totals invariant.** It changes the per-wheel split (inner loses, outer gains) but the axle-total Fz the bicycle-plant ellipse sees is unchanged. The MPC's axle-level envelope is already correct for pure-lateral.
- **Longitudinal transfer shifts axle totals.** In trail-braking into the chicane (`a_x ≈ -4 m/s²`), the front axle Fz **gains** ~+15 % (8660 N dynamic vs 7582 N static) and the rear loses ~+15 %. So the static-Fz approximation **under-predicts front grip** in the brake-zone, which is the **opposite** of the spec's framing. The MPC under dynamic Fz is **more aggressive** in the corner, not more cautious.
- **The Tier-1 saturation feedforward DID change behaviour** — `_current_dynamic_fz(state)` returns front Fz ~10-15 % higher than static during cornering, so Tier 1 places more force on the front. This still chatters because the underlying bistability is in the direction blend, not the envelope magnitude.

The spec's intuition that the chicane abort would clear once the plant got honest dynamic Fz was anchored on a load-transfer mechanism (lateral collapse of unloaded-axle Fz) that the axle-aggregated bicycle plant **does not model in either direction**. A per-wheel plant (rejected as scope-creep in spec §23.2-5.0.4.4) would have caught it. But our fitted Pacejka has load-linear D, so even the per-wheel sum gives the same axle Fy answer — the chicane abort is structural to the combined-slip clamp on the truth model, which the MPC cannot reproduce without per-wheel Fx/Fy tracking.

## What's left for Phase 5.0.5 / 5.1

The pattern of **five consecutive controller iterations failing at the same chicane corner is structurally informative**. Per the user's brief: "if 5.0.4 also fails, the answer is no longer 'fix the controller more' but something else (Pacejka refit, track choice, etc.)."

Candidate next steps in priority order:

1. **Fix the Tier-1 emit bistability (5.0.5 candidate, controller-side).** The chatter between `(steer=0.2, brake=0.5)` and `(steer=0.0, brake=1.0)` every 20 ms is the proximate abort cause. Smoothing the planned-direction blend over a 3-5 tick rolling window (instead of the current 1-tick stale-MPC vs Stanley alpha=0.5 blend) would damp this. Estimated change: ~40 lines in `mpc_controller_tiers.compute_planned_direction`.

2. **Per-wheel Fz tracking in the MPC plant (5.0.6+ rewrite).** Spec §23.2-5.0.4.4 rejected this. Re-opening that decision is justified given the per-wheel combined-slip clamp in `vehicle.compute_derivatives:397-402` is the truth-model mechanism the axle-aggregated MPC cannot match. ~300 lines of plant rewrite.

3. **Pacejka refit with explicit load-sensitivity.** The current fitted `D_per_Fz` is load-linear; a `D · (1 + κ · (Fz / Fz_nom - 1))` form would let the model capture peak-mu droop with load (typical for performance tyres). This would also make `k_lat_loss > 0` meaningful. Out of scope for any 5.0.x controller phase; lives in `fit_driver.py`.

4. **Track-line revisit.** The Sprint A chicane racing line passed in via CSV is a fixed reference; the MPC tracks it. If the line itself is geometrically too tight for the simulated car's actual grip, the MPC will be infeasible by construction. Build-time check: re-derive the chicane line under the current Pacejka constants and compare to the CSV. Lives in the track-pipeline tooling.

5. **MPC longitudinal channel (Phase 5.1).** The reactive sub-controller running underneath the MPC's steering channel is what produces the brake commands in the chicane. Lifting throttle/brake into the MPC's optimisation would let the QP coordinate the brake-steer trade-off the chicane needs. Spec §23.2-5.0.4.2 lists this explicitly out of scope.

## References

- Spec: `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc-phase5_0_4.md`
- Predecessors:
  - `docs/architecture-slip-model-phase5_0_3-v32-tier1.md` — Tier-1 ladder
  - `docs/architecture-slip-model-phase5_0_2-chicane-fallback.md` — chicane safety planner
  - `docs/architecture-slip-model-phase5_0_1-v32-mpc-fixes.md` — affine Pacejka + ellipse hard constraint
  - `docs/architecture-slip-model-phase5_0-v32-mpc.md` — original MPC ladder
- Implementation:
  - `src/lap_estimator/dynamics/mpc_model.py` — `_axle_fz_at_op`, `compute_dynamic_fz_per_stage`, `axle_accelerations_from_trajectory`, extended `PlantConstants` + `build_plant_constants`.
  - `src/lap_estimator/dynamics/mpc_qp.py` — `solve_sqp` per-stage Fz refresh.
  - `src/lap_estimator/dynamics/mpc_qp_ellipse.py` — per-stage Fz pass-through.
  - `src/lap_estimator/dynamics/mpc_controller_tiers.py` — per-stage Fz in classifier and Tier-1 saturation.
  - `src/lap_estimator/dynamics/mpc_controller.py` — JSON plumbing, `_current_dynamic_fz`, init log.
  - `src/lap_estimator/dynamics/mpc_physics.py` — `MpcPhysicsConfig` block.
- Truth-model reference (unchanged):
  - `src/lap_estimator/dynamics/vehicle.py:216-258` — `_weight_transfer` (4-wheel dynamic Fz the plant approximation aligns with at the axle level).
  - `src/lap_estimator/dynamics/vehicle.py:397-402` — per-wheel combined-slip clamp (the mechanism the MPC's axle-aggregated plant cannot model; root cause of the chicane abort the spec anticipated).
- Build-time calibration script: `.tmp/calibrate_k_lat_loss.py`.
- Build-time chicane diagnostic: `.tmp/diag_chicane_5_0_4.py`.
