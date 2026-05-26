# Spec §23.2 — v3.2 Phase 5.0.4 — Dynamic per-axle Fz in the MPC plant

**Parent spec:** `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc.md` (§23.2)
**Predecessor specs:**
- `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc-phase5_0_1.md` (operating-point Pacejka + ellipse hard constraint)
- `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc-phase5_0_3.md` (QP-divergence detection + ellipse-saturation Tier-1)
**Predecessor architecture:** `docs/architecture-slip-model-phase5_0_3-v32-tier1.md`
**Architecture (post-implementation, ArchDev to author):** `docs/architecture-slip-model-phase5_0_4-v32-dynamic-fz.md`
**Status:** Draft (controller-only; additive)
**Project:** LapTimeEstimator
**Branch:** `feature/sc-71955/lap-simulation`
**Created:** 2026-05-22
**Planned with:** Buddy

---

## §23.2-5.0.4.1 Why Phase 5.0.4 exists

Phase 5.0.3 shipped Tier-1 ellipse-saturation feedforward. It cleanly eliminated the QP infeasibility cascade (1013 → 27 infeasibles per 3-lap MC) and Tier-2 ghost handoffs (from many to near-zero). The headline failure remains: every Sprint A 3-lap completion aborts at the chicane (s ≈ 647 m). Five consecutive controller-side iterations (5.0, 5.0.1, 5.0.2, 5.0.3) have failed to clear it.

**Smoking gun from 5.0.3:** post-solve friction-ellipse violation p95 = **0.001**, against a Tier-1 trigger threshold of 0.15. The QP solves correctly. The Phase-5.0.1 ellipse-proxy linearisation is honest with respect to the inputs it is given. The **plant model itself is wrong**: `build_plant_constants` in `src/lap_estimator/dynamics/mpc_model.py:240-243` uses **static per-axle Fz** (mass · G · cg-split + downforce). During the chicane's right-then-left transition real load transfer (long+lat) drops the unloaded axle Fz to a fraction of static, collapsing available Fy. The MPC plans against grip that physically does not exist; the ODE truth model — which already does 4-wheel dynamic Fz in `src/lap_estimator/dynamics/solver.py:_weight_transfer` (line 216) — saturates and the trajectory diverges.

Phase 5.0.4 adds **per-axle dynamic Fz** to the MPC plant model: longitudinal weight transfer from `a_x`, lateral from `a_y`, both at per-stage horizon resolution. The ellipse hard constraint (5.0.1) and the Tier-1 saturation feedforward (5.0.3) are both `μ·Fz`-based and must consume the dynamic value too.

---

## §23.2-5.0.4.2 Scope

**In scope:**
- §23.2-5.0.4.3 — Dynamic Fz formulation (4-wheel → per-axle aggregation).
- §23.2-5.0.4.4 — Per-stage operating point: where `a_x`, `a_y` come from inside the MPC.
- §23.2-5.0.4.5 — Linearisation strategy (per-stage refresh inside the SQP loop).
- §23.2-5.0.4.6 — Downstream consumers: ellipse hard constraint, Tier-1 saturation, longitudinal DP plan headroom check.
- §23.2-5.0.4.7 — Pacejka operating-point integration (Phase 5.0.1 slope-only form stays; D′ now varies per stage per axle via Fz).
- §23.2-5.0.4.8 — Config plumbing (driver JSON `mpc.dynamic_fz_enabled`, `mpc.cg_height_m`, `mpc.track_width_m`; CLI `--static-fz` regression knob).

**Explicitly out of scope:**
- Phase 5.1 "free-driving" (path-deviation cost shift; spec'd separately).
- The reactive lateral-tracking abort at s ≈ 1090 m (ghost-driver bug, untouched by MPC work).
- Pacejka refit on a richer dataset (own initiative).
- Suspension dynamics (`a_z`, pitch/roll inertia). Phase 5.0.4 uses the quasi-static load-transfer model that already lives in `solver._weight_transfer`; transient roll dynamics deferred.

---

## §23.2-5.0.4.3 Truth-model alignment — and why scope is small

**Finding (verified):** the ODE truth model `solver._weight_transfer` (`solver.py:216-258`) **already** computes 4-wheel dynamic Fz with both longitudinal and lateral transfer terms, summed into per-wheel Fz, then floored at 100 N and consumed by `pacejka_fy` / `pacejka_fx` per wheel. The car geometry it needs — `dyn.h_cg`, `dyn.wheelbase`, `dyn.track_f`, `dyn.track_r`, `dyn.cg_front` — is already available on `CarDynamics` and loaded from `cars_csv/<car>/car.ini` via `vehicle.load_car_dynamics`.

**Implication:** Phase 5.0.4 is a **plant-only** change. The truth model already punishes the static-Fz misprediction; we just need the MPC to anticipate it.

That keeps the change surface tight:
1. `mpc_model.build_plant_constants` — promote `Fz_front`, `Fz_rear` from scalars to **functions of `(a_x, a_y)`**, or compute them per stage at SQP refresh time.
2. `mpc_qp_ellipse.build_ellipse_rows` — consume per-stage axle Fz instead of `pc.Fz_front` / `pc.Fz_rear`.
3. `mpc_controller_tiers.emit_ellipse_saturation` / `_max_ellipse_violation` — same.
4. `longitudinal_planner.plan_longitudinal` — optional one-line tightening of the static-Fz headroom term (see §23.2-5.0.4.6).

No truth-model file is edited.

---

## §23.2-5.0.4.4 Dynamic Fz — formulation

Use the same 4-wheel form the truth model uses, then aggregate per axle for the bicycle-plant MPC. Equations (`m` = total mass, `h` = `dyn.h_cg`, `L` = wheelbase, `T_f`, `T_r` = front/rear track):

**Static (already in `build_plant_constants`):**
```
Fz_front_static = m·g·(1 − cg_front) + F_down·(1 − cg_front)
Fz_rear_static  = m·g·cg_front       + F_down·cg_front
```

**Longitudinal transfer (front loses, rear gains under accel; sign convention `a_x > 0` ⇒ forward accel):**
```
dFz_long = m · a_x · h / L
Fz_front_axle = Fz_front_static − dFz_long
Fz_rear_axle  = Fz_rear_static  + dFz_long
```

**Lateral transfer (inside loses, outside gains):**
```
dFz_lat_front_pair = m · a_y · h / T_f  · (1 − cg_front)   # share carried by front axle
dFz_lat_rear_pair  = m · a_y · h / T_r  · cg_front
```

These split the per-axle Fz across the two wheels of that axle (inside − dFz_lat / 2, outside + dFz_lat / 2). For the bicycle plant the MPC uses, what matters is the **axle total**, which is unchanged by lateral transfer alone (it conserves axle sum). However Pacejka is non-linear in Fz: `Fy_axle = Fy(α, Fz_inner) + Fy(α, Fz_outer) ≠ 2 · Fy(α, Fz_axle/2)`. **The MPC plant must consume axle-total Fz but model the non-linear Fy loss from lateral split.**

**Decision:** use the per-axle "effective Fz" form
```
Fz_axle_eff = Fz_axle_total · (1 − k_lat_loss · |dFz_lat_axle / Fz_axle_total|^2)
```
with `k_lat_loss ≈ 0.15` calibrated against the truth model's `(α, Fz_axle, Fy_axle)` curve over the chicane regime. This is a structural approximation; ArchDev refines the calibration during build with a one-shot regression against `slip_simulator` traces (own initiative if a better functional form falls out). The 0.15 starting value comes from the Pacejka load-sensitivity exponent `p_Ky3 ≈ 0.7` evaluated at typical chicane split ratios.

Alternative considered and rejected: track all four wheels in the MPC plant. Cost is 4× state on the lateral side, the bicycle structure no longer fits, and the open-loop `a_y` ↔ `Fz_split` ↔ `Fy_axle` coupling becomes harder to linearise. The effective-Fz form preserves the bicycle and pays the lateral-split tax once, at plant-constants time.

---

## §23.2-5.0.4.5 Linearisation strategy

Two viable approaches:

**(a) Static Fz at the linearisation operating point, dynamic Fz only inside ODE integration.** Cheaper. `D'` stays constant across the horizon. Keeps `PlantConstants` schema scalar. Does **not** fix the chicane: the QP that aborts is exactly the one whose linearisation operating point is wrong.

**(b) Per-stage Fz from predicted state.** At every SQP iteration, for each stage `k ∈ [0, N_horizon)`, recompute `Fz_front_k(a_x_k, a_y_k)`, `Fz_rear_k(a_x_k, a_y_k)` from the candidate trajectory and rebuild the per-stage ellipse coefficients, the per-stage `D' = D · Fz_k`, and the Pacejka slope.

**Recommendation: (b).** The solve-time budget allows it: Phase 5.0.3 measured 18.9 ms mean / 32.8 ms p99 against gates of 30 / 50 ms. Per-stage Fz adds ~4 floats per stage per axle and a constant-factor recomputation of the ellipse denominators; that is well under the headroom. Honesty matters more than the 1-2 ms it costs.

**Per-stage `a_x`, `a_y` source — build-time choice, default value stated here for ArchDev:**
- Default: **predicted-trajectory values from the previous SQP iteration's solution**, evaluated at stage `k`. Falls back to the previous-tick ODE state for stage 0 if no prior solution exists (first MPC call after reset).
- Alternative: pinned to the previous-tick ODE-measured `a_x`, `a_y` for the whole horizon. Cheaper, less honest at the apex.

The first-iteration warm start uses the previous-tick MPC solution's `(a_x, a_y)` profile; if absent, use the ODE state values broadcast across the horizon. SQP convergence damping (§23.2-5.0.4.9 risk #2) addresses the inner feedback loop.

---

## §23.2-5.0.4.6 Downstream consumer updates

All sites currently using `pc.Fz_front` / `pc.Fz_rear` as a constant scalar must accept a `(stage_idx, axle)` lookup:

1. **`mpc_qp_ellipse.build_ellipse_rows`** (`mpc_qp_ellipse.py:215`)
   `D_long_front_Fz`, `D_lat_front_Fz` (and rear counterparts) at lines 238-241 are computed once per call. Promote to per-stage arrays of length `N_horizon`. Each ellipse row's coefficient picks up its stage's Fz.

2. **`mpc_controller_tiers._max_ellipse_violation`** (`mpc_controller_tiers.py:188`)
   The post-solve violation scan must compare against the same per-stage `D · Fz` envelopes the QP saw. Otherwise the Tier-1 trigger goes off on a phantom violation.

3. **`mpc_controller_tiers.emit_ellipse_saturation` / `_project`** (`mpc_controller_tiers.py:281, 395`)
   When the QP fails or post-solve violation breaches threshold, Tier-1 emits per-axle (Fx_ff, Fy_ff) projected onto the ellipse. The ellipse used must be the axle's current dynamic-Fz ellipse, not the static one. This is the **primary lever** for whether 5.0.4 closes the chicane: if Tier-1 still over-trusts the unloaded axle, the saturation feedforward will keep sending too much steering.

4. **`longitudinal_planner.plan_longitudinal`** (referenced from `slip_simulator._build_dp_plan`)
   The DP plan uses `static Fz, friction-ellipse-aware combined-slip headroom`. Phase 5.0.4 leaves the DP plan **static-Fz by default** (it is a forward-backward pass over the racing line; trying to feed it `a_y` predictions is circular). It tightens the DP's safety margin from 0.94 (Phase 5.0.1 default) to a configurable value if the chicane MC pass rate is still below gate. ArchDev's call at build time. Document the chosen value in the architecture doc.

5. **`build_plant_constants` signature** — add `(a_x_op, a_y_op)` optional arguments. When omitted (back-compat path, e.g. tests, the DP planner), the call reduces to the static-Fz path. When the SQP loop calls it per stage, pass the current `(a_x_k, a_y_k)`.

---

## §23.2-5.0.4.7 Pacejka operating-point integration

Phase 5.0.1 shipped slope-only Magic-Formula linearisation at `α_op = 0.5 · α_peak · skill_factor` (see `mpc_model.py:272-285` and the prose comment about the affine-form chicane sign-flip pathology). That decision **stays as-is**. What changes:

- `C_alpha_front_k = slope_op_front_per_fz(α_op) · Fz_front_axle_k(a_x_k, a_y_k)` — per stage.
- `D_lat_front_k = calib.front.lateral.D` is still a calibration constant (peak μ); the **available peak force** is `D_lat_front · Fz_front_axle_k`, which is the value the ellipse uses.
- `α_op` itself does not vary with Fz; it tracks the driver's skill-bounded steady-state angle, not the load.

Pacejka's true load sensitivity (`p_Ky3` and friends) is encoded inside the per-wheel call in `solver.py` and bleeds into the MPC only through the `Fz_axle_eff` non-linear-split term in §23.2-5.0.4.4. Build a one-shot regression of `D · Fz_eff` against the truth model in `tests/dynamics/test_mpc_dynamic_fz.py` at build time to confirm < 5 % deviation at chicane operating points.

---

## §23.2-5.0.4.8 Acceptance gates (§11.55-5.0.4)

| Gate | Threshold | Notes |
|---|---|---|
| MC 3-lap completions | **≥ 7 / 10** | **MUST PASS — the unmet gate since Phase 5.0.1.** |
| Lap 1 best | ≤ 2:15.14 | Match Phase 5.0.2 floor. Stretch: ≤ 2:00. |
| Tier 0 share | ≥ 80 % | Healthy QP usage. |
| Tier 1 share | ≤ 15 % | Saturation feedforward, not the workhorse. |
| Tier 2 share | ≤ 5 % | Ghost fallback rare. |
| Solve mean | < 30 ms | Per-tick MPC. |
| Solve p99 | < 50 ms | Per-tick MPC. |
| Cross-track | ≤ 6 m peak | Same as 5.0.3. |
| Post-solve ellipse violation p95 | ≤ 0.05 | Was 0.001 in 5.0.3; allow headroom for the per-stage Fz approximation feedback loop. |
| Front axle Fy / Fz_dyn at chicane apex | ≤ 1.0 (i.e. respects D · Fz_dyn) | New diagnostic; logged to the slip-trace CSV. |

The chicane completion is the **headline metric**. The architecture doc must report: was the Sprint A 3-lap MC pass rate ≥ 7 / 10? If not, what was the failure mode (still QP infeasibility? Tier-1 over-saturation? truth-model divergence elsewhere?).

---

## §23.2-5.0.4.9 Risks and mitigations

**Risk 1 — Per-stage re-linearisation blows the solve-time budget.**
Per-stage Fz adds an inner loop over horizon stages inside the SQP iteration. The numpy/QP overhead is small but real.
*Mitigation:* (a) keep a fast-path that retains the static-Fz coefficients when `|a_y_max_in_horizon| < 2 m/s²` (straight-line). (b) Vectorise the per-stage Fz across the horizon — single broadcast-multiply, no Python loop. (c) Profile p99; if > 50 ms, gate the per-stage refresh to "every N-th SQP iteration" with N = 2.

**Risk 2 — SQP feedback loop: predicted `a_y` ⇒ `Fz_axle` ⇒ `Fy_axle` ⇒ `a_y'`.**
The lateral acceleration the MPC plans for depends on the Fy it computes, which depends on Fz, which depends on the same `a_y` we're trying to find. Naive substitution can oscillate.
*Mitigation:* (a) update `Fz_axle_k` only between SQP iterations, never inside a single QP solve (one-step Picard iteration per SQP). (b) Damp the update: `Fz_k^{i+1} = (1 − β) · Fz_k^i + β · Fz_k_from_solution`, `β ∈ [0.3, 0.5]`. (c) Log SQP iteration count; if it grows above the existing cap, fail back to the previous solution rather than diverge.

**Risk 3 — Lap time can decrease, not increase.**
Static Fz was **optimistic** on the unloaded axle but **pessimistic** on the loaded axle. Switching to honest dynamic Fz takes grip away from the unloaded one (correctly — the truth model already saturates there) but **gives grip back to the loaded one** (the outer axle in a corner has more Fz than static would model, so D · Fz_dyn > D · Fz_static there). The chicane should clear because the unloaded-axle saturation is the binding failure mode, but on big-Fz corners the MPC might suddenly authorise *more* aggression than 5.0.3 did. Net effect on lap time is ambiguous a priori.
*Mitigation:* report lap-time delta with and without `--static-fz`. If 5.0.4 paradoxically *slows* the lap on non-chicane corners, drop the `Fz_axle_eff` lateral-split correction's `k_lat_loss` floor and re-measure. Document the choice in the architecture doc.

**Risk 4 — `dyn.h_cg` may not be calibrated correctly per car.**
`vehicle.load_car_dynamics` reads CG from the AC `car.ini`. The BMW 1M `car.ini` does not have an explicit `CG_HEIGHT`; `dyn.h_cg` is either inferred from `[GRAPHICS]` / `PICKUP_FRONT_HEIGHT` or hard-defaulted. Verify what the current value is during build.
*Mitigation:* expose an override via driver JSON `control_params.mpc.cg_height_m` (default: use the value already on `dyn`). Log at controller-init time the CG height actually used so it shows up in the architecture diagnostic.

---

## §23.2-5.0.4.10 Open questions for build time

1. **`a_x_k`, `a_y_k` source for the per-stage Fz refresh** — previous-tick ODE state pinned across the horizon vs. previous SQP iteration's predicted trajectory at stage `k`. §23.2-5.0.4.5 recommends the latter; ArchDev confirms during build that the SQP damping in Risk 2 is sufficient.
2. **Per-driver JSON exposure** — should `control_params.mpc.cg_height_m`, `mpc.track_width_m`, `mpc.anti_roll_bias` be configurable per driver? Anti-roll bias in particular shifts the lat-transfer split between front and rear axles. Phase 5.0.4 default: not configurable, use the car geometry. Revisit if MC pass rate is still below gate.
3. **`k_lat_loss` calibration** — 0.15 is a placeholder. Replace with a one-shot regression against `slip_simulator` chicane traces during build (own initiative).
4. **DP planner tightening** — if MC pass rate misses, the DP plan's `safety_margin` (Phase 5.0.1: 0.94) is the next knob. ArchDev's call; document the chosen value.

---

## §23.2-5.0.4.11 Migration & back-compat

**Driver JSON (`drivers/*.json`):**
```jsonc
"control_params": {
  "mpc": {
    "dynamic_fz_enabled": true,        // default true
    "cg_height_m": null,                // null ⇒ use car.ini value on CarDynamics
    "track_width_m": null               // null ⇒ use car.ini value on CarDynamics
  }
}
```
Driver JSON without these fields uses the defaults. Existing driver files unchanged.

**CLI:**
- `--static-fz` (default off): forces `dynamic_fz_enabled = False` regardless of driver JSON. Regression A/B knob.
- `--mpc-fz-update-rate {per-sqp-iter, per-tick}`: optional advanced override. Default `per-sqp-iter`. `per-tick` reverts to recomputing Fz once per controller tick (cheaper, less honest).

**Internal API changes:**
- `build_plant_constants(..., a_x_op=0.0, a_y_op=0.0, dynamic_fz=False)` — new kwargs default to the static-Fz path.
- `PlantConstants` schema gains `dynamic_fz: bool` field for diagnostics. `Fz_front`, `Fz_rear` stay as the operating-point values (now per-call, may equal static).
- `mpc_qp_ellipse.build_ellipse_rows(..., fz_front_per_stage=None, fz_rear_per_stage=None)` — when omitted, fall back to `pc.Fz_front` / `pc.Fz_rear` broadcast across all stages.
- `mpc_controller_tiers._max_ellipse_violation` and `emit_ellipse_saturation` gain per-stage Fz parameters with the same fallback.

The static-Fz code path is preserved end-to-end and reachable via `--static-fz`. Tests that pin against 5.0.3 behaviour run on the static path; the new dynamic-Fz path gets its own dedicated test set.

---

## §23.2-5.0.4.12 Test plan

1. **Unit: `Fz_axle_eff` regression vs. truth model** — over `(a_x, a_y) ∈ [−12, +6] × [−14, +14] m/s²`, `Fz_axle_eff(a_x, a_y)` vs. `solver._weight_transfer`'s per-wheel sum agrees within 5 %.
2. **Unit: SQP convergence damping** — synthetic chicane: confirm Picard iteration on `(a_y_k, Fz_k)` converges in ≤ 5 iterations for `β = 0.4`.
3. **Integration: chicane single-corner replay** — drive the existing `tomas.json` driver into the Sprint A chicane apex from the canonical seed; assert the per-stage `Fy_front / (D_lat_front · Fz_front_axle_k)` stays ≤ 1.0.
4. **Integration: full Sprint A 3-lap MC ×10** — the headline gate. Pass = ≥ 7 / 10.
5. **Regression: `--static-fz` reproduces 5.0.3** within numerical noise (lap times within 50 ms, tier-share within 1 percentage point).
6. **Perf: solve-time histogram** — confirm mean < 30 ms, p99 < 50 ms.

---

## §23.2-5.0.4.13 Deliverables

- **Code (ArchDev):** changes confined to `src/lap_estimator/dynamics/mpc_model.py`, `mpc_qp_ellipse.py`, `mpc_controller_tiers.py`, `mpc_controller.py` (call-site plumbing), `_control_params.py` (new JSON fields), `slip_simulator.py` (CLI flag wiring). Truth model untouched.
- **Tests:** under `tests/dynamics/`, new file `test_mpc_dynamic_fz.py`.
- **Architecture doc:** `docs/architecture-slip-model-phase5_0_4-v32-dynamic-fz.md` — measured chicane completion rate, lap-time delta, solve-time histogram, Fz-trace at the chicane apex, the chosen `k_lat_loss` and DP `safety_margin`.
- **Spec cross-link:** add a 3-line "next phase" pointer at the bottom of `spec-section-23-2-v32-mpc-phase5_0_3.md`.

---

## §23.2-5.0.4.14 References

- `docs/architecture-slip-model-phase5_0_3-v32-tier1.md` — the 0.001 ellipse-violation diagnostic that motivated this phase.
- `src/lap_estimator/dynamics/mpc_model.py:203` — `build_plant_constants`, the static-Fz site.
- `src/lap_estimator/dynamics/mpc_qp_ellipse.py:215` — `build_ellipse_rows`, ellipse coefficient site.
- `src/lap_estimator/dynamics/mpc_controller_tiers.py:188, 281` — Tier-1 violation scan + saturation feedforward.
- `src/lap_estimator/dynamics/solver.py:216` — `_weight_transfer`, the truth-model 4-wheel dynamic-Fz reference implementation.
- `cars_csv/bmw_1m/car.ini` — `TOTALMASS=1570`, `STEER_LOCK=450`, geometry source for `dyn.h_cg`, `dyn.wheelbase`, `dyn.track_f/r`.
