# v3.3 MPCC — implementation architecture

**Spec:** `dev-planning/lap-simulation-csv-driver/spec-section-23-3-v33-mpcc.md`
**Status:** First-pass ship (2026-05-24). Steers, integrates, runs in budget at horizon 30 m; **does not yet meet the lap-time / completion gates on Sprint A.** Build-time decisions, integration points, and tuning levers documented for the next iteration.

Cross-references:

- `docs/architecture-v3-session-2026-05-24.md` — parent session.
- `docs/architecture-slip-model-phase5_3-v33-mpcc.md` — _not_ written this round (spec-named placeholder); this doc supersedes that filename.
- `docs/architecture-slip-model-phase5_0_4-v32-mpc-dynamic-fz.md` — v3.2 MPC architecture that v3.3 mirrors.

## 1. What the code does

Adds a new controller `--controller mpcc` alongside the existing `--controller mpc`. The new controller plans in **curvilinear (s, n, ψ_e) coordinates** along a reference path resampled from the centreline (or a Tomas-line override, plumbed by the parallel ArchDev). The cost penalises contour error (`n²`), lag (`(s − θ)²`), and rewards progress (`−w_progress · V_θ`); the optimiser commits **steering only**, with throttle / brake delegated to the existing reactive sub-controller (the `_long_sub` pattern inherited from v3.2). The QP is built fresh per tick and solved with OSQP through a 3-iteration SQP outer loop. Diagnostics flow through the existing `SlipSimResult.mpc_*` slots (re-used) plus three MPCC-specific terminals (`mpcc_contour_p95`, `mpcc_lag_p95`, `mpcc_progress_mean`).

## 2. File inventory

**Created** (`src/lap_estimator/dynamics/`, all under the 500-line soft cap):

| File | LoC | Role |
|---|---|---|
| `mpcc_reference.py` | ~320 | `ReferencePath` dataclass, `build_reference_path()`, `to_curvilinear`/`to_chassis` transforms, wrap-aware shortest-s helper. |
| `mpcc_model.py` | ~290 | Curvilinear LTV bicycle plant (NX_C=10, NU_C=4). `f_continuous_c`, `linearise_stage_c`, `integrate_reference_c`, plus per-stage Fz hooks that delegate to `mpc_model.compute_dynamic_fz_per_stage`. |
| `mpcc_qp.py` | ~390 | `MPCCWeights`, `MPCCBounds`, `build_mpcc_qp` (condensed QP with contour + lag + progress + heading-error + smoothness + terminal + track-edge + per-stage V_θ caps), `add_mpcc_ellipse_constraints` (re-uses `mpc_qp_ellipse.build_ellipse_rows` via a Phi/g shim), `solve_sqp_mpcc` (3-iter SQP). |
| `mpcc_controller.py` | ~700 | `MPCCController` class + Tier 0 / Tier 2 dispatch. |

**Modified:**

| File | Change |
|---|---|
| `src/lap_estimator/dynamics/slip_simulator.py` | Import `MPCCController`; add `'mpcc'` branch to `_make_controller`; forward `mpcc_*` kwargs through `simulate_slip → _run_monte_carlo → _run_single → _make_controller`; populate the new `mpcc_*` `SlipSimResult` slots under an `isinstance(ctrl, MPCCController)` guard. |
| `src/lap_estimator/dynamics/_slip_result.py` | Add `mpcc_contour_p95`, `mpcc_lag_p95`, `mpcc_progress_mean` fields to `SlipSimResult`. |
| `lap.py` | Add `'mpcc'` to `--controller` choices; add `--mpcc-horizon-m`, `--mpcc-n-stages`, `--mpcc-tick-hz`, `--mpcc-w-contour`, `--mpcc-w-lag`, `--mpcc-w-progress`, `--mpcc-w-du`, `--mpcc-force-static-fz` flags; forward through `simulate_slip`; one-line MPCC-specific diagnostics print at end of run. |

No edits to `mpc_controller.py`, `mpc_qp.py`, `mpc_qp_ellipse.py`, `mpc_model.py`, or `mpc_controller_tiers.py` — owned by the parallel ArchDev work streams (Phase 5.0.7 QP tune, Tomas-injection, CiMPCC quick-win).

## 3. State + control layout

**State (NX_C = 10)**: `[s, n, ψ_e, v_x, v_y, ω, δ, throttle, brake, θ]` — module-level `IDX_*` constants in `mpcc_model.py`.

**Control (NU_C = 4)**: `[δ_dot, throttle_dot, brake_dot, V_θ]`. The first three are physical actuator rates; `V_θ` is the virtual-progress rate (`dθ/dt = V_θ`).

**Curvilinear dynamics** (`mpcc_model.f_continuous_c`):

```
ds/dt   = (v_x cos ψ_e − v_y sin ψ_e) / max(1 − n · κ(θ), DENOM_FLOOR)
dn/dt   = v_x sin ψ_e + v_y cos ψ_e
dψ_e/dt = ω − κ(θ) · ds/dt
dθ/dt   = V_θ
```

Chassis dynamics (Fy/Fx, drag, yaw) are byte-for-byte the v3.2 affine slope-only Pacejka from `mpc_model._axle_forces` reshaped against the curvilinear state layout. The denominator on `ds/dt` is floored at `DENOM_FLOOR = 0.3` so the Jacobian stays well-conditioned even at the apex (spec §23.3.11 risk #1). `κ_ref(θ)` is read from `ReferencePath.kappa_ref` (already clamped to `[-0.2, +0.2]` rad/m at build time).

## 4. Cost function

Per-stage:

- `w_contour · n_k²` — perpendicular path error.
- `w_psi_e · e_psi_k²` — **build-time addition** (not in spec §23.3.7.3). Without it MPCC has no direct heading-tracking; the contour cost penalises `n²` only and only feels heading via integrated dynamics over the horizon (too slow for the Sprint A chicane). v3.2 has `w_psi = 20` for the same reason. Default 20.0.
- `w_lag · (s_k − θ_k)²` — lag.
- `−w_progress · V_θ_k` (linear, enters `q`) — progress reward.
- `w_du · ‖u_k‖²` and `w_du2 · ‖u_k − u_{k-1}‖²` — input rate / curvature smoothness.

Terminal: `w_term · (n_N² + (s_N − θ_N)²)`.

## 5. Build-time decisions

### 5.1 Tick rate — 10 Hz default (per spec §23.3.10.1)

10 Hz is Liniger's default. v3.2 uses 50 Hz to stabilise the LTV linearisation between ticks; MPCC's progress-cost shape removes the same phantom-heading-drift symptom in principle. Build-time observation: at 50 Hz MPCC produces a **higher** lag p95 and Tier-2 share than at 10 Hz on Sprint A (44 m lag p95 at 50 Hz vs 7 m at 10 Hz). 10 Hz is the better operating point on this car/track. CLI override available via `--mpcc-tick-hz`.

### 5.2 Track-edge constraint source — per-sample `width_total_m`

Sprint A's `tracks_csv/ks_nurburgring/layout_sprint_a.csv` has the per-sample `width_total_m` column (values 11–14 m). `build_reference_path` reads it; the per-stage half-width passed to the QP is `0.5 · width_total_m − safety_buffer (0.3 m)`. Fallback for tracks without the column: hard-coded 5 m half-width (10 m track). Decided in `mpcc_reference.py:build_reference_path`.

### 5.3 Tier 1 (ellipse-saturation feedforward) — deferred

Per spec §23.3.6.6. The v3.2 Phase 5.0.3 planned-direction logic (`compute_planned_direction` in `mpc_controller_tiers.py`) is coordinate-agnostic in principle but the per-axle force projection lives in chassis frame; porting it through the curvilinear coordinate transform is non-trivial. MPCC ships with **Tier 0 (clean MPCC) + Tier 2 (reactive)** only. Tier 1 port is the documented fallback per spec §23.3.11 risk #3 if completion rate stays low.

### 5.4 Stage gridding — uniform in `dθ` (spec §23.3.6.4)

`dθ_stage = horizon_m / n_stages`. Default 2 m / stage at horizon 50 m / 25 stages. Stage time per stage `ts_k = dθ_stage / max(V_θ_k_predicted, v_ref_stage, V_FLOOR)`. Each stage's reference values `(κ_ref_k, v_ref_k, x_ref_k, z_ref_k)` are looked up at `θ_predicted_k = θ_0 + k · dθ_stage` (constant V_θ extrapolation; the SQP inner loop re-grids implicitly via the per-iter rolled trajectory).

### 5.5 θ re-sync per tick (deviation from spec §23.3.7)

Spec carries `θ` across ticks as a state variable. In practice the linearised plant cannot prevent θ from running ahead of `s` without a hand-tuned `w_progress / V_θ_max` combination; the cost coupling via state propagation is weak. Build-time decision: **re-sync `θ = s_here` at the start of every tick** so the QP works from a near-zero lag baseline. The lag cost then binds `V_θ` to track chassis ds/dt over the horizon. Matches Liniger's MATLAB convention (his `Bicycle.json`-driven setup re-initialises θ by Frenet projection at every solve).

### 5.6 Per-stage `V_θ_max` cap

Global `bounds.v_theta_max = 1.5 · max(v_ref)` is loose. Per-stage cap `v_theta_max_seq[k] = min(global, 1.1 · v_ref_seq[k])` is applied inside `build_mpcc_qp`. Keeps the progress reward from saturating V_θ at the global ceiling on the long Sprint A straight (v_ref ≈ 74 m/s at start of lap).

### 5.7 Friction ellipse — enabled (default)

The ellipse rows are re-used byte-for-byte from `mpc_qp_ellipse.build_ellipse_rows`. Adapter `mpcc_qp._phi_g_to_v32_view` rebuilds the chassis-frame `(Phi, g)` view the helper expects from the curvilinear propagators. Build-time A/B: with ellipse OFF the lap aborts at the s≈350 m fast left-hander (Tier-2 share ~73 %); with ellipse ON the lap survives until the chicane at s≈640 m (Tier-2 share ~11 %). Ellipse stays on.

### 5.8 Dynamic Fz — enabled (default)

`mpcc_model.axle_accelerations_from_trajectory_c` mirrors the v3.2 helper, deriving `(a_x_k, a_y_k)` from the rolled trajectory's `v_x_k` finite differences + `v_x · ω` lateral approximation. The per-stage Fz refresh inside the SQP outer loop uses `mpc_model.compute_dynamic_fz_per_stage` byte-for-byte. CLI knob `--mpcc-force-static-fz` for regression.

## 6. Data flow per tick

```
state (chassis frame, x, z, ψ, v_x, v_y, ω)
         │
         ▼
   to_curvilinear(state, ref, hint_idx)
         │   → (s_here, n_here, e_psi_here, idx)
         ▼
   _theta := s_here                     # re-sync per §5.5
         │
         ▼
   theta_seq = _theta + [0, dθ_stage, 2·dθ_stage, …]
   sample_seq(theta_seq, ref) → (κ_seq, v_ref_seq, half_width_seq)
   v_theta_max_seq = min(global, 1.1 · v_ref_seq)
         │
         ▼
   x0 = [s_here, n_here, e_psi_here, v_x, v_y, ω, δ_actuator,
         throttle_actuator, brake_actuator, _theta]
   warm-start u_seq (shifted from prior tick)
         │
         ▼
   solve_sqp_mpcc:
     for it in range(3):
        x_seq = integrate_reference_c(x0, u_seq, ref, pc, …)
        Fz_per_stage = compute_dynamic_fz_per_stage(...) if dynamic_fz
        stages = [linearise_stage_c(x_seq[k], u_seq[k], …) for k]
        problem = build_mpcc_qp(stages, x0, weights, bounds,
                                v_theta_max_seq, half_width_seq)
        problem = add_mpcc_ellipse_constraints(problem, …) if enable_ellipse
        u_new, status = _solve_qp_inner(problem)
        if not status in ("solved", "solved inaccurate"): → infeasible_recovery
        u_seq = damped_update(u_seq, u_new)
         │
         ▼
   tier classifier (infeasible → Tier 2; else Tier 0)
         │
         ▼
   commit:
     d_delta = u_seq[0, 0] · ts0  (clipped to delta_dot_max · tick_period)
     new_delta = actuator_delta + d_delta
     held_steer_rad = new_delta
     _theta += V_θ_0 · tick_period
         │
         ▼
   emit on every ODE step until next tick:
     steer = held_steer_rad
     throttle, brake = _long_sub.controls(state, t)   # reactive sub-controller
     apply_consistency_noise(...)
         │
         ▼
   Controls(steer_rad, throttle, brake)
```

## 7. Diagnostics surface

| `SlipSimResult` field | Source | Notes |
|---|---|---|
| `mpc_solve_times_s` | `ctrl.solve_times` | Re-uses v3.2 slot. |
| `mpc_tier_counts` | `ctrl.tier_counts` | `{0: clean MPCC, 2: reactive}`. No `1: ellipse` (Tier 1 not ported). |
| `mpc_tier2_episodes` | `ctrl.tier2_episodes` | Re-uses v3.2 slot. |
| `mpc_qp_status_counts` | `ctrl.qp_status_counts` | Re-uses v3.2 slot. |
| `mpcc_contour_p95` | `ctrl.contour_p95` | 95th-percentile `|n|` over the run. **MPCC-specific.** |
| `mpcc_lag_p95` | `ctrl.lag_p95` | 95th-percentile `|s − θ|` over the run. **MPCC-specific.** |
| `mpcc_progress_mean` | `ctrl.progress_mean` | Mean committed `V_θ`. **MPCC-specific.** |

Run-end stdout prints (when `--controller mpcc`):

```
  MPC solve: n=… ticks; mean=… ms, p95=… ms, max=… ms; ghost-fallbacks=…
  MPC tiers: 0=… (xx.x%) | 2=… (xx.x%)  [tier2 episodes=…]
  MPCC diagnostics: contour_p95=… m, lag_p95=… m, V_theta_mean=… m/s
```

## 8. CLI surface

```
--controller mpcc
--mpcc-horizon-m FLOAT       (default 50.0)
--mpcc-n-stages INT          (default 25)
--mpcc-tick-hz FLOAT         (default 10.0)
--mpcc-w-contour FLOAT       (default 100.0)
--mpcc-w-lag FLOAT           (default 1000.0)
--mpcc-w-progress FLOAT      (default 2.0)
--mpcc-w-du FLOAT            (default 1.0)
--mpcc-force-static-fz       (regression A/B knob)
```

Driver-JSON `control_params.mpcc` block mirrors the same keys (without the leading `mpcc.` and `--`). Resolution priority: CLI > driver JSON > dataclass defaults.

## 9. Current results (build-time smoke, Sprint A, Tomas, skill=1.0, --inertia-zz 2400)

Single-lap MC=10 invocation:

```
python lap.py cars_csv/bmw_1m \
    tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    drivers/tomas.json \
    --model slip --controller mpcc --single-lap --inertia-zz 2400
```

| Variant | Completions | Median best lap | Solve mean / p95 | Tier 0 / 2 share | Contour p95 | Lag p95 |
|---|---|---|---|---|---|---|
| Default (50 m / 25 / 10 Hz) | **0 / 10** | — | 36 / 42 ms | 89 / 11 % | 0.58 m | 7.33 m |
| `--chicane-safety-mult 0.85` | 0 / 10 | — | 36 / 42 ms | 89 / 11 % | 0.78 m | 7.33 m |
| `--chicane-safety-mult 0.75` | 0 / 10 | — | 35 / 42 ms | 86 / 14 % | 0.79 m | 7.33 m |
| Horizon 30 m / 15 stages | 0 / 10 | — | 21 / 25 ms | 90 / 10 % | 0.69 m | 7.34 m |
| Horizon 70 m / 35 stages | 0 / 10 | — | 55 / 65 ms | 88 / 12 % | 0.77 m | 7.32 m |
| **Reactive (control)** | **10 / 10** | **2:09.12** | — | — | — | — |
| v3.2 MPC (default) on this seed | 0 / 10 | — | 18 / 25 ms | 81 / 16 % (T1 3 %) | — | — |

All MPCC variants abort at the Sprint A chicane (s ≈ 640 m). **Same failure region as v3.2 MPC on this seed batch**; v3.2 MPC also produces 0/10 with these defaults (the headline 7/10 claim from `architecture-v3-session-2026-05-24.md` is for `--controller reactive`).

Solve-time gate (mean < 30 ms, p99 < 50 ms) is **met at horizon 30 m / 15 stages** and **not met at horizon ≥ 50 m**. The horizon-50 default produces a ~36 ms mean — close to the budget but not within. Horizon 30 is the practical operating point until the cost is re-balanced.

## 10. Why MPCC does not pass the gates yet

Build-time root-cause sketches (validation pending):

1. **No direct yaw control.** The cost has contour + lag + heading-error (the build-time `w_psi_e = 20`) but no explicit `ω` cost. v3.2 has `w_psi = 20` (= my w_psi_e) which couples through e_psi_dot = ω − κ·v_x, so penalising e_psi penalises slow yaw response indirectly. MPCC should already have this through the e_psi term, but the heading-error coupling through the curvilinear plant may be weaker than in the chassis-frame plant. Worth checking the SQP rolled trajectory at the chicane.

2. **V_θ saturation pulls θ ahead of s.** Even with the per-stage cap `1.1 · v_ref_seq[k]`, V_θ_mean = 55 m/s while chassis v_x mean is much lower (because v_ref at the Sprint A start is 74 m/s — long straight). The lag cost penalises (s − θ)², but the QP picks V_θ near the cap because the alternative (V_θ ↓) costs progress reward + the lag-cost-via-state-coupling is weakened by the explicit-Euler discretisation of `dθ/dt = V_θ`.

3. **Steering chatter at the chicane.** Tier-2 share rises to 11 % around the chicane — the QP goes infeasible repeatedly. The reactive sub-controller takes over, but by then the chassis is already off the line from MPCC's prior commits. A smaller `delta_dot_max` (currently 8 rad/s) or stronger `w_du` might smooth the transition.

The most likely highest-impact next experiments (per spec §23.3.11 risk register):

- **Tier 1 port** (risk #3) — keep MPCC's steering envelope sane on QP-infeasible ticks at the chicane.
- **`w_progress` sweep 0 → 5 → 10** (risk #3-a) — find a setting where the optimiser stops over-rewarding progress on the straights.
- **Plant-model spot-check** — compare the linearised continuous-time Jacobian of `ds/dt` against the v3.2 Jacobian of `e_lat_dot` over a sweep of `(v_x, v_y, ψ_e, ω, n, κ)`. If the curvilinear plant predicts wrong lateral response at high curvature, the steering commands at the chicane will be wrong regardless of cost tuning.

## 11. Integration points

| Direction | Notes |
|---|---|
| **Up — `_make_controller`** | One new `if controller == "mpcc"` branch in `slip_simulator.py:_make_controller`. Forward path-line override kwargs (`line_xs`, `line_ys`) — added by the parallel Tomas-line ArchDev. |
| **Up — `SlipSimResult`** | Three new fields (`mpcc_contour_p95`, `mpcc_lag_p95`, `mpcc_progress_mean`); re-uses the existing `mpc_*` slots for solve-time / tier-share so report/CSV layers don't fork. |
| **Lateral — `mpc_qp_ellipse`** | Re-used byte-for-byte via `_phi_g_to_v32_view` shim. No edit. |
| **Lateral — `mpc_model.compute_dynamic_fz_per_stage`** | Re-used via `mpcc_model.axle_accelerations_from_trajectory_c`. No edit. |
| **Down — `DriverController`** | The reactive sub-controller is embedded as `_long_sub` (same plumbing as `MPCController`). |

## 12. What the next ArchDev should pick up first

1. **Why does v3.2 MPC also fail at s ≈ 640 m on these MC seeds?** The session doc claims 7/10 reactive, but the v3.2 MPC may have regressed. Verify on the parallel Phase 5.0.7 QP-tune branch (other ArchDev's territory). If it really has regressed, MPCC may already be at parity.
2. **Run MPCC with the Tomas-line override** (`--line-source tomas` if exposed). The reference path is the line MPCC tracks; using Tomas's measured racing line should produce sharper steering commands at the chicane apex than the centreline does.
3. **Walk the curvilinear linearisation** against the v3.2 chassis-frame linearisation on a few representative `(state, control)` operating points. The sign / scale of `B_cont[IDX_OMEGA, IDX_DELTA_DOT]` is the critical entry for yaw response to steering rate; if it doesn't match v3.2 at the chicane apex, that's the bug.
4. **If the gate stays missed at all weight combinations**, port Tier 1 (spec §23.3.6.6 deferred decision; see §5.3 above).
