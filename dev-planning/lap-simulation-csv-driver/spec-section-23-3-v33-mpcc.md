# Spec §23.3 — v3.3 — MPCC (Model Predictive Contouring Control)

**Parent spec:** `dev-planning/lap-simulation-csv-driver/spec-section-23-slip-model.md` (§23)
**Predecessor specs (v3.2 tracking-MPC line; remain in-tree, NOT replaced):**
- `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc.md` (Phase 5.0 tracking-MPC baseline)
- `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc-phase5_0_1.md` (operating-point Pacejka + ellipse hard constraint)
- `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc-phase5_0_3.md` (QP-divergence detection + Tier-1)
- `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc-phase5_0_4.md` (dynamic per-axle Fz)
**SOTA research artefact (input to this spec):** `.tmp/sota_racing_controllers.md`
**Architecture (post-implementation, ArchDev to author):** `docs/architecture-slip-model-phase5_3-v33-mpcc.md`
**Status:** Draft (additive controller — does **not** retire `--controller mpc`)
**Project:** LapTimeEstimator
**Branch:** `feature/sc-71955/lap-simulation`
**Created:** 2026-05-24
**Planned with:** Buddy

---

## §23.3.1 Summary

v3.2's tracking-MPC (`--controller mpc`) reaches **7-of-10 MC completions** on Sprint A at chicane_safety_mult ≤ 0.85 with a best lap around **2:09**. The remaining ~22 s gap to Tomas's 1:47.56 is **structural to the controller's cost shape**, not the slip plant or the DP planner: a reference-tracking MPC penalises deviation from `v_max(s)` along the planned line and never *rewards* eating the apex. The minimum-time solution under friction-ellipse constraints is to **trail-brake to the ellipse edge and claim the apex earlier than the line dictates** — proved by Velenis & Tsiotras 2007 — and tracking cost does not produce this behaviour at any weight setting (the symptom is the persistent under-rotation logged by the Phase 5.0.x architecture docs).

**MPCC (Model Predictive Contouring Control)** is the canonical fix. The OCP is reformulated in **curvilinear (s, n, ψ_e)** coordinates along a reference path and the cost penalises three quantities:
1. **Contouring error** — perpendicular distance `n` from the reference path.
2. **Lag error** — along-path mismatch between actual progress `s_actual` and a virtual progress variable `θ`.
3. **Negative progress** — `−dθ/dt`, so going faster along the path is *rewarded*.

Rewarding progress is what produces ellipse-edge trail-braking without an explicit "trail-brake mode": the optimiser pays for extra rotation in lag/contour cost and is repaid in progress, naturally producing late-apex behaviour. Reference: Liniger MPCC (ETH Zurich, github.com/alexliniger/MPCC) is the canonical implementation; CiMPCC (arXiv 2502.03695) reports **11.4–12.5 % lap-time gain over baseline MPCC** on sharp-curvature tracks — the Sprint A failure regime exactly.

This spec ships MPCC as **`--controller mpcc`**, in parallel with the existing `--controller mpc`. The friction-ellipse constraint (`mpc_qp_ellipse.py`), the dynamic per-axle Fz refresh (`compute_dynamic_fz_per_stage`), the LTV bicycle plant (`mpc_model.f_continuous` and `linearise_stage`), and the OSQP backend are **reused**; what's new is the coordinate transform, the cost weights, and the reference-path resampling pipeline.

---

## §23.3.2 Goals

1. Ship `--controller mpcc` as a sibling of `--controller mpc`, dispatchable via `slip_simulator._make_controller`.
2. Reach **≥ 7/10 MC completions on Sprint A at chicane_safety_mult ≤ 0.85** (parity with v3.2's headline gate, no regression).
3. **Lap-time gate:** mean best lap **≤ 2:00** (improvement over reactive's ~2:09; ~9 s gain).
4. **Stretch lap-time gate:** ≤ 1:55 (within ~7.5 s of Tomas's 1:47.56). If hit, the simulator + controller pair is publishable.
5. Preserve the v3.2 tracking-MPC code path unchanged so regression A/Bs remain runnable for the lifetime of the branch.
6. Solve-time gates: **mean < 30 ms / p99 < 50 ms per tick** (same as Phase 5.0.4); horizon extended to 50 m / 25 stages at 10 Hz target tick. If OSQP can't hold these gates at horizon 50, drop to 40 m / 20 stages before reaching for acados.

## §23.3.3 Non-goals

- **Replace** the v3.2 tracking-MPC. `--controller mpc` stays; MPCC is additive.
- **Replace OSQP with acados.** OSQP first; acados is the §23.3.10 escalation path if OSQP can't carry the curvilinear linearisation.
- **Slip-aware longitudinal MPC channel.** Deferred from Phase 5.0.4; remains deferred. Throttle/brake stays delegated to the reactive sub-controller for MPCC just as it is in v3.2 (the `_long_sub` pattern in `mpc_controller.py:531`).
- **LMPC (Learning-MPC; Rosolia/Borrelli).** Separate research direction; not in this spec.
- **MPPI / sampling-based controllers.** Interesting fallback if MPCC also under-rotates; not in this spec.
- **Tomas-trajectory injection as reference.** The research artefact recommends doing it first (cheaper, capped at Tomas's line) — that's a separate small spec; this one builds MPCC against the DP plan only. The reference-path slot is plumbed so trajectory injection can be wired in later by passing a different `(x_ref, z_ref, ψ_ref, κ_ref, v_ref)(s)` profile.
- **Pacejka refit on richer data, new tracks, web UI changes.** Out of scope.

## §23.3.4 User stories / scenarios

1. **As a sim user**, I run `python lap.py --model slip --controller mpcc --track ks_nurburgring/layout_sprint_a --driver tomas --mc 10` and get a stint summary CSV showing ≥ 7/10 completions with mean best lap ≤ 2:00.
2. **As a regression check**, I run the same command with `--controller mpc` and `--controller mpcc` on the same track + driver and the printed solve-time / tier-share / completion stats compare side-by-side without me needing to edit code.
3. **As ArchDev**, I switch a single CLI flag (`--mpcc-horizon-m 50` → `30`) and re-run; the controller reflects the change with no other tuning. Same for `--mpcc-w-contour`, `--mpcc-w-lag`, `--mpcc-w-progress`, `--mpcc-w-du`.
4. **As a diagnostic step**, when the run ends, the simulator prints per-tick `(n_max, n_p95, lag_max, lag_p95, dθ/dt_mean)` so I can tell whether the controller is contour-limited, lag-limited, or progress-limited.
5. **As an A/B comparison**, I can pass `--mpcc-debug-trace` and get a per-step CSV at `.tmp/mpcc_trace_<track>_<driver>.csv` showing `(t, s, θ, n, ψ_e, v_x, v_y, δ, throttle, brake, J_total, J_contour, J_lag, J_progress, qp_status)`, identical in shape to the existing `--mpc-debug-trace` pattern from Phase 5.0.3.

## §23.3.5 Proposed design

**Single new controller class** `MPCCController` in a new module `src/lap_estimator/dynamics/mpcc_controller.py`, peer of `MPCController`. Mirrors the v3.2 `__init__` / `controls(state, t, track=None) -> Controls` surface so `_make_controller` dispatch only needs one new branch.

**State and cost.** The OCP uses a curvilinear state `(s, n, ψ_e, v_x, v_y, ω, δ, throttle, brake, θ)` with controls `u = (δ_dot, throttle_dot, brake_dot, V_θ)`. `θ` is a **virtual progress variable** — a state with its own rate-control `V_θ ≥ 0` — and the cost rewards `V_θ`. The lag cost is `(s − θ)²`; the contouring cost is `n²`. This is the standard Liniger formulation (his Bicycle MPCC paper §III.A; his `MPCC/Matlab/MPCC.m` `formulateMPCCProblem.m`).

**Dynamics.** The chassis dynamics are the v3.2 LTV bicycle (re-used from `mpc_model.f_continuous`), just re-expressed in (s, n, ψ_e) frame. The 2-line projection is `ds/dt = (v_x cos ψ_e − v_y sin ψ_e) / (1 − n · κ(θ))`, `dn/dt = v_x sin ψ_e + v_y cos ψ_e`, `dψ_e/dt = ω − κ(θ) · ds/dt`. `κ(θ)` is the reference-path curvature at the virtual progress.

**Constraints.** Reuse `mpc_qp_ellipse.build_ellipse_rows` byte-for-byte (the friction ellipse is in chassis-frame forces — it doesn't care about the spatial coordinate basis). Reuse `compute_dynamic_fz_per_stage` for per-stage axle Fz. Add **track-edge constraints** `n_min ≤ n ≤ n_max` (one row per stage per side; default `|n| ≤ track_half_width − safety_buffer`, with `safety_buffer = 0.3 m`).

**Solver.** Same SQP-with-OSQP-inner pattern as v3.2 (`mpc_qp.solve_sqp`). The curvilinear transform makes the dynamics more nonlinear at high `n · κ`, but at our typical operating point (small `n`, moderate `κ`) the linearisation stays well-conditioned; risks in §23.3.11.

**Reference path.** Pre-computed at controller construction from `(track.csv_data['x'], track.csv_data['z'], track.csv_data['distance_m'], track.csv_data['radius_m'])` plus the DP plan (`plan.distances`, `plan.speeds`). Resampled onto a **fine uniform s-grid (`ds_ref = 0.5 m`)** with `(x_ref(s), z_ref(s), ψ_ref(s), κ_ref(s), v_ref(s))`. `κ_ref(s)` is **clamped** to `[−κ_max, κ_max]` with `κ_max = 0.2 rad/m` (radius ≥ 5 m) to keep the curvilinear Jacobian numerically clean across the chicane apex. Lookup at runtime is a cached-hint nearest-index scan + linear interpolation; same hint-cache trick as `mpc_controller_geom.nearest_index`.

**Longitudinal channel.** As in v3.2 the throttle/brake commit is delegated to the embedded reactive sub-controller (`DriverController` instance, same `_long_sub` pattern). The MPCC plans (δ_dot, throttle_dot, brake_dot, V_θ) for cost-shape reasons but **only the steering rate is committed to the chassis**; pedals come from `_long_sub`. This matches Phase 5.0.4's deferred §23.2.4 decision exactly and means MPCC's job is **steering and racing-line allocation** — which is the symptom of the v3.2 failure and where the lap time is.

**Why this shape over alternatives.** Briefly here; §23.3.13 has the full reasoning.
- Tracking-MPC with new weights: spent four phases on this; doesn't structurally rotate enough.
- acados switch with same cost: solves faster, same wrong cost shape.
- MPPI: addresses the same symptom but is a step-change architecture; defer.
- LMPC: best ceiling but needs 10–30 laps to converge; defer.

---

## §23.3.6 Sub-features / work breakdown

Numbered; each is implementable in isolation and has explicit code touchpoints.

### §23.3.6.1 Reference-path builder

**What:** New module `src/lap_estimator/dynamics/mpcc_reference.py`. One factory function `build_reference_path(track, plan, *, ds_ref=0.5, kappa_clamp=0.2) -> ReferencePath`. Returns a dataclass with `s_grid` (uniform), `xs`, `zs`, `psi_ref` (path tangent, rad), `kappa_ref` (clamped, rad/m), `v_ref` (m/s), plus a `nearest_s(x, z, hint) -> (s, n, ψ_e)` helper that projects a chassis position onto the reference and returns the curvilinear triple.

**Touchpoints:** consumes `track.csv_data` (already required by `MPCController`'s `__init__`), `plan.distances`, `plan.speeds`. No edit to `track.py` or `longitudinal_planner.py`.

**Dependencies:** none.

**Suggested owner:** ArchDev.

### §23.3.6.2 Curvilinear plant + linearisation

**What:** New module `src/lap_estimator/dynamics/mpcc_model.py`. Mirrors `mpc_model.py`:
- State indices: `IDX_S=0, IDX_N=1, IDX_E_PSI=2, IDX_VX=3, IDX_VY=4, IDX_OMEGA=5, IDX_DELTA=6, IDX_THR=7, IDX_BRK=8, IDX_THETA=9` (NX_C = 10).
- Control indices: `IDX_DELTA_DOT=0, IDX_THR_DOT=1, IDX_BRK_DOT=2, IDX_V_THETA=3` (NU_C = 4).
- `f_continuous_c(x, u, *, ref: ReferencePath, pc: PlantConstants) -> np.ndarray` — same chassis force balance as `mpc_model.f_continuous` but `e_lat_dot`, `e_psi_dot`, `s_dot` come from the curvilinear projection formulas (§23.3.5). Reuses `pc.C_alpha_front`, `pc.C_alpha_rear`, the slope-only Pacejka, `k_throttle`, `k_brake_*`, `drag_coeff` unchanged.
- `linearise_stage_c(x_lin, u_lin, *, ref, pc, ds) -> StageLinearisation` — same central-difference Jacobian pattern as `mpc_model.linearise_stage`, just over NX_C+NU_C variables.
- `integrate_reference_c(x0, u_seq, *, ref, pc, ds) -> np.ndarray` — explicit-Euler roll-out with stage time `ts = ds / max(v_ref(θ), V_FLOOR)`. **Stage gridding is now in `dθ` (along reference progress), not `ds_arc` — see §23.3.6.4.**

**Touchpoints:** new file. Imports `PlantConstants, V_FLOOR, G` from `mpc_model.py`. Reuses `compute_axle_force_from_state`, `compute_dynamic_fz_per_stage`, `axle_accelerations_from_trajectory` (these read state indices that are different from v3.2; pass an axis-mapping wrapper or duplicate the small helper rather than parametrising — ArchDev's call at build time).

**Dependencies:** §23.3.6.1.

**Suggested owner:** ArchDev.

### §23.3.6.3 MPCC QP build

**What:** New module `src/lap_estimator/dynamics/mpcc_qp.py`. Mirrors `mpc_qp.solve_sqp`. Builds the OSQP problem with:
- **Cost** (per stage `k`, plus terminal):
  - Contouring: `w_contour · n_k²`
  - Lag: `w_lag · (s_k − θ_k)²` — both `s` and `θ` are state variables; this is a quadratic in the decision vector
  - Progress reward: `−w_progress · V_θ_k` — linear; encoded as a negative term in `q`
  - Input smoothness: `w_du · ‖u_k − u_{k-1}‖²` plus `w_du2 · ‖u_{k} − 2u_{k-1} + u_{k-2}‖²` (curvature-of-input penalty; matches v3.2 `mpc_qp.MPCWeights`)
  - Terminal: `w_term · (n_N² + (s_N − θ_N)²)`
- **Constraints:**
  - Equality: discrete-time dynamics rows from `linearise_stage_c`.
  - Friction ellipse: re-use `mpc_qp_ellipse.build_ellipse_rows` **as-is** (forces are in chassis frame; coordinate basis is irrelevant). Pass the per-stage dynamic Fz arrays from §23.3.6.5.
  - Track edges: `n_min ≤ n_k ≤ n_max` per stage. `track_half_width` from `track.csv_data['track_width']` if present, else fixed 5.0 m. ArchDev resolves the source-of-truth at build time.
  - Actuator limits: `δ_max`, `δ_dot_max`, `throttle/brake ∈ [0, 1]`, `throttle_dot_max`, `brake_dot_max`, `V_θ_min = 0`, `V_θ_max = 1.5 · v_ref_max`. Re-use `MPCBounds` shape.
- **Solver:** OSQP via the existing `mpc_qp` builder utilities. SQP outer loop: 3 iterations max, identical structure to v3.2.

**Touchpoints:** new file. Imports `mpc_qp.MPCBounds`, `mpc_qp.QPProblem` for sparse-matrix helpers. Re-uses `mpc_qp_ellipse.add_ellipse_constraints`.

**Dependencies:** §23.3.6.2.

**Suggested owner:** ArchDev.

### §23.3.6.4 Stage gridding strategy

**What:** MPCC stages are uniform in `dθ` (the virtual progress increment), **not** in arc-length along the chassis trajectory. Default: **`dθ_stage = 2.0 m, N_stages = 25 → horizon = 50 m`** along the reference. Each stage's reference values `(κ_ref_k, v_ref_k, x_ref_k, z_ref_k, ψ_ref_k)` are looked up at `θ_predicted_k = θ_0 + k · dθ_stage`.

Stage time `ts_k = dθ_stage / max(V_θ_k_predicted, V_FLOOR)`. Same explicit-Euler form as v3.2.

**Why uniform in `θ`, not `s`:** the cost is structured around progress along the reference; gridding in `θ` keeps the cost-quadratic stationary across stages (the lookahead always covers the same "amount of reference path"). Gridding in chassis arc length means a single tight corner shrinks the lookahead in reference-frame and the controller suddenly can't see past the apex.

**Touchpoints:** internal to `mpcc_controller.py`.

**Dependencies:** §23.3.6.1.

**Suggested owner:** ArchDev.

### §23.3.6.5 Dynamic-Fz refresh

**What:** Reuse `mpc_model.compute_dynamic_fz_per_stage` unchanged. The `(a_x_seq, a_y_seq)` source is the same as v3.2 Phase 5.0.4: predicted-trajectory values from the previous SQP iteration; first-iteration warm start broadcasts the previous-tick ODE state.

**Touchpoints:** call from inside the MPCC SQP outer loop. No edit to `mpc_model.py`.

**Dependencies:** §23.3.6.2.

**Suggested owner:** ArchDev.

### §23.3.6.6 Controller class + dispatch

**What:** New `MPCCController` in `mpcc_controller.py`. Surface identical to `MPCController` so `_make_controller` adds one branch:

```python
if controller == "mpcc":
    return MPCCController(
        driver, track, car,
        plan=plan, calib=calib, params=params,
        rng_seed=rng_seed,
        # MPCC-specific kwargs (all optional; resolved from driver JSON or CLI):
        horizon_m=mpcc_horizon_m, n_stages=mpcc_n_stages,
        tick_hz=mpcc_tick_hz,
        w_contour=mpcc_w_contour, w_lag=mpcc_w_lag,
        w_progress=mpcc_w_progress, w_du=mpcc_w_du,
        force_static_fz=mpcc_force_static_fz,
    )
```

**Tier-fallback ladder:** MPCC ships with **Tier 0 (clean MPCC commit) + Tier 2 (reactive)** only. Tier 1 (ellipse-saturation feedforward) is **deferred** for MPCC — the curvilinear coordinate transform makes the Tier-1 planned-direction logic (`compute_planned_direction` in `mpc_controller_tiers.py`) harder to keep correct, and the Phase 5.0.5/5.0.6 diagnostics suggest the Tier-1 path is the source of the residual chicane misalignment in v3.2. MPCC's better cost shape should not need it; if MC completions fall short of 7/10, the Tier-1 port is the first fallback (§23.3.11 risk #3).

Tier-2 (reactive sub-controller) trigger thresholds unchanged from v3.2: cross-track > 4 m OR `|ψ_e| > 20°`. Hysteresis-out: 3 consecutive clean MPCC ticks before re-engaging.

**Touchpoints:**
- New file `src/lap_estimator/dynamics/mpcc_controller.py`.
- `src/lap_estimator/dynamics/slip_simulator.py:471` — add the `"mpcc"` branch in `_make_controller`. Update the `choices=(...)` tuple to include `"mpcc"`.
- `lap.py:122-123` — add `"mpcc"` to the `--controller` choices; add the new CLI flags.

**Dependencies:** §23.3.6.1, §23.3.6.2, §23.3.6.3, §23.3.6.4, §23.3.6.5.

**Suggested owner:** ArchDev.

### §23.3.6.7 Diagnostics surface

**What:** Add to `SlipSimResult` the same shape as `mpc_tier_counts` + `mpc_solve_times_s`:
- `mpcc_solve_times_s: list[float]` — per-tick solve time.
- `mpcc_tier_counts: dict[int, int]` — `{0: clean, 2: reactive}`; no `1: ellipse` key (or it stays at 0).
- `mpcc_contour_p95: float`, `mpcc_lag_p95: float`, `mpcc_progress_mean: float` — terminal MPCC-specific diagnostics, surfaced at end-of-run.

`slip_simulator.py:345-348` currently pulls `ctrl.solve_times` and `ctrl.tier_counts`; the MPCC controller exposes the same property names so the existing code-path picks them up without per-controller branching. **New** properties `contour_p95`, `lag_p95`, `progress_mean` are read in a separate optional block guarded by `isinstance(ctrl, MPCCController)`.

Optional **debug trace CSV** at `.tmp/mpcc_trace_<track>_<driver>.csv` gated on `--mpcc-debug-trace`. Columns listed in §23.3.4 user story 5.

**Touchpoints:**
- `slip_simulator.py` — extend `SlipSimResult` (or a sidecar dataclass; ArchDev's call) with the three new fields.
- New `src/lap_estimator/dynamics/mpcc_debug.py` for the trace writer (analogous to the `pi_diag_*` CSVs at `safe_pi_controller.py:log_csv_path`).

**Dependencies:** §23.3.6.6.

**Suggested owner:** ArchDev.

### §23.3.6.8 CLI plumbing

**What:** Add CLI flags to `lap.py`:
- `--controller mpcc` — add to existing `choices`.
- `--mpcc-horizon-m FLOAT` (default 50.0)
- `--mpcc-n-stages INT` (default 25)
- `--mpcc-tick-hz FLOAT` (default 10.0 — matches `DEFAULT_TICK_HZ` after Phase 5.0.3's 50 Hz bump? **No** — research §23.3.5 recommends 10 Hz with longer horizon; the v3.2 tick at 50 Hz is a Phase 5.0.x mitigation for the LTV plant's heading-drift between ticks, which the MPCC's progress-cost formulation does not have the same way. ArchDev re-tunes if the chassis drifts between MPCC ticks; the 10 Hz default comes from Liniger's `Bicycle.json`.)
- `--mpcc-w-contour FLOAT` (default 100.0)
- `--mpcc-w-lag FLOAT` (default 1000.0)
- `--mpcc-w-progress FLOAT` (default 2.0; **positive** in the JSON / CLI, negated inside the controller because the cost subtracts it)
- `--mpcc-w-du FLOAT` (default 1.0)
- `--mpcc-force-static-fz BOOL` (matches `--static-fz` for v3.2)
- `--mpcc-debug-trace` (no-arg flag)

Driver-JSON `control_params.mpcc` block mirrors the same keys with the leading `mpcc.` stripped (so `control_params.mpcc.horizon_m`, `control_params.mpcc.w_contour`, etc.). Resolution priority same as v3.2 mpc block: CLI > driver JSON > dataclass defaults.

**Touchpoints:** `lap.py` arg parser; `slip_simulator.py` arg forwarding.

**Dependencies:** §23.3.6.6.

**Suggested owner:** ArchDev.

---

## §23.3.7 Data & interface contracts

### §23.3.7.1 `ReferencePath` dataclass (§23.3.6.1 output)

```python
@dataclass(frozen=True)
class ReferencePath:
    s_grid: np.ndarray       # (M,) uniform, ds_ref = 0.5 m
    xs: np.ndarray           # (M,)
    zs: np.ndarray           # (M,) — using existing track convention (z = the second planar axis)
    psi_ref: np.ndarray      # (M,) path tangent, rad
    kappa_ref: np.ndarray    # (M,) signed curvature, rad/m; clamped to [-0.2, 0.2]
    v_ref: np.ndarray        # (M,) DP plan speeds, m/s
    track_width: np.ndarray  # (M,) optional; constant if track.csv_data lacks per-sample width
```

### §23.3.7.2 MPCC state & control layout

State vector (NX_C = 10), order frozen at module level as IDX_* constants:
```
x = [s, n, ψ_e, v_x, v_y, ω, δ, throttle, brake, θ]
```

Control vector (NU_C = 4):
```
u = [δ_dot, throttle_dot, brake_dot, V_θ]
```

### §23.3.7.3 Cost weights — initial values

From Liniger's `MPCC/Matlab/MPCC.m` config (`model_params/Bicycle.json` in his repo):

| Weight | Default | Range | Notes |
|---|---|---|---|
| `w_contour` | 100.0 | [10, 500] | Penalises perpendicular path error. |
| `w_lag` | 1000.0 | [200, 5000] | Penalises `s − θ` mismatch. Higher than contour by design — keeps the virtual progress glued to actual progress. |
| `w_progress` | 2.0 | [0.5, 10] | Stored positive; the cost subtracts `w_progress · V_θ_k`. Increase to push for more aggression at the cost of more frequent ellipse-edge / track-edge contact. |
| `w_du` | 1.0 | [0.1, 10] | Input-rate smoothness. |
| `w_du2` | 1.0 | [0.1, 10] | Input-curvature smoothness. Matches v3.2. |
| `w_term` | 100.0 | [50, 500] | Terminal `(n² + (s−θ)²)`. |

ArchDev sweeps these on Sprint A at build time; the lap-time / completion gates in §23.3.9 are what passes. Final values + sweep table go in the architecture doc.

### §23.3.7.4 Cost weights — encoding in OSQP

The QP cost is `½ z^T P z + q^T z`. Contour + lag + terminal contribute to `P` as quadratic forms in `(s, n, θ)` slots; progress reward enters `q` as `q[V_θ_k] = −w_progress` (one row per stage). Input rate/curvature smoothness reuses the v3.2 builder (`mpc_qp._build_du_penalty` if extracted; otherwise inline).

### §23.3.7.5 `MPCCBounds`

Reuses `mpc_qp.MPCBounds` shape with one extra field `v_theta_max: float`. Default `v_theta_max = 1.5 · max(v_ref)` — wide enough that the progress-reward never wants to *exceed* the DP plan's headroom by more than 50 %. ArchDev re-tunes from build-time observation.

### §23.3.7.6 Driver JSON

Optional `control_params.mpcc` block. Schema:
```json
{
  "control_params": {
    "mpcc": {
      "horizon_m": 50.0,
      "n_stages": 25,
      "tick_hz": 10.0,
      "w_contour": 100.0,
      "w_lag": 1000.0,
      "w_progress": 2.0,
      "w_du": 1.0,
      "w_du2": 1.0,
      "w_term": 100.0,
      "dynamic_fz_enabled": true,
      "cg_height_m": null,
      "track_width_f_m": null,
      "track_width_r_m": null
    }
  }
}
```

`null` fields fall through to `CarDynamics` (same pattern as v3.2 `mpc` block).

### §23.3.7.7 `SlipSimResult` additions

```python
mpcc_solve_times_s: list[float] = field(default_factory=list)
mpcc_tier_counts: dict[int, int] = field(default_factory=dict)
mpcc_contour_p95: float = 0.0
mpcc_lag_p95: float = 0.0
mpcc_progress_mean: float = 0.0
```

Existing v3.2 `mpc_*` fields stay; an MPCC run leaves them at default and the new fields populated, and vice-versa.

### §23.3.7.8 Coordinate transform (chassis ⇄ curvilinear)

Forward (chassis `(x, z, ψ)` → curvilinear `(s, n, ψ_e)`):
1. Nearest-point project `(x, z)` onto the reference's `(xs, zs)` arrays — cached hint scan; same algorithm as `mpc_controller_geom.nearest_index`.
2. `s = s_grid[idx] + tangent_alignment_correction` — refined to sub-sample with one Newton step against `dot((p − p_ref(s)), t_ref(s)) = 0`. Standard.
3. `n = signed perpendicular distance` — `n = (p − p_ref) · normal_ref`, where `normal_ref = (−sin ψ_ref, cos ψ_ref)`. Sign convention: `n > 0` ⇒ left of reference.
4. `ψ_e = wrap(ψ − ψ_ref(s))` to `[−π, π]`.

Inverse (curvilinear → chassis, used inside the QP roll-out for diagnostics and for the bicycle dynamics' chassis-frame ellipse constraint):
```
x_chassis = x_ref(s) − n · sin(ψ_ref(s))
z_chassis = z_ref(s) + n · cos(ψ_ref(s))
ψ_chassis = ψ_ref(s) + ψ_e
```

These live in `mpcc_reference.py` as `to_curvilinear(...)` / `to_chassis(...)`.

---

## §23.3.8 Horizon length sweep

Spec a build-time sweep on Sprint A with the default driver `tomas`, 3-lap MC (`--mc 3`) for a cheap signal, then 10-lap MC for the chosen length:

| `horizon_m` | `n_stages` | `dθ_stage` | Expected behaviour |
|---|---|---|---|
| 30 | 15 | 2.0 m | Replicates v3.2's lookahead; expected to under-rotate (research §23.3.5 says 30 m is the existing failure mode). |
| 40 | 20 | 2.0 m | Mid-range; expected to clear the chicane but not maximise straight-line progress. |
| **50** | **25** | **2.0 m** | **Default.** Liniger / TUM Indy regime. Covers Sprint A's longest braking zone (~70 m/s straight → ~15 m/s apex). |
| 60 | 30 | 2.0 m | Stretch. If 50 m passes gates with budget to spare, push to 60. |

Pick the smallest horizon that hits ≥ 7/10 completion **and** mean best lap ≤ 2:00. ArchDev records the sweep in the architecture doc.

---

## §23.3.9 Acceptance gates (§11.55-MPCC)

| Gate | Threshold | Notes |
|---|---|---|
| MC 3-lap completions, Sprint A, chicane_safety_mult ≤ 0.85 | **≥ 7 / 10** | Parity with v3.2 Phase 5.0.4 headline gate. |
| Mean best lap (Sprint A, completed runs) | **≤ 2:00** | ~9 s gain over reactive. **Lap-time gate.** |
| Stretch mean best lap | ≤ 1:55 | Within 7.5 s of Tomas. Bonus. |
| Solve mean | < 30 ms | Same budget as v3.2. |
| Solve p99 | < 50 ms | Same budget as v3.2. |
| Cross-track peak | ≤ 6 m | Tier-2 hysteresis threshold; matches v3.2. |
| Post-solve ellipse violation p95 | ≤ 0.05 | Re-uses Phase 5.0.4 metric. |
| Tier-0 share | ≥ 90 % | Higher than v3.2's 80 % gate — MPCC has no Tier-1, so anything not clean is reactive. |
| Tier-2 share | ≤ 10 % | Hard reactive fallback. |
| MPCC contour_p95 | ≤ 1.5 m | "Within a car width of the reference." |
| MPCC lag_p95 | ≤ 3.0 m | Virtual progress is tracking actual progress. |
| MPCC progress_mean (`V_θ` mean) | within ±15 % of `v_ref` average | Confidence check; if `V_θ` runs > 1.15 × `v_ref` the lag cost is too low; if < 0.85 the progress reward is too low. |
| Regression: `--controller mpc` still ships its existing gates | Pass | MPCC is additive; v3.2 must not regress. |

---

## §23.3.10 Open questions

1. **Tick rate.** v3.2 raised tick to 50 Hz to stabilise the LTV linearisation across phantom-heading-drift between ticks. MPCC's curvilinear cost shape removes the heading-drift symptom (the chassis is rewarded for *progress along the reference*, not for matching a chassis-frame heading reference) — but the underlying issue of LTV staleness still exists. **Resolve at build time:** start at 10 Hz (Liniger default), and if the post-solve ellipse violation p95 exceeds 0.05 or the chassis drifts > 1 m between ticks, raise to 20 Hz or 50 Hz. Document in architecture doc.
2. **Track-edge constraint source.** Does `track.csv_data` carry a per-sample `track_width` column on Sprint A? If not, ArchDev hard-codes `track_half_width = 5.0 m` and notes it in the doc. (Check at build time; this spec is silent because the answer depends on the latest `tracks_csv/ks_nurburgring/...` headers.)
3. **`V_θ` smoothing.** A `V_θ` that jumps stage-to-stage produces a discontinuous lag cost. **Resolve at build time:** if the solve trace shows `V_θ` oscillation > 5 m/s between adjacent stages, add a `w_dvtheta · (V_θ_k − V_θ_{k-1})²` term (same shape as `w_du`).
4. **Slip-target / consistency-noise reuse.** v3.2's `slip_target_rad` modulates `bounds.alpha_axle_max`. MPCC inherits the same friction-ellipse + α-soft constraint, so the same `slip_target_rad` resolution path applies. Consistency-noise application at commit time also reuses `_apply_consistency_noise` byte-for-byte. **Resolved here:** copy the v3.2 logic; no change.
5. **Reference path vs. Tomas trajectory injection.** The research artefact recommends Tomas-trajectory injection first (cheaper, capped at Tomas's line). This spec uses DP plan only. **Deferred to a follow-up spec** that swaps the reference path source; the rest of MPCC is unchanged.

---

## §23.3.11 Risks and mitigations

**Risk 1 — Curvilinear Jacobian singularity at apex.** `ds/dt = (v_x cos ψ_e − v_y sin ψ_e) / (1 − n · κ)`. At high `n · κ` (off-line at a tight corner) the denominator → 0. **Mitigation:** clamp `κ` to `[−0.2, 0.2]` rad/m at reference-build time (§23.3.6.1); clamp `n` inside the QP via `|n| ≤ track_half_width − 0.3`; floor `(1 − n · κ)` at 0.3 in the linearisation (gives a 5 m minimum effective radius and 0.7 minimum geometric factor — well-conditioned).

**Risk 2 — Lag cost pulls θ behind s and chassis goes "backward in θ".** If `θ` drifts behind `s` enough, the controller can in principle command `V_θ < 0` to catch up. **Mitigation:** clip `V_θ ≥ 0` as a hard bound (already in §23.3.6.3). Independently, clip the lag-cost residual `(s − θ)` to be non-negative inside `q` so the cost gradient never pushes the chassis to *slow down* to match `θ`. This is a one-line guard inside the QP build.

**Risk 3 — MPCC under-rotates anyway, like v3.2.** If progress reward + lag cost weighting produces the same symptom, the controller cost isn't the binding constraint and the slip plant or the DP plan is. **Mitigation:** (a) sweep `w_progress` from 2.0 → 10.0 before declaring failure; (b) port the Phase 5.0.3 Tier-1 ellipse-saturation feedforward into MPCC (the `compute_planned_direction` + `emit_ellipse_saturation` logic is coordinate-agnostic; the planned per-axle force history is in chassis frame); (c) if neither helps, the plant model is the culprit and the next step is acados + nonlinear chassis dynamics rather than another controller refactor.

**Risk 4 — OSQP can't carry the curvilinear linearisation at horizon 50.** Liniger uses HPIPM / qpOASES, both of which exploit problem structure better than OSQP. **Mitigation:** (a) start at horizon 30 m, scale up. (b) If OSQP fails feasibility / time gates at 50 m, drop to 40 m. (c) **Escalation path: acados via CasADi front-end.** Spec a separate phase if needed; not in this spec. (d) Keep the SQP outer-iteration cap at 3, same as v3.2.

**Risk 5 — Stage gridding in `dθ` causes the chassis to outrun the reference.** If `V_θ` is small (corner) and the chassis is at high `v_x`, `s` grows faster than `θ` and the lag cost balloons. **Mitigation:** the lag cost itself is the corrective mechanism (it punishes `(s − θ)²`). If the punishment isn't strong enough at the chicane (apex `θ` lags by > 5 m), raise `w_lag` to 2000 or 5000. The default 1000 from Liniger is a starting point; ArchDev tunes.

**Risk 6 — Pedal channel divergence between MPCC and `_long_sub`.** MPCC plans pedals but commits only steering; the actual pedals come from `_long_sub`. The MPCC's internal predicted `v_x_k` therefore drifts from the chassis's true `v_x` over the horizon. **Mitigation:** identical to v3.2's `_long_sub` pattern — the v_x error is absorbed by the `w_lag` term (a slower-than-planned chassis falls behind in `s` and the lag cost corrects on the next tick). The friction ellipse is the safety belt. If diagnostics show > 5 m/s `v_x` mismatch between the MPCC's first stage prediction and the truth at the next tick, that's a sign the longitudinal channel needs MPCC-driven commits — Phase 5.1 territory.

**Risk 7 — `ψ_e` wraparound across the start/finish line.** The s-coordinate wraps; `ψ_ref(s)` is periodic. The chassis can be ahead of the reference by ε then suddenly behind by `track_length − ε`. **Mitigation:** project `s` modulo `track_length` at every tick; treat `(s − θ)` as a signed-shortest-path distance in `[−track_length/2, +track_length/2]`. One helper inside `mpcc_reference.py`.

---

## §23.3.12 Solver / horizon migration path if OSQP plateaus

If OSQP can't carry MPCC at horizon ≥ 40 m within the solve-time gates (§23.3.9), the escalation is:

1. **Reduce horizon to 30 m / 15 stages.** Match v3.2's lookahead. If MPCC at 30 m still beats v3.2 on the lap-time gate, ship at 30 m.
2. **Tighten OSQP settings.** `eps_abs/eps_rel` from 1e-3 → 1e-4 for solution quality; or → 1e-2 for speed. Re-tune at build time.
3. **acados via CasADi.** Separate spec; out of scope here. Re-uses the same `f_continuous_c` and cost terms. acados RTI mode reportedly delivers ~5 ms / tick for racing NMPC; that buys MPCC the full nonlinear dynamics without QP linearisation tax. **Decision criterion:** if MPCC at horizon 30 still doesn't hit the 2:00 lap-time gate, the linearisation tax is binding — escalate to acados.

---

## §23.3.13 Alternatives considered

**(a) More weight tuning on `--controller mpc`.** Four phases of weight tuning produced no structural rotation gain. The reference-tracking cost shape doesn't reward apex-aggression. Rejected.

**(b) acados with same reference-tracking cost.** Solves faster, same wrong cost shape. The bottleneck is formulation, not solver — same conclusion as `.tmp/sota_racing_controllers.md`. Rejected as standalone fix.

**(c) MPPI (sampling-based, no linearisation).** Doesn't under-rotate (samples explore aggressive entries). Step-change architecture: rollout vectorisation, kernel weighting, no SQP/QP reuse. Defer to a separate research line if MPCC plateaus.

**(d) LMPC (lap-over-lap learning).** Best ceiling under the model but needs 10–30 laps of bootstrap. Our MC runs are 1-3 lap; LMPC's terminal-set buildout doesn't fit cleanly. Defer.

**(e) Tomas-trajectory injection (real telemetry as reference).** Cheaper (0.5 day vs. 1-2 day), but capped at Tomas's line. Recommended as a sibling spec — the reference-path plumbing in §23.3.6.1 is already structured so trajectory injection is one factory swap. Not the primary direction here because the goal is "be faster than the reactive baseline by the cost-shape change", which is intrinsic to MPCC.

**(f) Hierarchical MPCC (Liniger 2020, arXiv 2003.04882).** Two-layer planner+tracker. Reports +20 % vs prior SOTA on RC platform. Higher engineering cost; the flat MPCC ships first. Hierarchical is a natural follow-up if flat MPCC plateaus near 1:55.

---

## §23.3.14 References

- Liniger MPCC: https://github.com/alexliniger/MPCC — canonical implementation. `Bicycle.json` for default weights.
- CiMPCC (Curvature-Inspired MPCC, 2025): https://arxiv.org/abs/2502.03695 — +11.4–12.5 % on sharp-curvature tracks.
- Liniger Hierarchical MPCC (2020): https://arxiv.org/abs/2003.04882 — +20 % vs prior SOTA on 1:43 RC.
- Velenis & Tsiotras 2007: trail-braking as minimum-time solution under friction-ellipse constraints. https://www.sciencedirect.com/science/article/abs/pii/S0947358008707751
- TUMFTM `mod_vehicle_dynamics_control`: https://github.com/TUMFTM/mod_vehicle_dynamics_control — used at IAC 270 km/h.
- TUMFTM `global_racetrajectory_optimization`: https://github.com/TUMFTM/global_racetrajectory_optimization — exact-match offline-planner reference (IPOPT + Pacejka + ellipse).
- SOTA research artefact (in this repo, scratch): `.tmp/sota_racing_controllers.md`
- Existing v3.2 entry points referenced throughout:
  - `src/lap_estimator/dynamics/mpc_controller.py`
  - `src/lap_estimator/dynamics/mpc_qp_ellipse.py`
  - `src/lap_estimator/dynamics/mpc_model.py`
  - `src/lap_estimator/dynamics/mpc_controller_geom.py`
  - `src/lap_estimator/dynamics/mpc_controller_tiers.py`
  - `src/lap_estimator/dynamics/longitudinal_planner.py`
  - `src/lap_estimator/dynamics/slip_simulator.py:_make_controller` (line 435)
  - `lap.py --controller` (lines 122-184)
