# CiMPCC Curvature-Weighted Velocity Cost Overlay (Phase 5.0.8)

**Status:** Shipped. Default OFF (`--cimpcc-weight None` → bit-identical to
pre-5.0.8 MPC). Opt-in via CLI / driver JSON.

**Branch:** `feature/sc-71955/lap-simulation`.

**Reference paper:** *Curvature-Inspired MPCC* (Wang et al., 2025,
arXiv:2502.03695). The paper reports an 11.8 % lap-time gain over baseline
MPCC on tracks with sharp turns; the term we ship here is the lower half
of their cost — a constant-weight version of the asymmetric speed-curvature
hinge, with the load-aware interpolation between aggressive and
conservative reserved for a later phase.

## Summary

Adds a soft asymmetric penalty
`w_κ · max(0, v_x[k] − v_κ_safe(κ[k]))²` to each MPC stage's cost,
encoded as a slack-variable hinge:

- `v_κ_safe(κ) = sqrt(D_lat · g / max(|κ|, ε) · safety_κ)` — the
  locally-safe speed given the upcoming curvature on the racing-line.
- Per-stage slack `s_v[k] ≥ 0` with `v_x[k] − s_v[k] ≤ v_κ_safe[k]`.
- Cost `w_κ · s_v[k]²` on the slack only — the asymmetry comes from the
  one-sided constraint plus the slack's `≥ 0` bound.

No penalty fires on straights (`|κ| < 1e-3` → `v_κ_safe = +∞`, hinge row
dropped) or when the controller is going slow into a corner. Only fires
when the controller carries excess speed into a turn — directly
attacking the spec's "1.3-1.5 m/s hot into chicane" abort mode.

## Architecture decision

**Approach: additive overlay module, not a `MPCWeights` extension.**

The parallel Phase 5.0.7 ArchDev was already editing `MPCWeights` and
`mpc_qp.py` for QP-tune work; adding a new field to `MPCWeights` would
have collided. Instead, CiMPCC ships as `mpc_qp_cimpcc.py`, mirroring
the established pattern of `mpc_qp_ellipse.py`:

- One file owns the slack-encoding + constraint rows.
- `solve_sqp` accepts an optional `cimpcc_params=CiMPCCParams(...)`
  kwarg; when `None` or `enabled=False`, the QP is bit-identical to
  pre-5.0.8.
- The MPC controller resolves `CiMPCCParams` from (CLI > driver JSON >
  default-disabled) once at construction and passes it to every
  `solve_sqp` call.

This keeps blast radius small: 1 new file (~230 LoC), and additive edits
across `mpc_qp.py` (~25 LoC), `mpc_controller.py` (~25 LoC),
`slip_simulator.py` (~10 LoC plumbing), `lap.py` (~22 LoC CLI).

## Math

For each stage `k = 0..N-1` with reference curvature `κ[k]`:

```
v_κ_safe[k] = √(D_lat · g / max(|κ[k]|, 1e-3) · safety_κ)

L_cimpcc[k] = w_κ · max(0, v_x[k+1] − v_κ_safe[k])²
```

`v_x[k+1]` is the END-of-stage longitudinal velocity (state propagator
`Phi[k+1]` * u + g[k+1]), matching the running-cost-loop convention in
`build_qp`. Curvature `κ[k]` is the START-of-stage value — the curvature
the controller is committing to over the stage.

The hinge is encoded with one slack variable per stage:

```
0 ≤ s_v[k]
v_x[k+1] − s_v[k] ≤ v_κ_safe[k]
```

with quadratic cost `w_κ · s_v[k]²`. At the unconstrained optimum,
`s_v[k] = max(0, v_x[k+1] − v_κ_safe[k])`, reproducing the asymmetric
hinge exactly (the slack's `≥ 0` lower bound carries the `max(0, ·)`
clip).

## Data flow

```
─────────────────────────────────────────────────────────────────────
lap.py CLI
  --cimpcc-weight FLOAT   (default None → overlay DISABLED)
  --cimpcc-safety FLOAT   (default None → 0.95)
       │
       ▼
_run_slip_model() → simulate_slip(
    mpc_cimpcc_weight=...,
    mpc_cimpcc_safety=...,
)
       │
       ▼
_run_single / _run_monte_carlo → _make_controller(
    mpc_cimpcc_weight=...,
    mpc_cimpcc_safety=...,
)
       │
       ▼
MPCController(
    cimpcc_weight=...,   # CLI > driver JSON cimpcc.weight > 0.0
    cimpcc_safety=...,   # CLI > driver JSON cimpcc.safety > 0.95
)
       │  (resolves once at construction)
       ▼
self.cimpcc_params = CiMPCCParams(
    enabled=(weight > 0),
    weight, safety,
)
       │
       │  ... per MPC tick ...
       ▼
solve_sqp(..., cimpcc_params=self.cimpcc_params)
       │
       │  ... per SQP iteration ...
       ▼
build_qp()  →  add_alpha_constraints()  →  add_ellipse_constraints()
                                                 │
                                                 ▼
                                  add_cimpcc_curvature_cost(
                                      problem, stages, x0,
                                      kappa_seq=kappa_seq,
                                      mu_lat=pc.D_lat_front,
                                      params=cimpcc_active,
                                  )
                                                 │  if enabled:
                                                 │    - extend P, q (+N slack cols)
                                                 │    - extend A (zero-pad existing
                                                 │      rows by N cols)
                                                 │    - append 2N new rows
                                                 │      (hinge + nonneg per stage)
                                                 ▼
                                              QPProblem
                                              (n_decision = old + N)
                                                 │
                                                 ▼
                                              solve_qp() → OSQP
─────────────────────────────────────────────────────────────────────
```

The MPC controller only reads the u-prefix (first `N * NU` entries) of
the decision vector after `solve_qp` returns; the trailing slack columns
are transparent.

## File inventory

| File | Change | LoC | Why |
|---|---|---|---|
| `src/lap_estimator/dynamics/mpc_qp_cimpcc.py` | NEW | +234 | The overlay module. `CiMPCCParams`, `compute_v_kappa_safe`, `add_cimpcc_curvature_cost`. |
| `src/lap_estimator/dynamics/mpc_qp.py` | modify | +25 | `solve_sqp` accepts `cimpcc_params`, calls `add_cimpcc_curvature_cost` after `add_ellipse_constraints`, warm-start widened by N when active. |
| `src/lap_estimator/dynamics/mpc_controller.py` | modify | +28 | New constructor kwargs `cimpcc_weight`, `cimpcc_safety`; resolves driver JSON `mpc.cimpcc` block; builds `self.cimpcc_params`; threads to `solve_sqp`. |
| `src/lap_estimator/dynamics/slip_simulator.py` | modify | +12 | `simulate_slip`, `_run_single`, `_run_monte_carlo`, `_make_controller` thread the two new kwargs through. |
| `lap.py` | modify | +22 | Two new CLI flags `--cimpcc-weight`, `--cimpcc-safety`; wiring into `simulate_slip` kwargs. |

Total: +321 LoC (1 new file, 4 modified files). Well under the 500-line
soft cap.

## Integration with neighboring features

- **Phase 5.0.1 ellipse hard constraint** (`mpc_qp_ellipse.py`): CiMPCC
  is appended AFTER the ellipse, on the same `QPProblem`. The ellipse
  constraints are zero-padded by `N` columns to match the new
  `n_decision`. Composition is straightforward — both are linear
  constraints; the QP is still convex.
- **Phase 5.0.3 tier-1 escalation** (`mpc_controller_tiers.py`): CiMPCC
  acts purely inside the MPC (Tier 0). When the controller escalates to
  Tier 1 (ellipse-saturation feedforward) or Tier 2 (reactive
  fallback), the CiMPCC overlay is bypassed because the QP is not
  solved. This matters for the Sprint A chicane: see smoke-test
  results below.
- **Phase 5.0.4 dynamic-Fz**: orthogonal — CiMPCC uses
  `pc.D_lat_front` (the static peak coefficient), not per-stage Fz.
  The product `D_lat * Fz / Fz = D_lat * g_factor` in `v_κ_safe` would
  vary by ~5 % under typical weight-transfer; we treat that as
  second-order tuning material absorbed by `safety_κ`.
- **Phase 5.0.6 Tier-1 thresholds**: orthogonal. The thresholds gate
  escalation; CiMPCC shapes the Tier-0 QP's solution. They can be tuned
  independently.
- **Phase 5.0.7 QP-tune** (parallel ArchDev): no edit-collision —
  CiMPCC lives in a separate file and only touches `solve_sqp`'s
  signature additively.
- **`--controller mpcc`** (Liniger-style MPCC, parallel ArchDev,
  separate new files): no overlap; that controller has its own cost
  structure.

## Smoke-test results

All runs: Tomas, Sprint A, BMW 1M, Semislicks, `--model slip --controller
mpc --inertia-zz 2400 --chicane-safety-mult 0.85`, MC=10 (auto-triggered
by Tomas's `consistency_sigma=1.5`), default tier-1 thresholds.

| Run | `--cimpcc-weight` | Completions | Best lap | Tier shares (0/1/2) | Abort site | MPC mean solve (ms) |
|---|---|---|---|---|---|---|
| Baseline (overlay disabled) | 0 | **0 / 10** | — (all DNF) | 81.6 % / 3.0 % / 15.4 % | s=655 m | 18.06 |
| w_κ = 200 | 200 | **0 / 10** | — (all DNF) | 81.7 % / 3.0 % / 15.3 % | s=654 m | 19.48 |
| w_κ = 500 | 500 | **0 / 10** | — (all DNF) | 81.6 % / 3.0 % / 15.4 % | s=655 m | 19.28 |
| w_κ = 1000 | 1000 | **0 / 10** | — (all DNF) | 80.8 % / 3.0 % / 16.2 % | s=655 m | 18.73 |

`safety_κ` was left at 0.95 for all CiMPCC runs.

**The +1.2-1.4 ms solve-time bump confirms the overlay is being
constructed and solved**, but the lap outcomes are identical to baseline.

## Why CiMPCC alone doesn't close the Sprint A chicane abort

**The failure trigger is upstream of where CiMPCC can act.**

Sprint A chicane apex at s ≈ 657 m, abort at t ≈ 13.4 s. The MPC's
horizon is 30 m (default). At the pre-chicane approach speed (~28 m/s
on a Sprint A braking entry), that's ~1.1 s of lookahead — meaning the
curvature peak first enters the MPC horizon at roughly t ≈ 12.3 s.

But Tier-2 escalation fires at **t ≈ 11.26 s** — almost 1 second
*before* the chicane is in `kappa_seq`. The escalation reason logged in
every run is `tier1-escalation-soft-divergence`: the SQP `||Δu||_∞` and
ellipse-violation soft signals climb during the brake-zone transient
(longitudinal Fx near saturation, e_lat tracking jitter under
deceleration) and trigger Tier-1 saturation feedforward, which then
cascades into Tier-2 reactive fallback.

Once Tier 2 is active, the reactive sub-controller drives — the MPC's
QP (with or without CiMPCC) is not consulted. By the time the chicane
curvature would actually appear in `kappa_seq`, the controller has
already handed off and CiMPCC has no influence.

**The cost term works as designed; the failure is in the wrong layer.**
CiMPCC shapes the QP's intent at corner entry. It does not address the
linearisation-cascade pathology that escalates the controller out of
the QP layer entirely in the brake zone before the corner is visible.

## What WOULD help (recommendations for next phase)

1. **Extend horizon to ≥ 60 m (currently 30 m).** With the chicane in
   view at the brake-point, CiMPCC's hinge would shape the brake
   profile and reduce the longitudinal-Fx saturation that's triggering
   Tier-1 escalation. The docstring at `mpc_controller.py:101-107`
   already notes the 30 m / 80 m discrepancy and recommends 80 m / 20
   stages — that change PLUS CiMPCC is the natural follow-up. Tested
   in isolation in Phase 5.0.x; never combined with CiMPCC.

2. **Address the tier-1 escalation cascade directly.** Per the forum
   research the better intervention is the "softener / steering
   smoother" path or the v3.2 MPC's pre-built MPC-with-MPCC pipeline.
   CiMPCC alone is not in the critical path.

3. **Pair CiMPCC with a planner update.** The DP planner already
   produces `v_ref_seq` that respects curvature — the QP's `w_v` cost
   tracks that. CiMPCC's value is REINFORCING this when the QP would
   otherwise prefer to overshoot. With `w_v = 0.5` (very low — the
   reactive sub-controller drives long), the speed-tracking signal is
   weak and CiMPCC's `w_κ = 500` should dominate. It does on TIER-0
   ticks — the problem is the ticks where the chicane matters are TIER
   2.

## CLI usage

```bash
# Enable with the spec's seed weight (default safety = 0.95):
python lap.py cars_csv/bmw_1m \
  tracks_csv/ks_nurburgring/layout_sprint_a.csv \
  drivers/tomas.json \
  --model slip --controller mpc --single-lap \
  --inertia-zz 2400 \
  --cimpcc-weight 500

# More conservative (rides further below the friction limit):
... --cimpcc-weight 500 --cimpcc-safety 0.85

# Override per-driver default via JSON:
# drivers/<name>.json:
#   "control_params": {
#     "mpc": {
#       "cimpcc": { "weight": 500.0, "safety": 0.95 }
#     }
#   }
```

## Future work

- **Curvature-dependent weight** (paper's full proposal): `w_κ(κ)`
  interpolates between aggressive on straights (`w_κ` low) and
  conservative in tight turns (`w_κ` high). Drop-in extension on
  `CiMPCCParams`; one line in `add_cimpcc_curvature_cost`.
- **Dynamic-Fz coupling**: replace `pc.D_lat_front` with per-stage
  `D_lat_front * Fz_front[k] / Fz_front_static` so the hinge tightens
  under brake-induced front load and loosens under acceleration-induced
  unload.
- **Combined-slip envelope**: replace `D_lat` with the effective
  lateral peak after the longitudinal-axis utilisation is accounted
  for. The Phase 5.0.1 ellipse already encodes this geometrically as a
  hard constraint; doubling it as a soft cost in the velocity
  dimension would catch the asymmetric case where the controller wants
  to brake hard AND turn hard at the same time.
