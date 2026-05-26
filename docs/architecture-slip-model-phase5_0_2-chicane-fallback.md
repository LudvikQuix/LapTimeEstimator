# Architecture — v3 Phase 5.0.2: chicane-safety planner cap

**Spec brief:** Phase 5.0.2 (in-conversation; supplements §23.2-5.0.2 in `spec-section-23-2-v32-mpc-phase5_0_1.md`)
**Branch:** `feature/sc-71955/lap-simulation`
**Status:** Shipped. **Reactive controller passes the Sprint A chicane** (default 0.80 cap). **MPC remains structurally broken** independent of the planner — see results table.

## What this ships

A dumb, robust, planner-side speed cap for tight-radius segments. The v3 DP planner (`longitudinal_planner.plan_longitudinal`) already builds an honest `v_max(s)` against the fitted Pacejka envelope — but the apex of the Sprint A chicane (r ≈ 27 m) sits right at the friction limit, leaving the controller with effectively zero longitudinal headroom for transient correction. Every controller iteration to date (Phase 4.2 reactive, Phase 5.0 / 5.0.1 MPC) aborts there with `OffTrackError`.

This phase adds:

- A new small module `src/lap_estimator/dynamics/chicane_safety.py` (~200 lines incl docstrings) defining:
  - `ChicaneSafetyConfig(radius_thresh_m=60.0, safety_mult=0.80, ramp_segments=5)` — frozen dataclass with `from_driver` / `resolve` classmethods (CLI > driver JSON > defaults).
  - `ChicaneSafetyReport` — one-line `fmt_line()` for the construction-time log.
  - `apply_chicane_cap(distances, v_corner, kappa, config)` — pure function that returns a NEW capped `v_corner` and a report.
- A `chicane_config` kwarg threaded through `plan_longitudinal` → `slip_simulator._build_dp_plan` / `_build_plan` / `simulate_slip` and surfaced as two CLI flags on `lap.py`:
  - `--chicane-safety-mult` (overrides JSON, default 0.80)
  - `--chicane-radius-thresh` (overrides JSON, default 60.0 m)
- A new optional driver-JSON block `control_params.chicane = {radius_thresh_m, safety_mult, ramp_segments}`. Existing JSONs without it use defaults; backwards-compatible.
- A one-line summary printed at planner construction:

  ```
  Chicane-safety: 316 segments flagged (s=[617..698 m], [809..819 m], ...), safety_mult=0.80, v_max cap min = 12.9 m/s.
  ```

`longitudinal_planner.py` grows by ~25 lines (new kwargs + one call site after Pass 1). Well under the 500-line soft cap (now 290 lines). MPC controller and reactive `DriverController` are **untouched** — this is plan-side only.

## Why a planner change and not a controller change

The MPC and reactive controllers both consume `LongitudinalPlan.speeds` as their target. Reducing that target in tight zones is the smallest possible intervention that gives downstream controllers more feasibility room:

1. **The plan is the controller's ground truth.** It already encodes everything the v3 model knows about the tyre envelope. Loosening it in tight zones is mathematically equivalent to "tell the controller it can go even slower here than the friction limit suggests."
2. **No controller logic changes.** The chicane fallback is invisible to `driver_controller.py` and `mpc_controller.py`. They just see a slightly lower `v_target` in the flagged segments.
3. **Symmetric with existing knobs.** `safety_margin` (Phase 5.0.1 default 0.94) is a global multiplier; `chicane_safety_mult` (default 0.80) is the same multiplier but localised to high-curvature zones. Same mechanism, different scope.
4. **Reversible and disable-able.** Pass `--chicane-safety-mult 1.0` to recover Phase 5.0.1 behaviour byte-for-byte.

## Detection rule + multiplier

A sample `i` is flagged "tight" iff `|kappa[i]| >= 1 / radius_thresh_m`. The default `radius_thresh_m = 60.0 m` catches the Sprint A chicane (r ≈ 27 m) with comfortable margin and also catches three other sub-50 m clusters on the layout (s ≈ 1040–1140, 1551–1650, 3255–3320).

The multiplier is applied as a **ramped** field, not a step:

```
For each sample i:
  dist[i] = number of samples between i and the nearest flagged sample
            (one-sided 1-D distance transform; forward + backward sweep)
  mult[i] = safety_mult + (1 - safety_mult) * min(dist[i], ramp) / ramp
v_capped[i] = v_corner[i] * mult[i]
```

So:
- A sample at the centre of a flagged cluster gets `mult = safety_mult` (0.80 default).
- A sample exactly `ramp_segments` away (default 5 samples ≈ 9.5 m on Sprint A) gets `mult = 1.0`.
- Two adjacent clusters merge into a single conservative valley (the min over the distance transform), rather than producing a notched profile.

The cap is applied **before** the backward / forward feasibility sweeps in `plan_longitudinal`. This is deliberate: the brake-feasibility walk into the chicane and the throttle-feasibility walk out of it both consume the ramped cap as the per-point `v_corner`, so the resulting plan smoothly decelerates into the cap and accelerates out — no `v_max` step discontinuity for the controller to fight.

## Data flow

```
                track CSV (distance_m, radius_m)
                              │
                              ▼
              plan_longitudinal(car, track, calib,
                                safety_margin=0.94,
                                chicane_config=cfg)
                              │
                              ▼
                Pass 1: v_corner = sqrt(D_lat·g·r)
                              │
                              ▼  apply_chicane_cap(d, v_corner, |kappa|, cfg)
                              │       (per-sample ramped multiplier)
                              ▼
                v_corner_capped, ChicaneSafetyReport
                              │
                              ▼
                Pass 2: backward brake-feasibility   (sees ramped cap)
                              │
                              ▼
                Pass 3: forward throttle-feasibility  (sees ramped cap)
                              │
                              ▼
                speeds = v_plan * safety_margin
                              │
                              ▼
               LongitudinalPlan(distances, speeds, chicane_report)
```

CLI → config resolution:

```
lap.py
  --chicane-safety-mult M
  --chicane-radius-thresh R
                            │
                            ▼
ChicaneSafetyConfig.resolve(driver,
                            cli_safety_mult=M,
                            cli_radius_thresh_m=R)
                            │
                            │  precedence: CLI > driver JSON > defaults
                            ▼
                       config passed to plan_longitudinal
```

Driver JSON shape (optional, backwards-compat):

```json
{
  "control_params": {
    "chicane": {
      "radius_thresh_m": 60.0,
      "safety_mult": 0.80,
      "ramp_segments": 5
    }
  }
}
```

## File inventory

| File | Action | Why |
|---|---|---|
| `src/lap_estimator/dynamics/chicane_safety.py` | new (~200 lines) | The fallback config + detector + ramp builder + report formatter. Self-contained, only imports numpy. |
| `src/lap_estimator/dynamics/longitudinal_planner.py` | modified (+~25 lines) | Adds `chicane_config` / `log_chicane` kwargs to `plan_longitudinal`, calls `apply_chicane_cap` between Pass 1 and Pass 2, prints summary, stores `ChicaneSafetyReport` on the returned `LongitudinalPlan`. |
| `src/lap_estimator/dynamics/slip_simulator.py` | modified | Threads `chicane_config` through `simulate_slip` → `_build_plan` → `_build_dp_plan` → `plan_longitudinal`; resolves from driver JSON when caller passes `None`. |
| `lap.py` | modified | Two new CLI flags `--chicane-safety-mult`, `--chicane-radius-thresh`; builds `ChicaneSafetyConfig` and passes it to `simulate_slip`. |
| `docs/architecture-slip-model-phase5_0_2-chicane-fallback.md` | new (this file) | Architecture note. |

No changes to: `driver_controller.py`, `mpc_controller.py`, `mpc_model.py`, `mpc_qp.py`, `mpc_qp_ellipse.py`, `solver.py`, `vehicle.py`, `pacejka.py`, `pacejka_fit.py`, the Pacejka calibration block format, any tyre / setup / track / car module.

## Results — Phase 5.0.2 vs Phase 5.0.1 (Tomas / Sprint A / skill=1.0)

Default chicane settings: `radius_thresh=60 m`, `safety_mult=0.80`, `ramp_segments=5`.

| Configuration | Lap 1 (representative) | Abort point | MC completions | Phase 5.0.1 baseline |
|---|---|---|---|---|
| Reactive, **no** chicane cap (`--chicane-safety-mult 1.0`) | aborts | s ≈ 658 m (chicane apex) | 0 / 10 | matches §23.2-5.0.1.5 |
| **Reactive, default cap (0.80)** | aborts at downstream corner | s ≈ 1087 m (different corner) | 0 / 10 | **chicane survived; new bottleneck** |
| **MPC, default cap (0.80), 1 lap** | **2:15.14** | (representative finished) | ~1 / 10 (median lap reported, σ=0.077) | Phase 5.0.1 was 0 / 10 |
| MPC, default cap (0.80), 3 laps | aborts on lap 2 | s ≈ 599–660 m (chicane entry / apex) | 0 / 10 | matches Phase 5.0.1 0 / 10 |
| MPC, cap 0.60, 3 laps | aborts on lap 2 | s ≈ 599 m | 0 / 10 | no improvement |

### What this means

- **Reactive controller, chicane apex (s ≈ 658) is solved.** The headline failure point — present in every controller iteration since Phase 4.2 — no longer fires. The new failure point is a separate ≈ 40 m-radius sustained corner at s ≈ 1087 m where `util_p85 ≈ 0.30` (well below tyre peak) and the cap further reduces target speed — but the **lateral** controller still loses the racing line. This is a steering-tracking bug, not a speed bug, and is **out of scope** for a planner-side fallback.
- **MPC remains structurally broken at the chicane.** MPC aborts fire upstream of the cap's ramp-in zone (s ≈ 599–660) with `infeasible QP after slip bump`. The MPC's QP is detecting infeasibility based on its slip-budget soft constraint and falling back to the ghost driver, which then mis-tracks the line. Increasing the cap to 0.60 made things worse, not better — the MPC's preview window (~30 m at 25 m/s) is wider than the 5-segment ramp (~9.5 m), so the cap arrives "suddenly" in the MPC's plan and the SQP fails to converge.
- **The §11.55-5.0.1 must-pass completion gate (7 / 10 MC seeds on Sprint A 3-lap) is NOT met.** Phase 5.0.2 closes the *named* Sprint A chicane abort (s = 658) for the reactive controller, but does not solve MPC infeasibility, and exposes a separate downstream corner that defeats the reactive controller laterally.

### Per the spec

> The lap-time target is NOT ±5 s of real here. It is "≥7/10 completions on Sprint A."

Phase 5.0.2 does not hit this. The remaining problems are controller-side (MPC ellipse / Pacejka linearisation per §23.2-5.0.1.3 / .4, and a separate steering-tracking failure at s ≈ 1090) and explicitly out of scope for this planner-only patch.

## Integration with neighbouring features

- **Phase 4.2 (`architecture-slip-model-phase4_2-v31-dp-planner.md`):** The chicane cap is composed multiplicatively with `safety_margin`. With both defaults active, the effective speed at a flagged apex is `v_corner * 0.80 * 0.94 = v_corner * 0.752` (24.8 % below the friction envelope). The pre-Phase-5.0.1 historical `safety_margin=0.97` path is still reachable; the chicane cap stacks on top.
- **Phase 5.0 / 5.0.1 (MPC, `architecture-slip-model-phase5_0-v32-mpc.md`, `..._phase5_0_1...md`):** No MPC code changed. The MPC sees a lower `LongitudinalPlan.speeds` in flagged zones; everything else (operating-point Pacejka linearisation, ellipse hard constraint, slip-budget slack) is unchanged. Phase 5.0.2 is layered onto Phase 5.0.1, not a replacement.
- **Reactive `DriverController` (`driver_controller.py`):** No code changed. Same plan-consumption path as v3.1.
- **Driver JSON schema:** Additive. Existing JSONs without `control_params.chicane` work verbatim. The chicane block sits alongside the other `control_params` fields (preview / gains / softener band).
- **Web UI (§22):** Not touched. A future UI pass could surface the two CLI knobs as a "Chicane safety" panel.

## Future work

This is intentionally a fallback. Once the controllers (especially MPC) can survive the friction-limit chicane on their own, the chicane cap can be relaxed (`safety_mult → 1.0`) to recover lap time. The real fixes live in:

1. **MPC structural** — operating-point Pacejka linearisation (§23.2-5.0.1.3), true friction-ellipse hard constraint (§23.2-5.0.1.4). Phase 5.0.2 explicitly does not touch these.
2. **Lateral steering tracker** — the s ≈ 1090 m abort for the reactive controller. `util_p85 ≈ 0.30` with a 0.65 chicane cap shows the car is not grip-limited; the Stanley-style preview tracker is failing to hold a 40 m-radius sustained corner. Separate root cause; not addressed here.
