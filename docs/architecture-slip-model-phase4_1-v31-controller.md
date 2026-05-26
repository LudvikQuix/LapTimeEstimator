# Architecture — v3.1 controller upgrade (Phase 4.1)

Spec source: `dev-planning/lap-simulation-csv-driver/spec-section-23-10-v31-controller.md`
(§23.10.5; replaces v3.0 §23.8 longitudinal half + the `target_scale` heuristic in
`slip_simulator._make_controller`).

Parent: `architecture-slip-model-phase4.md` (the v3.0 controller this section amends).

## What this phase builds

Phase 4.1 swaps the **longitudinal half** of the v3.0 `DriverController` for a
preview-based pre-braking loop and replaces the slip-target formula with an
α_peak-derived mapping. Steering, the Pacejka ODE solver, the racing line, and
the v2 plan as the controller target are all unchanged.

Three behaviour changes ship together:

1. **Preview longitudinal P-controller.** Lookahead is time-based
   (`preview_time_s`, default 1.5 s; horizon = `max(30 m, v_preview·t_preview)`)
   and the controller tracks the **minimum** target speed in that window.
   Brake fires whenever `v > v_target_min` (pre-braking), not just
   `v > v_target_local`. Throttle continues to target the **local** line speed
   (so we still accelerate up to apex speed in cool corners) but is gated off
   when the preview-window min is already below current `v`.
2. **Throttle rate limit** sourced from
   `control_params.throttle_rate_limit_pct_s`, falling back to
   `driver.profile.dynamic.throttle_ramp_pct_s`, then to 1000 (effectively
   unlimited). Applied **before** the slip-band throttle modulators so order
   dependence (spec §23.10.9 risk #3) is fixed.
3. **α_peak-derived slip target.** When `pacejka_calibration` is present on the
   driver JSON, the new `slip_target_deg = α_peak_front_deg * (0.5 + 0.5 *
   skill_pct)` replaces the v3.0 `6·skill + 1·(1 - skill)` formula. α_peak is
   computed by numerical argmax of `pacejka_lateral` over α ∈ [0, 15°] at a
   representative Fz (4000 N), clamped to [4°, 10°] before the skill mapping
   (risk #2 in spec §23.10.9: some fits land E ≈ -2 with peak at the curve
   edge). Computed lazily on first call to `Driver.derived_slip_target_deg()`
   and cached on the Driver instance + written into
   `raw['pacejka_calibration']['source']['alpha_peak_front_deg']` so future
   reads skip the recompute. **No refit required.**

The v3.0 `target_scale` band-aid in `slip_simulator._make_controller`
(`sqrt(D_per_Fz / mu_v2)` clamped to [0.70, 1.00]) is **deleted**.
`target_scale` is hard-coded `1.0`; the `DriverController.target_speed_scale`
kwarg stays as `DeprecatedKwarg` for ABI compat (removal scheduled for v3.2).
The v3.0 in-controller `slip_ratio > 0.85` feedforward speed-target clamp
(lines 270-291 of `driver_controller.py`) is also removed — pre-braking
handles the planning role honestly. The slip-band throttle modulators
(UNDER / OVER / HARD_CAP at slip-ratio 0.9 / 1.1 / 1.3) **stay** as
post-hoc safety.

## Why this architecture

- **Pre-braking is the single biggest expected improvement** (spec §23.10.4).
  The v3.0 reactive P-controller only braked after `v > v_target_local`, by
  which time the brake point had already passed. A 1.5 s preview window with
  a `min` reduction is structurally the same kind of lookahead v2's distance-
  marched optimizer enjoys, on top of honest Pacejka physics.
- **α_peak from a numerical argmax handles non-zero E.** The closed-form
  `tan(π/(2C))/B` approximation in the v3.0 driver code under-estimates the
  true peak when E ≠ 0 (Tomas's fit has E ≈ -0.65, peak shifts ~20% higher).
  Numerical argmax is robust and cheap (one 601-point evaluation per
  driver-load).
- **Clamping α_peak to [4°, 10°]** keeps the controller in a sane band when
  a fit lands at an edge of the search space (spec §23.10.9 risk #2). Tomas's
  fit yields α_peak ≈ 6.9° — well inside the clamp.
- **Removing `target_scale`** unblocks the controller from the ~7-8 s tax it
  was paying for the heuristic. Once the controller tracks the v2 plan
  honestly (preview + α_peak + slip-band safety), the 0.85x scale is no
  longer needed; it was a band-aid for poor tracking.
- **Throttle rate limit ordered before slip-band modulators** prevents the
  rate-limited output from being multiplied back up by an unrelated
  slip-modulator branch. Order matters; spec §23.10.9 risk #3 calls this
  out explicitly.

## Data flow

```
Driver JSON ───────────────────────────────────────────────────────────────┐
  control_params: preview_time_s, throttle_rate_limit_pct_s, gains, ...    │
  profile.dynamic.throttle_ramp_pct_s (fallback for rate-limit)            │
  pacejka_calibration.front.lateral (B, C, D_per_Fz, E) ──────┐            │
                                                              ▼            ▼
                                         Driver._alpha_peak_front_deg() ControlParams.from_driver()
                                                              │            │
                                                              ▼            │
                                         derived_slip_target_deg() ────────┤
                                                              │            │
                                                              ▼            ▼
slip_simulator._make_controller (target_scale = 1.0)  →  DriverController(...)
                                                              │
                          per-step state from solver ────────►│
                                                              ▼
                          1. Stanley steering (unchanged)
                          2. Preview window: v_preview = max(v, v_local),
                             horizon = max(30, v_preview * preview_time_s)
                          3. v_target_min = min target speed in window
                          4. Throttle: P-gain on (v_local - v); gated off
                             when v_target_min < v
                             Brake:    P-gain on (v_target_min - v)
                          5. Throttle rate-limit (last_throttle ± step)
                          6. Slip-band modulators (UNDER/OVER/HARD_CAP)
                          7. Trail-brake taper, consistency noise,
                             ghost-fallback for spin / panic slip
                                                              │
                                                              ▼
                                                          Controls(steer, throttle, brake)
```

## File inventory

| File | Change |
|---|---|
| `src/lap_estimator/dynamics/_control_params.py` | Added `preview_time_s` (default 1.5) and `throttle_rate_limit_pct_s` (optional) to dataclass + `from_driver` + `with_slip_target` copy ctor. |
| `src/lap_estimator/driver.py` | Replaced `derived_slip_target_deg()` body with α_peak-derived mapping. New private helper `_alpha_peak_front_deg()` (lazy + cached). New module-level helper `_argmax_alpha_peak_deg()` (numerical argmax over [0, 15°] α grid). |
| `src/lap_estimator/dynamics/driver_controller.py` | Replaced the longitudinal half of `controls()`: `_min_speed_within` lookahead now `max(30, v_preview · t_preview)`; brake fires on `v > v_target_min`; throttle targets local line speed and is gated off when `v_target_min < v`; throttle rate limit applied before slip-band modulators; the `slip_ratio > 0.85` feedforward speed-target clamp removed. Added `_last_throttle` rate-limit state. Steering channel unchanged. Module docstring + `__init__` docstring updated; `target_speed_scale` marked `DeprecatedKwarg`. |
| `src/lap_estimator/dynamics/slip_simulator.py` | `_make_controller`: hard-coded `target_scale = 1.0`; deleted the `sqrt(D_per_Fz / mu_v2)` heuristic. Ghost path still uses 0.85 (Phase 3 calibration retained). Docstring updated. |
| `docs/architecture-slip-model-phase4_1-v31-controller.md` | This file. |

`fit_driver.py`, the Pacejka fit pipeline, the ODE solver, the racing line,
and driver JSONs on disk are **unchanged**. α_peak fields are written into
the in-memory `Driver.raw['pacejka_calibration']['source']` but are not
persisted back to disk (Phase 4.1 ships without a fit-pipeline change; the
spec §23.10.5.3 notes that next fit will emit them).

## Integration with neighbouring features

- **Pacejka fit (Phase 2, `pacejka_fit.py`).** Consumes `B, C, D_per_Fz, E`
  from the fit verbatim. No refit required.
- **ODE solver (Phase 3, `solver.simulate_slip_lap`).** Consumes
  `controller.controls(state, t)` exactly as before; only the closure body
  changed.
- **v2 simulator (`simulator.simulate`).** Still produces the speed plan
  the controller tracks. Phase 4.1 does **not** introduce a v3 longitudinal
  planner — that is Phase 4.2 (DP planner against the Pacejka envelope,
  triggered if 4.1 misses ±3 s).
- **`simulate_stint_slip` / per-lap state passthrough.** Unchanged. The
  per-lap controller is replaced; the inter-lap state machinery is not.
- **Web UI (§22).** No surface change. Phase 4.2 (if it ships) adds an
  optional plan-source picker.
- **Ghost-driver fallback.** Still in place; controllers that spin or hit
  `slip_ratio > 3` for a step hand off to `GhostDriver` for that step.

## Phase 4.1 validation result (2026-05-18, Tomas Sprint A `--skill 1.0`)

The acceptance gate §23.10.5.4 (replaces §11.55):

- **±3 s of real 1:47.56 → target 1:44.5 to 1:50.5:** FAIL — the lap does
  not complete. The controller drives the car off-track at distance ≈
  889 m in the chicane after attempting to track the v2 plan's apex speed
  (~65 km/h) which is more aggressive than the measured Pacejka envelope
  can deliver (Tomas's real lake recording shows ~44 km/h at the same point).
- **util_p85 ≤ 1.05:** N/A — lap aborts before sufficient data accumulates;
  reported value 2.89 is for the aborted prefix and reflects the slip-panic
  excursion that triggers the off-track.
- **Ghost-fallback ≤ 20 per lap:** N/A — lap aborts after one fallback
  cluster.
- **MC σ ≤ 0.8 s:** Cannot compute (10 / 10 MC runs abort).
- **§11.55.A `target_scale = 1.0` hard-coded:** PASS — confirmed by
  inspection of `slip_simulator._make_controller`, and behaviour: at
  skill=0.5 the lap completes at 2:23.5 with `util_p85 = 0.50` (a slower
  v2 plan is feasible, controller tracks it without slip excursions).
- **§11.55.B `--skill-pct 0.5` ≥ 3 s slower than `--skill 1.0`:** Vacuously
  PASS in the sense that skill=0.5 completes (2:23.5) while skill=1.0 aborts;
  but a clean comparison isn't possible.

**Root cause** (spec §23.10.9 risk #1, anticipated). The v2 plan is more
aggressive than the measured Pacejka envelope. v2's grip circle assumes
`mu_v2 ≈ 1.20` but the Phase-2 fit yields `D_lat ≈ 1.03`. v2 plans an
apex of 65 km/h at the chicane; the real driver did 44 km/h; the
Pacejka envelope agrees with the real driver. Phase 4.1's preview
controller brakes on time but can't *make* the apex once it gets there
with the v2 plan as target — the front tyres saturate (slip_ratio > 3),
the ghost fallback engages, and the chassis still drifts wide enough
to trip the 50 m off-track abort.

**FAIL ACTION (per spec §23.10.5.4):** lap is outside ±5 s (does not
complete). Stop, report root cause, and do not iterate — the spec
prescribes Phase 4.2 (longitudinal DP planner against the fitted
Pacejka envelope, §23.10.6) as the next step. Phase 4.1 ships as
specified; the gap is structural, not a controller-tuning bug.
