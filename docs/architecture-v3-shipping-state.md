# Architecture — v3 shipping state (slip ODE + reactive controller + chicane safety cap)

**Status:** Decision recorded 2026-05-22. After 7 consecutive controller iterations (Phase 4.2 through Phase 5.0.5) failed §11.55 at the Sprint A chicane, the **MPC stack is parked**. v3 ships as **slip ODE + reactive controller + Phase 5.0.2 chicane safety cap**. MPC code remains in-tree under `src/lap_estimator/dynamics/mpc_*.py` and is reachable via `--controller mpc` (experimental, not default).

**Headline:** v3 reactive mode is the shipped slip-based path; it completes Sprint B / GP A / GP B as it always has at low utilisation but **aborts on every Nurburgring layout that contains the Sprint-A-style chicane geometry** (s≈670 m chicane on Sprint B / GP A; s≈1087 m chicane on Sprint A / GP B). v2 point-mass (1:48 on Sprint A) is **unchanged** and remains the default for `lap.py`.

## Modes

| Mode | CLI | Status | Use case |
|---|---|---|---|
| v2 point-mass (default) | (no flags / `--model point-mass`) | **Shipped, stable** | Production lap time prediction, stint simulation, inverse-PSI solver, all multi-compound work. ~1:48 on Sprint A. |
| v3 slip + reactive (default slip mode) | `--model slip --controller reactive` | **Shipped, honest** | Slip-aware single-lap prediction with Pacejka envelope + driver controller. Aborts at Sprint-A-style chicanes (see "Known limitations"). |
| v3 slip + MPC | `--model slip --controller mpc` | **EXPERIMENTAL / PARKED** | Receding-horizon MPC research path. 7 phases of work, all failed §11.55. Performs worse than reactive at the Sprint A chicane. Do not use in production. |

## Recommended config per use-case

- **Predict a lap time on a new track (production).** v2 point-mass, default flags:
  ```
  python lap.py cars_csv/<car> tracks_csv/<track>/layout_*.csv drivers/<driver>.json --single-lap
  ```
- **Multi-lap stint with tyre-state evolution.** v2 point-mass, `--laps N`:
  ```
  python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json --laps 3 --compound Semislicks
  ```
- **Validate slip dynamics against the fitted Pacejka envelope (research).** v3 slip + reactive:
  ```
  python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json --model slip --controller reactive --single-lap
  ```
  Expect: lap aborts at s≈1087 m on Sprint A; `util_p85` ≈ 0.5 (low utilisation, controller chatter is the binding constraint, not the tyre envelope).
- **MPC research replay (experimental).** v3 slip + MPC, with the `--mpc-*` knobs from `lap.py --help`. Off the production path; do not regress against.

## Defaults check (2026-05-22)

`lap.py` already carries the correct defaults for the "shipped v3" position. No code changes were required:

| Flag | Default | Confirmed correct |
|---|---|---|
| `--model` | `point-mass` | yes — v2 default; v3 is opt-in via `--model slip` |
| `--controller` | `reactive` | yes — slip-mode controller defaults to reactive |
| `--plan-source` | `v3_dp` | yes — DP planner against the fitted Pacejka envelope |
| `--chicane-safety-mult` | `None` (resolves to `0.80` via `ChicaneSafetyConfig.DEFAULT_SAFETY_MULT`) | yes |
| `--chicane-radius-thresh` | `None` (resolves to `60.0` via `ChicaneSafetyConfig.DEFAULT_RADIUS_THRESH_M`) | yes |

The CLI flags are kept (override via CLI or driver JSON `control_params.chicane.{safety_mult,radius_thresh_m}` still works); only the defaults are pinned to the shipped values.

## Known limitations per mode

### v2 point-mass

No new known limitations. Same caveats as v1.3 / v2.0:
- Per-wheel tyre state but no slip-based dynamics.
- Inverse-PSI solver is a greedy bisection — does not solve for combined wear + lap-time Pareto.
- `tyre_calibration` is car-and-compound-coupled; do not cross-port.

### v3 slip + reactive (the shipped slip path)

**Aborts at Sprint-A-style chicanes.** On the Tomas / Semislicks setup, skill=1.0, single-lap:

| Track | Outcome | Abort point | `util_p85` |
|---|---|---|---|
| Sprint A | abort | s≈1087 m | 0.541 |
| Sprint B | abort | s≈674 m | 1.468 |
| GP A | abort | s≈669 m | 1.471 |
| GP B | abort | s≈1087 m | 0.572 |

Sprint A's abort point (s≈1087 m) corresponds to a ~40 m sustained tight-radius corner; the Stanley preview controller chatters at low utilisation (util_p85 ≈ 0.54, well within tyre peak) and the chassis drifts ≥12 m from the racing line. **Sprint B / GP A abort *earlier* (s≈670 m) at a different geometry**, with util_p85 ≈ 1.47 (47 % over peak). Both abort signatures are *controller* failures, not tyre-envelope violations — the DP plan is feasible but the reactive Stanley + slip-band P-loop cannot track it.

**v2 point-mass + reactive driver completes all four Nurburgring layouts.** v3 reactive is the slip path; it is *not* a regression from v2 because v2 does not run the slip ODE. The "honest" shipping position is: ship v3 reactive as a research-quality slip-aware mode, document the chicane aborts as a known limitation, keep v2 as the production default.

**No `--all-tracks` for slip.** The slip-model entry point in `_run_slip_model` (`lap.py:368`) requires a CSV-backed track and exits early if `kind != "csv"`. The `--all-tracks` flag iterates over `BUILTIN_TRACKS` (synthetic geometries, no CSV); it is only honoured on the point-mass path (`lap.py:244`). Documented intentional asymmetry — do not "fix" without a spec change.

### v3 slip + MPC (parked / experimental)

The MPC stack is structurally working — OSQP solves, SQP outer loop converges, Tier 1/2/0 dispatch fires, post-Phase-5.0.4 dynamic-Fz refresh, post-Phase-5.0.5 Tier-1 FIR smoother. None of it gates §11.55. The binding constraint is **the reactive sub-controller running under Tier 2 (chassis-divergence fallback)** — see `architecture-slip-model-phase5_0_5-v32-tier1-bistability.md` §"What this DOESN'T fix" for the empirical finding.

What would resurrect the MPC work (research direction, not scheduled):
- **Per-wheel Fz plant.** The current MPC inner model uses per-axle Fz with Phase 5.0.4 dynamic refresh. The reactive sub-controller (and the OSQP feasibility check) both implicitly assume axle-symmetric load. The chicane abort signature is consistent with one front wheel saturating combined-slip while the other has headroom — a per-wheel Fz plant would let the controller redistribute brake / steering between sides instead of pulling the axle.
- **Pacejka refit on richer telemetry.** The fitted Pacejka coefficients underwrite both the DP plan and the MPC plant. Current fit is from Tomas's Sprint A + GP A samples; the chicane apex itself is ~5 % of the training distance. A refit weighted toward high-curvature segments would let the planner concede speed at the apex without the controller having to back off transient longitudinal demand.
- **Alternative validation track.** Sprint A's chicane is geometrically a worst-case — 27 m apex radius, 40 m sustained, off-camber. Validating on a track without this geometry (e.g. a Monza-style track with longer-radius turns) would let us measure controller quality independent of the chicane pathology and ship the MPC stack on its actual merits.

## Decision history (7 phases of MPC, why parked)

| Phase | Goal | Outcome | Doc |
|---|---|---|---|
| 4.1 | First v3.1 controller (preview-Stanley + slip-band P-loop) | Lap completes on most tracks; Sprint A chicane abort | `architecture-slip-model-phase4_1-v31-controller.md` |
| 4.2 | Forward-backward DP plan against fitted Pacejka envelope (the "v3_dp" plan source) | DP plan honest; lap 2:04 (+16.6 s vs real); controller now binding | `architecture-slip-model-phase4_2-v31-dp-planner.md` |
| 4.3 | Steering softener — bandlimit + rate-limit on steering | Helps chatter elsewhere; does not unbreak Sprint A | `architecture-slip-model-phase4_3-v31-steering-softener.md` |
| 5.0 | First v3.2 MPC (OSQP inner + SQP outer + Tier 1/2/0 dispatch) | MPC structurally works; Sprint A chicane still aborts | `architecture-slip-model-phase5_0-v32-mpc.md` |
| 5.0.1 | MPC stabilisation fixes (warm-start, ellipse cushion, plan_source v3_dp wiring) | Stabilises MPC; Sprint A still aborts | `architecture-slip-model-phase5_0_1-v32-mpc-fixes.md` |
| 5.0.2 | Planner-side chicane safety cap (radius < 60 m → v_corner × 0.80) | DP plan now has explicit margin at the apex. **Reactive** path with this cap is the shipped v3 (this doc). MPC path still aborts | `architecture-slip-model-phase5_0_2-chicane-fallback.md` |
| 5.0.3 | MPC Tier-1 ellipse-saturation feedforward | Adds tier-1 / tier-2 episode telemetry; chicane abort signature unchanged | `architecture-slip-model-phase5_0_3-v32-tier1.md` |
| 5.0.4 | Dynamic per-axle Fz refresh in MPC plant | Plant more honest; chicane still aborts | `architecture-slip-model-phase5_0_4-v32-load-transfer.md` |
| 5.0.5 | Tier-1 emit FIR smoother (rolling-window bistability damper) | Documented bistability damped ≥3.6×; chicane still aborts because **binding chatter is in Tier-2 reactive sub-controller, not Tier-1 emit** | `architecture-slip-model-phase5_0_5-v32-tier1-bistability.md` |

**Decision (2026-05-22):** Stop iterating on the MPC stack. Ship v3 reactive + chicane safety cap as the honest shipping position. Document the Sprint-A chicane abort as a known limitation. Park the MPC code (do not delete; `--controller mpc` flag still works for experimental use).

## File inventory

This phase is documentation only. Production code is unchanged from Phase 5.0.5.

- **Edited:**
  - `docs/AI_CONTEXT.md` — added v3 shipping-state section, demoted MPC to experimental.
  - `README.md` — added v3 slip path to Quick Start, documented `--model slip --controller reactive` as the shipped slip path.
- **Created:**
  - `docs/architecture-v3-shipping-state.md` — this doc.
- **Unchanged (production code):**
  - `lap.py` — defaults already correct (verified line-by-line).
  - `src/lap_estimator/dynamics/chicane_safety.py` — `DEFAULT_RADIUS_THRESH_M=60.0`, `DEFAULT_SAFETY_MULT=0.80` already match the shipped position.
  - `src/lap_estimator/dynamics/mpc_*.py` — left in place, reachable via `--controller mpc` for experimental use.

## Smoke-test record (2026-05-22)

Reactive Sprint A (`python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json --model slip --controller reactive --single-lap`):
- Lap: aborted at t=36.420 s, s≈1087 m, chassis 12.0 m from racing line.
- `util_p85 = 0.541` (slip_target = 6.99 deg, within tyre peak).
- MC sigma = 0.000 s (10 runs, all aborted at similar s; `--single-lap` defaults to the 10-run MC variant for slip).
- Ghost fallbacks: 0 (no MPC, no MPC-ghost path).
- Chicane-safety: 316 segments flagged, `v_max` cap min = 12.9 m/s.

Reactive multi-track sweep (single-lap, default flags):

| Track | Abort point | `util_p85` |
|---|---|---|
| `layout_sprint_a.csv` | s≈1087 m | 0.541 |
| `layout_sprint_b.csv` | s≈674 m | 1.468 |
| `layout_gp_a.csv` | s≈669 m | 1.471 |
| `layout_gp_b.csv` | s≈1087 m | 0.572 |

**This is wider than the Sprint-A-only limitation noted in the shipping decision.** Sprint B and GP A abort earlier (s≈670 m) at a different geometry, with util_p85 above peak (1.47, 47 % over). Honest report: v3 reactive does not complete any tested Nurburgring layout on the Tomas / Semislicks setup. Phase recommendation (per the user's "report honestly and STOP" instruction): do **not** attempt to fix inline — the point of this shipping pass is to record the honest position.

v2 point-mass regression check (`python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json --model point-mass --single-lap`):
- Lap: **1:48.055** on Sprint A. Matches the v2 baseline of ~1:48. **No regression** from the MPC iteration work.

## Integration with neighbouring features

- **v2 point-mass (`docs/architecture-lap-simulation-stint-v2.md`):** unchanged. Same `Car`/`Driver`/`Track`/`StintResult` stack. The v3 slip path is a side-by-side dispatch in `lap.py` (`_run_slip_model`), not a replacement.
- **Config pipeline (`docs/architecture-config-pipeline.md`):** unchanged. v3 slip reuses the same five config sources (car ini, track CSV, driver JSON, optional setup JSON, CLI). The `ChicaneSafetyConfig.resolve(driver, cli_*)` call adds one more precedence chain (CLI > driver `control_params.chicane` > defaults), consistent with the rest of the pipeline.
- **Slip-model phases (`docs/architecture-slip-model-phase*.md`):** all kept in place as decision history. The reader's entry point is now this doc; phase docs are reference material for resurrecting the MPC work later.
