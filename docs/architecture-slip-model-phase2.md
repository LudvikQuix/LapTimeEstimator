# Architecture — v3 slip-based dynamics model, Phase 2 (Pacejka fit)

Spec source: `dev-planning/lap-simulation-csv-driver/spec-section-23-slip-model.md`,
Phase 2 (§23.9, acceptance §11.52–§11.53).

## What this phase builds

The **Magic Formula tyre force calculator** and a **five-stage telemetry
fitter** that derives per-axle Pacejka coefficients (B, C, D, E) — lateral
and longitudinal — from real lake telemetry. The output lands as a
`pacejka_calibration` block inside the existing driver JSON, sitting next
to the v2 `tyre_calibration` block produced by `fit_driver.py`.

Phase 2 is **purely additive**: nothing in the v2 simulator, v2 fitter,
v2 tyre-state, or web UI changes. The new code lives under
`src/lap_estimator/dynamics/` (the package created by Phase 1) and a new
top-level CLI (`fit_slip.py`) at the repo root.

## Why this architecture

Three forcing functions from spec §23.7 + the Phase 2 brief:

1. **The Magic Formula must be vectorisable.** Both the fitter (large
   training clouds) and the future Phase 3 ODE solver (one call per
   wheel per RK4 stage) need to call `pacejka_fy/fx` on numpy arrays
   without per-sample Python overhead. The solution is pure-numpy math
   everywhere — no Python loops in the hot path.

2. **The fit is under-determined per-row.** AC telemetry exposes chassis
   acceleration and per-wheel Fz, but not per-wheel Fx / Fy directly.
   The five-stage algorithm in §23.7.2 closes the system by solving
   **axle-level** Fy from `F_y_chassis` and `M_z` (a 2x2 determined
   system) before splitting within axle by `Fz` fraction. This is the
   single most important architectural choice in Phase 2 — see Stage B
   below.

3. **No new third-party packages.** The spec earmarks
   `scipy.optimize.least_squares`, but scipy isn't installed in the
   project environment yet. We hand-roll a bounded Levenberg-Marquardt
   in `_fit_helpers.py` — ~80 lines of numpy. The bounds are enforced
   via a logistic transform so the LM's finite-difference Jacobian
   never sees the hard limits. This keeps the dep surface at "numpy
   only" for the entire v3 fitter.

## Module surface

```
src/lap_estimator/dynamics/
  pacejka.py            # Magic Formula + friction-ellipse + scalar entry points
  pacejka_fit.py        # 5-stage fit orchestrator (Stages A-E)
  _fit_helpers.py       # LM least-squares + IIR + central-diff helpers
  ...                   # (Phase 1 skeleton files unchanged)
src/lap_estimator/
  lake_loader.py        # (additively) extended column list for v3 channels
fit_slip.py             # NEW — repo-root CLI for the v3 fit workflow
```

### `pacejka.py`

Five public functions, all numpy-vectorised:

| Function | Purpose |
|---|---|
| `pacejka_fy(alpha, Fz, coeffs, mu_scale=1.0)` | Lateral force, dataclass-coeffs entry. |
| `pacejka_fx(kappa, Fz, coeffs, mu_scale=1.0)` | Longitudinal, dataclass-coeffs entry. |
| `pacejka_lateral(alpha, Fz, B, C, D, E)` | Lateral, raw-scalar entry (curve-fit objective). |
| `pacejka_longitudinal(kappa, Fz, B, C, D, E)` | Longitudinal, raw-scalar entry. |
| `combined_friction_ellipse(Fx, Fy, Fz, D_x, D_y, ellipse_exponent=2.0)` | Friction-ellipse projection. |
| `combined_slip_force(alpha, kappa, Fz, coeffs_lat, coeffs_long, mu, ...)` | One-shot combined-slip query. |

`PacejkaCoeffs` (also exported as `LateralCoeffs` / `LongitudinalCoeffs`)
is a frozen 4-tuple dataclass — `(B, C, D, E)`. We deliberately store the
peak-grip coefficient `D` as μ (Fz-normalised). Real tyre force is `D ·
Fz` inside the formula. This decouples B/C/D/E from the operating Fz —
which is essential because the v3 ODE solver will call `pacejka_fy` at
many different Fz values per integration step (load transfer changes Fz
moment-by-moment).

### `pacejka_fit.py`

The 5-stage algorithm runs sequentially per lap-batch. Each stage feeds
the next:

```
laps (list of merged-with-track dicts)
    │
    ▼
Stage A — slip-angle / slip-ratio inversion
    │   geometric: per-wheel α = atan2(-v_lat, |v_long|),
    │              κ = (ω·R - v_long) / max(|v_long|, 1)
    ▼
Stage B — per-wheel force decomposition
    │   axle-level Fy from sum(Fy) = m·a_y  and  Fy_f·a - Fy_r·b = M_z
    │   within-axle by Fz fraction; Fx by drivetrain + brake share
    ▼
Stage C — Magic Formula coefficient fit (LM, per axle, per direction)
    │   minimise sum_t ((Fy_pred - Fy_obs)/(1.5·Fz_safe))^2
    │   bounds: B ∈ [3, 20], C ∈ [1.0, 2.5], D ∈ [0.5, 2.5], E ∈ [-2.0, 1.0]
    ▼
Stage D — friction-ellipse exponent (Phase 2: hand-default n = 2.0)
    │
    ▼
Stage E — hold-out cross-validation on the newest lap
            RMSE_lat / mean(|Fy|),  RMSE_long / mean(|Fx|)
```

#### Stage A — slip-angle / slip-ratio inversion (geometric)

For each row, for each wheel:

- Wheel offsets `(x_w, y_w)` come from `suspensions.ini` (WHEELBASE +
  TRACK_FRONT/REAR) — we re-parse the ini directly because the existing
  `Car` wrapper conflates the two TRACK values into one.
- Body-frame wheel velocity: `v_w_x = v_x - ω·y_w`, `v_w_y = v_y +
  ω·x_w` where ω is the yaw rate (`localAngularVel_y`).
- Steering: prefer `tyreContactHeading{w}_x/y` when available (rotate
  into body frame), fall back to `steerAngle / 13` for front wheels
  (steering ratio default 13:1 — note: a v3.1 backlog item is reading
  the actual ratio from `car.ini` STEER_LOCK + `suspensions.ini` linkage
  geometry).
- α and κ clipped to physical ranges (±15°, ±0.3) before saving.

When the lake telemetry lacks any required v3 channel (e.g. for old
sessions), `_has_v3_channels(lap)` skips the lap with a stderr warning.
A fit needs ≥ 2 v3-complete laps (1 train + 1 holdout) or it raises.

#### Stage B — per-wheel force decomposition (the trickiest math)

The chassis-frame equations are exactly 3:
- `sum_w Fx_w = m · a_x`
- `sum_w Fy_w = m · a_y`
- `sum_w (Fy_w · x_w + Fx_w · y_w) = I_zz · dω/dt`

with 8 per-wheel unknowns. The spec acknowledges this is under-determined.
Our resolution:

1. **Axle-level Fy decoupling.** Cross-coupling `Fx·y_w` is small (the
   `y_w` axes are short — half-track ≈ 0.77 m vs `x_w` ≈ 1.3 m), so
   we treat the yaw moment as dominated by `Fy_f·a + Fy_r·(-b)`. That
   makes the (Fy_front_total, Fy_rear_total) pair a fully determined
   2x2 system per row:
   ```
   Fy_front_total = (M_z + b · F_y_chassis) / (a + b)
   Fy_rear_total  = (a · F_y_chassis - M_z) / (a + b)
   ```
   No bootstrap, no iteration — closed-form per timestep.

2. **Within-axle split by Fz fraction.** The outer wheel of a corner
   carries more vertical load, and therefore more lateral force —
   `Fy_FL = Fy_front_total · Fz_FL / (Fz_FL + Fz_FR)`. This prior is
   what the spec calls "the linear regime Fy ∝ α·Fz" extended to
   "Fy ∝ Fz" within an axle (α is shared per-axle in a non-Ackermann
   model).

3. **Per-wheel Fx by drivetrain + brake share.** When braking
   (`brake > 0.05`), Fx splits by axle brake share (`FRONT_SHARE` from
   `brakes.ini`); when accelerating, by drivetrain (RWD/FWD/AWD from
   `drivetrain.ini`). Within axle: by Fz fraction (load-proportional
   traction). The off-axis non-driven wheels in the spec
   (`Fx = -F_rolling`) are approximated as 0 — rolling resistance at
   typical road-car speeds is ~50 N per wheel, well below the noise
   floor of the lateral analysis.

`M_z` itself is `I_zz · d(ω)/dt`, where ω is low-pass-filtered at 10 Hz
(one-pole IIR in `_fit_helpers.low_pass_iir`) before central-differencing
to suppress numerical-derivative noise amplification. `I_zz` defaults to
`m · (wheelbase/2)² · 1.2` (typical sedan ratio per the Phase 2 brief)
when not directly available from `car.ini`.

This whole decomposition is **exact when the input chassis quantities are
self-consistent** (Newton's laws). Real lake telemetry from a working
simulation is self-consistent by construction. Synthetic test data must
also be generated by integrating the force-driven dynamics — synthesising
α, F_chassis, and ω independently will break Stage B's recovery.

#### Stage C — Magic Formula least-squares fit

For each axle × direction, the residual is:

```
r_i = (Fy_pred(α_i, Fz_i, B, C, D, E) - Fy_obs_i) / (1.5 · Fz_safe_i)
```

The `1.5 · Fz` normaliser ensures all samples contribute on the
fractional-of-peak-grip scale (heavy-load samples don't dominate light-
load samples by sheer N). The LM stops on relative cost change `< 1e-8`
or 200 iters. Bounds are enforced via a logistic transform applied to
the LM's "free space" parameters — the Jacobian is finite-differenced
in free space, so bound saturation never destabilises the gradient.

Initial guesses (`B=10, C=1.30, D=1.50, E=-0.20` lat;
`B=10, C=1.65, D=1.40, E=+0.30` long) are the spec's hand defaults —
the same numbers `pacejka_calibration.measured: false` defaults to.

#### Stage D — combined-slip ellipse exponent

Phase 2 hard-codes `friction_ellipse_exponent = 2.0` per the brief.
A future Phase 2.1 will scan exponent ∈ [1.5, 4.0] against samples
where `|α| > 1°` AND `|κ| > 0.02` (both slip channels loaded), seeking
the value that minimises `|((Fx/(D_x·Fz))^n + (Fy/(D_y·Fz))^n) - 1|²`
on the combined-slip cloud.

#### Stage E — hold-out cross-validation

The newest lap (last in the input list) is held out. Stage A + B run on
the holdout to derive observed `(α, κ, Fz, Fy_obs, Fx_obs)`; the fitted
coefficients then predict `Fy_pred / Fx_pred` on the same `(α, κ, Fz)`.
RMSE is reported both pooled (front+rear) and per-axle.

Phase 2 acceptance thresholds (§23.9):
- `rmse_lat_pct ≤ 15%`
- `rmse_long_pct ≤ 20%`

When the threshold fails, the block is still written but
`source.cv_passed = false` — letting the user (or future UI banner)
decide whether to use the calibration.

### `fit_slip.py` (CLI)

Mirrors `fit_driver.py`'s shape but is intentionally a separate file:

```
fit_slip.py <car_dir> <track_csv> <driver_json> \
    [--from-lake driver=...,car=...,track=...,n=N] \
    [--from-csvs <path>...] \
    [--lake-url ...] [--lake-token ...] \
    [--compound <name>] \
    [--require-cv-pass]
```

Either `--from-lake` or `--from-csvs` is required (mutually exclusive
group). The CLI prints the fitted B/C/D/E per axle and the cross-val
RMSEs, then merges the `pacejka_calibration` block into the supplied
driver JSON — preserving every other top-level key.

### Driver JSON additions

The new block sits at the top level of `drivers/<name>.json`:

```jsonc
{
  ...existing v2 fields preserved verbatim...,
  "pacejka_calibration": {
    "front": {
      "lateral":      {"B": ..., "C": ..., "D_per_Fz": ..., "E": ...},
      "longitudinal": {"B": ..., "C": ..., "D_per_Fz": ..., "E": ...}
    },
    "rear":  {"lateral": {...}, "longitudinal": {...}},
    "friction_ellipse_exponent": 2.0,
    "measured": true,
    "source": {
      "compound": "Semislicks (SM)",
      "fit_version": "v3.0",
      "n_laps_train": 4, "n_laps_holdout": 1, "n_samples": ...,
      "rmse_lat_pct": ..., "rmse_long_pct": ...,
      "rmse_lat_front_pct": ..., "rmse_lat_rear_pct": ...,
      "rmse_long_front_pct": ..., "rmse_long_rear_pct": ...,
      "cv_passed": true,
      "fitted_at": "2026-..."
    }
  }
}
```

The `D_per_Fz` naming makes the Fz-normalised semantics explicit — this
is the peak-grip coefficient μ, not a raw force.

### `lake_loader.py` extension (additive)

The v3 channels are appended to `_TELEM_COLUMNS`. Existing v2 consumers
(`fit_driver.py`, web service) are unaffected — they only read the
columns they already used. The new v3 columns are:

- `wheelLoadFL/FR/RL/RR` (per-wheel Fz, N)
- `wheelAngularSpeedFL/FR/RL/RR` (rad/s)
- `localVelocity_x/y/z` (body frame, m/s)
- `localAngularVel_x/y/z` (body frame, rad/s)
- `accG_x/y/z` (chassis-frame acceleration, g-units)
- `tyreContactHeading{FL,FR,RL,RR}_x/y/z` (per-wheel steer vector)
- `wheelSlipFL/FR/RL/RR` (AC's pre-computed slip — used for Stage A
  cross-check, v3.1 backlog).

If the lake schema doesn't yet expose any of these (older sessions or
in-progress backfill), `_has_v3_channels(lap)` skips the lap with a
stderr warning. A v3 fit needs ≥ 2 v3-complete laps to proceed.

## Data flow (single fit run)

```
fit_slip.py CLI
    │
    ▼
lake_loader.load_laps_from_lake(driver, car, track, n=5, track_obj)
    │   SELECT … FROM ac_telemetry WHERE driver=… AND carModel=… AND track=…
    │       ORDER BY session_id DESC, lap DESC
    │   _bucket_into_laps → newest-N per-lap frames
    │
    ▼
pacejka_fit.fit_pacejka_from_laps(laps, car_data_dir, compound_name)
    │
    ├── _build_car_geom → CarGeom (mass, wb, track_f/r, cg, drive, brake share, I_zz)
    ├── for each train lap: _stage_a(lap, geom) → α/κ/v_long per wheel
    ├── for each train lap: _stage_b(lap, geom, sa) → per-wheel Fx/Fy/Fz
    ├── pool per-axle (front: FL+FR; rear: RL+RR) → 4 clouds
    ├── _fit_lateral_axle(α, Fz, Fy)  ×2 (front, rear)
    ├── _fit_longitudinal_axle(κ, Fz, Fx) ×2
    ├── friction_ellipse_exponent = 2.0   (Phase 2 fixed)
    └── _cross_validate(holdout) → RMSE percentages
    │
    ▼
{"front": {...}, "rear": {...}, "source": {...}, ...} (dict)
    │
    ▼
merge_into_driver_json(driver_json_path, block)
    │   read existing JSON, set ["pacejka_calibration"], write
    │
    ▼
Wrote: drivers/<name>.json (v2 fields preserved verbatim)
```

## Integration points

- **`fit_driver.py` is untouched.** A future small change (§23.9 Phase 2
  brief originally suggested a `--fit-pacejka` flag) is *not* added in
  this phase; the v3 fit is a separate workflow via `fit_slip.py`. The
  brief explicitly preferred this separation: "since v3 fit is a
  distinct workflow".

- **`lap.py` is untouched.** The `--model {point-mass, slip}` dispatcher
  lands in Phase 1; Phase 2 doesn't read or run the slip simulator, only
  produces the calibration block that Phase 3+ will consume.

- **`tyre_state.py` is untouched.** Phase 2 produces Pacejka coefficients
  that Phase 3's `slip_simulator.py` will plug into the per-wheel slip-
  energy feed (`dE[w] += |Fx · κ · v_w_x| · dt + ...`). That wiring is
  the Phase 3 deliverable.

- **Web UI is untouched.** The "Slip model parameters" panel and the
  "Also fit Pacejka coefficients" checkbox are Phase 5 deliverables.

## Phase 2 verification

Three layers of test (run with `python .tmp/...`):

1. **`test_11_53_roundtrip.py`** — Stage C in isolation. Synthesise a
   clean (α, Fz, Fy) cloud from known `(B, C, D, E)`, fit it. Recovers
   B/C/D to <5%, D to <0.1%. E recovers to absolute-value-similar
   numbers (the Pacejka E parameter is intrinsically the noisiest under
   limited slip excitation; this matches published literature).

2. **`run_synth_full.py`** — full pipeline on physically-self-consistent
   synth laps. Demonstrates the JSON merge preserves v2 fields and the
   acceptance-gate machinery reports correctly. Synthetic data quality
   limits the recovered coefficient accuracy (Stage B's chassis-dynamics
   inversion requires Newton-consistent inputs that pure synthesis
   doesn't trivially produce; real telemetry is consistent by
   construction).

3. **End-to-end with `fit_slip.py`** — pending lake credentials. The
   CLI accepts `--from-lake "driver=tomas,car=bmw_1m,track=ks_nurburgring,n=5"`
   and would run the full pipeline against Tomas's 5 newest lake laps;
   without credentials configured locally, this surfaces the existing
   `RuntimeError: lake_loader: missing QUIXLAKE_URL or QUIX_LAKE_TOKEN`
   cleanly. The local sample CSVs lack the v3 channels (only 24
   columns), so `--from-csvs` against them surfaces the "no laps with
   v3 channels" guard.

## Known limitations and v3.1 follow-ups

- **Stage B's M_z is sensitive to omega numerical derivative noise.**
  The 10 Hz IIR filter helps but doesn't fully suppress it. v3.1 will
  use a Kalman smoother on (v_x, v_y, ω) jointly, or compute M_z from
  the integrated yaw-rate signal directly without differentiation.

- **Steering ratio is hard-coded to 13:1** when `tyreContactHeading*`
  channels aren't logged. v3.1 will derive from `car.ini` STEER_LOCK +
  the chassis Ackermann geometry.

- **Friction-ellipse exponent is fixed at 2.0.** Real tyres often
  fit 2.2-2.5 (blunter); v3.1 fits it from combined-slip samples.

- **Single compound per fit.** v3.0 records the compound the fit was
  run against; a future v3.1 stores `compound → coefficients` so
  `lap.py --model slip --compound <other>` works without a refit.

- **No SciPy dependency added.** A hand-rolled Levenberg-Marquardt
  with logistic bound transform sits in `_fit_helpers.py`. Switching
  to `scipy.optimize.least_squares(method="trf")` is a one-line
  change should we hit a fit-quality regression on real lake data.
