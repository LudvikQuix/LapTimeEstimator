# v3 Pacejka — non-linear D(Fz) load sensitivity (AC `LS_EXPY` / `LS_EXPX`)

Layered onto the v3 slip model the AC brush-model load-sensitivity term:

```
D(Fz) = D_REF · (Fz / FZ0)**(LS_EXP − 1)
F     = D(Fz) · Fz · sin(C · atan(B·x − E·(B·x − atan(B·x))))
```

with optional post-peak floor `FALLOFF_LEVEL` on `|sin(C · atan(...))|`.
At `Fz = FZ0` the load factor collapses to `1.0`, so the linear-load fit
calibrated at the static axle load is preserved exactly at the
operating point. Below `FZ0` the wheel gains grip; above `FZ0` it
loses grip — the textbook AC tyre curvature.

The patch is **production-wide**: it touches `vehicle.compute_derivatives`
(every controller's truth-model step), `longitudinal_planner` (the DP
plan all controllers track), and `mpc_qp_ellipse` (the MPC/MPCC/HMPC
ellipse cap). Defaults preserve the legacy linear-D behaviour: when a
driver JSON has no `FZ0` / `LS_EXP*` keys, the model degrades to the
pre-patch ``D · Fz`` formula identically.

---

## File inventory

| File | Change |
|---|---|
| `src/lap_estimator/dynamics/pacejka.py` | New `_load_sens_factor(Fz, Fz0, ls_exp)`; `_magic_formula` accepts `Fz0` / `ls_exp` / `falloff_level` kwargs; `pacejka_fy` / `pacejka_fx` / `combined_friction_ellipse` / `combined_slip_force` all gained matching pass-through kwargs. Defaults `None` keep the linear-D path bit-identical. |
| `src/lap_estimator/dynamics/vehicle.py` | `AxleCoeffs` gained `Fz0`, `ls_exp_lat`, `ls_exp_long`; `PacejkaCalibration` gained `falloff_level`. Per-wheel Pacejka calls + the friction-ellipse clamp now route the non-linear knobs through. |
| `src/lap_estimator/dynamics/_slip_result.py` | `_load_pacejka_calibration` reads the new fields from `pacejka_calibration.{front,rear}.{FZ0, LS_EXPY, LS_EXPX}` and `pacejka_calibration.falloff_level`. |
| `src/lap_estimator/dynamics/longitudinal_planner.py` | New scalar helper `_ls_factor`; `_available_long_accel` / `_available_long_decel` accept per-axle `fz0_*` + per-direction `ls_exp_*` and scale `D_lat` / `D_long` per axle. |
| `src/lap_estimator/dynamics/mpc_model.py` | `PlantConstants` gained six optional load-sensitivity fields; `build_plant_constants` copies them out of the `PacejkaCalibration`. |
| `src/lap_estimator/dynamics/mpc_qp_ellipse.py` | `build_ellipse_rows` multiplies each per-stage `D · Fz_k` denominator by `_ls_factor(Fz_k/2, FZ0, LS_EXP)` per axle per direction. |
| `drivers/tomas.json` | New `pacejka_calibration.{front,rear}.{FZ0, LS_EXPY, LS_EXPX}` and top-level `falloff_level`. Values from `cars_csv/bmw_1m/tyres.ini` Semislicks rows. |

Approximate line counts: pacejka.py +95 / −15 (load-sens + falloff +
docstrings), vehicle.py +28 / −5, _slip_result.py +20 / −3,
longitudinal_planner.py +60 / −8, mpc_model.py +18 / −1,
mpc_qp_ellipse.py +30 / −5, tomas.json +18.

---

## Data flow

```
driver JSON                                       runtime
─────────                                         ───────
pacejka_calibration                               _load_pacejka_calibration
  ├ front.FZ0     ──┐                              ├ AxleCoeffs(Fz0, ls_exp_lat, ls_exp_long)
  ├ front.LS_EXPY  ─┼──► AxleCoeffs.front  ───►   ├ AxleCoeffs(Fz0, ls_exp_lat, ls_exp_long)
  ├ front.LS_EXPX  ─┘                              └ PacejkaCalibration.falloff_level
  ├ rear.FZ0      ──┐
  ├ rear.LS_EXPY   ─┼──► AxleCoeffs.rear
  ├ rear.LS_EXPX   ─┘
  └ falloff_level  ─────► PacejkaCalibration.falloff_level

truth model (vehicle.compute_derivatives)
───────────────────────────────────────
for each wheel:
    Fz_w  := quasi-static weight transfer (per-wheel)
    fy    := pacejka_fy(alpha_w, Fz_w, lat_coeffs,
                        Fz0=axle.Fz0, ls_exp=axle.ls_exp_lat,
                        falloff_level=calib.falloff_level)
    fx    := pacejka_fx(kappa_w, Fz_w, lng_coeffs, ... ls_exp=ls_exp_long ...)
    (fx_c, fy_c) := combined_friction_ellipse(
                        fx, fy, Fz_w,
                        D_x, D_y,
                        Fz0_x=axle.Fz0, ls_exp_x=axle.ls_exp_long,
                        Fz0_y=axle.Fz0, ls_exp_y=axle.ls_exp_lat)

longitudinal planner (per-stage)
───────────────────────────────
D_lat_eff  := D_lat  * _ls_factor(0.5 * Fz_axle_front, FZ0_f, LS_EXPY_f)
D_long_eff := D_long * _ls_factor(0.5 * Fz_axle_drive, FZ0_*, LS_EXPX_*)
                              ▲
                              │  axle Fz halved to obtain per-wheel Fz

MPC ellipse cap (per-stage per-axle)
────────────────────────────────────
ls_lat_k := _ls_factor(0.5 * Fz_axle_k, FZ0, LS_EXPY)
ls_lng_k := _ls_factor(0.5 * Fz_axle_k, FZ0, LS_EXPX)
D_lat_Fz_k  := D_lat  * ls_lat_k * Fz_axle_k
D_long_Fz_k := D_long * ls_lng_k * Fz_axle_k
```

`FZ0` is stored **per wheel** in the driver JSON (matching AC's
`tyres.ini` convention), so the planner and the QP — which work with
axle-summed `Fz` — divide by 2 before computing the factor. The truth
model already operates per-wheel and uses `FZ0` directly.

---

## Build-time decision: per-wheel vs per-axle `FZ0`

AC's `tyres.ini` lists `FZ0` per tyre (per wheel). Two interpretations
for our model were available:

1. **Per-wheel reference** (chosen): JSON `FZ0` = per-wheel static load.
   The truth model uses it directly; planner / QP halve axle totals
   to compare against it. Matches the user's spec ("reference load =
   m·g·front_axle_share **per axle**, divided by 2 wheels") and AC's
   tyre-intrinsic convention.
2. **Per-axle reference**: JSON `FZ0` = axle-summed static load.
   Cleaner for the planner/QP but the truth model would need to multiply
   per-wheel Fz by 2, breaking the per-wheel mental model.

Option 1 keeps every Pacejka call consistent: `(Fz / FZ0)` is always the
per-wheel ratio. The two scaling points where Fz is axle-summed (planner,
ellipse-row builder) make the halving local and visible.

For the BMW 1M / Tomas at `cg_front = 0.5178` and `m = 1592.5 kg`
(`TOTALMASS=1570` + 30 L fuel × 0.75 kg/L):

- `FZ0_front` = 1592.5 · 9.81 · (1 − 0.5178) / 2 = **3766.4 N**
- `FZ0_rear` =  1592.5 · 9.81 · 0.5178 / 2       = **4044.2 N**

`tyres.ini` Semislicks listed 3337 N front and 3449 N rear; those are
**tyre-intrinsic** references, not car-specific. Using them would give
a non-unity factor at rest and silently shift the fitted `D_per_Fz`
calibration away from the static load it was fit at. The car-specific
static-load convention (above) makes the factor exactly `1.0` at rest,
which is the spec's sanity check: at `Fz = FZ0` the formula reproduces
the linear-load behaviour exactly.

---

## Values read from `cars_csv/bmw_1m/tyres.ini`

The active compound for Tomas is `Semislicks` (per
`pacejka_calibration.source.compound`), so we use the `[FRONT_1]` /
`[REAR_1]` rows — not the `[FRONT]` / `[REAR]` Street tyre rows.

| Param | Street (`FRONT` / `REAR`) | Semislicks (`FRONT_1` / `REAR_1`) — used |
|---|:---:|:---:|
| `LS_EXPY` front | 0.8352 | **0.8244** |
| `LS_EXPY` rear  | 0.8482 | **0.8374** |
| `LS_EXPX` front | 0.9002 | **0.8892** |
| `LS_EXPX` rear  | 0.9105 | **0.8995** |
| `FZ0` front (tyres.ini, unused) | 3117 N | 3337 N |
| `FZ0` rear  (tyres.ini, unused) | 3229 N | 3449 N |
| `FALLOFF_LEVEL` | 0.87 | **0.86** |

`LS_EXP` < 1 across the board (concave `D(Fz)`), and Semislicks have a
slightly stronger non-linearity than Street tyres (lower `LS_EXP*`).

---

## Validation — reactive controller on Sprint A ideal line

Command template (single-lap, 10-seed Monte Carlo via the driver's
`consistency_sigma = 1.5`):

```
python lap.py cars_csv/bmw_1m \
    tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv \
    drivers/tomas.json --model slip --controller reactive --single-lap \
    --no-plot --inertia-zz 2400 --chicane-safety-mult <MULT>
```

| `chicane_mult` | Pre-fix stable | Pre-fix lap | Post-fix stable | Post-fix lap |
|:---:|:---:|---:|:---:|---:|
| 0.80 | 10/10 | 2:09.22 | 10/10 | **2:09.20** |
| 0.85 | 10/10 | 2:06.06 | 10/10 | **2:06.04** |
| 0.90 |  7/10 | 2:03.32 |  7/10 | **2:03.32** |
| 0.95 |  0/10 | abort s≈675 m | 0/10 | abort s≈675 m |
| 1.00 |  0/10 | abort s≈670 m | 0/10 | abort s≈670 m |

**The fix is null on the headline path.** Lap times move by ≤ 20 ms;
stability counts are unchanged; aborts hit the same chicane apex at the
same arclength.

### Why the win didn't materialise

The user expected ~10 % extra grip at high-load-transfer corners
(chicane apex) from the inner wheel gaining grip. The inner wheel does
gain — but the outer wheel loses more under `LS_EXPY < 1` because
`D(Fz)` is **concave**: `f(0.6·FZ0) + f(1.4·FZ0)` is **less** than
`2 · f(FZ0)`. Quantitatively, at α = 7° (front peak slip) and 50 %
lateral transfer the axle-summed `Fy` *drops* 1.9 % vs the linear-D
baseline:

| Symmetric (no transfer) | 40 % transfer | 50 % transfer |
|---:|---:|---:|
| Linear-D axle Fy: 7774 N | 7774 N | 7774 N |
| Non-linear axle Fy: **7774 N** | **7680 N** (−1.2 %) | **7625 N** (−1.9 %) |

This is textbook tyre load-sensitivity: lateral weight transfer reduces
axle grip, the well-known "stiffer roll bar => less rear grip" lever.
The non-linear D **correctly captures this physics**; the user's
inner-wheel intuition was right about per-wheel forces but wrong about
the axle sum.

So why didn't the planner correctly slow down to compensate? Because at
static Fz (which the planner uses for the corner-cap pass), the factor
is exactly 1.0 — the chicane apex's lateral transfer happens *inside*
the truth model, not in the planner's static lookup. The reactive
controller, in turn, tracks slip-angle targets directly; it doesn't
read axle peak grip and adjust. Both layers are oblivious to the −1.9 %
axle effect.

### What this told us

1. The patch is **correct and active** — `pacejka_fy` produces measurably
   different forces at non-`FZ0` loads (smoke test verified: at
   `Fz = 0.6·FZ0` we see `+9.4 %` Fy, at `1.4·FZ0` we see `−5.7 %` Fy).
2. **The chicane abort is not a peak-grip problem.** Mid-corner peak Fy
   barely changes (and trends slightly worse); the abort mechanism must
   live elsewhere — controller saturation, yaw-inertia / chassis
   transient response, or the steering-rate / pedal-rate slew that
   keeps the controller from rotating the car through the chicane
   right→left transition fast enough.
3. **The MPC / MPCC paths inherit the patch automatically.** Their
   ellipse caps now use the same `D(Fz_k) · Fz_k` per stage; the QP
   linearisation point shifts a little under longitudinal weight
   transfer (front loaded under braking ⇒ factor < 1 ⇒ slightly
   tighter cap on the front axle), which is the right direction.

---

## Backward compatibility

- Driver JSONs without the new fields keep loading. `_load_pacejka_calibration`
  reads `FZ0` / `LS_EXPY` / `LS_EXPX` / `falloff_level` with `.get(...)`
  and `None` defaults; the Pacejka helpers fall back to the linear-load
  branch when any knob is `None`.
- The Pacejka fitter (`pacejka_fit.py`) still calls the legacy
  `pacejka_lateral` / `pacejka_longitudinal` raw-scalar wrappers, which
  don't accept the new kwargs and stay on the linear-D path. Fitting
  remains unchanged — the non-linearity is a runtime *layer*, not a
  refit. (A future refit could carry `LS_EXPY` as an additional free
  parameter, but doing so would inflate the parameter space against the
  noisy front-axle calibration we already struggle with.)
- The MPC `PlantConstants` defaults to `Fz0_* = None` so legacy
  controllers (without the load-sens fields in `calib`) keep their
  Phase 5.0.4 linear behaviour exactly.

---

## Open follow-ups

- **Outer-wheel loss is what the planner should price.** A planner-side
  fix would replace the per-axle `D_lat` with an "effective axle D under
  lateral transfer", `D_lat_eff(a_y) ≈ D_lat · (1 − k · (a_y · h / (t · g))²)`
  with `k` derived from the `LS_EXPY` concavity. That would honestly
  shave ~2 % off the corner-speed cap and the controller would have a
  matching reference. Not done here — out of scope for this targeted
  fix.
- **Refit `D_per_Fz` against AC telemetry with the non-linear D in
  place** would unbias the fit. Current `D_per_Fz` was calibrated
  against telemetry that already lives in the non-linear regime; under
  the new model `D_per_Fz` would absorb a small (≤ 2 %) constant
  offset. Marginal effect, plus the front-axle fit RMSE is already
  20 % so the bias is well below fit noise.
- **The chicane abort failure mode needs a separate investigation.** The
  diagnosis from this session: it is *not* a peak-grip problem, so the
  remaining levers are yaw-inertia (already at 2400 vs box-formula 3051),
  steering-rate slew (Tomas at `20 deg/s` is already aggressive), or the
  controller's preview window. The MPCC and HMPC paths are the
  spec'd answer for the "controller can't rotate the car through the
  right→left" failure mode.

---

## Cross-references

- Session index: `docs/architecture-v3-session-2026-05-24.md`
- Phase 5.0.4 dynamic-Fz spec discussion: `docs/architecture-slip-model-phase4.md`
- v3 shipping state: `docs/architecture-v3-shipping-state.md`
- AC tyres.ini reference: `cars_csv/bmw_1m/tyres.ini` `[FRONT_1]` /
  `[REAR_1]` rows for Tomas's Semislicks compound.
