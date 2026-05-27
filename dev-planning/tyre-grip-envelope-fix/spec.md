# Tyre Grip-Envelope Fix (v3 plant raised to Tomas's measured ~1.5 g friction circle)

**Status:** Draft
**Project:** LapTimeEstimator
**Branch:** feature/sc-71955/lap-simulation
**Created:** 2026-05-27
**Planned with:** Buddy
**Owner (build):** ArchDev

---

## 1. Motivation

The just-finished ideal-CSV bypass campaign (`docs/architecture-v3-hmpc-ideal-line-bypass.md`)
proved that the **binding constraint** on closing the lap-time gap to Tomas is the v3
tyre model's grip envelope — not the controller, not the reference line, not the outer
horizon. Both plant arms (DP-along-ref and ideal-line κ) abort off-track at the Sprint A
chicane (s ≈ 620–640 m) because the fitted Pacejka cannot produce the deceleration the
(realistic) speed profile demands. *"No config in either arm finishes a lap."*

We then measured Tomas's real tyre forces from his Lap 5 telemetry (`tomas_lap5_rich.csv`,
107.56 s) two independent ways — F = m·a from logged g-force channels, and kinematics
(F_lat = m·v²·κ, F_long = m·dv/dt). The two agree strongly (a_long corr 0.953, a_lat corr
0.944). The g-force channel is ground truth. Findings (`.tmp/tomas_force_vectors.md`):

| Quantity | Measured (ground truth) |
|---|---|
| Mass | **1570 kg** (`car.ini` `[BASIC].TOTALMASS`, incl. driver) |
| Axes | accG_z = longitudinal, accG_x = lateral (verified) |
| Peak lateral | **1.50 g** (s≈3015 m, ~71 km/h) |
| Peak longitudinal decel | **−1.54 g = −15.1 m/s²** (s≈2869 m, ~179 km/h) |
| Peak combined \|a\| | **1.54 g** |
| Chicane decel (s≈580 m) | **−1.34 g = −13.2 m/s²** |
| Chicane lateral | **1.25 g** |
| Chicane combined | **1.34 g** |

The v3 fitted Pacejka delivers only **~7 m/s² longitudinal at the chicane operating
point** — roughly half of Tomas's measured ~13 m/s². The geometric-CSV speed profile's
~15 m/s² demand is *realistic* (Tomas pulls that magnitude elsewhere on the lap). The
7 m/s² is the artifact.

**Root cause (quantified, `.tmp/ac_vs_v3_tyre_model_diff.md`):**
- Fitted lateral `D_per_Fz ≈ 1.032` sits **~16–20 % below** AC's reference peak
  `DY_REF = 1.28` (production semislick `[FRONT_1]`, `cars_csv/bmw_1m/tyres.ini`) and
  AC's raw `DY0/DX0 ≈ 1.31`. After combined-slip + load-sensitivity the effective grip
  drops further. The model under-delivers grip the car demonstrably has.
- The AC tyre holds a **`FALLOFF_LEVEL` post-peak floor (0.86 on the semislick)** — grip
  does not collapse past peak slip. This is what lets Tomas ride ~1.06×v_crit (past peak
  slip) without spinning.

### Current implementation state (verified this session — important for ArchDev)

The non-linear-load and falloff machinery **already exists and is wired end-to-end**.
Do not rebuild it; this fix tunes/strengthens it.

- `pacejka.py` — `_magic_formula` already implements `LS_EXPY/LS_EXPX` via
  `_load_sens_factor` and a `falloff_level` gate. `combined_friction_ellipse` and
  `combined_slip_force` apply the load-sensitivity per direction.
- `_slip_result.py:238-304` — the JSON→`PacejkaCalibration` builder. It **does** read
  `FZ0`, `LS_EXPY`, `LS_EXPX`, and `falloff_level` from `pacejka_calibration` and pass
  them into `AxleCoeffs` / `PacejkaCalibration`. **So `falloff_level=0.86` in
  `drivers/tomas.json` IS consumed** — it is not dead config.
- `vehicle.py:457-484` — the plant applies `falloff` + per-axle `Fz0`/`ls_exp_*` on every
  per-wheel Pacejka + ellipse call.
- `longitudinal_planner.py:163-535` — the DP planner consumes the same knobs
  (`_ls_factor`, per-axle FZ0).

So the levers below are **not "implement the feature"** — they are **(1) raise base D to
the measured envelope, and (2) verify/strengthen the falloff gate**, which is presently
a soft `tanh`-gated clip that may not hold the AC 0.86 floor as intended.

---

## 2. The hard constraint ArchDev must NOT relitigate

> **Load-sensitivity *shape* (LS_EXPY) is NOT the grip lever.** A prior "non-linear D(Fz)
> via LS_EXPY" change had **null lap-time effect** because it is concave: the inside
> (unloaded) wheel gains a little, the outside (loaded) wheel loses a little, and the
> axle Fy net dropped ~1.9 % under load transfer. The chicane gap is **+10.7 % axle grip**;
> LS_EXPY closes essentially none of it.

The binding levers are **BASE D magnitude** and the **FALLOFF floor**. Keep LS_EXPY/FZ0
as-is (they are already wired and harmless at static load); do not expect grip from
re-shaping them.

---

## 3. Levers (ordered by ROI; each with files + validation)

### Lever 1 — Raise base D to the MEASURED 1.5 g envelope (primary, ~80 % of gap)

**What:** The fitted `D_per_Fz ≈ 1.032` is too low. The measurement is ground truth:
Tomas uses ~1.5 g lateral, ~1.5 g longitudinal, ~1.54 g combined. Reconcile the three
numbers:
- Fitted D = 1.032 (steady-state pooled Pacejka fit, biased low — see §3.1).
- AC raw `DY0/DX0 ≈ 1.31`, `DY_REF/DX_REF = 1.28/1.30` (semislick `[FRONT_1]`).
- Measured peak ≈ 1.5–1.54 g.

The fit is the outlier. **AC's effective grip is genuinely higher than any single
steady-state Pacejka D** because of thermal optimum, pressure-at-ideal, brush load
curve, and the falloff floor — none of which a flat pooled fit captures, and the fit was
additionally biased low by high-Fz samples dominating by mass (`ac_vs_v3` §1, §5.1).

**Pragmatic fix (recommended):** re-fit / re-scale the lateral and longitudinal `D_per_Fz`
so the *augmented plant* reproduces the **measured** envelope — i.e. peak lateral ≈
1.5 g and peak longitudinal ≈ 1.5 g (−15 m/s²) at the operating points where Tomas hits
them, and ≈13 m/s² at the chicane. Target `D ≈ 1.25–1.30` (in line with AC `DY_REF`/`DX_REF`),
tuned so the closed-loop plant — *after* combined-slip ellipse + falloff + the production
grip multiplier — lands on 1.5 g, not so the raw `D·Fz` does. Ground truth is the measured
envelope, not the steady fit.

**Decision required (open question Q1):** how to express the higher D so existing recorded
laps don't break — see §5 Regression. Recommended: write the new D values into a **new
driver JSON** (`drivers/tomas_highgrip.json` or a `pacejka_calibration_v2` block selected
by flag) rather than overwriting `drivers/tomas.json`, AND/OR gate via a CLI flag that the
ideal-CSV consumer passes. Do **not** bake an override into the ini loader (project rule).

**Files:**
- `drivers/tomas.json` → `pacejka_calibration.front/rear.lateral.D_per_Fz` and
  `.longitudinal.D_per_Fz` (currently 1.0323 / 1.0408). The new-value carrier (new file or
  new block) is the deliverable.
- `src/lap_estimator/dynamics/pacejka_fit.py` — if re-fitting rather than hand-scaling.
  Note line ~46: `D_per_Fz` LAT bounds were tightened to `[0.95, 1.40]`; raising D to ~1.28
  is *inside* the existing bound, so no bound change needed for a re-fit. The fit objective
  must be retargeted to the **measured force envelope** (`.tmp/tomas_force_vectors.csv`),
  not just minimum-RMSE over all samples (which is what biased it low).
- `src/lap_estimator/dynamics/_slip_result.py:238-304` — reads `D_per_Fz`; no change needed
  unless a new `pacejka_calibration_v2` block name is introduced (then add the selector).

**Validation:** run the v3 plant at the chicane operating point and full lap; assert peak
combined ≈ 1.5 g and chicane decel ≈ 13 m/s² against `.tmp/tomas_force_vectors.csv` (see §4).

### Lever 1.1 — Per-axle vs pooled D (sub-decision of Lever 1)

The v3 fit **pooled axles for lateral** (front/rear lateral coeffs are identical by
construction; `pacejka_fit.py:354-356`, `_slip_result.py:281-283`). AC coefficients are
per-axle but the front/rear `DY_REF` are equal (1.28/1.28) and `DX_REF` equal (1.30/1.30)
on the semislick — only `DY0/DX0` and `LS_EXPY/FZ0` differ marginally. **Recommendation:
keep pooled lateral D for the 1.5 g target.** Per-axle D is a <1 % refinement and the
binding axle (front, BMW 1M) is the one the planner already uses. Revisit only if Lever 1
overshoots/undershoots asymmetrically in validation.

### Lever 2 — Enforce the FALLOFF floor properly (secondary, ~1–2 % + stability)

**What:** AC holds grip at `FALLOFF_LEVEL × peak` past peak slip; the production semislick
value is **0.86–0.87** (`tyres.ini [FRONT_1]/[REAR_1]` = 0.86; legacy street = 0.87). The
v3 Pacejka E-tail (`E_lat = −0.65`) lets grip drift faster. This floor is what lets the
plant ride past peak slip (1.06×v_crit) without the lateral force collapsing — directly
relevant to surviving the chicane apex.

**State:** `falloff_level=0.86` IS in `drivers/tomas.json` and IS consumed. **BUT** the
current implementation (`pacejka.py:118-138`) is a *soft, `tanh`-gated* clip:
`gate = tanh(|inner|·0.5)`, `floor = falloff·sin(C·π/2)·gate`. This ramps the floor in
gradually and may not actually hold a hard 0.86 floor deep in the tail at the slip values
the chicane reaches. **ArchDev task:** verify the realized floor at the chicane slip
operating point equals `0.86 × peak`; if the `tanh` gate is suppressing it, tighten the
gate (steeper ramp / higher multiplier) or switch to a saturation-region mask keyed on
`|slip| > alpha_peak` so the floor is fully applied past peak. Keep the sign-preserving,
odd-in-slip property and the smooth transition through α=0 (do not lift the central
small-slip region — that would corrupt linear-range stiffness).

**Files:**
- `src/lap_estimator/dynamics/pacejka.py:118-138` (the falloff gate in `_magic_formula`).
- No JSON change (value already present); confirm `falloff_level` flows for *both* lateral
  and longitudinal (it does — passed in both `pacejka_fy`/`pacejka_fx` calls in `vehicle.py`).

**Validation:** unit-style check — evaluate `pacejka_fy(alpha, Fz)` across α from 0 to 20°
and assert the post-peak tail asymptotes to ≥ 0.86×peak (not the unconstrained E-tail
value ~0.93 quoted in `ac_vs_v3` §1). Then confirm the chicane no longer aborts (§6).

### Lever 3 — Brush model (heavyweight alternative; DEFER)

Memory notes AC uses a brush model, not Pacejka. A brush re-implementation is the
physically-correct option and would capture load sensitivity, falloff, and combined slip
natively. **Recommendation: do NOT do this first.** The pragmatic Pacejka-envelope-match
(Levers 1+2) reproduces the *measured* ground-truth envelope at far lower cost and risk,
and the measured envelope is the acceptance target regardless of model form. Note brush as
the fallback only if Levers 1+2 cannot simultaneously hit 1.5 g peak *and* a physically
plausible combined-slip shape (e.g. if matching peak forces the ellipse to misbehave in
the transition). Out of scope for this branch unless validation forces it.

---

## 4. Validation against ground truth

All re-fit/re-scale work is validated against **`.tmp/tomas_force_vectors.csv`** (per-sample
distance, v, a_long/a_lat both methods, forces, combined |F|, utilisation). The augmented
plant must reproduce:

| Gate | Target | Tolerance |
|---|---|---|
| Peak combined grip | ~1.5 g (1.54 g braking) | within ~0.1 g |
| Peak lateral | ~1.5 g | within ~0.1 g |
| Peak longitudinal decel | ~15 m/s² | within ~1 m/s² |
| **Chicane decel (s≈580 m)** | **~13 m/s²** | **within ~1–2 m/s²** (vs the ~7 it does now) |
| Chicane lateral | ~1.25 g | within ~0.15 g |

Run commands are **plain `python ...` (Bash) — PowerShell is BLOCKED**. Example shapes
(ArchDev to confirm exact entrypoints against `lap.py`):

```bash
# Re-fit / re-scale D against the measured envelope
python fit_slip.py --driver drivers/tomas.json --target-envelope .tmp/tomas_force_vectors.csv

# Validate plant envelope at chicane + full lap
python lap.py --driver drivers/tomas_highgrip.json --slip --track ks_nurburgring/layout_sprint_a \
    --emit-force-trace .tmp/v3_force_check.csv
```

(Exact flag names are ArchDev's to wire; the contract is: produce a per-sample force trace
the validation can diff against `.tmp/tomas_force_vectors.csv`.)

---

## 5. Regression safety (project rule: no overrides baked into the ini loader)

The higher-grip envelope **must not silently change** existing recorded results — the
v2 point-mass 1:48 match and any committed v3 baselines.

**Requirement:** the OLD grip (`D_per_Fz ≈ 1.03`) remains the default for existing driver
JSONs and existing recorded laps. The new envelope is **opt-in** via one of:

1. **(Recommended)** A separate calibration carrier — new file `drivers/tomas_highgrip.json`
   *or* a `pacejka_calibration_v2` block in `tomas.json` selected by an explicit flag /
   loader argument. `_slip_result.py` builder gains a selector; default path unchanged.
2. A CLI flag (e.g. `lap.py --grip-envelope measured`) that the ideal-CSV consumer passes;
   absent → legacy D.

**Forbidden:** editing the AC `cars_csv/bmw_1m/tyres.ini` loader to inject higher grip, or
overwriting `drivers/tomas.json`'s `D_per_Fz` in place with no fallback. v2 / point-mass
grip paths (`_DriverScaledCar.tyre_grip_*`, `car.tyre_dy0_f`) are **untouched** by this
change — they read the AC ini directly and are a separate codepath from the Pacejka block.

---

## 6. Downstream win condition (follow-on ArchDev build — out of scope here)

After the augmented plant lands, re-run the ideal-CSV bypass campaign on the higher-grip
plant. Expected: the chicane (s ≈ 620–640 m) that **every** prior config aborted on now
**finishes**, and the lap approaches the **110.88 s ceiling** (the ideal-line CSV's own
integrated speed profile) — i.e. within ~3 s of Tomas's 107.56 s. That campaign is the
*consumer* of this fix and is specified separately; do not modify the bypass code here.

---

## 7. File inventory (exact touchpoints)

| File | Lever | Change |
|---|---|---|
| `drivers/tomas.json` | 1 | New higher `D_per_Fz` lat/long (via new file or `_v2` block — §5). `falloff_level=0.86`, FZ0, LS_EXP already present. |
| `src/lap_estimator/dynamics/pacejka_fit.py` | 1 | Retarget fit objective to measured envelope (only if re-fitting vs hand-scaling). D bounds `[0.95,1.40]` already admit ~1.28. |
| `src/lap_estimator/dynamics/_slip_result.py` | 1, regression | Add calibration selector if a `_v2` block is used; else no change (already reads D/FZ0/LS_EXP/falloff). |
| `src/lap_estimator/dynamics/pacejka.py` | 2 | Strengthen/verify the `falloff_level` gate (lines 118-138) so the realized post-peak floor = 0.86×peak at chicane slip. |
| `src/lap_estimator/dynamics/vehicle.py` | (read-only) | Already applies falloff + FZ0 + LS_EXP per wheel (457-484). Confirm, no change expected. |
| `src/lap_estimator/dynamics/longitudinal_planner.py` | (read-only) | Already consumes D + FZ0 + LS_EXP (163-535). New D flows through automatically. Confirm Pass-1 `v_corner` rises with higher `D_lat`. |
| `src/lap_estimator/dynamics/mpc_qp_ellipse.py`, `mpc_model.py` | (verify) | Confirm they read D from the same `PacejkaCalibration`; the higher D must reach the MPC grip ellipse, not a stale constant. ArchDev to check. |

**Do NOT touch:** reactive/v2 controller logic, mpcc, HMPC controllers
(`hmpc_*.py`), and the ideal-CSV bypass code (consumer, not the fix).

---

## 8. Acceptance gates

1. **Envelope match:** augmented plant reproduces Tomas's measured envelope — peak combined
   ~1.5 g, peak lateral ~1.5 g, chicane decel ~13 m/s² — validated against
   `.tmp/tomas_force_vectors.csv` (§4 table, within tolerances).
2. **Falloff floor:** post-peak lateral/longitudinal force asymptotes to ≥ 0.86×peak
   (not the unconstrained ~0.93 E-tail) at the chicane slip operating point.
3. **Regression:** legacy `D ≈ 1.03` remains the default; new envelope opt-in only; no AC
   ini-loader override; v2/point-mass paths unchanged and bit-identical on existing laps.
4. **(Hand-off, not gated here)** ideal-CSV bypass on the new plant finishes the chicane
   and approaches 110.88 s / <3 s of Tomas.

---

## 9. Risks & open questions

- **R1 — Over-grip / unrealistic lap:** scaling D to hit 1.5 g *peak* could overshoot in
  mid-corner where Tomas is below peak, producing an unrealistically fast sim. Mitigation:
  validate the *whole* `tomas_force_vectors.csv` trace (corr/RMSE across the lap), not just
  the peak point. The fit objective should minimise envelope error across operating points,
  with the peak as a hard upper anchor.
- **R2 — Falloff gate interaction:** strengthening the falloff floor (Lever 2) while raising
  D (Lever 1) compounds grip; tune them together and re-validate the peak, or Lever 2 could
  push the realized peak past 1.54 g.
- **R3 — Ellipse exponent:** at `n=2` (true ellipse) combined grip at 45° is `peak/√2`.
  Tomas's *combined* peak (1.54 g) ≈ his per-axis peak (1.5 g), implying he rarely loads
  both axes hard simultaneously — so `n=2` is probably fine, but if validation shows the
  chicane (combined 1.34 g) still falls short after Lever 1, a blunter ellipse (`n≈2.2–2.5`,
  `friction_ellipse_exponent`) is the next knob, not more base D.
- **Q1 (decision needed before build):** carrier for the new D — separate
  `drivers/tomas_highgrip.json`, a `pacejka_calibration_v2` block + selector, or a CLI
  `--grip-envelope` flag? Recommend separate file for cleanest regression isolation. **Needs
  user/ArchDev sign-off.**
- **Q2:** re-fit (retarget `pacejka_fit.py` objective to the measured envelope) vs
  hand-scale D from 1.03→~1.28? Re-fit is more defensible; hand-scale is faster to validate
  the hypothesis. Recommend a quick hand-scale spike to confirm the chicane finishes, then a
  proper re-fit for the committed value.

---

## 10. Alternatives considered

- **LS_EXPY load-sensitivity re-shape:** rejected — proven null lap-time effect (concave,
  axle Fy drops under load transfer). See §2.
- **Controller-side fixes (weights, chicane cap, outer horizon):** rejected — the bypass
  campaign exhausted these; *both* plant arms abort regardless. The wall is the plant's
  grip, not the controller. (`architecture-v3-hmpc-ideal-line-bypass.md` §"The wall".)
- **Full brush model:** deferred — physically correct but large; the measured envelope is
  the acceptance target and Pacejka can be scaled to hit it (§3 Lever 3).

## 11. References

- `.tmp/tomas_force_vectors.md` / `.csv` — measured ground-truth envelope (acceptance target).
- `.tmp/tomas_force_v2.csv` — drivetrain-efficiency force fit.
- `.tmp/ac_physics_model_summary.md`, `.tmp/ac_vs_v3_tyre_model_diff.md` — AC-vs-v3 term diff.
- `docs/architecture-v3-hmpc-ideal-line-bypass.md` — proof the grip envelope is the binding wall.
- `docs/architecture-v3-pacejka-nonlinear-load-sensitivity.md` — prior LS_EXPY work (the null path).
- `cars_csv/bmw_1m/tyres.ini` — AC raw coeffs (`[FRONT_1]/[REAR_1]` semislick: DY_REF/DX_REF
  1.28/1.30, DY0/DX0 ~1.31, FALLOFF_LEVEL 0.86).
- `drivers/tomas.json` — current calibration (D≈1.03, falloff 0.86, FZ0/LS_EXP present).
