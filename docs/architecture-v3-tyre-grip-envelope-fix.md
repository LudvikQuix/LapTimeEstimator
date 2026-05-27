# Architecture — v3 Tyre Grip-Envelope Fix (Lever 1 base-D scale)

**Branch:** feature/sc-71955/lap-simulation
**Spec:** `dev-planning/tyre-grip-envelope-fix/spec.md`
**Status:** Implemented (spike). Envelope + falloff gates pass; bypass moment-of-truth
documented honestly below (chicane still aborts — binding constraint shifted to the
inner-MPC tracking, not the plant grip).

## What this does

Adds an opt-in scale on the v3 plant's fitted Pacejka peak grip `D`, raising the
lateral/longitudinal `D_per_Fz` from the fitted `≈1.03` (which sits ~16–20 % below AC's
semislick reference) toward AC's raw `DY0/DX0 ≈ 1.31` to reproduce Tomas's *measured*
~1.5 g friction circle. The legacy `D≈1.03` envelope remains the default — the higher grip
is reached either by a CLI flag (`lap.py --grip-d-scale`) for the spike, or by a durable
opt-in driver JSON (`drivers/tomas_highgrip.json`) with the high `D` values baked in.

Because every grip consumer reads `D` from the single `PacejkaCalibration` object, scaling
it once at build time reaches the ODE plant, the DP speed planner, and the MPC friction
ellipse with no per-consumer edit.

## Why this architecture

- **Single injection point.** The fix scales `D` inside `_load_pacejka_calibration`
  (`_slip_result.py`), the one builder that produces the `PacejkaCalibration`. The
  `Car`-line `grip_y` / point-mass path reads the AC ini directly and is a *separate*
  codepath — it is untouched, so v2 results are bit-identical (verified: the `Car(...)`
  banner still prints `grip_y=1.280/1.284`, sourced from AC `DY_REF`).
- **Scale, not re-fit (Q2 RESOLVED — spike first).** Per the user decision we hand-scale
  the base `D` and sweep it rather than retargeting `pacejka_fit.py`. A flat scale on `D`
  is the cheapest faithful lever: combined-slip ellipse, the `FALLOFF` floor and the
  grip multiplier all apply *downstream*, so the realised envelope is tuned against the
  measured ground truth, not the raw `D·Fz`. A rigorous Pacejka re-fit is deferred to a
  follow-up only if the spike validates.
- **Two carriers (Q1 RESOLVED).** The spike uses a CLI override (`--grip-d-scale`, matching
  the `--cd-override`/`--boost-steady` per-driver override convention); the winner is
  persisted into a new `drivers/tomas_highgrip.json` (clone of `tomas.json` with the
  high-grip `D` baked into `pacejka_calibration`). Legacy `D≈1.03` stays the default in
  `tomas.json` with no ini-loader change — regression-safe per spec §5.
- **LS_EXPY left alone.** Spec §2: load-sensitivity *shape* is null for lap time (concave;
  axle Fy net drops under load transfer). We scale base `D` magnitude only; FZ0/LS_EXP/
  falloff are already wired and harmless at static load.

## Data flow

```
drivers/<driver>.json
  pacejka_calibration.{front,rear}.{lateral,longitudinal}.D_per_Fz
        │
        ▼
_load_pacejka_calibration(driver, grip_d_scale)        ← _slip_result.py
        │   D_lat *= d_scale ;  D_lng *= d_scale  (both axles)
        │   d_scale = 1.0 when grip_d_scale is None  → legacy envelope
        ▼
   PacejkaCalibration (front/rear AxleCoeffs, ellipse_exponent, falloff_level)
        │
        ├──────────────► vehicle.compute_derivatives        (ODE plant; pacejka_fy/fx
        │                                                     + combined ellipse + falloff)
        ├──────────────► longitudinal_planner (DP)           (v_corner rises with D_lat)
        └──────────────► mpc_model.plant_constants_from_calibration
                              → PlantConstants.{D_lat_front,D_lat_rear,D_long}
                              → mpc_qp_ellipse (MPC grip-ellipse constraint)
```

`grip_d_scale` is threaded `lap.py(--grip-d-scale) → simulate_slip(grip_d_scale=) →
_load_pacejka_calibration(grip_d_scale=)`. The `.tmp` bypass harness passes it directly to
`simulate_slip`.

## File inventory

| File | Change |
|---|---|
| `src/lap_estimator/dynamics/_slip_result.py` | `_load_pacejka_calibration` gains keyword-only `grip_d_scale`; applies `d_scale` to lateral+longitudinal `D` of both axles in the `axle()` builder. Default `None` → factor 1.0 (legacy). |
| `src/lap_estimator/dynamics/slip_simulator.py` | `simulate_slip` gains keyword-only `grip_d_scale`; passes it to `_load_pacejka_calibration`. |
| `lap.py` | New `--grip-d-scale` CLI flag (slip path); wired into the `simulate_slip` call. |
| `drivers/tomas_highgrip.json` | **New.** Clone of `tomas.json` with `D_per_Fz` baked at ×1.28 (lat 1.3214 / long 1.3322) and a `grip_envelope_fix` provenance block. Durable opt-in carrier (use with default scale). |
| `pacejka.py`, `vehicle.py`, `mpc_qp_ellipse.py`, `mpc_model.py`, `longitudinal_planner.py` | **No code change** — confirmed they read `D` from the single `PacejkaCalibration`, so the scaled `D` reaches them automatically. The falloff gate in `pacejka.py:118-138` was verified (see Lever 2 below), not modified. |

## Envelope validation (model vs Tomas measured)

Pure-physics harness `.tmp/grip_envelope_check.py` evaluates the front-axle single-channel
Pacejka peak (with FALLOFF + LS_EXP) at static per-wheel load (front 3766.4 N), as grip
coefficient in g. The v3 ODE plant runs at `mu_scale = 1.0` (no TyreState is threaded into
the ODE; `_mu_scale_from_tyre_state` returns 1.0), so the **mu=1.0 row is the operative
plant envelope**.

| scale | D_lat | D_lng | peak lat (g) | peak long (g) | long (m/s²) |
|---|---|---|---|---|---|
| 1.00 (legacy default) | 1.032 | 1.041 | 1.032 | 1.041 | 10.21 |
| 1.20 | 1.239 | 1.249 | 1.239 | 1.249 | 12.25 |
| 1.24 (D_lat = AC DY_REF 1.28) | 1.280 | 1.291 | 1.280 | 1.291 | 12.66 |
| **1.28 (D ≈ AC DY0/DX0 ~1.31)** | **1.321** | **1.332** | **1.321** | **1.332** | **13.07** |
| 1.32 | 1.363 | 1.374 | 1.363 | 1.374 | 13.48 |

Tomas measured (`.tmp/tomas_force_vectors.md`): peak lat ~1.50 g, peak long ~1.54 g
(−15.1 m/s²), **chicane decel ~13 m/s² (1.34 g)**, chicane lat ~1.25 g.

**Selected scale = 1.28** (baked into `tomas_highgrip.json`). It lands the spec's primary
gate — **chicane decel ~13 m/s² (13.07 m/s² at static load)** — and puts `D` at AC's raw
`DY0/DX0` semislick peaks, the most defensible anchor. The *static-load* single-channel peak
(1.32 g) is below Tomas's 1.5 g peak; that 1.5 g is hit in real corners where dynamic load
transfer raises the loaded-wheel `Fz` above static and `D·Fz` scales with it, so the
axle-summed peak in a full corner exceeds the static estimate. Scale > 1.32 was not chosen:
it over-grips mid-corner relative to the measured trace (spec R1).

## FALLOFF-floor verdict (Lever 2)

Verdict: **the floor HOLDS; no change to `pacejka.py` needed.** At scale=1.28, front lateral,
deep post-peak slip (20°): realised tail = 0.979 g, peak = 1.088 g → **tail/peak = 0.900 ≥
0.86 target**. The `tanh`-gated clip (`pacejka.py:118-138`) is *not* suppressing the floor
below `falloff_level` at the slip the chicane reaches — the unconstrained E-tail already
sits above 0.86×peak here, and the gate only ever raises the floor. The spec flagged a *risk*
that the soft gate might under-deliver; measurement shows it does not for this operating
point, so Lever 2 is satisfied as-is. (`falloff_level=0.86` is read and applied for both
lateral and longitudinal in `vehicle.py:457-484`.)

## Regression

- Default path (`grip_d_scale=None`): `D_lat=1.0323`, `D_lng=1.0408` — **unchanged**,
  byte-equal to pre-fix.
- Baseline reactive lap on `tomas.json` (default): finishes at **2:09.160**, MC σ=0.044 —
  matches the prior recorded behaviour.
- `tomas_highgrip.json` baked `D` (1.3214/1.3322) == `tomas.json × 1.28` (verified
  programmatically). The two carriers are interchangeable.
- v2 / point-mass grip (`Car` `grip_y`, reads AC ini) untouched.

## THE moment of truth — ideal-CSV bypass on the high-grip plant

Re-ran the previous task's bypass (`--hmpc-reference-source ideal_csv`, headline inner
config, both plant arms) with `grip_d_scale ∈ {1.20, 1.24, 1.28}` via
`.tmp/highgrip_bypass_run.py` / `.tmp/hmpc_ideal_bypass_sweep.py`.

| arm | scale | chicane v_max cap | finished | abort s | signature |
|---|---|---|---|---|---|
| baseline (doc) | — (1.03) | 12.9 m/s | 0/N | ~640 | inner stalls at apex |
| A (centerline) | 1.28 | 17.4 m/s | **0/1** | 601.9 | Tier-2 reactive (inner-infeasible-x2) then off-track |
| B (ideal-line) | 1.28 | 17.7 m/s | **0/1** | 603.8 | same |
| B (ideal-line) | 1.24 | — | 0/1 | 639.4 | inner-infeasible |
| B (ideal-line) | 1.20 | — | 0/1 | 630.3 | inner-infeasible |

**Honest result: the chicane still does NOT hold; the binding constraint shifted.** The
grip raise demonstrably propagated (the planner's chicane `v_max` cap rose 12.9 → 17.4/17.7
m/s with the higher `D_lat`, and the pure-physics check confirms the plant *can* pull
~13 m/s²). But with the planner now carrying more apex speed, the **inner MPC goes
Tier-2-reactive "inner-infeasible-x2" on chicane entry** and aborts off-track. Lower scales
(1.20/1.24) move the abort a few metres but never finish. MC was not run on a deterministic
abort (per the bypass-doc precedent).

**New binding constraint: inner-MPC tracking feasibility at the chicane**, not the plant
grip envelope. Raising `D` lets the planner *demand* more corner speed than the inner can
*track* through the chicane apex — the wall moved from "the plant can't decelerate" to "the
controller can't hold the line at the higher planned speed." Per spec §6 and the build
constraints, the HMPC controllers and the bypass code are out of scope here, so this is the
hand-off finding: the grip fix is necessary but not sufficient; the next lever is
inner-MPC / planner speed-cap reconciliation at the chicane (a controller task), not more
base `D`.

## How it integrates with neighbouring features

- **`architecture-v3-pacejka-nonlinear-load-sensitivity.md`** — that work wired
  FZ0/LS_EXP/falloff end-to-end; this fix rides on top, scaling the base `D` those terms
  modulate. No conflict: LS factor is 1.0 at static load, so the scaled `D` is the static
  peak unchanged in shape.
- **`architecture-v3-hmpc-ideal-line-bypass.md`** — that campaign proved the grip envelope
  was the *first* wall. This fix removes that wall (envelope now matches measured) and
  reveals the *next* one (inner tracking). The bypass code is consumed unchanged.
- v2 point-mass and reactive/v2 paths: untouched.

## Harnesses (`.tmp`, not committed)

- `.tmp/grip_envelope_check.py` — pure-physics envelope + falloff-floor validation (the
  cheap D-tuning loop).
- `.tmp/highgrip_bypass_run.py` + extended `.tmp/hmpc_ideal_bypass_sweep.py` (now accepts
  `grip_d_scale`) — the bypass moment-of-truth on the high-grip plant.
