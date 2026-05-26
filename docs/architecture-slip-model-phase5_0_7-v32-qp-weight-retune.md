# Phase 5.0.7 — MPC QP weight re-tune (negative result; escalation flagged)

**Predecessor specs / docs:**
- `docs/architecture-slip-model-phase5_0_6-v32-tier1-debrake-and-thresholds.md` — Tier 1 brake-handoff + threshold tuning; flagged this phase as the next step in the "Root cause of the residual abort" section.
- `docs/architecture-v3-session-2026-05-24.md` — v3 session index (reactive 7/10 baseline).
- Spec §23.2-5.0.4 (per-stage dynamic Fz) and §23.2-5.0.1 (ellipse hard constraint) — still active.

**Scope:** Diagnostic + re-tune of the existing `MPCWeights` (no code changes).
Coordinate-descent sweep over `(w_psi, w_lat, w_slip, w_v)` to test the
prior architect's hypothesis that the residual MPC chicane abort is closeable
via cost-weight changes alone.

**Status:** Negative result. The QP cost-weight knob is structurally
insufficient — best-found weights leave the v_x gap essentially unchanged
and MPC remains 0/10 on Sprint A at both `--chicane-safety-mult 0.85` and
`0.80`. Phase 5.0.8 escalation (slip-aware MPC longitudinal channel) is
recommended.

---

## Headline

QP weights do not move v_x at the chicane braking zone. Best-found weights
trim `|α_f` gap` from 5.58° → 4.60° but `Δv_x` only from +1.74 → +1.61 m/s,
which is below the 0.3 m/s acceptance threshold the prior architect set.
0/10 MC at chicane mults 0.85 and 0.80, `--inertia-zz 2400`.

---

## Diagnostic — what is really happening

### Per-arc-length trace, deterministic single-seed at mult=0.85

Comparing reactive vs MPC at matched `s` samples (`.tmp/phase5_0_7_alpha_diag3.py`):

```
    s_m |   v_re  α_re  fxF_re  fxR_re | v_mpc α_mpc fxF_mpc fxR_mpc |   Δv
   540  |  37.42  0.54   -3303   -1681 | 37.72  0.31   -3203   -1633 | +0.31
   570  |  32.13  0.20   -3315   -2001 | 32.82  0.00   -3281   -2009 | +0.70
   590  |  28.17  1.13   -3166   -1865 | 28.94  0.12   -2955   -1793 | +0.77
   610  |  24.14 10.87   -2223   -1366 | 25.68  2.09   -1938   -1376 | +1.54
   620  |  22.06  9.03   -2235   -2024 | 23.98  4.78   -1968   -1369 | +1.92
   640  |  18.27 15.66   -2149   -1922 | 20.01 19.72    -936   -2325 | +1.74
   656  |  13.77 17.10   -2154   -2830 | 14.56 17.83   -2135   -2851 | +0.79
```

The findings (units: m, m/s, degrees, Newtons):

1. **The v_x gap originates well before the chicane.** Δv climbs from
   +0.31 m/s at s=540 to +0.70 m/s at s=570 — both still on the Hatzenbach
   approach straight, ~75 m before the chicane braking zone proper. By
   s=590 (still 30 m before the chicane) Δv is +0.77 m/s and MPC's |α_f|
   is **0.12°** vs reactive's 1.13°. MPC is straight-tracking while
   reactive has already initiated a small turn-in.
2. **MPC's front-axle longitudinal force is consistently lower.** At
   s=550 fxF_re=−3346, fxF_mpc=−3096 (7 % less front braking). The rear
   axle force is similar between the two. The total decel is smaller for
   MPC despite identical car/plan/track.
3. **MPC's |α_f| is systematically lower in the pre-chicane straight
   (s=540–620) where the gap accumulates.** Inside the chicane (s>620)
   the |α_f| values are noisier because both controllers fight the
   transient. The accumulated v_x error is set BEFORE the chicane.

The implication: the v_x carry-over is not "MPC under-rotates at the
corner so the sub-controller relaxes the brake". It is upstream of that.
MPC commits less steering than Stanley would because the QP's lateral
cost is dominated by the friction-ellipse hard constraint (Phase 5.0.1),
not the e_lat / e_psi quadratic. The reactive sub-controller (delegated
longitudinal channel inside `MPCController._emit_tier0_controls`) reads
its OWN Stanley `steer_cmd` (not the MPC's commanded δ) when computing
the slip-band trail-brake taper at `driver_controller.py:469`. Both
controllers therefore see "the plan says steer here, so taper brake by
X%" identically — but the actual chassis state differs because MPC
emits a different δ to the plant.

### The architectural mechanism

The mode in summary:

```
MPC tick:
  - QP solves for (δ, throttle, brake) sequence
  - Ellipse hard constraint binds; the lateral cost prices e_lat/e_psi
    only modestly above the slack penalty
  - MPC emits δ_MPC (lower than Stanley's preview-Stanley δ_S)
  - MPC discards its own throttle/brake; delegates to DriverController

DriverController (sub-controller) tick:
  - Reads chassis state (which is the MPC-driven trajectory)
  - Computes its OWN steer_cmd via preview-Stanley
  - Computes brake from speed PI + steer_cmd-based trail-brake taper
  - Returns (steer_S, throttle_S, brake_S)
  - MPC keeps steer_MPC, ignores steer_S, emits (steer_MPC, throttle_S, brake_S)

Result: brake is computed FROM the Stanley plan, but applied TO a
chassis that is being steered by δ_MPC. Stanley taper "knows" how much
the car is steering only via the plan, not via what was actually
commanded — so the brake schedule is the same whether MPC or reactive
is driving. But MPC's lower-amplitude δ commits leave the chassis on a
shallower trajectory, which means at any given s the chassis is going
faster, so v_x is higher, so brake-applied-to-mass yields less Δv per
unit time, so v_x stays higher. Positive feedback through the steering
channel.

This is unbreakable by cost-weight changes inside the QP because:
- The friction-ellipse constraint (5.0.1) is what binds, not the cost.
  Weights re-rank infeasibilities; they do not relax the ellipse.
- The longitudinal channel is OUTSIDE the QP. Increasing w_v doesn't
  shrink Δv_x because the QP's `throttle` / `brake` decisions are
  discarded.
```

---

## Sweep design + results

Coordinate descent in three stages (deterministic single-seed reactive
baseline, then `(w_psi, w_lat, w_slip)` cycled). Each MPC run at chicane
mult 0.85, `--inertia-zz 2400`, consistency_sigma=0 for repeatability.
Metric: mean |α_f_MPC − α_f_reactive| over `s ∈ [617, 660]` at 2 m sample
spacing.

Default seed (5.0.6): `w_psi=20, w_lat=50, w_slip=200` —
`|Δα|_mean = 5.56°`, `Δv_mean = +1.72 m/s`.

### Stage 1 — sweep w_psi at w_lat=50, w_slip=200

| w_psi | \|Δα\|_mean (deg) | Δv_mean (m/s) |
|---:|---:|---:|
| 10 | 5.58 | +1.74 |
| 20 | 5.56 | +1.72 |
| 30 | 6.20 | +1.70 |
| 50 | 5.45 | +1.66 |
| **75** | **5.30** | **+1.62** |

Winner: w_psi = 75 (modest 5 % improvement on |Δα|, 7 % on Δv).

### Stage 2 — sweep w_lat at w_psi=75, w_slip=200

| w_lat | \|Δα\|_mean (deg) | Δv_mean (m/s) |
|---:|---:|---:|
| 25 | 5.44 | +1.77 |
| 50 | 5.30 | +1.62 |
| 75 | 5.34 | +1.60 |
| 100 | 4.77 | +1.61 |
| **150** | **4.60** | **+1.62** |

Winner: w_lat = 150 (15 % cumulative improvement on |Δα| over baseline).

### Stage 3 — sweep w_slip at w_psi=75, w_lat=150

| w_slip | \|Δα\|_mean (deg) | Δv_mean (m/s) |
|---:|---:|---:|
| **100** | **4.60** | **+1.61** |
| 200 | 4.60 | +1.62 |
| 400 | 4.61 | +1.62 |
| 600 | 5.21 | +1.65 |

Winner: w_slip = 100 (flat from 100 to 400; degrades at 600).

### Side check — w_v

The cost-weight sweep included a side check on `w_v` (currently 0.5
because longitudinal is delegated; the QP's speed-tracking cost only
influences the linearisation):

| w_v | finished? | v_mean(540..600) | Tier 2 entry |
|---:|---:|---:|---:|
| 0.5 (default) | no @ 13.6s | 32.27 m/s | t=11.3s |
| 5.0 | no @ 13.1s | 33.02 m/s | t=4.3s |
| 20.0 | no @ 13.5s | 32.07 m/s | t=3.7s |

Higher w_v makes things worse, not better — it forces the QP onto an
infeasibility cliff earlier and the Tier 1 cascade re-engages
prematurely. Confirms the diagnosis: increasing the QP's speed-tracking
weight cannot fix a problem whose actuators are outside the QP.

### Final 10-MC at best weights (`w_psi=75, w_lat=150, w_slip=100`)

```
chicane-safety-mult 0.85:
  seeds 0..9: ALL FAIL with OffTrackError at s≈655 m, t≈13.6s
  tier fractions: T0=80.1%, T1=3.0%, T2=16.9%  (median over 10 seeds)
  median lap time: n/a (0 completions)

chicane-safety-mult 0.80:
  seeds 0..9: ALL FAIL with OffTrackError at s≈655 m, t≈13.6s
  tier fractions: T0=80.3%, T1=3.0%, T2=16.8%
  median lap time: n/a (0 completions)
```

Compare to the 5.0.6 baseline at the SAME flags: T0=81.6%, T1=3.0%,
T2=15.4%, 0/10. The re-tune nudges the tier distribution by <2 % on T2.
There is no Tier-fraction improvement story to tell; the re-tune is
flat across the board on the operational metric.

### Acceptance gate

Phase 5.0.7 brief: MPC ≥7/10 at chicane mult 0.80 with `--inertia-zz 2400`,
lap time competitive with reactive 2:09–2:12.

**Verdict:** Failed. 0/10 at both 0.85 and 0.80; no MPC completions at
any weight in the sweep at any mult. Reactive baseline (same flags,
sigma=0 deterministic): 2:05.96.

---

## Why weights cannot close this

Mechanism stated above; restated as a constraint:

```
MPC chassis state at s = (v_MPC(s), x_MPC(s), y_MPC(s), ψ_MPC(s), ...)
DriverController-as-sub-controller commits:
  brake(s) = f_brake(state, plan, Stanley_steer(state, plan))
  Stanley_steer's effect on f_brake: trail-brake taper only.

  Stanley_steer is computed FROM the chassis state, INDEPENDENTLY of
  whatever the MPC commits. So MPC's δ_MPC influences brake(s) ONLY
  via its effect on chassis state — second-order, weak, and the
  feedback loop has the wrong sign:
    higher v_x_MPC -> chassis further from plan's intended preview
      → larger Stanley steer demand (which the MPC does not commit)
      → no direct brake amplification, only via taper
    Net: brake stays at ~the same %, v_x stays at +1.6 m/s offset.
```

The QP's cost weights affect ONLY the choice of δ_MPC and the (unused)
QP throttle/brake. Since δ_MPC influences the longitudinal channel only
through the second-order chassis-state path, no QP weight can change
the closed-loop v_x gap by more than the gain of that path — empirically
~0.1 m/s out of the +1.7 m/s offset.

---

## Recommendation — Phase 5.0.8

Per the 5.0.6 architect's escalation path, the fix is to replace the
reactive sub-controller's longitudinal channel with a slip-aware MPC
longitudinal channel. The shape of the change:

1. Promote `throttle` / `brake` from "delegated to sub-controller" back
   to "first-class QP decision variables", with the **same** ellipse
   hard constraint already in place from 5.0.1 governing how (Fx, Fy)
   per axle relate. The QP is already aware of `(throttle, brake)` —
   they ARE in the decision vector; we just emit them now and don't
   delegate.
2. Honest plan-tracking via the QP's `w_v` cost rather than the
   sub-controller's speed PI. With Phase 5.0.4's per-stage dynamic Fz
   the `k_throttle · throttle` approximation that originally pushed
   us to delegate is now refreshed against the predicted (a_x, a_y)
   state, so it's no longer too coarse.
3. Keep the Tier 1 ellipse-saturation feedforward (5.0.3 + 5.0.6
   debrake fix) for SQP-infeasibility recovery, but emit the saturated
   per-axle (Fx_sat, Fy_sat) inverse all the way to (throttle, brake)
   now that MPC owns longitudinal, rather than the 5.0.6 "lateral-only
   saturation, longitudinal from sub-controller" compromise.
4. Validate against the same Sprint A chicane single-seed deterministic
   trace. Acceptance: MPC `Δv_x` in [617, 660] zone within 0.3 m/s of
   reactive, completions ≥7/10 at mult 0.80.

Spec for 5.0.8 to be written by Buddy. Architecture surface change is
moderate (the QP, the Tier 1 emit, and `MPCController._emit_tier0_controls`).
Plant model is unchanged. Plan source is unchanged.

---

## File inventory

No production code changed in this phase. The investigation is documented
here so 5.0.8 starts with the right framing.

Diagnostic scripts (kept in `.tmp/` for reproducibility):

- `.tmp/phase5_0_7_alpha_diag.py` — α_f zone summary (mean / p85 / max)
  for reactive vs MPC at deterministic single-seed.
- `.tmp/phase5_0_7_alpha_diag2.py` — per-arc-length α_f and v_x at matched
  s samples in [617, 660].
- `.tmp/phase5_0_7_alpha_diag3.py` — extended s range [540, 670] including
  per-axle Fx; produces the per-arc-length table cited above.
- `.tmp/phase5_0_7_sweep.py` — coordinate-descent weight sweep + 10-MC at
  best weights for mult 0.85 / 0.80.
- `.tmp/phase5_0_7_sweep.log` — captured stdout of the full sweep
  (negative result; preserved).

Default MPC weights at end of phase: **unchanged from 5.0.6** (w_psi=20,
w_lat=50, w_slip=200, w_v=0.5). Re-tuning to (75, 150, 100) shows no
operational benefit; not propagated.

---

## Cross-reference

See `docs/architecture-v3-session-2026-05-24.md` (session index) for
the broader v3 status. This doc records Phase 5.0.7 outcome: MPC is
unchanged, and the unbreakable structural deficit is named and
escalated to 5.0.8.
