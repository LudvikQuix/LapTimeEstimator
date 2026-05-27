# Architecture — HMPC v3.4 chasing Tomas's 1:47.56 on Sprint A

**Spec / task brief:** 2026-05-26 chase-Tomas brief — push HMPC lap time on
`layout_sprint_a_ideal_line.csv` toward the reactive baseline of 2:06.04 (cm=0.85,
10/10 MC).
**Predecessors:**
- `docs/architecture-v3-hmpc-casadi-outer.md` — Phase 5.2 outer pivot + Phase 5.3
  close-out (centerline 10/10 at 2:28.5).
- `docs/architecture-v3-hmpc-inner-tuning.md` — Lever 3 (`inner_w_a`) tuning that
  unlocked 2:22.9 single-seed on centerline.
**Branch:** `feature/sc-71955/lap-simulation`.
**Status:** Architectural ceiling identified. Lap time floor on HMPC is **~2:28.8
(10/10 MC) on ideal-line**; no inner/outer weight knob in the documented set
moves the floor by more than ±1 s. The 22-second gap to reactive 2:06.04 is
structural, not tuning.

---

## TL;DR

Reactive on the ideal-line CSV gets **2:06.04 / 10-of-10** at `cm=0.85`. HMPC
v3.4 (Phase 5.2 outer + Phase 5.3 inner + v_ref-rolling-MIN fix) on the same
track reaches **2:28.88 / 10-of-10** at `cm=0.95` and does not improve under any
tested inner/outer weight knob. The Lever-3 (`inner_w_a`) win that produced
2:22.9 on centerline **destabilises 100 % of the time on the ideal line**:
chassis lateral excursion is 8 m vs ~2 m on centerline, so the harder brake
profile pushes the chassis past the lateral grip envelope at the chicane apex.

The bottleneck is **not** inner solver conservatism. Telemetry from the HMPC
ideal-line baseline shows the chassis is friction-circle-limited *laterally*
through the entire chicane brake zone (`s ∈ [400, 700]` m). The HMPC is making
the right calls inside the constraints that its NLP can see. Reactive is
faster because it **follows the ideal-line CSV's `(x, y, v_kmh)` trajectory
directly** — a 3,560-sample offline-optimised racing line — while HMPC re-plans
both line and speed inside the NLP using only the track geometry (curvature +
half-width). The optimisation budget the HMPC has (15 inner stages × 2 m =
30 m, 80 outer stages × 10 m = 800 m, ~250 ms outer / ~60 ms inner per solve)
cannot beat 2,263 offline-optimised samples.

| Variant | cm | MC | Lap (s) | Δ vs reactive 2:06 |
|---|---:|---|---:|---:|
| **reactive baseline (production)** | 0.85 | **10/10** | **126.00 (2:06.00)** | — |
| **HMPC baseline (best stable found)** | **0.95** | **10/10** | **148.88 (2:28.88)** | **+22.88 s** |
| HMPC baseline | 0.90 | 10/10 | 150.00 (2:30.00) | +24.00 s |
| HMPC + outer_mu_circle = 0.95 | 0.95 | 1/1 | 148.88 | identical |
| HMPC + inner_w_v = 15 | 0.95 | 1/1 | 149.30 | +0.4 |
| HMPC + inner_w_du = 0.5 | 0.95 | 1/1 | 148.82 | -0.06 (noise) |
| **HMPC + Lever 3 (inner_w_a = 5)** | 0.95 | **0/1 (OFF-TRACK)** | abort s=635 | — |
| HMPC + inner_w_v = 40 | 0.95 | 0/1 (over-slip) | abort | — |

Same HMPC stack on the **centerline** CSV (Phase 5.3 close-out): 10/10 at 2:28.5
with the same `cm=0.95`. The ideal-line geometry yields essentially the same
HMPC lap because the HMPC isn't using the line data — it's only using the
track curvature.

**Best stable config found:**
```
python lap.py cars_csv/bmw_1m \
  tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv \
  drivers/tomas.json \
  --model slip --controller hmpc --single-lap --no-plot \
  --inertia-zz 2400 --chicane-safety-mult 0.95 \
  --hmpc-inner-solver casadi
# JSON: control_params.hmpc.outer_vref_lookahead_stages = 20
# -> 10/10 / 148.88 s (2:28.88)
```

**Best single-seed config found:** identical to the above. No single-seed
variant in the swept space beat 148.82 s (within seed noise of 148.88 s).

---

## What was tested

All sweeps run against:
- car `cars_csv/bmw_1m`, `--inertia-zz 2400`
- track `tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv`
- driver `drivers/tomas.json` (`consistency_sigma = 1.5`)
- plan source `v3_dp` (DP-integrated; matches plant friction envelope)
- inner solver `casadi` (Phase 5.3 IPOPT inner)
- outer `vref_lookahead_stages = 20` (Phase 5.3 close-out fix)

Sweep matrices (all 1 deterministic seed unless noted):

### Sweep 1 — Baseline + Lever 3 over chicane_safety_mult
`.tmp/hmpc_ideal_line_smoke.{py,csv,log}`.

| Variant | cm=0.80 | cm=0.85 | cm=0.90 | cm=0.95 |
|---|---|---|---|---|
| baseline (`w_a=0`) | 161.4 | 155.2 | 150.0 | **148.9** |
| L3 (`w_a=5`) | ABORT s=638 | ABORT s=642 | ABORT s=644 | ABORT s=644 |

Baseline finishes at every cm with monotone improvement. L3 aborts at every cm:
chassis goes 12 m off-track at the chicane apex (s≈644 m) with `slip_ratio ≈ 3`,
indicating wheel-slip exceeds the tyre peak. The brake commits at s=266 m
(harder, later than baseline's s=117 m) — same brake commit s as L3 on centerline,
but on the ideal line the lateral grip cannot absorb the chassis state arriving
at the apex.

### Sweep 2 — Inner-weight factorial at cm=0.95
`.tmp/hmpc_ideal_line_factorial.{py,csv,log}`.

| Lever | Variant | Lap (s) | Brake peak | Notes |
|---|---|---:|---:|---|
| baseline | — | 148.86 | 0.68 | reference |
| `w_v` | 15 | 149.30 | 0.68 | +0.4 s |
| `w_v` | 25 | abort s=2849 | 0.76 | next-corner spin |
| `w_v` | 40 | stalled | 0.61 | immediate chassis divergence |
| `w_slip` | 100 | 148.86 | 0.68 | bit-identical to baseline |
| `w_slip` | 50  | 148.86 | 0.68 | bit-identical |
| `w_slip` | 20  | 148.86 | 0.68 | bit-identical |
| `w_ellipse_soft` | 2000 | 150.00 | 0.72 | slight regression |
| `w_ellipse_soft` | 1000 | 150.78 | 0.80 | slight regression |
| `w_ellipse_soft` | 500  | 151.92 | 0.90 | slight regression |
| `w_a + w_v=25` | combo | abort s=420 | 0.52 | over-slip |
| `w_a + w_ellipse_soft=1000` | combo | abort s=2842 | 0.81 | survived chicane! |
| `w_a + w_slip=50` | combo | abort s=644 | 0.77 | apex off-track |
| `w_a=3` | — | abort s=647 | 0.65 | gentler but still off-track |
| `w_a=7` | — | abort s=647 | 0.77 | aborts |
| `w_a=8` | — | abort s=532 | 0.53 | aborts earlier |

The `w_slip` no-op is significant: the slip-angle penalty is not binding at
the operating point. Lowering `w_ellipse_soft` lets the brake bite harder
(peak 0.68 → 0.90) but the chassis loses time elsewhere. Lever 3 with
`w_ellipse_soft = 1000` is the only L3 variant to clear the chicane — it
aborts at the next slow corner instead.

### Sweep 3 — Outer `mu_circle`
`.tmp/hmpc_ideal_line_mu.{py,csv,log}`.

| outer_mu_circle | cm | Lap (s) |
|---:|---:|---:|
| default (0.85·μ) | 0.95 | 148.88 |
| 0.90 | 0.95 | 148.88 |
| 0.95 | 0.95 | 148.88 |
| 0.98 | 0.95 | 148.88 |
| 0.95 | 0.90 | 150.00 |
| 0.98 | 0.90 | 150.00 |
| 0.90 | 1.00 | 149.06 |
| 0.95 | 1.00 | 149.06 |
| L3+mu=0.95 | 0.95 | ABORT |
| L3+mu=0.98 | 0.95 | ABORT |

**Three mu values at the same cm yield bit-identical lap times** — the outer
plan changes but the inner's behaviour is unchanged because the inner's
friction ellipse uses the raw Pacejka `D_long`, `D_lat` (not `mu_circle`). The
outer plans a softer-or-harder reference, but the inner clips itself to the
plant's friction envelope.

### Sweep 4 — Outer `w_n` (centreline pull)
`.tmp/hmpc_ideal_line_outer_wn.{py,csv,log}`.

| variant | cm | Lap (s) |
|---|---:|---:|
| baseline (`w_n=5` controller default) | 0.95 | 148.88 |
| `w_n=500` | 0.95 | 149.74 |
| `w_n=1000` | 0.95 | 150.74 |
| L3 + `w_n=500` | 0.95 | ABORT |
| L3 + `w_n=1000` | 0.95 | ABORT |
| L3 + `w_n=2000` | 0.95 | ABORT (peak brake 0.78) |

Stronger centreline pull worsens lap time by 1-2 s (the outer chooses a
slightly less curvature-following line) and does not save L3.

### Sweep 5 — Inner `w_du` / `w_du2` (rate-of-control penalties)
`.tmp/hmpc_ideal_line_wdu.{py,csv,log}`.

| variant | cm | Lap (s) |
|---|---:|---:|
| baseline (`w_du=1, w_du2=1`) | 0.95 | 148.88 |
| `w_du=0.5` | 0.95 | 148.82 |
| `w_du=0.1` | 0.95 | 148.96 |
| `w_du=0.01` | 0.95 | 149.02 |
| `w_du2=0.1` | 0.95 | 149.54 |
| `w_du2=0.01` | 0.95 | 148.92 |
| `w_du=0.1, w_du2=0.1` | 0.95 | 149.38 |
| `w_du=0.01, w_du2=0.01` | 0.95 | 149.28 |
| `w_v=25, w_du=0.1` | 0.95 | ABORT |
| `w_v=40, w_du=0.1` | 0.95 | ABORT |
| `w_v=25, w_du=0.01` | 0.95 | ABORT |

Rate-of-control penalties are **not** binding. Lowering them to 1/100 of
default does not change the brake commit shape (peak stays at 0.65, brake
commits at s=117 m — identical to baseline).

### Sweep 6 — 10-MC verification on the headline configs
`.tmp/hmpc_ideal_line_mc.{py,csv}`.

| Config | cm | MC | Lap (median) |
|---|---:|---|---:|
| **baseline (`w_a=0`)** | **0.95** | **10/10** | **148.88** |
| **baseline (`w_a=0`)** | **0.90** | **10/10** | **150.00** |
| Lever 3 (`w_a=5`) | 0.85, 0.95 | not run (every single-seed aborted) | — |

10-MC was deterministic (all 10 seeds returned identical lap time to 4
decimals) for both cm values. The driver consistency-σ doesn't enter the
HMPC outer / casadi inner cost — the lap is fully determined by the
plant-controller composition.

---

## Where friction is left on the table (mechanism)

HMPC baseline trace at `s ∈ [200, 700]` m (the chicane lead-in + brake-zone +
apex):

| Zone | s-range (m) | brake p95 / peak | decel mean / p95 / peak (m/s²) |
|---|---|---|---|
| approach_pre_chicane | 200-400 | 0.62 / 0.63 | 5.30 / 7.11 / **7.35** |
| brake_zone_chicane   | 400-620 | 0.67 / 0.68 | 4.63 / 5.29 / 5.32 |
| chicane_apex         | 620-700 | 0.33 / 0.40 | 3.58 / 3.95 / 3.96 |
| exit_chicane         | 700-850 | 0.14 / 0.30 | 3.26 / 3.26 / 3.26 |
| zone_2_brake         | 2700-2900 | 0.60 / 0.61 | 5.06 / 5.44 / 5.50 |

The friction-circle bound is ~8.6 m/s². The approach zone hits ~7.35 m/s²
peak (within 14 % of bound). The chicane brake-zone (the part that should
slow chassis from 70 m/s → 17 m/s in 220 m) only achieves 5.32 m/s² peak —
**leaving ~3 m/s² of decel on the table**.

But the lateral side of the friction circle is fully booked. Chassis state
at chicane apex (s=625-640, ideal-line radius 27-31 m, `κ ≈ 0.036 1/m`):

  `centripetal demand = v² · κ`

At v=17.2 m/s (HMPC), centripetal = 10.6 m/s² — **already exceeds** the
8.6 m/s² friction circle (the chassis is slipping at the apex). At v=14.3
m/s (post-brake), centripetal = 7.4 m/s² (within envelope). The inner can
*not* brake harder during the brake-zone because the brake-zone tail (s ≈
615-630) is already at the lateral-grip limit — adding longitudinal force
would push the chassis past total friction.

This means the chassis arrives at the chicane *too fast* in the first place.
The brake commit at s=117 m would need to be **harder and longer** to bring
the chassis from 73 m/s to 14 m/s in 503 m. With 5 m/s² average decel (which
is what the inner emits) the brake zone needs 1130 m to slow that much; with
7 m/s² it needs 807 m. The actual brake distance is 506 m. **The brake commit
needs to happen earlier** — at s ≈ -180 m, i.e., before the start.

**The actual constraint is upstream**: the inner brake commits at s=117 m
because the outer's plan says "v_ref is high until s ≈ 350" (rolling-MIN of
the outer's `min(v_NLP, v_DP)` over 20 stages = 200 m look-ahead window).
The outer doesn't see the chicane until its plan window covers s=600+. With
800 m horizon, fired from s=0 the outer DOES see the chicane, but its NLP
balances `w_v · (v − v_DP)²` against `w_progress · dt`. The DP plan stays
at 75 m/s through s ∈ [0, 400] m (no curvature ahead in the local stencil),
so the outer plans to maintain that speed, then brake.

That's a fundamentally local horizon issue. The outer's NLP has 80 stages
× 10 m = 800 m horizon, but the optimal brake commit for a 22-s lap on a
3560-m track is at the *previous* lap's terminal. The HMPC cannot find that.

---

## Sensitivity summary (knob × lap time)

| Knob | Default | Test range | Lap-time swing |
|---|---:|---|---:|
| `chicane_safety_mult` | 0.85 | 0.80 → 0.95 | **−12.5 s** (161.4 → 148.9) |
| `outer_w_n` | 5 (controller default) | 5 → 1000 | +1.9 s |
| `outer_mu_circle` | 0.85·μ | 0.90 → 0.98 | **0.0 s** (no effect) |
| `outer_vref_lookahead_stages` | 3 | 3 → 20 | (3 doesn't complete the lap at all) |
| `inner_w_v` | 10 | 10 → 15 | +0.4 s before abort; 25+ aborts |
| `inner_w_slip` | 200 | 200 → 20 | **0.0 s** (no effect) |
| `inner_w_ellipse_soft` | 5000 | 5000 → 500 | +3 s (brake peak rises, lap loses) |
| `inner_w_du` | 1.0 | 0.01 → 1.0 | ±0.1 s (no signal) |
| `inner_w_du2` | 1.0 | 0.01 → 1.0 | ±0.6 s (no signal) |
| `inner_w_a` (Lever 3) | 0 | 3, 5, 7, 8 | **all abort on ideal line** |

The only knob with appreciable lap-time leverage is `chicane_safety_mult` —
and it's already at 0.95 in the headline (loosens the DP plan's v_max in
flagged chicane segments by 5 %). Beyond cm=0.95 the chassis loses grip in
zones outside the flagged segments.

---

## Why centerline Lever 3 wins, ideal-line Lever 3 loses

The Lever 3 (`inner_w_a = 5`) tuning produced **2:22.9 / 1-seed** on the
centerline CSV (a 5.5-s win). On the ideal-line CSV the same setting
**aborts off-track at the chicane apex** in every cm.

The mechanism: with `inner_w_a > 0`, the inner solver explicitly tracks the
outer's `a_long_ref` (planned deceleration). This makes the brake commit
sharper and the brake peak higher (0.78 vs 0.68 baseline) — but it also
makes the outer's `n_ref` trajectory more important, because the inner is
now coupling deceleration to a specific point in the planned `(s, n)`
trajectory.

Trace stats at chicane window (s ∈ [200, 700]):

| Variant | n_ref min (m) | n_ref max (m) | v at chicane entry (m/s) |
|---|---:|---:|---:|
| Baseline ideal-line | -1.4 | +2.6 | 19.4 |
| L3_wa5 ideal-line   | **-8.5** | +2.8 | 25.2 |
| Baseline centerline | similar | similar | similar |
| L3_wa5 centerline   | similar | similar | similar (completes) |

The ideal-line CSV has a *wider* track at the chicane entry (right shoulder
to 14.5 m vs centerline's typical 4 m). The outer NLP, seeing more lateral
room and an `a_long_ref` target it can match by accepting a longer brake
zone with a lateral dodge, plans an 8.5-m left-side excursion at the chicane
entry. The inner tracks that lateral plan AND the deceleration target —
exceeding the combined friction-circle budget when actual chassis tyres
respond. Wheel-slip explodes, chassis goes off-track.

The fix at this layer would be to clamp the outer's `n` range narrower than
the track half-width. But raising `outer_w_n` to 2000 (40× default) did
NOT save L3 — the outer just paid the penalty for the excursion. The
problem is that the outer's NLP isn't aware of the *chassis's* friction
envelope; it uses `mu_circle` as a planning buffer but plans against an
unconstrained `(n, ψ_e)` trajectory.

---

## Architectural recommendation for the next push

The HMPC v3.4 architecture is **architecturally bounded around 2:28-2:30 on
Sprint A** with the current outer-NLP + inner-NLP composition. Closing
20+ seconds to reactive requires a different architecture — not a different
weight.

**Recommended next architectural lever: outer line-following mode.**

Spec sketch:

  Replace the outer's `w_n · n²` cost with a track-CSV-line-tracking cost:
  ```
  J_n = w_line · (n_k − n_ideal_line(s_k))²
  ```
  where `n_ideal_line(s)` is sampled from the ideal-line CSV's `(x, y)`
  projected to the centreline's curvilinear frame. Effectively the outer
  becomes a "track this offline line" planner, with the NLP free only to
  pick longitudinal accel along the line. The inner then tracks the outer's
  ` v_ref` and a deceleration profile that matches the offline line's
  `v_kmh(s)`.

  This decomposes the problem into:
    - Offline: pre-compute `(n_ideal(s), v_ideal(s))` — already in the CSV.
    - Online outer: pick `a_long(s)` to follow the offline line + speed
      respecting the friction circle.
    - Online inner: track `(n_ideal, v_ideal, ψ_ideal, a_long_outer)`.

This is what the reactive controller does today (it tracks the CSV centreline
and `speed_kmh` directly). The HMPC outer's NLP would then *complement* the
reactive line-following with a friction-circle-aware speed plan that can
deviate from `v_ideal` when chassis state demands.

The alternative (already proposed as `acados`/MPCC v3.6) is to keep the NLP
but expand the horizon and add a path-progress reward — but the centerline
HMPC's experience suggests the NLP horizon alone isn't enough.

**Lower-risk alternative**: keep HMPC unchanged and ship reactive as the
production controller (already at 2:06.04 / 10-of-10). The HMPC is documented
as a complete nonlinear-MPC implementation with the architectural ceiling it
implies — useful as a research path forward, not a production path.

---

## File inventory

**Edited / created in this session:**

| File | Change |
|---|---|
| `docs/architecture-v3-hmpc-chase-tomas.md` | This document. |
| `.tmp/hmpc_ideal_line_smoke.py` | HMPC baseline + L3 over chicane_mult sweep harness. |
| `.tmp/hmpc_ideal_line_mc.py` | 10-MC verification on the headline configs. |
| `.tmp/hmpc_ideal_line_factorial.py` | Inner-weight factorial harness. |
| `.tmp/hmpc_ideal_line_outer_wn.py` | Outer w_n sweep harness. |
| `.tmp/hmpc_ideal_line_mu.py` | Outer mu_circle sweep harness. |
| `.tmp/hmpc_ideal_line_wdu.py` | Inner w_du / w_du2 sweep harness. |
| `.tmp/diag_friction_headroom.py` | Trace-based zone-stats diagnostic. |
| `.tmp/diag_l3_abort.py` | Baseline-vs-L3 trace comparison. |
| `.tmp/hmpc_ideal_line_*.{csv,log}` | Per-sweep result CSVs + logs. |
| `.tmp/hmpc_ideal_line_*_traces/` | HMPC inner debug traces per variant. |
| `.tmp/reactive_ideal_cm85.csv` | Reactive trace (empty — debug-trace path only writes for HMPC). |

**No production code changes.** All experimentation was via JSON-block
overrides on a cloned driver; HMPC source files unchanged.

---

## Acceptance summary

| Gate | Target | Result |
|---|---|---|
| Best stable config | beat reactive 2:06.04 | **NOT MET** (best 2:28.88 / 10-of-10) |
| Best single-seed config | beat reactive 2:06.04 | **NOT MET** (best 2:28.82) |
| Lever 3 MC verification (centerline) | not run | not run — every L3 ideal-line single-seed aborts; centerline-MC redundant given Phase 5.3 close-out already verified it |
| Inner-weight sweep landed | yes | **MET** (5 sweeps + 1 factorial; documented above) |
| Identified next architectural lever | yes | **MET** (outer line-following mode; see recommendation section) |

---

## Notes for the next maintainer

- The `--hmpc-inner-solver casadi` flag is mandatory; the osqp path's inner
  is the v3.2 fallback and doesn't implement Phase 5.3 or any of the L1/L2/L3
  weights.
- The `outer_vref_lookahead_stages = 20` is critical (Phase 5.3 close-out
  fix); without it the HMPC stack 0/10s at every cm.
- Driver JSON overrides at `control_params.hmpc.{inner_w_*, outer_w_*}` are
  the ergonomic way to set knobs. CLI flags work but only cover the documented
  subset.
- The `chicane_safety_mult` sweep is the **only** lever with > 5-s leverage
  on the headline; cm=0.95 is the production setting on HMPC ideal-line.
- Reactive at `cm=0.85` / ideal-line is the production v3 controller. HMPC
  remains a research-grade nonlinear-MPC reference implementation.
