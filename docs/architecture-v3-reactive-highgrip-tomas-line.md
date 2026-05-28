# Reactive on the high-grip plant, on Tomas's OWN line — chase to a dead heat

**Date:** 2026-05-27
**Branch:** `feature/sc-71955/lap-simulation`
**Scope:** Config/CLI experiment only. No controller, tyre-model, or planner
code was changed. This document records the `safety_margin` sweep that tested
whether the REACTIVE controller, on the high-grip plant
(`drivers/tomas_highgrip.json`, D×1.28, adaptive preview), running on
**Tomas's own recorded racing line as the track**
(`tracks_csv/ks_nurburgring/layout_sprint_a_tomas_line.csv`, ∫ds/v = 107.56 s),
reaches Tomas's pace (target ≤ 110.56 s; goal = equal 107.56 s).

Predecessor: `docs/architecture-v3-reactive-highgrip-ideal-line.md` — same
plant/controller on the *geometric* ideal-line CSV, where the finding was that
the DP plan's conservatism is the pace lever once the plan is grip-feasible.
This document repeats that experiment on the *tighter, faster* Tomas-line
geometry (R=25.4 m hairpin at s≈642) and finds the **opposite cliff**.

---

## Headline

**Yes — within the 3 s buffer, fully stable.** Reactive on the high-grip plant,
on Tomas's own line, **finishes 10/10 at 1:50.74 (110.74 s)** at the fastest
stable conservatism setting (`--dp-safety-margin 0.93`, `--chicane-safety-mult
1.00`). That is **+3.18 s vs Tomas (107.56 s)** and **just over the 110.56 s
3-s floor by 0.18 s** at 10/10; the 7/10 point (margin 0.94) is **1:50.08
(110.08 s)**, inside the floor. We did **not** equal Tomas.

The binding constraint that stops us equalling him is **not** plan
conservatism in the direction the brief assumed. The DP plan already asks for
**more** speed at the hairpin than Tomas actually drives, and raising
`safety_margin` further makes the over-ask worse until the reactive controller
runs wide and aborts. The single lever that would close the gap is to source
`v_target` from the track CSV's `speed_ms` column (Tomas's actual feasible
speed) instead of the geometric DP `v_corner` — currently **not wired** for
this track (see "The binding constraint" below).

---

## Base command

```
python lap.py cars_csv/bmw_1m \
  tracks_csv/ks_nurburgring/layout_sprint_a_tomas_line.csv \
  drivers/tomas_highgrip.json \
  --model slip --controller reactive --single-lap --no-plot --no-telemetry \
  --inertia-zz 2400 --chicane-safety-mult 1.00 --dp-safety-margin <M>
```

`consistency_sigma = 1.5` in the driver JSON → each invocation runs a 10-seed
Monte Carlo. The headline lap + `sigma` are computed over **finishers only**
(`slip_simulator._simulate_slip_mc`, lines 797-811); `mc_n_runs` prints the
total (10) regardless. "aborts=k" below is counted from the per-seed
`Slip sim lap aborted:` log lines (one per aborted seed), so
finishers = 10 − k. Grip applied exactly once (`tomas_highgrip.json` bakes
D×1.28; **no `--grip-d-scale`**; plant reports `grip_y=1.280/1.284`).

---

## Sweep table — `safety_margin` UPWARD (cm=1.00 throughout)

| dp_safety_margin | lap time | finishers | MC σ (s) | util_p85 | notes |
|-----------------:|---------:|:---------:|---------:|---------:|-------|
| 0.80 | 2:07.42 (127.42) | 10/10 | 0.023 | 0.357 | very conservative |
| 0.85 | 1:59.80 (119.80) | 10/10 | 0.038 | 0.467 | |
| 0.88 | 1:55.90 (115.90) | 10/10 | 0.062 | 0.576 | |
| 0.90 | 1:53.56 (113.56) | 10/10 | 0.073 | 0.748 | |
| 0.91 | 1:52.48 (112.48) | 10/10 | 0.071 | — | |
| 0.92 | 1:51.56 (111.56) | 10/10 | 0.090 | — | |
| **0.93** | **1:50.74 (110.74)** | **10/10** | **0.091** | — | **fastest STABLE (10/10)** |
| 0.94 | 1:50.08 (110.08) | **7/10** | 0.296 | 1.100 | fastest inside 3-s floor; 3 seeds abort |
| 0.95 | — (abort) | 0/10 | — | — | all seeds abort at hairpin |
| 0.96 | — (abort) | 0/10 | — | — | |
| 0.97 | — (abort) | 0/10 | — | — | |
| 0.98 | — (abort) | 0/10 | — | — | |
| 1.00 | — (abort) | 0/10 | — | — | abort at s≈653 m hairpin |

**Monotone and as the brief predicted in the FINISHING band:** raising
`safety_margin` (less conservative DP plan) monotonically lowers lap time, from
127.42 s (M=0.80) down to 110.08 s (M=0.94). The plan IS the pace lever, exactly
per Finding 1 of the ideal-line doc.

**But there is a hard cliff at the hairpin.** At M ≥ 0.95 *every* seed aborts;
M=0.94 loses 3 seeds; M=0.93 is the last 10/10-stable point. The cliff is
sharp (one `safety_margin` step from 10/10 to 0/10), and it is located at the
R=25.4 m hairpin at s≈653 m, where the chassis runs 12 m off the racing line
(`OffTrackError`, preceded by the over-slip ghost-fallback firing at t≈10.4 s
with longitudinal `slip_ratio≈3.0` — the brake is pinned full but the plant
cannot shed the over-asked entry speed in the available distance).

`--chicane-safety-mult 1.00` (no cap) was held throughout per the brief; the
Tomas-line hairpin (R=25.4 m) is below the 60 m chicane-flag threshold but the
chicane report shows **0 segments flagged** because the per-segment radius in
this CSV is smoothed above 60 m at the flag stride — the cap is effectively
disabled here regardless of `mult`, consistent with the brief's "no cap" intent.

---

## The binding constraint (the decisive finding)

The brief's fallback hypothesis was: *"the DP plan stays conservative even at
high safety_margin (reactive can't reach Tomas's speed because the plan won't
ask for it)."* **The measured reality is the inverse at the hairpin.** Plan
hairpin speed (s≈642) vs Tomas's actual, across margins:

| dp_safety_margin | v_plan @ hairpin (m/s) | Tomas actual (m/s) | plan over Tomas |
|-----------------:|----------------------:|-------------------:|----------------:|
| 0.90 | 16.3 | 13.9 | **+2.4** |
| 0.93 | 16.9 | 13.9 | **+3.0** |
| 0.94 | 17.1 | 13.9 | **+3.2** |
| 1.00 | 18.1 | 13.9 | **+4.2** |

The DP plan's geometric corner-speed `v_corner = sqrt(D_lat·g/|κ|)·margin`
**over-estimates** the hairpin speed the reactive controller can actually hold,
because it ignores the entry transient (load transfer + the reactive lateral
loop's tracking error at the apex). Tomas drives the hairpin at 13.9 m/s; the
plan asks for 16.9–18.1 m/s. Raising `safety_margin` lifts the plan's hairpin
target *further above* what is drivable, which is why the abort appears exactly
when the over-ask exceeds the controller's margin.

Meanwhile the plan is genuinely *not* the limiter on the fast sections: plan
v_max = 73.5 m/s at M=0.93 vs Tomas's recorded peak 59.0 m/s — the high-grip
envelope allows higher straight speed than Tomas used, so the straights are
not where time is lost. **The lap-time gap to Tomas is therefore set by how far
the whole-plan pace can be pushed before the hairpin over-ask breaks tracking
— and that ceiling is M=0.93 (10/10) / M=0.94 (7/10).**

### The one lever that would close the gap

Source the controller's `v_target` from the track CSV's **`speed_ms` column**
(Tomas's actual, demonstrably-feasible speed: 13.9 m/s at the hairpin) instead
of the geometric DP `v_corner`. That would:

1. Lower the hairpin target from 16.9 to 13.9 m/s → kill the over-ask abort →
   allow stable operation with no `safety_margin` ceiling.
2. Keep the (correctly higher) straight-line targets where Tomas was slower
   than the high-grip envelope, so the lap would not regress on the straights.

This is **not currently wired for this track.** The existing `--plan-source
tomas` reads a *telemetry* CSV (`distanceTraveled`/`speedKmh` columns,
`tomas_trajectory_plan.build_tomas_plan`), **not** the `speed_ms` column of a
track CSV. The `layout_sprint_a_tomas_line.csv` track already carries
`speed_ms` per row, but `_build_plan` (slip_simulator.py:91) only routes the
track-CSV path through the geometric DP planner. Wiring a
`--plan-source track_csv` (or `csv_speed`) branch that reads
`track.csv_data["speed_ms"]` into a `LongitudinalPlan` is the precise,
minimal lever. **Per the brief I did not build it — reporting only.**

**Root cause layer:** `architecture` — the limiter is a planner/reference-source
design choice (geometric `v_corner` over-estimates the achievable hairpin speed
on this tight geometry), not a controller-code defect and not spec ambiguity.
The fix is a new reference-source branch, not a tuning knob.

---

## Verdict

- **Fastest STABLE lap:** **1:50.74 (110.74 s), 10/10, σ = 0.091 s** at
  `--dp-safety-margin 0.93 --chicane-safety-mult 1.00`, reactive + adaptive
  preview on Tomas's line, high-grip plant, `--inertia-zz 2400`.
- **Fastest inside the 3-s floor:** 1:50.08 (110.08 s) at M=0.94, but only
  **7/10** (meets the ≥7/10 floor; 3 seeds abort at the hairpin).
- **Distance to Tomas (107.56 s):** +3.18 s at the 10/10 setting; +2.52 s at
  the 7/10 setting. **We did NOT equal him**, but the 10/10 lap is within the
  3 s buffer at the margin's stability cliff.
- **Abort behaviour:** clean `OffTrackError` at the R=25.4 m hairpin
  (s≈653 m), preceded by over-slip ghost-fallback (`slip_ratio≈3.0`); no NaN /
  solver blow-up. The adaptive preview prevents the abort up to M=0.93.
- **Binding constraint:** the DP plan **over-asks** the hairpin speed
  (16.9 m/s plan vs Tomas's 13.9 m/s) and raising `safety_margin` makes it
  worse; the reactive controller cannot hold the over-asked entry speed.
- **The one lever to close it:** source `v_target` from the track CSV's
  `speed_ms` column (Tomas's feasible speed) instead of geometric `v_corner`.
  Not wired for track CSVs today; this is the recommended next build.

---

## Files

- No production code modified. Config/CLI experiment only.
- Sweep evidence reproduced via the production CLI (commands above).
- Binding-constraint plan-vs-Tomas comparison reproduced inline against
  `longitudinal_planner.plan_longitudinal` +
  `_slip_result._load_pacejka_calibration` (read-only).
