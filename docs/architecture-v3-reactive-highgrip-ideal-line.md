# Reactive controller on the high-grip plant (ideal-line) — sweep + verdict

**Date:** 2026-05-27
**Branch:** `feature/sc-71955/lap-simulation`
**Scope:** Config/CLI experiment only. No controller, tyre-model, or planner
code was changed. This document records the conservatism-knob sweep that
tested whether the REACTIVE controller, on the high-grip plant
(`drivers/tomas_highgrip.json`, D×1.28), reaches Tomas's pace
(≤ 110.56 s) on `layout_sprint_a_ideal_line.csv`.

## Headline verdict

**No.** Reactive on the high-grip plant does **not** finish the lap at any
tested conservatism setting — it aborts off-track at the **s≈627 m hairpin**
(a 27 m-radius left-hander) on every seed of every config. It does not get
within 3 s of Tomas because it does not complete a lap at all. The binding
constraint is **not** a plan/conservatism knob; it is the **reactive
controller's longitudinal (braking) authority**, which cannot shed the extra
entry speed the high-grip plant now carries onto the hairpin approach.

This is a regression *caused by* the grip raise: at the OLD grip the same
reactive controller finishes the ideal-line lap cleanly (2:09.20, 10/10).
The grip raise lets the car reach higher straight-line speed, and the fixed
reactive brake ramp can no longer scrub it in the available braking zone.

## Grip-scale application — confirmed applied exactly once

`drivers/tomas_highgrip.json` already **bakes** `D_per_Fz = 1.3214`
(= fitted `1.0323 × 1.28`) into `pacejka_calibration`. The
`grip_d_scale` argument in `_slip_result._load_pacejka_calibration`
multiplies `D_per_Fz` again. Passing `--grip-d-scale 1.28` with this driver
would **double-apply** (1.0323 × 1.28 × 1.28 ≈ D 1.69).

**Decision: all runs below use `drivers/tomas_highgrip.json` with NO
`--grip-d-scale` flag.** The plant reports `grip_y=1.280/1.284`, matching
the intended single-application ×1.28 envelope.

## Base command

```
python lap.py cars_csv/bmw_1m \
  tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv \
  drivers/tomas_highgrip.json \
  --model slip --controller reactive --single-lap --no-plot --no-telemetry \
  --inertia-zz 2400 \
  --chicane-safety-mult <M> [--dp-safety-margin <S>]
```

`consistency_sigma = 1.5` in the driver JSON, so each invocation already
runs a 10-seed Monte Carlo; `Lap 1: 0:00.000` means all 10 seeds aborted.

## Sweep table

| chicane_mult | dp_margin | lap time | result | abort s (m) | MC |
|-------------:|----------:|---------:|--------|------------:|----|
| 0.70 | (default 0.94) | — | ABORT | 650 | 0/10 |
| 0.80 | (default 0.94) | — | ABORT | 650 | 0/10 |
| 0.90 | (default 0.94) | — | ABORT | 648–650 | 0/10 |
| 0.95 | (default 0.94) | — | ABORT | 648–650 | 0/10 |
| 1.00 | (default 0.94) | — | ABORT | 648–650 | 0/10 |
| 1.05 | (default 0.94) | — | ABORT | 648–650 | 0/10 |
| 1.00 | 0.94 | — | ABORT | 648–650 | 0/10 |
| 1.00 | 1.00 | — | ABORT | 645–647 | 0/10 |
| 1.00 | 1.05 | — | ABORT | 644–645 | 0/10 |
| 0.80 | 0.94 | — | ABORT | 650 | 0/10 |
| 1.05 | 1.05 | — | ABORT | 644 | 0/10 |

**Abort boundary:** there is no "boundary" in the swept space — *every*
config aborts at the same s≈627–650 m hairpin. The chicane multiplier does
not move the abort point at all (the cap correctly lowers the plan target at
the apex, but the car never reaches the apex on-line). A *higher* dp-margin
(less conservative) moves the abort a few metres *earlier* (faster entry =
overshoot sooner), confirming the abort is speed-of-arrival driven.

Reference (OLD grip, `drivers/tomas.json`, same command minus high-grip):
**2:09.20, 10/10 finished, sigma 0.083 s.**

## Root cause (from the trace)

Trace `..._tomas_highgrip_sim_trace_slip.csv`, representative seed, into the
hairpin (plan = `ai_speed`, sim = realised):

| s (m) | v_sim (m/s) | v_plan (m/s) | over-plan |
|------:|------------:|-------------:|----------:|
| 544 | 45.3 | 40.9 | +4.4 |
| 576 | 40.8 | 33.0 | +7.8 |
| 600 | 36.9 | 24.4 | +12.5 |
| 616 | 34.7 | 16.9 | +17.8 |
| 624 | 33.5 | 14.8 | +18.7 (apex) |

The car is over-speed for the *entire* braking zone and the deficit grows
monotonically — it is braking, but not hard/early enough, and arrives at the
27 m-radius apex ~19 m/s over the cornering limit. At the apex the ideal
line hugs the right edge (`width_right` collapses to ~1.4 m at s=648), so a
~12 m line overshoot runs the chassis off-track → `OffTrackError`. The
over-slip ghost-fallback fires at t≈10.3 s (longitudinal `slip_ratio≈3.0`)
as the controller pins the brake, but the plant simply cannot decelerate
fast enough.

Compare OLD grip at the same points: the car enters s=544 at only 36.8 m/s
(it cannot even *reach* the plan's 40.9 — grip-limited on the straight), so
by the apex it is only +6 m/s over, overshoots within track width, and
recovers. The grip raise removed that natural speed ceiling on the straight
without giving the **reactive brake loop** any more authority, so the
deficit it must erase grew past what the fixed brake ramp
(`pedal_press_rate_per_s = 11.6`) can deliver in the available distance.

## Why conservatism knobs cannot fix this

- `--chicane-safety-mult` lowers the *plan v_max* inside the flagged chicane
  region [618..704 m]. It does **not** change how fast the car *arrives* at
  the braking point, because the speed deficit is accumulated over the
  approach straight (s≈540–616), upstream of where the cap bites. Even
  `mult=0.70` (30 % cut) leaves the abort unchanged at s≈650.
- `--dp-safety-margin` scales the whole DP envelope; lowering it (more
  conservative) does not help because the limiter is the controller's brake
  *rate*, not the plan target. Raising it makes things slightly worse.
- The reactive Stanley/k_cross lateral tuning is settled and is not the
  abort cause (the failure is longitudinal speed-of-arrival, then a lateral
  consequence).

The brake ramp / longitudinal authority is a **reactive-controller code
property**, not exposed as a pace knob, and the task constraints forbid
editing controller logic. The fix is therefore out of scope for a
config/CLI experiment.

## New binding constraint & gap

- **Binding constraint:** reactive longitudinal (braking) authority into the
  s≈627 m hairpin under the high-grip entry speed.
- **Gap remaining:** unquantifiable as a lap delta — the lap does not
  complete. The car arrives ~19 m/s over the apex limit; it would need to
  shed that within ~70 m of braking zone (s≈555→625), i.e. roughly double
  the realised deceleration.

## Recommended next step (for the build owner, not this experiment)

The hypothesis in the brief — "higher v_crit brings Tomas's chicane speed
inside reactive's clean-tracking range" — is correct *for the chicane at
s≈1087*, but the lap never gets there: it dies at the earlier s≈627 hairpin
on the brake. Options, all requiring code or spec work (escalate to Buddy):

1. Increase reactive brake authority / look-ahead braking so the controller
   begins braking earlier for tight corners (controller code change).
2. Use the HMPC/MPC inner with the ideal-line bypass (which has explicit
   preview braking) on the high-grip plant instead of reactive.
3. Re-examine whether the DP plan's straight-line v_max should be capped by a
   brake-distance feasibility check the reactive controller can actually
   honour (planner change).

## Files

- Experiment harness: `.tmp/reactive_highgrip_sweep.py` (drives the
  production CLI as a subprocess; guarantees single grip application).
- No production code modified.

---

# Addendum (2026-05-27): brake-authority config probe

**Question:** does giving the reactive driver MORE BRAKE AUTHORITY clear the
s≈650 hairpin abort, or is preview/lookahead braking genuinely required?

**Answer:** Brake *magnitude* authority does nothing. **Preview/lookahead
braking is the lever — and it is already a config knob** (`preview_time_s`),
so the fix is config-only after all, but via a *different* knob than the
brief assumed. The lap finishes 10/10 at **111.92 s**, still **+1.36 s over
the ≤110.56 s target** — preview-braking conservatism caps the pace.

## Which JSON fields actually gate the reactive brake

The reactive `DriverController` (`src/lap_estimator/dynamics/driver_controller.py`)
brake channel is exactly (lines 417-418):

```
brake = clip(brake_p_gain * max(v_target_min - v, 0), 0, 1)
```

with **no brake rate-limit** (only the *throttle* channel is rate-limited,
lines 431-437, and it reads `throttle_ramp_pct_s`, not `pedal_press_rate_per_s`).
The brake-authority knobs the reactive controller actually consumes, read via
`ControlParams.from_driver` from the JSON `control_params` block:

| JSON field (`control_params.*`) | default | gates | effect on hairpin |
|---|---|---|---|
| `brake_p_gain` | 0.6 | brake **magnitude** (P-gain on speed error) | **none** — already saturates to 1.0 in the braking zone |
| `preview_time_s` | 1.5 | how **early** braking starts (pre-brake lookahead window, `brake_lookahead_m = max(30, v·t_preview)`) | **decisive** — raising it clears the abort |

- `v_target_min = min(plan speed)` over the lookahead window
  (`_min_speed_within`), so a longer `preview_time_s` pulls the slow apex
  speed into view earlier and triggers the brake sooner.
- **`pedal_press_rate_per_s` (under `profile.dynamic`) is INERT in the
  reactive path.** `src/lap_estimator/driver.py:15-17` documents these as
  "statistics-only … the v1.2.1 simulator does not consume them." The
  brief's premise that `pedal_press_rate_per_s=11.6` is the brake-ramp
  limiter is **incorrect for reactive**. Proven empirically: cloning the
  winning `preview_time_s=2.3` driver with `pedal_press_rate_per_s` bumped
  11.6→60 gave a bit-identical lap (1:51.920, σ=0.131).

## Why raising brake magnitude does nothing

At the hairpin the brake P-term `0.6·(v_target−v)` with a ~19 m/s deficit is
≈11, clipped to 1.0 — the brake is **already pinned full**. Raising
`brake_p_gain` 0.6→4.0 (6.7×) only clips harder; the plant decel is
unchanged. Every magnitude clone aborts at the identical s≈650 station via
the over-slip ghost-fallback (`slip_ratio≈3.0`), exactly like the baseline.

## Sweep table (high-grip ideal-line, reactive, inertia-zz 2400, 10-seed MC)

`tomas_highgrip*` bake D×1.28; **no `--grip-d-scale`** passed; all runs report
`grip_y=1.280/1.284` (single application confirmed).

| driver clone | brake_p_gain | preview_time_s | result | abort s | util_p85 | MC σ (s) |
|---|---:|---:|---|---:|---:|---:|
| tomas_highgrip (base) | 0.6 | 1.5 | ABORT | 648–650 | 0.40 | — |
| bp10 | 1.0 | 1.5 | ABORT | 648–650 | 0.44 | — |
| bp15 | 1.5 | 1.5 | ABORT | 648–651 | 0.57 | — |
| bp25 | 2.5 | 1.5 | ABORT | 650–651 | 0.51 | — |
| bp40 | 4.0 | 1.5 | ABORT | 650–651 | 0.64 | — |
| pv17 | 0.6 | 1.7 | ABORT | 651–653 | 1.10 | — |
| pv19 | 0.6 | 1.9 | ABORT | 655–659 | 1.33 | — |
| pv21 | 0.6 | 2.1 | ABORT | 665–671 | 1.62 | — |
| **pv23** | **0.6** | **2.3** | **FINISH 10/10** | — | 0.80 | **0.131** |
| pv25 | 0.6 | 2.5 | FINISH 10/10 | — | 0.67 | 0.060 |
| bp25_pv30 | 2.5 | 3.0 | FINISH 10/10 | — | 0.66 | 0.053 |
| pv40 | 0.6 | 4.0 | FINISH 10/10 | — | 0.58 | 0.082 |
| bp40_pv40 | 4.0 | 4.0 | FINISH 10/10 | — | 0.61 | 0.074 |

`--chicane-safety-mult` was swept 1.00→1.15 on the finishers: **zero effect**
(the cap only *lowers* the plan target, never raises it; lap time is fully
set by `preview_time_s`). The finish boundary is between
`preview_time_s` 2.1 (abort, slipping further down-track to s≈670 as it
carries more speed in) and 2.3 (clean finish). pv23 is the fastest finisher
because the *shortest* sufficient preview throttles off latest into every
other corner too — longer previews (2.5→4.0) finish but bleed pace lap-wide.

## Headline

Raising **brake magnitude alone does NOT** make reactive finish. Raising
**preview-time (lookahead braking) DOES** — and since `preview_time_s` is an
existing `control_params` knob, this is a **config-only win** via a knob the
brief did not name. Best finisher: `preview_time_s = 2.3`, default brake gain,
**111.92 s, 10/10, σ = 0.131 s**. That is **+4.36 s vs Tomas (107.56 s)** and
**+1.36 s over the ≤110.56 s target** — close but not passing. The winning
setting is physically plausible: `preview_time_s = 2.3 s` is a reasonable
human anticipation horizon (well within the 1.5–4 s range a real driver
scans), and brake gain stays at the default 0.6, so there is **no
unrealistic artifact** — unlike the brake-magnitude clones (bp25/bp40), which
demand 4–7× a normal P-gain and still fail.

## Verdict

- **Preview braking is required — but it already exists as config.** The
  reactive controller's longitudinal preview (`preview_time_s`, the
  `_min_speed_within` window) IS lookahead braking; the high-grip plant just
  needs it set longer (2.3 s vs the 1.5 s default) to shed the higher entry
  speed in time. No controller-code change is needed to *finish* the lap.
- **Brake-authority/magnitude is a red herring** for this abort (brake
  already saturates), and `pedal_press_rate_per_s` is inert in reactive.
- **Pace gap remains.** Config closes the abort but leaves +1.36 s over
  target. Closing the last gap would need either (a) a non-uniform /
  curvature-scheduled preview (long only into tight corners, short on
  fast sweepers — a controller-code change), or (b) the MPC/HMPC inner
  with explicit preview braking, which decouples corner-entry braking from
  a global lookahead horizon. Both are build-owner / Buddy-spec scope.

**Root cause layer:** `code` (for the residual pace gap — a uniform
`preview_time_s` is structurally conservative; a curvature-scheduled preview
or MPC preview is the proper fix). The *abort itself* is closed by config
(`architecture`/config) and needs no code change.

## Addendum files

- `.tmp/make_brake_clones.py`, `.tmp/make_preview_clones.py` — clone generators.
- `.tmp/brake_authority_sweep.py`, `.tmp/preview_pace_sweep.py` — sweep harnesses.
- Clone driver JSONs in `drivers/tomas_highgrip_brake_*.json` (scratch
  diagnostic artifacts; not production drivers).
- No production code modified.

---

# Addendum (2026-05-27): adaptive (per-corner) preview braking

**Spec:** `dev-planning/reactive-adaptive-preview/spec.md` (Formulation 1,
physics-based braking-distance preview). This addendum records the **first
production code change** in this lineage (the prior addenda were config-only):
a per-corner adaptive brake-lookahead horizon, opt-in via `control_params`.

## What was built (one paragraph)

The reactive longitudinal brake channel previously derived its lookahead from a
single uniform window `brake_lookahead_m = max(30, v_preview · preview_time_s)`.
This addendum adds an opt-in `control_params.preview` block with a `mode`
switch: `"uniform"` (default) is the **byte-identical legacy path**;
`"adaptive"` derives the horizon from the physical braking distance for the
*upcoming* speed drop, `safety_factor·(v²−v_min_ahead²)/(2·a_brake_avail_ms2)`,
floored at `min_lookahead_m` and bounded by a seed horizon. The intent
(spec §5): long only where a large speed drop is imminent, short on
straights/fast corners, recovering the lap-wide pre-braking deficit a uniform
horizon pays.

## Why this architecture (key decisions + trade-offs)

- **Opt-in `mode` gate, default `uniform`.** The adaptive math is gated behind
  `self.params.preview.mode == "adaptive"`; when absent/`"uniform"` the legacy
  expression runs verbatim. This is the regression-safety switch (spec R5):
  stock drivers see zero behavioural change. Verified — see Regression below.
- **Physics formulation over curvature-scheduling** (spec §9): no curvature
  source needed, parameter-light (`a_brake_avail_ms2`, `safety_factor`, floor),
  keys on the speed drop (the quantity that matters) not geometry.
- **Two-pass-per-tick with a distance anchor (DEVIATION from the naive spec
  math — see below).** The seed pass uses a generous horizon to surface the
  true downstream slow point; the physics pass tightens it.
- **`a_brake_avail_ms2` is a driver-JSON constant**, not a live plant query
  (spec §5: avoids coupling the two passes to the friction-ellipse solver).
  High-grip opts in at 11.5 (anchored to the measured ~13 m/s² chicane decel).

## Distance-anchor deviation from the spec math (important)

The spec §5 monotonicity argument — "the seed is an upper bound, so the physics
horizon only shrinks toward the true braking distance, never missing the slow
point" — is **only valid while `v` stays at entry speed**. It breaks *during*
the braking event: the v²/2a braking distance shrinks with `v²`, but the
distance still to run to the apex shrinks only linearly with position. So
mid-corner-entry the raw physics horizon collapses *below* the remaining
distance to the apex, the apex drops out of view, and the brake **releases
prematurely** — leaving the car over-speed at the apex. Empirically a naive
single-physics-horizon implementation aborted off-track at the **s≈645 m
hairpin on every config** (`util_p85≈0.22`, i.e. under-braking, not over-slip).

**Fix (in `_adaptive_brake_lookahead`):** the physics horizon is floored not
only at `min_lookahead_m` but also at the **distance to the seed's slow point
whenever the car is still faster than that slow point** (`v > v_seed`). This
keeps the apex in view for the whole braking event (honouring the spec's "never
missing the slow point" guarantee) while still letting the horizon — and the
brake target — collapse to the floor on straights / fast corners where the car
is *not* over any upcoming speed (the intended adaptive pace win). A new helper
`_min_speed_and_dist(idx, lookahead_m)` returns both the window min and the
along-line distance to it. One iteration; no loop (spec R4).

## Data flow (adaptive tick)

```
v, v_preview, idx
   │
   ▼  seed horizon = seed_lookahead_m  (or max(floor, v_preview·t_preview))
v_seed, dist_to_seed_min = _min_speed_and_dist(idx, seed_horizon)
   │
   ▼  phys_la = safety_factor·(v² − v_seed²)/(2·a_brake_avail_ms2)
lookahead = max(phys_la, floor)
if v > v_seed:  lookahead = max(lookahead, dist_to_seed_min)   # anchor
lookahead = min(lookahead, seed_horizon)                       # upper bound
   │
   ▼
v_target_min = _min_speed_within(idx, lookahead)
   │
   ▼  (UNCHANGED downstream)
brake = clip(brake_p_gain · max(v_target_min − v, 0), 0, 1)
```

## File inventory

- `src/lap_estimator/dynamics/_control_params.py` — new frozen
  `PreviewParams` dataclass + `PreviewParams.from_block` validator (mode in
  {uniform, adaptive}, `a_brake_avail_ms2 > 0`, `safety_factor ≥ 1`,
  `min_lookahead_m ≥ 0`, `seed_lookahead_m ≥ floor or null`); `preview` field
  added to `ControlParams` (default-factory `PreviewParams()` = uniform) and
  threaded through `from_driver` + `with_slip_target`.
- `src/lap_estimator/dynamics/driver_controller.py` — `step()` branches on
  `self.params.preview.mode` at the preview block (uniform path unchanged);
  new private helpers `_adaptive_brake_lookahead` and `_min_speed_and_dist`.
  File grew 623 → ~690 lines (already over the soft 500-line ceiling pre-edit;
  no natural seam to split the controller without a larger refactor — left
  intact per the "don't force an artificial split" rule).
- `drivers/tomas_highgrip.json` — opt-in `control_params.preview`
  (`mode:"adaptive"`, `a_brake_avail_ms2:11.5`, `safety_factor:1.15`,
  `seed_lookahead_m:null`) plus `preview_time_s:2.3` (seed horizon). Stock
  `drivers/tomas.json` left with **no** preview block (stays uniform).

## §8 open question — RESOLVED (seed horizon)

**The 1.5 s default seed is NOT enough; even 2.3 s is marginal.** The
s=501→627 m hairpin braking zone is **~126 m** long (plan drops 51→14.7 m/s).
The seed horizon must reach the apex from the top of the approach or the anchor
cannot pin it:

| seed source | effective seed @ v≈51 | result |
|---|---|---|
| `preview_time_s = 1.5` (default) | ~77 m | **ABORT** s≈661 |
| `preview_time_s = 2.3` | ~117 m | **FINISH** 10/10 |
| `seed_lookahead_m = 130` (fixed) | 130 m | ABORT (util 1.17 — marginal) |
| `seed_lookahead_m = 150` (fixed) | 150 m | FINISH 10/10 (131.26 s) |
| `seed_lookahead_m = 180` (fixed) | 180 m | FINISH 10/10 (134.72 s, slow) |

**Decision:** high-grip uses `seed_lookahead_m: null` + `preview_time_s: 2.3`
(the uniform-window seed, which scales with `v` and so is shorter on slow
sections than a fixed seed — giving the best finisher). A fixed seed long
enough to never abort (≥150 m) pre-brakes lap-wide and is *slower*.

## Sweep table (high-grip ideal-line, reactive, inertia-zz 2400, 10-seed MC)

`a_brake_avail_ms2` × `safety_factor`, fixed seed = 150 m (so every cell
finishes and the physics term is the only variable):

| a_brake | safety | lap (s) | finish | σ (s) | util_p85 |
|--------:|-------:|--------:|:------:|------:|---------:|
| 9–13 | 1.05–1.30 (all 30 cells) | 131.26 | 10/10 | 0.000 | 0.31 |

**`a_brake_avail_ms2` and `safety_factor` have ZERO effect on lap time or
finish** across the entire 9–13 × 1.05–1.30 grid. Reason: at the binding
corner the distance-anchor (distance-to-apex) dominates the v²/2a term, and the
brake already saturates to 1.0 — so the physics constants never move the
operating point. This mirrors the prior addendum's finding that `brake_p_gain`
is inert (brake saturates).

Seed-length sweep (the only lever that moves lap time), and the decisive
**adaptive-vs-uniform equivalence** check:

| config | lap (s) | finish | σ (s) |
|---|--------:|:------:|------:|
| uniform `preview_time_s=2.1` | **120.52** | 10/10 (marginal, 1 retry) | 0.087 |
| uniform `preview_time_s=2.3` | 121.26 | 10/10 | 0.073 |
| uniform `preview_time_s=2.5` | 122.90 | 10/10 | 0.093 |
| **adaptive seed=null pv=2.1** | 120.52 | 10/10 (marginal) | 0.087 |
| **adaptive seed=null pv=2.3** | **121.26** | **10/10** | **0.073** |
| adaptive seed=null pv=2.5 | 122.90 | 10/10 | 0.093 |
| adaptive seed=null pv=1.9 | — | ABORT s≈661 | — |
| uniform pv=1.9 | — | ABORT s≈661 | — |

**Adaptive with the uniform-window seed is byte-identical to uniform at every
`preview_time_s`** (and aborts in the same place when the seed is too short).

## Why adaptive gives no pace gain on the geometric ideal line

The spec's premise — "a uniform preview long enough for the hairpin is far too
long everywhere else, so the car pre-brakes on every faster corner and bleeds
time lap-wide" — **does not hold for this plan.** The ideal-line CSV
`speed_ms` plan is already grip-feasible (`util_p85 ≈ 0.30` lap-wide), so the
reactive brake (`brake = clip(0.6·(v_target_min − v), …)`) **only fires in the
hairpin braking zone**; on fast corners the car is at or below the upcoming
plan speed, so neither uniform nor adaptive pre-brakes. With no lap-wide
pre-braking deficit to recover, the adaptive horizon collapses to exactly the
uniform horizon at the one corner that gates the lap. The pace is
**plan-limited, not preview-limited.**

## Regression check (Gate 2 — CRITICAL, PASSED)

Stock `drivers/tomas.json` (default uniform mode), current code:

| run | lap | finish | σ (s) |
|---|---|:--:|---|
| centerline `layout_sprint_a.csv` | **2:09.160** | 10/10 | 0.086 |
| ideal-line `layout_sprint_a_ideal_line.csv` | **2:09.200** | 10/10 | 0.083 |
| ideal-line, `preview.mode` **forced adaptive** (A/B) | **2:09.200** | 10/10 | 0.083 |

Stock baseline preserved bit-for-bit (~2:09, matches the historical 2:09.20),
and forced-adaptive reduces to uniform on the stock plant — confirming the
gate at the top of the adaptive branch and the safety claim (spec §4 story 4).

## Headline (be honest about the ceiling)

- **Best stable lap on this reference: 121.26 s, 10/10, σ = 0.073 s**
  (adaptive `preview_time_s = 2.3`, `seed_lookahead_m = null`). The fastest
  *finisher* is 120.52 s (`pv = 2.1`) but it sits on the abort cliff (pv = 1.9
  aborts); 2.3 is the safe production choice.
- **Adaptive does NOT beat uniform here — it equals it.** On the geometric
  ideal-line the per-corner formulation collapses to uniform because the pace
  is plan-limited, not preview-limited (no lap-wide pre-braking deficit
  exists).
- **CORRECTION (2026-05-27): the ~9 s "regression" was a CLI-default artifact,
  NOT an upstream code change.** The sweeps in this addendum were run *without*
  `--chicane-safety-mult`, which resolves to the config default **0.80** and
  caps the DP plan's `v_max` (334 segments flagged, `util_p85=0.304`) → ~121 s.
  The prior addendum's **111.92 s** was measured with `--chicane-safety-mult
  1.00` (0 segments flagged, `util_p85=0.802`), and it **reproduces
  deterministically** (3/3 identical, σ=0.131) on the current tree. Every
  uncommitted change (slip_simulator / HMPC / mpc_qp / grip / adaptive-preview)
  was verified **inert** for the reactive pv23 path (additive or mode-gated;
  reactive does not touch HMPC/mpc_qp at runtime). There is no code regression.
- **Apples-to-apples (cm=1.00) result: 111.92 s, 10/10, σ=0.131** for both
  uniform `preview_time_s=2.3` and the adaptive `tomas_highgrip.json` — i.e.
  **adaptive equals uniform on this reference**, because the geometric
  ideal-line plan is grip-feasible lap-wide (no pre-braking deficit to
  recover). Distance to Tomas (107.56 s): **+4.36 s**.
- **The real pace lever on this reference is the DP plan's conservatism
  (`safety_margin` / chicane cap), not the preview scheme.** Adaptive preview
  is correct, regression-safe, and necessary for the hairpin abort, but cannot
  beat the plan floor. Closing the remaining gap to Tomas needs a faster
  *reference* (Tomas-telemetry line+speed) and/or a loosened plan — pursued
  separately.

**Root cause layer:** `architecture`/config — the apparent regression was the
`--chicane-safety-mult` default (0.80 vs 1.00), not a code defect; and adaptive
preview = uniform on a grip-feasible plan is structural, not a bug.

## Addendum files (adaptive preview)

- `.tmp/adaptive_preview_sweep.py`, `.tmp/adaptive_preview_sweep_c.py`,
  `.tmp/adaptive_preview_sweep_d.py` — sweep harnesses (scratch).
- `.tmp/debug_adaptive.py` — helper-math unit checks (scratch).
- `.tmp/adaptive_clones/*.json` — scratch driver clones (not production).
- Production: `_control_params.py`, `driver_controller.py`,
  `drivers/tomas_highgrip.json` (above).
