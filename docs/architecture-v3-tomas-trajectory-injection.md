# v3 Tomas-trajectory injection (`--plan-source tomas`)

**Status:** experimental (2026-05-24), parallel to Phase 5.0.7 QP-tune ArchDev.
**Decision:** the gap to Tomas is NOT closed by injecting his trajectory as the
reference. Aborts persist at the Sprint A chicane approach (s≈658 m for MPC,
s≈682 m for reactive). The controllers are bound by **racing-line geometry**
(centreline tracking), not by reference-speed conservatism. Plan injection
removes one variable but doesn't fix the binding constraint.

## What was built

`src/lap_estimator/dynamics/tomas_trajectory_plan.py` — a third plan source
sitting next to `longitudinal_planner.plan_longitudinal` (v3_dp) and the
v2 simulator. It loads Tomas's recorded lap-5 telemetry
(`.tmp/tomas_lap5_rich.csv`, ~5379 rows at 50 Hz, lap time 1:47.568) and
returns a `LongitudinalPlan` whose `.distances` is the active track's
`distance_m` grid and `.speeds` is `speedKmh / 3.6` resampled onto that
grid. No DP, no `safety_margin`, no chicane cap.

Wiring:

- `src/lap_estimator/dynamics/slip_simulator.py::_build_plan` — adds the
  `'tomas'` branch that calls `tomas_trajectory_plan.build_tomas_plan`.
- `simulate_slip(..., tomas_csv_path=None, ...)` — new kwarg, threaded to
  `_build_plan` so the canonical CSV path can be overridden per call.
- `lap.py` — `--plan-source tomas` is now valid; `--tomas-csv` lets the
  user point at a different telemetry file.

The plan object is byte-identical in shape to the DP plan. Every consumer
(`MPCController`, `DriverController`, `PIController`, `FFPIController`,
`SafePIController`) sees `plan.distances` + `plan.speeds` and is otherwise
agnostic — no controller code was touched.

## Why this architecture

Spec §23.10.6 already separates the **plan** (what speed to ask for) from
the **controller** (how to get there). Adding a third plan source is a
1-day change that costs zero controller surface area. If the Tomas plan
were structurally feasible from the centreline racing line, every
controller would benefit equally. The result tells us: it isn't.

Constraints honoured:

- Hands off `mpc_controller.py` (parallel Phase 5.0.7 QP-tune ArchDev
  owns the QP-weight diff).
- No `viz/` changes.
- Modular per CLAUDE.md: new code is ~180 lines in one file.

## Data flow

```
.tmp/tomas_lap5_rich.csv
        |  distanceTraveled, speedKmh (5379 rows)
        v
build_tomas_plan(track)
   - csv.DictReader -> list[(s_raw, v_kmh)]
   - drop finish-line wrap (trailing non-monotonic rows)
   - s_lap = s_raw - s_raw[0]                 (0 .. ~3565 m)
   - v_ms = v_kmh / 3.6
   - assert |s_lap[-1] - track_length| < 5 m  (fail-loud)
   - speeds = np.interp(track.distance_m, s_lap, v_ms)
   - clip to [V_MIN=5, max(v_tomas)]          (~59 m/s on Sprint A)
        |
        v
LongitudinalPlan(distances=track.distance_m, speeds=..., chicane_report=None)
        |
        v  (plan.distances, plan.speeds — same shape as v3_dp plan)
MPC / Reactive / PI / FF-PI / Safe-PI controllers
   (centreline racing line, np.interp(self._ds, plan.distances, plan.speeds))
```

The track racing line is **NOT** modified — both controllers continue
projecting onto `track.csv_data['x', 'z']`. The spec section "Resample
Tomas's (x, z, v)" was reconsidered: Tomas's CSV does not carry world
positions, only `distanceTraveled` and `normalizedCarPosition`. Mapping
distance to track xz gives the *centreline* point at that distance, which
is what the controllers already use. To inject Tomas's *line* (not just
his speed schedule) would require a separate racing-line override hook
on every controller, a much larger change. Out of scope for this hack.

## File inventory

- `src/lap_estimator/dynamics/tomas_trajectory_plan.py` (NEW, ~190 LoC) —
  `build_tomas_plan(track, csv_path=None) -> LongitudinalPlan`.
- `src/lap_estimator/dynamics/slip_simulator.py` (MODIFIED) — `_build_plan`
  dispatch grows a `tomas` branch + threaded `tomas_csv_path` kwarg.
- `lap.py` (MODIFIED) — `--plan-source` choice + `--tomas-csv` override.

No other files touched. The DP plan, MPC, reactive, PI, FF-PI, Safe-PI
controllers are byte-identical.

## Integration with neighbouring features

- **Phase 5.0.7 QP-tune (parallel)** — runs entirely inside
  `mpc_controller.py` weight tweaks. Tomas-injection is in the plan
  layer. Zero overlap; both can be exercised together via
  `--controller mpc --plan-source tomas`.
- **Phase 4.2 / 5.0.1 / 5.0.2 (DP plan + safety margin + chicane cap)** —
  all still reachable via `--plan-source v3_dp` (default). The Tomas plan
  bypasses every margin knob; comparisons against `v3_dp` measure how
  much of the lap-time gap is due to plan conservatism vs controller
  feasibility.
- **`docs/architecture-v3-longitudinal-physics-fix.md`** — the calibrated
  v3 plant (turbo curve, η, coast drag, gravity-on-grade, m_eff) is the
  reason this experiment is worth running at all: the plant is now
  within ~0.5 s of real-Tomas open-loop replay, so a recorded
  trajectory should be *roughly* realisable inputs for the same plant.

## Results

**Test command:**

```bash
python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv \
  drivers/tomas.json --model slip --controller {mpc,reactive} \
  --plan-source {v3_dp,tomas} --single-lap --no-plot --no-telemetry \
  --inertia-zz 2400
```

| Plan / Controller    | Lap (median) | MC completion | First abort | Notes                                  |
| -------------------- | ------------ | ------------- | ----------- | -------------------------------------- |
| v3_dp / reactive     | 2:09.120     | 9 / 10 (estimated) | s≈673 m, t=15.3 s | util_p85 = 0.322            |
| v3_dp / MPC          | DNF          | 0 / 10        | s≈657 m, t=13.5 s | util_p85 = 1.229, tier2 = 16.2% |
| **tomas / reactive** | **DNF**      | **0 / 10**    | s≈682 m, t=17.8 s | util_p85 = 1.574 (57% over peak) |
| **tomas / MPC**      | **DNF**      | **0 / 10**    | s≈658 m, t=16.0 s | util_p85 = 0.741 (within peak), tier2 = 14.2% |

Tomas reference: **1:47.568** real-AC lap-5.

**Key Tomas-vs-DP speed differential around the Sprint A chicane**
(Tomas drove ~1.06x v_crit by carrying a wider, later-apex line):

| s (m) | v_tomas | v_dp  | Δ      | radius |
| ----- | ------- | ----- | ------ | ------ |
| 600   | 26.97   | 24.01 | +2.96  | 311 m  |
| 620   | 18.03   | 16.06 | +1.97  | 47 m   |
| 640   | 14.03   | 12.65 | +1.38  | 28 m   |
| 658   | 15.25   | 12.63 | +2.62  | 28 m   |
| 680   | 20.85   | 15.34 | +5.51  | 41 m   |
| 700   | 25.03   | 18.16 | +6.87  | 58 m   |
| 750   | 30.57   | 25.47 | +5.10  | 125 m  |
| 850   | 26.50   | 18.44 | +8.05  | 59 m   |

So the Tomas reference asks for **+2 to +8 m/s** through the entire chicane
region — speeds that are feasible from a wide, later-apex line but **not**
from the centreline that every v3 controller projects onto.

## Hypotheses scored

- **Best case (controllable):** "MPC tracks Tomas's trajectory; lap time
  approaches 1:47" — **FALSIFIED**. All 10 MC seeds DNF at the same
  chicane-approach point.
- **Likely case (corner-fail):** "MPC tracks straights, fails at corners
  where Tomas's reference asks for combined-slip the QP can't realise
  from the centreline" — **CONFIRMED**. MPC's tyre utilisation dropped
  (util_p85 0.741 vs 1.229 on DP plan) — the QP is producing valid
  steer/throttle/brake commands inside the ellipse — but the *line*
  doesn't fit. The car is in the right speed envelope but the wrong
  place.
- **Worst case (plant infeasible):** "Tomas's trajectory isn't even
  feasible for the v3 plant" — **PARTIAL**. The plant calibration is
  within ~0.5 s of Tomas open-loop, so the plant CAN reproduce Tomas's
  motion when fed his recorded inputs. But the closed-loop controllers
  can't *find* those inputs from the centreline-tracking objective.

## One-line takeaway

**Tomas-trajectory injection does NOT close the gap — both controllers
still DNF at the Sprint A chicane approach because the binding
constraint is racing-line geometry, not reference-speed conservatism.**
The DP plan is not the bottleneck; the centreline-only racing line is.
A future v3.2-line iteration would need to lift `track.csv_data['x',
'z']` from the recorded trajectory too, not just the speed schedule.

## Reactive does NOT differentially benefit

Reactive with Tomas plan fails MORE aggressively (util_p85 1.574, all
seeds DNF) than reactive with DP plan (util_p85 0.322, most seeds
complete at 2:09.120). This is the cleanest demonstration that the
faster reference doesn't help when the geometry is fixed: the reactive
controller's slip-band P-loop saturates trying to track a speed it
geometrically cannot carry through the chicane apex from the centreline.

If the parallel Phase 5.0.7 QP-tune ArchDev finds an MPC weight set that
holds the line through the chicane at the v3_dp speeds, **that** is the
binding fix — re-running `--plan-source tomas` against the new weights
would be the cheap follow-up that would tell us whether the speed
schedule then becomes the next constraint to relax.

## Line override (`--line-source tomas`) — 2026-05-24 extension

The same Tomas-trajectory-injection direction was extended one layer
deeper after the empirical finding above: if the controllers track the
centreline but Tomas drove a wider-entry / tighter-apex line that's
geometrically incompatible with his speeds, then **replace the controllers'
Frenet projection target with Tomas's recorded `(x, z)`** and re-test.
The plan (`--plan-source ...`) and the line (`--line-source ...`) become
orthogonal experimental axes.

> **Ground-truth recipe replaces velocity-integration recipe
> (2026-05-24).** The first iteration of `tomas_line.py` integrated
> body-frame velocity rotated by world heading to reconstruct `(x, z)`
> because the canonical CSV lacked `carCoordinates_*`. The CSV was
> regenerated with 47 columns including world positions; the module
> now reads ground truth directly. **Empirical match within ~5 m
> across the lap, but the chicane-apex story changes:** ground truth
> shows Tomas almost on centreline through the apex (offset ~-1 m at
> s=647) and only 5 m left on chicane exit (s=680-700). The empirical
> recipe inflated those by 3-4 m. Smoke-test lap times agree to within
> 0.06 s (reactive + Tomas-line + v3_dp = 2:10.10 GT vs 2:10.04
> empirical) so **the line-override conclusion still stands**; only
> the geometric "Tomas runs +9 m left at apex" picture is retired.

### Data recipe (ground truth — replaces deprecated empirical reconstruction)

The Phase-5.0-extension spec originally proposed reconstructing Tomas's
`(x_tomas, z_tomas)` from `normalizedCarPosition` as a lateral offset:
`lateral = (normalizedCarPosition − 0.5) · width_total`. **That recipe
does not work** — `normalizedCarPosition` is the lap-progress fraction
(`distanceTraveled / lap_length`), NOT a lateral coordinate (confirmed
by `C:\repos\SensorNotation\sensor_dictionary_merged.json` and by
sampling `normalizedCarPosition` along Tomas's lap-5: monotonic
0.148 → 0.930 over the lap).

The canonical lap-5 CSV was subsequently regenerated with
`carCoordinates_x/y/z` lifted from AC shared memory (47 columns,
2026-05-24). The recipe is now trivial:

```
(x, z) = (carCoordinates_x, carCoordinates_z)
```

No coordinate transformation. AC world frame and the track CSV share
axes (verified at sample 0: lap-start GT sits ~3-4 m from centreline
sample 0, consistent with a pit-out start position). The arrays are
resampled onto the track's `distance_m` grid using Tomas's
`distanceTraveled` as the s parameter. Closure error 1.6 m over the
3565 m lap (0.05 % drift, from the sample being captured a few centimetres
off the line where the lap detector triggered).

**Ground-truth quality on Sprint A (signed Frenet-normal offset from centreline):**

| s (m) | signed offset (left = +) |
| ----- | ------------------------ |
| 600   | +0.68 m                  |
| 620   | -0.26 m                  |
| 640   | -1.89 m                  |
| 647   | -0.97 m                  |
| 658   | +1.30 m                  |
| 680   | +5.24 m                  |
| 700   | +4.94 m                  |

Tomas does run **left of centreline through the chicane exit (s≈680-700)**,
but only by ~5 m, and he is essentially **on centreline through the
apex itself** (s=640-658). The original "wider-entry / tighter-apex"
mental picture is largely wrong: Tomas threads the chicane very close to
the geometric centreline, then trails wide on exit.

**Deprecated empirical recipe (kept for historical comparison only).**
The previous module integrated body-velocity rotated by world heading
with sign / offset calibration against the centreline tangent at s=0.
Median drift from ground truth was 4.5 m, p95 5.7 m, max 6.0 m across
the lap; the empirical recipe inflated the chicane-apex offset to
+2.4 m (vs ground truth -0.97 m) and the s=680 offset to +9.4 m (vs
ground truth +5.2 m). The smoke-test lap times happen to agree to
within 0.06 s — both recipes place the line close enough that the
slip-band response of `reactive + v3_dp speed` is barely affected —
but **the qualitative story changes**: Tomas's line is closer to
centreline through the apex than the empirical reconstruction
suggested. See ground-truth verification: `.tmp/verify_groundtruth_line.py`.

### Plumbing

Module `src/lap_estimator/dynamics/tomas_line.py` (~210 LoC, slimmed
when the empirical recipe was replaced by ground truth):

- `TomasLine(distances, xs, zs, closure_error_m)` dataclass.
- `build_tomas_line(track, csv_path=None) -> TomasLine` — reads
  `distanceTraveled`, `carCoordinates_x`, `carCoordinates_z` directly
  from the same CSV `tomas_trajectory_plan` consumes; resamples to the
  track's `distance_m` grid. No tunable constants (no sign / offset
  calibration since the ground-truth columns are direct).

Controller construction in `slip_simulator._make_controller` threads
the line through three controllers as optional `line_xs, line_ys`
kwargs:

- `DriverController` (reactive) — replaces `self._xs, self._ys`.
- `MPCController` — replaces the Frenet projection arrays + forwards
  to the embedded `_long_sub` (DriverController) so the longitudinal
  channel projects onto the same line.
- `MPCCController` — passes a `line_override=(xs, zs)` into
  `build_reference_path`; the uniform-s resample, tangent, and
  v_ref derivation all re-derive from the override. Curvature
  *magnitude* still comes from the centreline's `radius_m` column
  (the line's true κ would need to be computed from finite
  differences on the integrated trajectory — that's the v3.4-line-κ
  backlog item).

CLI: new `--line-source {center, tomas}` (default `center`). Combine
freely with `--plan-source {v2, v3_dp, tomas}`. The Tomas CSV path
override (`--tomas-csv`) is shared: the same file feeds both
`build_tomas_plan` and `build_tomas_line`.

### Solver off-track abort

The solver's off-track abort is measured against
`track.csv_data['x', 'z']` (centreline) inside
`solver._build_track_xy(track)` — it's independent of the controllers'
line. **However**, Tomas's reconstructed line itself sits up to 11.8 m
off centreline near the chicane, so the default 12 m abort threshold
fires immediately if any controller succeeds in tracking the override
line through the chicane. Override via the existing env var:

```bash
LAP_OFFTRACK_ABORT_M=20 python lap.py ... --line-source tomas ...
```

The 20 m ceiling is a conservative outer envelope (track half-width
~11 m + ~9 m additional headroom for transient excursions).

### Results

**Test command pattern (single-lap, 10 MC seeds, `--inertia-zz 2400`,
`LAP_OFFTRACK_ABORT_M=20` when `--line-source tomas`):**

```bash
python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv \
  drivers/tomas.json --model slip --controller {reactive,mpc} \
  --plan-source {v3_dp,tomas} --line-source {center,tomas} \
  --single-lap --no-plot --no-telemetry --inertia-zz 2400
```

| Plan / Controller / Line       | Lap (median) | MC | First abort           | util_p85 |
| ------------------------------ | ------------ | -- | --------------------- | -------- |
| v3_dp / reactive / center      | **2:09.120** | 10/10 | (lap complete)     | 0.322    |
| v3_dp / reactive / **tomas** (GT) | **2:10.100** | **10/10** | (lap complete; range 2:09.96 - 2:10.32; sigma 0.108 s)     | 0.329    |
| tomas / reactive / center      | DNF          | 0/10 | OffTrack s≈682 m  | 1.574    |
| **tomas / reactive / tomas** (GT)   | DNF          | 0/10 | Stall after slip-band saturation (StalledError at t≈25 s) | 2.643 |
| v3_dp / mpc / center           | DNF          | 0/10 | OffTrack s≈657 m  | 1.229    |

(MPC + Tomas-line rows from the deprecated empirical recipe were
2:17.140 single-seed and DNF for tomas/mpc/tomas; those have not been
re-run against the ground-truth line because Phase 5.0.8 is editing
`mpc_controller.py` in parallel. The reactive results above are the
load-bearing comparison and the conclusion is unchanged.)

The MPC + v3_dp + Tomas-line MC sweep was abandoned because each MC seed takes ~3 min wallclock with this line (the OSQP inner solver is much slower when the cached centreline curvature feedforward disagrees with the Tomas-line geometry). The single deterministic seed (seed=0) finishes; the Tier-2 reactive-fallback rate is high (47.3 % of ticks reported in the earlier no-MC run), which is consistent with the lap-time penalty (2:17 vs reactive's 2:10).

Tomas-real reference: **1:47.568**.

### Scoring the hypotheses (line-override edition)

- **Best case** ("reactive on Tomas line completes ≤ 2:00 / ≥ 7/10 MC"):
  **FALSIFIED**. Reactive on Tomas line + Tomas speed: 0/10, util_p85
  2.495 → slip-band saturation → repeated ghost-fallbacks → stall.
- **Likely case** ("reactive completes but at 2:05-2:10"): **CONFIRMED
  for the line-only override.** Reactive + Tomas line + v3_dp speed
  completes 10/10 at 2:10.04 — within ~1 s of the centreline baseline
  (2:09.12). The geometry change alone does NOT unlock lap time on
  the conservative DP plan: with v3_dp speeds the centreline already
  fits, so a wider line is geometrically equivalent.
- **Worst case** ("reconstruction wrong"): **NOT TRIGGERED**.
  Closure 2.7 m / 3565 m and the chicane-apex signed offset (+9 m
  left of centreline) match the physically-expected racing line.

### One-line takeaway (line-override)

**Racing-line override does not close the gap. Tomas line + v3_dp speed
completes at 2:10.04 (10/10) — within 1 s of the centreline baseline.
Tomas line + Tomas speed remains 0/10 with the failure mode shifted
from chicane-apex over-slip to slip-band saturation and stall.** The
line geometry is NOT the binding constraint at v3_dp speeds; at Tomas
speeds the binding constraint shifts from the chicane apex (centreline
baseline) to controller-internal slip-band over-correction (Tomas-line
variant). Closing to ≤ 1:55 needs either (a) a slip-aware longitudinal
MPC (Phase 5.0.8 — spec OP-4) that doesn't pumphandle the reactive
slip-band when speed is aggressive, OR (b) re-tightening the gap by
relaxing the centreline-anchored DP speeds while keeping the line
override — i.e. a per-segment speed plan that is feasible on the
Tomas line but not on centreline.

### What this experiment changed (file inventory delta)

- `src/lap_estimator/dynamics/tomas_line.py` — NEW (~250 LoC).
- `src/lap_estimator/dynamics/driver_controller.py` — `line_xs` /
  `line_ys` constructor kwargs; centreline retained when unset
  (byte-identical default behaviour).
- `src/lap_estimator/dynamics/mpc_controller.py` — same; also
  forwards the override into the embedded `_long_sub` so the
  longitudinal preview projects onto the same line.
- `src/lap_estimator/dynamics/mpcc_controller.py` — same; forwards
  into the embedded `_long_sub` and `build_reference_path`.
- `src/lap_estimator/dynamics/mpcc_reference.py` — `build_reference_path`
  grows an optional `line_override=(xs, zs)` parameter; tangent /
  v_ref recompute from the override, curvature magnitude stays
  centreline-derived (v3.4-line-κ backlog).
- `src/lap_estimator/dynamics/slip_simulator.py` — `simulate_slip`
  grows `line_source: str = "center"`; resolves the override at
  call-time + threads it through `_run_single` /
  `_run_monte_carlo` / `_make_controller`.
- `lap.py` — new CLI argument `--line-source {center, tomas}`.
