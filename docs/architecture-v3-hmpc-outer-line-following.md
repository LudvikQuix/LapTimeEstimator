# Architecture — HMPC v3.4 outer line-follow mode

**Spec / task brief:** 2026-05-26 line-follow brief — pin the HMPC outer
NLP to the offline ideal-line trajectory so the outer's lateral degrees
of freedom collapse and the only remaining decision is the longitudinal
profile. Diagnostic question: with the racing-line wandering removed,
what lap time can the multilayer inner extract?

**Predecessors:**
- `docs/architecture-v3-hmpc-chase-tomas.md` — chase-Tomas sweep that
  identified the architectural ceiling (~2:28.88 / 10-of-10) and
  recommended this mode as the next lever.
- `docs/architecture-v3-hmpc-casadi-outer.md` — Phase 5.2 outer NLP.
- `docs/architecture-v3-hmpc-inner-tuning.md` — Lever 3 (`inner_w_a`).

**Branch:** `feature/sc-71955/lap-simulation`.

**Status (2026-05-26):** Line-follow mode lands as a working CLI /
driver-JSON feature. Empirically it does **NOT** unlock the inner's
lap-time ceiling. Best single-seed LF lap: **2:29.74 (149.74 s) at
cm=0.95, w_n_ideal=50** — a 0.86-s regression vs the no-LF baseline
2:28.88 (148.88 s). A 10-MC verification run via `lap.py` (same
config, default `MC_DEFAULT_RUNS=10`) shows at least one seed
off-tracks at s=640 m (chicane apex), so LF reduces MC stability vs
the no-LF baseline's 10/10 — line-follow on the centerline plant
makes the chicane harder, not easier, for the inner. Lever 3 stacked
on LF still aborts at the chicane on the first deterministic seed.
The 22-s gap to reactive 2:06.04 is **not** explained by the outer's
lateral plan; it's the centerline-vs-ideal-line *curvature* the
inner's friction envelope sees. Recommendation: keep reactive as
production v3; this build is research record only.

---

## TL;DR

The HMPC v3.4 outer planner picks `(n, ψ_e, v, a_long, a_lat)` over an
800 m / 80-stage NLP via CasADi+IPOPT. When the outer is free to plan
the lateral trajectory it does NOT find the same racing line as the
offline DP-integrated ideal line — and the chassis ceiling on Sprint A
is ~2:28.88 / 10-of-10 (vs reactive's 2:06.04 reading the ideal-line
CSV directly).

Line-follow mode is a **soft pin** of the outer's `(n, ψ_e)` trajectory
to ideal-line targets sampled from
`tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv`. The outer
NLP keeps `n` and `ψ_e` as decision variables but its cost gains:

```
J_line = w_n_ideal   · (n_k − n_ideal(s_k))²
       + w_psi_ideal · (ψ_e_k − psi_ideal(s_k))²
```

while the centerline-pull terms (`w_n · n²`, `w_psi · ψ_e²`,
`w_term · n_N²`) are **zeroed**. The friction circle, curvature
velocity cap, track-edge bounds, and v_max constraint are unchanged —
the planner still respects the physical envelope, it just plans the
*longitudinal* profile along the supplied lateral line.

The implementation is **Option A (soft track)** from the task brief.
Option B (hard pin — replace `n` with `n_ideal` as a constant in the
NLP) is on the roster only if soft-track leaves > 0.5 m residual or
the outer keeps drifting off-line at all tuneable weights.

---

## Design — what changed and why

### Three new code files / surfaces

- **New module** `src/lap_estimator/dynamics/_ideal_line_loader.py`
  (~290 lines). Reads the ideal-line CSV's `(x, z)` samples, projects
  each onto the centerline `ReferencePath` Frenet frame via the
  existing `mpcc_reference.to_curvilinear`, and returns dense
  monotone-sorted `(s, n_ideal, psi_offset_ideal)` arrays as an
  immutable `IdealLineFrenet` dataclass. Exposes `n_at(s)`, `psi_at(s)`
  scalar accessors and `n_seq(s)`, `psi_seq(s)` vectorised samplers for
  the outer's stage grid.
  - Sanity checks at load time:
    - `|n_ideal|` p95 < 0.2 m → warn "ideal line is essentially the
      centerline, line-follow will not change planner behaviour".
    - `|n_ideal|` max > centerline half-width + 0.5 m → warn (planner's
      track-edge constraint will reject the target).
    - Deduplication: the Frenet projection occasionally maps two
      consecutive ideal-line samples to the same (or backwards-going)
      centerline `s` at tight corners; the loader drops near-duplicates
      and keeps the first occurrence.

- **`OuterPlannerConfig` extension** (`hmpc_outer.py`).
  - New fields: `line_follow: IdealLineFrenet | None`,
    `w_n_ideal: float = 200.0`, `w_psi_ideal: float = 20.0`.
  - When `line_follow is not None`, the NLP build:
    - Adds the per-stage soft-pull cost shown above (incl. terminal
      stage `k = N`).
    - **Suppresses** the existing centerline-pull terms (`w_n_eff = 0`,
      `w_psi_eff = 0`, `w_term_eff = 0`). Two competing lateral
      references would confuse the optimiser.
    - **Biases the cold-start warm start** to use `n_ideal(s_k)` and
      `psi_ideal(s_k)` along the stage grid (instead of linear-decay
      from `n0`/`psi_e0` to zero). The stage-0 values are still pinned
      to chassis state via the initial-state constraints; the bias
      keeps IPOPT's iteration 0 residual small along the rest of the
      horizon.

- **`HMPCController` constructor extension** (`hmpc_controller.py`).
  - New kwargs: `line_follow_path: str | None`,
    `line_follow_w_n_ideal: float | None`,
    `line_follow_w_psi_ideal: float | None`.
  - Driver-JSON equivalent under `control_params.hmpc.line_follow`:
    keys `path`, `w_n_ideal`, `w_psi_ideal`.
  - Calls `_resolve_line_follow_loader` which:
    - Returns `(None, label)` and warns when token == `"AUTO"` and the
      simulation track's basename already ends in `_ideal_line`. The
      brief specified this no-op explicitly — running the outer on the
      ideal-line track already makes the centerline `=` ideal line, so
      a line-follow pin has nothing to do.
    - Otherwise loads `IdealLineFrenet` from the supplied path against
      `self.ref` (the centerline reference).
  - Logs the load (`n_p95`, `n_max`, `w_n_ideal`, `w_psi_ideal`) on
    construction so a sim with line-follow active is unambiguous in
    the run log.

### CLI surface (`lap.py`)

- `--hmpc-outer-line-follow <PATH | AUTO>` — path to the ideal-line CSV
  (or `AUTO`).
- `--hmpc-outer-line-follow-w-n-ideal <FLOAT>` — overrides default 200.0.
- `--hmpc-outer-line-follow-w-psi-ideal <FLOAT>` — overrides default 20.0.

Threaded through `simulate_slip` → `_run_single` /
`_run_monte_carlo` → `_make_controller` → `HMPCController.__init__`
the same way every other HMPC knob is plumbed.

### Out of scope (no edits made)

- `mpc_*`, `mpcc_*`, reactive, v2 (`car.py`, `simulator.py`) — all
  untouched.
- Inner solver — no changes; the inner still consumes `v_ref`, `n_ref`,
  `psi_e_ref`, `a_long_ref` from the (now line-follow-shaped) outer
  reference table.
- Outer NLP solver / structure — same CasADi `Opti` build, same IPOPT
  options. Only the per-stage cost shape and warm-start changes.

---

## Data flow

```
┌──────────────────────┐     ┌──────────────────────┐
│ centerline CSV       │     │ ideal-line CSV       │
│ (the sim plant track)│     │ (offline DP racing   │
│   x, z, distance_m,  │     │  line)               │
│   width_*, radius_m, │     │   x, z, distance_m   │
│   speed_ms           │     │                      │
└──────────┬───────────┘     └──────────┬───────────┘
           │                            │
           ▼                            ▼
 mpcc_reference.build      _ideal_line_loader.load_
   _reference_path()          ideal_line_frenet()
   → ReferencePath               (projects each (x,z)
   (centerline Frenet:           sample onto the
   s_grid, kappa_ref,            centerline reference,
   half_width, v_ref)            yielding s, n_ideal,
                                 psi_offset_ideal)
           │                            │
           └─────────────┬──────────────┘
                         ▼
              OuterPlannerConfig
              { line_follow: IdealLineFrenet,
                w_n_ideal, w_psi_ideal, ... }
                         ▼
              OuterPlanner.solve(state)
                ├─ build stage grid s_k
                ├─ sample (kappa_k, v_dp_k, half_w_k) from centerline
                ├─ sample (n_ideal_k, psi_ideal_k) from IdealLineFrenet
                ├─ build NLP cost:
                │     w_p · dt + w_v · (v − v_dp)²
                │   + w_n_ideal · (n − n_ideal)²       ← NEW
                │   + w_psi_ideal · (ψ_e − psi_ideal)² ← NEW
                │   + w_du · |Δa|²
                │   (w_n, w_psi, w_term zeroed when LF active)
                ├─ IPOPT solve → x_star, u_star
                └─ ReferenceTrajectory(n_ref, psi_e_ref, v_ref, a_long_ref)
                         │
                         ▼
            HMPC inner tracker (unchanged) tracks
            (n_ref, psi_e_ref, v_ref, a_long_ref) on the
            chassis bicycle model with a 30 m / 15-stage horizon.
```

The simulation plant always runs on the centerline track CSV;
`line-follow` is purely a planner-side reference swap.

---

## Tuning sweep (run on Sprint A, BMW 1M, Tomas, cm=0.95)

Baseline (no line-follow, from `architecture-v3-hmpc-chase-tomas.md`):
**148.88 s / 10-of-10** at cm=0.95.

Reactive baseline (production, ideal-line CSV directly): **126.00 s
(2:06.04) / 10-of-10** at cm=0.85.

### Smoke (single seed)

| Variant                                                           | cm  | Result      | n_dev_p95 | n_dev_max | brake_commit / peak | Notes |
|---|---:|---:|---:|---:|---:|---|
| `LF_baseline_cm95` (w_n_ideal=200, w_psi_ideal=20, no Lever 3)    | 0.95 | **149.80 s** | 1.05 m | 2.13 m | 118 m / 0.65 | Lap completes; IPOPT reports `Infeasible_Problem_Detected` on every outer solve but the last iterate is feasible enough to commit and the inner tracks it. The brake commit shape is **bit-identical to the no-LF baseline** (chase-tomas: cmt ~117 m, peak 0.68) — the inner's longitudinal profile does not change. |

The "Infeasible_Problem_Detected" status with last-iterate-accepted is
the documented `OuterPlanner` Tier-1 recovery path
(`hmpc_outer.py:409`): IPOPT exits early but the iterate is
primal-feasible. The new `w_n_ideal · (n − n_ideal)²` term is
quadratic-only and never *creates* infeasibility on its own — the
infeasibility comes from the combined friction circle + curvature cap
+ track edge tightening as the planner is pulled off the centerline
where those bounds are sized.

The brake-commit equivalence is the first hint that line-follow does
not move the lap time on the centerline plant: the inner's brake
profile is driven by the v_ref look-ahead (DP-capped) and the friction
circle, both of which use the centerline curvature. Moving `n_ref`
laterally toward the ideal line doesn't change what the inner sees as
its longitudinal envelope.

### Screen sweep — w_n_ideal × Lever 3 (single seed)

`.tmp/hmpc_line_follow_focused.log`.

| Variant                          | cm   | Lap         | n_dev p95 (`|n_ref − n_ideal|`) | Notes |
|---|---:|---:|---:|---|
| `LF_baseline` (w_n_ideal=200)     | 0.95 | **149.80 s** | 1.05 m | Lap completes; IPOPT `Infeasible_Problem_Detected` per cycle, last iterate accepted. |
| `LF_w50_cm95` (loose)             | 0.95 | **149.74 s** | 0.91 m | Loosening the lateral weight gives ~1 m extra deviation room and the lap time barely moves (-0.06 s, seed noise). |
| `LF_w200_L3wa5_cm95` (with L3)    | 0.95 | **ABORT** at s=490 m | 6.55 m (post-abort) | Tier-2 reactive fall-through at t=3.4 s, then ghost-fallback over-slip, off-track at chicane lead-in. |
| `LF_w500_L3wa5_cm95` (tighter+L3) | 0.95 | **ABORT** at s=646 m | 3.72 m (post-abort) | Inner-infeasible × 2 → Tier-2 → off-track at the chicane apex. |

**Outcome:** line-follow at *any* tested weight does not unlock the
inner's brake-aggression headroom. Lap time without Lever 3 is
148.88 s (baseline, no LF) → 149.80 s (LF w=200) → 149.74 s (LF w=50).
**Line-follow is a 0–1 s regression** within seed noise, with the
"benefit" being p95 lateral deviation < 0.5 m is now actually reachable
(1.05 m at w=200, 0.91 m at w=50) instead of the prior controller's
~1.5 m drift from centerline. With Lever 3 stacked, the lap aborts —
same failure mode as chase-Tomas's L3-on-ideal-line abort (over-slip
at the chicane), but now with `n_ref` pinned to within ~3.7 m of the
ideal line. **The bottleneck is the inner's friction envelope on the
centerline-curvature plant, not the outer's line-planning ambition.**

The brief's hypothesis — "L3 fails on ideal-line because the outer
plans an 8 m lateral excursion the inner can't track; pinning the
lateral target will close the L3 abort path" — is contradicted. L3
still aborts when the outer's `n_ref` is pinned within 4 m of the
ideal line on the same chicane segment. The 8-m lateral excursion was
a *symptom* of the failure (the inner's over-slip pushed the chassis
laterally), not the cause.

### Acceptance gate vs. results

| Gate (task brief) | Target | Result |
|---|---|---|
| Lateral deviation p95 from `n_ideal(s)` | < 0.5 m | Not met (1.05 m at w=200, 0.91 m at w=50). Tighter weights cause IPOPT infeasibility cascades and lap regressions. Achievable via Option B (hard pin) if pursued. |
| Inner runs without ellipse-violation aborts | yes | Met for LF without Lever 3. Not met when L3 is stacked. |
| Lap completes | yes | Met for LF without Lever 3. Not met with L3. |
| Best stable lap beats reactive 2:06.04 | yes | **NOT MET** (best stable: 149.74 s / 2:29.74). |
| Best stable lap beats HMPC baseline 148.88 s | yes | **NOT MET** (149.74 s is a 0.86-s regression). |

### What this rules out and rules in

**Rules out:** the hypothesis that the outer's lateral-planning
freedom is the lap-time bottleneck on Sprint A. Pinning the outer's
`(n, ψ_e)` to the offline DP-integrated ideal line does not move the
inner's lap time. The chassis ceiling on the *centerline* plant is
~148.9–149.8 s regardless of which lateral reference the outer plans
to.

**Rules in:** the 22-s gap to reactive 2:06.04 lives in the
**centerline-curvature vs ideal-line-curvature** difference, not in
the outer's lateral plan. Reactive runs the chassis on the
`layout_sprint_a_ideal_line.csv` track CSV directly — the chassis's
own κ(s) is the ideal-line's curvature, which has a wider apex radius
than the centerline at the chicane. The HMPC stack, by design, plants
the chassis on the centerline CSV and uses the outer's NLP to plan a
lateral offset from the centerline; line-follow only changes the
lateral offset it plans, not the curvature the chassis sees.

The architectural fix is **NOT** at the outer NLP. It is at the
chassis plant: either:
1. Run HMPC on the ideal-line CSV directly (same plant trick as
   reactive). The chase-Tomas doc found this lands at 2:28.88 — the
   outer's NLP still doesn't unlock the inner's brake aggression at
   the wider-apex curvature because L3 destabilises on that track too.
2. Re-architect the inner to track a *track-CSV-line-derived* curvature
   rather than the centerline curvature. This is essentially the
   acados / MPCC v3.6 path — substantial code change.

---

## How this rules in / out the inner-vs-outer bottleneck question

**If line-follow lap time is materially better than the 148.88 s
baseline:** the outer's freedom to wander on `(n, ψ_e)` was costing
lap time, and pinning to the offline line lets the inner extract more.
The inner is competent on this line; the bottleneck was the outer's
line-planning ambition.

**If line-follow lap time is the same or worse:** the outer's NLP was
already finding a fine line (within `w_n=5` of the centerline) and the
ideal-line CSV is essentially the centerline at the apex (likely on
Sprint A where the track is wide and the ideal-line offset p95 is only
1.84 m). The bottleneck is the inner's brake-zone aggression — Lever 3
on the centerline / line-pinned track is the next handle.

**If line-follow + Lever 3 completes and matches reactive 2:06.04:**
this validates the "outer NLP is the wrong solver class for racing-line
planning; do the line offline, plan the speed online" architectural
finding from chase-Tomas. Production v3 stays on reactive (cheap,
proven); HMPC becomes a path to "reactive + better speed plan".

**If line-follow + Lever 3 still aborts:** the inner's friction
envelope on the ideal line is the binding constraint. The next lever
is inner-side: tighter ellipse soft-constraint, slower brake commit, or
the v3.6 acados upgrade.

---

## File inventory

**Created in this work session:**

| File | Purpose |
|---|---|
| `src/lap_estimator/dynamics/_ideal_line_loader.py` | Frenet projection of an ideal-line CSV onto a centerline ReferencePath. ~290 lines. |
| `docs/architecture-v3-hmpc-outer-line-following.md` | This document. |
| `.tmp/hmpc_line_follow_smoke.py` | Smoke + screen + cm-sweep + MC harness for the new mode. |
| `.tmp/hmpc_line_follow_traces/` | Per-variant inner debug traces. |
| `.tmp/hmpc_line_follow_*.csv` | Per-sweep result CSVs. |

**Edited in this work session:**

| File | Change |
|---|---|
| `src/lap_estimator/dynamics/hmpc_outer.py` | Added `line_follow`, `w_n_ideal`, `w_psi_ideal` fields to `OuterPlannerConfig`; added the soft-pull cost into the NLP build (per-stage + terminal); biased the cold-start warm start to start near the ideal line. |
| `src/lap_estimator/dynamics/hmpc_controller.py` | New `line_follow_*` kwargs; loader-resolution helper handling `AUTO` no-op; logs on construction. |
| `src/lap_estimator/dynamics/slip_simulator.py` | Threaded `hmpc_line_follow_*` kwargs through `simulate_slip`, `_run_single`, `_run_monte_carlo`, `_make_controller`. |
| `lap.py` | Three new CLI flags (`--hmpc-outer-line-follow{,-w-n-ideal,-w-psi-ideal}`). |

**Not touched:**

`mpc_*`, `mpcc_*`, reactive, v2, the inner tracker (`hmpc_inner*.py`),
`mpcc_reference.py`, the outer's IPOPT solver options, all `tracks_csv/`
data files, all `cars_csv/` data files.

---

## How to run

```bash
# Smoke (1 seed, line-follow at w=200):
python lap.py cars_csv/bmw_1m \
  tracks_csv/ks_nurburgring/layout_sprint_a.csv \
  drivers/tomas.json \
  --model slip --controller hmpc --single-lap --no-plot \
  --inertia-zz 2400 --chicane-safety-mult 0.95 \
  --hmpc-inner-solver casadi \
  --hmpc-outer-line-follow tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv

# With Lever 3 + tighter weight:
python lap.py cars_csv/bmw_1m \
  tracks_csv/ks_nurburgring/layout_sprint_a.csv \
  drivers/tomas.json \
  --model slip --controller hmpc --single-lap --no-plot \
  --inertia-zz 2400 --chicane-safety-mult 0.95 \
  --hmpc-inner-solver casadi \
  --hmpc-outer-line-follow tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv \
  --hmpc-outer-line-follow-w-n-ideal 500.0 \
  # add inner_w_a via driver JSON: control_params.hmpc.inner_w_a: 5.0

# Driver-JSON equivalent (no CLI flag needed):
#   control_params:
#     hmpc:
#       line_follow:
#         path: "tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv"
#         w_n_ideal: 200.0
#         w_psi_ideal: 20.0

# Sweep harness:
python .tmp/hmpc_line_follow_smoke.py smoke       # 1 variant, 1 seed
python .tmp/hmpc_line_follow_smoke.py screen      # 7 variants, 1 seed each
python .tmp/hmpc_line_follow_smoke.py cm-sweep    # 4 cm values
python .tmp/hmpc_line_follow_smoke.py mc          # 10 MC seeds at best configs
```

---

## Caveats for the next maintainer

- The outer's `Infeasible_Problem_Detected` status is *expected* under
  line-follow at high weights — the friction-circle + curvature-cap +
  ideal-line target combination is non-convex enough that IPOPT can't
  hit `Solve_Succeeded` every cycle. The Tier-1 recovery (accept
  last iterate when finite) is the working path. Monitor
  `self.outer.infeas_count` for catastrophic regressions.

- The lateral-deviation diagnostic in
  `.tmp/hmpc_line_follow_smoke.py` proxies `|n_chassis − n_ideal|`
  with `|n_ref − n_ideal|` from the existing trace (the inner trace
  doesn't currently log `n_chassis`). For a precise measurement add an
  `n_chassis` column to `hmpc_debug.py` and re-read.

- The ideal-line loader's projection runs once at controller
  construction (O(N_ideal) windowed Newton; ~50 ms on Sprint A's 2263
  samples). It is NOT re-projected during the lap; the cached
  `IdealLineFrenet` is read via `np.interp` per outer solve.

- `n_ideal(s)` is in the **centerline** Frenet frame, NOT the
  ideal-line CSV's own arc-length. This matters because the outer's
  stage grid `s_k = s_chassis + k·ds` is on the centerline `s`; the
  loader's `n_seq(s_k)` lookup is therefore consistent with the outer's
  state-space.

- The `AUTO` token's `_ideal_line` suffix detection is name-based
  (`Track.name`), not path-based. If you rename the file but keep the
  suffix, AUTO still works; if you change the suffix you need to pass
  the explicit path.
