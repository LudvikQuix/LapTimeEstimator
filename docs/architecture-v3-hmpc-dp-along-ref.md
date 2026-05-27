# Architecture — v3 HMPC inner DP-sourced a_long reference (Lever-3 source switch)

**Spec / task brief:** 2026-05-27 DP a_long-ref brief — source the HMPC
inner's Lever-3 `a_long_ref` from the whole-lap **DP plan** instead of the
short-horizon **outer NLP** plan, to give the inner a longer-look-ahead
deceleration target so it commits brake earlier.

**Predecessors:**
- `docs/architecture-v3-hmpc-inner-asymmetric-pedals.md` — Phase 2
  ellipse-soft sweep; the "Phase-2 revised next lever" note that scoped
  this task (es=500 + L3 wa=5 → 2:23.04 best / 2:24.09 median 10-MC).
- `docs/architecture-v3-hmpc-inner-tuning.md` — Lever-3 mechanics
  (`inner_w_a · (a_long − a_long_ref)²`).
- `docs/architecture-v3-hmpc-casadi-outer.md` — outer NLP +
  `ReferenceTrajectory.a_long_ref_at(s)`.

**Branch:** `feature/sc-71955/lap-simulation`.

---

## What the code does (one paragraph)

The HMPC inner's Lever-3 cost term `w_a · (a_long − a_long_ref)²` gives the
inner a longitudinal-deceleration target. Until now `a_long_ref` was sampled
exclusively from the outer NLP's planned `a_long` (`ReferenceTrajectory.
a_long_ref_at(s)`), whose horizon is ~1 s (20 stages × 50 ms ≈ 1 s at the
outer's 800 m / 80-stage plan), so at the chicane lead-in the reference does
not yet "see" the apex and under-asks for brake. This work adds a
source-switch (`control_params.hmpc.inner_a_long_ref_source` ∈
{`outer`, `dp`, `blend`}) that lets the inner instead source `a_long_ref`
from the **whole-lap DP plan** — the same `v3_dp` plan the controller
already carries — by deriving `a_long_dp(s) = v(s) · dv/ds` once from the
plan's `v(s)` profile and sampling it at the inner stage grid. The DP plan
is computed over the entire lap against the fitted Pacejka envelope, so its
`a_long(s)` knows the apex deceleration far upstream. The cost term itself
is unchanged — only the *source* of the reference sequence changes.

---

## Why this architecture

### Source switch, not a new cost

Lever 3 already works (Phase 2: es=500 + wa=5 → 2:23.04). The diagnosis
from the Phase-2 doc is that the brake commits *late* because the
reference's look-ahead is the outer horizon. The minimal, architecturally
clean fix is therefore to swap the **reference source**, leaving the inner
cost term (`hmpc_inner_casadi.py`) bit-identical. The inner already accepts
an `a_long_ref_seq` sequence sampled at its stage grid; we only change what
fills that array in `HMPCController._resolve`.

### DP `a_long` derived as `v · dv/ds`

The `LongitudinalPlan` (see `longitudinal_planner.py`) exposes `distances`
and `speeds` (`v(s)`) but not `a_long(s)`. Along arc length the kinematic
identity `a_long = dv/dt = (dv/ds)·(ds/dt) = v · dv/ds` gives the planned
longitudinal accel directly from `v(s)`. We compute it once at controller
construction with `np.gradient(v, s)` (handles the plan's non-uniform grid
and one-sided endpoints) and cache it as a callable
(`DPLongitudinalReference`). This is read-only against the planner — the DP
solver's logic is untouched.

### Three sources, blend as the hedge

- `outer` (default): unchanged Phase 5.3 behaviour. Bit-identical for every
  existing caller.
- `dp`: pure DP plan. Longest look-ahead; the risk (per the Phase-2 doc's
  scattered-brake failure mode) is that pure-DP over-commits brake on the
  lead-in straight, where the friction budget is wasted on straight-line
  braking.
- `blend`: `(1−α)·outer + α·dp`, α = `inner_a_long_ref_blend` (default 0.5).
  This hedges: a low α nudges the brake commit upstream without fully
  committing to the DP profile on the lead-in.

### Sign / frame check (load-bearing)

DP `a_long`, outer `a_long`, and the inner's chassis-frame `a_long_k` must
share sign convention (decel negative; accel positive along the path),
otherwise the inner would brake on straights and accelerate into corners —
the same class of bug as the v_ref-lookahead sign inversion in the Phase 5.3
close-out (`hmpc_outer.py` history).

Convention check:
- Outer NLP control: `v_{k+1} = v_k + a_long_k · dt_k` ⟹ decel ⟹ `a_long < 0`.
- Inner chassis-frame: `a_long_k = (Fx − F_drag)/m` ⟹ decel ⟹ `a_long < 0`.
- DP: `a_long_dp = v · dv/ds` ⟹ `dv/ds < 0` in brake zone ⟹ `a_long < 0`.

All three agree. Verified empirically on the Sprint A ideal-line
(`tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv`, BMW 1M,
Tomas, cm=0.95, DP `safety_margin` per ChicaneSafetyConfig):

| Zone | s-range (m) | mean DP a_long (m/s²) | expected |
|---|---|---:|---|
| threshold-brake | 475–585 | **−9.465** | < 0 (brake) |
| main straight | 50–200 | **0.000** | ≥ 0 (cruise/accel) |
| most-negative point | s=2842 | **−10.601** (v=52 m/s) | < 0 (downstream fast corner) |

The straight-zone value is exactly 0.0 because the DP plan is already at the
corner-entry speed cap there (`dv/ds ≈ 0`); the convention (`≥ 0`) holds.
`HMPCController._log_dp_along_sign_check` runs this check on every HMPC build
and logs a WARNING (not an abort) on violation, since non-Sprint-A tracks
may not have these exact zones.

---

## Data flow (per inner tick, Tier-0 / HMPC path)

```
HMPCController._resolve(state, t)
  |
  +-- build inner stage grid: s_seq = s_now + arange(N) · ds_stage
  |
  +-- outer reference valid + covers s_seq?
  |     |
  |     +-- a_long_ref_seq = self._resolve_a_long_ref(s_seq):
  |           source == "outer":  [ ReferenceTrajectory.a_long_ref_at(s) for s in s_seq ]
  |           source == "dp":     DPLongitudinalReference.sample(s_seq)      # v·dv/ds
  |           source == "blend":  (1-α)·outer_seq + α·dp_seq
  |
  +-- inner.solve(..., a_long_ref_seq=a_long_ref_seq)   # CasADi inner UNCHANGED
        |
        +-- cost (gated on w_a>0): w_a · mask[k] · (a_long_k − a_long_ref[k])²
  |
  +-- first_stage_commit → (steer, throttle, brake)
```

Construction-time, once:

```
HMPCController.__init__
  +-- self._dp_along_ref = DPLongitudinalReference.from_plan(plan)   # v·dv/ds cached
  +-- self._log_dp_along_sign_check()                               # sign evidence
```

Tier-1 (DP-plan fallback) and Tier-2 (reactive) are unchanged: on Tier-1 the
inner is called with `a_long_ref_seq=None` (mask=0), so the source switch is
irrelevant there.

---

## File inventory

**Created:**

| File | Purpose |
|---|---|
| `src/lap_estimator/dynamics/_dp_reference.py` | `DPLongitudinalReference` — derives + caches `a_long_dp(s) = v·dv/ds` from a `LongitudinalPlan`; provides `a_long_at(s)` / `sample(s_seq)`. ~95 lines. Keeps the DP-extraction out of the already-oversized `hmpc_controller.py`. |
| `docs/architecture-v3-hmpc-dp-along-ref.md` | This document. |
| `.tmp/hmpc_inner_dp_along_sweep.py` | Sweep + MC harness (modes smoke/blend/retune_wa/mc/sweep_cm/all). Reuses `.tmp/hmpc_inner_ellipse_soft_sweep.py` patterns. |

**Modified:**

| File | Change |
|---|---|
| `src/lap_estimator/dynamics/hmpc_controller.py` | New module constants (`A_LONG_REF_SOURCES`, defaults, sign-check zones); two new constructor kwargs (`inner_a_long_ref_source`, `inner_a_long_ref_blend`) resolved against the `control_params.hmpc` driver-JSON block; build the cached `DPLongitudinalReference` + run the one-time sign check; new `_resolve_a_long_ref(s_seq)` helper implementing the source switch; the `_resolve` tick now calls it instead of sampling the outer reference inline. Build-log line surfaces the active source/blend. |

**Not touched (per brief constraint):**

- `src/lap_estimator/dynamics/hmpc_inner_casadi.py` — the inner already
  accepts `a_long_ref_seq` and the Lever-3 cost is unchanged; we only change
  what is passed in. Zero edits.
- `src/lap_estimator/dynamics/longitudinal_planner.py` — DP planner logic
  untouched; `DPLongitudinalReference` reads `plan.distances`/`plan.speeds`
  read-only.
- `src/lap_estimator/dynamics/hmpc_outer.py` — outer NLP unchanged.
- reactive / v2 / MPC / MPCC / line-follow code — untouched.

**CLI:** no new `lap.py` flags. Consistent with the existing inner
brake-aggression knobs (`inner_w_a`, `inner_w_ellipse_soft`) and the v3.7
asym-pedal knobs, all of which are driver-JSON-only. The driver-JSON
`control_params.hmpc` block is the single entry point. The brief explicitly
states the JSON path is sufficient.

---

## How to enable

Driver-JSON block (extends the Phase-2 headline config):

```json
"control_params": {
  "hmpc": {
    "inner_solver": "casadi",
    "outer_vref_lookahead_stages": 20,
    "inner_w_du_brake": 0.5,
    "inner_w_du_throttle": 10.0,
    "inner_w_brake_double_well": 50.0,
    "inner_w_brake_throttle_overlap": 50.0,
    "inner_w_ellipse_soft": 500.0,
    "inner_w_a": 5.0,
    "inner_a_long_ref_source": "dp",
    "inner_a_long_ref_blend": 0.5
  }
}
```

`inner_a_long_ref_source` defaults to `"outer"` (bit-identical to prior
builds). `inner_a_long_ref_blend` is only consulted when source is
`"blend"`; default 0.5, clipped to [0, 1].

---

## Results

All runs: Sprint A ideal-line, BMW 1M (`inertia_zz=2400`), Tomas driver
carrying the Phase-2 headline config (es=500, wa=5, asym defaults
du_brk=0.5, du_thr=10, dw=50, ov=50), `inner_solver=casadi`,
`outer_vref_lookahead_stages=20`, `plan_source=v3_dp`. Single deterministic
seed unless noted. Harness: `.tmp/hmpc_inner_dp_along_sweep.py`.

### Smoke — `outer` baseline vs pure `dp` (cm=0.95, single seed)

| source | lap (s) | brake commit s (m) | brake peak | p95 thresh | frac>0.95 | mean decel |
|---|---:|---:|---:|---:|---:|---:|
| `outer` (baseline) | **143.86** | 342.5 | 1.00 | 0.92 | 0.07 | 5.76 |
| `dp` (α=1) | 150.06 | **226.1** | 0.95 | 0.87 | 0.01 | 5.39 |

**The key proof: DP-sourced `a_long_ref` moves the brake commit upstream by
116 m (s=342 → s=226).** This is exactly the intended effect — the DP plan's
whole-lap look-ahead lets the inner anticipate the chicane brake far earlier
than the outer's ~1 s horizon. But **pure `dp` over-commits**: the early
brake bleeds speed on the lead-in straight where the friction budget doesn't
pay off in lap time, costing +6.2 s. This is the scattered-brake failure
mode the Phase-2 doc warned about, and the reason `blend` exists.

### Blend sweep — α ∈ {0.25, 0.5, 0.75, 1.0} (cm=0.95, single seed, wa=5)

| α (blend) | lap (s) | brake commit s (m) | brake peak | p95 thresh | finished |
|---:|---:|---:|---:|---:|:--:|
| 0.25 | 146.22 | 230.4 | 0.91 | 0.84 | yes |
| 0.50 | ABORT (off-track s=648) | 230.4 | 1.00 | 0.92 | **no** |
| 0.75 | ABORT (off-track s=658) | 267.2 | 1.00 | 0.91 | **no** |
| 1.00 (pure dp) | 150.06 | 226.1 | 0.95 | 0.87 | yes |

Mid blends (0.5, 0.75) **abort off-track at the chicane apex** — the DP
contribution over-commits brake into the lateral-load zone (the same
over-decel mechanism that kills high `inner_w_a`). The stable points are the
extremes: α=0.25 (146.22 s) and α=1.0 (150.06 s). **Both move the brake
commit ~112-116 m upstream, but neither beats the 143.86 s `outer`
baseline at wa=5.** The longer-horizon DP reference asks for more brake than
the chassis can spend profitably on the lead-in — consistent with the
brief's stated risk. The next lever per the brief is to re-tune Lever 3
(lower `inner_w_a`) at the best stable blend, since the optimal `w_a` shifts
down once the reference is longer-horizon and the brake commits earlier.

### Lever-3 re-tune at α=0.25

The brief's hypothesis was that the optimal `inner_w_a` shifts *down* once
the reference is the longer-horizon DP-blend and the brake already commits
earlier — i.e. a softer Lever-3 pull would let the chassis recover lap time
lost to the early commit. Swept `wa ∈ {2, 3, 5, 8}` at source=`blend`,
α=0.25, cm=0.95, single seed (harness mode `retune_wa blend 0.25`).

| `inner_w_a` | lap (s) | brake commit s (m) | brake peak | p95 thresh | finished |
|---:|---:|---:|---:|---:|:--:|
| 2 | 150.08 | 204.3 | 0.92 | 0.83 | yes |
| 3 | 149.34 | 212.7 | 0.91 | 0.83 | yes |
| 5 | **146.22** | 230.4 | 0.91 | 0.84 | yes |
| 8 | ABORT (off-track s=653) | 242.8 | 0.96 | 0.90 | **no** |

The hypothesis is **refuted**: lowering `w_a` moves the brake commit *further*
upstream (s=230 → 204 m) and makes lap time monotonically **worse**
(146.22 → 149.34 → 150.08 s), not better. The mechanism is the inverse of
what the brief expected: at α=0.25 the blended reference already over-asks
for brake on the lead-in, and a *weaker* Lever-3 pull lets the inner track
that over-eager DP contribution more loosely on the *downstream* side while
still committing early — net, it spends more of the lead-in straight off
throttle. Raising `w_a` to 8 over-commits into the lateral-load zone and
aborts off-track (same over-decel pathology as the mid-blend failures).
The best finishing variant remains `wa=5` at **146.22 s**.

### Verdict

DP-sourcing the inner's Lever-3 `a_long_ref` does **not** beat the `outer`
NLP baseline (143.86 s, wa=5) on Sprint A, at any tested blend or re-tuned
`w_a`. Across the full investigation the best DP-influenced finisher is
α=0.25 / wa=5 at 146.22 s — **+2.36 s slower** than `outer`. Every DP-blend
variant moves the brake commit ~112–116 m upstream as designed, but that
early commit consistently *costs* lap time on the lead-in straight where the
friction budget does not pay off in straight-line braking, and re-tuning
`w_a` does not recover it. The `outer` NLP's ~1 s horizon, far from being a
deficiency, is the *right* look-ahead for the inner's Lever-3 target on this
track: it commits brake just-in-time rather than early.

**Recommendation:** keep `inner_a_long_ref_source="outer"` (the default) as
the production setting. The DP/blend source switch stays in the codebase as
a tested, sign-verified, bit-identical-when-off lever, but it is not a
lap-time win here. The 10-seed MC at the best blend was deliberately **not**
run, since no DP-influenced variant beat the `outer` baseline — running it
would only characterise the stability of a known-slower configuration.

---

## Notes for the next maintainer

- The DP `a_long` is whole-lap and static (computed once at construction);
  it does not change tick-to-tick. The outer `a_long` is re-planned every
  outer fire (~1 Hz) against the current chassis state. `blend` therefore
  mixes a static long-horizon signal with a live short-horizon one.
- If pure `dp` (α=1) aborts on the lead-in, that is the expected
  scattered-brake pathology (Phase-2 doc); the blend exists for exactly this
  case. Do not force pure-`dp`.
- `DPLongitudinalReference` clips at the plan's grid edges (same convention
  as the outer's `*_at` accessors). The inner stage grid (≤ ~30 m window) is
  always well inside the plan's ~3570 m extent on Sprint A.
- The sign-check log is diagnostic-only (WARNING, no abort). If you port
  this to another track, eyeball the logged brake/straight means or adjust
  `_SIGN_CHECK_*_ZONE_M` to that track's geometry.
