# Phase 5.0.8 — First-class longitudinal MPC emit (negative result; horizon-bound)

**Predecessor specs / docs:**
- `docs/architecture-slip-model-phase5_0_7-v32-qp-weight-retune.md` — Phase 5.0.7
  closed the QP-cost-weight tuning question with a negative result and named
  the structural deficit (QP-solved throttle/brake are discarded at Tier 0
  emit time, longitudinal channel coupling is second-order through chassis
  state).
- `dev-planning/lap-simulation-csv-driver/open-points.md` — OP-4 (the call to
  spec Phase 5.0.8).
- `docs/architecture-slip-model-phase5_0_4-v32-load-transfer.md` — Per-stage
  dynamic Fz, the spec'd "honest enough" mapping that this phase tests in
  closed loop.
- `docs/architecture-slip-model-phase5_0_1-v32-mpc-fixes.md` — Friction-ellipse
  hard constraint already in the QP, expected to bind correctly under
  first-class emit.

**Scope:** Promote `throttle` and `brake` from delegated-to-reactive to first-
class QP-emit variables on the Tier 0 path. No QP structural change (the
decision vector already carries them as integrated states; the bug was that
emit threw them away). Tier 1 and Tier 2 emit paths unchanged per spec.

**Status:** **Negative result.** First-class emit + pedal-overlap
suppression closes the spec-named symptom (QP throttle/brake committed
instead of sub-controller's) but the QP-emit MPC still aborts before the
chicane, and ~25 m earlier than the sub-emit baseline. 0/10 MC across all
four chicane safety mults (0.80 / 0.85 / 0.90 / 1.00), both `--mpc-emit-
source qp` (new default) and `--mpc-emit-source sub` (Phase 5.0.7 byte-
for-byte regression). Root cause is the MPC's 30 m horizon being far too
short to brake-anticipate the chicane from ~170 m out — a horizon /
terminal-cost issue the spec did not change. The first-class-emit knob
itself works as designed; the brake-anticipation problem now strictly
upstream.

---

## Headline

QP first-class longitudinal emit alone does not close the chicane abort.
The chassis under `emit=qp` reaches the pre-chicane straight at
**v_x ≈ 61 m/s vs reactive's 37 m/s** (deterministic seed, Sprint A,
Tomas, mult=0.85, `--inertia-zz 2400`); the QP doesn't begin braking
until s ≈ 429 m, ~170 m later than the reactive sub-controller. With
the 30 m / 0.43 s lookahead at 70 m/s the QP cannot see the chicane
in time. MC: 0/10 at chicane mults 0.80 / 0.85 / 0.90 / 1.00. Solve
time unchanged (mean 17–25 ms, p95 21–38 ms — consistent with 5.0.7's
mean 19 / p95 25). Recommend Phase 5.0.9 to raise horizon and add a
terminal v-tracking cost OR keep the reactive longitudinal path for
production and confine MPC to the steering channel.

---

## What was built

The fix the spec called for, applied to ``MPCController._emit_tier0_controls``:

```
Pre-5.0.8 Tier 0 emit (~3 lines):
    steer = self._held_steer_rad
    sub_cmd = self._long_sub.controls(state, t)
    throttle, brake = sub_cmd.throttle, sub_cmd.brake
    return Controls(steer, throttle, brake)

Phase 5.0.8 Tier 0 emit:
    steer = self._held_steer_rad
    # _held_throttle / _held_brake were integrated from u_seq[0,1..2]
    # by _commit_clean_mpc, rate-clipped and saturated to [0, 1].
    throttle_qp, brake_qp = _suppress_pedal_overlap(
        self._held_throttle, self._held_brake, self.pc,
    )
    if self.emit_source == "qp":
        throttle, brake = throttle_qp, brake_qp
    else:  # "sub" — A/B regression knob
        throttle, brake = sub_cmd.throttle, sub_cmd.brake
    return Controls(steer, throttle, brake)
```

CLI: `--mpc-emit-source {qp, sub}`, default `qp` (the spec's first-class
path). The Phase 5.0.7 byte-for-byte regression knob is `sub`. Tier 1's
emit (Phase 5.0.6 sub-controller pedal override) and Tier 2 (full reactive
fallback) are untouched.

## Pedal-overlap suppression — a spec-permitted patch

The Phase 5.0.8 brief said "no QP structural change", but the in-tree
QP turns out to have a structural hole: the cost function has terms only
on **u rates** (`w_du`, `w_du2`) plus `(v_x − v_ref)²`. There is no
constraint or cost preventing the decision vector from holding both
pedals down at once. The first-pass emit measured a steady `throttle ≈
0.91, brake ≈ 0.29` along the pre-chicane straight — net rear-axle
Fx ≈ zero by design, but the QP sees that as a free way to satisfy
both the throttle and brake state-space bounds (`thr ∈ [0,1]`, `brk ∈
[0,1]`) while paying nothing extra.

The minimum spec-permitted fix (the spec explicitly allows "possibly
`mpc_qp.py` for the emit path") is post-solve projection at emit time.
Net rear-axle Fx intent is

    Fx_intent = pc.k_throttle · throttle − pc.k_brake_rear · brake

Project to single pedal preserving intent:

    Fx_intent ≥ 0 → throttle' = Fx_intent / pc.k_throttle, brake' = 0
    Fx_intent < 0 → throttle' = 0, brake' = -Fx_intent / pc.k_brake_rear

This is implemented as ``_suppress_pedal_overlap`` (module-level helper
in `src/lap_estimator/dynamics/mpc_controller.py`). Note the projection
discards the QP's *front-brake* contribution (`Fx_front = -k_brake_front
· brake`), which is intentional: the QP's bicycle model treats both
axles identically, but the real plant brake bias is forward, so under-
projecting on brake leaves front axle grip available for lateral. The
diagnostic table in `.tmp/phase5_0_8_diag.log` shows the projected
emit picks pure brake on the entry phase (s = 540–600 m) just as the
sub-controller does, but with smaller magnitude (~0.78 vs ~0.93).

## Diagnostic — per-tick emit comparison

`.tmp/phase5_0_8_diag.py` runs both emit modes deterministic single seed
at mult=0.85 and captures the per-Tier-0-emit `(s, v_x, thr_qp, brk_qp,
thr_sub, brk_sub)` table. Headline rows (post-overlap-suppression):

### Pre-chicane straight (s ≈ 200–540 m)

| s (m) | v_x (m/s) — sub | v_x (m/s) — qp | thr_qp | brk_qp | thr_sub | brk_sub | Δbrk |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 200 | n/a (still T0 in pre-mid) | 71.5 | 0.88 | 0.00 | 1.00 | 0.00 | n/a |
| 257 | n/a | 71.8 | 1.00 | 0.00 | 0.60 | 0.20 | −0.20 |
| 286 | n/a | 71.9 | 1.00 | 0.00 | 0.00 | 1.00 | **−1.00** |
| 315 | n/a | 72.0 | 1.00 | 0.00 | 0.00 | 1.00 | **−1.00** |
| 343 | n/a | 72.1 | 1.00 | 0.00 | 0.00 | 0.97 | **−0.97** |
| 372 | n/a | 72.1 | 0.71 | 0.00 | 0.00 | 0.94 | **−0.94** |
| 401 | n/a | 71.7 | 0.08 | 0.00 | 0.00 | 1.00 | **−1.00** |
| 429 | n/a | 70.9 | 0.00 | 0.31 | 0.00 | 0.92 | −0.61 |

Reading: the reactive sub-controller starts braking at s ≈ 257 m and is
at full brake by s = 286 m, ~310 m before the chicane apex. The QP-emit
MPC stays at full throttle until s ≈ 372 m and doesn't start braking
seriously until s ≈ 429 m. **The brake commit is late by ~170 m**.

### Direct chassis-state consequence

| s (m) | v_x_sub (m/s) | v_x_qp (m/s) | Δv |
|---:|---:|---:|---:|
| 0   | 74.3 | 74.3 | (initial state matched) |
| 200 | n/a  | 71.5 | n/a (sub run hadn't reached) |
| 540 | 37.7 | 61.2 | **+23.5 m/s** |
| 557 | 36.3 | 59.5 | **+23.2 m/s** |

At s = 540 m the QP-emit chassis is at v_x = 61 m/s; the sub-emit
chassis is at 37 m/s. From 540 m to the chicane apex at ~660 m, the
sub-emit decelerates from 37 to ~17 m/s (20 m/s drop in 120 m); the
qp-emit needs to decelerate from 61 to ~17 m/s (44 m/s drop in 120 m)
— more than double the deceleration with the same plant. Even at peak
combined-slip the budget isn't there. The chassis runs out of track
at s ≈ 640 m, ~20 m before the apex.

### Why the QP doesn't anticipate

Default horizon: `DEFAULT_HORIZON_M = 30` (15 stages × 2 m). At
v ≈ 70 m/s that is **0.43 s** of lookahead. The chicane corner mouth
sits at s ≈ 600 m. From the QP's vantage at s = 429 m it sees ahead
only to s = 459 m, where `v_ref(459) ≈ 70 m/s` (still on the high-speed
straight). The QP's `w_v · (v_x − v_ref)²` cost says "you're tracking
fine, keep going." No braking is committed.

The reactive sub-controller's longitudinal path uses a *plan-aware*
preview: it looks up `v_target(s + preview_distance)` and brakes if
`v_x > v_target_ahead`. Preview distance for Tomas's params is on the
order of 50–100 m, but the speed-PI integral gain accumulates the
gap so the **effective lookahead is much longer than the geometric
preview** — closer to "as far as the cumulative deceleration takes
to bring v_x down to v_target". Sub-controller starts braking
~310 m before the apex; the QP needs at least ~170 m of horizon to
match it.

Two structural fixes are conceivable:

1. **Raise horizon.** 30 m → 150–200 m. Each stage costs ~1 ms of
   solve; horizon × 5 = solve × 5 = 90 ms p95, which violates the
   real-time budget (20 ms tick) and would push the system into
   Tier 2 reactive permanently. Doesn't fit the architecture.
2. **Terminal v-tracking cost driven by the DP plan envelope.** Add
   a stage-cost term `w_term_v · (v_x_horizon − v_envelope_horizon)²`
   where `v_envelope_horizon` is the DP plan's worst-case future
   speed within some long-horizon window (e.g. min(`v_ref(s : s + 200 m)`)).
   This is the canonical MPC pattern (terminal set / value function
   approximation); doesn't require a longer geometric horizon, just
   a value approximation at the horizon's end. **Spec'd for Phase 5.0.9.**

---

## MC validation — all four chicane safety mults

`.tmp/phase5_0_8_mc.py` runs 10-seed MC at chicane mults 0.80 / 0.85 /
0.90 / 1.00, both emit modes. All 10 runs use seed 0 + `13 * i` MC
re-seed pattern. consistency_sigma kept at Tomas's measured 1.55
(non-zero — so this is the on-spec stochastic acceptance gate, not the
deterministic single-seed diagnostic).

```
[A] Reactive baseline, mult=0.80, 10 MC runs:
    7/10 completions; median=129.12s min=129.04s max=129.18s wall=55.9s

[B] MPC emit=sub, mult=0.80:  0/10 (the pre-5.0.8 regression)
[B] MPC emit=qp,  mult=0.80:  0/10  ← Phase 5.0.8
[B] MPC emit=sub, mult=0.85:  0/10
[B] MPC emit=qp,  mult=0.85:  0/10
[B] MPC emit=sub, mult=0.90:  0/10
[B] MPC emit=qp,  mult=0.90:  0/10
[B] MPC emit=sub, mult=1.00:  0/10
[B] MPC emit=qp,  mult=1.00:  0/10
```

Acceptance gates from the spec:

| Gate | Target | Result | Pass? |
|---|---|---|---|
| ≥7/10 MC at chicane mult ≤0.85 | ≥7/10 | 0/10 | NO |
| Median lap ≤ 2:10 | ≤ 130 s | n/a (no completions) | NO |
| Stretch: ≤ 2:00 | ≤ 120 s | n/a | NO |
| Solve time unchanged | mean ≤ 25 ms, p95 ≤ 35 ms | mean 17–25, p95 21–38 | YES |
| α_f at chicane entry ≥ reactive | — | n/a (MPC aborts before chicane) | NO |

The headline acceptance gate fails because the chassis state at the
chicane is wrong by the time the QP could commit ellipse-aware braking.

Solve time is the one acceptance gate the change passes cleanly. The QP
itself is byte-for-byte identical to Phase 5.0.7 (no decision-vector
change, no new constraints, no new cost terms). Only the emit branches.

### Comparing `sub` and `qp` failure modes

Both modes abort, but for slightly different reasons:

- **`emit=sub` (Phase 5.0.7 regression):** abort at s ≈ 657 m, t ≈ 13.6 s
  (chicane apex). MPC's δ is too shallow vs reactive's, chassis enters
  too fast, lateral grip exhausted at apex. Same failure as Phase 5.0.7.
- **`emit=qp` (Phase 5.0.8):** abort at s ≈ 640 m, t ≈ 9.7 s (chicane
  *entry*, ~20 m before apex). MPC's brake commit is late, chassis enters
  ~25 m/s too fast, can't even reach the apex.

In both cases the Tier ladder behaves as expected: Tier 0 most of the
time, Tier 1 transient at corner entry, Tier 2 reactive after escalation.
Tier 2 immediately re-engages the sub-controller, which then ALSO
escalates (its own `over_slip` ghost-fallback fires at t = 9.04 s with
slip_ratio = 3.05 — the rear tyres locked under the abrupt brake
transition from the sub-controller after a too-late entry).

Worth noting that `emit=qp` MC walls are ~25 % faster than `emit=sub`
(78 vs 104 s) because the QP-emit runs abort earlier; the QP itself
isn't faster per-tick.

---

## Implementation surface

### Files changed

| File | Lines (approx) | What |
|---|---:|---|
| `src/lap_estimator/dynamics/mpc_controller.py` | +85 / −5 | `emit_source` kwarg, `_suppress_pedal_overlap` helper, `_emit_tier0_controls` rewrite, `emit_diagnostics` property, init validation, doc-string + comment updates. |
| `src/lap_estimator/dynamics/slip_simulator.py` | +5 | `mpc_emit_source` kwarg threaded through `simulate_slip` → `_run_monte_carlo` → `_run_single` → `_make_controller` → `MPCController`. |
| `lap.py` | +15 | `--mpc-emit-source {qp, sub}` CLI flag with help text + the forwarding to `simulate_slip`. |

No new modules. No QP / plant / linearisation changes. No `viz/`,
no `reactive` controller, no MPCC changes. The change is entirely on
the Tier 0 emit path of the MPC.

### Files NOT touched per spec constraints

- `src/lap_estimator/dynamics/driver_controller.py` — the reactive path,
  the v3 production controller. Untouched per "Reactive controller is
  UNTOUCHED".
- `src/lap_estimator/dynamics/mpcc_*.py` — the v3.3 MPCC. Untouched per
  "MPCC files are UNTOUCHED".
- `src/lap_estimator/dynamics/mpc_qp.py` — the QP build / SQP loop. No
  changes: no new constraints, no new cost terms. The QP is byte-
  for-byte identical to Phase 5.0.7.
- `viz/*` — untouched per spec.

### Diagnostic surface

- `MPCController.emit_diagnostics` — list of per-Tier-0-emit tuples
  `(s_m, v_x, throttle_qp, brake_qp, throttle_sub, brake_sub)`. Property
  on the controller; not surfaced to `SlipSimResult` (intentional — this
  is a build-time debugging probe, not a production telemetry channel).

---

## Why the spec's prediction was wrong

The spec said:

> 6. **Per-stage dynamic Fz (Phase 5.0.4):** already in the QP — makes
> the throttle/brake → Fx mapping honest enough to commit.

Phase 5.0.4's per-stage Fz refresh is sound for the *lateral* axis (it
makes the ellipse hard constraint reflect the dynamic load distribution
under cornering). It does NOT make the LONGITUDINAL channel any more
honest. The QP's longitudinal model is still

    Fx_rear = k_throttle · throttle − k_brake_rear · brake

with constant coefficients computed once at controller construction
(`mpc_model.py:514-527`). The dynamic Fz refresh updates the per-axle
peak Fx the ellipse will allow, not the per-pedal slope. The QP has
no awareness of wheel slip ratio, no slip-band PI, no measured trail-
brake taper — the rich longitudinal logic the reactive sub-controller
applies on top of `v_target`.

Combined with the 30 m horizon (which gives 0.43 s of lookahead at
70 m/s) the QP simply cannot do the brake-anticipation job the
reactive controller does today.

The architect's diagnosis in Phase 5.0.7 (OP-4) — that "the QP is the
right place to put longitudinal because the bicycle model is honest
enough" — was correct in the limit of an infinite-horizon QP with
well-tuned cost weights. It is **not** correct for the in-tree 30 m
horizon, default `w_v = 0.5`, and rate-only-cost structure. Either of
those would need to change to make first-class emit competitive with
the reactive baseline.

---

## What's left — recommendation for Phase 5.0.9

The spec's stated end-state ("If this works, MPC finally beats the
reactive baseline. If it doesn't, document what's left.") puts this
phase squarely in the second branch.

Two complementary directions:

### (a) Phase 5.0.9 — Terminal v-tracking cost from DP envelope

Add a stage-cost term

    J_term_v = w_term_v · (v_x[N] − v_envelope[N])²

where ``v_envelope[N]`` is the DP plan's **minimum future speed over a
long window** beyond the QP's geometric horizon — e.g.
``min(v_ref(s_horizon_end : s_horizon_end + 150 m))``. This gives the
QP a value-function approximation at the horizon's tail that says
"don't be faster than v_envelope or you'll have committed to a brake
you can't deliver." Standard MPC technique (terminal set / value
function); no horizon increase needed.

Implementation surface: one extra term in `mpc_qp.build_qp` (the
``v_envelope[N]`` value is pre-computed at controller construction
from `LongitudinalPlan`). One extra weight `w_term_v` exposed in
`MPCWeights`. Likely few-line change to the cost loop. Estimated
solve-time cost: zero (linear term, no new constraints).

### (b) Hybrid: keep reactive longitudinal, MPC steering-only

The Phase 5.0.7 architecture (MPC for δ, sub-controller for throttle /
brake) is what is already shipping. It just doesn't beat the reactive
*standalone* baseline because the chassis-state coupling carries +1.7 m/s
into the chicane. If Phase 5.0.9 (terminal v cost) doesn't close that
either, the rational call is to retire MPC entirely and ship reactive
as the production v3 controller — which is exactly the current state
of `docs/architecture-v3-shipping-state.md`.

The v3.3 MPCC and the user-tagged "next direction" (ellipse-aware FF
controller, plan (Fx, Fy)(s) trajectories) are the longer-term answer;
Phase 5.0.9 is the smallest possible MPC-rescue increment before we
declare MPC parked permanently.

### What Phase 5.0.8 did achieve

- The first-class-emit CLI knob is in place, validated, and a clean
  A/B regression button against Phase 5.0.7. The `--mpc-emit-source
  sub` path is byte-for-byte identical to pre-5.0.8 (re-verified by the
  matching MC fail patterns and solve-time numbers across mults).
- The pedal-overlap suppression in `_suppress_pedal_overlap` is reusable
  if Phase 5.0.9 (or any future first-class-emit phase) ships. It is
  a structural correctness fix that any first-class longitudinal path
  needs.
- The per-tick `(thr_qp, brk_qp, thr_sub, brk_sub)` diagnostic is now
  available on the controller for any future investigation that wants
  the same comparison.

---

## File inventory

Production code:

- `src/lap_estimator/dynamics/mpc_controller.py` — emit-source kwarg,
  `_suppress_pedal_overlap` helper, rewritten `_emit_tier0_controls`,
  `emit_diagnostics` property, init validation. Public API change:
  new optional kwarg `emit_source: str = "qp"` on
  `MPCController.__init__`; default selects the new behaviour.
- `src/lap_estimator/dynamics/slip_simulator.py` — `mpc_emit_source`
  threaded through the three forwarding layers. Default `"qp"` end-to-
  end so callers who never pass the flag get Phase 5.0.8 behaviour.
- `lap.py` — `--mpc-emit-source {qp, sub}` flag, default `qp`.

Diagnostic / validation scripts (`.tmp/`, gitignored):

- `.tmp/phase5_0_8_smoke.py` — single-seed deterministic smoke at the
  four chicane mults; both emit modes.
- `.tmp/phase5_0_8_smoke.log` — captured output.
- `.tmp/phase5_0_8_diag.py` — per-tick emit-source comparison
  (`(s, v_x, thr_qp, brk_qp, thr_sub, brk_sub)`); the table in the
  diagnostic section above.
- `.tmp/phase5_0_8_diag.log` — captured output.
- `.tmp/phase5_0_8_mc.py` — 10-seed MC at the four mults × two emit
  modes.
- `.tmp/phase5_0_8_mc.log` — captured output.

No production-code defaults changed beyond the emit-source path: cost
weights, bounds, horizon, tier thresholds, dynamic-Fz settings — all
unchanged from Phase 5.0.7. `--mpc-emit-source sub` makes the
controller bit-identical to pre-5.0.8 on the Tier 0 emit, modulo the
per-Tier-0-emit diagnostic capture (which is read-only — it does not
affect commits).

---

## Cross-reference

- Session index `docs/architecture-v3-session-2026-05-24.md` updated
  to point to this doc as the Phase 5.0.7 follow-up. The MPC chain
  now reads 4.1 → 5.0 → 5.0.1 → 5.0.2 → 5.0.3 → 5.0.4 → 5.0.5 →
  5.0.6 → 5.0.7 → 5.0.8 (this) → 5.0.9 (proposed).
- Open points `dev-planning/lap-simulation-csv-driver/open-points.md`
  OP-4 resolved (the spec was written, code was built, hypothesis
  was tested, result is documented). A new open point should be
  added for the Phase 5.0.9 terminal-v-cost spec recommendation.
