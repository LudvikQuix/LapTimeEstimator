# Phase 5.0.6 — Tier 1 brake-fix + soft-divergence threshold tuning

**Predecessor specs / docs:**
- Spec §23.2-5.0.3 (Tier 1 ellipse-saturation feedforward).
- Spec §23.2-5.0.4 (per-stage dynamic Fz).
- `docs/architecture-slip-model-phase5_0_3-v32-tier1.md`.
- `docs/architecture-slip-model-phase5_0_5-v32-tier1-bistability.md`.
- `docs/architecture-slip-model-phase5_0_5-v32-reactive-chatter.md`.

**Scope:** Controller-only tuning + a one-channel structural change to Tier 1's
output. No QP, plant, or DP-plan code modified. Driver JSON unchanged
(thresholds load from existing `control_params.mpc.tier1` block with new
defaults).

**Status:** Partial fix. Tier 1 cascade behaviour repaired; underlying
chicane-tracking deficit in MPC steering survives the fix and is documented
below as the next-phase target.

---

## What changed

Three changes in `src/lap_estimator/dynamics/mpc_controller.py` and one
diagnostic cleanup in `mpc_controller_tiers.py`. Total LoC delta ~20.

### 1. Soft-divergence thresholds raised

`Tier1Config` defaults (also propagated to the JSON-loader's per-field
fallbacks):

| Field | Was | Now | Rationale |
|---|---:|---:|---|
| `sqp_du_inf_threshold` | `0.01` | `0.25` | The Phase 5.0.3 default sat just above the natural noise floor of `||Δu||_∞` on clean ticks at corner-entry transients (observed range 0.011–0.015). Two consecutive natural transients fired Tier 1's 2-of-2 hysteresis, the saturation emit displaced the actuators well off the prior linearisation point, and the next QP's `du_inf` cascaded into the 0.5–3.7 regime within 1–2 ticks. 0.25 sits well above the clean-tick ceiling and well below the runaway-cascade floor. |
| `ellipse_violation_threshold` | `0.15` | `0.40` | Same diagnosis applied to the 4-stage post-solve rolled-trajectory ellipse-residual check. Natural pre-chicane transients reach 0.18–0.25 routinely; 0.15 was tripping on the transient itself. 0.40 keeps detection 4 sensitive to genuine ellipse blow-up (post-Tier-1 cascade values >0.6) while absorbing the natural-transient class. |

### 2. Tier 1 emit: only steering is saturated; longitudinal stays with the sub-controller

Previously `_emit_tier1_controls` ran the saturated per-axle `(Fx, Fy)`
through `_invert_pedals` and emitted the resulting `(throttle, brake)`
verbatim. Diagnostic instrumentation (TIER-DIAG run, `--inertia-zz 2400
--chicane-safety-mult 0.75`) logged the held actuator state at the moment
of Tier-2 escalation as `brake=1.000` — i.e. full lockup at 24-26 m/s
into a chicane. Reactive standalone at the same instant emits
`brake=0.5–0.8` (slip-band trail-braking with > 50 % lateral allocation).

The saturated inverse `_invert_pedals` is correct for what the ellipse
projection says, but the ellipse projection assumes the per-axle planned
direction has the chassis at full grip; for a transient soft-divergence
the planned direction carries enough longitudinal component that the
rear-axle `Fx` lands on the ellipse boundary, which inverse-maps to
`brake = 1.0`. Combined with corner steering this is grip-exhaustion in
one tick.

The fix: in `_emit_tier1_controls` use the reactive sub-controller's
`(throttle, brake)` directly and only emit the saturated steering. The
sub-controller's slip-band P-loop and measured trail-brake taper produce
the brake schedule the chicane needs; Tier 1's saturation adds value
only on the lateral channel where the analytical projection genuinely
beats Stanley's preview tracker at the friction limit.

```python
ctrl, new_delta, _, _ = emit_ellipse_saturation(...)
new_thr = float(sub_cmd.throttle)   # NEW: reactive sub-controller's
new_brk = float(sub_cmd.brake)      # NEW: trail-brake schedule
```

`_held_steer_rad` is still updated with `new_delta` so the next QP's `x0`
sees Tier 1's actual steering commit; `_held_throttle` / `_held_brake`
now track the sub-controller's reactive trail-brake — which is also what
the next QP would see at the actual chassis state (the sub-controller is
called every tick anyway).

### 3. Removed instrumented per-tick TIER-DIAG logging

The investigation logged every soft signal at WARNING level. Once the
root causes were identified the per-tick prints were removed (kept only
the once-per-controller-lifetime Tier-2 entry log that was always there).

---

## What is fixed

`util_p85` dropped from 1.47 → 1.02 on the `--inertia-zz 2400
--chicane-safety-mult 0.85` single-seed regression: the controller is no
longer driving the chassis over the friction-ellipse peak during Tier 1
episodes. The brake-at-handoff value is now in the reactive standalone's
range (0.5–0.7 instead of 1.0).

The Tier 1 episode count dropped 40 → 31 (per single-seed pre/post),
because fewer ticks need to escalate. Tier 0 fraction rose 76.2 % →
81.6 %. Tier 2 fraction stayed near 15 % (the structural fallback path).

The Phase 5.0.3 acceptance gates §11.55-5.0.3 A/B/C (Tier 0 ≥ 80 %,
Tier 1 ≤ 15 %, Tier 2 ≤ 5 %) now read A=passed (81.6 %), B=passed
(3.0 %), C=failed (15.4 %). Tier 2 is still the dominant fallback at
the chicane; threshold tuning alone cannot close C.

---

## What is NOT fixed

Sprint A chicane lap completion: **0/10 MC at 0.85, 0.80, 0.75**
(`--inertia-zz 2400`, default Tomas driver). Aborts consistently at s
≈ 654-657 m (chicane apex) regardless of `chicane_safety_mult`.

The reactive standalone completes Sprint A at 0.85 single-seed (2:06.10),
0.80 (2:09.20), and at 0.75 in 9/10 MC. **MPC is structurally worse at
this chicane than the reactive controller, independent of the planner-
side cap.**

### Root cause of the residual abort

Per-tick chassis-trace comparison at `--chicane-safety-mult 0.85`:

| t (s) | s (m) | v reactive (m/s) | v MPC (m/s) | Δv |
|---:|---:|---:|---:|---:|
| 10.6 | 601.8 | 26.15 | 27.06 | +0.91 |
| 11.4 | 621.6 / 623.1 | 22.45 | 23.90 | +1.45 |
| 12.0 | 635.3 | 19.95 | 21.35 | +1.40 |
| 12.6 | 645.9 | 17.29 | 18.80 | +1.51 |
| 13.2 | 655.1 / 653.6 | 14.85 | 16.23 | +1.38 |

MPC carries 1.3–1.5 m/s more speed than reactive through the entire
braking zone (5–7 %). The longitudinal sub-controller is identical
between the two builds; the difference is steering-driven:

- Reactive uses preview-based Stanley + Phase 4.3 slip-band coupling.
  At corner entry the front-axle slip-angle `α_f` ramps to a measurable
  fraction of `α_peak_front`, and the slip-band P-loop reduces the brake
  target accordingly *while* the preview tracker rolls steering in.
- MPC's QP solves for an optimal `δ_seq` over the 30 m / 0.6 s horizon.
  At chicane approach the QP under-rotates the front axle (`α_f` lower
  than reactive's at the same speed and yaw rate), and the longitudinal
  sub-controller — which reads the actual chassis `α_f` — relaxes the
  brake target. Net: MPC arrives at the chicane apex carrying excess
  speed. Stanley taking over via Tier 2 cannot recover.

The mechanism is a structural feedback through the longitudinal sub-
controller's slip-band coupling. The fix lives in one of two places:

1. **MPC plant linearisation refresh** — Phase 5.0.4 already refreshes
   per-stage `Fz` and per-stage Pacejka cornering stiffness, but the
   QP's `w_psi` weight (currently 20) under-prices the heading error
   that would drive a more aggressive front-axle slip command in the
   chicane brake zone. A re-tune of `w_psi`, `w_lat`, and (critically)
   the slip-budget soft constraint `w_slip` should narrow the steering
   gap.
2. **Replace the reactive sub-controller with an MPC longitudinal
   channel** — the spec's §23.2.4 deferred decision was to keep the
   reactive sub-controller because the QP's `k_throttle · throttle`
   approximation was too coarse for honest plan tracking. With Phase
   5.0.4's per-stage dynamic Fz this is now revisitable; a slip-aware
   MPC longitudinal channel would eliminate the steering-side feedback
   asymmetry.

Option 1 is the smaller-surface change and should be tried first as a
Phase 5.0.7 spec.

---

## File inventory

- `src/lap_estimator/dynamics/mpc_controller.py` — three changes:
  - `Tier1Config.ellipse_violation_threshold` default `0.15` → `0.40`
    (+ comment).
  - `Tier1Config.sqp_du_inf_threshold` default `0.01` → `0.25`
    (+ comment).
  - JSON loader (`from_block`) defaults mirrored.
  - `_emit_tier1_controls` rewires throttle/brake to `sub_cmd` and
    only writes the saturated steering to `_held_steer_rad`; updated
    docstring + comment.
- `src/lap_estimator/dynamics/mpc_controller_tiers.py` — diagnostic
  `stats["tier_reason"]` field added during investigation, then removed.
  No functional change versus pre-investigation.

No spec change, no driver JSON change, no CLI change, no plant change,
no QP change, no plan change.

---

## Validation

Single-seed MPC, `--inertia-zz 2400 --chicane-safety-mult 0.85`:

```
util_p85: 1.467 → 1.022 (within tyre peak)
Tier 0: 76.2% → 81.6%
Tier 1: 2.9% → 3.0% (episodes 40 → 31)
Tier 2: 20.9% → 15.4%
Held-brake at Tier-2 entry: 1.000 → 0.561 (now in trail-brake range)
Lap result: abort at s=655 in both pre and post (root cause is
            structural, not Tier-1-cascade-driven).
```

10-seed MC at 0.85, 0.80, 0.75: 0/10 completions. The structural deficit
(MPC carrying 1.3–1.5 m/s excess into chicane apex) is the dominant
unblocked failure mode.

Recommended next-step spec: Phase 5.0.7 — `w_psi` / `w_lat` / `w_slip`
re-tune against a chicane-specific subset of the Sprint A geometry. If
the re-tune doesn't close the speed-carry gap to within 0.3 m/s of
reactive at chicane apex, escalate to Option 2 (replace the reactive
sub-controller).
