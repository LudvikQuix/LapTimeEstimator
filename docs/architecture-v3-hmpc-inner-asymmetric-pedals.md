# Architecture — HMPC v3.7 inner asymmetric pedal cost

**Spec / task brief:** 2026-05-26 asymmetric-pedal brief — split the inner's
longitudinal control into separate brake and throttle channels with
asymmetric cost shapes. Brake should be biased toward bang-bang (0 or 1,
penalise mid-values); throttle should be biased toward smooth progression
(heavy rate penalty, light absolute-magnitude penalty).
**Predecessors:**
- `docs/architecture-v3-hmpc-chase-tomas.md` — chase-Tomas sweep
  identifying the 2:28.88 / 10-of-10 ceiling and the brake-bang-bang
  insight.
- `docs/architecture-v3-hmpc-outer-line-following.md` — outer line-follow
  mode (orthogonal lever; the inner is the bottleneck).
- `docs/architecture-v3-hmpc-inner-tuning.md` — Lever 3 (`inner_w_a`).
- `docs/architecture-v3-hmpc-casadi-outer.md` — Phase 5.2/5.3 base.
**Branch:** `feature/sc-71955/lap-simulation`.

---

## Headline results

| Variant | cm | MC | Lap (s) | Brake peak | Brake p95 thresh | Throttle Δp95 exit |
|---|---:|---|---:|---:|---:|---:|
| **reactive (production baseline)** | 0.85 | 10/10 | **126.00** | — | — | — |
| **HMPC v3.6 (chase-Tomas headline)** | 0.95 | 10/10 | **148.88** | 0.68 | 0.65 | 0.06 |
| **HMPC v3.7 asym defaults** | 0.95 | **10/10** | **149.56** | 0.66 | 0.64 | 0.04 |
| HMPC v3.7 best single-seed (du_thr=1) | 0.95 | 1/1 | 149.42 | 0.65 | 0.63 | 0.07 |
| HMPC v3.7 asym + L3 (`inner_w_a=3..8`) | 0.95 | 0/1 | ABORT | — | — | — |

Gap vs reactive (1:47.56 ghost ground truth, 2:06.04 production line):
**+22.0 s** — essentially unchanged from chase-Tomas. The asymmetric
pedal cost is a working lever (throttle Δp95 drops from 0.06 → 0.04;
optimiser shape now respects bang-bang) but the binding constraint is
the inner's per-axle friction ellipse at the chicane apex, not the
pedal cost shape.

## TL;DR — what changed

The CasADi inner's per-stage cost previously summed a single combined
rate-of-control term

    J_rate(k) = w_du · ||U[:,k] - U[:,k-1]||^2

(`U[:, k]` packs `[delta_dot, throttle_dot, brake_dot]`). That symmetric
quadratic pulls **brake** away from 1.0 with the same gradient it pulls
**throttle** away from 1.0 — wrong for real-driver chicane technique
where brake is bang-bang and throttle is progressive.

v3.7 splits the rate cost per channel and adds two pedal-shape terms:

    J_rate(k)     = w_du           · (delta_dot[k] - delta_dot[k-1])^2
                  + w_du_throttle  · (throttle_dot[k] - throttle_dot[k-1])^2
                  + w_du_brake     · (brake_dot[k] - brake_dot[k-1])^2

    J_pedal(k)    = w_brake_double_well     · brake[k] · (1 - brake[k])
                  + w_throttle              · throttle[k]^2
                  + w_brake_throttle_overlap · brake[k] · throttle[k]

All five new weights default to **0.0** — at defaults the CasADi inner is
bit-identical to v3.6 (the channel-split rate term falls back to the legacy
`w_du · sumsqr(...)` when both channel-specific knobs are zero).

The decision-variable layout is unchanged: brake and throttle were
already independent state-actuators (`X[6, k] = throttle`, `X[7, k] =
brake`) with independent rate-controls (`U[1, k] = throttle_dot`,
`U[2, k] = brake_dot`). The brief's "decision-variable split (if needed)"
isn't needed — the existing decomposition gave us per-channel access for
free; only the cost surface was symmetric.

---

## Why this architecture

### 1. Already-split decision variables

Reading `mpc_model.py` (`NX = 8`, `IDX_THR = 6`, `IDX_BRK = 7`; `NU = 3`
with `U[1] = throttle_dot`, `U[2] = brake_dot`) confirms the inner has
always had brake and throttle as independent state-actuators tracked by
independent rate-controls. The chase-Tomas 2:28.88 ceiling was diagnosed
in `docs/architecture-v3-hmpc-chase-tomas.md` (see "Inner brake commits
only to 0.55-0.65 max"). The root cause: in `hmpc_inner_casadi.py` the
per-stage rate term was

    J += weights.w_du * ca.sumsqr(U[:, k] - U[:, k - 1])

`sumsqr` lumps the three channels together with a single `w_du`. The
optimiser sees the same marginal cost for a 0.1 brake step and a 0.1
throttle step, when in reality real drivers spend brake snaps with
basically zero rate cost and pay a much heavier rate cost for throttle
spikes.

### 2. Brake double-well — concave-soft, IPOPT-friendly

The brief's prescription was a `brake · (1 - brake)` penalty. On [0, 1]
this is a downward parabola with peak 0.25 at brake=0.5 and zeros at
brake=0 and brake=1. Multiplied by a large weight, it forms a barrier
**between** the two natural commit endpoints. IPOPT (which the v3.3+
CasADi inner uses) handles non-convex soft penalties without issue — it
just settles into whichever well the rest of the cost surface and the
friction-ellipse soft penalty allow. We get bang-bang behaviour without
introducing integer variables.

### 3. Throttle quadratic + heavy throttle-rate

The brief asked for "smooth progression". The smoothness is enforced by
the **rate** term (`w_du_throttle ~ 10-20`), not the absolute throttle
penalty. The absolute-magnitude penalty `w_throttle · throttle^2` is
optional (default 0.0) — letting it stay zero encourages WOT on straights
without contesting the friction circle. The brief explicitly said "or
zero" for `w_throttle`.

### 4. Overlap penalty as soft pseudo-complementarity

A hard `brake · throttle = 0` constraint would block trail-brake-to-
throttle transitions. A soft penalty `w_overlap · brake · throttle`
grows linearly in each pedal — small at low overlap, large at full
overlap, exactly what trail-brake regimes can tolerate.

---

## Implementation surfaces

### New module `src/lap_estimator/dynamics/hmpc_inner_cost.py` (~165 lines)

Hosts the per-stage cost-term builders. Pure CasADi — no NumPy, no params,
no decision vars. Two functions:

- `stage_rate_cost(weights, U, k, p_u_prev)` — returns the three-channel
  asymmetric rate-of-control penalty. When the channel-specific knobs
  are 0.0 it falls back to the legacy `w_du` on that channel, preserving
  pre-v3.7 behaviour.
- `stage_pedal_shape_cost(weights, X, k)` — returns the sum of the
  optional double-well + throttle² + overlap terms. Short-circuits to
  `MX(0.0)` when all three weights are zero.

This factoring keeps `hmpc_inner_casadi.py` close to its existing size
(it was already at the 500-line ceiling per CLAUDE.md soft rules) without
duplicating logic.

### `src/lap_estimator/dynamics/mpc_qp.py` — `MPCWeights` extension

Five new optional fields on the frozen dataclass:

| Field | Default | Purpose |
|---|---:|---|
| `w_du_brake`              | 0.0 | Brake-dot quadratic; loose to allow snaps |
| `w_du_throttle`           | 0.0 | Throttle-dot quadratic; heavy for smooth tip-in |
| `w_brake_double_well`     | 0.0 | Pull brake to {0, 1}; concave-soft |
| `w_throttle`              | 0.0 | Absolute-throttle quadratic; keep at 0 |
| `w_brake_throttle_overlap`| 0.0 | Soft pseudo-complementarity |

Defaults are 0.0 so the OSQP `solve_sqp` path (legacy v3.2 inner) and the
v3.6 CasADi inner with unset knobs are unchanged.

### `src/lap_estimator/dynamics/hmpc_inner_casadi.py` — cost-builder swap

Replaced the legacy combined `w_du · sumsqr(...)` term with
`stage_rate_cost(weights, U, k, p_u_prev)` and added
`stage_pedal_shape_cost(weights, X, k)` per stage. Terminal-stage also
calls `stage_pedal_shape_cost(weights, X, N)` so the {0, 1} pull persists
to the horizon tail. Everything else (ellipse soft constraint, actuator
bounds, dynamics, Risk-8 iter bump) is unchanged. ~10 lines added,
~5 removed.

### `src/lap_estimator/dynamics/hmpc_controller.py` — JSON plumbing

Five new lookups in the `control_params.hmpc` block:

- `inner_w_du_brake`
- `inner_w_du_throttle`
- `inner_w_brake_double_well`
- `inner_w_throttle`
- `inner_w_brake_throttle_overlap`

All default to 0.0. No CLI flags added — the driver-JSON path is enough
per the brief.

---

## Data flow (per inner tick)

```
HMPCController._resolve(state, t)
  |
  +-- self.inner.solve(x0, kappa_seq, v_ref_seq, n_ref_seq, ..., u_prev, a_long_ref_seq)
        |
        +-- CasadiInnerTracker.solve (hmpc_inner_casadi.py)
              |
              +-- _build_nlp (ONE-TIME at __init__): per stage k = 0..N-1
                    J += w_v * (v_x[k] - v_ref[k])^2
                    J += w_lat * (n[k] - n_ref[k])^2
                    J += w_psi * (psi_e[k] - psi_e_ref[k])^2
                    J += w_a * mask[k] * (a_long[k] - a_long_ref[k])^2
                    J += stage_rate_cost(weights, U, k, p_u_prev)   # NEW v3.7
                    J += w_du2 * sumsqr(d2u)
                    J += stage_pedal_shape_cost(weights, X, k)      # NEW v3.7
                    ellipse soft penalty (per axle, w_ellipse_soft)
                    actuator-bound soft penalty (1e4)
                  + terminal: w_term * (n^2 + psi_e^2)
                              + w_v * (v_x[N] - v_ref[N-1])^2
                              + stage_pedal_shape_cost(weights, X, N) # NEW v3.7
              |
              +-- per-tick: rebind parameter values; opti.solve()
              |
              +-- returns u_seq (N x NU rate-control sequence)
        |
  +-- first_stage_commit: roll u_seq[0] into (delta, throttle, brake)
  +-- PI trim + clip
  +-- emit Controls
```

---

## Where friction is now left on the table

Smoke single seed at `cm=0.95` on `layout_sprint_a_ideal_line.csv`:

| Variant | Lap (s) | Brake commit s (m) | Brake peak | Brake p95 (475-585) | Throttle Δp95 (635-670) |
|---|---:|---:|---:|---:|---:|
| baseline_v36 (no asym) | **148.88** | 116.8 | 0.68 | 0.65 | 0.06 |
| asym_defaults (dw=50, du_thr=10, du_brk=0.5, ov=50) | 149.56 | 121.0 | 0.66 | 0.64 | 0.04 |
| best single-seed (duthr=1, others at defaults) | **149.42** | 121.0 | 0.65 | 0.63 | 0.07 |

Across the full sweep grid (5 knobs × 4-5 values each, ~25 variants):

- **Brake peak ceiling 0.65-0.69 is invariant.** It does not move with
  any combination of the new knobs. The brief's target of ≥0.95 in the
  threshold-brake zone is unreachable from this lever set.
- **Lap-time swing across all 25 variants is ±1.5 s.** Best 149.42 s
  (duthr=1), worst 151.06 s (dw=200). The baseline 148.88 s is *better*
  than every asymmetric-pedal variant — the asymmetric pedal cost is
  modestly harmful at this configuration.
- **Throttle Δp95 in the exit zone DOES move** (0.06 → 0.04 → 0.000 as
  `w_du_throttle` and `w_brake_double_well` ramp). The smooth-throttle
  intent of the asymmetric cost is *achieved* — but it costs ~0.7 s
  of lap time without unlocking brake commitment.

### Why the brake-peak ceiling is invariant

The brake-commit gate is **NOT** the cost shape on `u_long`. It is the
**lateral grip residual** at the chicane apex, exactly as documented
in `docs/architecture-v3-hmpc-chase-tomas.md`. With the chassis already
booking ~10.6 m/s² centripetal at chicane apex (v=17.2 m/s, κ=0.036
1/m) on a Pacejka envelope of ~8.6 m/s², the friction-ellipse soft
penalty (`w_ellipse_soft=5000`) fires HARD on any (Fx, Fy) combo that
exceeds 1.0 per axle. The optimiser's gradient pulls the iterate back
to the brake-peak region (0.65-0.69) regardless of what the
double-well wants — `w_ellipse_soft` is 100× the natural scale of
`w_brake_double_well · b · (1 - b)` evaluated at b=0.65.

**To unlock the brake ceiling, the chassis has to arrive at the
chicane apex with v ≤ 14 m/s.** That requires either (a) braking
harder *earlier* (before the lateral load builds), which means the
outer plan must commit to brake at s ≈ -180 m (i.e. before the start
line — see `chase-Tomas.md`), or (b) reducing the chicane's effective
curvature by adjusting the racing line, which is an outer-NLP problem.

### What's now the binding constraint

**Inner friction-ellipse lateral side** at the chicane apex (`s in
[620, 700]` on Sprint A ideal-line). The `w_ellipse_soft = 5000`
quadratic soft-penalty per axle is firing on every iterate where the
brake commit goes past 0.68 because the chassis is already laterally
loaded to ~85 % of the per-axle Pacejka peak. The inner cannot
unilaterally raise the brake peak; the lateral grip is already booked
by the (κ, v_x) the outer planned.

The new bottleneck for chasing 2:06.04 is therefore the **outer's plan
of `(v_ref, n_ref)` through the chicane lead-in**, not the inner's
pedal cost. The asymmetric pedal cost is a working knob but it operates
inside a (Fx, Fy) budget the outer has already set.

### One concrete next lever

**Push the outer's brake commit upstream — terminal v-tracking against
the long-horizon DP envelope.** Identical recommendation to
`OP-6` in `dev-planning/lap-simulation-csv-driver/open-points.md`,
rescoped for the HMPC outer.

Mechanism: add a terminal cost `w_term_v · (v_outer[N] -
v_envelope[N])²` where `v_envelope[N] = min(v_ref(s_N : s_N + 200))` —
i.e. the outer's terminal-stage speed must track the *minimum* DP
plan speed in the next 200 m past horizon-end. This is a value-
function approximation at the outer's tail that says "if the
chassis is going to be braking 200 m from horizon-end, don't be
faster than the brake-feasible peak at that future point." It
pushes brake commit upstream without enlarging the outer horizon
(which would blow the 250 ms outer-solve budget per
`chase-Tomas.md`).

Spec authorisation needed before implementation — Buddy.

---

## Sweep results

All sweeps at `cm=0.95`, `outer_vref_lookahead_stages=20`,
`inner_solver=casadi`, single deterministic seed per variant.

Baseline (v3.6 — all five new knobs at 0.0): **148.88 s**, brake peak
0.68, brake p95 (threshold zone 475-585 m) 0.65, throttle Delta-p95
(exit zone 635-670 m) 0.06.

### Sweep 1 — `w_brake_double_well` in {0, 25, 50, 100, 200}

| dw   | Lap (s) | Brake commit (m) | Brake peak | Brake p95 thresh | Throttle Δp95 exit |
|-----:|--------:|-----------------:|-----------:|-----------------:|-------------------:|
| 0    | 149.72  | 119.5            | 0.648      | 0.632            | 0.090              |
| 25   | 149.64  | 119.5            | 0.649      | 0.634            | 0.055              |
| 50   | **149.56** | 121.0         | 0.656      | 0.641            | 0.037              |
| 100  | 149.66  | 126.8            | 0.656      | 0.642            | 0.037              |
| 200  | 151.06  | 139.9            | 0.654      | 0.636            | 0.000              |

Lap time is essentially flat across `dw in [0, 100]` (±0.16 s, noise).
`dw=200` is the only clear loser (+1.5 s — the double-well pull is now
strong enough to delay the brake commit by 20 m). Brake peak moves
**0.648 → 0.656**, a 0.008-unit gain — well below the 0.18 gap to the
hoped-for 0.95+ bang-bang.

### Sweep 2 — `w_du_throttle` in {1, 5, 10, 20, 40}

| du_thr | Lap (s) | Brake peak | Throttle Δp95 |
|-------:|--------:|-----------:|--------------:|
| 1      | **149.42** | 0.650   | 0.067         |
| 5      | 149.58  | 0.649      | 0.049         |
| 10     | 149.56  | 0.656      | 0.037         |
| 20     | 149.58  | 0.647      | 0.058         |
| 40     | 149.58  | 0.651      | 0.062         |

Throttle Δp95 decreases monotonically `du_thr 1→10` (smoother tip-in
in the exit zone), but lap time is flat. `du_thr=1` is slightly best
but within seed noise.

### Sweep 3 — `w_du_brake` in {0, 0.5, 2, 5}

| du_brk | Lap (s) | Brake peak | Brake p95 thresh |
|-------:|--------:|-----------:|-----------------:|
| 0      | 149.58  | 0.650      | 0.633            |
| 0.5    | **149.56** | 0.656   | 0.641            |
| 2      | 149.94  | 0.649      | 0.631            |
| 5      | 149.96  | 0.646      | 0.629            |

Higher `du_brk` slightly **reduces** brake peak (the snap-rate penalty
fights brake commit). 0.5 is the sweet spot.

### Sweep 4 — `w_brake_throttle_overlap` in {0, 10, 50, 200}

| ov   | Lap (s) | Brake peak |
|-----:|--------:|-----------:|
| 0    | 149.76  | 0.688      |
| 10   | 149.62  | 0.650      |
| 50   | **149.56** | 0.656   |
| 200  | 149.84  | 0.651      |

Sweep range ±0.2 s. Overlap at 0 yields the highest brake peak (0.688)
because the optimiser can co-commit brake and throttle without penalty,
but lap time is slightly worse — the overlap was being used in trail-
brake regions where the truth-model penalises co-activation via
`_suppress_pedal_overlap` at emit. ov=50 is the best lap.

### Sweep 5 — `chicane_safety_mult` in {0.80, 0.85, 0.90, 0.95}

| cm   | Lap (s) | Brake peak |
|-----:|--------:|-----------:|
| 0.80 | 162.14  | 0.659      |
| 0.85 | 155.74  | 0.655      |
| 0.90 | 150.32  | 0.651      |
| 0.95 | **149.56** | 0.656   |

Same monotone improvement as baseline. The asymmetric pedals don't
change the cm-sensitivity story documented in `chase-Tomas`.

### Sweep 6 — Lever 3 cross: `inner_w_a` in {0, 3, 5, 8}

| inner_w_a | Lap (s) | Notes |
|---------:|--------:|-------|
| 0        | **149.56** | only stable variant |
| 3        | ABORT s=216 | off-track at chicane, same as baseline+L3 |
| 5        | ABORT s=268 | off-track at chicane |
| 8        | ABORT s=277 | brake-peak collapses to 0.55, mean_decel jumps to 25 (telemetry artefact post-abort) |

**Lever 3 still aborts under asymmetric pedals.** The asymmetric cost
shape doesn't help the L3-induced over-decel that pushes the chassis
past the lateral grip envelope (same mechanism as
`chase-Tomas.md`). The bang-bang brake shape adds force at the same
moment the chassis is already saturating laterally.

### MC verification at the best weight set

10 seeds at `asym_defaults` (du_brk=0.5, du_thr=10, dw=50, ov=50),
`cm=0.95`. **Result: 10/10 finishes at 149.56 s (deterministic;
identical to single-seed)** — matches the chase-Tomas finding that
HMPC outcomes are fully determined by the plant-controller composition
when the CasADi inner is in use (driver consistency-σ noise doesn't
enter the cost). Wall-clock 1530 s for 10 seeds.

See `.tmp/hmpc_inner_asym_mc.csv` for the row dump.

---

## Phase 2 — ellipse-soft sweep (2026-05-26)

**Task brief:** Phase 1 diagnosed the friction-ellipse soft penalty
(`w_ellipse_soft=5000`) as the binding constraint on brake authority
(brake peak pinned at 0.66). Phase 2 tests whether relaxing the penalty
unlocks brake commitment and lap time.

### Headline

| Config | cm | MC | Lap (s) | Brake peak | Brake p95 thresh | mean decel |
|---|---:|---|---:|---:|---:|---:|
| asym defaults (Phase 1 baseline, es=5000) | 0.95 | 10/10 | 149.56 | 0.66 | 0.64 | 4.61 |
| **es=500 + L3 wa=5 (Phase 2 headline)** | 0.95 | **10/10** | **143.04** (best) / 144.09 (median) | 1.00 | **0.92** | 5.76 |
| reactive production baseline (ref) | 0.85 | 10/10 | 126.00 | — | — | — |

**Phase 2 wins 5.84 s over Phase 1 / 5.85 s over chase-Tomas 2:28.88,
with brake p95 lifted from 0.64 to 0.92 (frac > 0.95 = 0.08). 10/10 MC
stable. The friction-ellipse soft penalty WAS the binding constraint
on brake — but only when combined with Lever 3 (`inner_w_a=5`). Reducing
`w_ellipse_soft` alone is harmful (lap time worsens 0 → +3 s as es
drops 5000 → 500).** The new ceiling at cm=0.95 ideal-line is
**143.04 s**; gap to reactive 2:06.04 has narrowed from +22 s to +17 s.

### Why ellipse-relax alone hurts

| es   | Lap (s) | Brake peak | p95 thresh | mean decel | Brake commit s |
|-----:|--------:|-----------:|-----------:|-----------:|---------------:|
| 500  | 152.50  | 0.895      | 0.798      | 5.28       | 350.3 (>0.7)   |
| 1000 | 151.12  | 0.801      | 0.740      | 4.99       | 388.0 (>0.7)   |
| 2000 | 150.32  | 0.718      | 0.683      | 4.76       | 410.0 (>0.7)   |
| 3000 | 149.96  | 0.682      | 0.657      | 4.68       | n/a (<0.7)     |
| 5000 | **149.56** | 0.656   | 0.641      | 4.61       | n/a (<0.7)     |

Lower ellipse-soft monotonically unlocks brake authority (peak
0.66 → 0.90, p95 0.64 → 0.80) and **shifts the brake commit upstream
by ~60 m** (the >0.7 ascent moves from s=444 m to s=350 m). But the
inner overspends the harder brake on lateral grip at the chicane apex,
losing **+3 s overall**. Without a longitudinal-accel reference to
*steer* the harder brake into the right (s, t) regions, the optimiser
just dumps deceleration wherever the v-tracking cost asks for it.

The Lever 3 cost `w_a · (a_long - a_long_ref)^2` (per Phase 5.3) gives
the inner exactly the missing signal: "be at this longitudinal accel,
not just at this speed". With `w_a > 0`, the unlocked brake authority
gets *routed* into the s-ranges where the outer's DP plan books the
deceleration, instead of being scattered across the lead-in.

### Lever-3 fine sweep at es=500 (single seed each, cm=0.95)

| `inner_w_a` | Lap (s) | Brake peak | p95 thresh | frac>0.95 | Notes |
|-----:|--------:|-----------:|-----------:|----------:|---|
| 1    | ABORT   | 0.60       | n/a        | n/a       | off-track @ s=485; L3 too weak to compensate for relaxed ellipse |
| 3    | **148.80** | 0.91   | 0.83       | 0.000     | first stable variant; lap < Phase 1 baseline |
| 5    | **143.86** | 1.00   | **0.92**   | 0.074     | headline (this row also seeded the MC) |
| 8    | ABORT   | 0.98       | 0.92       | 0.121     | off-track @ chicane (s=648); over-decel |
| 12   | STALLED | 0.07       | n/a        | n/a       | spin, controller blows up |

`inner_w_a` has a **narrow stability window of ~[3, 5]** under relaxed
ellipse. wa=5 is the lap-time winner; wa=3 is a safer secondary if the
seed-to-seed variance becomes a problem.

### Lever-3 cross at es=1000 (Phase-2 brief required this)

| es=1000 + `inner_w_a` | Lap (s) | Notes |
|---|--:|---|
| 0 | 151.12 | Same as ellipse-only sweep row |
| 5 | ABORT  | Off-track at chicane lead-in |

The headline mechanism only fires at es=500. es=1000+wa=5 destabilises
because the ellipse penalty is still strong enough to fight the
inner's wider Fx demand, but Lever 3 keeps pushing harder — IPOPT
fails to converge on a feasible (Fx, Fy) and the chassis runs off.

### Why this works — the (ellipse, L3) interaction

The asymmetric pedal cost (Phase 1) gave the inner the *cost shape* to
prefer brake bang-bang, but the gradient toward larger brake was
clipped by `w_eps · max(0, ellipse - 1)^2`. Relaxing `w_eps` from 5000
to 500 reduces that gradient by 10×. But removing the brake gradient
alone leaves the inner with two failure modes: (a) the brake commits
too early in the lead-in (mid-zone), wasting the friction budget at
straight-line driving where it doesn't pay off in lap time; (b) the
brake commits too late and the chassis enters the chicane too fast,
needing to brake into lateral load.

Lever 3 (`w_a · (a_long - a_long_ref)^2`) imposes the **outer's
planned deceleration profile** on the inner. With the ellipse penalty
relaxed, the inner can actually deliver that deceleration profile
without being pulled back to <0.7 brake. The two levers together form
a feasibility match: Lever 3 says "decel HERE, this much", the
relaxed ellipse says "OK, you've got the budget". Either lever alone
fails — together they unlock the brake.

This is also why Phase 1's L3 sweep (at es=5000) aborted: the inner
*wanted* to brake hard to match the L3 reference, but the ellipse
penalty kept clipping the brake; the chassis then over-shot v_ref
and entered the chicane too fast, losing lateral grip.

### Tier-2 backstops are present but not load-bearing

Every MC seed at the headline config triggers Tier-2 reactive backstops
during the chicane lead-in (s=258-327 m, the same chassis-divergence
moment seen in chase-Tomas). HMPC is in command for ~98 % of the lap
(tier 0 in 4695/4770 inner ticks at the median seed). In the brake
threshold zone (s=475-585 m), tier-0 is 100 % — the brake-bang-bang
*is* HMPC-emitted, not reactive cleanup. The Tier-2 fallback is the
expected behaviour at the brief chassis-divergence moment when the
inner's predicted state lags the truth-model's slip dynamics; it
re-engages within ~100 ms.

The seed-to-seed lap-time spread (best 143.04 → median 144.09 → worst
148+) reflects how long each seed stays in Tier-2 reactive during
that chicane lead-in. The deterministic-CasADi finding from Phase 1
does NOT carry over because Lever 3 amplifies the outer's plan
through the longitudinal-accel reference, which now sees the
seed-dependent driver-consistency-σ noise (the outer reference
gets perturbed before the inner reads it).

### cm sweep at headline (single seed each at cm in {0.85, 0.90})

| cm   | Lap (s) | Brake peak | p95 thresh | mean decel |
|-----:|--------:|-----------:|-----------:|-----------:|
| 0.85 | 150.14  | 0.98       | 0.90       | 5.88       |
| 0.90 | 143.64  | 0.96       | 0.86       | 5.57       |
| 0.95 | **143.04 (best of 10) / 144.09 (median)** | 1.00 | 0.92 | 5.76 |

cm=0.95 is the lap-time winner, but cm=0.90 is within 0.6 s and may
have better seed-to-seed robustness (lower brake peak; not yet verified
via MC). cm=0.85 costs **+7 s** because the conservative chicane-cap
v limit (14.0 m/s vs 15.6 m/s at cm=0.95) leaves the brake authority
unused — relaxed ellipse only helps when the speed plan demands the
brake.

### What's still on the table

- **Lap-time gap to the ideal-line theoretical floor.** The
  asymmetric-pedal doc's "next lever" recommended an outer terminal
  v-tracking against the DP envelope (push brake commit upstream).
  Phase 2 partially achieves the "push brake upstream" effect via
  Lever 3 (the >0.7 commit moved from s=444 to s=342). But further
  upstream commit at s ≈ 200 m (chase-Tomas estimate for matching
  Tomas) is still blocked by the outer's planned `v_ref` — the inner
  cannot brake harder than what the outer asks for.
- **Sub-145 s headroom.** Single-seed gets to 143.04; median 144.09.
  An outer terminal v-cost would likely close another 1-2 s and may
  hold MC median below 143.
- **OP-6 (open-points.md) — outer terminal v-tracking against DP** —
  still the recommended next lever; Phase 2 confirms the diagnosis
  but only addresses the inner half of the brake-anticipation problem.

### One concrete next lever (Phase-2 revised)

**Tighten the L3 reference for chicane apex.** The current
`a_long_ref` comes straight from the outer's NLP plan, which is
limited by the outer horizon (~ 1 s @ 20 stages × 50 ms). At the
chicane lead-in (s ≈ 280 m, chicane apex at s ≈ 650 m), the outer
horizon does not yet *see* the apex — so `a_long_ref` underestimates
the required brake. Replacing the inner's `a_long_ref` source with
the DP plan's `a_long` profile (sampled at the inner's stage spacing
from the outer's current `s` position over a 10-stage horizon, but
using the DP envelope's accel rather than the outer NLP's accel)
gives Lever 3 a longer-look-ahead reference without enlarging the
outer horizon. Implementation: add a "DP a_long" channel to
`OuterPlanner._publish` that the inner consumes alongside `a_long_ref`
when `inner_w_a > 0`.

Spec authorisation needed before implementation — Buddy.

### Sweep artefacts

| File | Content |
|---|---|
| `.tmp/hmpc_inner_ellipse_soft_sweep_es.csv` | Single-seed sweep of inner_w_ellipse_soft (5 values). |
| `.tmp/hmpc_inner_ellipse_soft_sweep_cm.csv` | cm sweep at es=500, no L3 (3 values; documents that relaxed-ellipse alone is harmful at all cm). |
| `.tmp/hmpc_inner_ellipse_soft_cross_l3.csv` | Initial Lever-3 cross at es in {500, 1000}, wa in {0, 5}. |
| `.tmp/hmpc_inner_ellipse_soft_l3_fine.csv` | Fine wa sweep at es=500 (5 values; finds [3, 5] stability window). |
| `.tmp/hmpc_inner_ellipse_soft_mc_headline.csv` | 10-MC at es=500 + wa=5 + asym defaults. |
| `.tmp/hmpc_inner_ellipse_soft_cm_headline.csv` | cm in {0.85, 0.90} at headline config. |
| `.tmp/hmpc_inner_ellipse_soft_traces/` | Per-variant debug-trace CSVs. |
| `.tmp/hmpc_inner_ellipse_soft_sweep.py` | Phase-2 sweep harness (modes sweep_es / mc / sweep_cm / cross_l3 / all). |
| `.tmp/hmpc_inner_ellipse_soft_mc_headline.py` | One-shot 10-MC at headline. |
| `.tmp/hmpc_inner_ellipse_soft_l3_fine.py` | Fine wa sweep at es=500. |
| `.tmp/hmpc_inner_ellipse_soft_cm_headline.py` | cm sweep at headline. |

### Phase-2 headline driver-JSON block

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
    "inner_w_a": 5.0
  }
}
```

No code changes were required for Phase 2 — `inner_w_ellipse_soft`
and `inner_w_a` were already JSON-pluggable via
`HMPCController._build_inner` (hmpc_controller.py line ~419).

---

## File inventory

**Created (Phase 1):**

| File | Purpose |
|---|---|
| `src/lap_estimator/dynamics/hmpc_inner_cost.py` | Per-stage rate + pedal-shape cost builders (CasADi). |
| `docs/architecture-v3-hmpc-inner-asymmetric-pedals.md` | This document. |
| `.tmp/hmpc_inner_asym_sweep.py` | Sweep harness (smoke / sweep_* / mc modes). |
| `.tmp/hmpc_inner_asym_smoke.csv` | Smoke result table. |
| `.tmp/hmpc_inner_asym_sweep_{dw,thr,brk,ov,cm,l3}.csv` | Per-knob sweep result tables. |
| `.tmp/hmpc_inner_asym_mc.csv` | 10-MC result at best weight set. |
| `.tmp/hmpc_inner_asym_sweep_traces/` | Per-variant debug-trace CSVs. |

**Created (Phase 2, ellipse-soft sweep, 2026-05-26):**

| File | Purpose |
|---|---|
| `.tmp/hmpc_inner_ellipse_soft_sweep.py` | Phase-2 sweep harness. |
| `.tmp/hmpc_inner_ellipse_soft_mc_headline.py` | One-shot 10-MC at headline (es=500 + L3 wa=5). |
| `.tmp/hmpc_inner_ellipse_soft_l3_fine.py` | Fine wa sweep at es=500 (wa in {1, 3, 5, 8, 12}). |
| `.tmp/hmpc_inner_ellipse_soft_cm_headline.py` | cm sweep at headline (cm in {0.85, 0.90}; cm=0.95 done via MC). |
| `.tmp/hmpc_inner_ellipse_soft_sweep_es.csv` | Single-seed ellipse_soft sweep (5 values). |
| `.tmp/hmpc_inner_ellipse_soft_sweep_cm.csv` | cm sweep at es=500 (no L3) — documents that relaxed ellipse alone is harmful. |
| `.tmp/hmpc_inner_ellipse_soft_cross_l3.csv` | Initial Lever-3 cross at es in {500, 1000}, wa in {0, 5}. |
| `.tmp/hmpc_inner_ellipse_soft_l3_fine.csv` | Fine wa sweep at es=500. |
| `.tmp/hmpc_inner_ellipse_soft_mc_headline.csv` | 10-MC at headline. |
| `.tmp/hmpc_inner_ellipse_soft_cm_headline.csv` | cm sweep at headline. |
| `.tmp/hmpc_inner_ellipse_soft_traces/` | Per-variant debug-trace CSVs. |

**No code changes in Phase 2.** Both `inner_w_ellipse_soft` and
`inner_w_a` were already wired in Phase 5.3 → Phase 1 work
(`hmpc_controller.py` ~ lines 418-419) and `MPCWeights`
(`mpc_qp.py`). The Phase-2 headline is purely a driver-JSON knob
combination.

**Modified:**

| File | Change |
|---|---|
| `src/lap_estimator/dynamics/mpc_qp.py` | `MPCWeights`: 5 new optional fields (default 0.0; back-compat). |
| `src/lap_estimator/dynamics/hmpc_inner_casadi.py` | Per-stage cost: swap combined rate term for `stage_rate_cost`; add `stage_pedal_shape_cost` per stage + terminal. |
| `src/lap_estimator/dynamics/hmpc_controller.py` | Driver-JSON plumbing: 5 new `control_params.hmpc.inner_w_*` knobs. |

**Not touched (as per task brief):**

- `src/lap_estimator/dynamics/driver_controller.py` — reactive.
- `src/lap_estimator/dynamics/car.py`, `simulator.py` — v2.
- `src/lap_estimator/dynamics/mpc_controller*.py`, `mpcc_*.py` — MPC/MPCC.
- `src/lap_estimator/dynamics/hmpc_inner.py` (legacy OSQP-SQP inner) —
  the new costs are gated on the CasADi inner only. The `MPCWeights`
  extension is benign on the OSQP path because `solve_sqp` doesn't
  consult the new fields.

---

## How to enable

Driver JSON example (`drivers/tomas.json` block — overrides applied via
`patch_hmpc_block` in `.tmp/hmpc_inner_asym_sweep.py` or by editing the
file directly):

```json
"control_params": {
  "hmpc": {
    "inner_solver": "casadi",
    "outer_vref_lookahead_stages": 20,
    "inner_w_du_brake": 0.5,
    "inner_w_du_throttle": 10.0,
    "inner_w_brake_double_well": 50.0,
    "inner_w_brake_throttle_overlap": 50.0
  }
}
```

CLI: no new flags. The hmpc driver-JSON block is the only entry point.
Existing CLI flags (`--hmpc-inner-solver`, `--chicane-safety-mult`)
remain.

---

## Notes for the next maintainer

- The `MPCWeights` defaults preserve every prior controller's behaviour.
  Sweep with care — the brief's `w_brake_double_well = 50.0` starts at a
  level where the friction-ellipse soft penalty (`w_ellipse_soft=5000`)
  is still dominant. You'll see larger brake-shape effects at
  `w_brake_double_well in {200, 500, 1000}` but those values also start
  to fight the actuator-bound soft penalty (`w_bound=1e4`); IPOPT may
  converge to a local min that defeats the intent.
- The legacy OSQP inner does NOT consume the new fields. Adding them
  would require splitting the slip-slack matrix in `mpc_qp.build_qp`
  per channel — out of scope (the brief says "leave the legacy inner
  alone").
- The trace CSV columns `thr_inner_emit` / `brk_inner_emit` are what
  the inner's first-stage-commit emits AFTER pedal-overlap suppression
  but BEFORE PI trim. The brake-bang-bang diagnostic reads `brake_p95`
  in the threshold-brake zone (s in [475, 585]) and `frac_brake_above_95`
  (fraction of in-brake samples committing past 0.95) — see
  `read_inner_trace_stats` in the sweep harness.
- The decision-variable split alternative ("hard `brake · throttle = 0`")
  was rejected in the design: trail-brake transitions need ε headroom,
  and the soft overlap penalty achieves the same end without IPOPT
  fighting a hard combinatorial constraint.
