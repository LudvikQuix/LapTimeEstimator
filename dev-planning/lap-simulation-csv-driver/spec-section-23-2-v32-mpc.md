# Spec §23.2 — v3.2 receding-horizon MPC controller (line-following first)

**Parent spec:** `dev-planning/lap-simulation-csv-driver/spec-section-23-slip-model.md`
**Predecessor:** `dev-planning/lap-simulation-csv-driver/spec-section-23-10-v31-controller.md` (v3.1, Phases 4.1 + 4.2 shipped, 4.3 dead)
**Status:** Draft (additive; replaces the §23.10.7 v3.2 stub)
**Project:** LapTimeEstimator
**Branch:** feature/sc-71955/lap-simulation
**Created:** 2026-05-22
**Planned with:** Buddy

This file scopes the controller redesign for v3.2. It is the planned answer
to the empirically-dead Phase 4.3 attempt (§23.10.12): no further reactive
scalar tweak on top of Stanley will close §11.55, because the controller
needs a **horizon** — it has to know what the next few corners want before
it commits to a steering and throttle command. v3.2 introduces a
receding-horizon model-predictive controller (MPC) that subsumes the
reactive Stanley + P-loop and the §23.10.12 softener, replacing them with
one optimisation per control step.

The Pacejka calibration, ODE solver, weight-transfer model, tyre-state
plumbing, and racing-line input are **unchanged** from v3.1 in Phase 5.0.
Free-driving (the controller picks its own line) is deferred to Phase 5.1.

Cross-references: §23.6 (ODE solver), §23.7 (Pacejka fit), §23.8 (v3.0
controller), §23.9 (phasing), §23.10 (v3.1 controller upgrade — what 5.0
replaces), §11.55 (headline gate, kept as the v3.2 acceptance bar).

---

## §23.2.1 Why v3.2 — the binding constraint moved

### State of v3.1 at end of Phase 4.3

| Configuration | Lap | Δ vs real | util_p85 | Notes |
|---|---|---|---|---|
| Real (Tomas lake avg, Sprint A) | 1:47.56 | — | — | — |
| v2 point-mass (current best) | 1:48.05 | +0.49 s | — | Friction circle, perfect lookahead |
| v3.1 Phase 4.1 (preview P + α_peak) | abort | n/a | n/a | OffTrackError on infeasible v2 plan |
| v3.1 Phase 4.2 (DP plan over Pacejka) | 2:04.14 | +16.6 s | 2.74 | Plan feasible, controller over-steers |
| v3.1 Phase 4.3 (softener) | abort | n/a | 0.16 | Cross-track 38 m mid-lap; lap aborts |

What Phase 4.3 proved (and the spec §23.10.12.8 fail-action commits to):

- **The plan is honest.** Lowering `safety_margin` 0.97 → 0.88 floors lap
  at 2:06.6 with `util_p85 = 1.10`. The controller is the binding
  constraint, not the plan.
- **Stanley δ does two coupled jobs.** It chases the apex (heading + κ
  feedforward) AND cancels cross-track error. A scalar attenuator on the
  output kills both at once: lateral demand collapses → cross-track grows
  → δ_cmd grows → attenuator clamps harder → divergence.
- **The fixed point is dynamic.** What the controller should do at any
  given step depends on what comes *next* — pre-turn-in for the upcoming
  apex, pre-rotate for a chicane reversal, hold a tight line through a
  decreasing-radius corner. No reactive law sees this.

### Why an MPC

MPC's basic mechanism — solve a finite-horizon optimal-control problem at
every step, apply the first control, re-solve next step — directly
addresses each failure mode:

- **Coupled objectives are encoded in a cost, not stacked on one law.**
  Cross-track error and slip-budget are weighted independently in the
  cost function; the solver finds the trade-off, not a hand-tuned scalar.
- **Constraints are first-class.** The tyre envelope (Pacejka curve or
  friction-ellipse proxy), steer-rate limits, brake-rate limits, and
  pedal slew enter the QP/NLP as hard inequalities. The controller
  cannot ask for an α the tyre cannot deliver.
- **Lookahead is structural.** A 30 m / 1.5 s horizon means the
  controller knows the next corner before the brake point.

The cost in shoehorning this into a real-time sim is **solve-time
budget**: v3.1 RK4 runs 200 Hz × 100-second lap = 20k control steps. At
1 ms/solve we add 20 s wallclock; at 10 ms/solve, 200 s. Solver choice
(§23.2.6) is therefore the single most important architecture decision.

---

## §23.2.2 Scope decision — Phase 5.0 line-following, Phase 5.1 free-driving

The §23.10.7 stub framed v3.2 as "MPC that chooses line + speed jointly".
That is the right end state but the wrong starting phase.

### Recommended phasing

1. **Phase 5.0 — line-following MPC.** Controller consumes the same
   externally-given racing line as v3.1 (v2 CSV ideal-line or v3 DP plan).
   Optimises **steering + throttle + brake** over a finite horizon to
   minimise a weighted sum of (a) cross-track deviation, (b) speed
   deviation from the reference plan, (c) slip-budget overshoot, and (d)
   control rate. Subsumes v3.1 Stanley + P-loop. **This is the phase
   that closes §11.55.**
2. **Phase 5.1 — free-driving MPC.** Controller drops the reference line
   and instead optimises **time-to-finish** subject to track-edge
   constraints and Pacejka envelope. The line emerges as part of the
   solution. This is the v3.2 stretch goal; it does not gate §11.55.

### Why split

- **Phase 5.0 is the smaller, more testable change.** State, dynamics
  model, constraints are largely the same in 5.0 and 5.1; the cost
  function and the search space differ. Shipping 5.0 first proves the
  MPC machinery (model linearisation, solver integration, real-time
  budget) on a problem where we *already know* the right answer
  (v2 lap = 1:48). If 5.0 doesn't close §11.55, the bug is in the MPC
  pipeline itself, not in the line-search heuristic.
- **Free-driving introduces line-search artefacts that mask MPC bugs.**
  A free-driving lap that comes in slow could be: (i) bad MPC, (ii) the
  cost function rewards a slow-but-stable line over a fast-but-marginal
  one, (iii) the horizon is too short to discover the racing line. With
  5.0, only (i) is in play.
- **Joint line+speed is meaningfully harder.** Adding 2-D lateral
  position as a free variable (vs cross-track error to a fixed
  reference) at least doubles the search space and turns a convex-ish
  QP into a non-convex NLP that needs warm-starting from the prior
  step's solution to converge in real time. Defer.

### What becomes of `longitudinal_planner.py` (Phase 4.2 DP)

- **Phase 5.0:** the DP plan stays. It is the MPC's reference speed
  trajectory `v_ref(s)` along the line. The MPC tracks it (with a soft
  cost) rather than chasing the local point speed reactively. The
  `--plan-source {v2, v3_dp}` flag survives unchanged.
- **Phase 5.1:** the DP plan is used as a **warm-start** for the MPC's
  first solve at lap start, then discarded once the MPC has built its
  own internal forward sweep. The `--plan-source` flag stays for
  regression / debugging — `--plan-source v2` skips the DP and feeds
  the MPC the v2 friction-circle plan as warm-start.

### Driver controller class layout

`DriverController` (the existing Phase 4.x reactive class) stays as a
**sibling** in `dynamics/driver_controller.py`, reachable via a new flag
`--controller {reactive, mpc}`, default `mpc` once Phase 5.0 ships.
Rationale:

- Reactive is byte-comparable for the v3.1 acceptance bar (Phase 4.2
  passes §11.55.C regression: `--plan-source v2` reproduces Phase 4.1
  diagnostic results). Keeping it reachable preserves the regression path
  without committing to ABI surgery.
- The MPC lives in a new file: `src/lap_estimator/dynamics/mpc_controller.py`.
  `_make_controller` in `slip_simulator.py` dispatches on the new
  `controller` kwarg.

This is the same architecture posture as v2 vs v3 itself (parallel
tracks, opt-in, default flips when the new path passes its acceptance
gate — §23.1 verbatim).

---

## §23.2.3 Horizon — 30 m, 15 stages, 100 ms tick

### Choice

- **Horizon length:** 30 m of forward arc-distance along the reference
  line. Equivalent to ~1.0 s at 30 m/s (Sprint A average speed
  ~28 m/s).
- **Number of stages:** 15.
- **Stage time:** variable, derived from `ds = 2.0 m / v_ref(s)`. Stages
  are *distance-uniform* (constant ds) rather than time-uniform; this
  keeps the cost weighting balanced across straights and corners.
- **Control tick (MPC re-solve cadence):** 100 ms (10 Hz) — every 20th
  ODE step at the v3.1 default `dt = 5 ms`. Between MPC solves, the
  first stage's controls are held constant; the ODE keeps integrating
  at 5 ms.

### Justification

- **30 m covers one corner-and-brake on Sprint A.** The CSV's median
  corner-to-corner distance on Sprint A's racing line is ~45 m
  (chicane-to-chicane on the sequence T1→T2→T3 is 38 m, 41 m, 52 m).
  30 m gives the controller "this corner and most of its braking
  zone". Longer horizons cost solve time without changing decisions —
  the next-next corner doesn't influence the current commit.
- **15 stages at 2 m each.** Distance-uniform on a 2 m grid matches
  the controller-state grid in v3.1's `_min_speed_within` lookup.
  Stage count was chosen so the QP has ~15 × n_controls (typically
  ~45) decision variables — small enough that hand-rolled SQP
  converges in 1-3 iterations, large enough that the horizon is
  meaningful.
- **10 Hz re-solve.** The ODE runs 200 Hz; re-solving at every ODE
  step is wasteful (the optimal solution barely changes between 5 ms
  steps). 100 ms is also the driver-input-slew time we already measure
  (v1.2 `driver_tau_s ~ 80-120 ms`); re-solving faster than that is
  modelling-incoherent. Below 10 Hz the controller becomes laggy on
  fast chicanes (where the chassis state changes more than the MPC
  expects between solves).
- **Trade-off summary.** At 10 Hz × 1000 laps in a Monte-Carlo sweep, a
  solve budget of 1 ms gives wallclock = (lap_count × steps_per_lap ×
  1 ms) ≈ 1000 × 1000 × 1 ms = 1000 s ≈ 17 min. 10 ms/solve → 170 min.
  This sets the solver-choice bar at §23.2.6.

---

## §23.2.4 State, controls, dynamics model inside the MPC

### MPC state vector (8 vars — reduced from solver's 10)

```
x_mpc = [e_lat, e_psi, v_x, v_y, omega_yaw, omega_drive_avg, throttle, brake]
```

| Symbol | Meaning | Units |
|---|---|---|
| `e_lat` | Cross-track error to reference line | m |
| `e_psi` | Heading error vs reference-line tangent | rad |
| `v_x, v_y` | Body-frame velocities | m/s |
| `omega_yaw` | Yaw rate | rad/s |
| `omega_drive_avg` | Average angular speed of driven wheels (proxy for engine RPM) | rad/s |
| `throttle` | Current throttle position (state, not control — to enable rate cost) | [0, 1] |
| `brake` | Current brake position (likewise) | [0, 1] |

**What's dropped vs the ODE state (§23.6.1):**

- `x, y, psi` (world position + absolute heading) → projected to
  `(e_lat, e_psi)` in line-relative frame. This is the standard MPC
  parameterisation; cuts 1 variable and makes the cost weights
  meaningful across the track.
- Individual wheel speeds `omega_FL/FR/RL/RR` → one averaged
  `omega_drive_avg`. The MPC doesn't model differential dynamics; the
  ODE does. Passes a single `engine_rpm = omega_drive_avg ·
  final_drive` proxy to the engine-torque curve.

### Controls (3 vars)

```
u_mpc = [delta_dot, throttle_dot, brake_dot]
```

Rates, not absolute values. The cost penalises large rates (smoothness
+ pedal-slew compliance — §23.2.5); the integrator inside the MPC
accumulates them into absolute `(delta, throttle, brake)` for the
plant model.

- `delta_dot ∈ [-max_steer_rate_rad_s, +max_steer_rate_rad_s]`.
  `max_steer_rate_rad_s` from v3.1 default 10 rad/s, or
  `control_params.steer_rate_limit_rad_s` if present.
- `throttle_dot ∈ [-throttle_rate_per_s, +throttle_rate_per_s]`.
  `throttle_rate_per_s` derived from v1.2-measured
  `profile.dynamic.throttle_ramp_pct_s` (same lookup as v3.1
  `DriverController.__init__`, lines 165-184).
- `brake_dot ∈ [-brake_rate_per_s, +brake_rate_per_s]`. Default
  symmetric with throttle; future field
  `profile.dynamic.brake_ramp_pct_s` if v1.2 measures it.

### Dynamics model inside the MPC — linear time-varying bicycle

Phase 5.0 uses a **linear time-varying (LTV) bicycle model** as the
MPC plant. NOT the full 4-wheel Pacejka ODE — that is too expensive to
linearise at every stage.

```
e_lat_dot    = v_x · sin(e_psi) + v_y · cos(e_psi)
e_psi_dot    = omega_yaw - kappa_ref(s) · v_x
v_x_dot      = (F_x_front · cos(δ) + F_x_rear + drag) / m + v_y · omega_yaw
v_y_dot      = (F_y_front · cos(δ) + F_y_rear) / m - v_x · omega_yaw
omega_yaw_dot = (a_f · F_y_front · cos(δ) - a_r · F_y_rear) / I_zz
omega_drive_dot = ...  # engine-torque - F_x_rear · R_tyre, single-axle proxy
```

Where `F_y_axle = -C_alpha(Fz, axle) · alpha_axle` is the **linearised
Pacejka** at the reference operating point (`alpha_ref(s) =
slip_target_rad`), and `F_x_axle` is the same for the longitudinal
Pacejka. Cornering stiffness `C_alpha` is computed once at controller
construction from the fitted `(B, C, D, E)` by differentiation:
`C_alpha = D · Fz · B · C` (the slope of the Magic Formula at α=0,
scaled by D/B and the C-shape factor — closed-form).

The LTV bit: `kappa_ref(s)`, `v_ref(s)`, and the slip operating point
re-linearise at each stage from the reference plan. This is the
standard "tracking MPC" trick — the plant model is locally linear about
a known good trajectory.

**Why a bicycle, not full 4-wheel:** Phase 5.0's job is *line-following*.
The lateral force balance the MPC needs to know is total axle force vs
yaw moment — the 4-wheel split only matters for combined-slip
interactions (which happen at the tyre, not at the axle), and those are
captured by the ODE's full Pacejka call on each integration step.
Adding 4-wheel detail to the MPC would 4× the state and ~16× the QP
size with no behavioural gain inside the horizon. v3.1 reactive Stanley
also used a bicycle approximation (one steer command, no per-wheel
Ackermann inside the controller) and that wasn't where v3.1 failed.

**Pacejka envelope inside the MPC.** The grip envelope is enforced as a
constraint, not via the linearised force model. See §23.2.5.

---

## §23.2.5 Constraints

### Hard (the QP rejects infeasible solutions)

1. **Tyre slip envelope (friction-ellipse proxy).** Per-axle:
   `(F_x_axle / (D_long · Fz_axle))² + (F_y_axle / (D_lat · Fz_axle))² ≤ 1`.
   Fz from static loads + the previous step's quasi-static weight
   transfer (held constant over the horizon — no WT prediction in 5.0).
   Friction-ellipse proxy chosen over full Pacejka because:
   - It's a convex constraint on `(F_x, F_y)` (ellipse interior is
     convex). Real Pacejka is non-convex on the falling side.
   - The fitted ellipse exponent for Tomas is `n = 2.0` (true ellipse,
     not blunted). For drivers with `n > 2`, the constraint is
     conservative — the MPC under-uses the corner of the envelope but
     the ODE simulator catches it.
2. **Actuator absolute limits.** `delta ∈ [-max_steer_rad, +max_steer_rad]`;
   `throttle, brake ∈ [0, 1]`. These are state-space bounds (because the
   controls are rates).
3. **Actuator rate limits.** Already on `u_mpc` directly — §23.2.4.
4. **Speed lower bound.** `v_x ≥ 0.5 m/s` over the horizon. Prevents the
   MPC from planning a stop; the ODE's stall guard at §23.6.5 catches
   real stalls.

### Soft (cost terms — see §23.2.6)

5. **Slip budget.** `|alpha_axle| ≤ alpha_peak · skill_factor`, where
   `skill_factor = (0.5 + 0.5 · skill_pct)`. Soft because the linearised
   model can't precisely predict α at the horizon end; a hard
   constraint here causes infeasibility recovery to dominate. See §23.2.6
   for the cost weight; spec §23.2.10 for fallback when infeasible.
6. **Track edges (Phase 5.1 only).** `|e_lat| ≤ track_half_width(s)` —
   soft in 5.0 (cost weight on `e_lat`); hard in 5.1 (where the line
   is free).
7. **Cross-track to reference line (Phase 5.0).** Soft cost on `e_lat²`
   over the horizon; no hard bound. The Phase 4.3 cross-track abort
   threshold (8 m) is **kept** as the simulator's safety guard and as
   the §11.55.E acceptance check.

---

## §23.2.6 Cost function

```
J = Σ_{k=0..N-1} [
        w_lat  · e_lat[k]²
      + w_psi  · e_psi[k]²
      + w_v    · (v_x[k] - v_ref(s[k]))²
      + w_slip · max(0, |alpha_axle[k]| - alpha_peak·skill_factor)²
      + w_du   · u[k]ᵀ · diag(R) · u[k]
      + w_du2  · (u[k] - u[k-1])ᵀ · diag(R) · (u[k] - u[k-1])
    ] + w_term · e_term²
```

### Weights (default tuning)

| Weight | Symbol | Default | Units / meaning |
|---|---|---|---|
| `w_lat` | cross-track | 50 | per m² |
| `w_psi` | heading error | 5 | per rad² |
| `w_v` | speed deviation | 2 | per (m/s)² |
| `w_slip` | slip overshoot | 200 | per rad² above peak |
| `w_du` | control magnitude | 0.1 | per (rad/s, fraction/s)² |
| `w_du2` | control rate-of-rate (chatter) | 0.05 | same |
| `w_term` | terminal cost | 100 | weighted sum of state errors at horizon end |

These are seed values; expect ArchDev to retune `(w_lat, w_v, w_slip)`
during Phase 5.0 acceptance. The **ratio** `w_lat / w_v ≈ 25` makes
the controller prefer hitting the line over hitting the reference
speed — i.e. it will under-drive a corner rather than miss the apex by
1 m. This is the v3.1 lesson: missing the apex by 1 m means radius is
tighter than κ_plan → demanded α exceeds peak → tyre force collapses.
A high `w_lat` removes that failure mode by spec.

### Cost objective discussion — minimise time? deviation? both?

Three candidates were weighed:

| Cost | Pros | Cons | Decision |
|---|---|---|---|
| Track v_ref(s) deviation (above) | Convex QP; warm-starts; cheap | Slowdown is bounded by plan quality | **Phase 5.0** |
| Maximise terminal s along line | True minimum-time; intrinsically rewards going fast | Non-convex; needs full NLP; slow solves | Phase 5.1 |
| Pure time-to-finish (`J = T`) | Cleanest formulation | Infinite-horizon; requires terminal cost approximation | Out of scope |

**Phase 5.0 recommendation: track v_ref(s) deviation.** This is
**not** minimum-time — it's "track the DP plan tightly". The DP plan
*is* minimum-time against the envelope; if the MPC tracks it well,
the lap is minimum-time-of-the-tracked-plan, which is the §11.55
target. If §11.55 closes, we have proof that the plan is right and the
controller is the bug; if it doesn't close, the DP plan needs revisit
(see §23.2.11 risks).

**Phase 5.1 (free-driving) flips to terminal s.** Once the line is free,
"track the plan" is incoherent (there is no plan). The objective
becomes maximise(s[N-1]) subject to track edges; the resulting NLP is
warm-started from the Phase 5.0 solution and solved with a path-tracking
MPC fallback for infeasibility recovery.

---

## §23.2.7 Solver choice — **hand-rolled SQP with OSQP inner-QP**

### Recommendation

**Use `osqp` as the inner-loop convex QP solver, with a thin SQP wrapper
that re-linearises every 2-3 outer iterations.** No `acados`, no
`forces`, no CasADi for Phase 5.0.

### Why not acados / forces

- **`acados`** is purpose-built for embedded MPC and would give us
  ~100 µs solve times. But: (a) Windows installation is a non-trivial
  CMake + MSVC dance with code generation; users have reported
  multi-hour setup. The user's env is win32 (`OS Version: Windows 11`,
  `Platform: win32`). (b) `acados` expects model code generated via
  CasADi → C; debugging is two-language. (c) Heavy dependency for a
  Python research codebase that wants to stay `pip install`-able.
- **`forces` (FORCES Pro)** is commercial; not viable.
- **CasADi + IPOPT** runs cleanly on Windows (`pip install casadi`) but
  IPOPT solve times for a 15-stage MPC are 10-50 ms — too slow at
  our 10 Hz × 1000-step lap budget.

### Why OSQP + SQP

- **OSQP** is a high-performance ADMM-based convex QP solver with a
  clean Python binding (`pip install osqp`). Pure Python install on
  Windows; one wheel; no C toolchain. Solve times for 15-stage MPC
  with ~45 decision vars and ~60 constraints are sub-millisecond on
  modern x86 (osqp benchmarks: 50-300 µs).
- **The MPC is mostly convex** once we use the linearised bicycle +
  friction-ellipse-proxy constraint — both the cost and the
  constraints are quadratic / linear in the decision variables. The
  remaining non-linearity (the speed-dependent linearisation of
  Pacejka, kappa_ref schedule) is handled by SQP outer iterations:
  solve QP, integrate model with new controls, re-linearise, repeat.
- **2-3 SQP iterations is typically enough** for tracking MPC with a
  good warm-start. Total solve time per MPC tick: 0.5 ms × 3 = 1.5 ms
  worst case. Within the 100 ms control tick by 60×.
- **Hand-rolled keeps the codebase Python.** The MPC module
  (`mpc_controller.py`) is plain NumPy + OSQP. No code generation, no
  C deps beyond OSQP's pre-compiled wheel.

### Dependency surface — new

- `osqp` (BSD-3, pure-Python install on Windows via prebuilt wheel).
  Added to repo-root `requirements.txt`.
- That is the only new dependency for Phase 5.0.

### Real-time budget — target & sanity

| Phase | Lap time (s) | Steps (200 Hz ODE) | MPC ticks (10 Hz) | Budget @ 1.5 ms/tick |
|---|---|---|---|---|
| Single lap | ~108 | 21,600 | 1,080 | 1.6 s wallclock |
| MC sweep (10 runs) | 1,080 | 216,000 | 10,800 | 16 s wallclock |
| Stint (10 laps) | 1,080 | 216,000 | 10,800 | 16 s wallclock |

A single Sprint A lap should add **~1-2 s of MPC overhead** to the v3.1
~5 s wallclock — totally acceptable. If solve time exceeds 5 ms per tick
in practice, the SQP iteration count is dialled back from 3 to 1
(single-shot QP) at the cost of slightly less accurate linearisation.

---

## §23.2.8 Integration with the existing simulator

### Where the MPC lives

**New file:** `src/lap_estimator/dynamics/mpc_controller.py` (~450 LoC).

Soft seams for splitting if it grows past 500 lines:
- `mpc_controller.py` — public surface (`MPCController` class) +
  `controls()` per-step wrapper.
- `mpc_model.py` — LTV bicycle model + linearisation utility.
- `mpc_qp.py` — OSQP problem setup + SQP outer loop.

### Class layout

```python
class MPCController:
    """Receding-horizon MPC controller (spec §23.2).

    Interface contract matches DriverController and GhostDriver:
        .controls(state, t, track=None) -> Controls
    """
    def __init__(
        self,
        driver: Driver,
        track: Track,
        params: ControlParams | None = None,
        *,
        plan: LongitudinalPlan,                # required (replaces reactive plan lookup)
        calib: PacejkaCalibration,             # required for envelope linearisation
        rng_seed: int | None = None,
        horizon_m: float = 30.0,
        n_stages: int = 15,
        tick_hz: float = 10.0,
        mpc_params: MPCParams | None = None,   # weights, solver opts; defaults from MPCParams
    ): ...

    def controls(self, state: VehicleState, t: float,
                 track: Track | None = None) -> Controls: ...
```

### How `slip_simulator.py` wires it up

The existing `_make_controller` helper in `slip_simulator.py` (Phase 4)
gains a `controller` dispatcher arg:

```python
def _make_controller(car, track, driver, calib, plan, *, controller: str, ...):
    if controller == "reactive":
        return DriverController(driver, track, ..., target_speed_ds=plan.distances,
                                target_speeds=plan.speeds)
    if controller == "mpc":
        return MPCController(driver, track, plan=plan, calib=calib, ...)
    raise ValueError(f"Unknown controller={controller!r}")
```

`simulate_slip(...)` gains a new kwarg `controller: str = "mpc"` (default
flipped post-5.0 acceptance; ships as `"reactive"` during Phase 5.0
development to keep the regression bar intact, then flips after the
gate passes).

### Driver skill mapping — preserved

The v3.1 mapping `slip_target_deg = α_peak_front_deg · (0.5 + 0.5 ·
skill_pct)` is **kept verbatim**. The MPC consumes `slip_target_rad`
exactly the same way v3.1 reactive did:

- `w_slip` cost term is keyed off `alpha_peak · skill_factor` where
  `skill_factor = (0.5 + 0.5 · skill_pct)`.
- Low-skill drivers cruise lower on the Pacejka curve → smaller
  `alpha_peak · skill_factor` → MPC slows down in corners to stay
  under-budget. Same physical behaviour as v3.1.
- The §11.56 skill-monotonicity check (`skill=0.5` is ≥ 3 s slower
  than `skill=1.0`) is **kept** as a §11.55.B equivalent for v3.2.

### Lap-end / stint hand-off — unchanged

Stint mode (§23.6.4 carry-over of `v_x, v_y, omega_yaw, omega_w[w]`
across laps) is unchanged. The MPC's internal warm-start buffer is
cleared at lap boundaries; it re-initialises from the first stage of
the new lap's reference plan. This costs ~1 extra MPC tick per lap
(negligible).

### Consistency noise — applied at output, not inside MPC

The v3.1 per-channel Gaussian noise (`consistency_noise_std_steer_deg`
etc.) is applied to the **first stage of the MPC solution**, post-solve.
This matches v3.1 semantics (the MPC plans the noiseless trajectory;
the driver-execution noise is layered on the commit). Monte-Carlo runs
flip noise on/off the same way they do in v3.1.

---

## §23.2.9 Phase plan

### Phase 5.0 — line-following MPC (~3-5 days ArchDev)

**Scope:**
- `mpc_controller.py` per §23.2.8.
- LTV bicycle model + friction-ellipse-proxy constraint (§23.2.4, §23.2.5).
- Cost function per §23.2.6 with the recommended seed weights.
- OSQP integration; SQP outer loop (3 iterations cap).
- `slip_simulator.py` gains `--controller {reactive, mpc}` dispatch.
- `lap.py` gains `--controller {reactive, mpc}` CLI flag, default
  `mpc` once gate passes.
- `--plan-source` flag unchanged; the MPC consumes whichever plan is
  built.
- New CLI flag `--mpc-horizon-m FLOAT` (default 30.0) for solver
  tuning; not exposed on Web UI.

**Acceptance gate — Phase 5.0 (= §11.55 verbatim):**

> Tomas on Sprint A `--model slip --controller mpc --skill-pct 1.0`
> produces a lap time within **±3 s** of his real lake average (1:47.56
> → target 1:44.5–1:50.5). `util_p85 ≤ 1.05` honestly computed.
> Ghost-fallback step count ≤ **20** per lap. MC-σ (10 runs at
> skill=1.0) ≤ 0.8 s.

Plus supporting checks:

- **§11.55.F (Phase 5.0):** Max cross-track error ≤ 4 m across the lap.
  Same threshold the dead Phase 4.3 was measured against; with the MPC's
  horizon, this should be achievable.
- **§11.55.G (Phase 5.0):** Skill=0.5 lap is ≥ 3 s slower than skill=1.0.
  Equivalent to §11.55.B; proves skill mapping survives.
- **§11.55.H (Phase 5.0):** `--controller reactive` reproduces the
  Phase 4.2 result (2:04.14 ± 0.5 s, util_p85 ≈ 2.74). Regression.
- **§11.55.I (Phase 5.0):** Median MPC solve time per tick < 5 ms;
  99th percentile < 20 ms. Real-time budget compliance.

**Seconds-shaved target:** 2:04.14 → 1:48-50. Closing ~14-16 s.

**Fail action:** If Phase 5.0 misses ±3 s but lands ±5 s, ship 5.0 and
revisit the cost weights / DP plan safety margin. If ±5 s also missed,
the bug is structural (model linearisation drift? plan infeasibility?)
— post-mortem before 5.1.

### Phase 5.1 — free-driving MPC (~5-7 days ArchDev, stretch)

**Scope:**
- Drop reference line; add `(e_lat, e_psi)` to free variables.
- Cost flips to terminal-s maximisation (§23.2.6 alternative).
- Track-edge constraints from track CSV (left/right edge polylines).
- Warm-start from Phase 5.0's last-tick solution at lap start, from
  prior tick thereafter.
- Solver upgrade to CasADi-IPOPT (NLP) OR a 2-level decomposition
  (outer: line search; inner: speed MPC) — decide during 5.1 design.
  Phase 5.0 ships pure-OSQP regardless.

**Acceptance gate — Phase 5.1 (stretch):**

- Lap time **at-or-below** Phase 5.0 on Sprint A + Tomas + skill=1.0.
  The free-driving line should be at least as fast as the recorded
  line; ideally faster by 0.5-1.5 s.
- All Phase 5.0 supporting checks still pass.
- No new abort modes (OffTrackError, NaN).

**Seconds-shaved target:** marginal — Tomas's recorded line is already
close to optimal on Sprint A. Phase 5.1 is more useful for unfamiliar
tracks or non-Tomas drivers where the recorded line is suboptimal.

### Phase 5.2 — UI surface (Web UI, ~half day ArchDev + FrontEndEsthetic)

**Scope:** Sim tab gets a third radio under the existing "Plan source"
group (the §23.10.8 precedent — collapsed by default):

```
[Sim tab → Advanced (collapsed) →]
  Controller:  ( ) Reactive (Phase 4.x — fast, lossy)
               (•) MPC (Phase 5.0 — slow, accurate)
  Plan source: ( ) v2 friction circle
               (•) v3 DP (Pacejka envelope)
  Free driving: [ ] enable (Phase 5.1)
```

Driver tab gets no new fields — the MPC reads `control_params` already.
The dropdown wires through `--controller` and `--free-driving` flags.

**Acceptance gate — Phase 5.2:** Manual smoke test on user's machine.
No code-level gate.

### Total wall-clock estimate

**Phase 5.0:** ~3-5 days at ArchDev pace, 1-2 weeks if real-time
tuning is hairy. **Phase 5.1:** ~5-7 days (NLP step is the unknown).
**Phase 5.2:** half day.

---

## §23.2.10 Infeasibility & fallback policy

The MPC's QP can return INFEASIBLE — typically when the chassis is
already past the slip envelope's edge and there's no horizon-end state
that satisfies all constraints. Phase 5.0 must define a clean recovery.

### Three-tier fallback

1. **Soften slip constraint and re-solve.** The slip-budget constraint
   is soft (§23.2.5 item 5). On INFEASIBLE return, OSQP-MPC bumps
   `w_slip` from 200 → 1000 and re-solves; this typically converges in
   a single extra iteration and produces a "I'm overshooting, give me
   the least-bad solution" trajectory. ≤ 2 ms extra wallclock.
2. **If still INFEASIBLE: hand control to `GhostDriver` for this step.**
   Same fallback path the Phase 4 reactive controller uses (the
   `_ghost_fallback()` helper in `driver_controller.py`). Counts toward
   the §11.55 ghost-fallback cap of 20/lap. Logs a one-shot warning.
3. **If GhostDriver also produces a divergent command (yaw > 4 rad/s,
   speed < 0.5 m/s persistent), abort the lap.** This is the ODE's
   existing `SpunError` / `StalledError` path; no MPC-specific
   handling needed.

### Why not hand-roll a recovery in the MPC

A "feasibility-restoration" outer loop (à la IPOPT) would add
complexity for negligible benefit — the GhostDriver hand-off is
already proven on Sprint A across thousands of v3.1 ghost-fallback
events and is a smaller, more testable code path.

---

## §23.2.11 Risks and open questions

### Risks

1. **MPC solve time blows real-time budget.** Median solve target is
   < 5 ms; 99th percentile < 20 ms. If OSQP regularly takes 50+ ms
   on a 15-stage problem (e.g. on a slow Windows laptop), the lap
   wallclock balloons from ~7 s to ~60+ s and Monte-Carlo sweeps
   become impractical.
   - **Mitigation A.** Cap SQP outer iterations at 1 if median per-tick
     time exceeds 5 ms.
   - **Mitigation B.** Reduce horizon to 10 stages if Mitigation A
     isn't enough; the corner-spacing argument in §23.2.3 still holds
     at 20 m horizon for most of Sprint A.
   - **Detection:** §11.55.I gates median + 99p solve time. Visible in
     the per-step trace CSV.

2. **Pacejka linearisation drift.** The MPC uses `C_alpha = D · Fz ·
   B · C` at α = 0; for the operating point at `α = α_peak`, this
   over-estimates lateral force (the true Pacejka curve is below the
   tangent line). The cost function compensates via `w_slip` but the
   controller may still commit a δ that the ODE then under-delivers.
   - **Mitigation A.** Linearise about `α = 0.5 · alpha_peak ·
     skill_factor` (operating-point linearisation rather than
     small-signal) for the front axle, where most of the action is.
   - **Mitigation B.** If gate fails, add an MPC SQP-iteration that
     evaluates true Pacejka at the planned trajectory and adjusts
     `C_alpha` per stage. Adds 1-2 ms per tick.
   - **Detection:** plot `alpha_ref_mpc(stage)` vs `alpha_observed_ode`
     over a lap; gap > 30% means the linearisation is bad.

3. **Infeasibility recovery dominates on tight chicanes.** The Sprint A
   chicane at s ≈ 890 m (the one that aborted every Phase 4.1 run)
   has a curvature reversal in ~12 m of arc — well inside the MPC
   horizon. If the controller's linearisation can't predict the
   reversal correctly, the QP returns INFEASIBLE at every tick through
   the chicane and the GhostDriver fallback fires 40+ times per lap,
   blowing the §11.55 ≤ 20/lap cap.
   - **Mitigation A.** Pre-process the reference plan to enforce a
     minimum curvature-change-per-stage; smooth the chicane's
     curvature profile into the LTV linearisation.
   - **Mitigation B.** Increase `w_psi` weight specifically through
     chicane stages so the MPC prioritises rotation over cross-track.
   - **Detection:** ghost-fallback count broken down by `s_along_track`
     in the trace CSV.

4. **Cost-weight tuning is fragile.** `(w_lat, w_v, w_slip)` ratios
   determine the trade-off; a 2× change in `w_slip` can flip the lap
   from "tracks the plan" to "ignores the plan and over-drives". v3.1
   reactive Stanley had two scalar gains (`steering_p_gain`,
   `throttle_p_gain`); the MPC has 7. Risk of "works for Tomas, breaks
   for Ludvik".
   - **Mitigation A.** Express weights as **scale-invariant ratios**
     internally (`w_lat / w_v`, `w_slip / w_lat`) so tuning one
     driver doesn't tank another. Phase 5.0 ships with one weight set
     validated on Tomas; Ludvik runs use the same weights and are
     measured at acceptance time as a generalisation check.
   - **Mitigation B.** Document the weight defaults explicitly in
     `_control_params.py` MPC block; surface a `--mpc-weight-set`
     flag for power users.
   - **Detection:** comparison run on Ludvik before Phase 5.0 ships.

5. **Windows-specific OSQP failure modes.** OSQP's prebuilt wheel
   targets recent Python versions on x86_64. If the user is on an
   ARM Windows machine or an older Python, install fails.
   - **Mitigation A.** Document the supported Python range in
     `requirements.txt` (>= 3.10).
   - **Mitigation B.** Provide a graceful "OSQP unavailable, MPC
     disabled, falling back to reactive" error path in
     `_make_controller` rather than a raw `ImportError`.

### Open questions (resolve during build)

- **Should the MPC re-linearise more often than every SQP outer
  iteration?** v3.1 changes its α-peak only at controller construction;
  the MPC could amortise re-linearisation across ticks. Decision
  deferred to Phase 5.0 acceptance; if median solve < 1 ms, re-linearise
  every tick; otherwise every other tick.
- **How to encode skill in the cost vs. constraints?** Current spec:
  via `slip_target = α_peak · skill_factor` in the soft constraint.
  Alternative: scale `w_v` (low-skill = lower speed weight, controller
  willingly under-drives). Phase 5.0 ships with the slip-constraint
  approach; if §11.55.G fails, revisit.
- **Warm-start strategy on first MPC tick of a lap.** Cold start (zeros)
  is safe but the first 100 ms of the lap is sub-optimal. Warm-start
  from the DP plan's first 30 m? Decision: yes, warm-start. Cost in
  complexity is low.
- **Should the MPC see weight-transfer predictions across stages?**
  v3.1 didn't; the simulator handles it. Phase 5.0 keeps the same
  approach (static Fz inside the MPC horizon). If §11.55.F fails on
  high-decel corners, revisit by adding a single quasi-static WT
  pass per stage.

---

## §23.2.12 Migration & back-compat

### Driver JSON additions

`control_params` block gains optional MPC sub-block. Absent → defaults
apply:

```jsonc
{
  "control_params": {
    "preview_distance_m": 18.0,
    "preview_time_s": 1.5,
    "steering_p_gain": 1.2,
    "throttle_p_gain": 0.5,
    "brake_p_gain": 0.6,
    "throttle_rate_limit_pct_s": 250,
    "slip_target_deg": null,
    "steering_softener_engage": 1.5,    // Phase 4.3 dead — set to disable
    "steering_softener_full":   1.5,
    "mpc": {                              // NEW — Phase 5.0
      "horizon_m": 30.0,
      "n_stages": 15,
      "tick_hz": 10.0,
      "w_lat":  50.0,
      "w_psi":   5.0,
      "w_v":     2.0,
      "w_slip": 200.0,
      "w_du":    0.1,
      "w_du2":   0.05,
      "w_term": 100.0,
      "sqp_max_iter": 3
    }
  }
}
```

- All MPC sub-block fields optional. Absent → coded defaults match
  §23.2.6 / §23.2.7.
- Validation at driver-load (in `ControlParams.from_driver`): all
  weights non-negative; `0 < n_stages ≤ 50`; `1 ≤ tick_hz ≤ 100`;
  `1 ≤ horizon_m ≤ 200`. Out-of-range raises a clear ValueError at
  load time (matches Phase 4.3 schema-validation precedent).
- Existing driver JSONs on disk (Tomas, Ludvik) load unchanged —
  no `mpc` sub-block means use defaults.

### CLI surface

`lap.py` gains two new flags. **Order of precedence (high → low):**
explicit CLI > driver JSON `mpc` block > coded defaults.

```
--controller {reactive, mpc}   default: mpc  (default flips post Phase 5.0 acceptance gate)
--mpc-horizon-m FLOAT          default: 30.0
```

`--model point-mass` ignores both; `--controller reactive --model slip`
runs Phase 4.2 (the regression path).

### Web UI surface

Per §23.2.9 Phase 5.2: Sim-tab "Advanced" collapsed-by-default panel
adds a Controller radio (`Reactive` / `MPC`) with `MPC` default. No
changes to the Driver tab. Matches the §23.10.8 precedent ("Plan
source" radio under the same panel).

### `target_speed_scale` kwarg

Already deprecated in v3.1 (§23.10.5.3). v3.2 **removes** it from
`DriverController.__init__` as the documented v3.2 ABI cleanup. No
production caller passes it; `_make_controller` already forces 1.0.

### Phase 2 fit pipeline / Pacejka calibration / track CSV / lake schema

**Unchanged.** v3.2 reads the existing `pacejka_calibration` block
verbatim. `fit_driver.py` untouched. Track CSV format untouched.
Lake schema untouched.

---

## §23.2.13 Decisions block update — proposed §23.M

Add this item to the Decisions block at the bottom of `spec.md` (after
the §27 v3.1 amendment in §23.10.10):

> **28. (v3.2) Receding-horizon MPC controller — line-following first.**
> The v3.1 reactive Stanley + P-controller stack (§23.10) reached its
> structural limit at Tomas-Sprint-A 2:04.14 (+16.6 s vs real 1:47.56);
> Phase 4.3's scalar steering softener did not close §11.55 and was
> empirically dead by spec §23.10.12.8. v3.2 ships a receding-horizon
> MPC in `src/lap_estimator/dynamics/mpc_controller.py` as a sibling to
> `DriverController`, dispatched via `lap.py --controller {reactive,
> mpc}` (default `mpc` post-acceptance) and an additive Sim-tab
> Advanced radio on the §22 web UI. Phasing: **Phase 5.0** —
> line-following MPC consuming the same v3 DP plan as a reference,
> minimising a weighted sum of cross-track error, speed deviation, slip
> overshoot, and control rate over a 30 m / 15-stage / 10 Hz horizon;
> closes §11.55. **Phase 5.1 (stretch)** — free-driving MPC; line
> emerges as part of the solution. **Phase 5.2** — UI radio.
> Architecture: LTV bicycle model linearised about the reference
> trajectory at each MPC tick; friction-ellipse-proxy as a convex
> constraint; OSQP convex-QP inner solver with a 3-iteration SQP outer
> loop. New dependency: `osqp` (pure-Python install, BSD-3). The
> Pacejka calibration, ODE solver, weight-transfer model, tyre-state
> plumbing, racing-line input, and driver JSON schema are all
> unchanged from v3.1 except an optional `control_params.mpc`
> sub-block (defaults apply when absent). Driver skill mapping
> (`slip_target = α_peak · (0.5 + 0.5 · skill)`) is preserved.
> Infeasibility recovery: bump `w_slip` and re-solve → GhostDriver
> hand-off (counts against §11.55 ≤ 20/lap cap) → simulator's existing
> abort guards. Acceptance: §11.55 verbatim plus §11.55.F (max
> cross-track ≤ 4 m), §11.55.G (skill monotonicity), §11.55.H
> (`--controller reactive` reproduces Phase 4.2), §11.55.I (median MPC
> solve < 5 ms). (§23.2.1–§23.2.13, §11.55, §11.55.F–I.)

---

## §23.2.14 References

- `dev-planning/lap-simulation-csv-driver/spec.md` — parent spec; §27
  Decisions item for v3 / v3.1 amended here by §23.M.
- `dev-planning/lap-simulation-csv-driver/spec-section-23-slip-model.md`
  — v3.0 slip-based dynamics; §23.6 (ODE state vector), §23.8 (v3.0
  controller), §23.10.7 (the v3.2 stub this file expands).
- `dev-planning/lap-simulation-csv-driver/spec-section-23-10-v31-controller.md`
  — v3.1 controller upgrade; §23.10.7 → 2-line pointer at this file;
  §23.10.12 Phase 4.3 post-mortem (dead).
- `docs/architecture-slip-model-phase4_2-v31-dp-planner.md` — DP plan
  the MPC consumes as `v_ref(s)`.
- `docs/architecture-slip-model-phase4_3-v31-steering-softener.md` —
  Phase 4.3 post-mortem; the §11.55.E cross-track gate this file
  re-uses as §11.55.F.
- `src/lap_estimator/dynamics/driver_controller.py` — sibling reactive
  controller; stays reachable via `--controller reactive`.
- `src/lap_estimator/dynamics/longitudinal_planner.py` — Phase 4.2 DP
  planner; consumed by the MPC as the reference plan.
- `src/lap_estimator/dynamics/slip_simulator.py` — entry point;
  `_make_controller` gains the `controller` dispatcher.
- `src/lap_estimator/dynamics/solver.py` — RK4 ODE driver; unchanged.
  Off-track abort threshold (8 m, tightened by dead Phase 4.3)
  remains as the §11.55.F observability guard.
- OSQP — https://osqp.org/ — convex QP solver; BSD-3; `pip install osqp`.
- Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed., Ch. 4 — Magic Formula;
  cornering stiffness `C_alpha = D · Fz · B · C` derivation.
- Borrelli, Bemporad, Morari, *Predictive Control for Linear and Hybrid
  Systems* — LTV-MPC tracking formulation reference.

---

## §23.2.15 Phase 5.0.1 corrective patch — pointer

Phase 5.0 shipped against this spec but missed §11.55 (Tomas/Sprint A: +36.1 s,
0/10 MC finishes — see `docs/architecture-slip-model-phase5_0-v32-mpc.md`).
The two named structural fixes (operating-point Pacejka linearisation +
true friction-ellipse-proxy QP constraint) plus regression triage are
specified in `dev-planning/lap-simulation-csv-driver/spec-section-23-2-v32-mpc-phase5_0_1.md`.
That file also revises the §11.55 acceptance numbers for v3.2; §11.55 verbatim
above remains historical context for the Phase 5.0 attempt.
