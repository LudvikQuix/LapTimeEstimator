# Spec §23 — Slip-based dynamics model (v3 parallel track)

**Parent spec:** `dev-planning/lap-simulation-csv-driver/spec.md`
**Status:** Draft (v3 parallel-track, additive — does not modify v2/v1.3 simulator behaviour)
**Project:** LapTimeEstimator
**Branch:** feature/sc-71955/lap-simulation
**Created:** 2026-05-15
**Planned with:** Buddy

This file is an additive section of the main spec. It should be appended verbatim
at the end of `spec.md` (after §22 web-UI section, before the "Decisions block"),
and the new Decisions item §23.L below should be merged into the Decisions block.

The §22 web UI gains a **Model dropdown** (Sim tab) and a **Slip model
parameters** panel (Driver tab) — see §23.4. §22 itself is otherwise
unchanged.

---

## §23.1 Goal

Today the simulator is a single 3-pass kinematic point-mass model (`simulator.py`)
with v2 post-lap tyre-state scaling layered on top (§21). The model is fast and
predicts lap time well for clean, on-the-limit laps. It is, however, *kinematic*
— the grip envelope is a scalar `(mu_x, mu_y)` clipped at the friction circle,
the chassis has no yaw, the tyres have no slip angle, and the driver has no
controller. The two consequences that matter:

1. **Util > 1.0 in real telemetry.** Observed `util_p85` on Tomas's lake laps
   exceeds 1.0, which is physically impossible against the v2 grip circle. The
   model's `mu` is too low because the kinematic envelope doesn't represent the
   true non-linear Pacejka response — it averages where it should peak.
2. **No drift / oversteer / understeer.** The car is a particle; there is no
   way to express a slip-angle window, no way to ask "what does a 6° slip
   driver look like vs a 3° slip driver", no way to bind tyre-state evolution
   to *honest* slip energy (v2 §21.3 uses a `v² / R` proxy for slip energy).

**v3** replaces the kinematic envelope with a **slip-based 3-DOF planar
chassis + 4-wheel Pacejka Magic Formula + time-domain ODE integrator + driver
controller**. It runs **in parallel** with the v2 point-mass model — both
coexist in the codebase, both are reachable from the CLI and the web UI, the
user picks per run. v3 is opt-in; v2 stays the default.

**User intent (verbatim, 2026-05-15):**
> "Can we store this kinematic approach model we have now, keep it working
> with UI and try to build this v3 model in parallel?"

Yes — that is the architectural posture this section locks.

---

## §23.2 Non-goals

- **Replacing v2.** v2 stays as-is, stays the default, stays the recommended
  path for "give me a lap time fast". v3 is the right answer when v2 is
  physically wrong (util>1, slip behaviour, driver-style sensitivity).
- **Aero refinement.** Dive/squat, downforce surface effects beyond the v2
  `Cd·A` model — v3.1.
- **Mid-stint compound change (pit stops).** Out of scope, ever.
- **Differential modelling.** Open / LSD / preload / ramp coupling between
  rear wheels — v3.1. v3 treats RL and RR as independently driven (no
  inter-wheel coupling beyond shared rear-axle weight transfer).
- **Driver fatigue / consistency drift over stint.** v3.1.
- **Replacing v2's tyre-state plumbing.** v3 reuses `tyre_state.py` for
  wear/temp/pressure evolution; v3 only changes the **slip-energy input** that
  feeds it (Pacejka-derived instead of `v²/R` proxy). The
  `f_pressure_grip`/`f_pressure_drag` functions stay; the
  `combined_grip_envelope` scalar reduction is bypassed in v3 (per-wheel grip
  is consumed directly).
- **Driver JSON schema replacement.** v3 *adds* optional `pacejka_calibration`
  and `control_params` blocks; existing fields are untouched. A v2 driver JSON
  with no v3 blocks runs in v2 mode without modification.
- **Track CSV / setup JSON / lake schema changes.** All unchanged.
- **MF4 telemetry output for v3.** Inherits §18 v2-PLANNED status — v3.1.

---

## §23.3 Module layout

The existing `src/lap_estimator/simulator.py` stays. Its module docstring
gains one header line:

```
Point-mass kinematic model (v2 branch). For the slip-based dynamics model
see `src/lap_estimator/dynamics/`.
```

New package `src/lap_estimator/dynamics/`:

| File | Role | Soft LoC target |
|---|---|---|
| `__init__.py` | Re-exports `simulate_slip`, `simulate_stint_slip`, `SlipSimResult`. | <50 |
| `pacejka.py` | Magic Formula tyre force calculator. `pacejka_fy(alpha, Fz, coeffs) -> Fy`, `pacejka_fx(kappa, Fz, coeffs) -> Fx`, `combined_friction_ellipse(Fx, Fy, mu, Fz) -> (Fx', Fy')`. Pure functions, NumPy-vectorised. | ~250 |
| `vehicle.py` | `VehicleState` dataclass (10-element state vector — §23.6), `compute_derivatives(state, controls, car, compound) -> dstate/dt`, weight-transfer (longitudinal + lateral, quasi-static), Ackermann steering, per-wheel slip-angle / slip-ratio geometry. | ~350 |
| `solver.py` | Time-domain integrator. Default: hand-rolled RK4 at fixed `dt = 5 ms`. Optional: `scipy.integrate.solve_ivp` with `method="LSODA"` for stiff regimes (corner exit at low speed). Selection via `solver: "rk4" \| "lsoda"` kwarg, default `"rk4"`. Lap-completion detection via `normalizedCarPosition` crossing 1.0 (nearest-point on track ideal-line, same logic as v2's `Track.to_points`). Stall guard: abort with `StalledError("v_x < 0.5 m/s for >2 s")` if speed collapses. | ~350 |
| `driver_controller.py` | `DriverController` class. Preview-target steering (P-controller on cross-track error to a point `preview_distance_m` ahead on the racing line), throttle P-controller on `(target_speed_at_preview - current_speed)`, brake P-controller (same error, negative branch). `skill_pct` modulates `slip_target_lat_deg` (1.0 → 6.0°, 0.5 → 3.0°). `consistency_sigma` adds Gaussian noise to control outputs (replaces v2's grip-noise approach for slip-model Monte Carlo). | ~250 |
| `pacejka_fit.py` | Fits `(B, C, D, E)` Magic Formula coefficients per axle per compound from lake telemetry. Writes the result to `drivers/<name>.json` under `pacejka_calibration`. Five-stage algorithm — see §23.5. | ~450 |
| `slip_simulator.py` | Top-level `simulate_slip(car, track, driver, setup, *, n_laps, ...) -> SlipSimResult` and `simulate_stint_slip(...)`. Wires `VehicleState` + `solver` + `DriverController` + `tyre_state.update_segments_in_place` together. Single-lap mode delegates to one solver pass; stint mode loops per lap with end-of-lap → start-of-lap state passthrough (mirrors v2's `simulate_stint` skeleton). | ~400 |

All files stay under the project-wide 500-line soft ceiling. If `vehicle.py` or
`pacejka_fit.py` grow past that during build-out, ArchDev splits along the
suggested seams: `vehicle.py` → `vehicle_state.py` + `weight_transfer.py` +
`tyre_geometry.py`; `pacejka_fit.py` → `slip_inversion.py` +
`force_inversion.py` + `mf_lsq_fit.py`.

**Imports stay one-directional:** `slip_simulator.py` → `solver.py` →
`vehicle.py` → `pacejka.py`. `driver_controller.py` is consumed by
`solver.py` (controller called once per integration step to update steering /
throttle / brake from chassis state). `tyre_state.py` is imported by
`slip_simulator.py` only — neither the ODE solver nor Pacejka know about
wear/temp.

**Dependency surface.** `numpy` (already present) + `scipy` (already present
in `web/requirements.txt` per §22, lifted to repo-root requirements for v3).
No new third-party packages.

---

## §23.4 CLI + UI surface changes

### 23.4.1 CLI (`lap.py`)

New flag: `--model {point-mass, slip}`, default `point-mass`.

- `point-mass` (default) → dispatches to existing `simulate(...)` /
  `simulate_stint(...)`. Bit-for-bit identical output to today.
- `slip` → dispatches to `simulate_slip(...)` / `simulate_stint_slip(...)`
  from `src/lap_estimator/dynamics/`.

Compatibility with existing flags:
- `--single-lap`, `--laps N`, `--validate-against`, `--solve-pressure-for-wear`,
  `--compound`, `--setup`, `--pressure`, `--ambient-temp-c` all continue to
  work. Where v3 doesn't yet support a flag (inverse-PSI solver is v3.1), the
  flag raises a `NotImplementedError` with a clear "use `--model point-mass`
  for inverse-PSI in v3.0" message.
- Output filenames gain a `_slip` suffix in slip mode to keep v2 and v3 traces
  side-by-side without overwriting: `<track_stem>__<driver>_sim_trace_slip.csv`,
  `<track_stem>__<driver>_sim_vs_ai_slip.png`,
  `<track_stem>__<driver>_sim_telemetry_slip.csv`. The
  `.gitignore` sim-output ignore rules in §6.9 are extended with
  `*_sim_trace_slip.csv`, `*_sim_vs_ai_slip.png`,
  `*_sim_telemetry_slip.csv`, `*_validation_bins_slip.csv`,
  `*_validation_overlay_slip.png`.

### 23.4.2 Web UI (§22 Sim tab — additive)

**New picker on the Sim tab (`web/static/sim.js` per §22.D.1):**

- **Model:** `<select>` with two options:
  - `Point-mass (v2 — default, fast)` → value `point-mass`.
  - `Slip-based dynamics (v3 — slower, honest physics)` → value `slip`.
  - Selecting `slip` while the chosen driver has no `pacejka_calibration`
    block disables the Run button and surfaces an inline hint: *"This driver
    has no Pacejka calibration. Fit it from lake telemetry on the Driver tab
    first."*

**New panel on the Driver tab (`web/static/driver.js` per §22.D.2 view mode):**

- When the loaded driver JSON has a `pacejka_calibration` block, render a
  collapsible **"Slip model parameters"** section under the existing key-stat
  summary. Contents (read-only):
  - Per-axle `(B, C, D, E)` table.
  - Combined-slip ellipse coefficient.
  - `control_params` table (preview distance, P gains, slip-target).
  - Fit metadata: `fit_version`, `fit_date`, `source_laps`, RMSE of fit
    against the calibration cloud (lat + long, separately), R² per axle.
- When the loaded driver has no `pacejka_calibration`, the panel is hidden
  entirely (not "greyed out" — fully absent). Mirrors §22.D.2's existing
  "section-when-present" idiom.

**New section on Driver tab fit-mode (post-§22.D.2):**

- "Fit new driver" gets a checkbox: **"Also fit Pacejka coefficients (slip
  model)"**, default unchecked. When checked, the fitter additionally runs
  the v3 `pacejka_fit.py` pipeline after the standard v2 fit completes, and
  writes both calibration blocks to the same `drivers/<name>.json`. Each
  block's `source` records what was fit and when, independently.

The §22 dropdown additions are non-breaking — defaulting `model=point-mass`
preserves the current §22.I acceptance criteria.

---

## §23.5 Driver JSON schema additions

Both blocks are **optional**. A v2 driver JSON without them is still valid
and runs in v2 mode. The schema additions are top-level siblings of
`tyre_calibration`.

### 23.5.1 `pacejka_calibration` block

```jsonc
{
  "pacejka_calibration": {
    "source": {
      "compound": "Semislicks (SM)",
      "compound_index": 1,
      "fit_version": "v3.0",
      "fit_date": "2026-05-15T12:30:00Z",
      "n_laps": 5,
      "n_samples": 23400,
      "lap_ids": ["abc123", "abc124", "abc125", "abc126", "abc127"],
      "rmse_fy_N": 412.7,
      "rmse_fx_N": 305.1,
      "r2_front_lat": 0.91,
      "r2_rear_lat": 0.88,
      "r2_front_long": 0.84,
      "r2_rear_long": 0.81
    },
    "front_axle": {
      "B_lat": 9.5,  "C_lat": 1.30, "D_lat": 1.55, "E_lat": -0.20,
      "B_long": 10.2, "C_long": 1.65, "D_long": 1.40, "E_long": 0.35
    },
    "rear_axle": {
      "B_lat": 10.1, "C_lat": 1.30, "D_lat": 1.50, "E_lat": -0.18,
      "B_long": 10.8, "C_long": 1.65, "D_long": 1.35, "E_long": 0.30
    },
    "combined": {
      "ellipse_exponent": 2.0   // 2.0 = true ellipse; >2 = blunter (more grip near combined max).
    },
    "measured": true
  }
}
```

- `measured: false` indicates hand-default coefficients (chosen so the slip
  model produces a sane lap for a typical road tyre when no fit has been run).
  Defaults: front/rear identical, `B=10, C=1.3, D=1.5, E=-0.2` for lat;
  `B=10, C=1.65, D=1.4, E=0.3` for long; `ellipse_exponent=2.0`.
- Coefficients are **per-compound on disk** in v3.1, but v3.0 stores only the
  *active compound* the fit was run against. The `source.compound` field
  records which one. Running `--model slip --compound <other>` against a
  driver fit on a different compound is an error in v3.0 ("Driver
  pacejka_calibration was fit against Semislicks; cannot run against Street.
  Refit with `--compound Street` or use `--model point-mass`."). v3.1 lifts
  this to a `compound → coefficients` mapping.
- The `D_lat` / `D_long` coefficients are *Pacejka's mu peak* and are tied
  to load. The fitter normalises against the segment's `Fz`, so the JSON
  carries `D_lat_per_kN` style normalised values when `D_LAT_NORMALISED` is
  set true under `source` — but for v3.0, we store the raw peak at the
  fit's median `Fz` and accept the load-coupling error as documented
  simplification. Full load-normalised Pacejka (`D(Fz) = a1·Fz + a2·Fz²`) is
  v3.1.

### 23.5.2 `control_params` block

```jsonc
{
  "control_params": {
    "preview_distance_m": 18.0,
    "steering_p_gain": 1.2,
    "throttle_p_gain": 0.5,
    "brake_p_gain": 0.6,
    "slip_target_lat_deg": 6.0,
    "consistency_noise_std_steer_deg": 0.3,
    "consistency_noise_std_throttle_pct": 1.5,
    "measured": false
  }
}
```

- Defaults are hand-tuned (v3.0). Values listed above are the v3.0 hand
  defaults. `measured: false` flags that no fitter pass has refined them.
- v3.1 (backlog): `pacejka_fit.py` learns these from telemetry — preview
  distance from steering-lead correlation, P gains from input-vs-error
  regression, slip target from observed steady-state cornering slip-angle
  distribution.
- `consistency_noise_std_*` replaces v2's `consistency_sigma` for slip-model
  Monte-Carlo. v2's scalar `consistency_sigma` is still read for the v2 path
  unchanged; the v3 path *additionally* consumes the new per-channel stds
  when present, else derives them as `consistency_sigma · {0.5°, 2%, 2%}`
  for steer/throttle/brake.

---

## §23.6 ODE solver — state vector & force model

### 23.6.1 State vector (10 variables)

```
state = [x, y, psi, v_x, v_y, omega_yaw, omega_FL, omega_FR, omega_RL, omega_RR]
```

| Symbol | Meaning | Units |
|---|---|---|
| `x, y` | Chassis CG position in track frame | m |
| `psi` | Chassis heading (yaw angle) | rad |
| `v_x, v_y` | Velocity in chassis body frame (longitudinal, lateral) | m/s |
| `omega_yaw` | Yaw rate | rad/s |
| `omega_FL..RR` | Wheel angular speed (rotation) | rad/s |

Initial state at lap start: `x, y` = first track-CSV ideal-line point; `psi` =
ideal-line tangent at that point; `v_x = v_initial` (0 on lap 1, end-of-lap
N-1 velocity for lap N ≥ 2 — same universal rule as §21.4 / Decisions item 26);
`v_y = 0`; `omega_yaw = 0`; `omega_w = v_initial / R_tyre` for all 4 wheels.

### 23.6.2 Force model per integration step

Inputs: current `state`, current `controls = (steer_deg, throttle_pct,
brake_pct)`, `car` (mass, wheelbase, track widths front/rear, cg height,
cg_front, drive_type, brake_front_share, max_torque(omega_engine),
max_brake_torque, Cd·A, Cl·A, tyre_radius, etc.), `compound` (active
`Compound` per §21.11 — for the friction-ellipse `mu` scaling that
`f_pressure_grip` / `f_temp` / `f_wear` modulate per wheel via current
`TyreState`).

Computation order:

1. **Per-wheel steering angle** (Ackermann).
   - Inner-wheel angle: `delta_inner = atan(L / (L/tan(delta_avg) - track_f/2))`.
   - Outer-wheel angle: `delta_outer = atan(L / (L/tan(delta_avg) + track_f/2))`.
   - Rear wheels: `delta_RL = delta_RR = 0` (no rear steer).
2. **Per-wheel velocity at contact patch** (body frame):
   - `v_w_x[FL] = v_x - omega_yaw · (track_f/2)`, etc. — four (`v_w_x`, `v_w_y`) tuples accounting for chassis yaw rate × wheelbase / track offsets.
3. **Per-wheel slip angle** `alpha[w]`:
   - `alpha[w] = delta[w] - atan2(v_w_y[w], v_w_x[w])`.
4. **Per-wheel slip ratio** `kappa[w]`:
   - `kappa[w] = (omega_w[w] · R_tyre - v_w_x[w]) / max(|v_w_x[w]|, 0.5)`.
   - 0.5 m/s denominator floor avoids singularity at corner-exit low speed
     (also the stall-guard trigger threshold — see §23.6.5).
5. **Quasi-static weight transfer** → per-wheel normal load `Fz[w]`.
   - Static: `Fz_static_front = m · g · (1 - cg_front)`, `Fz_static_rear = m · g · cg_front`.
   - Longitudinal: `dFz_long = m · a_x · (h_cg / wheelbase)`. Subtract from front pair, add to rear pair (negate for braking).
   - Lateral: `dFz_lat_front = m · a_y · (h_cg / track_f) · front_load_share`. Subtract from inside pair, add to outside pair. (Inside/outside derived from sign of `omega_yaw`.)
   - Aero downforce: `F_down = 0.5 · rho · v_x² · ClA`. Split by `cg_front`.
   - Final `Fz[w]` clamped to `[100 N, +inf)` — a wheel can't carry less than 100 N even mid-lift; below that it's effectively airborne and Pacejka returns zero force.
6. **Pacejka per wheel** (`pacejka.py`):
   - `Fy_tyre[w] = pacejka_fy(alpha[w], Fz[w], coeffs[axle])`.
   - `Fx_tyre[w] = pacejka_fx(kappa[w], Fz[w], coeffs[axle])`.
   - `mu_w = compound.f_pressure_grip(p[w]) · compound.f_temp(T[w]) · compound.f_wear(wear[w])` — per-wheel grip multiplier from the current `TyreState`. Applied as `D_eff = D · mu_w` inside the Pacejka call (D scales linearly with mu).
   - **Combined-slip friction ellipse:** `(Fx_tyre, Fy_tyre)` clamped onto an ellipse of semi-axes `(D_x · Fz, D_y · Fz)` with exponent `combined.ellipse_exponent`. If `((Fx/(D_x·Fz))^n + (Fy/(D_y·Fz))^n) > 1`, scale both proportionally to land on the boundary.
7. **Rotate per-wheel tyre forces to chassis body frame:**
   - `Fx_body[w] = Fx_tyre[w] · cos(delta[w]) - Fy_tyre[w] · sin(delta[w])`.
   - `Fy_body[w] = Fx_tyre[w] · sin(delta[w]) + Fy_tyre[w] · cos(delta[w])`.
8. **Sum forces and moments:**
   - `F_x_total = sum(Fx_body[w]) - 0.5 · rho · v_x² · CdA · drag_scale` (drag scale comes from the active TyreState per §21.3 v1.3).
   - `F_y_total = sum(Fy_body[w])`.
   - `M_z_total = (Fx_body[FL] + Fx_body[FR]) · (wheelbase · (1 - cg_front))`
     `- (Fx_body[RL] + Fx_body[RR]) · (wheelbase · cg_front)`
     `+ (Fy_body[FR] - Fy_body[FL]) · (track_f / 2)`
     `+ (Fy_body[RR] - Fy_body[RL]) · (track_r / 2)`.
9. **Wheel torque** (engine + brake, per wheel):
   - Engine torque available: `tau_engine = car.max_torque(omega_engine)`.
   - omega_engine derived from `omega_w` of the driven wheels via `final_drive` (read from `drivetrain.ini`).
   - Drive split: RWD → split between RL/RR equally (open-diff assumption,
     §23.2 non-goal flags real diff modelling); FWD → FL/FR; AWD → all four
     via `awd_front_share` from `drivetrain.ini`.
   - Brake torque: `tau_brake[w] = brake_pct · car.max_brake_torque · brake_share[axle]`.
   - Net wheel torque: `tau_w[w] = tau_engine[w] - tau_brake[w] - Fx_tyre[w] · R_tyre`.
10. **State derivatives:**
    - `dx/dt = v_x · cos(psi) - v_y · sin(psi)`.
    - `dy/dt = v_x · sin(psi) + v_y · cos(psi)`.
    - `dpsi/dt = omega_yaw`.
    - `dv_x/dt = F_x_total / m + v_y · omega_yaw`.
    - `dv_y/dt = F_y_total / m - v_x · omega_yaw`.
    - `domega_yaw/dt = M_z_total / I_zz` (`I_zz` from `car.ini`, fallback to `m · ((wheelbase² + track_f²) / 12)`).
    - `domega_w/dt = tau_w[w] / I_wheel` (`I_wheel` from `tyres.ini` `[FRONT_n].RATE` if present, fallback hand-default `1.2 kg·m²`).

### 23.6.3 Integration cadence

- Default `dt = 5 ms` (200 Hz) — chosen to comfortably resolve the
  ~50 ms slip-angle response time of a road car without over-sampling.
- RK4 hand-rolled is the default solver. Six evaluations per step
  (`f(state)` + 4 stages + post-step Pacejka cache) — well within budget for
  a ~120 s lap × 200 Hz = 24k steps × ~few µs each = sub-second sim.
- `solver="lsoda"` (scipy `solve_ivp(method="LSODA")`) is an optional path
  for cases where the corner-exit-low-speed regime makes RK4 unstable. LSODA
  switches automatically between non-stiff and stiff regimes. ~5–10× slower
  than RK4 in practice; recommend as a fallback, not a default.

### 23.6.4 Lap completion detection

Every step, project current `(x, y)` onto the track ideal-line (same
nearest-neighbour logic as v2's `Track.to_points`) → `s_along_track` →
`normalized_position = s_along_track / track_length`. Lap completes when
`normalized_position` crosses 1.0 (with hysteresis: must have been below 0.5
within the last second to ignore start-line jitter).

Stint mode: at lap completion, snapshot `(v_x, v_y, omega_yaw, omega_w[w])`
as the lap-N→lap-(N+1) carry-over; `x, y, psi` reset to the ideal-line start
point (a small kinematic discontinuity at the start-finish line — acceptable
because the universal rule across both v2 and v3 stints is "v_end of lap N =
v_start of lap N+1"). Per-wheel `TyreState` is updated via
`tyre_state.update_segments_in_place` over the lap's integrated trajectory,
with **slip-energy attribution from Pacejka** rather than the v2 `v²/R`
proxy:
- `dE[w] += |Fx_tyre[w] · kappa[w] · v_w_x[w]| · dt + |Fy_tyre[w] · v_w_y_slip[w]| · dt`,
  where `v_w_y_slip[w] = v_w_x[w] · tan(alpha[w])`.
- This `dE[w]` feeds directly into `tyre_state.update_segments_in_place`'s
  thermal Euler step (§21.3 step 2) and wear update (step 3). No code change
  required in `tyre_state.py`; v3 just supplies a more accurate `dE`.

### 23.6.5 Numerical-stability guards

- **Stall guard.** If `v_x < 0.5 m/s` for more than 2 s of integrated time,
  abort with `StalledError("Vehicle stalled at t=<t>, s=<s_along_track> —
  controller failed to track ideal line")`. Surfaces cleanly via CLI; UI
  catches and shows a banner.
- **Divergence guard.** If `|omega_yaw| > 5 rad/s` (spinning), abort with
  `SpunError(...)`. v3.0 does not yet recover from spins.
- **NaN guard.** Any NaN in `state` after an RK4 step → abort with
  `NumericalError(...)` and dump the last 100 steps to
  `<repo>/.tmp/slip_solver_nan_dump_<timestamp>.csv` for ArchDev debugging.

---

## §23.7 Pacejka coefficient fit (`pacejka_fit.py`)

This is the hardest sub-feature. The fit takes lake telemetry (the per-wheel
and body-frame channels Buddy documented as v3 prerequisites — see
`memory/reference_ac_telemetry_schema.md`) and returns
`(B, C, D, E)` per axle per friction direction.

### 23.7.1 Required input channels

Per-row in the merged lake frame (all available in AC telemetry):

| Channel | Used for |
|---|---|
| `wheelSlipFL/FR/RL/RR` | AC's pre-computed slip-ratio — cross-check. |
| `wheelLoadFL/FR/RL/RR` | `Fz[w]` directly. |
| `wheelAngularSpeedFL/FR/RL/RR` | `omega_w[w]`. |
| `localVelocity_x` | Body-frame longitudinal velocity. |
| `localVelocity_y` | Body-frame lateral velocity. |
| `localVelocity_z` | Vertical velocity (sanity — should be ~0). |
| `tyreContactHeadingFL/FR/RL/RR` | Per-wheel steer angle. |
| `steerAngle` | Cross-check for averaged front steer. |
| `accG_x` / `accG_y` / `accG_z` | Chassis-frame acceleration. |
| `worldRotationYaw` (or numerical derivative of `worldPositionYaw`) | `omega_yaw`. |
| `engineRPM`, `gear` | Drivetrain coupling sanity. |
| `tyreCompound` | Active compound (§21.11 selector). |
| `lap_id`, `lap_time_progress` | Per-lap segmentation, ignore in/out laps. |

If any required channel is missing, the fitter prints which lap is missing
which channel and falls back to hand defaults (`measured: false` in the
output JSON).

### 23.7.2 Algorithm — five stages

#### Stage A — Invert per-wheel slip-angle α and slip-ratio κ from geometry

For each timestep, for each wheel:

1. Compute wheel-contact-patch velocity in body frame:
   - `v_w_x[w] = localVelocity_x - omega_yaw · y_offset[w]`
   - `v_w_y[w] = localVelocity_y + omega_yaw · x_offset[w]`
   - Where `(x_offset[w], y_offset[w])` is wheel position in body frame from
     `car.ini` geometry (`WHEELBASE`, `TRACK_FRONT`, `TRACK_REAR`, `CG_LOCATION`).
2. Slip angle: `alpha[w] = tyreContactHeading[w] - atan2(v_w_y[w], v_w_x[w])`.
3. Slip ratio: `kappa[w] = (wheelAngularSpeed[w] · R_tyre - v_w_x[w]) / max(|v_w_x[w]|, 0.5)`.
4. Cross-check `kappa[w]` against `wheelSlip[w]` — RMS difference should be
   <0.02. Log warning if not.

#### Stage B — Invert per-wheel forces (Fy, Fx) from chassis dynamics

The Pacejka fit needs `(alpha, Fz, Fy)` and `(kappa, Fz, Fx)` clouds. AC
gives `Fz` directly via `wheelLoad`; `Fy` and `Fx` per wheel are not
directly logged and must be back-solved from chassis acceleration + yaw
moment.

1. Total chassis-frame force: `(F_x_chassis, F_y_chassis) = m · (accG_x, accG_y) · g`.
2. Yaw moment: `M_z = I_zz · d(omega_yaw)/dt` (numerical derivative of yaw
   rate, low-pass at 10 Hz first to suppress noise).
3. Set up the 4-wheel linear system:
   - `sum(Fx_body[w]) = F_x_chassis + drag_force(v_x)` (drag known from
     `CdA · 0.5 · rho · v_x²`).
   - `sum(Fy_body[w]) = F_y_chassis`.
   - `M_z_body = M_z_chassis` (per §23.6.2 step 8).
   - 3 equations, 8 unknowns (`Fx[w]`, `Fy[w]` per wheel). Under-determined.
4. **Resolution.** Use two physical priors to close the system:
   - Per-wheel `Fy` is **proportional to its slip-angle direction and Fz**
     in the linear regime (this is *what we're fitting*, so we bootstrap
     with a linear `Fy = C_alpha · alpha · Fz` and iterate).
   - Per-wheel `Fx` driven only by driven axles (engine torque split
     equally across driven wheels) + braked axles (brake-share-weighted).
   - Off-axis (non-driven, non-braked) wheels: `Fx[w] = -F_rolling` (small,
     known).
5. **Iterate.** Stage B is run *after* Stage A on the same telemetry. Initial
   guess `Fy[w] = (alpha[w] · Fz[w]) / sum(alpha · Fz) · F_y_chassis`. Refine
   via least-squares against the three chassis-dynamics equations + the four
   per-wheel torque-balance equations. Converges in ~5 iterations on real
   laps. The output is a per-row, per-wheel `(Fx[w], Fy[w])` estimate.

#### Stage C — Least-squares fit of Magic Formula coefficients

For each axle, for each direction (lateral, longitudinal), fit:

`Fy_pred = D · Fz · sin(C · atan(B·alpha - E·(B·alpha - atan(B·alpha))))`

(And the symmetric form for `Fx_pred(kappa, Fz)`.)

- Variables: `(B, C, D, E)` — 4 per (axle, direction) = 16 total.
- Objective: minimise `sum_t (Fy_pred[t] - Fy_observed[t])²`, weighted by
  `1 / max(Fz[t], 1000)` to avoid low-load corners dominating.
- Solver: `scipy.optimize.least_squares` with `method="trf"` (trust-region
  reflective, handles bounds). Bounds:
  - `B ∈ [3, 20]` (stiffness factor — typical road tyre 8–12).
  - `C ∈ [1.0, 2.0]` (shape factor — typical 1.3 lat, 1.65 long).
  - `D ∈ [0.5, 2.5]` (peak factor / mu — typical 1.0–1.6).
  - `E ∈ [-2.0, 1.0]` (curvature factor — typical -0.2 to 0.5).
- Initial guess: hand defaults (`B=10, C=1.3, D=1.5, E=-0.2` lat).
- Reject the fit and fall back to defaults if final RMSE > 25% of mean
  `|Fy_observed|`.

#### Stage D — Combined-slip ellipse exponent fit

Once `(B, C, D, E)` are fit per axle per direction, run all combined-slip
samples (where `|alpha| > 1°` AND `|kappa| > 0.02` — both lat and long
loaded simultaneously) and fit the friction-ellipse exponent `n` to:

`((Fx_observed / (D_x·Fz))^n + (Fy_observed / (D_y·Fz))^n) = 1`

Single-parameter scalar fit. `n=2.0` is true ellipse; real tyres often
fit `n=2.2–2.5` (blunter — more grip near combined max). Bounds `n ∈ [1.5, 4.0]`.

#### Stage E — Cross-validation

Hold out the newest lap (the last one of the lap-id ordering used). Re-derive
predicted `(Fy[w], Fx[w])` from observed `(alpha[w], kappa[w], Fz[w])` through
the fitted Pacejka. Compare to observed `(Fy[w], Fx[w])` from Stage B on
the held-out lap. Acceptance:

- RMSE(Fy_pred − Fy_observed) ≤ 10% of `mean(|Fy_observed|)` on held-out lap.
- RMSE(Fx_pred − Fx_observed) ≤ 15% (longitudinal is noisier — drivetrain coupling).
- `R²` per axle ≥ 0.85 lat, ≥ 0.75 long.

If cross-validation fails, write the calibration but mark `source.measured =
true, source.cv_passed = false` and surface a warning. CLI exit code 0, UI
shows a yellow banner.

### 23.7.3 Output

```bash
fit_driver.py <lap1.csv> <lap2.csv> ... --output drivers/tomas.json --fit-pacejka
```

Adds the `pacejka_calibration` block (§23.5.1) to the existing
`drivers/tomas.json`. The v2 fit (skill, tyre_calibration, etc.) is still
written too — the two pipelines run in sequence on the same input laps. Both
blocks land in one file.

Web UI: see §23.4.2 "Also fit Pacejka coefficients" checkbox.

---

## §23.8 Driver controller (`driver_controller.py`)

### 23.8.1 Inputs

Per integration step, given chassis state and the racing-line lookup table
from the track CSV:

1. **Preview point.** Project current `(x, y)` onto the racing line → current
   `s_along_track`. Read the racing-line point at
   `s_preview = s_along_track + preview_distance_m`. This is `(x_p, y_p)` in
   the track frame, plus `target_speed_p` (read from the `speed_ms` column
   of the track CSV).
2. **Cross-track error** to preview point, signed (positive = preview is to
   the left of the chassis nose):
   - `dx = x_p - x, dy = y_p - y`.
   - `e_lat = -dx · sin(psi) + dy · cos(psi)`.
3. **Speed error:** `e_v = target_speed_p - v_x`.

### 23.8.2 Controllers

- **Steering** (P-controller on cross-track error, bounded):
  - `steer_deg = clip(steering_p_gain · e_lat, -max_steer_deg, +max_steer_deg)`.
  - `max_steer_deg` from `car.ini` (`STEER_LOCK`).
- **Throttle** (P-controller, positive branch only):
  - `throttle_pct = clip(throttle_p_gain · max(e_v, 0), 0, 1)`.
- **Brake** (P-controller, negative branch only):
  - `brake_pct = clip(brake_p_gain · max(-e_v, 0), 0, 1)`.
- **Skill modulation:** the target slip-angle window is `slip_target_lat_deg`,
  derived from `skill_pct`:
  - `skill_pct = 1.0` → `slip_target = 6.0°` (peak lateral grip on a typical tyre).
  - `skill_pct = 0.5` → `slip_target = 3.0°` (linear regime — under-driven).
  - Linear interpolation in between.
  - The controller doesn't *enforce* the slip target directly; instead it
    scales `target_speed_p` by `(slip_target / 6.0)^0.5` — a lower slip
    target means the controller seeks lower cornering speeds (because it
    can't extract the peak Pacejka mu).
- **Consistency noise:** Gaussian noise per channel from `control_params`:
  - `steer_deg += N(0, consistency_noise_std_steer_deg)`.
  - `throttle_pct += N(0, consistency_noise_std_throttle_pct / 100)`.
  - `brake_pct += N(0, consistency_noise_std_throttle_pct / 100)`.
  - Same RNG seed across Monte Carlo runs to make results reproducible — the
    seed lives on the `SlipSimResult`.

### 23.8.3 Driver-controller-only vs free-driving distinction

v3.0 ships **preview-line-following only**. The controller does not "race"
the line — it tries to track `target_speed_p`, which is the racing-line's
recorded speed (centerline or ideal). It can't *exceed* the racing line's
pace, only fall short. This is the deliberate v3.0 simplification: the
v2 model already gives us the best-case lap time; v3 gives us *honest physics
under that best-case driver-line assumption*.

v3.1 backlog: replace the throttle / brake P-controllers with a proper
minimum-time controller (MPC or a single-step DP optimum) that chooses
target_speed_p instead of being given it. That's the "v3 also predicts
which line to take" use case.

---

## §23.9 Phasing — five phases, with acceptance gates

Each phase has a single **acceptance gate**. Phase N+1 does not start until
Phase N's gate passes. Buddy hands one phase at a time to ArchDev — Phase 1
brief is at the bottom of this file.

### Phase 1 — Skeleton + dispatcher (~2 hours ArchDev)

**Scope:**
- Create `src/lap_estimator/dynamics/` with all 7 files, each containing
  module docstring + class/function signatures + `raise NotImplementedError`
  bodies.
- `simulate_slip(...)` and `simulate_stint_slip(...)` raise
  `NotImplementedError("v3 phase 1: dynamics module skeleton in place")`.
- `lap.py` gains `--model {point-mass, slip}`, default `point-mass`.
- `--model slip` surfaces the `NotImplementedError` cleanly (with
  human-readable wrapping, not a traceback).
- Web UI Sim tab dropdown landed but **disabled** (the `slip` option is
  rendered but `<option disabled>`). Hint: *"Coming soon — Phase 1
  scaffolding only."*

**Acceptance gate:** `pytest tests/dynamics/test_phase1_skeleton.py` passes,
verifying (a) `--model point-mass` produces byte-identical output to today,
(b) `--model slip` exits non-zero with the expected error message, (c) the
package imports cleanly with no `ImportError`.

### Phase 2 — Pacejka fit (~1 day ArchDev)

**Scope:**
- `pacejka.py` — implement `pacejka_fy`, `pacejka_fx`,
  `combined_friction_ellipse`.
- `pacejka_fit.py` — implement Stages A through E (§23.7.2).
- `fit_driver.py` gains `--fit-pacejka` flag; writes the
  `pacejka_calibration` block to the driver JSON.
- Web UI driver-tab "Also fit Pacejka coefficients" checkbox wired to the
  `--fit-pacejka` code path.

**Acceptance gate:** running `fit_driver.py samples/aclog/tomas_lake_*.csv
--fit-pacejka --output drivers/tomas.json` against Tomas's five lake laps:
- (a) writes a valid `pacejka_calibration` block,
- (b) Stage E cross-validation passes (RMSE thresholds, R² thresholds),
- (c) `pytest tests/dynamics/test_pacejka.py` passes — verifies that for
  fixed `(B, C, D, E)`, `pacejka_fy(0, Fz, ...) = 0`, `pacejka_fy(alpha,
  Fz, ...)` is monotonic up to peak then decreases (E < 0 sense check),
  and combined-ellipse clamping returns a point on the ellipse boundary.

### Phase 3 — ODE solver (~1 day ArchDev)

**Scope:**
- `vehicle.py` — full `VehicleState` + `compute_derivatives`.
- `solver.py` — RK4 + lap-completion detection + stall/divergence/NaN guards.
- `slip_simulator.py` — `simulate_slip` that uses a **hard-coded ghost
  driver** (no controller yet — just feeds back the racing-line's
  recorded speed as the throttle/brake target, with a P-controller proxy
  on cross-track error for steering, with hand-tuned gains).
- `--model slip` runs end-to-end; UI dropdown enabled.

**Acceptance gate:** running `lap.py cars_csv/Volvo_AMG ... drivers/tomas.json
... --model slip` on Sprint A:
- (a) completes without raising `StalledError` / `SpunError` / `NumericalError`,
- (b) lap-time within ±10% of the v2 point-mass lap time on the same inputs
  (sanity — they're different physics but should agree on this car/track to
  first order),
- (c) `util_p85` honestly computed (no clamping in the Pacejka call) is ≤ 1.05
  — first taste that v3 fixes the v2 grip envelope bias.

### Phase 4 — Driver controller (~half day ArchDev)

**Scope:**
- `driver_controller.py` — full controller per §23.8.
- `slip_simulator.py` swaps the ghost driver for `DriverController`.
- `skill_pct` mapped to `slip_target_lat_deg` per §23.8.2.
- `consistency_noise_*` consumed for Monte-Carlo branch.
- `control_params` block read from driver JSON when present; defaults
  otherwise.

**Acceptance gate:** running `lap.py ... --model slip` on Sprint A with
Tomas at `skill_pct=1.0` produces a lap time within ±3 s of his real lake
average (the headline cross-track validation criterion). At `skill_pct=0.5`
the lap time is meaningfully slower (≥3 s slower than skill=1.0) — proving
the controller responds to skill modulation.

**Phase 4 status (2026-05-18) — DID NOT PASS:** Tomas Sprint A real = 1:47.56,
v3 = 2:09.46 (Δ +21.9 s). With diagnostic D_lat=1.28 override, v3 = 2:00.92
(Δ +13.4 s — still outside the ±3 s gate). The residual gap is the reactive
P-controller architecture, not Pacejka. Phase 4 will not pass without a
controller redesign — scoped in **§23.10 v3.1 controller upgrade** (Buddy
spec, to be written).

### Phase 5 — UI polish + validation overlay (~2 hours, ArchDev + FrontEndEsthetic)

**Scope:**
- Web UI Sim tab dropdown fully enabled, with the inline guard ("driver has
  no pacejka_calibration — go fit it first") wired up.
- Driver tab "Slip model parameters" panel rendered when block is present.
- Telemetry overlay PNG (`<track>__<driver>_sim_vs_ai_slip.png`) includes
  three traces side-by-side: v2 sim, v3 sim, real lake (if available).
- `_slip` suffix on all v3 output files; `.gitignore` rules added.

**Acceptance gate:** end-to-end demo run with v2 vs v3 vs real on the same
plot, lap-time table, util curve. Approval from user; no code-level
acceptance test.

### Total wall-clock estimate

**~2 days at AI-pace, ~1 week at human-pace.** Phase 2 is the long pole;
Phases 1, 4, 5 are short. Phase 3 carries the most numerical-stability
risk (`StalledError` / `SpunError` shaking out in the first integration
runs).

---

## §23.10 Validation criteria (§23.acceptance, additive to §11)

These extend the existing acceptance criteria block (§11.30 — single-lap
regression, §11.47/48 — stint continuity, etc.). They are checked under
the §23 work and gated per phase as above.

- **§11.50 (Phase 1):** `--model point-mass` is byte-equivalent to today's
  output for the §11.30 / §11.31 reference inputs (Sprint A + Tomas).
- **§11.51 (Phase 1):** `--model slip` exits non-zero with the v3-phase-1
  `NotImplementedError` message; UI dropdown shows the disabled option.
- **§11.52 (Phase 2):** `fit_driver.py --fit-pacejka` against Tomas's five
  lake laps writes a `pacejka_calibration` block with `source.cv_passed =
  true` (RMSE Fy ≤ 10%, R² ≥ 0.85 lat; RMSE Fx ≤ 15%, R² ≥ 0.75 long).
- **§11.53 (Phase 2):** A round-trip sanity check — synthesise a (alpha, Fz,
  Fy) cloud from known coefficients `(B*, C*, D*, E*)`, fit it, recover
  `(B, C, D, E)` within 5% of truth on each.
- **§11.54 (Phase 3):** `--model slip` on Sprint A + Tomas (ghost driver)
  completes without abort guard tripping; lap time within ±10% of the v2
  reference.
- **§11.55 (Phase 4 — headline):** Tomas on Sprint A `--model slip
  --skill-pct 1.0` predicts a lap time within ±3 s of his real lake average,
  and `util_p85 ≤ 1.0` honestly (no clamping). This is *the* test that v3
  fixes the v2 grip-envelope bias.
- **§11.56 (Phase 4):** Same as §11.55 but at `--skill-pct 0.5` is ≥ 3 s
  slower than `--skill-pct 1.0`. Confirms controller responds to skill.
- **§11.57 (Phase 5):** Web UI Sim tab end-to-end: select Tomas + Sprint A +
  Volvo_AMG + Model=Slip → Run → renders the validation overlay PNG with
  v2/v3/real traces. Driver tab shows the "Slip model parameters" panel
  with non-empty contents.

The **headline** is §11.55 — that's the criterion that justifies the entire
v3 build.

---

## §23.11 What v3 explicitly does NOT change

To make the parallel-track posture crisp:

- **Driver JSON top-level fields** (`skill_pct`, `consistency_sigma`,
  `profile.dynamic`, `tyre_calibration`) — unchanged. v3 *adds*
  `pacejka_calibration` + `control_params` blocks as siblings.
- **Setup JSON** (`compound`, `pressure_*`, `ambient_temp_c`) — unchanged.
- **Track CSVs** — unchanged. v3 reads the same `speed_ms` and
  ideal-line columns.
- **Lake schema** — unchanged. v3 reads channels that are already there
  (per `reference_ac_telemetry_schema.md`).
- **UI tabs** — unchanged structure (§22.D.1-D.4 intact). v3 adds a
  dropdown + an optional panel, doesn't restructure.
- **`tyre_state.py` wear/temp/pressure plumbing** — unchanged. v3 reuses
  it. Only the *input slip energy* changes (Pacejka-derived vs v²/R
  proxy).
- **v2 point-mass code path** — unchanged. `simulator.py`,
  `simulate_stint`, `combined_grip_envelope`'s scalar reduction — all
  stay verbatim. v3 lives in a sibling package.
- **CLI surface** for everything except `--model` — unchanged.

---

## §23.12 Decisions block update — proposed §23.L

Add this item to the Decisions block at the bottom of `spec.md`:

> **27. (v3) Slip-based dynamics model — parallel track, opt-in.** The v2
> kinematic point-mass model (`src/lap_estimator/simulator.py`) stays as-is
> and is the **default**. The v3 slip-based dynamics model lives in
> `src/lap_estimator/dynamics/` (Pacejka Magic Formula + 3-DOF planar chassis
> + 4-wheel weight transfer + RK4 ODE integrator + preview-target driver
> controller). Dispatch is via `lap.py --model {point-mass, slip}` (default
> `point-mass`) and a Sim-tab dropdown on the §22 web UI (default
> `point-mass`). Both models consume the same `car` / `track` / `driver`
> / `setup` config schema; v3 adds **optional** `pacejka_calibration` +
> `control_params` blocks to driver JSON. Driver JSONs without these blocks
> still load and run in `--model point-mass`. `fit_driver.py` gains
> `--fit-pacejka` to derive the Pacejka coefficients from lake telemetry via
> a five-stage algorithm (slip-angle/ratio geometric inversion → per-wheel
> force inversion from chassis dynamics + yaw moment → axle-wise
> least-squares Magic Formula fit → combined-slip ellipse exponent fit →
> hold-out cross-validation, §23.7.2). v3 reuses `tyre_state.py` for
> wear/temp/pressure evolution, feeding it Pacejka-derived per-wheel slip
> energy instead of the v2 `v²/R` proxy. v3.0 ships preview-line-following
> only (controller targets the recorded `speed_ms` of the racing line);
> minimum-time controllers (MPC / single-step DP) are v3.1. Aero refinement,
> differential modelling, pit-stop compound switching, driver fatigue —
> v3.1 or out of scope (§23.2). Phasing: Phase 1 skeleton + dispatcher;
> Phase 2 Pacejka fit; Phase 3 ODE solver (ghost driver); Phase 4 driver
> controller; Phase 5 UI + validation overlay (§23.9). Headline acceptance
> §11.55: Tomas on Sprint A `--model slip --skill-pct 1.0` lap-time within
> ±3 s of real, `util_p85 ≤ 1.0` honestly. (§23.1–§23.12, §11.50–§11.57.)

---

## §23.13 References

- `dev-planning/lap-simulation-csv-driver/spec.md` §19 (v3 backlog,
  originating spec).
- `dev-planning/lap-simulation-csv-driver/spec.md` §21 (v2 per-wheel
  tyre state — the plumbing v3 reuses).
- `dev-planning/lap-simulation-csv-driver/spec-section-22-web-ui.md`
  (web UI — §22.D.1 Sim tab gets the Model dropdown; §22.D.2 Driver tab
  gets the Slip model panel).
- `src/lap_estimator/simulator.py` (the v2 3-pass kinematic model — stays
  as-is, gets one new header line in the module docstring).
- `src/lap_estimator/tyre_state.py` (the per-wheel state machinery — v3
  reuses for wear/temp/pressure plumbing).
- `memory/reference_ac_telemetry_schema.md` (per-wheel + body-frame lake
  channels — Pacejka fit prerequisites).
- Hans Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed., Ch. 4 — Magic
  Formula reference.
