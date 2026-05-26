"""Point-mass kinematic model (v2 branch). For the slip-based dynamics model see src/lap_estimator/dynamics/.

Point-mass lap time simulator.

Uses the classic 3-pass approach:
  1. Per-point max cornering speed (lateral grip limit).
  2. Forward pass - acceleration-limited from previous point.
  3. Backward pass - braking-limited into next point.

Then integrates time from the merged speed profile. The merge also produces
a per-point `limit_label` ("corner" / "accel" / "brake") used by the
synthetic-telemetry emitter (sim_telemetry).

v1.1: two-lap "tiled" simulation (spec §20). The segment list is tiled twice;
lap 2's forward-pass initial speed is lap 1's end-of-lap speed (flying start);
output arrays carry a per-point `lap_id` (1 or 2). `times` is monotonic across
the boundary. `--single-lap` (CLI) maps to `two_lap=False`.

v2 (spec §21.4): `simulate_stint(...)` runs an N-lap stint by calling
`simulate(..., two_lap=False)` once per lap and threading per-wheel tyre state
through `tyre_state.update_segments_in_place`. Between laps the state is
reduced to a scalar grip multiplier `(mu_x_scale, mu_y_scale)` that scales the
next lap's solver pass. Back-compat: with `n_laps == 2` and uncalibrated
tyres (calibration.measured == False), `simulate_stint` delegates straight to
the existing `simulate(..., two_lap=True)` path so output is byte-equivalent
to v1.2.1 (spec §11.31, §20 v2 note).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .tyre_state import (
    GripScaledCar,
    TyreCalibration,
    TyreState,
    build_car_tyre_model,
    combined_grip_envelope,
    derive_radius_sign,
    update_segments_in_place,
)


@dataclass
class SimResult:
    lap_time: float  # primary lap time -- lap 2 in two-lap mode, lap 1 in single-lap.
    distances: np.ndarray  # per-lap-relative distance (resets at lap 2 start in two-lap mode).
    speeds: np.ndarray  # m/s, monotonic-time order across both laps.
    times: np.ndarray = field(default_factory=lambda: np.zeros(0))  # monotonic across laps.
    ai_speeds: np.ndarray | None = None
    limit_label: np.ndarray | None = None  # array of str: corner|accel|brake
    sectors: list = field(default_factory=list)
    lap_id: np.ndarray | None = None  # per-point int in {1, 2} (or all 1 in single-lap).
    lap1_time: float = 0.0  # lap 1 lap-time in seconds (0 if single-lap mode and only lap 1).
    lap2_time: float = 0.0  # lap 2 lap-time in seconds (0 if single-lap mode).
    two_lap: bool = False

    @property
    def lap_time_str(self):
        return _fmt_time(self.lap_time)

    @property
    def lap1_time_str(self):
        return _fmt_time(self.lap1_time)

    @property
    def lap2_time_str(self):
        return _fmt_time(self.lap2_time)

    @property
    def max_speed_kph(self):
        return float(np.max(self.speeds) * 3.6)

    @property
    def min_speed_kph(self):
        return float(np.min(self.speeds) * 3.6)

    @property
    def avg_speed_kph(self):
        # Use the primary-lap distance / time (lap 2 in two-lap mode).
        if self.two_lap and self.lap_id is not None:
            mask = self.lap_id == 2
            if mask.any():
                d = self.distances[mask]
                lap_dist = float(d[-1] - d[0])
                return (lap_dist / self.lap_time) * 3.6 if self.lap_time > 0 else 0.0
        total_dist = float(self.distances[-1] - self.distances[0])
        return (total_dist / self.lap_time) * 3.6 if self.lap_time > 0 else 0.0


@dataclass
class MCResult:
    mean_lap_time: float  # lap-2 mean (v1.1: MC is lap-2 only).
    std_lap_time: float
    n_runs: int
    representative: SimResult  # deterministic skill-only run.


def _three_pass(scaled, distances, radii, *, v_init=0.0, v_min=5.0):
    """Run the 3-pass speed solver on a (possibly tiled) distance/radius grid.

    `v_init` is the forward-pass initial speed; for lap 1 (standing) this is 0
    (clamped to v_min*3 floor at the first point, matching existing v1 behaviour),
    for a continuous tiled run lap 2's initial speed is implied by the forward
    pass crossing the lap boundary (we just run forward across the whole grid).

    Returns `(speeds, labels, v_corner, v_forward, v_brake)`.
    """
    n = len(distances)
    # Pass 1: max cornering speed
    v_corner = np.array([scaled.max_cornering_speed(r) for r in radii])
    v_corner = np.clip(v_corner, v_min, 999.0)

    # Pass 2: forward (accel-limited)
    v_forward = np.copy(v_corner)
    # Initial point: clamp to start-of-lap floor (v_min*3) when v_init is small
    # (matches v1 behaviour for standing start). For a non-zero initial speed
    # (e.g. mid-run start), trust the caller.
    if v_init <= v_min * 3:
        v_forward[0] = min(v_corner[0], v_min * 3)
    else:
        v_forward[0] = min(v_corner[0], v_init)
    for i in range(1, n):
        v_prev = v_forward[i - 1]
        d = distances[i] - distances[i - 1]
        if d <= 0:
            v_forward[i] = min(v_forward[i], v_prev)
            continue
        accel = max(scaled.max_accel(v_prev), 0.1)
        v_new = float(np.sqrt(max(v_prev ** 2 + 2 * accel * d, v_min ** 2)))
        v_forward[i] = min(v_corner[i], v_new)

    # Pass 3: backward (brake-limited)
    v_brake = np.copy(v_forward)
    for i in range(n - 2, -1, -1):
        v_next = v_brake[i + 1]
        d = distances[i + 1] - distances[i]
        if d <= 0:
            continue
        decel = max(scaled.max_braking_decel(v_next), 0.1)
        v_new = float(np.sqrt(max(v_next ** 2 + 2 * decel * d, v_min ** 2)))
        v_brake[i] = min(v_brake[i], v_new)

    speeds = v_brake

    # Limit-label: which pass is binding at index i?
    eps = 0.05
    labels = np.empty(n, dtype=object)
    for i in range(n):
        vc, vf, vb = v_corner[i], v_forward[i], v_brake[i]
        v = speeds[i]
        if abs(vc - v) <= eps:
            labels[i] = "corner"
        elif abs(vb - v) <= eps and vb < vf - eps:
            labels[i] = "brake"
        elif abs(vf - v) <= eps and vf < vc - eps:
            labels[i] = "accel"
        else:
            diffs = {
                "corner": abs(vc - v),
                "accel": abs(vf - v),
                "brake": abs(vb - v),
            }
            labels[i] = min(diffs, key=diffs.get)

    return speeds, labels, v_corner, v_forward, v_brake


class _DragScaledCar:
    """Wrap a car (or driver-scaled car) so total drag is multiplied by `drag_scale`.

    v1.3 (spec §21.3 — Solver wiring): `drag_scale` from
    `combined_grip_envelope` scales the total drag force the 3-pass solver
    applies. Drag enters the solver via `max_accel(v)` (= traction − drag − rr)
    and `max_braking_decel(v)` (= (grip_force + drag) / mass), so we scale
    both `drag_force(v)` and `rolling_resistance(v)` at the wrapper. This is
    the natural single-point seam — every caller of `max_accel` /
    `max_braking_decel` (Car, DriverScaledCar, GripScaledCar) ultimately
    delegates the drag computation to `self._car.drag_force / .rolling_resistance`,
    so wrapping the inner car here covers all three.

    `drag_scale == 1.0` is byte-equivalent to the unwrapped car (§11.30).
    """

    def __init__(self, inner, drag_scale: float):
        self._inner = inner
        self._drag_scale = float(drag_scale)

    def __getattr__(self, name):
        return getattr(self._inner, name)

    def drag_force(self, speed_ms):
        return self._drag_scale * self._inner.drag_force(speed_ms)

    def rolling_resistance(self, speed_ms):
        return self._drag_scale * self._inner.rolling_resistance(speed_ms)


def simulate(car, track, driver=None, *, ds=2.0, rng=None, noise=False,
             two_lap=True, drag_scale: float = 1.0, v_initial: float = 0.0):
    """Run a single deterministic (or noisy) lap, optionally as a two-lap tiled run.

    `driver` is optional. When `None`, the raw `car` is used (skill_pct=1.0).
    `two_lap=True` (default, v1.1) tiles the segment list twice and runs a single
    3-pass over the 2x grid; lap 2 inherits lap 1's end-of-lap forward speed
    naturally via the forward pass crossing the boundary.

    v1.3 (spec §21.3 — Drag plumbing): `drag_scale` (kwarg-only) multiplies the
    total drag force (aerodynamic drag + rolling resistance) applied per
    segment in the 3-pass solver. Default `1.0` is byte-equivalent to v2
    (§11.30). When called from `simulate_stint`, the value comes from the
    third return of `tyre_state.combined_grip_envelope` and reflects the
    asymmetric pressure-drag model (`f_pressure_drag`).

    v2.0.1 (spec §21.4 — universal velocity-continuity rule): `v_initial`
    (kwarg-only, default `0.0`) seeds the forward pass's initial speed. Lap 1
    of any stint stays at the default (`0.0` → standing start, identical
    behaviour to v1/v1.1/v1.2/v1.3 and the v1.1 two-lap tile). `simulate_stint`
    passes `v_initial=lap_(N-1).speeds[-1]` for laps 2..N so each lap starts
    where the previous lap ended (flying laps). If `v_initial >= v_corner_max`
    of the first segment, the forward pass naturally caps it at `v_corner[0]`
    inside `_three_pass` — no special handling needed. Default `0.0` preserves
    §11.30 / §11.31 byte-compat with all existing callers.
    """
    # v1.3 drag plumbing (spec §21.3 "Solver wiring"). Apply the drag wrapper
    # to the INNERMOST car so every downstream wrapper (`DriverScaledCar`,
    # `GripScaledCar`) reads the scaled drag via `self._car.drag_force(...)`
    # and `self._car.rolling_resistance(...)`. Wrapping the outer layer would
    # not work — those wrappers delegate to `self._car`, not back through
    # `self`, when computing `max_accel` / `max_braking_decel`.
    if drag_scale != 1.0:
        car_for_sim = _DragScaledCar(car, drag_scale)
    else:
        car_for_sim = car
    if driver is not None:
        scaled = driver.wrap(car_for_sim, rng=rng, noise=noise)
    else:
        scaled = car_for_sim

    distances_lap, radii_lap = track.to_points(ds)
    n_lap = len(distances_lap)
    lap_length = float(distances_lap[-1] - distances_lap[0])
    v_min = 5.0

    if two_lap and n_lap >= 2:
        # Tile: concatenate two copies of the lap grid. Lap 2 distances are
        # shifted by `lap_length` so the forward pass sees a continuous grid;
        # we keep the per-lap-relative `distances_lap` for output.
        # Avoid a zero-step at the seam by offsetting lap 2 by the first step.
        d_step0 = float(distances_lap[1] - distances_lap[0]) if n_lap >= 2 else ds
        d_lap2 = distances_lap + lap_length + d_step0
        distances_full = np.concatenate([distances_lap, d_lap2])
        radii_full = np.concatenate([radii_lap, radii_lap])
        lap_id_full = np.concatenate([
            np.ones(n_lap, dtype=int),
            np.full(n_lap, 2, dtype=int),
        ])
    else:
        distances_full = distances_lap
        radii_full = radii_lap
        lap_id_full = np.ones(n_lap, dtype=int)

    speeds, labels, _, _, _ = _three_pass(
        scaled, distances_full, radii_full, v_init=float(v_initial), v_min=v_min,
    )

    # Integrate time (monotonic across the whole grid).
    n_full = len(distances_full)
    times = np.zeros(n_full)
    total_time = 0.0
    for i in range(1, n_full):
        d = distances_full[i] - distances_full[i - 1]
        if d <= 0:
            times[i] = total_time
            continue
        v_avg = max(0.5 * (speeds[i - 1] + speeds[i]), v_min)
        total_time += d / v_avg
        times[i] = total_time

    if two_lap and n_lap >= 2:
        # Per-lap distance: lap 1 keeps `distances_lap`, lap 2 resets to `distances_lap`.
        distances_out = np.concatenate([distances_lap, distances_lap])
        # Lap 1 lap-time = time at end of lap 1.
        lap1_time = float(times[n_lap - 1])
        lap2_time = float(times[-1] - times[n_lap - 1])
        primary_time = lap2_time
    else:
        distances_out = distances_full
        lap1_time = float(times[-1])
        lap2_time = 0.0
        primary_time = lap1_time

    ai = track.to_ai_reference(ds) if hasattr(track, "to_ai_reference") else None
    if ai is not None and two_lap and n_lap >= 2:
        ai = np.concatenate([ai, ai])

    return SimResult(
        lap_time=float(primary_time),
        distances=distances_out,
        speeds=speeds,
        times=times,
        ai_speeds=ai,
        limit_label=labels,
        lap_id=lap_id_full,
        lap1_time=lap1_time,
        lap2_time=lap2_time,
        two_lap=bool(two_lap and n_lap >= 2),
    )


def simulate_monte_carlo(car, track, driver, *, ds=2.0, n_runs=20, seed=0, two_lap=True):
    """Run N noisy laps; in two-lap mode, MC stats are computed on lap 2 only."""
    rng = np.random.default_rng(seed)
    times = []
    for _ in range(n_runs):
        r = simulate(car, track, driver, ds=ds, rng=rng, noise=True, two_lap=two_lap)
        # lap_time is already the primary (lap 2 in two-lap mode, lap 1 otherwise).
        times.append(r.lap_time)
    times = np.array(times)
    rep = simulate(car, track, driver, ds=ds, two_lap=two_lap)
    return MCResult(
        mean_lap_time=float(times.mean()),
        std_lap_time=float(times.std(ddof=1)) if len(times) > 1 else 0.0,
        n_runs=n_runs,
        representative=rep,
    )


def print_report(car, track, result, *, driver=None, mc: MCResult | None = None):
    """Print a stdout lap-time report.

    v1.1: in two-lap mode prints both lap-1 (standing) and lap-2 (flying) times.
    With `--single-lap` (result.two_lap == False), falls back to the v1 single-line
    format for byte-compatible behaviour.
    """
    print("=" * 60)
    print("  LAP TIME ESTIMATION")
    print("=" * 60)
    print(f"  Car:    {car}")
    print(f"  Track:  {track}")
    if driver is not None:
        print(
            f"  Driver: {driver.name}  (skill={driver.skill_pct:.2f}, "
            f"sigma={driver.consistency_sigma:.2f})"
        )
    print("-" * 60)
    if result.two_lap:
        # Two-lap output: lap 1 deterministic, lap 2 optionally MC.
        print(f"  Lap 1 (standing): {_fmt_time(result.lap1_time)}")
        if mc is not None:
            print(
                f"  Lap 2 (flying):   {_fmt_time(mc.mean_lap_time)} "
                f"+/- {mc.std_lap_time:.3f} (N={mc.n_runs})"
            )
        else:
            print(f"  Lap 2 (flying):   {_fmt_time(result.lap2_time)}")
    else:
        if mc is not None:
            print(
                f"  Lap Time:     {_fmt_time(mc.mean_lap_time)} "
                f"+/- {mc.std_lap_time:.3f} (N={mc.n_runs})"
            )
        else:
            print(f"  Lap Time:     {result.lap_time_str}")
    print(f"  Max Speed:    {result.max_speed_kph:.1f} km/h")
    print(f"  Min Speed:    {result.min_speed_kph:.1f} km/h")
    print(f"  Avg Speed:    {result.avg_speed_kph:.1f} km/h")
    print("-" * 60)

    if not getattr(track, "is_csv_backed", False) and getattr(track, "segments", None):
        # For segment-table reporting, use lap 2 if available (or single-lap data).
        if result.two_lap and result.lap_id is not None:
            mask_primary = result.lap_id == 2
            distances_primary = result.distances[mask_primary]
            speeds_primary = result.speeds[mask_primary]
        else:
            distances_primary = result.distances
            speeds_primary = result.speeds

        print(f"\n  {'Segment':<30} {'Entry':>7} {'Min':>7} {'Exit':>7}")
        print(f"  {'':30} {'km/h':>7} {'km/h':>7} {'km/h':>7}")
        print(f"  {'-' * 51}")
        dist_pos = 0.0
        for seg in track.segments:
            seg_start = dist_pos
            seg_end = dist_pos + seg.length
            mask = (distances_primary >= seg_start) & (distances_primary < seg_end)
            if np.any(mask):
                seg_speeds = speeds_primary[mask] * 3.6
                label = (
                    f"{'Straight' if seg.is_straight else f'R={abs(seg.radius):.0f}m'} "
                    f"({seg.length:.0f}m)"
                )
                print(
                    f"  {label:<30} {seg_speeds[0]:>6.1f} "
                    f"{seg_speeds.min():>6.1f} {seg_speeds[-1]:>6.1f}"
                )
            dist_pos = seg_end

    print("=" * 60)
    print("\n  Performance Summary:")
    print(f"  Top speed (top gear):  {car.top_speed() * 3.6:.1f} km/h")
    print(f"  0-100 km/h:            {_estimate_0_x(car, 100):.1f} s")
    print(f"  0-200 km/h:            {_estimate_0_x(car, 200):.1f} s")
    print("=" * 60)


def _fmt_time(seconds: float) -> str:
    m = int(seconds // 60)
    s = seconds - m * 60
    return f"{m}:{s:06.3f}"


def _estimate_0_x(car, target_kph: float = 100.0) -> float:
    target = target_kph / 3.6
    v, t, dt = 0.5, 0.0, 0.01
    while v < target and t < 30:
        a = car.max_accel(v)
        if a <= 0:
            break
        v += a * dt
        t += dt
    return t


# ---------------------------------------------------------------------------
# v2 stint simulation (spec §21.4)
# ---------------------------------------------------------------------------

@dataclass
class StintResult:
    """Multi-lap stint result (spec §21.4).

    Lists are ordered lap-1-first. `tyre_state_history` has length `n_laps + 1`
    (index 0 is the initial state, index k is end-of-lap-k).

    `per_point_states` (v2): list (length n_laps) of dicts; each dict has the
    structure
        {"temp_C": {FL: np.array, ...},
         "wear_pct": {FL: np.array, ...},
         "pressure_psi": {FL: np.array, ...}}
    aligned with `per_lap_sim_results[k].distances`. Used by sim_telemetry to
    emit the 12 trailing state columns (spec §7.12).

    `compound` (v2, spec §21.11): the active `Compound` the stint was run
    against. Drives `f_pressure/f_temp/f_wear` and the `dy0/dx0` baselines.
    """
    n_laps: int
    setup: object  # Setup; typed as object to avoid import cycles
    calibration: TyreCalibration
    lap_times_s: list  # length n_laps
    tyre_state_history: list  # length n_laps + 1
    per_lap_sim_results: list  # length n_laps (SimResult per lap)
    per_point_states: list = field(default_factory=list)  # length n_laps
    compound: object | None = None  # Compound; None for pre-v2.5 callers.


def simulate_stint(car, track, driver, *, n_laps: int, setup,
                   calibration: TyreCalibration | None = None,
                   compound=None,
                   ds: float = 2.0) -> StintResult:
    """Run an N-lap stint with per-wheel tyre state (spec §21.4).

    Args:
        car: Car physics model.
        track: CSV-backed Track.
        driver: Driver model.
        n_laps: Number of laps in [1, 50].
        setup: Setup (initial cold pressures + ambient).
        calibration: TyreCalibration (defaults to hand-defaults if None).
        compound: Active `Compound` (spec §21.11). When None, falls back to
            `car.default_compound`. Determines `f_pressure/f_temp/f_wear` LUTs
            and the `dy0/dx0` baselines used for grip scaling.
        ds: Distance step (m). Same as `simulate(...)`.

    Returns:
        StintResult.

    Back-compat: when `n_laps == 2` and `calibration.measured == False`, this
    delegates to `simulate(..., two_lap=True)` and returns a StintResult whose
    `per_lap_sim_results` carries the v1.1 two-lap result. Telemetry emission
    treats this as constant-state across both laps (spec §14.12 item 14,
    §11.31).
    """
    if not 1 <= n_laps <= 50:
        raise ValueError(f"n_laps must be in [1, 50], got {n_laps}")
    if calibration is None:
        calibration = TyreCalibration()
    calibration = calibration.clamped()
    if compound is None:
        compound = car.default_compound

    state = TyreState.from_setup(setup)
    history = [state.copy()]
    lap_times: list[float] = []
    per_lap_results: list[SimResult] = []
    per_point_states: list = []
    # v2.0.1 (spec §21.4): universal velocity-continuity. Lap 1 is a standing
    # start (`v_prev_end == 0.0`); each subsequent lap inherits the previous
    # lap's end-of-lap speed. The two-lap tile fast path below already enforces
    # this via a single forward pass across the lap boundary, so this seed only
    # affects the multi-lap explicit loop.
    v_prev_end = 0.0

    # Back-compat fast path: n_laps == 2 + uncalibrated + default-compound -> v1.2.1 two-lap.
    use_back_compat = (
        n_laps == 2
        and not calibration.measured
        and compound.index == car.default_compound_index
    )
    if use_back_compat:
        result = simulate(car, track, driver, ds=ds, two_lap=True)
        # Synthesize lap_times from result's lap1/lap2 fields.
        lap_times = [float(result.lap1_time), float(result.lap2_time)]
        # State stays constant (no evolution): record same state twice.
        history.append(state.copy())
        history.append(state.copy())
        per_lap_results = [result, result]
        # Constant-state per-point arrays for telemetry emission.
        n_each = int(np.sum(result.lap_id == 1)) if result.lap_id is not None else len(result.distances)
        n_total = len(result.distances)
        for n_pts in (n_each, n_total - n_each if n_total > n_each else 0):
            if n_pts <= 0:
                continue
            per_point_states.append(_constant_state_arrays(state, n_pts))
        return StintResult(
            n_laps=2,
            setup=setup,
            calibration=calibration,
            lap_times_s=lap_times,
            tyre_state_history=history,
            per_lap_sim_results=per_lap_results,
            per_point_states=per_point_states,
            compound=compound,
        )

    # Build car-level tyre model once (LUTs cached at module level).
    tyre_model = build_car_tyre_model(car, compound)

    # Pre-compute the signed-radius array on the simulator's grid so we don't
    # rebuild it every lap. `to_points(ds)` returns absolute radii on a uniform
    # grid; we re-derive signs from track.csv_data's (x, y).
    distances_grid, radii_grid = track.to_points(ds)
    if getattr(track, "is_csv_backed", False):
        radius_signs = derive_radius_sign(distances_grid, track.csv_data)
    else:
        radius_signs = np.zeros(len(distances_grid), dtype=int)

    for _lap_idx in range(1, n_laps + 1):
        # Scale the car by current state's grip envelope. Active compound's
        # dy0/dx0 baseline is swapped in via GripScaledCar(compound=...).
        # v1.3 (spec §21.3): `combined_grip_envelope` returns a 3-tuple; the
        # third value (`drag_scale`) is threaded into the 3-pass solver via
        # `simulate(..., drag_scale=...)` and multiplies the total drag force
        # (aero + rolling resistance) for this lap. Wrap `car` with
        # `_DragScaledCar` at the INNERMOST layer here so `GripScaledCar`'s
        # internal `self._car.drag_force` reads through the scaler.
        mu_x, mu_y, drag_scale = combined_grip_envelope(state, tyre_model)
        if drag_scale != 1.0:
            inner_car = _DragScaledCar(car, drag_scale)
        else:
            inner_car = car
        scaled_car = GripScaledCar(inner_car, mu_x, mu_y, compound=compound)

        # Run a single-lap solver pass. Passing `drag_scale=1.0` to `simulate`
        # avoids double-wrapping; the inner wrapper above already applied it.
        # v2.0.1 (spec §21.4 step 2c): seed the forward pass with the previous
        # lap's end-of-lap velocity. Lap 1: `v_prev_end == 0.0` (standing
        # start). Lap N >= 2: `v_prev_end == lap_(N-1).speeds[-1]` so the new
        # lap begins where the previous lap ended (flying lap). If
        # `v_prev_end >= v_corner[0]`, the forward pass naturally caps it.
        lap_result = simulate(
            scaled_car, track, driver, ds=ds, two_lap=False,
            drag_scale=1.0, v_initial=v_prev_end,
        )
        per_lap_results.append(lap_result)
        lap_times.append(float(lap_result.lap_time))
        # v2.0.1 (spec §21.4 step 2f): thread end-of-lap velocity into the
        # next iteration. `speeds[-1]` is the last per-point sample of this
        # lap, which the spec defines as "lap N-1's end-of-lap velocity".
        v_prev_end = float(lap_result.speeds[-1])

        # Walk per-point arrays and update state in place. Snapshot per-point.
        n_pts = len(lap_result.distances)
        point_states = _allocate_per_point_state_arrays(n_pts)

        def _snap(i, st, _arrs=point_states):
            for w in ("FL", "FR", "RL", "RR"):
                _arrs["temp_C"][w][i] = st.temp_C[w]
                _arrs["wear_pct"][w][i] = st.wear_pct[w]
                _arrs["pressure_psi"][w][i] = st.pressure_psi[w]

        update_segments_in_place(
            state, car, calibration, tyre_model,
            distances=lap_result.distances,
            speeds=lap_result.speeds,
            radii=radii_grid[: len(lap_result.distances)],
            radius_signs=radius_signs[: len(lap_result.distances)],
            labels=lap_result.limit_label,
            on_segment=_snap,
        )
        per_point_states.append(point_states)
        history.append(state.copy())

    return StintResult(
        n_laps=n_laps,
        setup=setup,
        calibration=calibration,
        lap_times_s=lap_times,
        tyre_state_history=history,
        per_lap_sim_results=per_lap_results,
        per_point_states=per_point_states,
        compound=compound,
    )


def _allocate_per_point_state_arrays(n: int) -> dict:
    return {
        "temp_C": {w: np.zeros(n) for w in ("FL", "FR", "RL", "RR")},
        "wear_pct": {w: np.zeros(n) for w in ("FL", "FR", "RL", "RR")},
        "pressure_psi": {w: np.zeros(n) for w in ("FL", "FR", "RL", "RR")},
    }


def _constant_state_arrays(state, n: int) -> dict:
    out = _allocate_per_point_state_arrays(n)
    for w in ("FL", "FR", "RL", "RR"):
        out["temp_C"][w][:] = state.temp_C[w]
        out["wear_pct"][w][:] = state.wear_pct[w]
        out["pressure_psi"][w][:] = state.pressure_psi[w]
    return out
