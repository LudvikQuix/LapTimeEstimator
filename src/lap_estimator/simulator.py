"""Point-mass lap time simulator.

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
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


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


def simulate(car, track, driver=None, *, ds=2.0, rng=None, noise=False, two_lap=True):
    """Run a single deterministic (or noisy) lap, optionally as a two-lap tiled run.

    `driver` is optional. When `None`, the raw `car` is used (skill_pct=1.0).
    `two_lap=True` (default, v1.1) tiles the segment list twice and runs a single
    3-pass over the 2x grid; lap 2 inherits lap 1's end-of-lap forward speed
    naturally via the forward pass crossing the boundary.
    """
    if driver is not None:
        scaled = driver.wrap(car, rng=rng, noise=noise)
    else:
        scaled = car

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
        scaled, distances_full, radii_full, v_init=0.0, v_min=v_min,
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
