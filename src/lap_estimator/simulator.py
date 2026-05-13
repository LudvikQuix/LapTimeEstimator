"""Point-mass lap time simulator.

Uses the classic 3-pass approach:
  1. Per-point max cornering speed (lateral grip limit).
  2. Forward pass - acceleration-limited from previous point.
  3. Backward pass - braking-limited into next point.

Then integrates time from the merged speed profile. The merge also produces
a per-point `limit_label` ("corner" / "accel" / "brake") used by the
synthetic-telemetry emitter (sim_telemetry).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class SimResult:
    lap_time: float
    distances: np.ndarray
    speeds: np.ndarray  # m/s
    times: np.ndarray = field(default_factory=lambda: np.zeros(0))
    ai_speeds: np.ndarray | None = None
    limit_label: np.ndarray | None = None  # array of str: corner|accel|brake
    sectors: list = field(default_factory=list)

    @property
    def lap_time_str(self):
        m = int(self.lap_time // 60)
        s = self.lap_time - m * 60
        return f"{m}:{s:06.3f}"

    @property
    def max_speed_kph(self):
        return float(np.max(self.speeds) * 3.6)

    @property
    def min_speed_kph(self):
        return float(np.min(self.speeds) * 3.6)

    @property
    def avg_speed_kph(self):
        total_dist = self.distances[-1] - self.distances[0]
        return (total_dist / self.lap_time) * 3.6


@dataclass
class MCResult:
    mean_lap_time: float
    std_lap_time: float
    n_runs: int
    representative: SimResult  # deterministic skill-only run, for plotting


def simulate(car, track, driver=None, *, ds=2.0, rng=None, noise=False):
    """Run a single deterministic (or noisy) lap.

    `driver` is optional. When `None`, the raw `car` is used (equivalent to
    skill_pct=1.0, no noise).
    """
    if driver is not None:
        scaled = driver.wrap(car, rng=rng, noise=noise)
    else:
        scaled = car

    distances, radii = track.to_points(ds)
    n = len(distances)
    v_min = 5.0

    # Pass 1: max cornering speed
    v_corner = np.array([scaled.max_cornering_speed(r) for r in radii])
    v_corner = np.clip(v_corner, v_min, 999.0)

    # Pass 2: forward (accel-limited)
    v_forward = np.copy(v_corner)
    v_forward[0] = min(v_corner[0], v_min * 3)
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

    # Integrate time
    times = np.zeros(n)
    total_time = 0.0
    for i in range(1, n):
        d = distances[i] - distances[i - 1]
        if d <= 0:
            times[i] = total_time
            continue
        v_avg = max(0.5 * (speeds[i - 1] + speeds[i]), v_min)
        total_time += d / v_avg
        times[i] = total_time

    ai = track.to_ai_reference(ds) if hasattr(track, "to_ai_reference") else None
    return SimResult(
        lap_time=float(total_time),
        distances=distances,
        speeds=speeds,
        times=times,
        ai_speeds=ai,
        limit_label=labels,
    )


def simulate_monte_carlo(car, track, driver, *, ds=2.0, n_runs=20, seed=0):
    """Run N noisy laps for a driver with non-zero consistency_sigma."""
    rng = np.random.default_rng(seed)
    times = []
    for _ in range(n_runs):
        r = simulate(car, track, driver, ds=ds, rng=rng, noise=True)
        times.append(r.lap_time)
    times = np.array(times)
    rep = simulate(car, track, driver, ds=ds)
    return MCResult(
        mean_lap_time=float(times.mean()),
        std_lap_time=float(times.std(ddof=1)) if len(times) > 1 else 0.0,
        n_runs=n_runs,
        representative=rep,
    )


def print_report(car, track, result, *, driver=None, mc: MCResult | None = None):
    """Print a stdout lap-time report."""
    print("=" * 60)
    print("  LAP TIME ESTIMATION")
    print("=" * 60)
    print(f"  Car:    {car}")
    print(f"  Track:  {track}")
    if driver is not None:
        print(f"  Driver: {driver.name}  (skill={driver.skill_pct:.2f}, "
              f"sigma={driver.consistency_sigma:.2f})")
    print("-" * 60)
    if mc is not None:
        print(f"  Lap Time:     {_fmt_time(mc.mean_lap_time)} "
              f"± {mc.std_lap_time:.3f} (N={mc.n_runs})")
    else:
        print(f"  Lap Time:     {result.lap_time_str}")
    print(f"  Max Speed:    {result.max_speed_kph:.1f} km/h")
    print(f"  Min Speed:    {result.min_speed_kph:.1f} km/h")
    print(f"  Avg Speed:    {result.avg_speed_kph:.1f} km/h")
    print("-" * 60)

    if not getattr(track, "is_csv_backed", False) and getattr(track, "segments", None):
        print(f"\n  {'Segment':<30} {'Entry':>7} {'Min':>7} {'Exit':>7}")
        print(f"  {'':30} {'km/h':>7} {'km/h':>7} {'km/h':>7}")
        print(f"  {'-' * 51}")
        dist_pos = 0.0
        for seg in track.segments:
            seg_start = dist_pos
            seg_end = dist_pos + seg.length
            mask = (result.distances >= seg_start) & (result.distances < seg_end)
            if np.any(mask):
                seg_speeds = result.speeds[mask] * 3.6
                label = f"{'Straight' if seg.is_straight else f'R={abs(seg.radius):.0f}m'} ({seg.length:.0f}m)"
                print(f"  {label:<30} {seg_speeds[0]:>6.1f} "
                      f"{seg_speeds.min():>6.1f} {seg_speeds[-1]:>6.1f}")
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
