"""Point-mass lap time simulator.

Uses the classic 3-pass approach:
1. Calculate max cornering speed at every point
2. Forward pass: limit by acceleration from previous point
3. Backward pass: limit by braking into next point
4. Integrate time from the speed profile
"""
import numpy as np


class SimResult:
    def __init__(self, lap_time, distances, speeds, sectors=None):
        self.lap_time = lap_time
        self.distances = distances
        self.speeds = speeds  # m/s
        self.sectors = sectors or []

    @property
    def lap_time_str(self):
        m = int(self.lap_time // 60)
        s = self.lap_time - m * 60
        return f"{m}:{s:06.3f}"

    @property
    def max_speed_kph(self):
        return np.max(self.speeds) * 3.6

    @property
    def min_speed_kph(self):
        return np.min(self.speeds) * 3.6

    @property
    def avg_speed_kph(self):
        total_dist = self.distances[-1] - self.distances[0]
        return (total_dist / self.lap_time) * 3.6


def simulate(car, track, ds=2.0):
    """Run a lap time simulation.

    Args:
        car: Car object with physics model
        track: Track object with segment definitions
        ds: Distance step in meters (smaller = more accurate, slower)

    Returns:
        SimResult with lap time and speed trace
    """
    distances, radii = track.to_points(ds)
    n = len(distances)
    v_min = 5.0  # minimum speed (m/s) to avoid division issues

    # Pass 1: Max cornering speed at each point
    v_corner = np.array([car.max_cornering_speed(r) for r in radii])
    v_corner = np.clip(v_corner, v_min, 999.0)

    # Pass 2: Forward (acceleration-limited)
    v_forward = np.copy(v_corner)
    v_forward[0] = min(v_corner[0], v_min * 3)  # start from reasonable speed

    for i in range(1, n):
        v_prev = v_forward[i - 1]
        d = distances[i] - distances[i - 1]
        if d <= 0:
            v_forward[i] = min(v_forward[i], v_prev)
            continue

        accel = car.max_accel(v_prev)
        accel = max(accel, 0.1)  # ensure some minimum acceleration
        # v^2 = v0^2 + 2*a*d
        v_new = np.sqrt(max(v_prev ** 2 + 2 * accel * d, v_min ** 2))
        v_forward[i] = min(v_corner[i], v_new)

    # Pass 3: Backward (braking-limited)
    v_brake = np.copy(v_forward)

    for i in range(n - 2, -1, -1):
        v_next = v_brake[i + 1]
        d = distances[i + 1] - distances[i]
        if d <= 0:
            continue

        decel = car.max_braking_decel(v_next)
        decel = max(decel, 0.1)
        # v^2 = v_next^2 + 2*decel*d  (braking backward means we can be faster)
        v_new = np.sqrt(max(v_next ** 2 + 2 * decel * d, v_min ** 2))
        v_brake[i] = min(v_brake[i], v_new)

    # Final speed profile is minimum of all constraints
    speeds = v_brake

    # Integrate time
    total_time = 0.0
    for i in range(1, n):
        d = distances[i] - distances[i - 1]
        if d <= 0:
            continue
        v_avg = 0.5 * (speeds[i - 1] + speeds[i])
        v_avg = max(v_avg, v_min)
        total_time += d / v_avg

    return SimResult(total_time, distances, speeds)


def print_report(car, track, result):
    """Print a formatted lap time report."""
    print("=" * 60)
    print(f"  LAP TIME ESTIMATION")
    print("=" * 60)
    print(f"  Car:    {car}")
    print(f"  Track:  {track}")
    print("-" * 60)
    print(f"  Lap Time:     {result.lap_time_str}")
    print(f"  Max Speed:    {result.max_speed_kph:.1f} km/h")
    print(f"  Min Speed:    {result.min_speed_kph:.1f} km/h")
    print(f"  Avg Speed:    {result.avg_speed_kph:.1f} km/h")
    print("-" * 60)

    # Speed trace through segments
    print(f"\n  {'Segment':<30} {'Entry':>7} {'Min':>7} {'Exit':>7}")
    print(f"  {'':30} {'km/h':>7} {'km/h':>7} {'km/h':>7}")
    print(f"  {'-'*51}")

    dist_pos = 0.0
    for seg in track.segments:
        seg_start = dist_pos
        seg_end = dist_pos + seg.length
        mask = (result.distances >= seg_start) & (result.distances < seg_end)
        if np.any(mask):
            seg_speeds = result.speeds[mask] * 3.6
            label = f"{'Straight' if seg.is_straight else f'R={abs(seg.radius):.0f}m'} ({seg.length:.0f}m)"
            print(f"  {label:<30} {seg_speeds[0]:>6.1f} {seg_speeds.min():>6.1f} {seg_speeds[-1]:>6.1f}")
        dist_pos = seg_end

    print("=" * 60)

    # Performance summary
    print(f"\n  Performance Summary:")
    print(f"  Top speed (top gear):  {car.top_speed() * 3.6:.1f} km/h")
    print(f"  0-100 km/h:            {_estimate_0_100(car):.1f} s")
    print(f"  0-200 km/h:            {_estimate_0_200(car):.1f} s")
    print("=" * 60)


def _estimate_0_100(car, target_kph=100):
    """Estimate 0-target_kph time."""
    target = target_kph / 3.6
    v, t, dt = 0.5, 0.0, 0.01
    while v < target and t < 30:
        a = car.max_accel(v)
        if a <= 0:
            break
        v += a * dt
        t += dt
    return t


def _estimate_0_200(car, target_kph=200):
    return _estimate_0_100(car, target_kph)
