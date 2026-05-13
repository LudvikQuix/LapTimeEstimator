"""Measure dynamic driver-profile signals from merged telemetry frames (v1.2).

Single entry point: `measure_dynamics(merged_frames) -> ProfileDynamics`.

The five measured fields are described in spec §13.11:
- `driver_tau_s` -- median time-to-50% on hysteresis-armed gas/brake edges (s).
- `trail_brake_m` -- median distance over which `brake` decays 0.8 -> 0.1
  entering corner-limited segments (metres).
- `throttle_ramp_m` -- median distance over which `gas` ramps 0.4 -> 0.95
  exiting corner-limited segments (metres).
- `pedal_press_rate_per_s` -- median slope on the same leading edges (1/s).
- `steering_aggression_deg_per_s` -- 95th percentile |d(steerAngle)/dt|,
  unit-detected (deg/s output). `None` when `steerAngle` is absent.

All measurements fall back to hand-defaults when their candidate pool is
under-supplied. `measured.<field>` flags whether the value came from real
telemetry (`true`) or the fallback (`false`).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# Fallback hand-defaults (mirror `driver.DEFAULT_*`).
_FALLBACK_DRIVER_TAU_S = 0.12
_FALLBACK_TRAIL_BRAKE_M = 30.0
_FALLBACK_THROTTLE_RAMP_M = 40.0

# Clamp ranges (spec §13.11).
_TAU_CLAMP = (0.02, 0.50)
_TRAIL_CLAMP = (5.0, 150.0)
_RAMP_CLAMP = (5.0, 200.0)
_STEER_CLAMP = (0.0, 5000.0)

# Minimum candidate counts before a measurement is trusted.
_MIN_EDGES = 10
_MIN_TAPER_SEGMENTS = 5
_MIN_RAMP_SEGMENTS = 5


@dataclass
class ProfileDynamics:
    driver_tau_s: float | None
    trail_brake_m: float | None
    throttle_ramp_m: float | None
    pedal_press_rate_per_s: float | None
    steering_aggression_deg_per_s: float | None
    measured: dict = field(default_factory=dict)
    sample_counts: dict = field(default_factory=dict)


def measure_dynamics(
    merged_frames: list,
    *,
    rising_edge_low: float = 0.10,
    rising_edge_high: float = 0.50,
    min_rise_window_s: float = 0.20,
    pedal_hysteresis_low: float = 0.05,
    taper_high: float = 0.80,
    taper_low: float = 0.10,
    ramp_low: float = 0.40,
    ramp_high: float = 0.95,
    corner_detect_speed_window_s: float = 2.0,
    steering_percentile: float = 95.0,
) -> ProfileDynamics:
    """Measure all five dynamic profile fields from per-lap merged frames.

    `merged_frames` is the list of per-lap dicts emitted by
    `telemetry.merge_with_track` (each contains `timestamp_ms`, `distance_m`,
    `speedKmh`, `gas`, `brake`, optionally `steerAngle`).
    """
    measured: dict[str, bool] = {}
    counts: dict[str, int] = {}

    tau_candidates: list[float] = []
    slope_candidates: list[float] = []
    for frame in merged_frames:
        t_s = np.asarray(frame["timestamp_ms"], dtype=float) / 1000.0
        for channel in ("gas", "brake"):
            y = np.asarray(frame[channel], dtype=float)
            for tau, slope in _iter_leading_edges(
                y, t_s,
                hysteresis_low=pedal_hysteresis_low,
                edge_low=rising_edge_low,
                edge_high=rising_edge_high,
                max_window_s=min_rise_window_s,
            ):
                tau_candidates.append(tau)
                slope_candidates.append(slope)

    n_edges = len(tau_candidates)
    counts["pedal_leading_edges"] = n_edges
    if n_edges >= _MIN_EDGES:
        tau_med = float(np.median(tau_candidates))
        driver_tau_s = float(np.clip(tau_med, *_TAU_CLAMP))
        slope_med = float(np.median(slope_candidates))
        pedal_press_rate_per_s = slope_med
        measured["driver_tau_s"] = True
        measured["pedal_press_rate_per_s"] = True
    else:
        driver_tau_s = _FALLBACK_DRIVER_TAU_S
        pedal_press_rate_per_s = None
        measured["driver_tau_s"] = False
        measured["pedal_press_rate_per_s"] = False

    trail_candidates: list[float] = []
    ramp_candidates: list[float] = []
    for frame in merged_frames:
        speed_kmh = np.asarray(frame["speedKmh"], dtype=float)
        t_s = np.asarray(frame["timestamp_ms"], dtype=float) / 1000.0
        dist = np.asarray(frame["distance_m"], dtype=float)
        brake = np.asarray(frame["brake"], dtype=float)
        gas = np.asarray(frame["gas"], dtype=float)

        apex_indices = _find_corner_apex_indices(
            speed_kmh, t_s, window_s=corner_detect_speed_window_s,
        )
        for apex_idx in apex_indices:
            tc = _trail_brake_candidate(
                brake, dist, apex_idx,
                taper_high=taper_high, taper_low=taper_low,
            )
            if tc is not None:
                trail_candidates.append(tc)
            rc = _throttle_ramp_candidate(
                gas, dist, apex_idx,
                ramp_low=ramp_low, ramp_high=ramp_high,
            )
            if rc is not None:
                ramp_candidates.append(rc)

    counts["brake_taper_segments"] = len(trail_candidates)
    if len(trail_candidates) >= _MIN_TAPER_SEGMENTS:
        trail_med = float(np.median(trail_candidates))
        trail_brake_m = float(np.clip(trail_med, *_TRAIL_CLAMP))
        measured["trail_brake_m"] = True
    else:
        trail_brake_m = _FALLBACK_TRAIL_BRAKE_M
        measured["trail_brake_m"] = False

    counts["throttle_ramp_segments"] = len(ramp_candidates)
    if len(ramp_candidates) >= _MIN_RAMP_SEGMENTS:
        ramp_med = float(np.median(ramp_candidates))
        throttle_ramp_m = float(np.clip(ramp_med, *_RAMP_CLAMP))
        measured["throttle_ramp_m"] = True
    else:
        throttle_ramp_m = _FALLBACK_THROTTLE_RAMP_M
        measured["throttle_ramp_m"] = False

    steering_value, steering_unit, n_steer = _measure_steering(
        merged_frames, steering_percentile=steering_percentile,
    )
    counts["steering_samples"] = n_steer
    if steering_unit is not None:
        counts["steering_unit_detected"] = steering_unit
    if steering_value is None:
        measured["steering_aggression_deg_per_s"] = False
    else:
        measured["steering_aggression_deg_per_s"] = True

    return ProfileDynamics(
        driver_tau_s=driver_tau_s,
        trail_brake_m=trail_brake_m,
        throttle_ramp_m=throttle_ramp_m,
        pedal_press_rate_per_s=pedal_press_rate_per_s,
        steering_aggression_deg_per_s=steering_value,
        measured=measured,
        sample_counts=counts,
    )


def _iter_leading_edges(
    y: np.ndarray,
    t_s: np.ndarray,
    *,
    hysteresis_low: float,
    edge_low: float,
    edge_high: float,
    max_window_s: float,
):
    """Yield (tau_s, slope_per_s) for every hysteresis-armed rising edge.

    State machine:
      armed -> a sample with y <= hysteresis_low re-arms.
      candidate -> first sample with y >= edge_low starts an edge candidate
      (records edge_start time + value at the cross).
      complete -> first subsequent sample with y >= edge_high completes the
      candidate. Yields (t_50 - edge_start, (y_at_50 - y_at_start) / dt).
      If max_window_s elapses before edge_high is reached, abort candidate;
      caller must dip below `hysteresis_low` to re-arm.
    """
    if len(y) < 2:
        return
    armed = bool(y[0] <= hysteresis_low)
    candidate = False
    t_start = 0.0
    y_start = 0.0
    for i in range(1, len(y)):
        yi = float(y[i])
        ti = float(t_s[i])
        if not candidate:
            if armed and yi >= edge_low and y[i - 1] < edge_low:
                # Linear-interpolate the exact crossing for sub-sample accuracy.
                prev_t = float(t_s[i - 1])
                prev_y = float(y[i - 1])
                denom = yi - prev_y
                if denom > 0:
                    frac = (edge_low - prev_y) / denom
                    t_start = prev_t + frac * (ti - prev_t)
                    y_start = edge_low
                else:
                    t_start = ti
                    y_start = yi
                candidate = True
                armed = False
            elif yi <= hysteresis_low:
                armed = True
        else:
            if ti - t_start > max_window_s:
                # Aborted: re-arm logic in next iteration.
                candidate = False
                if yi <= hysteresis_low:
                    armed = True
                continue
            if yi >= edge_high and y[i - 1] < edge_high:
                prev_t = float(t_s[i - 1])
                prev_y = float(y[i - 1])
                denom = yi - prev_y
                if denom > 0:
                    frac = (edge_high - prev_y) / denom
                    t_50 = prev_t + frac * (ti - prev_t)
                    y_50 = edge_high
                else:
                    t_50 = ti
                    y_50 = yi
                dt = t_50 - t_start
                if dt > 1e-6:
                    tau = dt
                    slope = (y_50 - y_start) / dt
                    yield tau, slope
                candidate = False
                # After completion, look for another full dip before re-arming.
                if yi <= hysteresis_low:
                    armed = True


def _moving_average(x: np.ndarray, window: int) -> np.ndarray:
    """Simple moving-average smoothing with reflection at boundaries."""
    if window <= 1 or len(x) <= 1:
        return x.astype(float, copy=True)
    w = min(window, len(x))
    kernel = np.ones(w, dtype=float) / w
    return np.convolve(x, kernel, mode="same")


def _find_corner_apex_indices(
    speed_kmh: np.ndarray,
    t_s: np.ndarray,
    *,
    window_s: float,
    min_drop_ratio: float = 0.85,
) -> list[int]:
    """Return indices of local minima of smoothed `speed_kmh` below
    `min_drop_ratio * max(smoothed)`.
    """
    n = len(speed_kmh)
    if n < 5:
        return []
    # Convert time-window to sample count using median dt.
    if n >= 2:
        dts = np.diff(t_s)
        med_dt = float(np.median(dts[dts > 0])) if (dts > 0).any() else 0.01
    else:
        med_dt = 0.01
    if med_dt <= 0:
        med_dt = 0.01
    window_samples = max(3, int(round(window_s / med_dt)))
    smooth = _moving_average(speed_kmh, window_samples)
    threshold = min_drop_ratio * float(smooth.max())
    apex: list[int] = []
    last_kept = -window_samples
    for i in range(1, n - 1):
        if smooth[i] >= threshold:
            continue
        if smooth[i] <= smooth[i - 1] and smooth[i] <= smooth[i + 1]:
            # Suppress neighbour minima that fall inside the smoothing window.
            if i - last_kept >= window_samples:
                apex.append(i)
                last_kept = i
    return apex


def _trail_brake_candidate(
    brake: np.ndarray,
    distance: np.ndarray,
    apex_idx: int,
    *,
    taper_high: float,
    taper_low: float,
    max_window_m: float = 200.0,
) -> float | None:
    """Distance over which `brake` decays from `>= taper_high` to `<= taper_low`
    around `apex_idx`, or `None` if the candidate fails the spec gates.
    """
    if apex_idx <= 0 or apex_idx >= len(brake):
        return None
    peak_idx = -1
    for j in range(apex_idx - 1, -1, -1):
        if brake[j] >= taper_high:
            peak_idx = j
            break
    if peak_idx < 0:
        return None
    off_idx = -1
    for j in range(peak_idx + 1, len(brake)):
        if brake[j] <= taper_low:
            off_idx = j
            break
    if off_idx < 0:
        return None
    dist = float(distance[off_idx] - distance[peak_idx])
    if dist < 0:
        return None
    if dist > max_window_m:
        return None
    return dist


def _throttle_ramp_candidate(
    gas: np.ndarray,
    distance: np.ndarray,
    apex_idx: int,
    *,
    ramp_low: float,
    ramp_high: float,
    max_window_m: float = 250.0,
) -> float | None:
    """Distance over which `gas` ramps from `>= ramp_low` to `>= ramp_high`
    after the apex.
    """
    if apex_idx < 0 or apex_idx >= len(gas):
        return None
    partial_idx = -1
    for j in range(apex_idx, len(gas)):
        if gas[j] >= ramp_low:
            partial_idx = j
            break
    if partial_idx < 0:
        return None
    full_idx = -1
    for j in range(partial_idx, len(gas)):
        if gas[j] >= ramp_high:
            full_idx = j
            break
    if full_idx < 0:
        return None
    dist = float(distance[full_idx] - distance[partial_idx])
    if dist < 0:
        return None
    if dist > max_window_m:
        return None
    return dist


def _measure_steering(
    merged_frames: list,
    *,
    steering_percentile: float,
) -> tuple[float | None, str | None, int]:
    """Pool |d(steerAngle)/dt| across all laps, take the requested percentile.

    Returns (value_deg_per_s, unit_detected, sample_count). Value is `None`
    when no frame contains a usable `steerAngle` trace.
    """
    pooled_abs: list[np.ndarray] = []
    max_abs = 0.0
    sample_total = 0
    for frame in merged_frames:
        steer = frame.get("steerAngle")
        if steer is None:
            continue
        steer = np.asarray(steer, dtype=float)
        finite = np.isfinite(steer)
        if finite.sum() < 2:
            continue
        t_s = np.asarray(frame["timestamp_ms"], dtype=float) / 1000.0
        # Compute pairwise finite differences. Drop entries where either side
        # is NaN or where dt <= 0 (preserves robustness against duplicates).
        s = steer[finite]
        t = t_s[finite]
        if len(s) < 2:
            continue
        ds = np.diff(s)
        dt = np.diff(t)
        ok = dt > 0
        if not ok.any():
            continue
        rate = np.abs(ds[ok] / dt[ok])
        pooled_abs.append(rate)
        local_max = float(np.max(np.abs(s)))
        if local_max > max_abs:
            max_abs = local_max
        sample_total += int(finite.sum())
    if not pooled_abs:
        return None, None, 0
    pool = np.concatenate(pooled_abs)
    if pool.size == 0:
        return None, None, sample_total
    # Unit heuristic per spec: max |steerAngle| < 5 -> radians.
    unit = "rad" if max_abs < 5.0 else "deg"
    raw = float(np.percentile(pool, steering_percentile))
    if unit == "rad":
        value = raw * 180.0 / np.pi
    else:
        value = raw
    value = float(np.clip(value, *_STEER_CLAMP))
    return value, unit, sample_total
