"""Telemetry-driven driver fitting (v1.1 multi-lap + v1.2 profile dynamics).

Given merged per-lap telemetry frames and a Car physics model, derive
`skill_pct` and `consistency_sigma` from the pooled cornering samples and
measure the v1.2 dynamic profile fields.

Algorithm (spec §13.5):
  1. (Per lap) Out-lap trim happens at the CLI layer before merge.
  2. Per-lap lat_g_obs = v^2 / R / g, exclude straights (R >= 500 m).
  3. Per-lap lat_g_max = car.tyre_grip_lateral(v) * (1 + downforce / (m*g)).
  4. Per-lap util = lat_g_obs / lat_g_max, clipped to [0, 1.2].
  5. Pool every lap's cornering-sample util array; take the 85th percentile
     for skill_pct, scaled stdev for consistency_sigma_seconds.
  5b. (v1.2) `profile_dynamics.measure_dynamics(merged_frames)` -> dynamic
      profile fields, plumbed into `FitResult.profile`.
  6+ Caller writes JSON and runs the validation sim.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .profile_dynamics import ProfileDynamics, measure_dynamics

STRAIGHT_THRESHOLD_M = 500.0
G = 9.81
# A lap is "finished" when its post-trim normalizedCarPosition span >= this.
FINISHED_NCP_SPAN = 0.9


@dataclass
class FitResult:
    skill_pct: float
    consistency_sigma: float
    util_p85: float
    util_stdev: float
    n_corner_samples: int
    n_over_unity: int
    n_laps: int = 1
    n_finished_laps: int = 0
    real_lap_times_s: list = field(default_factory=list)
    real_lap_time_s: float | None = None
    pooled_sample_count: int = 0
    profile: ProfileDynamics | None = None


def fit_driver(
    car,
    merged_frames,
    *,
    straight_threshold_m: float = STRAIGHT_THRESHOLD_M,
    util_percentile: float = 85.0,
) -> FitResult:
    """Fit a driver from a list of per-lap merged telemetry frames.

    Backward compatibility shim: a single dict argument (legacy v1 callers)
    is wrapped into a one-element list and treated as a single-lap fit. The
    multi-lap guard (`>=2 laps required`) lives at the CLI layer where the
    user-facing error message can be emitted.
    """
    frames = _normalise_frames(merged_frames)

    pooled_util: list[np.ndarray] = []
    n_over_unity_total = 0
    n_corner_total = 0
    real_lap_times: list[float] = []
    n_finished = 0

    for merged in frames:
        v = merged["speed_ms"]
        r = merged["radius_m"]
        corner_mask = (r < straight_threshold_m) & (v > 1.0)
        if not corner_mask.any():
            # This lap contributes nothing to the skill pool but still counts
            # toward `n_laps`. Skip its util calc.
            pass
        else:
            v_c = v[corner_mask]
            r_c = r[corner_mask]
            lat_g_obs = (v_c ** 2) / np.clip(r_c, 1.0, None) / G
            lat_g_max = np.array([
                car.tyre_grip_lateral(float(vv))
                * (1.0 + car.downforce(float(vv)) / (car.total_mass * G))
                for vv in v_c
            ])
            lat_g_max = np.clip(lat_g_max, 1e-3, None)
            util = lat_g_obs / lat_g_max
            util_clipped = np.clip(util, 0.0, 1.2)
            pooled_util.append(util_clipped)
            n_over_unity_total += int((util > 1.0).sum())
            n_corner_total += int(corner_mask.sum())

        ncp = merged.get("normalizedCarPosition")
        if ncp is not None and len(ncp) > 0:
            ncp_span = float(np.max(ncp) - np.min(ncp))
        else:
            ncp_span = 0.0
        if ncp_span >= FINISHED_NCP_SPAN:
            ts = merged.get("timestamp_ms")
            if ts is not None and len(ts) >= 2:
                real_lap_times.append(float((ts.max() - ts.min()) / 1000.0))
                n_finished += 1

    if not pooled_util:
        raise ValueError("No cornering samples across any lap (all radii above straight threshold).")

    pool = np.concatenate(pooled_util)
    p85 = float(np.percentile(pool, util_percentile))
    skill = float(np.clip(p85, 0.05, 1.0))
    stdev = float(np.std(pool, ddof=1)) if len(pool) > 1 else 0.0
    sigma_s = float(np.clip(stdev / 0.03, 0.0, 1.5))

    profile = measure_dynamics(frames)

    real_lap_time_s = (
        float(np.mean(real_lap_times)) if real_lap_times else None
    )

    return FitResult(
        skill_pct=skill,
        consistency_sigma=sigma_s,
        util_p85=p85,
        util_stdev=stdev,
        n_corner_samples=n_corner_total,
        n_over_unity=n_over_unity_total,
        n_laps=len(frames),
        n_finished_laps=n_finished,
        real_lap_times_s=[round(t, 3) for t in real_lap_times],
        real_lap_time_s=round(real_lap_time_s, 3) if real_lap_time_s is not None else None,
        pooled_sample_count=int(len(pool)),
        profile=profile,
    )


def _normalise_frames(merged_frames):
    """Accept either a single merged dict or a list of merged dicts."""
    if isinstance(merged_frames, dict):
        return [merged_frames]
    return list(merged_frames)
