"""Telemetry-driven driver fitting.

Given a merged telemetry+track frame and a Car physics model, derive
`skill_pct` and `consistency_sigma` for a v1 driver YAML.

Algorithm (spec §13.5):
  1. Compute lat_g_obs = v^2 / R / g, exclude straights (R >= 500 m).
  2. lat_g_max = car.tyre_grip_lateral(v) * (1 + downforce / (m*g)).
  3. util = lat_g_obs / lat_g_max.
  4. skill_pct = clip(percentile(util, 85), 0.05, 1.0).
  5. consistency_sigma_s = clip(stdev(util) / 0.03, 0.0, 1.5).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

STRAIGHT_THRESHOLD_M = 500.0
G = 9.81


@dataclass
class FitResult:
    skill_pct: float
    consistency_sigma: float
    util_p85: float
    util_stdev: float
    n_corner_samples: int
    n_over_unity: int


def fit_driver(car, merged, *, straight_threshold_m: float = STRAIGHT_THRESHOLD_M,
               util_percentile: float = 85.0) -> FitResult:
    v = merged["speed_ms"]
    r = merged["radius_m"]
    corner_mask = (r < straight_threshold_m) & (v > 1.0)
    if not corner_mask.any():
        raise ValueError("No cornering samples (all radii above straight threshold).")

    v_c = v[corner_mask]
    r_c = r[corner_mask]

    lat_g_obs = (v_c ** 2) / np.clip(r_c, 1.0, None) / G

    # Car-side theoretical max lateral G across the same speed series
    lat_g_max = np.array([
        car.tyre_grip_lateral(float(vv)) * (1.0 + car.downforce(float(vv)) / (car.total_mass * G))
        for vv in v_c
    ])
    lat_g_max = np.clip(lat_g_max, 1e-3, None)

    util = lat_g_obs / lat_g_max
    util_clipped = np.clip(util, 0.0, 1.2)

    p85 = float(np.percentile(util_clipped, util_percentile))
    skill = float(np.clip(p85, 0.05, 1.0))
    stdev = float(np.std(util_clipped, ddof=1)) if len(util_clipped) > 1 else 0.0
    sigma_s = float(np.clip(stdev / 0.03, 0.0, 1.5))

    return FitResult(
        skill_pct=skill,
        consistency_sigma=sigma_s,
        util_p85=p85,
        util_stdev=stdev,
        n_corner_samples=int(corner_mask.sum()),
        n_over_unity=int((util > 1.0).sum()),
    )
