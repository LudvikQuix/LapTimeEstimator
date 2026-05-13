"""Emit a synthetic telemetry CSV with AC's exact schema.

Resamples a SimResult onto a uniform `dt_ms` time grid and reconstructs
`gas` / `brake` per-point from the simulator's binding-limit label.
"""
from __future__ import annotations

import csv
import os

import numpy as np

AC_HEADER = ("timestamp_ms", "gas", "brake", "distanceTraveled",
             "speedKmh", "normalizedCarPosition")


def write_synthetic_log(sim_result, car, track_total_length_m: float,
                        output_path: str, *, telemetry_dt_ms: int = 100) -> None:
    """Write the synthetic telemetry CSV.

    Args:
        sim_result: SimResult with `distances`, `speeds`, `times`, `limit_label`.
        car: Car physics object (used to scale `gas` on corner-bound samples).
        track_total_length_m: length used for normalized position.
        output_path: where to write the CSV (parent dirs created if needed).
        telemetry_dt_ms: cadence of the output grid.
    """
    if telemetry_dt_ms <= 0:
        raise ValueError("telemetry_dt_ms must be > 0")
    if sim_result.times is None or len(sim_result.times) == 0:
        raise ValueError("SimResult.times is empty")

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

    t_src = sim_result.times
    d_src = sim_result.distances
    v_src = sim_result.speeds  # m/s
    labels_src = sim_result.limit_label

    t_total = float(t_src[-1])
    dt = telemetry_dt_ms / 1000.0
    n = int(np.floor(t_total / dt)) + 1
    if n < 2:
        n = 2

    t_out = np.arange(n) * dt
    t_out = np.clip(t_out, t_src[0], t_src[-1])

    d_out = np.interp(t_out, t_src, d_src)
    v_out_ms = np.interp(t_out, t_src, v_src)
    v_out_kmh = v_out_ms * 3.6

    # Nearest-neighbour for the binding label
    idx = np.searchsorted(t_src, t_out, side="left")
    idx = np.clip(idx, 0, len(t_src) - 1)
    labels_out = (labels_src[idx] if labels_src is not None
                  else np.array(["accel"] * n, dtype=object))

    # Reconstruct gas/brake
    gas = np.zeros(n)
    brake = np.zeros(n)
    for i in range(n):
        lab = labels_out[i]
        if lab == "accel":
            gas[i] = 1.0
            brake[i] = 0.0
        elif lab == "brake":
            gas[i] = 0.0
            brake[i] = 1.0
        else:  # corner-bound: partial throttle to hold v
            v = float(v_out_ms[i])
            drag = car.drag_force(v)
            rr = car.rolling_resistance(v)
            required = drag + rr  # zero net accel through the apex
            max_force = car.max_traction_force(v)
            ratio = 0.0 if max_force <= 1e-6 else required / max_force
            gas[i] = float(np.clip(ratio, 0.0, 1.0))
            brake[i] = 0.0

    if track_total_length_m <= 0:
        track_total_length_m = float(d_src[-1]) if d_src[-1] > 0 else 1.0
    norm = np.clip(d_out / track_total_length_m, 0.0, 1.0 - 1e-9)

    with open(output_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(list(AC_HEADER))
        for i in range(n):
            w.writerow([
                int(round(i * telemetry_dt_ms)),
                f"{gas[i]:.4f}",
                f"{brake[i]:.4f}",
                f"{float(d_out[i]):.4f}",
                f"{float(v_out_kmh[i]):.4f}",
                f"{float(norm[i]):.8f}",
            ])
