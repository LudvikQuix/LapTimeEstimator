"""Emit a synthetic telemetry CSV with AC's exact schema plus a trailing `lap` column.

v1.1 three-layer pipeline (spec §14.3):
  1. Layer 1 -- limit-label rule: accel->1/0, brake->0/1, corner->partial-throttle.
  2. Layer 2 -- corner-shape heuristic: trail-brake taper + throttle ramp (per-driver
     distances `trail_brake_m` / `throttle_ramp_m`; either set to 0 disables that leg).
  3. Layer 3 -- driver-lag 1st-order IIR low-pass on gas and brake independently
     (alpha = dt / (driver_tau_s + dt); driver_tau_s=0 bypasses).

Order matters: heuristic THEN low-pass. Inverting the order would smear the
limit-label transitions before the heuristic can read them.

Two-lap (v1.1, spec §20): callers pass a SimResult with `lap_id` per-point.
`timestamp_ms` is monotonic across the lap boundary; `distanceTraveled` and
`normalizedCarPosition` reset to 0 at the start of lap 2. The trailing `lap`
column carries values from {1, 2} (or constant 1 in single-lap mode).
"""
from __future__ import annotations

import csv
import os

import numpy as np

AC_HEADER = (
    "timestamp_ms", "gas", "brake", "distanceTraveled",
    "speedKmh", "normalizedCarPosition", "lap",
)


def write_synthetic_log(sim_result, car, driver, track_total_length_m: float,
                        output_path: str, *, telemetry_dt_ms: int = 100) -> None:
    """Write the synthetic telemetry CSV (AC schema + `lap` column).

    Args:
        sim_result: SimResult with `distances`, `speeds`, `times`, `limit_label`,
            and (v1.1) `lap_id`.
        car: Car physics object (used to scale `gas` on corner-bound samples).
        driver: Driver object (v1.1; reads `driver_tau_s`, `trail_brake_m`,
            `throttle_ramp_m`). Pass `None` for bang-bang behaviour.
        track_total_length_m: per-lap length used for normalized position.
        output_path: where to write the CSV (parent dirs created if needed).
        telemetry_dt_ms: cadence of the output grid.
    """
    if telemetry_dt_ms <= 0:
        raise ValueError("telemetry_dt_ms must be > 0")
    if sim_result.times is None or len(sim_result.times) == 0:
        raise ValueError("SimResult.times is empty")

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

    # --- 1. Build a uniform output time grid across the entire (1- or 2-lap) sim ---
    t_src = sim_result.times
    d_src = sim_result.distances  # per-lap-relative distances (resets at lap 2 in two-lap mode).
    v_src = sim_result.speeds
    labels_src = sim_result.limit_label
    lap_id_src = sim_result.lap_id

    t_total = float(t_src[-1])
    dt = telemetry_dt_ms / 1000.0
    n = int(np.floor(t_total / dt)) + 1
    if n < 2:
        n = 2

    t_out = np.arange(n) * dt
    t_out = np.clip(t_out, t_src[0], t_src[-1])

    # --- 2. Per-output-sample lap_id (nearest-neighbour from sim grid) ---
    idx_nn = np.searchsorted(t_src, t_out, side="left")
    idx_nn = np.clip(idx_nn, 0, len(t_src) - 1)
    if lap_id_src is not None:
        lap_out = lap_id_src[idx_nn]
    else:
        lap_out = np.ones(n, dtype=int)

    # --- 3. Build a continuous-distance source for interpolation ---
    # The sim's `distances` array resets at the lap 2 boundary; build a
    # continuous (per-output-sample) distance from sim's per-lap-relative
    # distance + lap_offset. Interpolate against time on this continuous grid.
    if lap_id_src is not None and np.any(lap_id_src == 2):
        # lap 1 starts at offset 0, lap 2 at offset = track_total_length_m
        lap_offset_src = np.where(lap_id_src == 2, float(track_total_length_m), 0.0)
        d_cont_src = d_src + lap_offset_src
    else:
        d_cont_src = d_src

    d_cont_out = np.interp(t_out, t_src, d_cont_src)
    v_out_ms = np.interp(t_out, t_src, v_src)
    v_out_kmh = v_out_ms * 3.6

    # Per-lap distance (resets at lap boundary).
    if lap_id_src is not None and np.any(lap_id_src == 2):
        d_per_lap = np.where(lap_out == 2,
                             d_cont_out - float(track_total_length_m),
                             d_cont_out)
        d_per_lap = np.clip(d_per_lap, 0.0, None)
    else:
        d_per_lap = d_cont_out

    labels_out = (
        labels_src[idx_nn] if labels_src is not None
        else np.array(["accel"] * n, dtype=object)
    )

    # --- 4. Layer 1: limit-label rule -> piecewise gas/brake ---
    gas, brake = _layer1_limit_label(labels_out, v_out_ms, car)

    # --- 5. Layer 2: corner-shape heuristic (operates on the CONTINUOUS distance
    #       so transitions across the lap-1/lap-2 boundary are treated as ordinary
    #       transitions, matching spec §20.6). ---
    if driver is not None:
        trail_m = float(getattr(driver, "trail_brake_m", 0.0))
        ramp_m = float(getattr(driver, "throttle_ramp_m", 0.0))
        if trail_m > 0.0:
            _apply_trail_brake(brake, labels_out, d_cont_out, trail_m)
        if ramp_m > 0.0:
            _apply_throttle_ramp(gas, labels_out, d_cont_out, ramp_m)

    # --- 6. Layer 3: driver-lag 1st-order low-pass ---
    if driver is not None:
        tau = float(getattr(driver, "driver_tau_s", 0.0))
        if tau > 0.0:
            alpha = dt / (tau + dt)
            gas = _iir_lowpass(gas, alpha)
            brake = _iir_lowpass(brake, alpha)

    # --- 7. normalizedCarPosition (per-lap, resets at lap boundary) ---
    if track_total_length_m <= 0:
        track_total_length_m = float(d_src[-1]) if d_src[-1] > 0 else 1.0
    norm = np.clip(d_per_lap / track_total_length_m, 0.0, 1.0 - 1e-9)

    # --- 8. Write CSV ---
    with open(output_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(list(AC_HEADER))
        for i in range(n):
            w.writerow([
                int(round(i * telemetry_dt_ms)),
                f"{gas[i]:.4f}",
                f"{brake[i]:.4f}",
                f"{float(d_per_lap[i]):.4f}",
                f"{float(v_out_kmh[i]):.4f}",
                f"{float(norm[i]):.8f}",
                int(lap_out[i]),
            ])


def _layer1_limit_label(labels, v_ms, car):
    """Layer 1: derive piecewise gas/brake from the per-sample binding label.

    Returns (gas, brake) numpy arrays, both shape `(n,)`, float64.
    """
    n = len(labels)
    gas = np.zeros(n)
    brake = np.zeros(n)
    for i in range(n):
        lab = labels[i]
        if lab == "accel":
            gas[i] = 1.0
            brake[i] = 0.0
        elif lab == "brake":
            gas[i] = 0.0
            brake[i] = 1.0
        else:  # corner-bound: partial throttle to hold v
            v = float(v_ms[i])
            drag = car.drag_force(v)
            rr = car.rolling_resistance(v)
            required = drag + rr  # zero net accel through the apex
            max_force = car.max_traction_force(v)
            ratio = 0.0 if max_force <= 1e-6 else required / max_force
            gas[i] = float(np.clip(ratio, 0.0, 1.0))
            brake[i] = 0.0
    return gas, brake


def _find_label_transitions(labels):
    """Yield (i_prev, i_next, prev_label, next_label) for every label change.

    i_prev is the last index of the run before the transition; i_next is the
    first index of the run after. Single-sample runs are included.
    """
    n = len(labels)
    if n < 2:
        return
    for i in range(1, n):
        if labels[i] != labels[i - 1]:
            yield i - 1, i, labels[i - 1], labels[i]


def _apply_trail_brake(brake, labels, distances, trail_m):
    """Layer 2 (a): linear brake taper over the last `trail_m` of every
    `brake -> corner|accel` transition, replacing the constant `brake=1.0` with
    a 1.0 -> 0.0 linear-in-distance ramp.

    Operates in-place on `brake`. `distances` must be monotonically
    non-decreasing (i.e. the continuous-distance grid -- the caller is
    responsible for passing the right one).
    """
    n = len(brake)
    if n < 2 or trail_m <= 0:
        return
    for i_prev, i_next, lab_prev, lab_next in _find_label_transitions(labels):
        if lab_prev != "brake":
            continue
        if lab_next not in ("corner", "accel"):
            continue
        # Taper over the `trail_m` immediately preceding the transition (index i_next).
        d_end = float(distances[i_next])
        d_start = d_end - trail_m
        # Walk backward over brake-region samples and replace `brake` with the
        # linear ramp value.
        j = i_prev
        while j >= 0 and labels[j] == "brake" and distances[j] >= d_start:
            # ramp: 1.0 at d_start, 0.0 at d_end -> brake = (d_end - d[j]) / trail_m
            ramp_val = float(np.clip((d_end - float(distances[j])) / trail_m, 0.0, 1.0))
            brake[j] = min(brake[j], ramp_val)
            j -= 1


def _apply_throttle_ramp(gas, labels, distances, ramp_m):
    """Layer 2 (b): linear throttle ramp over the first `ramp_m` of every
    `corner -> accel` transition, replacing the constant `gas=1.0` with a
    `corner_exit_gas_value` -> 1.0 linear-in-distance ramp.

    Operates in-place on `gas`. The starting value is the corner-region's
    partial throttle at the transition (i.e. `gas[i_prev]`).
    """
    n = len(gas)
    if n < 2 or ramp_m <= 0:
        return
    for i_prev, i_next, lab_prev, lab_next in _find_label_transitions(labels):
        if lab_prev != "corner":
            continue
        if lab_next != "accel":
            continue
        # Starting throttle: the corner-region partial throttle at i_prev.
        gas_start = float(gas[i_prev])
        if gas_start >= 0.999:
            continue  # nothing to ramp -- already at full throttle.
        d_start = float(distances[i_next])
        d_end = d_start + ramp_m
        j = i_next
        while j < n and labels[j] == "accel" and distances[j] <= d_end:
            frac = float(np.clip((float(distances[j]) - d_start) / ramp_m, 0.0, 1.0))
            ramp_val = gas_start + (1.0 - gas_start) * frac
            # Only lower or hold the existing accel-region gas (which is 1.0).
            gas[j] = min(gas[j], ramp_val)
            j += 1


def _iir_lowpass(x, alpha):
    """First-order IIR low-pass: y[n] = y[n-1] + alpha * (x[n] - y[n-1]).

    Initial condition y[0] = x[0]. Pure-numpy implementation.
    """
    n = len(x)
    if n == 0:
        return x
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, n):
        y[i] = y[i - 1] + alpha * (x[i] - y[i - 1])
    return y
