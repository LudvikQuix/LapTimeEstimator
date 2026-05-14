"""Emit a synthetic telemetry CSV with AC's exact schema plus a trailing `lap` column.

v1.2.1 two-layer pipeline (spec §14.3; the v1.1 Layer 3 IIR low-pass is removed):
  1. Layer 1 -- limit-label rule: accel->1/0, brake->0/1, corner->partial-throttle.
  2. Layer 2 -- corner-shape heuristic: trail-brake taper + throttle ramp (per-driver
     distances `trail_brake_m` / `throttle_ramp_m`; either set to 0 disables that leg).

Layer 2's output is the final output; no post-hoc smoothing is applied (spec §14.3).

Two-lap (v1.1, spec §20): callers pass a SimResult with `lap_id` per-point.
`timestamp_ms` is monotonic across the lap boundary; `distanceTraveled` and
`normalizedCarPosition` reset to 0 at the start of lap 2. The trailing `lap`
column carries values from {1, 2} (or constant 1 in single-lap mode).

v2 (spec §7.12, §14.6, §14.12 item 14): when a `StintResult` is supplied, the
schema gains 12 trailing per-wheel state columns (temp/wear/pressure for
FL/FR/RL/RR). Values forward-filled from the per-segment state arrays
produced by `simulate_stint`. Always emitted in stint mode -- constant at
setup defaults when calibration.measured == False.
"""
from __future__ import annotations

import csv
import os

import numpy as np

AC_HEADER = (
    "timestamp_ms", "gas", "brake", "distanceTraveled",
    "speedKmh", "normalizedCarPosition", "lap",
)

STATE_HEADER = (
    "tempFL", "tempFR", "tempRL", "tempRR",
    "wearFL", "wearFR", "wearRL", "wearRR",
    "pressureFL", "pressureFR", "pressureRL", "pressureRR",
)
WHEELS = ("FL", "FR", "RL", "RR")


def write_synthetic_log(sim_result, car, driver, track_total_length_m: float,
                        output_path: str, *, telemetry_dt_ms: int = 10) -> None:
    """Write the synthetic telemetry CSV (AC schema + `lap` column).

    Args:
        sim_result: SimResult OR StintResult. For StintResult (v2), the 12
            trailing per-wheel state columns are appended.
        car: Car physics object (used to scale `gas` on corner-bound samples).
        driver: Driver object (v1.2.1; reads `trail_brake_m`, `throttle_ramp_m`
            only -- `driver_tau_s` is no longer consumed, per spec §14.3).
            Pass `None` for bang-bang behaviour.
        track_total_length_m: per-lap length used for normalized position.
        output_path: where to write the CSV (parent dirs created if needed).
        telemetry_dt_ms: cadence of the output grid.
    """
    # Stint dispatch: detect StintResult by duck-typing.
    if hasattr(sim_result, "per_lap_sim_results") and hasattr(sim_result, "tyre_state_history"):
        _write_stint_log(sim_result, car, driver, track_total_length_m, output_path,
                         telemetry_dt_ms=telemetry_dt_ms)
        return

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

    # --- 6. normalizedCarPosition (per-lap, resets at lap boundary) ---
    if track_total_length_m <= 0:
        track_total_length_m = float(d_src[-1]) if d_src[-1] > 0 else 1.0
    norm = np.clip(d_per_lap / track_total_length_m, 0.0, 1.0 - 1e-9)

    # --- 7. Write CSV ---
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


def _write_stint_log(stint, car, driver, track_total_length_m: float,
                     output_path: str, *, telemetry_dt_ms: int = 10) -> None:
    """Write a synthetic stint telemetry CSV with the 12 trailing state columns.

    Stitches per-lap SimResults into a single monotonic-time stream. Constant
    state (no evolution) when `n_laps == 2` and `calibration.measured == False`
    -- in that case `stint.per_lap_sim_results[0]` is the v1.1 two-lap result
    and we delegate to the legacy single-result writer with extra state cols.
    """
    if telemetry_dt_ms <= 0:
        raise ValueError("telemetry_dt_ms must be > 0")

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

    n_laps = stint.n_laps
    per_lap = stint.per_lap_sim_results
    per_pt_states = stint.per_point_states

    # Build a stitched continuous-time grid that walks all laps end-to-end.
    # For the back-compat n_laps==2 / uncalibrated path, per_lap[0] is the
    # v1.1 two-lap result and per_lap[1] points to the same object. Detect that
    # and short-circuit to the legacy path.
    if n_laps == 2 and len(per_lap) == 2 and per_lap[0] is per_lap[1]:
        _write_stint_compat_v11(stint, car, driver, track_total_length_m,
                                output_path, telemetry_dt_ms=telemetry_dt_ms)
        return

    # Lap-by-lap stitch: concatenate distances/speeds/times/labels with cumulative
    # `t_offset` between laps so the output is monotonic in time. Per-lap state
    # arrays also concatenate. lap_id is each lap's index (1-based).
    d_all, v_all, t_all, lab_all, lap_id_all = [], [], [], [], []
    state_cat: dict = {key: {w: [] for w in WHEELS} for key in ("temp_C", "wear_pct", "pressure_psi")}
    t_offset = 0.0
    for k, r in enumerate(per_lap):
        d_all.append(r.distances)
        v_all.append(r.speeds)
        t_all.append(r.times + t_offset)
        lab_all.append(r.limit_label if r.limit_label is not None
                       else np.array(["accel"] * len(r.distances), dtype=object))
        lap_id_all.append(np.full(len(r.distances), k + 1, dtype=int))
        n_pts = len(r.distances)
        if k < len(per_pt_states):
            for key in ("temp_C", "wear_pct", "pressure_psi"):
                for w in WHEELS:
                    state_cat[key][w].append(per_pt_states[k][key][w])
        else:
            # No per-point state: forward-fill from end-of-lap state.
            end_state = stint.tyre_state_history[k + 1]
            for w in WHEELS:
                state_cat["temp_C"][w].append(np.full(n_pts, end_state.temp_C[w]))
                state_cat["wear_pct"][w].append(np.full(n_pts, end_state.wear_pct[w]))
                state_cat["pressure_psi"][w].append(np.full(n_pts, end_state.pressure_psi[w]))
        t_offset = float(t_all[-1][-1])

    distances_src = np.concatenate(d_all)
    speeds_src = np.concatenate(v_all)
    times_src = np.concatenate(t_all)
    labels_src = np.concatenate(lab_all)
    lap_id_src = np.concatenate(lap_id_all)
    state_src = {key: {w: np.concatenate(state_cat[key][w]) for w in WHEELS}
                 for key in ("temp_C", "wear_pct", "pressure_psi")}

    t_total = float(times_src[-1])
    dt = telemetry_dt_ms / 1000.0
    n = max(2, int(np.floor(t_total / dt)) + 1)
    t_out = np.clip(np.arange(n) * dt, times_src[0], times_src[-1])

    idx_nn = np.clip(np.searchsorted(times_src, t_out, side="left"),
                     0, len(times_src) - 1)
    lap_out = lap_id_src[idx_nn]
    labels_out = labels_src[idx_nn]

    # Continuous distance: shift each lap by (lap_index - 1) * track_length.
    lap_offset_src = (lap_id_src - 1).astype(float) * float(track_total_length_m)
    d_cont_src = distances_src + lap_offset_src
    d_cont_out = np.interp(t_out, times_src, d_cont_src)
    v_out_ms = np.interp(t_out, times_src, speeds_src)
    v_out_kmh = v_out_ms * 3.6
    d_per_lap = np.where(lap_out > 1,
                         d_cont_out - (lap_out - 1) * float(track_total_length_m),
                         d_cont_out)
    d_per_lap = np.clip(d_per_lap, 0.0, None)

    # Layer 1 / Layer 2 over the stitched stream.
    gas, brake = _layer1_limit_label(labels_out, v_out_ms, car)
    if driver is not None:
        trail_m = float(getattr(driver, "trail_brake_m", 0.0))
        ramp_m = float(getattr(driver, "throttle_ramp_m", 0.0))
        if trail_m > 0.0:
            _apply_trail_brake(brake, labels_out, d_cont_out, trail_m)
        if ramp_m > 0.0:
            _apply_throttle_ramp(gas, labels_out, d_cont_out, ramp_m)

    if track_total_length_m <= 0:
        track_total_length_m = float(distances_src[-1]) if distances_src[-1] > 0 else 1.0
    norm = np.clip(d_per_lap / track_total_length_m, 0.0, 1.0 - 1e-9)

    # Per-sample state via nearest-neighbour from the sim grid.
    temp_out = {w: state_src["temp_C"][w][idx_nn] for w in WHEELS}
    wear_out = {w: state_src["wear_pct"][w][idx_nn] for w in WHEELS}
    pres_out = {w: state_src["pressure_psi"][w][idx_nn] for w in WHEELS}

    with open(output_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(list(AC_HEADER) + list(STATE_HEADER))
        for i in range(n):
            w.writerow([
                int(round(i * telemetry_dt_ms)),
                f"{gas[i]:.4f}",
                f"{brake[i]:.4f}",
                f"{float(d_per_lap[i]):.4f}",
                f"{float(v_out_kmh[i]):.4f}",
                f"{float(norm[i]):.8f}",
                int(lap_out[i]),
                f"{float(temp_out['FL'][i]):.4f}",
                f"{float(temp_out['FR'][i]):.4f}",
                f"{float(temp_out['RL'][i]):.4f}",
                f"{float(temp_out['RR'][i]):.4f}",
                f"{float(wear_out['FL'][i]):.4f}",
                f"{float(wear_out['FR'][i]):.4f}",
                f"{float(wear_out['RL'][i]):.4f}",
                f"{float(wear_out['RR'][i]):.4f}",
                f"{float(pres_out['FL'][i]):.4f}",
                f"{float(pres_out['FR'][i]):.4f}",
                f"{float(pres_out['RL'][i]):.4f}",
                f"{float(pres_out['RR'][i]):.4f}",
            ])


def _write_stint_compat_v11(stint, car, driver, track_total_length_m: float,
                            output_path: str, *, telemetry_dt_ms: int = 10) -> None:
    """v1.1 two-lap byte-compat path with constant-state trailing 12 columns.

    Spec §11.31: with `n_laps == 2` and `calibration.measured == False`, lap-1/
    lap-2 times must reproduce v1.2.1 within ±0.05 s; the 12 new state columns
    are emitted but constant at setup defaults.
    """
    result = stint.per_lap_sim_results[0]
    initial = stint.tyre_state_history[0]

    if telemetry_dt_ms <= 0:
        raise ValueError("telemetry_dt_ms must be > 0")
    if result.times is None or len(result.times) == 0:
        raise ValueError("SimResult.times is empty")

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

    # Re-run the legacy interp pipeline (mirrors the non-stint path below).
    t_src = result.times
    d_src = result.distances
    v_src = result.speeds
    labels_src = result.limit_label
    lap_id_src = result.lap_id
    t_total = float(t_src[-1])
    dt = telemetry_dt_ms / 1000.0
    n = max(2, int(np.floor(t_total / dt)) + 1)
    t_out = np.clip(np.arange(n) * dt, t_src[0], t_src[-1])
    idx_nn = np.clip(np.searchsorted(t_src, t_out, side="left"), 0, len(t_src) - 1)
    lap_out = lap_id_src[idx_nn] if lap_id_src is not None else np.ones(n, dtype=int)
    if lap_id_src is not None and np.any(lap_id_src == 2):
        lap_offset_src = np.where(lap_id_src == 2, float(track_total_length_m), 0.0)
        d_cont_src = d_src + lap_offset_src
    else:
        d_cont_src = d_src
    d_cont_out = np.interp(t_out, t_src, d_cont_src)
    v_out_ms = np.interp(t_out, t_src, v_src)
    v_out_kmh = v_out_ms * 3.6
    if lap_id_src is not None and np.any(lap_id_src == 2):
        d_per_lap = np.where(lap_out == 2,
                             d_cont_out - float(track_total_length_m), d_cont_out)
        d_per_lap = np.clip(d_per_lap, 0.0, None)
    else:
        d_per_lap = d_cont_out
    labels_out = labels_src[idx_nn] if labels_src is not None else np.array(["accel"] * n, dtype=object)
    gas, brake = _layer1_limit_label(labels_out, v_out_ms, car)
    if driver is not None:
        trail_m = float(getattr(driver, "trail_brake_m", 0.0))
        ramp_m = float(getattr(driver, "throttle_ramp_m", 0.0))
        if trail_m > 0.0:
            _apply_trail_brake(brake, labels_out, d_cont_out, trail_m)
        if ramp_m > 0.0:
            _apply_throttle_ramp(gas, labels_out, d_cont_out, ramp_m)
    if track_total_length_m <= 0:
        track_total_length_m = float(d_src[-1]) if d_src[-1] > 0 else 1.0
    norm = np.clip(d_per_lap / track_total_length_m, 0.0, 1.0 - 1e-9)

    with open(output_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(list(AC_HEADER) + list(STATE_HEADER))
        for i in range(n):
            w.writerow([
                int(round(i * telemetry_dt_ms)),
                f"{gas[i]:.4f}",
                f"{brake[i]:.4f}",
                f"{float(d_per_lap[i]):.4f}",
                f"{float(v_out_kmh[i]):.4f}",
                f"{float(norm[i]):.8f}",
                int(lap_out[i]),
                f"{initial.temp_C['FL']:.4f}", f"{initial.temp_C['FR']:.4f}",
                f"{initial.temp_C['RL']:.4f}", f"{initial.temp_C['RR']:.4f}",
                f"{initial.wear_pct['FL']:.4f}", f"{initial.wear_pct['FR']:.4f}",
                f"{initial.wear_pct['RL']:.4f}", f"{initial.wear_pct['RR']:.4f}",
                f"{initial.pressure_psi['FL']:.4f}", f"{initial.pressure_psi['FR']:.4f}",
                f"{initial.pressure_psi['RL']:.4f}", f"{initial.pressure_psi['RR']:.4f}",
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
