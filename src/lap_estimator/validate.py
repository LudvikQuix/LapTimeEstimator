"""Cross-track validation: compare a SimResult against a real AC telemetry lap.

Reused by `lap.py --validate-against`. v1.1: validation targets sim lap 2 by
default (the flying-lap analogue of a real learned lap). `target_lap=1` is the
fallback for `--single-lap` mode.
"""
from __future__ import annotations

import csv
import os
from dataclasses import dataclass

import numpy as np

from .telemetry import lap_time_seconds, merge_with_track, read_ac_log


@dataclass
class ValidationResult:
    real_lap_time_s: float
    sim_lap_time_s: float
    delta_s: float
    delta_pct: float
    verdict: str
    bins: list
    target_lap: int = 2


def _verdict(delta_s: float, real_lap_s: float) -> str:
    if real_lap_s <= 0:
        return "UNKNOWN"
    pct = abs(delta_s / real_lap_s) * 100.0
    if abs(delta_s) < 3.0 and pct < 5.0:
        return "GOOD"
    if pct <= 10.0:
        return "LOOSE"
    return "BAD"


def _slice_to_target_lap(sim_result, target_lap: int):
    """Return (distances, times-from-lap-start, speeds_kmh) for the target lap.

    For two-lap results: extracts the rows where `lap_id == target_lap` and
    re-zeros the time so `times[0] == 0` (lap-relative).
    For single-lap results: returns the full sim arrays as-is.
    """
    if sim_result.lap_id is not None and sim_result.two_lap:
        mask = sim_result.lap_id == target_lap
        if not mask.any():
            # Fall back to the whole result.
            mask = np.ones(len(sim_result.distances), dtype=bool)
        d = sim_result.distances[mask]
        t = sim_result.times[mask]
        v = sim_result.speeds[mask] * 3.6
        # Re-zero time to lap start.
        t = t - t[0]
        return d, t, v
    return (
        sim_result.distances,
        sim_result.times - sim_result.times[0],
        sim_result.speeds * 3.6,
    )


def validate_lap(car, track, sim_result, real_telem_path, *, bin_m: int = 100,
                 per_corner: bool = False, target_lap: int = 2) -> ValidationResult:
    """Compare `sim_result` against a real AC telemetry CSV.

    `track` must be CSV-backed. Returns a ValidationResult; callers handle I/O.
    `target_lap` selects which sim lap to compare against (default 2 = flying).
    """
    telem = read_ac_log(real_telem_path)
    real_lap = lap_time_seconds(telem)
    merged = merge_with_track(telem, track)

    sim_d, sim_t, sim_v_kmh = _slice_to_target_lap(sim_result, target_lap)

    real_d = merged["distance_m"]
    real_v_kmh = merged["speedKmh"]
    real_t = (merged["timestamp_ms"] - merged["timestamp_ms"][0]) / 1000.0

    sim_lap = float(sim_t[-1])
    delta_s = sim_lap - real_lap
    delta_pct = (delta_s / real_lap * 100.0) if real_lap > 0 else 0.0
    verdict = _verdict(delta_s, real_lap)

    total = float(sim_d[-1])
    if per_corner:
        spans = _corner_spans(track, total)
    else:
        edges = np.arange(0.0, total + bin_m, bin_m)
        spans = [(float(edges[i]), float(min(edges[i + 1], total)), "bin")
                 for i in range(len(edges) - 1)]

    bins = []
    for (a, b, kind) in spans:
        t_sim = _segment_time(sim_d, sim_t, a, b)
        t_real = _segment_time(real_d, real_t, a, b)
        v_sim = _segment_avg(sim_d, sim_v_kmh, a, b)
        v_real = _segment_avg(real_d, real_v_kmh, a, b)
        bins.append({
            "bin_start_m": a,
            "bin_end_m": b,
            "kind": kind,
            "t_sim_s": t_sim,
            "t_real_s": t_real,
            "delta_s": t_sim - t_real,
            "v_avg_sim_kmh": v_sim,
            "v_avg_real_kmh": v_real,
        })

    return ValidationResult(
        real_lap_time_s=real_lap,
        sim_lap_time_s=sim_lap,
        delta_s=delta_s,
        delta_pct=delta_pct,
        verdict=verdict,
        bins=bins,
        target_lap=target_lap,
    )


def _segment_time(d, t, a, b):
    a = max(a, float(d[0]))
    b = min(b, float(d[-1]))
    if b <= a:
        return 0.0
    ta = float(np.interp(a, d, t))
    tb = float(np.interp(b, d, t))
    return tb - ta


def _segment_avg(d, vals, a, b):
    mask = (d >= a) & (d <= b)
    if not np.any(mask):
        return float(np.interp((a + b) / 2.0, d, vals))
    return float(np.mean(vals[mask]))


def _corner_spans(track, total_m):
    """Generate (start, end, kind) spans over a CSV-backed track."""
    if not getattr(track, "is_csv_backed", False):
        return [(0.0, total_m, "lap")]
    d = track.csv_data["distance_m"]
    r = track.csv_data["radius_m"]
    in_corner = False
    start = 0.0
    spans = []
    for i in range(len(d)):
        is_c = r[i] < 500.0
        if is_c and not in_corner:
            if d[i] > start:
                spans.append((start, float(d[i]), "straight"))
            start = float(d[i])
            in_corner = True
        elif (not is_c) and in_corner:
            spans.append((start, float(d[i]), "corner"))
            start = float(d[i])
            in_corner = False
    tail_kind = "corner" if in_corner else "straight"
    if total_m > start:
        spans.append((start, total_m, tail_kind))
    return spans


def write_bins_csv(result: ValidationResult, output_path: str) -> None:
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    cols = ["bin_start_m", "bin_end_m", "kind", "t_sim_s", "t_real_s",
            "delta_s", "v_avg_sim_kmh", "v_avg_real_kmh"]
    with open(output_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for b in result.bins:
            w.writerow([
                f"{b['bin_start_m']:.2f}", f"{b['bin_end_m']:.2f}",
                b["kind"],
                f"{b['t_sim_s']:.4f}", f"{b['t_real_s']:.4f}",
                f"{b['delta_s']:.4f}",
                f"{b['v_avg_sim_kmh']:.2f}", f"{b['v_avg_real_kmh']:.2f}",
            ])
