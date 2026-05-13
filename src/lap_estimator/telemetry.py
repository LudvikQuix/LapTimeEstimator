"""AC telemetry CSV parsing + merging with a track CSV by distance.

v1.1: tolerates an optional trailing `lap` column (present in sim-emitted
telemetry from `sim_telemetry.write_synthetic_log`). When present, callers
can filter by lap before merging (default consumer: `fit_driver.py` picks
lap 2 by default).
"""
from __future__ import annotations

import csv

import numpy as np

REQUIRED_COLUMNS = ("timestamp_ms", "gas", "brake", "distanceTraveled",
                    "speedKmh", "normalizedCarPosition")
OPTIONAL_COLUMNS = ("lap",)


def read_ac_log(path):
    """Parse an AC telemetry CSV into a dict of numpy arrays.

    Hard error if any required column is missing. The optional `lap` column
    (sim-emitted only) is exposed when present.
    """
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"Empty AC telemetry CSV: {path}")
        missing = set(REQUIRED_COLUMNS) - set(reader.fieldnames)
        if missing:
            raise ValueError(
                f"AC telemetry CSV {path} missing required columns: {sorted(missing)}"
            )
        present_optional = [c for c in OPTIONAL_COLUMNS if c in reader.fieldnames]
        rows = list(reader)
    if not rows:
        raise ValueError(f"AC telemetry CSV {path} has no data rows")

    out = {}
    for col in REQUIRED_COLUMNS:
        out[col] = np.array([float(r[col]) for r in rows], dtype=float)
    for col in present_optional:
        # `lap` is an int; keep it as int.
        out[col] = np.array([int(float(r[col])) for r in rows], dtype=int)

    # Sort by timestamp (lap-wrap-at-front fix). Apply to all columns.
    order = np.argsort(out["timestamp_ms"])
    if not np.array_equal(order, np.arange(len(order))):
        for col in list(out.keys()):
            out[col] = out[col][order]
    return out


def lap_time_seconds(telem):
    """Real lap time from timestamps (seconds).

    When a `lap` column is present and contains multiple laps, this returns the
    full span (lap 1 + lap 2). Callers that care about a specific lap should
    filter first via `filter_to_lap`.
    """
    ts = telem["timestamp_ms"]
    return float((ts.max() - ts.min()) / 1000.0)


def filter_to_lap(telem, lap_value):
    """Return a new telem dict containing only rows where `lap == lap_value`.

    If the input has no `lap` column, returns the input unchanged.
    """
    if "lap" not in telem:
        return telem
    mask = telem["lap"] == int(lap_value)
    if not mask.any():
        raise ValueError(
            f"filter_to_lap: no rows for lap={lap_value}; "
            f"available laps={sorted(set(int(x) for x in telem['lap']))}"
        )
    out = {}
    for k, v in telem.items():
        out[k] = v[mask]
    return out


def merge_with_track(telem, track):
    """Interpolate track features (radius, gradient) onto telemetry samples."""
    if not getattr(track, "is_csv_backed", False):
        raise ValueError("merge_with_track requires a CSV-backed track")
    d_src = track.csv_data["distance_m"]
    r_src = track.csv_data["radius_m"]

    dist = telem["distanceTraveled"].copy()
    # AC log's distanceTraveled is session-cumulative; track CSV starts at 0.
    dist = dist - dist.min()
    dist = np.clip(dist, d_src[0], d_src[-1])

    radius = np.interp(dist, d_src, r_src)
    out = {
        "timestamp_ms": telem["timestamp_ms"],
        "gas": telem["gas"],
        "brake": telem["brake"],
        "distance_m": dist,
        "speedKmh": telem["speedKmh"],
        "speed_ms": telem["speedKmh"] / 3.6,
        "normalizedCarPosition": telem["normalizedCarPosition"],
        "radius_m": radius,
    }
    if "gradient_pct" in track.csv_data:
        out["gradient_pct"] = np.interp(dist, d_src, track.csv_data["gradient_pct"])
    if "lap" in telem:
        out["lap"] = telem["lap"]
    return out
