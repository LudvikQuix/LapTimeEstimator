"""AC telemetry CSV parsing + merging with a track CSV by distance.

v1.1: tolerates an optional trailing `lap` column (present in sim-emitted
telemetry from `sim_telemetry.write_synthetic_log`). When present, callers
can filter by lap before merging (default consumer: `fit_driver.py` picks
lap 2 by default).

v1.2: opportunistically loads `steerAngle` (warning once if absent across a
batch is the CLI's job, not ours). Any extra numeric columns present in the
CSV pass through unchanged onto the merged frame so callers can read them.
"""
from __future__ import annotations

import csv

import numpy as np

REQUIRED_COLUMNS = ("timestamp_ms", "gas", "brake", "distanceTraveled",
                    "speedKmh", "normalizedCarPosition")
OPTIONAL_INT_COLUMNS = ("lap",)
# Known opportunistic float passthroughs. Anything else that parses as float
# also passes through (see `read_ac_log`).
OPTIONAL_FLOAT_COLUMNS = ("steerAngle",)


def read_ac_log(path):
    """Parse an AC telemetry CSV into a dict of numpy arrays.

    Hard error if any required column is missing. Optional `lap` column is
    exposed when present (int). Opportunistic `steerAngle` and any other
    numeric extras pass through as float arrays.
    """
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"Empty AC telemetry CSV: {path}")
        fieldnames = list(reader.fieldnames)
        missing = set(REQUIRED_COLUMNS) - set(fieldnames)
        if missing:
            raise ValueError(
                f"AC telemetry CSV {path} missing required columns: {sorted(missing)}"
            )
        rows = list(reader)
    if not rows:
        raise ValueError(f"AC telemetry CSV {path} has no data rows")

    out = {}
    for col in REQUIRED_COLUMNS:
        out[col] = np.array([float(r[col]) for r in rows], dtype=float)
    # Integer passthrough (only `lap` today).
    for col in OPTIONAL_INT_COLUMNS:
        if col in fieldnames:
            out[col] = np.array([int(float(r[col])) for r in rows], dtype=int)
    # Float passthroughs: known opportunistic columns first, then any other
    # CSV column that parses as a float. Non-numeric extras are silently
    # ignored (warnings live at the CLI layer).
    handled = set(REQUIRED_COLUMNS) | set(OPTIONAL_INT_COLUMNS)
    for col in fieldnames:
        if col in handled:
            continue
        values: list[float] = []
        ok = True
        for r in rows:
            raw = r.get(col, "")
            if raw is None or raw == "":
                values.append(float("nan"))
                continue
            try:
                values.append(float(raw))
            except (TypeError, ValueError):
                ok = False
                break
        if ok and values:
            out[col] = np.array(values, dtype=float)

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
    """Interpolate track features (radius, gradient) onto telemetry samples.

    Any extra numeric columns present on `telem` (e.g. `steerAngle`) are
    passed through to the merged frame unchanged.
    """
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
    # Passthrough: any numeric extras the loader picked up.
    handled = {
        "timestamp_ms", "gas", "brake", "distanceTraveled", "speedKmh",
        "normalizedCarPosition", "lap",
    }
    for col, arr in telem.items():
        if col in handled:
            continue
        out[col] = arr
    return out
