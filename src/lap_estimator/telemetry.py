"""AC telemetry CSV parsing + merging with a track CSV by distance."""
from __future__ import annotations

import csv

import numpy as np

REQUIRED_COLUMNS = ("timestamp_ms", "gas", "brake", "distanceTraveled",
                    "speedKmh", "normalizedCarPosition")


def read_ac_log(path):
    """Parse an AC telemetry CSV into a dict of numpy arrays.

    Hard error if any required column is missing.
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
        rows = list(reader)
    if not rows:
        raise ValueError(f"AC telemetry CSV {path} has no data rows")

    out = {}
    for col in REQUIRED_COLUMNS:
        out[col] = np.array([float(r[col]) for r in rows], dtype=float)

    # AC logs are usually time-ordered but a single "lap N" CSV can wrap with
    # the next-lap rows appended at the front. Sort by timestamp so downstream
    # math (lap-time = max-min, merge by distance) is stable.
    order = np.argsort(out["timestamp_ms"])
    if not np.array_equal(order, np.arange(len(order))):
        for col in REQUIRED_COLUMNS:
            out[col] = out[col][order]
    return out


def lap_time_seconds(telem):
    """Real lap time from timestamps (seconds)."""
    ts = telem["timestamp_ms"]
    return float((ts.max() - ts.min()) / 1000.0)


def merge_with_track(telem, track):
    """Interpolate track features (radius, gradient) onto telemetry samples.

    `track` must be a Track loaded via `Track.from_csv(...)` (i.e. CSV-backed).
    Returns a dict of numpy arrays with telemetry fields plus track-derived
    `distance_m`, `radius_m`, and (when available) `gradient_pct`.
    """
    if not getattr(track, "is_csv_backed", False):
        raise ValueError("merge_with_track requires a CSV-backed track")
    d_src = track.csv_data["distance_m"]
    r_src = track.csv_data["radius_m"]

    dist = telem["distanceTraveled"].copy()
    # The AC log's distanceTraveled is cumulative across the session; the
    # track CSV's distance_m starts at 0 each lap. Normalise.
    dist = dist - dist.min()
    # Clamp so np.interp doesn't extrapolate wildly off the end of the track.
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
    return out
