"""CSV loaders for the Rerun replay tool.

Three input shapes are supported, all already produced by the simulator /
fitter pipeline:

1. **Track layout CSV** (e.g. `tracks_csv/<track>/layout_sprint_a.csv`).
   Required columns: `distance_m`, `x`, `y`, `z`, `width_left_m`,
   `width_right_m`. The track is rendered on the **(x, z) ground plane**
   (AC world frame; `y` is vertical elevation).

2. **Ghost (real AC reference) telemetry CSV** — files named
   `*_sim_telemetry.csv`. Despite the `sim_` prefix these are recorded
   from the real driver via the AC bridge. Required columns:
   `timestamp_ms`, `distanceTraveled`, `speedKmh`. Optional scalars:
   `gas`, `brake`.

3. **Sim trace CSV** — files named `*_sim_trace.csv` or
   `*_sim_trace_slip.csv`. Required columns: `distance_m`, `time_s`,
   `sim_speed_ms`. Optional: `sim_speed_kmh`, `ai_speed_kmh`, `lap`.

Neither trace CSV carries world position — both record progress as
arclength along the centreline. The loader reconstructs `(x, z)` per
sample by linear interpolation against the track layout's
`distance_m -> (x, z)` lookup. This is the same convention the simulator
and validator use to place samples back on the map.
"""
from __future__ import annotations

import csv
from dataclasses import dataclass

import numpy as np


# ----------------------------- Track --------------------------------------- #

@dataclass
class Track:
    """Centreline + edges in the (x, z) ground plane (AC world frame)."""

    name: str
    distance_m: np.ndarray  # (N,)
    x: np.ndarray           # (N,)
    z: np.ndarray           # (N,) — longitudinal-ish in AC world frame
    elevation_m: np.ndarray  # (N,)
    left_xz: np.ndarray     # (N, 2)
    right_xz: np.ndarray    # (N, 2)
    width_left_m: np.ndarray  # (N,)
    width_right_m: np.ndarray  # (N,)

    @property
    def centerline_xz(self) -> np.ndarray:
        return np.column_stack([self.x, self.z])

    @property
    def total_length_m(self) -> float:
        return float(self.distance_m[-1])


def _read_csv(path: str) -> tuple[list[str], list[dict]]:
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"Empty CSV: {path}")
        return list(reader.fieldnames), list(reader)


def _col(rows: list[dict], name: str) -> np.ndarray:
    return np.array(
        [float(r[name]) if r.get(name) not in (None, "") else np.nan for r in rows],
        dtype=float,
    )


def _maybe_col(rows: list[dict], fields: set[str], name: str) -> np.ndarray | None:
    if name not in fields:
        return None
    return _col(rows, name)


def load_track(path: str, name: str | None = None) -> Track:
    """Load a layout CSV and resolve left/right edges in the (x, z) plane.

    Edges are built by stepping the centreline tangent's perpendicular
    by `width_left_m` and `width_right_m` per sample. Convention used:
      tangent_hat = d(x,z)/d(arclength)
      left_normal = rotate(tangent, +90 degrees) = (-tz, +tx)
      right_normal = -left_normal
    This matches `prep/prep_track.py`'s convention for left/right widths
    (left of travel direction, right of travel direction).
    """
    fields, rows = _read_csv(path)
    field_set = set(fields)
    required = {"distance_m", "x", "y", "z", "width_left_m", "width_right_m"}
    missing = required - field_set
    if missing:
        raise ValueError(
            f"Track CSV {path} missing required columns: {sorted(missing)}"
        )
    d = _col(rows, "distance_m")
    x = _col(rows, "x")
    y = _col(rows, "y")
    z = _col(rows, "z")
    wl = _col(rows, "width_left_m")
    wr = _col(rows, "width_right_m")

    # Tangent via central differences on (x, z). Handle endpoints with
    # forward/backward differences to keep array length stable.
    tx = np.gradient(x)
    tz = np.gradient(z)
    norm = np.hypot(tx, tz)
    norm[norm < 1e-9] = 1.0
    tx /= norm
    tz /= norm

    # Left-of-travel normal is the +90 deg rotation of tangent.
    nx_left, nz_left = -tz, tx
    left_x = x + nx_left * wl
    left_z = z + nz_left * wl
    right_x = x - nx_left * wr
    right_z = z - nz_left * wr

    return Track(
        name=name or path,
        distance_m=d,
        x=x,
        z=z,
        elevation_m=y,
        left_xz=np.column_stack([left_x, left_z]),
        right_xz=np.column_stack([right_x, right_z]),
        width_left_m=wl,
        width_right_m=wr,
    )


# ----------------------------- Lap traces ---------------------------------- #

@dataclass
class LapTrace:
    """Generic time + arclength + reconstructed (x, z) + optional scalars.

    All arrays are aligned to length `N` (the trace's own sample count).
    `t_s` starts at zero. `xz` is reconstructed against the supplied track.
    `scalars` maps a short name (e.g. ``speed_ms``) to a numpy array of
    the same length; missing channels are simply absent from the dict.
    """

    label: str
    source_path: str
    t_s: np.ndarray
    distance_m: np.ndarray
    xz: np.ndarray  # (N, 2)
    scalars: dict[str, np.ndarray]
    missing_columns: list[str]

    @property
    def duration_s(self) -> float:
        return float(self.t_s[-1] - self.t_s[0])


def _interp_xz(track: Track, distance_m: np.ndarray) -> np.ndarray:
    """Linearly interpolate centreline (x, z) at arbitrary arclength.

    Arclengths beyond the track end are clipped to the last point (rather
    than wrapping) so that any out-of-range sample at the end of a lap
    renders at the start/finish line instead of jumping to nonsense.
    """
    d_clipped = np.clip(distance_m, track.distance_m[0], track.distance_m[-1])
    x = np.interp(d_clipped, track.distance_m, track.x)
    z = np.interp(d_clipped, track.distance_m, track.z)
    return np.column_stack([x, z])


def load_ghost_trace(path: str, track: Track, label: str = "ghost") -> LapTrace:
    """Load a ``*_sim_telemetry.csv`` (real AC reference lap)."""
    fields, rows = _read_csv(path)
    field_set = set(fields)
    required = {"timestamp_ms", "distanceTraveled", "speedKmh"}
    missing = required - field_set
    if missing:
        raise ValueError(
            f"Ghost telemetry {path} missing required columns: {sorted(missing)}"
        )

    ts_ms = _col(rows, "timestamp_ms")
    t_s = (ts_ms - ts_ms[0]) / 1000.0
    dist = _col(rows, "distanceTraveled")
    speed_kmh = _col(rows, "speedKmh")
    speed_ms = speed_kmh / 3.6
    xz = _interp_xz(track, dist)

    scalars: dict[str, np.ndarray] = {
        "speed_ms": speed_ms,
        "speed_kmh": speed_kmh,
    }
    missing_cols: list[str] = []
    for opt in ("gas", "brake"):
        arr = _maybe_col(rows, field_set, opt)
        if arr is None:
            missing_cols.append(opt)
        else:
            scalars[opt] = arr

    return LapTrace(
        label=label,
        source_path=path,
        t_s=t_s,
        distance_m=dist,
        xz=xz,
        scalars=scalars,
        missing_columns=missing_cols,
    )


def load_sim_trace(path: str, track: Track, label: str = "sim") -> LapTrace:
    """Load a ``*_sim_trace.csv`` (or ``*_sim_trace_slip.csv``) sim output."""
    fields, rows = _read_csv(path)
    field_set = set(fields)
    required = {"distance_m", "time_s", "sim_speed_ms"}
    missing = required - field_set
    if missing:
        raise ValueError(
            f"Sim trace {path} missing required columns: {sorted(missing)}"
        )

    t_raw = _col(rows, "time_s")
    t_s = t_raw - t_raw[0]
    dist = _col(rows, "distance_m")
    speed_ms = _col(rows, "sim_speed_ms")
    speed_kmh = (
        _maybe_col(rows, field_set, "sim_speed_kmh")
        if "sim_speed_kmh" in field_set
        else speed_ms * 3.6
    )
    xz = _interp_xz(track, dist)

    scalars: dict[str, np.ndarray] = {
        "speed_ms": speed_ms,
        "speed_kmh": speed_kmh,
    }
    # Sim traces are not (yet) emitting gas/brake/steer; record as missing
    # so the CLI can surface it in stdout.
    missing_cols: list[str] = []
    for opt in ("gas", "brake", "steer"):
        if opt in field_set:
            scalars[opt] = _col(rows, opt)
        else:
            missing_cols.append(opt)

    return LapTrace(
        label=label,
        source_path=path,
        t_s=t_s,
        distance_m=dist,
        xz=xz,
        scalars=scalars,
        missing_columns=missing_cols,
    )
