"""Tomas racing-line reconstruction (ground-truth recipe, 2026-05-24).

Sibling of :mod:`tomas_trajectory_plan`. Where that module exposes Tomas's
recorded ``v(s)`` as a controller reference, this one exposes Tomas's
recorded **world-frame (x, z) trajectory** so the controller's cross-track
projection can be computed against *his* line rather than the centreline.

Data source (ground truth)
--------------------------

The canonical lap-5 telemetry at ``.tmp/tomas_lap5_rich.csv`` carries
direct world-frame positions: ``carCoordinates_x`` and
``carCoordinates_z`` (the two horizontal AC-world axes; ``y`` is
vertical). These are the same frame the track CSV uses
(``track.csv_data['x', 'z']``) — confirmed by the lap-start sample
sitting ~3-4 m from the centreline sample 0.

No reconstruction, no integration, no sign / offset calibration is
needed: ``(x, z) = (carCoordinates_x, carCoordinates_z)`` directly.
The arrays are resampled onto the track's ``distance_m`` grid using
Tomas's ``distanceTraveled`` as the s parameter so the controllers'
``np.interp`` look-ups produce the same shape they get from the
centreline.

History (deprecated empirical recipe)
-------------------------------------

A prior iteration of this module integrated ``localVelocity_x`` /
``localVelocity_z`` rotated by ``heading`` (sign / offset calibrated
against the centreline tangent at s=0). That recipe drifted by ~4-6 m
RMS across the lap and inflated the chicane-apex lateral offset
(reported +9 m left at s≈680 m vs ground truth +5 m left). The
ground-truth recipe replaces it; see
``docs/architecture-v3-tomas-trajectory-injection.md`` for the
re-scored line-override conclusion.

Public API
----------

- :func:`build_tomas_line` — returns a :class:`TomasLine` keyed by the
  track's ``distance_m`` grid. ``xs`` and ``zs`` replace the centreline
  in the controllers' line-following geometry; everything else (off-
  track abort against ``track.csv_data['x','z']``, lap-completion
  projection in :mod:`solver`) stays untouched.
"""

from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from ..track import Track


# Default location of the canonical Tomas lap-5 telemetry. Lives under
# ``.tmp/`` per the repo's scratch-hygiene convention.
DEFAULT_TOMAS_CSV = os.path.join(".tmp", "tomas_lap5_rich.csv")

# Minimum reconstructed-trajectory length (m) we accept before raising. A
# severely truncated CSV would slip through ``_read_lap`` if the
# monotonic-trim happened too early; this is a fail-loud floor.
_MIN_LAP_LENGTH_M = 100.0


@dataclass(frozen=True)
class TomasLine:
    """Ground-truth (x, z) trajectory for Tomas's lap.

    Attributes
    ----------
    distances : np.ndarray
        Track ``distance_m`` grid (m). Same shape as
        ``track.csv_data['distance_m']``.
    xs : np.ndarray
        World-frame ``x`` of Tomas's line at each ``distances`` sample.
    zs : np.ndarray
        World-frame ``z`` (the second horizontal axis; AC has ``y`` for
        elevation) of Tomas's line at each ``distances`` sample.
    closure_error_m : float
        Distance from the last recorded ``(x, z)`` to the first
        recorded ``(x, z)``. Reported for diagnostics; a clean
        single-lap CSV produces ~0-3 m on Sprint A.
    """

    distances: np.ndarray
    xs: np.ndarray
    zs: np.ndarray
    closure_error_m: float


def build_tomas_line(
    track: "Track",
    *,
    csv_path: str | None = None,
) -> TomasLine:
    """Return Tomas's ground-truth ``(x(s), z(s))`` for the active track.

    Reads ``carCoordinates_x`` and ``carCoordinates_z`` from the
    telemetry CSV and resamples them onto ``track.csv_data['distance_m']``
    using ``distanceTraveled`` as the s parameter. No coordinate
    transformation is applied — AC world frame and the track CSV share
    the same axes.

    Parameters
    ----------
    track : Track
        CSV-backed Sprint A track. We need its ``distance_m`` grid (the
        output is resampled onto it).
    csv_path : str | None
        Override the input telemetry path. ``None`` -> use the package
        default ``.tmp/tomas_lap5_rich.csv``. Required columns:

        - ``distanceTraveled`` (s parameter)
        - ``carCoordinates_x``
        - ``carCoordinates_z``

    Returns
    -------
    TomasLine

    Raises
    ------
    FileNotFoundError
        Telemetry CSV missing.
    ValueError
        Required columns missing, track is not CSV-backed, or the
        recorded lap length disagrees with the track length by > 5 m.
    """
    if not getattr(track, "is_csv_backed", False):
        raise ValueError("build_tomas_line requires a CSV-backed track.")

    path = csv_path if csv_path is not None else DEFAULT_TOMAS_CSV
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"Tomas telemetry CSV not found: {path}. Canonical lap-5 "
            f"file lives at {DEFAULT_TOMAS_CSV}; pass csv_path= to override."
        )

    tom = _read_lap(path)
    ds_c = np.asarray(track.csv_data["distance_m"], dtype=float)
    track_length = float(ds_c[-1])
    tomas_length = float(tom["dist"][-1] - tom["dist"][0])
    if abs(tomas_length - track_length) > 5.0:
        raise ValueError(
            f"Tomas lap length {tomas_length:.1f} m does not match track "
            f"length {track_length:.1f} m (diff {tomas_length - track_length:+.1f} m). "
            f"Wrong track CSV or wrong telemetry file."
        )

    # Cheap length-of-trajectory sanity (rough).
    seg = np.hypot(np.diff(tom["x"]), np.diff(tom["z"])).sum()
    if seg < _MIN_LAP_LENGTH_M:
        raise ValueError(
            f"Recorded Tomas trajectory length {seg:.1f} m below floor "
            f"{_MIN_LAP_LENGTH_M} m — telemetry likely truncated."
        )

    # Closure error: distance from last sample to first (single lap).
    closure = math.hypot(tom["x"][-1] - tom["x"][0],
                         tom["z"][-1] - tom["z"][0])

    # Resample (x, z) onto the track distance_m grid using
    # the recorded distanceTraveled as the s parameter.
    s_tomas = tom["dist"] - tom["dist"][0]
    # Force strict monotonicity (small ties at finish line).
    eps = 1e-6
    for i in range(1, len(s_tomas)):
        if s_tomas[i] <= s_tomas[i - 1]:
            s_tomas[i] = s_tomas[i - 1] + eps
    xs_out = np.interp(ds_c, s_tomas, tom["x"])
    zs_out = np.interp(ds_c, s_tomas, tom["z"])

    return TomasLine(
        distances=ds_c.copy(),
        xs=xs_out,
        zs=zs_out,
        closure_error_m=float(closure),
    )


# ---------------------------------------------------------------------------
# Internals
# ---------------------------------------------------------------------------


def _read_lap(path: str) -> dict:
    """Read one lap of Tomas telemetry as a dict of float arrays.

    Trims any trailing finish-line wrap (non-monotonic ``distanceTraveled``)
    so the output is exactly one lap.
    """
    required = ("distanceTraveled", "carCoordinates_x", "carCoordinates_z")
    out: dict[str, list[float]] = {"dist": [], "x": [], "z": []}
    with open(path, newline="") as f:
        rd = csv.DictReader(f)
        if rd.fieldnames is None:
            raise ValueError(f"Empty Tomas CSV: {path}")
        missing = [c for c in required if c not in rd.fieldnames]
        if missing:
            raise ValueError(
                f"Tomas CSV {path} missing required columns: {missing}. "
                f"Need ground-truth world positions for the line "
                f"reconstruction (carCoordinates_x/z)."
            )
        for row in rd:
            out["dist"].append(float(row["distanceTraveled"]))
            out["x"].append(float(row["carCoordinates_x"]))
            out["z"].append(float(row["carCoordinates_z"]))
    arrs = {k: np.asarray(v, dtype=float) for k, v in out.items()}
    if len(arrs["dist"]) < 10:
        raise ValueError(
            f"Tomas CSV {path} has too few rows: {len(arrs['dist'])}"
        )
    # Trim trailing wrap: drop trailing rows whose distance does not
    # exceed the previous row.
    s = arrs["dist"]
    last = len(s) - 1
    while last > 0 and s[last] <= s[last - 1]:
        last -= 1
    return {k: v[: last + 1] for k, v in arrs.items()}


__all__ = [
    "DEFAULT_TOMAS_CSV",
    "TomasLine",
    "build_tomas_line",
]
