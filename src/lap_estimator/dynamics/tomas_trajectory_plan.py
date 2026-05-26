"""Tomas-trajectory plan source for the v3 slip controllers.

Experimental innovation (2026-05-24) parallel to the Phase 5.0.7 QP-tune
work. The DP plan in :mod:`longitudinal_planner` builds a
friction-envelope-bounded reference (``v_corner = sqrt(D_lat*g/|kappa|)``)
with conservative ``safety_margin`` (Phase 5.0.1 default 0.94) and a
chicane-radius cap (Phase 5.0.2). That plan is structurally an L-shape
and never crosses the friction envelope.

Tomas's recorded lap 5 at ``.tmp/tomas_lap5_rich.csv`` IS however a known
feasible trajectory for the BMW 1M / Sprint A / Pacejka calibration —
he hit 1:47.568, 1.06x v_crit at the chicane, ~1.27 g p95 lateral. If
that trajectory is feasible in AC, the v3 plant (which we've now
calibrated against Tomas's open-loop replay to ~0.5 s) should also be
able to follow it. The hack: just hand the recorded ``v(s)`` to the
controller as a reference and let the combined-slip QP figure out the
inputs.

Inputs
------

This module reads the same CSV that the calibration scripts in
``.tmp/`` already consume. We expect exactly *one* lap of Sprint A
telemetry (5379 rows, lap-time 1:47.568 in the canonical
``tomas_lap5_rich.csv``). Columns we touch:

- ``distanceTraveled`` (cumulative across the AC session — we delta
  it against the first sample to get a per-lap 0..L distance grid).
- ``speedKmh`` (converted to m/s).

Behaviour
---------

We resample Tomas's ``(s, v)`` onto the track's existing
``distance_m`` samples (the same grid the DP planner uses, so the
controllers' ``np.interp(self._ds, plan.distances, plan.speeds)``
look-ups produce a smooth reference). The output is a
:class:`LongitudinalPlan` so callers don't need to special-case the
source.

No safety_margin, no chicane_safety_mult, no DP sweeps — pure recorded
target. If Tomas drove at 17 m/s through the chicane, that's the
target. The MPC's combined-slip QP enforces feasibility; if Tomas
drove it the QP can (in principle) solve for inputs that produce it.

Architecture doc: :file:`docs/architecture-v3-tomas-trajectory-injection.md`.
"""

from __future__ import annotations

import csv
import os
from typing import TYPE_CHECKING

import numpy as np

from .longitudinal_planner import LongitudinalPlan

if TYPE_CHECKING:
    from ..track import Track


# Default location of the canonical Tomas lap-5 telemetry. Lives under
# ``.tmp/`` per the repo's scratch-hygiene convention.
DEFAULT_TOMAS_CSV = os.path.join(".tmp", "tomas_lap5_rich.csv")

# Speed floor (m/s) used to clip the resampled reference. Matches
# :data:`longitudinal_planner.V_MIN` so the controllers see the same
# "never ask for stopped" guarantee they'd get from the DP plan.
V_MIN = 5.0


def build_tomas_plan(
    track: "Track",
    *,
    csv_path: str | None = None,
) -> LongitudinalPlan:
    """Return a :class:`LongitudinalPlan` whose speeds trace Tomas's lap.

    Loads ``csv_path`` (default :data:`DEFAULT_TOMAS_CSV`), strips any
    finish-line wrap, normalises ``distanceTraveled`` to start at 0,
    converts ``speedKmh`` to m/s, and interpolates onto the track's
    ``distance_m`` samples.

    Parameters
    ----------
    track : Track
        CSV-backed Sprint A track. We need its ``distance_m`` grid to
        resample onto so the controllers' look-ups produce a smooth
        reference.
    csv_path : str | None
        Override the input telemetry path. ``None`` -> use the
        package default. Caller-supplied path is used verbatim; the
        CSV must have at minimum ``distanceTraveled`` and
        ``speedKmh`` columns (the rest are ignored here).

    Returns
    -------
    LongitudinalPlan
        ``distances`` = ``track.csv_data["distance_m"]`` copy.
        ``speeds`` = ``np.interp(distances, s_tomas, v_tomas)``, clipped
        to ``[V_MIN, v_top]`` where ``v_top`` is the max observed
        Tomas speed. ``chicane_report`` is ``None`` — no DP / chicane
        cap applies.

    Raises
    ------
    FileNotFoundError
        Telemetry CSV is missing.
    ValueError
        Required columns missing, or the lap doesn't cover the track
        length (off by > 5 m total).
    """
    if not getattr(track, "is_csv_backed", False):
        raise ValueError("build_tomas_plan requires a CSV-backed track.")

    path = csv_path if csv_path is not None else DEFAULT_TOMAS_CSV
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"Tomas telemetry CSV not found: {path}. The canonical lap-5 "
            f"file lives at {DEFAULT_TOMAS_CSV}; pass csv_path= to override."
        )

    s_tomas, v_tomas = _read_lap_distance_speed(path)
    track_distances = np.asarray(track.csv_data["distance_m"], dtype=float)
    track_length = float(track_distances[-1])
    tomas_length = float(s_tomas[-1])
    if abs(tomas_length - track_length) > 5.0:
        raise ValueError(
            f"Tomas lap length {tomas_length:.1f} m does not match track "
            f"length {track_length:.1f} m (diff {tomas_length - track_length:+.1f} m). "
            f"Wrong track CSV or wrong telemetry file."
        )

    # Resample onto the track grid. np.interp handles the (very small)
    # endpoint mismatch by extrapolating with the boundary value.
    speeds = np.interp(track_distances, s_tomas, v_tomas)
    v_top = float(np.max(v_tomas))
    speeds = np.clip(speeds, V_MIN, v_top)

    return LongitudinalPlan(
        distances=track_distances.copy(),
        speeds=speeds,
        chicane_report=None,
    )


def _read_lap_distance_speed(path: str) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(s_lap_m, v_ms)`` from the Tomas CSV.

    Strategy: read all rows, take ``distanceTraveled`` as a cumulative
    distance, find the first / last rows of the single lap by looking
    for a monotonic positive-delta segment, then offset distances to
    start at 0. The canonical lap-5 file has a 1-row finish-line wrap
    at the end (``currentTime`` resets); we detect & drop any trailing
    row where the distance jumps backwards or forwards by more than a
    plausible 5 m step.

    Returns numpy arrays sorted by distance (monotonic).
    """
    s_raw: list[float] = []
    v_raw: list[float] = []
    required = ("distanceTraveled", "speedKmh")
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"Empty Tomas CSV: {path}")
        missing = [c for c in required if c not in reader.fieldnames]
        if missing:
            raise ValueError(
                f"Tomas CSV {path} missing required columns: {missing}"
            )
        for row in reader:
            s_raw.append(float(row["distanceTraveled"]))
            v_raw.append(float(row["speedKmh"]))
    if len(s_raw) < 10:
        raise ValueError(f"Tomas CSV {path} has too few rows: {len(s_raw)}")

    s_arr = np.asarray(s_raw, dtype=float)
    v_arr_kmh = np.asarray(v_raw, dtype=float)

    # Drop the finish-line wrap: any trailing rows whose distance does
    # not exceed the previous row by a sensible per-tick step (< 5 m
    # at 50 Hz = 250 m/s; we'd never see > 5 m in 20 ms). This catches
    # the lap-wrap row where currentTime resets to ~0 but distance
    # continues to advance — we want to keep those if the distance is
    # still monotonic. The real cut is: stop at the LAST monotonic
    # index.
    last = len(s_arr) - 1
    while last > 0 and s_arr[last] <= s_arr[last - 1]:
        last -= 1
    s_arr = s_arr[: last + 1]
    v_arr_kmh = v_arr_kmh[: last + 1]

    # Normalise to lap-relative distance.
    s_lap = s_arr - float(s_arr[0])
    v_ms = v_arr_kmh / 3.6

    # Force strict monotonicity. np.interp does NOT require strictly
    # increasing xp, but its behaviour on ties is undefined; we coerce
    # ties forward by a hair so the resample is deterministic.
    eps = 1e-6
    for i in range(1, len(s_lap)):
        if s_lap[i] <= s_lap[i - 1]:
            s_lap[i] = s_lap[i - 1] + eps

    return s_lap, v_ms
