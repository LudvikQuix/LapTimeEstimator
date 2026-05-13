"""Track definitions: legacy segment-based built-ins plus rich per-point CSV.

Legacy `segments`-based tracks remain for built-ins (`monza`, `spa`, ...).
The preferred input for v1+ is the rich per-point CSV produced by
`prep/prep_track.py` and consumed via `Track.from_csv(path)`.
"""
import csv
import json
import os

import numpy as np


class TrackSegment:
    __slots__ = ("length", "radius")

    def __init__(self, length, radius=0.0):
        self.length = length
        self.radius = radius

    @property
    def is_straight(self):
        return abs(self.radius) < 1.0

    @property
    def abs_radius(self):
        return abs(self.radius) if not self.is_straight else float("inf")


class Track:
    """Unified track object.

    Two backings:
      - Segment list (legacy built-ins, JSON tracks). `csv_data` is None.
      - CSV-backed (rich per-point). `csv_data` holds parsed columns; `segments`
        is a single placeholder segment so legacy reporting still works.
    """

    def __init__(self, name, segments, csv_data=None):
        self.name = name
        self.segments = segments
        self.csv_data = csv_data
        if csv_data is not None:
            self.total_length = float(csv_data["distance_m"][-1])
        else:
            self.total_length = sum(s.length for s in segments)

    # ---- factories ----

    @classmethod
    def from_json(cls, filepath):
        with open(filepath) as f:
            data = json.load(f)
        segs = [TrackSegment(s["length"], s.get("radius", 0)) for s in data["segments"]]
        return cls(data["name"], segs)

    @classmethod
    def from_csv(cls, filepath):
        """Load a rich per-point CSV produced by `prep_track.py`.

        Required columns: distance_m, radius_m, speed_ms.
        Optional columns are kept when present: segment_length_m,
        gradient_pct, elevation_m, x, y, z, speed_kmh, width_*.
        """
        required = {"distance_m", "radius_m", "speed_ms"}
        with open(filepath, newline="") as f:
            reader = csv.DictReader(f)
            if reader.fieldnames is None:
                raise ValueError(f"Empty CSV: {filepath}")
            field_set = set(reader.fieldnames)
            missing = required - field_set
            if missing:
                raise ValueError(
                    f"Track CSV {filepath} missing required columns: {sorted(missing)}"
                )
            rows = list(reader)

        def col_strict(name):
            vals = []
            for r in rows:
                v = r.get(name, "")
                vals.append(float(v) if v not in (None, "") else np.nan)
            return np.array(vals, dtype=float)

        data = {
            "distance_m": col_strict("distance_m"),
            "radius_m": col_strict("radius_m"),
            "speed_ms": col_strict("speed_ms"),
        }
        for opt in (
            "segment_length_m",
            "gradient_pct",
            "elevation_m",
            "x",
            "y",
            "z",
            "speed_kmh",
            "width_left_m",
            "width_right_m",
            "width_total_m",
        ):
            if opt in field_set:
                data[opt] = col_strict(opt)

        name = os.path.splitext(os.path.basename(filepath))[0]
        # A single placeholder segment so legacy reporting (which iterates
        # `track.segments`) does not blow up.
        placeholder = [TrackSegment(float(data["distance_m"][-1]), 0.0)]
        return cls(name, placeholder, csv_data=data)

    # ---- access ----

    @property
    def is_csv_backed(self):
        return self.csv_data is not None

    @property
    def total_length_m(self):
        return self.total_length

    def to_points(self, ds=1.0):
        """Return (distances, radii) sampled at uniform `ds`.

        For CSV-backed tracks, interpolates curvature (1/R) on a uniform grid to
        avoid spikes at sample boundaries. For segment-backed tracks, replicates
        the legacy behaviour.
        """
        if self.csv_data is not None:
            d_src = self.csv_data["distance_m"]
            r_src = self.csv_data["radius_m"]
            curv_src = 1.0 / np.clip(np.abs(r_src), 1e-3, None)
            d_total = float(d_src[-1])
            n = max(2, int(d_total / ds) + 1)
            d_out = np.linspace(0.0, d_total, n)
            curv_out = np.interp(d_out, d_src, curv_src)
            r_out = 1.0 / np.clip(curv_out, 1e-6, None)
            r_out = np.clip(r_out, 1.0, 2000.0)
            return d_out, r_out

        distances = []
        radii = []
        d = 0.0
        for seg in self.segments:
            n = max(1, int(seg.length / ds))
            step = seg.length / n
            for _ in range(n):
                distances.append(d)
                radii.append(seg.abs_radius)
                d += step
        return np.array(distances), np.array(radii)

    def to_ai_reference(self, ds=1.0):
        """Return AI reference speeds (m/s) on the same grid as `to_points(ds)`.

        Returns None for segment-backed tracks.
        """
        if self.csv_data is None:
            return None
        d_src = self.csv_data["distance_m"]
        v_src = self.csv_data["speed_ms"]
        d_total = float(d_src[-1])
        n = max(2, int(d_total / ds) + 1)
        d_out = np.linspace(0.0, d_total, n)
        return np.interp(d_out, d_src, v_src)

    def __repr__(self):
        if self.is_csv_backed:
            return f"Track('{self.name}', {self.total_length:.0f}m, CSV)"
        n_corners = sum(1 for s in self.segments if not s.is_straight)
        return f"Track('{self.name}', {self.total_length:.0f}m, {n_corners} corners)"


# --- Built-in track definitions (legacy; CSV-backed tracks are preferred) ---

def monza():
    """Autodromo Nazionale Monza (simplified)."""
    return Track("Monza", [
        TrackSegment(800),
        TrackSegment(120, 85),
        TrackSegment(50, -60),
        TrackSegment(350),
        TrackSegment(200, 290),
        TrackSegment(450),
        TrackSegment(80, 45),
        TrackSegment(60, -40),
        TrackSegment(250),
        TrackSegment(350, 160),
        TrackSegment(180),
        TrackSegment(250, 80),
        TrackSegment(550),
        TrackSegment(100, 75),
        TrackSegment(80, -55),
        TrackSegment(100, 100),
        TrackSegment(700),
        TrackSegment(250, 65),
        TrackSegment(350),
    ])


def spa():
    """Spa-Francorchamps (simplified)."""
    return Track("Spa-Francorchamps", [
        TrackSegment(250),
        TrackSegment(180, 110),
        TrackSegment(800),
        TrackSegment(150, -120),
        TrackSegment(200, 200),
        TrackSegment(600),
        TrackSegment(120, 65),
        TrackSegment(80, -50),
        TrackSegment(180),
        TrackSegment(250, -110),
        TrackSegment(350),
        TrackSegment(180, 45),
        TrackSegment(450),
        TrackSegment(300, -180),
        TrackSegment(250, -150),
        TrackSegment(300),
        TrackSegment(120, 60),
        TrackSegment(80, -55),
        TrackSegment(350),
        TrackSegment(200, 100),
        TrackSegment(150, -120),
        TrackSegment(750),
        TrackSegment(350, 250),
        TrackSegment(250),
        TrackSegment(100, 40),
        TrackSegment(80, -35),
        TrackSegment(400),
    ])


def nurburgring_gp():
    """Nurburgring GP circuit (simplified)."""
    return Track("Nurburgring GP", [
        TrackSegment(600),
        TrackSegment(200, 70),
        TrackSegment(100, -55),
        TrackSegment(250),
        TrackSegment(180, -90),
        TrackSegment(120, 80),
        TrackSegment(150, -70),
        TrackSegment(200),
        TrackSegment(150, 50),
        TrackSegment(350),
        TrackSegment(120, -40),
        TrackSegment(250),
        TrackSegment(180, 35),
        TrackSegment(300),
        TrackSegment(160, 150),
        TrackSegment(200),
        TrackSegment(300, 80),
        TrackSegment(400),
        TrackSegment(250, 55),
        TrackSegment(200),
    ])


def brands_hatch_gp():
    """Brands Hatch GP circuit (simplified)."""
    return Track("Brands Hatch GP", [
        TrackSegment(350),
        TrackSegment(180, 75),
        TrackSegment(200),
        TrackSegment(120, -60),
        TrackSegment(250),
        TrackSegment(180, 120),
        TrackSegment(200, -90),
        TrackSegment(450),
        TrackSegment(200, 80),
        TrackSegment(350),
        TrackSegment(250, 150),
        TrackSegment(250),
        TrackSegment(100, 35),
        TrackSegment(150),
        TrackSegment(120, -50),
        TrackSegment(80, 45),
        TrackSegment(250),
        TrackSegment(200, 200),
        TrackSegment(300),
    ])


# Deprecation note (kept verbal — legacy callers still work):
# Built-in tracks are retained for backwards-compatibility. Prefer
# CSV-backed tracks produced by `prep/prep_track.py`.
BUILTIN_TRACKS = {
    "monza": monza,
    "spa": spa,
    "nurburgring": nurburgring_gp,
    "brands_hatch": brands_hatch_gp,
}
