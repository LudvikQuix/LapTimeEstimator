"""Track definitions as sequences of segments (length, radius).

Radius > 0 = right turn, < 0 = left turn, 0 = straight.
All distances in meters.
"""
import json
import numpy as np


class TrackSegment:
    __slots__ = ('length', 'radius')

    def __init__(self, length, radius=0.0):
        self.length = length
        self.radius = radius

    @property
    def is_straight(self):
        return abs(self.radius) < 1.0

    @property
    def abs_radius(self):
        return abs(self.radius) if not self.is_straight else float('inf')


class Track:
    def __init__(self, name, segments):
        self.name = name
        self.segments = segments
        self.total_length = sum(s.length for s in segments)

    @classmethod
    def from_json(cls, filepath):
        with open(filepath) as f:
            data = json.load(f)
        segs = [TrackSegment(s['length'], s.get('radius', 0)) for s in data['segments']]
        return cls(data['name'], segs)

    def to_points(self, ds=1.0):
        """Convert segments to evenly-spaced point array with curvature."""
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

    def __repr__(self):
        n_corners = sum(1 for s in self.segments if not s.is_straight)
        return f"Track('{self.name}', {self.total_length:.0f}m, {n_corners} corners)"


# --- Built-in track definitions ---

def monza():
    """Autodromo Nazionale Monza (simplified)."""
    return Track("Monza", [
        TrackSegment(800),              # Main straight
        TrackSegment(120, 85),          # Variante del Rettifilo chicane R
        TrackSegment(50, -60),          # Variante del Rettifilo chicane L
        TrackSegment(350),              # Short straight
        TrackSegment(200, 290),         # Curva Grande
        TrackSegment(450),              # Straight to Variante della Roggia
        TrackSegment(80, 45),           # Variante della Roggia R
        TrackSegment(60, -40),          # Variante della Roggia L
        TrackSegment(250),              # Straight
        TrackSegment(350, 160),         # Lesmo 1
        TrackSegment(180),              # Short straight
        TrackSegment(250, 80),          # Lesmo 2
        TrackSegment(550),              # Straight to Ascari
        TrackSegment(100, 75),          # Ascari chicane R
        TrackSegment(80, -55),          # Ascari chicane L
        TrackSegment(100, 100),         # Ascari exit
        TrackSegment(700),              # Back straight
        TrackSegment(250, 65),          # Parabolica
        TrackSegment(350),              # Run to start/finish
    ])


def spa():
    """Spa-Francorchamps (simplified)."""
    return Track("Spa-Francorchamps", [
        TrackSegment(250),              # Start/finish
        TrackSegment(180, 110),         # La Source hairpin
        TrackSegment(800),              # Eau Rouge straight approach
        TrackSegment(150, -120),        # Eau Rouge left
        TrackSegment(200, 200),         # Raidillon right
        TrackSegment(600),              # Kemmel straight
        TrackSegment(120, 65),          # Les Combes R
        TrackSegment(80, -50),          # Les Combes L
        TrackSegment(180),              # Short straight
        TrackSegment(250, -110),        # Malmedy
        TrackSegment(350),              # Straight
        TrackSegment(180, 45),          # Rivage hairpin
        TrackSegment(450),              # Downhill straight
        TrackSegment(300, -180),        # Pouhon double left
        TrackSegment(250, -150),        # Pouhon exit
        TrackSegment(300),              # Straight
        TrackSegment(120, 60),          # Fagnes chicane R
        TrackSegment(80, -55),          # Fagnes chicane L
        TrackSegment(350),              # Straight
        TrackSegment(200, 100),         # Stavelot R
        TrackSegment(150, -120),        # Stavelot L
        TrackSegment(750),              # Blanchimont straight
        TrackSegment(350, 250),         # Blanchimont
        TrackSegment(250),              # Approach to Bus Stop
        TrackSegment(100, 40),          # Bus Stop R
        TrackSegment(80, -35),          # Bus Stop L
        TrackSegment(400),              # Run to start
    ])


def nurburgring_gp():
    """Nurburgring GP circuit (simplified)."""
    return Track("Nurburgring GP", [
        TrackSegment(600),              # Start/finish straight
        TrackSegment(200, 70),          # Turn 1 (Yokohama-S R)
        TrackSegment(100, -55),         # Turn 2 (Yokohama-S L)
        TrackSegment(250),              # Short straight
        TrackSegment(180, -90),         # Mercedes Arena L
        TrackSegment(120, 80),          # Mercedes Arena R
        TrackSegment(150, -70),         # Mercedes Arena exit L
        TrackSegment(200),              # Straight
        TrackSegment(150, 50),          # Valvoline Kurve
        TrackSegment(350),              # Straight
        TrackSegment(120, -40),         # Ford-Kurve
        TrackSegment(250),              # Straight
        TrackSegment(180, 35),          # Dunlop hairpin
        TrackSegment(300),              # Straight
        TrackSegment(160, 150),         # Bit-Kurve
        TrackSegment(200),              # Short straight
        TrackSegment(300, 80),          # Veedol chicane complex
        TrackSegment(400),              # Back straight
        TrackSegment(250, 55),          # Coca-Cola Kurve
        TrackSegment(200),              # Run to start
    ])


def brands_hatch_gp():
    """Brands Hatch GP circuit (simplified)."""
    return Track("Brands Hatch GP", [
        TrackSegment(350),              # Start straight
        TrackSegment(180, 75),          # Paddock Hill Bend
        TrackSegment(200),              # Hailwood Hill
        TrackSegment(120, -60),         # Druids hairpin
        TrackSegment(250),              # Graham Hill Bend approach
        TrackSegment(180, 120),         # Graham Hill Bend
        TrackSegment(200, -90),         # Cooper Straight entry
        TrackSegment(450),              # Cooper Straight
        TrackSegment(200, 80),          # Surtees
        TrackSegment(350),              # Pilgrim's Drop straight
        TrackSegment(250, 150),         # Hawthorn Bend
        TrackSegment(250),              # Westfield straight
        TrackSegment(100, 35),          # Westfield Bend
        TrackSegment(150),              # Short straight
        TrackSegment(120, -50),         # Dingle Dell
        TrackSegment(80, 45),           # Dingle Dell corner
        TrackSegment(250),              # Stirlings straight
        TrackSegment(200, 200),         # Clark Curve
        TrackSegment(300),              # Run to start
    ])


BUILTIN_TRACKS = {
    'monza': monza,
    'spa': spa,
    'nurburgring': nurburgring_gp,
    'brands_hatch': brands_hatch_gp,
}
