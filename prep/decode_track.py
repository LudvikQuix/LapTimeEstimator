#!/usr/bin/env python3
"""Decode Assetto Corsa track data (fast_lane.ai / ideal_line.ai).

Usage:
    python decode_track.py <path/to/fast_lane.ai> [output.json]

The AI line file contains the racing line as a series of 3D points.
This script converts it into track segments (straights + corners with radii)
suitable for lap time simulation.

Example:
    python decode_track.py tracks/monza/ai/fast_lane.ai monza.json
    python decode_track.py tracks/monza/ai/fast_lane.ai --plot
"""
import struct
import json
import math
import sys
import os


def read_ai_line(filepath):
    """Parse an AC fast_lane.ai or ideal_line.ai file.

    AC AI line format (binary, little-endian):
        4 bytes: header/version (int32)
        4 bytes: number of points (int32)
        4 bytes: lap length in meters (float32) -- in some versions
        Then per point:
            12 bytes: x, y, z (3x float32) -- world position
            4 bytes: length from start (float32)
            Optionally more fields depending on version

    Returns:
        List of dicts with 'x', 'y', 'z', 'dist' keys
    """
    with open(filepath, 'rb') as f:
        data = f.read()

    pos = 0

    # Read header
    header = struct.unpack_from('<i', data, pos)[0]
    pos += 4

    n_points = struct.unpack_from('<i', data, pos)[0]
    pos += 4

    # Some versions have extra header fields
    # Try to detect format by checking if n_points is reasonable
    if n_points <= 0 or n_points > 100000:
        # Might be a different format, try skipping header
        pos = 0
        n_points = header
        if n_points <= 0 or n_points > 100000:
            raise ValueError(f"Cannot parse AI file: header={header}, n_points={n_points}")

    # Try to figure out record size
    # Standard format: 3 floats (xyz) + 1 float (length) = 16 bytes per point
    # Extended format: also has speed, gas, brake, direction etc.
    remaining = len(data) - pos
    record_size_16 = n_points * 16
    record_size_20 = n_points * 20

    # Version 7 format (most common in modern AC):
    # header(4) + n_detail(4) + n_points(4) + lap_length(4)
    # then n_points * (x,y,z,length + extra fields)
    # Let's try multiple known formats

    points = []

    # Try version 7: 4+4+4+4 header, then 18 floats per point
    if not points:
        points = _try_parse_v7(data)

    # Try simple format: just xyz + cumulative distance
    if not points:
        points = _try_parse_simple(data, n_points, pos)

    # Try extended format
    if not points:
        points = _try_parse_extended(data, n_points, pos)

    if not points:
        raise ValueError("Could not parse AI line file with any known format")

    return points


def _try_parse_v7(data):
    """Parse AC version 7 AI line format."""
    if len(data) < 16:
        return []

    pos = 0
    header = struct.unpack_from('<i', data, pos)[0]
    pos += 4

    if header != 7:
        return []

    n_detail = struct.unpack_from('<i', data, pos)[0]
    pos += 4
    n_points = struct.unpack_from('<i', data, pos)[0]
    pos += 4
    lap_length = struct.unpack_from('<f', data, pos)[0]
    pos += 4

    if n_points <= 0 or n_points > 100000:
        return []

    # Each point: x(4) y(4) z(4) length(4) + extra depending on n_detail
    # Standard: 4 floats base + n_detail * 4 bytes of extra data
    # Typical n_detail values: 1-4
    # Fields: position(xyz), distance, speed, gas, brake, lateral_offset...

    # Record size: 4 floats (xyz + dist) = 16 bytes minimum
    # Plus extra per-detail fields
    record_size = 16 + n_detail * 4

    needed = pos + n_points * record_size
    if needed > len(data):
        # Try without detail fields
        record_size = 16
        needed = pos + n_points * record_size
        if needed > len(data):
            return []

    points = []
    for i in range(n_points):
        x, y, z = struct.unpack_from('<fff', data, pos)
        pos += 12
        dist = struct.unpack_from('<f', data, pos)[0]
        pos += 4
        # Skip extra detail fields
        pos += (record_size - 16)
        points.append({'x': x, 'y': y, 'z': z, 'dist': dist})

    return points


def _try_parse_simple(data, n_points, pos):
    """Parse simple format: n_points then xyz+dist per point."""
    if pos + n_points * 16 > len(data):
        return []

    points = []
    for i in range(n_points):
        x, y, z, dist = struct.unpack_from('<ffff', data, pos)
        pos += 16
        # Sanity check: coordinates should be reasonable
        if abs(x) > 50000 or abs(z) > 50000:
            return []
        points.append({'x': x, 'y': y, 'z': z, 'dist': dist})
        if i > 5:
            break  # quick sanity check

    # If sanity passed, parse all
    if len(points) > 5:
        pos_start = pos - len(points) * 16
        points = []
        pos = pos_start
        for i in range(n_points):
            x, y, z, dist = struct.unpack_from('<ffff', data, pos)
            pos += 16
            points.append({'x': x, 'y': y, 'z': z, 'dist': dist})

    return points


def _try_parse_extended(data, n_points, pos):
    """Try parsing with various record sizes."""
    for extra_bytes in [4, 8, 12, 16, 20, 24, 28, 32]:
        record_size = 16 + extra_bytes
        if pos + n_points * record_size > len(data):
            continue

        points = []
        p = pos
        valid = True
        for i in range(min(10, n_points)):
            x, y, z, dist = struct.unpack_from('<ffff', data, p)
            p += record_size
            if abs(x) > 50000 or abs(z) > 50000 or math.isnan(x):
                valid = False
                break
            points.append({'x': x, 'y': y, 'z': z, 'dist': dist})

        if valid and len(points) > 5:
            # Parse all points
            points = []
            p = pos
            for i in range(n_points):
                x, y, z, dist = struct.unpack_from('<ffff', data, p)
                p += record_size
                points.append({'x': x, 'y': y, 'z': z, 'dist': dist})
            return points

    return []


def points_to_segments(points, min_radius=10, straight_threshold=500, smooth_window=5):
    """Convert a list of XYZ points into track segments (length + radius).

    Uses local curvature calculation from 3 consecutive points.

    Args:
        points: List of dicts with x, y, z, dist
        min_radius: Minimum corner radius to consider (filter noise)
        straight_threshold: Radius above which a segment is considered straight
        smooth_window: Number of points to average curvature over

    Returns:
        List of dicts with 'length' and 'radius' keys
    """
    n = len(points)
    if n < 3:
        return []

    # Calculate curvature at each point using Menger curvature (3-point circle)
    curvatures = [0.0] * n
    for i in range(1, n - 1):
        p0 = points[i - 1]
        p1 = points[i]
        p2 = points[i + 1]

        # Use XZ plane (horizontal) for curvature
        ax, az = p0['x'], p0['z']
        bx, bz = p1['x'], p1['z']
        cx, cz = p2['x'], p2['z']

        # Triangle area * 2
        area2 = abs((bx - ax) * (cz - az) - (cx - ax) * (bz - az))

        # Side lengths
        ab = math.sqrt((bx - ax) ** 2 + (bz - az) ** 2)
        bc = math.sqrt((cx - bx) ** 2 + (cz - bz) ** 2)
        ac = math.sqrt((cx - ax) ** 2 + (cz - az) ** 2)

        denom = ab * bc * ac
        if denom > 0.001:
            curvatures[i] = area2 / denom  # 1/R
        else:
            curvatures[i] = 0.0

        # Sign: cross product for direction
        cross = (bx - ax) * (cz - az) - (bz - az) * (cx - ax)
        if cross < 0:
            curvatures[i] = -curvatures[i]

    # Smooth curvatures
    smoothed = [0.0] * n
    half = smooth_window // 2
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        smoothed[i] = sum(curvatures[lo:hi]) / (hi - lo)

    # Convert to segments by grouping consecutive points with similar curvature
    segments = []
    seg_start = 0
    prev_is_straight = abs(smoothed[0]) < (1.0 / straight_threshold)

    for i in range(1, n):
        cur_is_straight = abs(smoothed[i]) < (1.0 / straight_threshold)

        # Also detect sign changes (left->right corner)
        sign_change = (i > 0 and smoothed[i] * smoothed[i - 1] < 0
                       and not cur_is_straight and not prev_is_straight)

        if cur_is_straight != prev_is_straight or sign_change:
            # End current segment
            seg_len = points[i]['dist'] - points[seg_start]['dist']
            if seg_len < 0:
                seg_len += points[-1]['dist']  # wrap around

            if seg_len > 0:
                if prev_is_straight:
                    segments.append({'length': round(seg_len, 1), 'radius': 0})
                else:
                    avg_curv = sum(smoothed[seg_start:i]) / max(1, i - seg_start)
                    if abs(avg_curv) > 0.0001:
                        radius = 1.0 / avg_curv
                        if abs(radius) >= min_radius:
                            segments.append({'length': round(seg_len, 1), 'radius': round(radius, 1)})
                        else:
                            segments.append({'length': round(seg_len, 1), 'radius': round(min_radius * (1 if radius > 0 else -1), 1)})
                    else:
                        segments.append({'length': round(seg_len, 1), 'radius': 0})

            seg_start = i
            prev_is_straight = cur_is_straight

    # Last segment
    seg_len = points[-1]['dist'] - points[seg_start]['dist']
    if seg_len > 0:
        if prev_is_straight:
            segments.append({'length': round(seg_len, 1), 'radius': 0})
        else:
            avg_curv = sum(smoothed[seg_start:]) / max(1, n - seg_start)
            if abs(avg_curv) > 0.0001:
                radius = 1.0 / avg_curv
                segments.append({'length': round(seg_len, 1), 'radius': round(radius, 1)})
            else:
                segments.append({'length': round(seg_len, 1), 'radius': 0})

    return segments


def decode_track(ai_filepath, track_name=None):
    """Decode an AC track AI line file into a track definition.

    Args:
        ai_filepath: Path to fast_lane.ai or ideal_line.ai
        track_name: Track name (defaults to parent directory name)

    Returns:
        Dict with 'name', 'total_length', 'segments', 'points'
    """
    if track_name is None:
        track_name = os.path.basename(os.path.dirname(os.path.dirname(ai_filepath)))

    points = read_ai_line(ai_filepath)
    segments = points_to_segments(points)

    total_length = sum(s['length'] for s in segments)
    n_corners = sum(1 for s in segments if s['radius'] != 0)

    return {
        'name': track_name,
        'total_length': round(total_length, 1),
        'n_points': len(points),
        'n_segments': len(segments),
        'n_corners': n_corners,
        'segments': segments,
        'points': points,
    }


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    ai_path = sys.argv[1]
    do_plot = '--plot' in sys.argv
    output_path = None
    for arg in sys.argv[2:]:
        if not arg.startswith('--'):
            output_path = arg

    print(f"Reading: {ai_path}")
    result = decode_track(ai_path)

    print(f"Track:    {result['name']}")
    print(f"Length:   {result['total_length']:.0f} m")
    print(f"Points:   {result['n_points']}")
    print(f"Segments: {result['n_segments']} ({result['n_corners']} corners)")
    print()

    # Print segments
    print(f"{'#':>3}  {'Type':<10} {'Length':>8} {'Radius':>8}")
    print("-" * 35)
    for i, seg in enumerate(result['segments']):
        if seg['radius'] == 0:
            stype = "Straight"
            rstr = "-"
        else:
            stype = "Right" if seg['radius'] > 0 else "Left"
            rstr = f"{abs(seg['radius']):.0f}m"
        print(f"{i + 1:>3}  {stype:<10} {seg['length']:>7.1f}m {rstr:>8}")

    # Save JSON (without raw points to keep file small)
    if output_path is None:
        output_path = os.path.splitext(ai_path)[0] + '.json'

    export = {
        'name': result['name'],
        'total_length': result['total_length'],
        'segments': result['segments'],
    }
    with open(output_path, 'w') as f:
        json.dump(export, f, indent=2)
    print(f"\nSaved: {output_path}")

    # Optional: plot track map
    if do_plot:
        try:
            plot_track(result['points'], result['name'])
        except ImportError:
            print("Install matplotlib for plotting: pip install matplotlib")


def plot_track(points, name):
    """Plot a top-down track map from points."""
    import matplotlib.pyplot as plt

    xs = [p['x'] for p in points]
    zs = [p['z'] for p in points]

    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    ax.plot(xs, zs, 'b-', linewidth=1)
    ax.plot(xs[0], zs[0], 'go', markersize=10, label='Start')
    ax.set_aspect('equal')
    ax.set_title(f'{name} - Track Map')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{name.lower().replace(" ", "_")}_map.png', dpi=150)
    print(f"Saved track map: {name.lower().replace(' ', '_')}_map.png")
    plt.show()


if __name__ == '__main__':
    main()
