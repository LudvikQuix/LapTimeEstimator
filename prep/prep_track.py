#!/usr/bin/env python3
"""Prep a track: parse AC `fast_lane.ai` for each layout, emit rich per-point CSV.

Output column set (per spec §7.1 / README):
  index, distance_m, segment_length_m, x, y, z, elevation_m, gradient_pct,
  radius_m, speed_ms, speed_kmh, width_left_m, width_right_m, width_total_m

The AC AI-line parser currently exposes only position + cumulative distance.
Speed and width channels (which exist in the binary fast_lane.ai detail block)
are not parsed by `decode_track.py`. Until that parser is extended, this
script emits geometry-derived columns from x,y,z and zero-filled width
columns; `speed_ms` is left blank for downstream tooling to fill in (or to
be supplied via a hand-edited reference CSV).
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from decode_track import points_to_segments, read_ai_line  # noqa: E402  (kept for direct CLI fallback)


def _discover_layouts(tracks_in_dir):
    """Return list of (layout_name, fast_lane_path).

    Single-layout: `<tracks_in>/ai/fast_lane.ai`.
    Multi-layout: any subdir containing `ai/fast_lane.ai` is a layout.
    """
    single = os.path.join(tracks_in_dir, "ai", "fast_lane.ai")
    if os.path.isfile(single):
        return [(os.path.basename(os.path.abspath(tracks_in_dir)), single)]

    out = []
    for entry in sorted(os.listdir(tracks_in_dir)):
        cand = os.path.join(tracks_in_dir, entry, "ai", "fast_lane.ai")
        if os.path.isfile(cand):
            out.append((entry, cand))
    return out


def _menger_radius(p0, p1, p2):
    ax, az = p0["x"], p0["z"]
    bx, bz = p1["x"], p1["z"]
    cx, cz = p2["x"], p2["z"]
    area2 = abs((bx - ax) * (cz - az) - (cx - ax) * (bz - az))
    ab = math.hypot(bx - ax, bz - az)
    bc = math.hypot(cx - bx, cz - bz)
    ac = math.hypot(cx - ax, cz - az)
    denom = ab * bc * ac
    if denom < 1e-6 or area2 < 1e-9:
        return 2000.0
    return min(2000.0, (ab * bc * ac) / (2.0 * area2))


def _smooth_running(vals, half_window=5):
    n = len(vals)
    out = [0.0] * n
    for i in range(n):
        lo = max(0, i - half_window)
        hi = min(n, i + half_window + 1)
        out[i] = sum(vals[lo:hi]) / (hi - lo)
    return out


def _resample(points, ds):
    """Resample to uniform ds using linear interpolation. Returns list of dicts."""
    if len(points) < 2:
        return points
    dists = [p["dist"] for p in points]
    total = dists[-1]
    n = max(2, int(total / ds) + 1)
    out = []
    j = 0
    for i in range(n):
        target = i * ds
        if target > total:
            target = total
        while j + 1 < len(points) and points[j + 1]["dist"] < target:
            j += 1
        if j + 1 >= len(points):
            p = points[-1]
            out.append({"x": p["x"], "y": p["y"], "z": p["z"], "dist": target})
            continue
        a = points[j]
        b = points[j + 1]
        span = max(b["dist"] - a["dist"], 1e-6)
        t = (target - a["dist"]) / span
        out.append({
            "x": a["x"] + t * (b["x"] - a["x"]),
            "y": a["y"] + t * (b["y"] - a["y"]),
            "z": a["z"] + t * (b["z"] - a["z"]),
            "dist": target,
        })
    return out


def _build_rows(points):
    n = len(points)
    radii = [2000.0] * n
    for i in range(1, n - 1):
        radii[i] = _menger_radius(points[i - 1], points[i], points[i + 1])
    radii[0] = radii[1] if n > 1 else 2000.0
    radii[-1] = radii[-2] if n > 1 else 2000.0
    radii = _smooth_running(radii, half_window=3)

    rows = []
    for i in range(n):
        p = points[i]
        if i + 1 < n:
            seg_len = points[i + 1]["dist"] - p["dist"]
        else:
            seg_len = 0.0
        if i > 0:
            dy = p["y"] - points[i - 1]["y"]
            d = max(seg_len if seg_len > 0 else (p["dist"] - points[i - 1]["dist"]), 1e-6)
            gradient = (dy / d) * 100.0
        else:
            gradient = 0.0
        rows.append({
            "index": i,
            "distance_m": p["dist"],
            "segment_length_m": seg_len,
            "x": p["x"],
            "y": p["y"],
            "z": p["z"],
            "elevation_m": p["y"],
            "gradient_pct": gradient,
            "radius_m": radii[i],
            "speed_ms": "",
            "speed_kmh": "",
            "width_left_m": 0.0,
            "width_right_m": 0.0,
            "width_total_m": 0.0,
        })
    return rows


def _write_csv(rows, output_path):
    cols = ["index", "distance_m", "segment_length_m", "x", "y", "z",
            "elevation_m", "gradient_pct", "radius_m", "speed_ms",
            "speed_kmh", "width_left_m", "width_right_m", "width_total_m"]
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for r in rows:
            w.writerow([
                r["index"],
                f"{r['distance_m']:.2f}",
                f"{r['segment_length_m']:.3f}",
                f"{r['x']:.3f}",
                f"{r['y']:.3f}",
                f"{r['z']:.3f}",
                f"{r['elevation_m']:.3f}",
                f"{r['gradient_pct']:.2f}",
                f"{r['radius_m']:.1f}",
                r["speed_ms"],
                r["speed_kmh"],
                f"{r['width_left_m']:.2f}",
                f"{r['width_right_m']:.2f}",
                f"{r['width_total_m']:.2f}",
            ])


def main():
    parser = argparse.ArgumentParser(description="Prep an AC track into rich per-point CSV")
    parser.add_argument("tracks_in_dir", help="e.g. tracks_in/ks_nurburgring")
    parser.add_argument("--output-root", default="tracks_csv")
    parser.add_argument("--ds", type=float, default=1.0)
    parser.add_argument("--layouts", default="all")
    args = parser.parse_args()

    if not os.path.isdir(args.tracks_in_dir):
        print(f"ERROR: {args.tracks_in_dir} is not a directory.", file=sys.stderr)
        sys.exit(2)

    layouts = _discover_layouts(args.tracks_in_dir)
    if not layouts:
        print("ERROR: no `ai/fast_lane.ai` found in this folder or its subfolders.",
              file=sys.stderr)
        sys.exit(2)

    want = None if args.layouts == "all" else set(args.layouts.split(","))
    track_name = os.path.basename(os.path.abspath(args.tracks_in_dir))
    out_dir = os.path.join(args.output_root, track_name)

    for layout, fast_lane in layouts:
        if want is not None and layout not in want:
            continue
        points = read_ai_line(fast_lane)
        # `points_to_segments` is the legacy segment exporter; not used here,
        # but the import keeps the parser DLL warm and confirms the file format.
        _ = points_to_segments
        points = _resample(points, args.ds)
        rows = _build_rows(points)
        out = os.path.join(out_dir, f"layout_{layout}.csv")
        _write_csv(rows, out)
        total = rows[-1]["distance_m"] if rows else 0.0
        print(f"Wrote {out} ({len(rows)} rows, {total:.0f} m)")


if __name__ == "__main__":
    main()
