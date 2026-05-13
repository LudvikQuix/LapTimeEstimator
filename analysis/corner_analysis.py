#!/usr/bin/env python3
"""Corner analysis: classify corners in a track CSV using `tracks_config.json`.

Telemetry-free. Reads `tracks_csv/<track>/layout_*.csv` plus the repo-root
config, writes:
  - `<layout_stem>_corners.json` (schema in spec §17.4)
  - `<layout_stem>_corner_map.png`
  - `<layout_stem>_speed_vs_position.png`
"""
from __future__ import annotations

import argparse
import csv
import datetime as _dt
import json
import math
import os
import sys


def _project_root():
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, os.pardir))


def _load_config(path):
    with open(path) as f:
        cfg = json.load(f)
    ct = cfg.setdefault("corner_thresholds", {})
    ct.setdefault("hairpin_max", 60)
    ct.setdefault("tight_max", 150)
    ct.setdefault("sweeper_max", 400)
    ct.setdefault("straight_threshold_m", 500)
    cfg.setdefault("colors", {})
    return cfg


def _load_csv(path):
    """Return dict-of-lists with columns we care about. Raises if required cols missing."""
    required = {"distance_m", "radius_m"}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"Empty CSV: {path}")
        missing = required - set(reader.fieldnames)
        if missing:
            raise ValueError(f"Track CSV missing columns: {sorted(missing)}")
        rows = list(reader)

    out = {
        "distance_m": [float(r["distance_m"]) for r in rows],
        "radius_m": [float(r["radius_m"]) for r in rows],
    }
    for opt in ("x", "z", "speed_kmh", "y"):
        if opt in (reader.fieldnames or []):
            out[opt] = [float(r[opt]) if r[opt] not in (None, "") else 0.0 for r in rows]
    return out


def _smooth(vals, window=25):
    out = []
    hw = window // 2
    n = len(vals)
    for i in range(n):
        s = max(0, i - hw)
        e = min(n, i + hw + 1)
        out.append(sum(vals[s:e]) / (e - s))
    return out


def _signed_curvature(xs, zs, i):
    if i == 0 or i >= len(xs) - 1:
        return 0.0
    ax, az = xs[i - 1], zs[i - 1]
    bx, bz = xs[i], zs[i]
    cx, cz = xs[i + 1], zs[i + 1]
    cross = (bx - ax) * (cz - az) - (bz - az) * (cx - ax)
    return cross


def _classify(min_radius, thresholds):
    if min_radius < thresholds["hairpin_max"]:
        return "hairpin"
    if min_radius < thresholds["tight_max"]:
        return "tight"
    if min_radius < thresholds["sweeper_max"]:
        return "sweeper"
    return "straight"


def _detect_corners(data, thresholds):
    d = data["distance_m"]
    r = [abs(v) for v in data["radius_m"]]
    smooth_r = _smooth(r, 25)
    straight_thr = thresholds["straight_threshold_m"]

    raw = []
    in_corner = False
    start = 0
    n = len(smooth_r)
    for i in range(n):
        is_c = smooth_r[i] < straight_thr
        last = i == n - 1
        if is_c and not in_corner:
            in_corner = True
            start = i
        elif (not is_c or last) and in_corner:
            end = i if not last else n
            if end - start > 10:
                raw.append((start, end))
            in_corner = False

    # Merge adjacent corners < 1% of track length apart
    total_d = d[-1] if d else 1.0
    merged = []
    i = 0
    while i < len(raw):
        s, e = raw[i]
        while i + 1 < len(raw):
            s2, e2 = raw[i + 1]
            if d[s2] - d[e - 1] < 0.01 * total_d:
                e = e2
                i += 1
            else:
                break
        merged.append((s, e))
        i += 1

    xs = data.get("x", [0.0] * n)
    zs = data.get("z", [0.0] * n)
    speeds = data.get("speed_kmh", [0.0] * n)

    corners = []
    for cid, (s, e) in enumerate(merged, start=1):
        block_r = r[s:e]
        min_r = min(block_r)
        avg_r = sum(block_r) / len(block_r)
        ctype = _classify(min_r, thresholds)
        if ctype == "straight":
            continue  # only true corners in v1 output
        mid = (s + e) // 2
        cross = _signed_curvature(xs, zs, mid)
        direction = "left" if cross > 0 else "right"
        spd_block = speeds[s:e] if any(speeds[s:e]) else [0.0]
        corners.append({
            "id": cid,
            "type": ctype,
            "direction": direction,
            "distance_start_m": round(d[s], 2),
            "distance_end_m": round(d[min(e, len(d) - 1)], 2),
            "length_m": round(d[min(e, len(d) - 1)] - d[s], 2),
            "min_radius_m": round(min_r, 2),
            "avg_radius_m": round(avg_r, 2),
            "ai_min_speed_kmh": round(min(spd_block), 2),
            "ai_avg_speed_kmh": round(sum(spd_block) / len(spd_block), 2),
            "_start_idx": s,
            "_end_idx": e,
        })
    # Renumber after the straight-filter
    for i, c in enumerate(corners, start=1):
        c["id"] = i
    return corners


def _track_meta(csv_path):
    """Derive track + layout from <root>/tracks_csv/<track>/<csv_stem>.csv."""
    csv_path = os.path.abspath(csv_path)
    track = os.path.basename(os.path.dirname(csv_path))
    stem = os.path.splitext(os.path.basename(csv_path))[0]
    layout = stem
    if layout.startswith("layout_"):
        layout = layout[len("layout_"):]
    if layout.endswith("_ideal_line"):
        layout = layout[:-len("_ideal_line")]
    return track, layout


def _write_json(path, payload):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)


def _plot_corner_map(data, corners, out_path, colors):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping corner map")
        return
    xs = data.get("x")
    zs = data.get("z")
    if xs is None or zs is None:
        print("CSV has no x/z columns; skipping corner map")
        return
    r = [abs(v) for v in data["radius_m"]]
    point_colors = [_severity_color(rv, colors) for rv in r]

    fig, ax = plt.subplots(figsize=(14, 9))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#1a1a2e")
    for i in range(len(xs) - 1):
        ax.plot([xs[i], xs[i + 1]], [zs[i], zs[i + 1]],
                color=point_colors[i], linewidth=3, solid_capstyle="round")
    ax.plot(xs[0], zs[0], "ws", markersize=10)
    for c in corners:
        s, e = c["_start_idx"], c["_end_idx"]
        mid = (s + e) // 2
        ax.annotate(
            f"T{c['id']}", xy=(xs[mid], zs[mid]),
            xytext=(xs[mid] + 20, zs[mid] + 20),
            fontsize=8, fontweight="bold", color="white",
            arrowprops=dict(arrowstyle="-", color="white", alpha=0.4, lw=0.5),
            bbox=dict(boxstyle="round,pad=0.2",
                      facecolor=colors.get(c["type"], "#888"),
                      edgecolor="white", alpha=0.85, linewidth=0.5),
        )
    ax.set_aspect("equal")
    ax.set_title("Corner Severity Map", color="white", fontweight="bold")
    ax.tick_params(colors="white")
    for sp in ax.spines.values():
        sp.set_color("white")
        sp.set_alpha(0.3)
    ax.grid(True, alpha=0.1, color="white")
    fig.tight_layout()
    fig.savefig(out_path, dpi=160, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"Saved: {out_path}")


def _plot_speed_position(data, corners, out_path, colors):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping speed-vs-position plot")
        return
    speeds = data.get("speed_kmh")
    d = data["distance_m"]
    if not speeds or not any(speeds):
        print("CSV has no speed_kmh data; skipping speed-vs-position plot")
        return
    fig, ax = plt.subplots(figsize=(20, 5))
    fig.patch.set_facecolor("#181B2B")
    ax.set_facecolor("#181B2B")
    for c in corners:
        s, e = c["_start_idx"], c["_end_idx"]
        ax.axvspan(d[s], d[min(e, len(d) - 1)],
                   color=colors.get(c["type"], "#888"), alpha=0.2)
        mid = (s + e) // 2
        ax.text(d[mid], 0.97, f"T{c['id']}",
                transform=ax.get_xaxis_transform(),
                ha="center", va="top", fontsize=8,
                fontweight="bold", color="white", alpha=0.7)
    ax.plot(d, speeds, color="#4A9EF5", linewidth=1.2)
    ax.set_xlabel("Distance [m]", color="white")
    ax.set_ylabel("Speed [km/h]", color="white")
    ax.tick_params(colors="white")
    for sp in ax.spines.values():
        sp.set_visible(False)
    ax.grid(True, axis="both", color="#2a2e45", linewidth=0.5, alpha=0.6)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"Saved: {out_path}")


def _severity_color(radius, colors):
    if radius < 60:
        return colors.get("hairpin", "#f87171")
    if radius < 150:
        return colors.get("tight", "#fb923c")
    if radius < 400:
        return colors.get("sweeper", "#fbbf24")
    return colors.get("straight", "#34d399")


def _print_summary(track, layout, corners):
    print("=" * 84)
    print(f"  {track.upper()} {layout.upper()} - CORNER ANALYSIS  ({len(corners)} corners)")
    print("=" * 84)
    print(f"{'Turn':>5}  {'Type':<9} {'Dir':<5} {'Start m':>9} {'End m':>9}  "
          f"{'Min R':>7} {'Avg R':>7}  {'Min spd':>8} {'Avg spd':>8}")
    print("-" * 84)
    for c in corners:
        print(
            f"  T{c['id']:<3}  {c['type'].upper():<9} {c['direction']:<5} "
            f"{c['distance_start_m']:9.2f} {c['distance_end_m']:9.2f}  "
            f"{c['min_radius_m']:7.1f} {c['avg_radius_m']:7.1f}  "
            f"{c['ai_min_speed_kmh']:8.1f} {c['ai_avg_speed_kmh']:8.1f}"
        )
    print("=" * 84)


def main():
    parser = argparse.ArgumentParser(description="Corner analysis (telemetry-free)")
    parser.add_argument("track_csv", nargs="?", default=None)
    parser.add_argument("--config", default=None,
                        help="Path to tracks_config.json (default: repo root)")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--no-json", action="store_true")
    args = parser.parse_args()

    root = _project_root()
    cfg_path = args.config or os.path.join(root, "tracks_config.json")
    cfg = _load_config(cfg_path)

    csv_path = args.track_csv or cfg.get("default_track")
    if not csv_path:
        print("ERROR: no track CSV given and no default_track in config.",
              file=sys.stderr)
        sys.exit(2)
    if not os.path.isabs(csv_path):
        csv_path_abs = os.path.join(root, csv_path)
        csv_path = csv_path_abs if os.path.isfile(csv_path_abs) else csv_path
    if not os.path.isfile(csv_path):
        print(f"ERROR: track CSV not found: {csv_path}", file=sys.stderr)
        sys.exit(2)

    data = _load_csv(csv_path)
    corners = _detect_corners(data, cfg["corner_thresholds"])
    track, layout = _track_meta(csv_path)

    payload = {
        "track": track,
        "layout": layout,
        "source_csv": os.path.relpath(csv_path, root).replace("\\", "/"),
        "total_length_m": round(data["distance_m"][-1], 2),
        "config": {
            "hairpin_max_m": cfg["corner_thresholds"]["hairpin_max"],
            "tight_max_m": cfg["corner_thresholds"]["tight_max"],
            "sweeper_max_m": cfg["corner_thresholds"]["sweeper_max"],
            "straight_threshold_m": cfg["corner_thresholds"]["straight_threshold_m"],
        },
        "generated_at": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "version": "1",
        "corners": [
            {k: v for k, v in c.items() if not k.startswith("_")}
            for c in corners
        ],
    }

    stem = os.path.splitext(csv_path)[0]
    if not args.no_json:
        out_json = f"{stem}_corners.json"
        _write_json(out_json, payload)
        print(f"Wrote: {out_json}")

    _print_summary(track, layout, corners)

    if not args.no_plot:
        _plot_corner_map(data, corners, f"{stem}_corner_map.png", cfg.get("colors", {}))
        _plot_speed_position(data, corners, f"{stem}_speed_vs_position.png",
                             cfg.get("colors", {}))


if __name__ == "__main__":
    main()
