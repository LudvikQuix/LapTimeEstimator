import csv
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.collections as mcollections
import numpy as np

# ---------------------------------------------------------------------------
# Load merged data
# ---------------------------------------------------------------------------
with open("Curve_Analysis/merged_lap2_track.csv", "r") as f:
    reader = csv.reader(f)
    header = next(reader)
    rows = [row for row in reader if row]

col = {name: idx for idx, name in enumerate(header)}

radii = [abs(float(r[col["radius_m"]])) for r in rows]
xs = [-float(r[col["x"]]) for r in rows]
zs = [float(r[col["z"]]) for r in rows]
nds = [float(r[col["normalizedCarPosition"]]) for r in rows]
speeds = [float(r[col["speedKmh"]]) for r in rows]
elevations = [float(r[col["elevation_m"]]) for r in rows]

# ---------------------------------------------------------------------------
# Classify every point by turn severity
# ---------------------------------------------------------------------------
# Thresholds on corner radius (metres)
#   red    : R < 60    – hairpins / very tight
#   orange : 60 <= R < 150  – medium-tight
#   yellow : 150 <= R < 400 – sweepers / gentle curves
#   green  : R >= 400       – straights / very gentle
def severity_color(r):
    if r < 60:
        return "red"
    elif r < 150:
        return "orange"
    elif r < 400:
        return "#FFD700"  # gold-yellow (visible on white bg)
    else:
        return "green"

colors = [severity_color(r) for r in radii]

# ---------------------------------------------------------------------------
# Smooth radius for corner grouping
# ---------------------------------------------------------------------------
def smooth(vals, window=25):
    out = []
    hw = window // 2
    for i in range(len(vals)):
        s, e = max(0, i - hw), min(len(vals), i + hw + 1)
        out.append(sum(vals[s:e]) / (e - s))
    return out

smooth_r = smooth(radii, 25)

STRAIGHT_THRESHOLD = 500

corners = []
in_corner = False
start = 0
for i in range(len(smooth_r)):
    if smooth_r[i] < STRAIGHT_THRESHOLD and not in_corner:
        in_corner = True
        start = i
    elif (smooth_r[i] >= STRAIGHT_THRESHOLD or i == len(smooth_r) - 1) and in_corner:
        in_corner = False
        if i - start > 10:
            min_r = min(radii[start:i])
            avg_r = sum(radii[start:i]) / (i - start)
            mid = (start + i) // 2
            corners.append(
                {
                    "id": len(corners) + 1,
                    "start": start,
                    "end": i,
                    "min_radius": min_r,
                    "avg_radius": avg_r,
                    "nd_start": nds[start],
                    "nd_end": nds[i - 1],
                    "x": xs[mid],
                    "z": zs[mid],
                    "severity": severity_color(min_r),
                    "min_speed": min(speeds[start:i]),
                    "avg_speed": sum(speeds[start:i]) / (i - start),
                }
            )

# Merge corners 4+5 (very close together – they form one complex)
merged_corners = []
i = 0
while i < len(corners):
    c = dict(corners[i])
    # merge if next corner starts within 3% of this one ending
    while i + 1 < len(corners) and corners[i + 1]["nd_start"] - c["nd_end"] < 0.01:
        nxt = corners[i + 1]
        c["end"] = nxt["end"]
        c["nd_end"] = nxt["nd_end"]
        c["min_radius"] = min(c["min_radius"], nxt["min_radius"])
        combined_len = c["end"] - c["start"]
        c["avg_radius"] = sum(radii[c["start"] : c["end"]]) / combined_len
        c["min_speed"] = min(c["min_speed"], nxt["min_speed"])
        c["avg_speed"] = sum(speeds[c["start"] : c["end"]]) / combined_len
        c["severity"] = severity_color(c["min_radius"])
        mid = (c["start"] + c["end"]) // 2
        c["x"] = xs[mid]
        c["z"] = zs[mid]
        i += 1
    merged_corners.append(c)
    i += 1

# Re-number
for idx, c in enumerate(merged_corners):
    c["id"] = idx + 1

# ---------------------------------------------------------------------------
# Print corner analysis table
# ---------------------------------------------------------------------------
severity_label = {
    "red": "HAIRPIN",
    "orange": "TIGHT",
    "#FFD700": "SWEEPER",
    "green": "FAST",
}

print("=" * 90)
print(f"  NURBURGRING SPRINT – CORNER ANALYSIS  ({len(merged_corners)} significant corners)")
print("=" * 90)
print(
    f"{'Turn':>5}  {'Type':<9} {'ND start':>9} {'ND end':>9}  "
    f"{'Min R(m)':>9} {'Avg R(m)':>9}  {'Min spd':>8} {'Avg spd':>8}"
)
print("-" * 90)
for c in merged_corners:
    print(
        f"  T{c['id']:<3}  {severity_label[c['severity']]:<9} "
        f"{c['nd_start']:9.4f} {c['nd_end']:9.4f}  "
        f"{c['min_radius']:9.1f} {c['avg_radius']:9.1f}  "
        f"{c['min_speed']:7.1f}  {c['avg_speed']:7.1f}"
    )
print("=" * 90)

# ---------------------------------------------------------------------------
# Plot 2D track map
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(16, 10))
fig.patch.set_facecolor("#1a1a2e")
ax.set_facecolor("#1a1a2e")

# Draw track as colored line segments
for i in range(len(xs) - 1):
    ax.plot(
        [xs[i], xs[i + 1]],
        [zs[i], zs[i + 1]],
        color=colors[i],
        linewidth=3.5,
        solid_capstyle="round",
    )

# Add thin white border for track edges
for i in range(len(xs) - 1):
    ax.plot(
        [xs[i], xs[i + 1]],
        [zs[i], zs[i + 1]],
        color="white",
        linewidth=5.5,
        solid_capstyle="round",
        alpha=0.08,
    )

# Mark start/finish
ax.plot(xs[0], zs[0], "ws", markersize=12, label="Start / Finish", zorder=5)

# Label corners
for c in merged_corners:
    mid = (c["start"] + c["end"]) // 2
    tx, tz = xs[mid], zs[mid]

    # Offset label slightly outward from track centre
    if mid > 5 and mid < len(xs) - 5:
        dx = xs[mid + 5] - xs[mid - 5]
        dz = zs[mid + 5] - zs[mid - 5]
        norm = math.sqrt(dx * dx + dz * dz) + 1e-9
        ox, oz = -dz / norm * 30, dx / norm * 30
    else:
        ox, oz = 15, 15

    ax.annotate(
        f"T{c['id']}",
        xy=(tx, tz),
        xytext=(tx + ox, tz + oz),
        fontsize=9,
        fontweight="bold",
        color="white",
        ha="center",
        va="center",
        arrowprops=dict(arrowstyle="-", color="white", alpha=0.4, lw=0.8),
        bbox=dict(
            boxstyle="round,pad=0.25",
            facecolor=c["severity"],
            edgecolor="white",
            alpha=0.85,
            linewidth=0.5,
        ),
    )

# Legend
from matplotlib.lines import Line2D

legend_elements = [
    Line2D([0], [0], color="red", lw=4, label="Hairpin  (R < 60 m)"),
    Line2D([0], [0], color="orange", lw=4, label="Tight    (60–150 m)"),
    Line2D([0], [0], color="#FFD700", lw=4, label="Sweeper  (150–400 m)"),
    Line2D([0], [0], color="green", lw=4, label="Straight (R ≥ 400 m)"),
    Line2D([0], [0], marker="s", color="w", label="Start / Finish", markersize=8, linestyle="None"),
]
ax.legend(
    handles=legend_elements,
    loc="center left",
    bbox_to_anchor=(1.02, 0.5),
    fontsize=9,
    facecolor="#2a2a4a",
    edgecolor="white",
    labelcolor="white",
)

ax.set_title(
    "Nurburgring Sprint - Corner Severity Map",
    fontsize=16,
    fontweight="bold",
    color="white",
    pad=15,
)
ax.set_xlabel("X (m)", color="white", fontsize=10)
ax.set_ylabel("Z (m)", color="white", fontsize=10)
ax.tick_params(colors="white")
for spine in ax.spines.values():
    spine.set_color("white")
    spine.set_alpha(0.3)
ax.set_aspect("equal")
ax.grid(True, alpha=0.1, color="white")

fig.subplots_adjust(right=0.78)
plt.savefig("Curve_Analysis/track_corner_map.png", dpi=200, facecolor=fig.get_facecolor())
plt.close(fig)
print("\nSaved: Curve_Analysis/track_corner_map.png")

# ---------------------------------------------------------------------------
# Plot Speed vs Track Position with corner overlays
# ---------------------------------------------------------------------------
BG = "#181B2B"
GRID_COLOR = "#2a2e45"

fig2, ax2 = plt.subplots(figsize=(22, 5))
fig2.patch.set_facecolor(BG)
ax2.set_facecolor(BG)

# Corner severity overlays
severity_fill = {
    "red": ("red", 0.18),
    "orange": ("orange", 0.15),
    "#FFD700": ("#FFD700", 0.12),
    "green": ("green", 0.10),
}

for c in merged_corners:
    fc, alpha = severity_fill[c["severity"]]
    ax2.axvspan(c["nd_start"], c["nd_end"], color=fc, alpha=alpha, zorder=1)
    # Label at top
    mid_nd = (c["nd_start"] + c["nd_end"]) / 2
    ax2.text(
        mid_nd,
        0.97,
        f"T{c['id']}",
        transform=ax2.get_xaxis_transform(),
        ha="center",
        va="top",
        fontsize=8,
        fontweight="bold",
        color="white",
        alpha=0.7,
    )

# Speed trace
ax2.plot(nds, speeds, color="#4A9EF5", linewidth=1.2, zorder=3, label="Lap 2")

# Axes styling
ax2.set_xlabel("Track Position [-]", color="white", fontsize=11)
ax2.set_ylabel("Speed [km/h]", color="white", fontsize=11)
ax2.set_xlim(0, 1)
ax2.set_ylim(0, max(speeds) * 1.08)
ax2.tick_params(colors="white", labelsize=10)
for spine in ax2.spines.values():
    spine.set_visible(False)
ax2.grid(True, axis="both", color=GRID_COLOR, linewidth=0.5, alpha=0.6)

# Legend (top-right, matching reference style)
from matplotlib.patches import Patch

leg_handles = [
    Line2D([0], [0], color="#4A9EF5", lw=1.5, label="Lap 2"),
    Patch(facecolor="red", alpha=0.3, label="Hairpin (R<60m)"),
    Patch(facecolor="orange", alpha=0.3, label="Tight (60-150m)"),
    Patch(facecolor="#FFD700", alpha=0.3, label="Sweeper (150-400m)"),
]
ax2.legend(
    handles=leg_handles,
    loc="center left",
    bbox_to_anchor=(1.02, 0.5),
    fontsize=9,
    facecolor="#252842",
    edgecolor="#3a3e5c",
    labelcolor="white",
    framealpha=0.9,
)
fig2.subplots_adjust(right=0.85)

plt.tight_layout()
plt.savefig(
    "Curve_Analysis/speed_vs_position.png",
    dpi=200,
    facecolor=fig2.get_facecolor(),
)
plt.close(fig2)
print("Saved: Curve_Analysis/speed_vs_position.png")

# ---------------------------------------------------------------------------
# Export DB-ready CSVs
# ---------------------------------------------------------------------------
import base64

TRACK_ID = "ks_nurburgring_sprint_a"
TRACK_NAME = "Nurburgring Sprint"

# --- 1. track_points.csv ---------------------------------------------------
# Based on the raw track layout (layout_sprint_a.csv), NOT the merged lap data.
# This is pure track geometry — lap data lives in a separate DB.
with open("tracks_csv/ks_nurburgring/layout_sprint_a.csv", "r") as f:
    reader = csv.reader(f)
    src_header = next(reader)
    src_rows = [row for row in reader if row]

# Build corner lookup using normalizedDistance from track data
nd_idx = src_header.index("normalizedDistance")
track_nds = [float(r[nd_idx]) for r in src_rows]

# Map each track point to a corner based on normalizedDistance range
def find_corner(nd_val):
    for c in merged_corners:
        if c["nd_start"] <= nd_val <= c["nd_end"]:
            return c
    return None

tp_header = ["track_id"] + src_header + [
    "corner_id",
    "corner_name",
    "corner_type",
    "corner_color",
    "is_corner_start",
    "is_corner_end",
]

tp_rows = []
for i, row in enumerate(src_rows):
    nd_val = track_nds[i]
    c = find_corner(nd_val)
    if c:
        cid = f"T{c['id']}"
        cname = f"Turn {c['id']}"
        ctype = severity_label[c["severity"]]
        ccolor = c["severity"]
        # Mark start/end: first/last track point inside this corner range
        is_start = "1" if i == 0 or find_corner(track_nds[i - 1]) != c else "0"
        is_end = "1" if i == len(src_rows) - 1 or find_corner(track_nds[min(i + 1, len(src_rows) - 1)]) != c else "0"
    else:
        cid = ""
        cname = ""
        ctype = "STRAIGHT"
        ccolor = "green"
        is_start = "0"
        is_end = "0"
    tp_rows.append([TRACK_ID] + row + [cid, cname, ctype, ccolor, is_start, is_end])

with open("Curve_Analysis/track_points.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(tp_header)
    writer.writerows(tp_rows)

print(f"\nSaved: Curve_Analysis/track_points.csv  ({len(tp_rows)} rows, {len(tp_header)} cols)")

# --- 2. track_corners.csv --------------------------------------------------
# One row per corner — queryable by track_id + corner_id.
tc_header = [
    "track_id",
    "corner_id",
    "corner_name",
    "corner_type",
    "corner_color",
    "nd_start",
    "nd_end",
    "min_radius_m",
    "avg_radius_m",
    "min_speed_kmh",
    "avg_speed_kmh",
]

tc_rows = []
for c in merged_corners:
    tc_rows.append([
        TRACK_ID,
        f"T{c['id']}",
        f"Turn {c['id']}",
        severity_label[c["severity"]],
        c["severity"],
        f"{c['nd_start']:.6f}",
        f"{c['nd_end']:.6f}",
        f"{c['min_radius']:.1f}",
        f"{c['avg_radius']:.1f}",
        f"{c['min_speed']:.1f}",
        f"{c['avg_speed']:.1f}",
    ])

with open("Curve_Analysis/track_corners.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(tc_header)
    writer.writerows(tc_rows)

print(f"Saved: Curve_Analysis/track_corners.csv  ({len(tc_rows)} corners)")

# --- 3. track_meta.csv -----------------------------------------------------
# Track-level metadata + images stored as base64.
# One row per track — images as base64 columns.

def file_to_base64(path):
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode("ascii")

tm_header = [
    "track_id",
    "track_name",
    "layout",
    "length_m",
    "num_corners",
    "severity_legend_json",
    "track_map_png_base64",
    "speed_chart_png_base64",
]

import json

legend = [
    {"type": "HAIRPIN", "color": "red", "radius_max_m": 60, "label": "Hairpin (R < 60 m)"},
    {"type": "TIGHT", "color": "orange", "radius_min_m": 60, "radius_max_m": 150, "label": "Tight (60-150 m)"},
    {"type": "SWEEPER", "color": "#FFD700", "radius_min_m": 150, "radius_max_m": 400, "label": "Sweeper (150-400 m)"},
    {"type": "STRAIGHT", "color": "green", "radius_min_m": 400, "label": "Straight (R >= 400 m)"},
]

tm_row = [
    TRACK_ID,
    TRACK_NAME,
    "sprint_a",
    "3565.0",
    str(len(merged_corners)),
    json.dumps(legend),
    file_to_base64("Curve_Analysis/track_corner_map.png"),
    file_to_base64("Curve_Analysis/speed_vs_position.png"),
]

with open("Curve_Analysis/track_meta.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(tm_header)
    writer.writerow(tm_row)

print(f"Saved: Curve_Analysis/track_meta.csv  (images as base64)")
print(f"  track_map_png_base64: {len(tm_row[-2])} chars")
print(f"  speed_chart_png_base64: {len(tm_row[-1])} chars")
