"""Rerun-based replay of a real (ghost) lap against a simulated lap.

Run from the repo root:

    python -m viz.rerun_replay \
        --track tracks_csv/ks_nurburgring/layout_sprint_a.csv \
        --ghost tracks_csv/ks_nurburgring/layout_sprint_a__tomas_full_sim_telemetry.csv \
        --sim   tracks_csv/ks_nurburgring/layout_sprint_a__tomas_full_sim_trace_slip.csv

The Rerun desktop viewer opens (``rr.spawn()``); both cars share a single
``sim_time`` timeline starting at zero, so the scrubber moves them
together. Scalar telemetry (speed, gas, brake when present) is logged on
the same timeline and appears as synchronised plots in side panes.

The 2D world is the AC ``(x, z)`` ground plane (``y`` is vertical
elevation in AC and is ignored for the top-down view).

A 3D scene is also logged by default: track + cars in 3D plus a
chase camera attached behind the sim car (see ``viz.scene3d``). Pass
``--no-3d`` to skip the 3D logging.

See ``docs/architecture-rerun-replay.md`` for design notes.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import rerun as rr

from viz.loader import LapTrace, Track, load_ghost_trace, load_sim_trace, load_track
from viz.scene3d import (
    build_blueprint,
    log_car_pose_stream,
    log_chase_camera,
    log_environment_3d,
    log_track_3d,
    log_trajectory_3d,
)

# Fixed colours per car role. RGBA, 0-255.
GHOST_COLOR = (220, 220, 220, 255)   # light grey
SIM_COLOR = (235, 60, 60, 255)       # red
CENTERLINE_COLOR = (90, 90, 90, 255)
EDGE_COLOR = (160, 160, 160, 255)

# Visibility radii (world units, metres). At the overview zoom level
# where a ~1.7 km circuit fits in roughly 600-1000 CSS pixels, anything
# below ~1 m is sub-pixel and disappears. See round-1 complaints #4
# (trajectories invisible) and #6 (track outline barely perceptible).
CAR_RADIUS_M = 4.0          # car dot in the 2D view
TRACK_CENTERLINE_RADIUS_M = 1.5
TRACK_EDGE_RADIUS_M = 2.0
TRAJECTORY_RADIUS_M = 1.0

# Chequered S/F flag in the 2D top-down view. Mirrors the 3D grid in
# ``viz.scene3d`` (``SF_FLAG_N_LAT`` x ``SF_FLAG_N_LON`` squares).
# 2D box orientation note: ``Boxes2D`` in Rerun 0.32 has no rotation
# field, so each square is axis-aligned in the world (x, z) plane. The
# lateral axis is world X and the longitudinal axis is world Z. At
# Nurburgring Sprint / GP the start-line tangent runs nearly along Z
# (psi ~ 90 deg) and the lateral axis is essentially world X, so the
# axis-aligned approximation reproduces the real layout to within ~0.1
# degree; on a hypothetical track with the S/F line rotated 45 deg the
# squares would look misaligned and a future change should switch this
# function to ``LineStrips2D`` polygons of rotated quads (or upgrade if
# Rerun adds rotation to ``Boxes2D``).
SF_FLAG_N_LAT_2D = 8
SF_FLAG_N_LON_2D = 4
SF_FLAG_LON_SIZE_M_2D = 1.25
SF_FLAG_COLOR_BLACK_2D = (15, 15, 15, 255)
SF_FLAG_COLOR_WHITE_2D = (245, 245, 245, 255)

WORLD_ROOT = "world"
TRACK_ROOT = "world/track"
TRACK_ROOT_3D = "world/track3d"
ENV_ROOT = "world/env"

# 2D environment overlay. The 2D top-down view treats world (x, z)
# directly as scene coordinates; we log a single large green Boxes2D
# rectangle covering the track bounding box padded by ENV_GRASS_PAD_M_2D
# so it reads as grass behind the trajectory / lines.
#
# We deliberately do NOT render the 2D track surface here. Rerun 0.32's
# Spatial2DView doesn't render Mesh3D, and a per-segment Boxes2D ribbon
# would mean thousands of axis-aligned boxes that don't line up with
# the actual track tangent — the result looks worse than just leaving
# the 2D pane line-only. The grass alone gives enough "field vs road"
# contrast against the existing grey centerline+edge polylines for the
# 2D pane to read as a real circuit.
ENV_GRASS_COLOR_2D = (60, 130, 60, 255)
ENV_GRASS_PAD_M_2D = 200.0


# ----------------------------- environment overlay ------------------------ #

def _log_environment_2d(track: Track, entity: str) -> None:
    """Log a single large green Boxes2D as the 2D grass backdrop.

    Centred on the track bbox midpoint, padded by ``ENV_GRASS_PAD_M_2D``
    on all sides. Because every other 2D entity (centreline, edges,
    trajectories, dots, S/F flag) is logged AFTER this one, Rerun 0.32
    paints them on top so nothing gets occluded. The 2D top-down pane
    treats world (x, z) directly as the (x, y) scene plane so the
    Boxes2D rectangle in world units lines up with everything else.
    """
    x_min = float(track.x.min()) - ENV_GRASS_PAD_M_2D
    x_max = float(track.x.max()) + ENV_GRASS_PAD_M_2D
    z_min = float(track.z.min()) - ENV_GRASS_PAD_M_2D
    z_max = float(track.z.max()) + ENV_GRASS_PAD_M_2D
    cx = 0.5 * (x_min + x_max)
    cz = 0.5 * (z_min + z_max)
    half_x = 0.5 * (x_max - x_min)
    half_z = 0.5 * (z_max - z_min)
    rr.log(
        entity,
        rr.Boxes2D(
            centers=[(cx, cz)],
            half_sizes=[(half_x, half_z)],
            colors=[ENV_GRASS_COLOR_2D],
            # Hairline outline; the fill is what we want visible.
            radii=[0.5],
        ),
        static=True,
    )


# ----------------------------- track logging ------------------------------- #

def _log_chequered_flag_2d(track: Track, entity: str) -> None:
    """Log an axis-aligned 2D chequered S/F flag at distance=0.

    8 x 4 grid of small ``Boxes2D`` rectangles alternating black/white,
    centred at ``track.centerline_xz[0]`` and spanning the full corridor
    width (``left_xz[0]`` to ``right_xz[0]``) laterally. The 2D top-down
    pane treats world (x, z) directly as scene coordinates, so each
    square's centre is in world (x, z) and its half-extents are in
    world metres along (x, z) axes.

    Because Rerun 0.32's ``Boxes2D`` has no rotation field, the boxes
    are axis-aligned in world (x, z). At Nurburgring Sprint / GP the
    start tangent runs nearly along world Z, so axis-aligned squares
    line up with the real start-line to within ~0.1 deg — see the
    SF_FLAG_* constant block above for the limitation discussion.

    ``Boxes2D`` draws outlines only (no ``fill_mode`` in this archetype
    in Rerun 0.32). To get the visual effect of filled squares we set
    the stroke ``radii`` (line width, in world metres) to half the
    larger box dimension; that makes the stroke cover the box's
    interior. Adjacent boxes of opposite colour cleanly overpaint each
    other along the shared edge because Rerun rasterises later-logged
    archetypes on top, but here all boxes ship in one batch so the
    order is implementation-defined — radii is sized so the outline
    fills its OWN box without bleeding past, by clamping the stroke to
    ``min(half_lat, half_lon)``.
    """
    n_lat = SF_FLAG_N_LAT_2D
    n_lon = SF_FLAG_N_LON_2D

    cx0 = float(track.x[0])
    cz0 = float(track.z[0])
    left = track.left_xz[0].astype(np.float64)
    right = track.right_xz[0].astype(np.float64)
    lat_vec = right - left
    lat_len = float(np.hypot(lat_vec[0], lat_vec[1]))
    if lat_len < 1e-3:
        return  # degenerate; nothing to draw
    # Lateral unit vector (left -> right) in (x, z).
    lat_hat = lat_vec / lat_len
    # Longitudinal unit vector (track tangent, in xz). Use the first
    # non-degenerate forward step from the centreline.
    tx = float(track.x[1] - track.x[0])
    tz = float(track.z[1] - track.z[0])
    tan_len = float(np.hypot(tx, tz))
    if tan_len < 1e-6:
        # Fall back to a tangent perpendicular to the lateral axis (rotate
        # +90 deg in the (x, z) plane).
        tan_hat = np.array([-lat_hat[1], lat_hat[0]])
    else:
        tan_hat = np.array([tx, tz]) / tan_len

    square_lat = lat_len / n_lat
    square_lon = SF_FLAG_LON_SIZE_M_2D
    # Axis-aligned half-extents: since the squares are NOT rotated, we
    # pick the projection of the lateral square edge onto world X for
    # the X half-size, and the projection of the longitudinal edge onto
    # world Z for the Z half-size. With tan_hat ~ (0, 1) at Nurburgring
    # this becomes (square_lat/2, square_lon/2) which is exactly the
    # intended square. For tilted tracks it gives a parallelogram-ish
    # approximation — acceptable for an orientation marker.
    half_x = abs(lat_hat[0]) * square_lat * 0.5 + abs(tan_hat[0]) * square_lon * 0.5
    half_z = abs(lat_hat[1]) * square_lat * 0.5 + abs(tan_hat[1]) * square_lon * 0.5
    half_size = (float(half_x), float(half_z))
    # Stroke width: clamp to half of the smaller box dimension so the
    # stroke fills the box from edge to edge without spilling into the
    # neighbour cell. (Boxes2D radii is line width; the outline grows
    # both inward and outward, so the box appears filled when the line
    # is at least as wide as the box's smallest half-extent.)
    stroke_radius = float(min(half_x, half_z))

    centers: list[tuple[float, float]] = []
    colors: list[tuple[int, int, int, int]] = []
    radii: list[float] = []
    # Anchor at the LEFT edge of the start line so the lateral grid
    # spans cleanly to the right edge.
    for j in range(n_lon):
        s_lon = (j - (n_lon - 1) * 0.5) * square_lon
        for i in range(n_lat):
            s_lat = (i + 0.5) * square_lat
            cx_box = left[0] + s_lat * lat_hat[0] + s_lon * tan_hat[0]
            cz_box = left[1] + s_lat * lat_hat[1] + s_lon * tan_hat[1]
            centers.append((float(cx_box), float(cz_box)))
            is_white = (i + j) % 2 == 0
            colors.append(
                SF_FLAG_COLOR_WHITE_2D if is_white else SF_FLAG_COLOR_BLACK_2D
            )
            radii.append(stroke_radius)

    rr.log(
        entity,
        rr.Boxes2D(
            centers=centers,
            half_sizes=[half_size] * (n_lat * n_lon),
            colors=colors,
            radii=radii,
        ),
        static=True,
    )
    # Drop the explicit "S/F" label nearby, so the pattern still reads
    # as the start/finish line even from the overview zoom level where
    # the squares are sub-pixel.
    rr.log(
        f"{entity}_label",
        rr.Points2D(
            positions=[(cx0, cz0)],
            colors=[(80, 220, 80, 0)],   # invisible dot, label only
            radii=[0.1],
            labels=["S/F"],
        ),
        static=True,
    )


def _log_track_static(track: Track) -> None:
    """Log centreline + left/right edges once as static geometry."""
    rr.log(
        f"{TRACK_ROOT}/centerline",
        rr.LineStrips2D(
            [track.centerline_xz],
            colors=[CENTERLINE_COLOR],
            radii=[TRACK_CENTERLINE_RADIUS_M],
        ),
        static=True,
    )
    rr.log(
        f"{TRACK_ROOT}/left",
        rr.LineStrips2D(
            [track.left_xz],
            colors=[EDGE_COLOR],
            radii=[TRACK_EDGE_RADIUS_M],
        ),
        static=True,
    )
    rr.log(
        f"{TRACK_ROOT}/right",
        rr.LineStrips2D(
            [track.right_xz],
            colors=[EDGE_COLOR],
            radii=[TRACK_EDGE_RADIUS_M],
        ),
        static=True,
    )
    # Chequered start/finish line for orientation.
    _log_chequered_flag_2d(track, f"{TRACK_ROOT}/start")


# ----------------------------- trace logging ------------------------------- #

def _log_car_positions(
    entity: str,
    trace: LapTrace,
    color: tuple[int, int, int, int],
    label: str,
    timeline: str,
) -> None:
    """Log the moving car as Points2D over the shared timeline.

    One position per sample; Rerun draws a single point at each scrubber
    tick. Uses ``send_columns`` so that 10 k-sample ghost traces don't
    cost 10 k Python-side ``log`` calls.

    A separate static label is logged once at the trajectory start so
    the car identity stays visible even when the timeline is paused
    before either trace begins.
    """
    n = trace.xz.shape[0]
    # send_columns partitions one row per index by default, so per-row
    # shapes are (1, 2) for positions and (1, 4) for colours.
    positions = trace.xz.reshape(n, 1, 2).astype(np.float32)
    colors = np.tile(np.array(color, dtype=np.uint8), (n, 1, 1))
    radii = np.full((n, 1), CAR_RADIUS_M, dtype=np.float32)

    rr.send_columns(
        entity,
        indexes=[rr.TimeColumn(timeline, duration=trace.t_s)],
        columns=rr.Points2D.columns(
            positions=positions,
            colors=colors,
            radii=radii,
        ),
    )
    # Static identity label anchored at the start of the trace; Rerun's
    # per-instance labels on time-varying Points2D batches were flaky in
    # 0.32, so we attach the label to a sibling static entity instead.
    rr.log(
        f"{entity}_label",
        rr.Points2D(
            positions=[trace.xz[0]],
            colors=[color],
            radii=[CAR_RADIUS_M * 0.5],
            labels=[label],
        ),
        static=True,
    )


def _dedup_consecutive_2d(xz: np.ndarray) -> np.ndarray:
    """Drop runs of consecutive identical 2D points (see scene3d for why).

    ``LineStrips2D`` does not extrude tubes like ``LineStrips3D`` does,
    so coincident vertices are visually harmless in the top-down view
    — but the v3 sim trace can hold the same (x, z) for several
    consecutive ticks (~68 % of samples in the standard repro lap),
    and feeding ~5 k zero-length segments to the viewer wastes Arrow
    payload and pollutes any future hit-test / hover code in the
    top-down pane. Mirroring the 3D dedup keeps the two views aligned.
    """
    if xz.shape[0] < 2:
        return xz
    diff = np.any(np.diff(xz, axis=0) != 0.0, axis=1)
    keep = np.empty(xz.shape[0], dtype=bool)
    keep[0] = True
    keep[1:] = diff
    return xz[keep]


def _log_trajectory_static(
    entity: str, trace: LapTrace, color: tuple[int, int, int, int]
) -> None:
    """Static polyline of the full reconstructed path (for orientation)."""
    faded = (color[0], color[1], color[2], 90)
    xz = _dedup_consecutive_2d(trace.xz)
    rr.log(
        entity,
        rr.LineStrips2D(
            [xz], colors=[faded], radii=[TRAJECTORY_RADIUS_M]
        ),
        static=True,
    )


def _log_scalars(
    entity_root: str,
    trace: LapTrace,
    timeline: str,
) -> None:
    """Log per-channel scalar series on the shared timeline.

    One entity path per channel so Rerun gives each its own plot pane.
    """
    times = rr.TimeColumn(timeline, duration=trace.t_s)
    for name, arr in trace.scalars.items():
        # NaN-filter for safety; Rerun handles NaN but pandas-loaded floats
        # can carry surprises.
        if not np.all(np.isfinite(arr)):
            arr = np.where(np.isfinite(arr), arr, 0.0)
        rr.send_columns(
            f"{entity_root}/{name}",
            indexes=[times],
            columns=rr.Scalars.columns(scalars=arr),
        )


# ----------------------------- driver -------------------------------------- #

def _summarize(trace: LapTrace, kind: str) -> str:
    lines = [
        f"  {kind}: {os.path.basename(trace.source_path)}",
        f"    samples = {len(trace.t_s)}",
        f"    duration = {trace.duration_s:.3f} s",
        f"    distance = {trace.distance_m[-1] - trace.distance_m[0]:.1f} m",
    ]
    if trace.missing_columns:
        lines.append(f"    missing optional columns: {trace.missing_columns}")
    return "\n".join(lines)


def run(args: argparse.Namespace) -> int:
    track = load_track(args.track)
    ghost = load_ghost_trace(args.ghost, track, label=args.ghost_label)
    sim = load_sim_trace(args.sim, track, label=args.sim_label)

    print(f"Loaded track: {os.path.basename(args.track)} "
          f"(length {track.total_length_m:.1f} m, {len(track.distance_m)} samples)")
    print(_summarize(ghost, "ghost (real)"))
    print(_summarize(sim, "sim"))

    enable_3d = not args.no_3d
    blueprint = None
    if enable_3d:
        # Build the blueprint up front so we can pass it into init/spawn.
        # That avoids the viewer briefly auto-laying-out before our
        # blueprint takes effect.
        # The chase-cam path is the PINHOLE entity, which is a CHILD of
        # the chase-cam extrinsics path. See ``log_chase_camera`` for
        # the rationale (Rerun 0.32 requires the pinhole and its
        # extrinsic Transform3D to live on different entity paths so
        # the 2D view's target frame matches the pinhole root).
        blueprint = build_blueprint(
            world_root=WORLD_ROOT,
            chase_cam_path="world/sim_car/chase_cam/sensor",
            telemetry_roots=[
                f"telemetry/{ghost.label}",
                f"telemetry/{sim.label}",
            ],
        )

    if args.no_spawn:
        # Headless smoke: write a .rrd recording instead of opening a
        # window. ``rr.init(default_blueprint=...)`` only ships the
        # blueprint to a spawned viewer; for the headless ``rr.save``
        # sink we must pass it through ``save()`` explicitly, otherwise
        # the resulting .rrd contains no blueprint chunks and the
        # viewer falls back to its auto-layout when the file is loaded
        # later (root cause of round-1 complaint #1).
        rr.init("lap_replay", spawn=False)
        out_path = args.save or os.path.join(".tmp", "lap_replay.rrd")
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
        rr.save(out_path)
        # ``save(..., default_blueprint=)`` emits the blueprint with
        # ``make_active=False``, so the Rerun viewer keeps using any
        # cached active blueprint at
        # ``AppData/Roaming/rerun/data/blueprints/lap_replay.rbl``
        # from a previous run. Round-3: the original chase-cam fix
        # would never reach the user that way. Use ``send_blueprint``
        # with ``make_active=True`` so the rrd's embedded blueprint
        # replaces the cached one on load.
        if blueprint is not None:
            rr.send_blueprint(blueprint, make_active=True, make_default=True)
        print(f"Headless mode: writing recording to {out_path}")
    else:
        rr.init("lap_replay", spawn=True)
        if blueprint is not None:
            rr.send_blueprint(blueprint, make_active=True, make_default=True)
        if args.save:
            rr.save(args.save)
            if blueprint is not None:
                rr.send_blueprint(blueprint, make_active=True, make_default=True)

    if enable_3d:
        # Declare the world frame so the viewer's 3D camera + gizmos
        # match AC's "y is up" convention.
        rr.log(
            WORLD_ROOT,
            rr.ViewCoordinates.RIGHT_HAND_Y_UP,
            static=True,
        )

    # Environment overlay (grass + asphalt). Logged before track lines
    # so the polylines paint on top in 2D; the 3D layered Y offsets
    # (LAYER_OFFSETS_M in viz.scene3d) handle ordering in the 3D pane.
    _log_environment_2d(track, f"{ENV_ROOT}/grass2d")
    if enable_3d:
        log_environment_3d(track, ENV_ROOT)

    _log_track_static(track)
    # Static trajectory polylines must live OUTSIDE the per-tick
    # Transform3D subtree at world/<car>_car; otherwise Rerun cascades
    # the moving car pose onto the polylines and they drift around the
    # scene with the car. World-frame geometry stays at world/<role>_path.
    _log_trajectory_static("world/ghost_path", ghost, GHOST_COLOR)
    _log_trajectory_static("world/sim_path", sim, SIM_COLOR)

    timeline = "sim_time"
    # The 2D car dots MUST be logged at top-level world paths, NOT as
    # children of world/<role>_car. The latter carry a per-tick
    # Transform3D logged by log_car_pose_stream; Rerun cascades parent
    # transforms to children, so any 2D archetype underneath would
    # (a) inherit the per-tick translation and drift around the
    # top-down view away from its absolute (x, z) coordinate, and
    # (b) have its 2D plane re-oriented by the Y-up yaw rotation,
    # producing the "perpendicular phantom track" the user reported.
    # Round-1 of this bug already moved the trajectory polylines out
    # of the moving subtree; round-2 (here) moves the per-tick dots.
    _log_car_positions("world/ghost_dot", ghost, GHOST_COLOR,
                       ghost.label, timeline)
    _log_car_positions("world/sim_dot", sim, SIM_COLOR,
                       sim.label, timeline)

    if enable_3d:
        log_track_3d(track, TRACK_ROOT_3D)
        # Same rationale as the 2D paths: keep these off the moving car
        # transform so they nail the world frame rather than ride along.
        log_trajectory_3d("world/ghost_path3d", track, ghost, GHOST_COLOR)
        log_trajectory_3d("world/sim_path3d", track, sim, SIM_COLOR)
        log_car_pose_stream(
            "world/ghost_car", track, ghost, GHOST_COLOR, timeline
        )
        log_car_pose_stream(
            "world/sim_car", track, sim, SIM_COLOR, timeline
        )
        # Chase camera lives only on the sim car (per spec).
        log_chase_camera("world/sim_car")

    _log_scalars(f"telemetry/{ghost.label}", ghost, timeline)
    _log_scalars(f"telemetry/{sim.label}", sim, timeline)

    if enable_3d:
        print("3D scene enabled: top-down + world-3D + chase-cam views.")
    else:
        print("3D scene disabled (--no-3d). 2D top-down only.")
    print("Rerun stream initialised. Use the viewer's timeline to scrub.")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="viz.rerun_replay",
        description="Replay a real lap (ghost) vs a simulated lap on a "
                    "2D top-down track view using the Rerun viewer.",
    )
    p.add_argument(
        "--track",
        required=True,
        help="Track layout CSV "
             "(e.g. tracks_csv/ks_nurburgring/layout_sprint_a.csv).",
    )
    p.add_argument(
        "--ghost",
        required=True,
        help="Real-lap *_sim_telemetry.csv (the AC reference lap).",
    )
    p.add_argument(
        "--sim",
        required=True,
        help="Simulator *_sim_trace.csv or *_sim_trace_slip.csv.",
    )
    p.add_argument("--ghost-label", default="ghost",
                   help="Label shown next to the ghost car (default: ghost).")
    p.add_argument("--sim-label", default="sim",
                   help="Label shown next to the sim car (default: sim).")
    p.add_argument("--no-spawn", action="store_true",
                   help="Don't open the viewer; write a .rrd file instead "
                        "(useful for smoke tests / CI).")
    p.add_argument("--no-3d", action="store_true",
                   help="Disable the 3D world view and chase camera. "
                        "Default is enabled.")
    p.add_argument("--save", default=None,
                   help="Optional path to also save the recording as .rrd.")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
