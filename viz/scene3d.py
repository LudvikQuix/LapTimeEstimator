"""3D scene + chase-camera logging for the Rerun lap replay.

Companion to ``viz.rerun_replay``. Kept in a separate module so the 2D
top-down logging stays focused and easy to read, and to honour the
~500-line soft ceiling on ``rerun_replay.py``.

Blueprint
---------
``build_blueprint`` returns the explicit three-view layout the viz tool
needs (top-down 2D, world 3D, chase-cam 2D-through-Pinhole). Any failure
inside its construction is now allowed to propagate: a missing
``rerun.blueprint`` import (very old Rerun) returns ``None`` so the
viewer auto-builds a layout, but every other failure is a real bug in
this file or in the Rerun API contract and must not be silently
swallowed (round-1 complaint #1 traced the user-visible "only 2 panes"
symptom to a broad ``except Exception`` here that masked the real
failure).

Frame conventions
-----------------
- The AC world ground plane is ``(x, z)``; ``y`` is vertical (up). The
  loader already preserves that — see ``viz.loader``.
- Inside Rerun we declare the root frame as
  ``rr.ViewCoordinates.RIGHT_HAND_Y_UP`` so the viewer's 3D camera and
  gizmos agree with AC's "y is up" convention. Positions are logged
  directly as ``(x, y, z)`` in AC world coordinates.
- A car's heading on the ground plane is ``psi = arctan2(dz, dx)``. The
  car-local forward axis (after applying the pose's rotation about y) is
  therefore +X in world space when ``psi = 0`` — i.e. our convention for
  "forward in the car's local frame" is +X. The chase camera offset is
  expressed in that same car-local frame.
- Rerun's ``Pinhole`` looks down its own +Z axis by default. We rotate
  the chase-cam transform so that the camera's +Z points along the car's
  local +X (forward). That rotation is baked into the static
  ``chase_cam`` transform so the per-tick car pose carries the whole
  rig with it.

Nothing here writes back to ``src/lap_estimator``. All inputs are
``Track`` / ``LapTrace`` objects produced by ``viz.loader``.
"""
from __future__ import annotations

import numpy as np
import rerun as rr

from viz.loader import LapTrace, Track

# Car-body box (length along forward, width across, height up). Loosely
# matches a saloon; purely visual.
CAR_BODY_HALF_SIZES = (2.0, 0.6, 0.9)  # (half_x, half_y, half_z) car-local
# Chase camera in the car's local frame:
#   x  = forward (positive in front of the car)
#   y  = up
#   z  = lateral
# We want the camera 2 m up and 6 m behind, looking forward.
CHASE_CAM_LOCAL = (-6.0, 2.0, 0.0)  # 6 m behind, 2 m up, on centreline
CHASE_CAM_RESOLUTION = (1280, 720)
CHASE_CAM_FOCAL_PX = 720.0  # ~ 60 deg vertical FOV at 720 px height

# How heavily to smooth heading. A moving-average over this many samples
# kills the per-tick jitter from arclength resampling without smearing
# real corner entries.
HEADING_SMOOTH_WINDOW = 7

# 3D appearance.
CENTERLINE_COLOR_3D = (90, 90, 90, 255)
EDGE_COLOR_3D = (160, 160, 160, 255)
GROUND_Y_OFFSET = 0.0  # we use the layout's own elevation; no flattening

# Environment surfaces (grass + asphalt) rendered as Mesh3D under
# world/env/**. The grass is a single large green quad spanning the
# track bounding box padded outward. The track surface is a triangle
# strip ribbon between left_xz and right_xz, lifted slightly above the
# grass to dodge z-fighting.
ENV_GRASS_COLOR = (60, 130, 60, 255)
ENV_ASPHALT_COLOR = (70, 70, 75, 255)
ENV_GRASS_PAD_M = 200.0       # padding outward from track bbox
ENV_GRASS_Y_DROP_M = 1.0      # grass sits this far below min track elevation

# Layered Y offsets (metres above the grass plane). Earlier (smaller)
# values render lower; later (larger) values render on top. The exact
# numbers don't matter as long as each successive layer is high enough
# to dodge GPU depth-buffer z-fighting (~0.05 m on typical viewer
# distance is plenty for floats). Tweaking these here is the central
# knob; the rest of the file reads from this dict to stay consistent.
LAYER_OFFSETS_M = {
    "grass": 0.0,
    "asphalt": 0.05,
    "sf_flag": 0.10,
    "track_lines": 0.15,
    "trajectory": 0.20,
}

# Chequered start/finish flag laid flat on the ground.
#
# Geometry: an N_LAT x N_LON grid of small flat boxes alternating black and
# white, aligned to the track tangent and the lateral (cross-track) axis
# at distance=0. Sized to span the full track corridor laterally and a
# few metres along the track direction longitudinally. The slab thickness
# is intentionally tiny so it reads as paint on the road, not a wall.
SF_FLAG_N_LAT = 8           # squares across the track (lateral)
SF_FLAG_N_LON = 4           # squares along the track (longitudinal)
SF_FLAG_LON_SIZE_M = 1.25   # per-square longitudinal extent (along tangent)
SF_FLAG_SLAB_HALF_Y = 0.05  # vertical half-thickness (5 cm; hugs ground)
# Lift the chequered flag above the new asphalt surface (LAYER_OFFSETS_M
# ["asphalt"] = 0.05 m). 0.10 m sits the flag clearly on the road but
# still under the polylines (track lines at +0.15 m, trajectory at
# +0.20 m). Kept in sync via LAYER_OFFSETS_M["sf_flag"].
SF_FLAG_LIFT_M = 0.10
SF_FLAG_COLOR_BLACK = (15, 15, 15, 255)
SF_FLAG_COLOR_WHITE = (245, 245, 245, 255)


# ----------------------------- heading ------------------------------------- #

def _smooth(x: np.ndarray, window: int) -> np.ndarray:
    """Centered moving-average smoother. Window is clamped to len(x).

    Edge handling: convolution with mode='same' would taper the ends; we
    instead pad with edge values so the smoothed signal stays anchored
    to the first/last sample. That matters because the chase camera at
    t=0 must not snap to a half-window-shifted heading.
    """
    if window <= 1 or x.size < 3:
        return x.astype(np.float32, copy=False)
    w = min(window, x.size)
    if w % 2 == 0:
        w -= 1
    pad = w // 2
    padded = np.pad(x, pad, mode="edge")
    kernel = np.ones(w, dtype=np.float64) / w
    return np.convolve(padded, kernel, mode="valid").astype(np.float32)


def heading_from_xz(xz: np.ndarray) -> np.ndarray:
    """Per-sample heading psi (radians) from a sequence of (x, z) points.

    psi = arctan2(dz, dx). The naive implementation runs
    ``np.gradient(x), np.gradient(z)`` over the full input and produces
    junk at coincident-sample clusters: when the simulator emits the
    same (x, z) for several consecutive ticks (~68 % of v3 slip-trace
    samples are held at the previous distance for one or two extra
    ticks; cf. the visualization-red-squiggle bug report), the
    gradient at those samples is exactly ``(0, 0)`` and ``arctan2(0,
    0)`` returns ``0`` (East), which then sloshes through the
    smoother and yaws the chase camera ~90 degrees on every duplicate
    cluster.

    Robust path: compute heading on the *unique-position* sub-sequence,
    then fan that value back across all original samples that share
    each unique position. Endpoints of the unique sub-sequence use
    forward/backward differences via ``np.gradient`` as before. The
    unwrap + smooth happens AFTER the fan-out so the smoother sees
    a continuous signal across duplicates rather than a square wave
    between the real heading and 0.
    """
    n = xz.shape[0]
    if n < 2:
        return np.zeros(n, dtype=np.float32)

    # Identify the indices of the first occurrence of each new position.
    # Two consecutive samples are 'duplicates' if both dx and dz are
    # exactly zero (the simulator copies the previous state verbatim).
    dx_raw = np.diff(xz[:, 0])
    dz_raw = np.diff(xz[:, 1])
    is_new = np.ones(n, dtype=bool)
    is_new[1:] = (dx_raw != 0.0) | (dz_raw != 0.0)
    unique_idx = np.where(is_new)[0]

    if unique_idx.size < 2:
        # Degenerate input (everything coincident). Best we can do is
        # report zero heading; the box won't move anyway.
        return np.zeros(n, dtype=np.float32)

    unique_xz = xz[unique_idx]
    udx = np.gradient(unique_xz[:, 0])
    udz = np.gradient(unique_xz[:, 1])
    psi_unique = np.arctan2(udz, udx)
    psi_unique = np.unwrap(psi_unique)

    # Fan out: every sample inherits the heading of the most recent
    # unique-position index <= itself. ``searchsorted`` gives that
    # mapping in O(n log u). The first segment (before any motion) is
    # filled with the first valid heading so the car at t=0 doesn't
    # point East before its first real step.
    pos = np.searchsorted(unique_idx, np.arange(n), side="right") - 1
    pos = np.clip(pos, 0, unique_idx.size - 1)
    psi = psi_unique[pos]

    psi_smoothed = _smooth(psi.astype(np.float64), HEADING_SMOOTH_WINDOW)
    return psi_smoothed.astype(np.float32)


def _yaw_to_quat(psi: np.ndarray) -> np.ndarray:
    """Convert yaw angles into (x, y, z, w) quaternions for Rerun.

    In a right-handed Y-up frame, the standard right-hand-rule rotation
    about +Y by angle ``theta`` maps ``(1, 0, 0)`` to
    ``(cos theta, 0, -sin theta)``. Our heading is defined as
    ``psi = arctan2(dz, dx)``, i.e. the world direction the car should
    end up pointing in is ``(cos psi, 0, sin psi)``. To get the car's
    local +X (forward) to land there we therefore need ``theta = -psi``,
    i.e. a rotation about -Y by ``psi`` (equivalently about +Y by
    ``-psi``). The quaternion is built accordingly.

    Quaternion convention returned: ``(x, y, z, w)``.
    """
    half = -psi.astype(np.float64) * 0.5
    qx = np.zeros_like(half)
    qy = np.sin(half)
    qz = np.zeros_like(half)
    qw = np.cos(half)
    return np.stack([qx, qy, qz, qw], axis=1).astype(np.float32)


# ----------------------------- elevation lookup ---------------------------- #

def _elevation_at_distance(track: Track, distance_m: np.ndarray) -> np.ndarray:
    """Per-sample y (elevation) interpolated along the centreline."""
    d_clipped = np.clip(distance_m, track.distance_m[0], track.distance_m[-1])
    return np.interp(d_clipped, track.distance_m, track.elevation_m).astype(
        np.float32
    )


def _xyz_from_trace(
    track: Track, trace: LapTrace, y_offset: float = 0.0
) -> np.ndarray:
    """Stack (x, y, z) per sample: (x, z) from the trace, y from layout.

    ``y_offset`` is added to the per-sample elevation so callers can
    lift the polyline above the asphalt surface to avoid z-fighting.
    """
    y = _elevation_at_distance(track, trace.distance_m) + float(y_offset)
    return np.column_stack([trace.xz[:, 0], y, trace.xz[:, 1]]).astype(
        np.float32
    )


# ----------------------------- static track -------------------------------- #

def _xz_to_xyz(
    track: Track, xz: np.ndarray, y_offset: float = 0.0
) -> np.ndarray:
    """Resolve elevation for a track-edge polyline in (x, z) form.

    The edges share the centreline's arclength sampling (same row count)
    so we can reuse ``track.elevation_m`` directly without
    re-interpolation. ``y_offset`` is added to every sample so the line
    can be lifted above the asphalt surface to avoid z-fighting.
    """
    y = (track.elevation_m + float(y_offset)).astype(np.float32)
    return np.column_stack([xz[:, 0], y, xz[:, 1]]).astype(np.float32)


# Visibility radii (world units, metres). Sized for the overview zoom
# level where a ~1.7 km circuit fits in roughly 600-1000 CSS pixels:
# at that scale anything below ~1 m is sub-pixel and disappears. See
# round-1 complaints #4 and #6.
TRACK_CENTERLINE_RADIUS_M = -1.5  # negative = screen-space pixel width
TRACK_EDGE_RADIUS_M = -2.0
# TRAJECTORY_RADIUS_M must satisfy radius >= max_segment_length so the
# extruded cylindrical tubes of consecutive LineStrips3D segments overlap
# at their end-caps and read as a smooth line instead of a chain of beads.
# Empirically the sim trace's dedup'd polyline has ~1.5 m max segment
# length (2320 verts over 3.5 km on Nurburgring Sprint; the sim emits
# samples roughly every 1.5 m of arclength). Picking 2.5 m gives ~70 %
# tube overlap (diameter 5 m vs segment 1.5 m). At 1.0 m the radius was
# *below* segment length, the end-caps did not overlap, and the user
# reported a "string of beads" in the chase-cam pane (dotted-trajectory
# bug, follow-up to the elevation 2D-leak fix). See "Visibility radii"
# in docs/architecture-rerun-replay.md.
TRAJECTORY_RADIUS_M = -2.0  # negative = screen-space pixel width (flat line, not extruded tube)


def _log_chequered_flag_3d(track: Track, entity: str) -> None:
    """Log a flat chequered start/finish flag at distance=0 in the 3D scene.

    Produces an ``SF_FLAG_N_LAT`` x ``SF_FLAG_N_LON`` grid of thin
    ``Boxes3D`` slabs alternating black and white, laid flat on the
    ground and oriented with the track tangent at the start of the lap:

    - **Lateral axis** (across-track): perpendicular to the centreline
      tangent in the ``(x, z)`` ground plane, scaled to span the full
      corridor from ``track.left_xz[0]`` to ``track.right_xz[0]`` so the
      flag covers the same width as the painted asphalt — asymmetric
      ``width_left_m`` / ``width_right_m`` columns are handled because
      we use the resolved edges, not the centreline + a symmetric half-
      width.
    - **Longitudinal axis** (along-track): the centreline tangent at
      distance=0, ``SF_FLAG_N_LON`` rows centred on the start. Each row
      has a fixed extent ``SF_FLAG_LON_SIZE_M``.
    - **Vertical axis**: a tiny ``2 * SF_FLAG_SLAB_HALF_Y`` total
      thickness so the slabs read as paint on the road. Centres are
      raised by ``SF_FLAG_LIFT_M`` to avoid z-fighting with the track
      mesh / overlay polylines.

    Rotation: each slab is rotated by a single yaw quaternion that maps
    box-local +X to the world lateral direction. Because the entire
    grid shares one orientation we ship one quaternion and broadcast it
    via ``Boxes3D``'s batch ``quaternions=`` argument (one entry per
    box). Half-sizes are batched per-box too (all identical) so the
    archetype lays out cleanly.

    Colour pattern: classic checkerboard, ``(i_lon + i_lat) % 2`` toggles
    between ``SF_FLAG_COLOR_BLACK`` and ``SF_FLAG_COLOR_WHITE``.
    """
    n_lat = SF_FLAG_N_LAT
    n_lon = SF_FLAG_N_LON

    # Lateral vector spans the full corridor at distance=0. We use the
    # resolved track edges (not centreline +/- half-width) so asymmetric
    # tracks come out right.
    left_xz0 = track.left_xz[0].astype(np.float64)
    right_xz0 = track.right_xz[0].astype(np.float64)
    lat_xz = right_xz0 - left_xz0  # vector from left to right edge
    lat_len = float(np.hypot(lat_xz[0], lat_xz[1]))
    if lat_len < 1e-3:
        return  # degenerate track; nothing useful to draw
    lat_hat = lat_xz / lat_len

    # Tangent at distance=0 from successive centreline points; pick the
    # first non-degenerate forward step so a duplicate sample at the
    # head of the layout doesn't blow up the heading.
    cx = track.x.astype(np.float64)
    cz = track.z.astype(np.float64)
    tan_xz = np.array([cx[1] - cx[0], cz[1] - cz[0]])
    tan_len = float(np.hypot(tan_xz[0], tan_xz[1]))
    if tan_len < 1e-6:
        # Fall back to a tangent perpendicular to the lateral axis.
        tan_xz = np.array([-lat_hat[1], lat_hat[0]])
        tan_len = 1.0
    tan_hat = tan_xz / tan_len

    square_lat = lat_len / n_lat
    square_lon = SF_FLAG_LON_SIZE_M

    # Box-local frame layout for the slab:
    #   box-local +X -> world lateral (lat_hat in xz-plane)
    #   box-local +Y -> world +Y (up)
    #   box-local +Z -> world tangent (tan_hat in xz-plane)
    # The rotation about world +Y that takes (1,0,0) into
    # (lat_hat[0], 0, lat_hat[1]) is theta with cos(theta)=lat_hat[0],
    # -sin(theta)=lat_hat[1] (because rotating (1,0,0) about +Y by theta
    # gives (cos theta, 0, -sin theta) in a right-handed Y-up frame).
    # Hence theta = -arctan2(lat_hat[1], lat_hat[0]); the quaternion
    # about +Y is (0, sin(theta/2), 0, cos(theta/2)).
    theta = -float(np.arctan2(lat_hat[1], lat_hat[0]))
    qy = float(np.sin(theta * 0.5))
    qw = float(np.cos(theta * 0.5))
    quat = (0.0, qy, 0.0, qw)

    # Anchor at the centreline start (resolves elevation from the layout).
    anchor = np.array(
        [cx[0], float(track.elevation_m[0]) + SF_FLAG_LIFT_M, cz[0]],
        dtype=np.float64,
    )

    # Build the grid in world space. The lateral index i runs 0..n_lat-1
    # from left edge to right edge; longitudinal index j runs from
    # -(n_lon/2) ..  (n_lon/2 - 1) so the pattern straddles the S/F line.
    half_size = (square_lat * 0.5, SF_FLAG_SLAB_HALF_Y, square_lon * 0.5)

    centers: list[tuple[float, float, float]] = []
    colors: list[tuple[int, int, int, int]] = []
    for j in range(n_lon):
        # Longitudinal offset relative to the anchor (centred on S/F line).
        s_lon = (j - (n_lon - 1) * 0.5) * square_lon
        for i in range(n_lat):
            # Lateral offset relative to the left edge, into a per-square centre.
            s_lat = (i + 0.5) * square_lat
            cx_box = (
                left_xz0[0] + s_lat * lat_hat[0] + s_lon * tan_hat[0]
            )
            cz_box = (
                left_xz0[1] + s_lat * lat_hat[1] + s_lon * tan_hat[1]
            )
            centers.append((float(cx_box), anchor[1], float(cz_box)))
            is_white = (i + j) % 2 == 0
            colors.append(
                SF_FLAG_COLOR_WHITE if is_white else SF_FLAG_COLOR_BLACK
            )

    n_total = n_lat * n_lon
    rr.log(
        entity,
        rr.Boxes3D(
            centers=centers,
            half_sizes=[half_size] * n_total,
            quaternions=[quat] * n_total,
            colors=colors,
            fill_mode="solid",
        ),
        static=True,
    )


def _build_grass_mesh(track: Track) -> tuple[np.ndarray, np.ndarray]:
    """Two-triangle quad spanning the track bbox padded by ENV_GRASS_PAD_M.

    Returns ``(vertex_positions (4, 3), triangle_indices (2, 3))``.

    The quad sits at ``track.elevation_m.min() - ENV_GRASS_Y_DROP_M``
    (slightly below the lowest track sample) so the asphalt surface mesh,
    which inherits per-sample elevation, always sits above the grass.
    Counter-clockwise winding when viewed from above (camera looking down
    along -Y) so Rerun's default front-face rule matches the visible side.
    """
    x_min = float(track.x.min()) - ENV_GRASS_PAD_M
    x_max = float(track.x.max()) + ENV_GRASS_PAD_M
    z_min = float(track.z.min()) - ENV_GRASS_PAD_M
    z_max = float(track.z.max()) + ENV_GRASS_PAD_M
    y = float(track.elevation_m.min()) - ENV_GRASS_Y_DROP_M + LAYER_OFFSETS_M["grass"]
    # Vertex order (looking down -Y, +X right, +Z up-in-screen):
    #   v0 (-x, -z)   v1 (+x, -z)
    #   v3 (-x, +z)   v2 (+x, +z)
    # CCW from above: v0 -> v1 -> v2 and v0 -> v2 -> v3.
    verts = np.array(
        [
            [x_min, y, z_min],
            [x_max, y, z_min],
            [x_max, y, z_max],
            [x_min, y, z_max],
        ],
        dtype=np.float32,
    )
    tris = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)
    return verts, tris


def _build_track_surface_mesh(
    track: Track,
) -> tuple[np.ndarray, np.ndarray]:
    """Triangle-strip ribbon between left_xz and right_xz with track elevation.

    For each centreline sample ``i`` we emit a left vertex and a right
    vertex at ``track.elevation_m[i] + LAYER_OFFSETS_M["asphalt"]``. Each
    pair of consecutive samples then forms a quad split into two
    triangles. Winding is chosen so the upward-facing side is the front
    face (CCW when viewed from above, i.e. from +Y looking toward -Y).

    With ``left_normal = rotate(tangent, +90 deg)`` (see
    ``viz.loader.load_track``), left lies to the +90 deg side of the
    tangent in (x, z). Viewed from above with +Y up:
      left_i  -> v[2 i]
      right_i -> v[2 i + 1]
    Triangles per segment ``i`` (between samples i and i+1):
      tri A: (left_i, right_i, right_{i+1})  indices (2i, 2i+1, 2i+3)
      tri B: (left_i, right_{i+1}, left_{i+1})  indices (2i, 2i+3, 2i+2)
    These are CCW from above when ``right`` is to the right of the
    tangent (the AC convention). If a future track inverts that,
    backface culling is off in Rerun's default Mesh3D, so the ribbon
    still renders correctly either way.
    """
    n = track.x.shape[0]
    y = track.elevation_m + LAYER_OFFSETS_M["asphalt"]
    # Interleave left/right vertices: row 2i = left_i, row 2i+1 = right_i.
    verts = np.empty((2 * n, 3), dtype=np.float32)
    verts[0::2, 0] = track.left_xz[:, 0]
    verts[0::2, 1] = y
    verts[0::2, 2] = track.left_xz[:, 1]
    verts[1::2, 0] = track.right_xz[:, 0]
    verts[1::2, 1] = y
    verts[1::2, 2] = track.right_xz[:, 1]
    # Two triangles per segment between consecutive centreline samples.
    seg = np.arange(n - 1, dtype=np.uint32)
    base = 2 * seg
    tri_a = np.column_stack([base, base + 1, base + 3])
    tri_b = np.column_stack([base, base + 3, base + 2])
    tris = np.empty((2 * (n - 1), 3), dtype=np.uint32)
    tris[0::2] = tri_a
    tris[1::2] = tri_b
    return verts, tris


def _log_environment_3d(track: Track, env_root: str) -> None:
    """Log green grass + grey asphalt ribbon as static Mesh3D under env_root.

    Two entities:
      * ``<env_root>/grass3d``        — large green ground quad
      * ``<env_root>/track_surface3d`` — grey ribbon between track edges

    The grass sits at ``min(elevation) - ENV_GRASS_Y_DROP_M``; the
    asphalt rides ``LAYER_OFFSETS_M["asphalt"]`` (5 cm) above per-sample
    elevation. Polylines and the chequered flag use higher offsets in
    ``LAYER_OFFSETS_M`` so they paint cleanly on top.
    """
    grass_verts, grass_tris = _build_grass_mesh(track)
    rr.log(
        f"{env_root}/grass3d",
        rr.Mesh3D(
            vertex_positions=grass_verts,
            triangle_indices=grass_tris,
            albedo_factor=ENV_GRASS_COLOR,
        ),
        static=True,
    )
    surf_verts, surf_tris = _build_track_surface_mesh(track)
    rr.log(
        f"{env_root}/track_surface3d",
        rr.Mesh3D(
            vertex_positions=surf_verts,
            triangle_indices=surf_tris,
            albedo_factor=ENV_ASPHALT_COLOR,
        ),
        static=True,
    )


def log_environment_3d(track: Track, env_root: str = "world/env") -> None:
    """Public entry point for grass + asphalt logging (3D scene)."""
    _log_environment_3d(track, env_root)


def log_track_3d(track: Track, root: str) -> None:
    """Log centreline + edges + S/F chequered flag in 3D under ``root``.

    Centreline and edge polylines are lifted by
    ``LAYER_OFFSETS_M["track_lines"]`` so they paint above the asphalt
    surface mesh (logged separately under ``world/env/track_surface3d``)
    without z-fighting.
    """
    line_lift = LAYER_OFFSETS_M["track_lines"]
    centerline_xyz = np.column_stack(
        [track.x, track.elevation_m + line_lift, track.z]
    ).astype(np.float32)
    rr.log(
        f"{root}/centerline",
        rr.LineStrips3D(
            [centerline_xyz],
            colors=[CENTERLINE_COLOR_3D],
            radii=[TRACK_CENTERLINE_RADIUS_M],
        ),
        static=True,
    )
    rr.log(
        f"{root}/left",
        rr.LineStrips3D(
            [_xz_to_xyz(track, track.left_xz, y_offset=line_lift)],
            colors=[EDGE_COLOR_3D],
            radii=[TRACK_EDGE_RADIUS_M],
        ),
        static=True,
    )
    rr.log(
        f"{root}/right",
        rr.LineStrips3D(
            [_xz_to_xyz(track, track.right_xz, y_offset=line_lift)],
            colors=[EDGE_COLOR_3D],
            radii=[TRACK_EDGE_RADIUS_M],
        ),
        static=True,
    )
    _log_chequered_flag_3d(track, f"{root}/start")


def _dedup_consecutive(xyz: np.ndarray) -> np.ndarray:
    """Drop runs of consecutive identical 3D points.

    Rerun renders ``LineStrips3D`` as extruded cylindrical tubes;
    zero-length segments between coincident vertices produce degenerate
    end-caps whose face normals are essentially random, which the GPU
    then expands into the "crinkly / branching red shapes" the
    visualization-red-squiggle bug report described. The fix is to
    feed the LineStrips3D archetype only the unique positions in
    order — the resulting polyline is geometrically the same path but
    contains no degenerate segments.

    Keeps the first sample and any sample that differs from its
    predecessor on at least one of (x, y, z). Order is preserved.
    """
    if xyz.shape[0] < 2:
        return xyz
    diff = np.any(np.diff(xyz, axis=0) != 0.0, axis=1)
    keep = np.empty(xyz.shape[0], dtype=bool)
    keep[0] = True
    keep[1:] = diff
    return xyz[keep]


def log_trajectory_3d(
    entity: str,
    track: Track,
    trace: LapTrace,
    color: tuple[int, int, int, int],
) -> None:
    """Static faded polyline of the full reconstructed path in 3D.

    Lifted by ``LAYER_OFFSETS_M["trajectory"]`` above the elevation
    profile so the trajectory paints on top of the asphalt surface and
    the track edge lines.
    """
    xyz = _dedup_consecutive(
        _xyz_from_trace(track, trace, y_offset=LAYER_OFFSETS_M["trajectory"])
    )
    faded = (color[0], color[1], color[2], 90)
    rr.log(
        entity,
        rr.LineStrips3D(
            [xyz], colors=[faded], radii=[TRAJECTORY_RADIUS_M]
        ),
        static=True,
    )


# ----------------------------- car poses ----------------------------------- #

def log_car_pose_stream(
    car_root: str,
    track: Track,
    trace: LapTrace,
    color: tuple[int, int, int, int],
    timeline: str,
) -> np.ndarray:
    """Log per-tick Transform3D + a static car-body Boxes3D under ``car_root``.

    Returns the per-sample heading (radians) so callers can reuse it
    (e.g. to log it as a scalar) without re-computing.

    The car body is a child static box; because Rerun transforms cascade,
    any other 3D entity logged under ``car_root`` (the chase camera, a
    future per-wheel slip arrow, etc.) automatically inherits the car's
    pose at the current timeline tick.
    """
    xyz = _xyz_from_trace(track, trace)
    psi = heading_from_xz(trace.xz)
    quats = _yaw_to_quat(psi)

    rr.send_columns(
        car_root,
        indexes=[rr.TimeColumn(timeline, duration=trace.t_s)],
        columns=rr.Transform3D.columns(
            translation=xyz,
            quaternion=quats,
        ),
    )
    # Static car body in the car-local frame. Forward = +X, up = +Y,
    # lateral = +Z. half_sizes is per-axis half-extent.
    rr.log(
        f"{car_root}/body",
        rr.Boxes3D(
            half_sizes=[CAR_BODY_HALF_SIZES],
            centers=[(0.0, CAR_BODY_HALF_SIZES[1], 0.0)],  # sit on the ground
            colors=[color],
        ),
        static=True,
    )
    return psi


# ----------------------------- chase camera -------------------------------- #

def _forward_facing_quat() -> tuple[float, float, float, float]:
    """Quaternion that aligns the camera's +Z with the car's local +X.

    Rerun's ``Pinhole`` archetype defaults to **RDF**: camera +X is
    image-right, camera +Y is image-down, camera +Z is "into the scene"
    (forward). We want the camera, sitting at a static offset in the
    car-local frame, to look along the car's forward direction (car +X)
    with the world's up direction appearing up in the image.

    Car-local frame is right-handed Y-up (forward=+X, up=+Y, third axis
    = +Z = forward x up = the car's LEFT side by the right-hand rule).
    For the camera basis to also be right-handed (``right x down =
    forward``) while looking forward with world-up appearing up, the
    only consistent choice of axes in car-local coords is:

      camera_x (right)   -> car +Z   (camera right = car LEFT)
      camera_y (down)    -> car -Y   (image y is down, car y is up)
      camera_z (forward) -> car +X   (forward)

    Note the small parity quirk: the screen's left/right are mirrored
    relative to the car's left/right. This is an unavoidable consequence
    of pairing a right-handed RDF camera with a right-handed Y-up world;
    the alternative would require flipping vertical (worse). For the
    chase-cam use case it's invisible — the car appears centred,
    upright, driving away from the viewer.

    Required rotation matrix (columns are the camera-axis directions
    expressed in car-local coords):

        [[0, 0, 1],
         [0,-1, 0],
         [1, 0, 0]]

    Determinant is +1 (proper rotation). Its quaternion (x, y, z, w) is
    ``(sqrt(2)/2, 0, sqrt(2)/2, 0)`` — verified via scipy
    ``Rotation.from_matrix``. We just hard-code it; the rig is static.
    """
    s = np.sqrt(0.5)
    return (s, 0.0, s, 0.0)


def log_chase_camera(car_root: str) -> str:
    """Attach a chase-cam rig under ``car_root`` and return the pinhole path.

    The rig is split across two entities to satisfy Rerun 0.32's
    Spatial2DView constraint that the view's target frame BE the
    pinhole-rooted frame (``re_view_spatial::is_valid_space_for_content``).
    Logging both the Transform3D and the Pinhole on the same entity
    path makes the view's target frame land one rung above the pinhole
    in the subspace topology, which then fails the equality check and
    produces a per-entity "3D visualizers require a pinhole at the
    origin of the 2D view" warning for every world 3D entity. Round-3
    investigation traced this back to the residual surface of
    rerun-io/rerun#6138 — the team's "fixed" comment refers to the
    parent-visibility regression, not the topology equality.

    Layout produced:

        car_root/                       (per-tick Transform3D from caller)
          chase_cam/                    (static Transform3D: extrinsics)
            sensor/                     (static Pinhole only: intrinsics)

    The pinhole entity ``chase_cam/sensor`` carries only the Pinhole
    archetype; its parent ``chase_cam`` owns the camera extrinsics
    (translation + rotation in the car-local frame). The
    Spatial2DView is then rooted at the pinhole entity so it is its
    own subspace root.
    """
    cam_xform_path = f"{car_root}/chase_cam"
    pinhole_path = f"{cam_xform_path}/sensor"
    rr.log(
        cam_xform_path,
        rr.Transform3D(
            translation=CHASE_CAM_LOCAL,
            quaternion=rr.Quaternion(xyzw=_forward_facing_quat()),
        ),
        static=True,
    )
    rr.log(
        pinhole_path,
        rr.Pinhole(
            focal_length=CHASE_CAM_FOCAL_PX,
            resolution=CHASE_CAM_RESOLUTION,
        ),
        static=True,
    )
    return pinhole_path


# ----------------------------- blueprint ----------------------------------- #

def build_blueprint(
    world_root: str,
    chase_cam_path: str,
    telemetry_roots: list[str],
):
    """Build a three-view layout: top-down 2D, world 3D, chase-cam 2D.

    Returns the constructed ``Blueprint`` (always non-``None`` on a
    supported Rerun) plus, as a side effect, prints a one-line
    ``Blueprint OK: <n> views`` confirmation to stdout so smoke tests
    can cheaply assert that the blueprint actually shipped.

    The only failure mode that returns ``None`` is the ``rerun.blueprint``
    sub-package being entirely absent (a very old Rerun install). Any
    other exception inside this function is a real bug — either a
    Rerun API drift or a programming error in this file — and is
    allowed to propagate so the caller (and the user) sees a real
    stack trace instead of an unexplained empty layout.
    """
    try:
        from rerun import blueprint as rrb
    except ImportError as exc:
        import sys
        print(
            f"[viz.scene3d] WARNING: rerun.blueprint unavailable "
            f"({type(exc).__name__}: {exc}); falling back to viewer "
            "auto-layout.",
            file=sys.stderr,
        )
        return None

    # Top-down 2D view: explicit allow-list of world-frame 2D entities.
    # Crucially this EXCLUDES anything under world/sim_car/** or
    # world/ghost_car/** — those subtrees carry a per-tick Transform3D
    # (3D-only content for the World-3D and Chase-cam panes). Letting
    # the top-down view pull from them used to drag the 2D car dots
    # through the moving transform and either hide them or render the
    # "perpendicular phantom track" the user reported.
    # Top-down 2D allow-list: only the 2D grass overlay from world/env/**
    # (the sibling 3D Mesh3D entities under that subtree are 3D-only and
    # would otherwise produce per-entity "3D in 2D view" warnings).
    spatial2d = rrb.Spatial2DView(
        origin=world_root,
        contents=[
            f"+ /{world_root}/env/grass2d",
            f"+ /{world_root}/track/**",
            f"+ /{world_root}/sim_dot",
            f"+ /{world_root}/sim_dot_label",
            f"+ /{world_root}/ghost_dot",
            f"+ /{world_root}/ghost_dot_label",
            f"+ /{world_root}/sim_path",
            f"+ /{world_root}/ghost_path",
        ],
        name="Top-down",
    )
    # World 3D view: explicit allow-list of 3D-frame entities. We
    # exclude the top-level 2D dots/labels/paths to keep the 3D pane
    # purely 3D — a Points2D inside a Spatial3DView is ignored by the
    # 3D visualizer anyway, but the explicit selector documents intent
    # and avoids future surprises if Rerun ever decides to up-project
    # 2D archetypes.
    # World 3D allow-list: only the 3D mesh env entities (the sibling 2D
    # grass overlay under world/env/grass2d is 2D-only and would
    # otherwise be a no-op carrying a "2D in 3D view" warning).
    spatial3d_world = rrb.Spatial3DView(
        origin=world_root,
        contents=[
            f"+ /{world_root}/env/grass3d",
            f"+ /{world_root}/env/track_surface3d",
            f"+ /{world_root}/track3d/**",
            f"+ /{world_root}/sim_car/**",
            f"+ /{world_root}/ghost_car/**",
            f"+ /{world_root}/sim_path3d",
            f"+ /{world_root}/ghost_path3d",
        ],
        name="World 3D",
    )
    # Chase cam is a Spatial2DView rooted at the Pinhole entity, NOT
    # a Spatial3DView. In Rerun 0.32 a 2D view at a Pinhole origin
    # reprojects the scene through that camera; a 3D view at the
    # same origin just shows a free-orbit 3D scene that happens to
    # be centred near the pinhole and ignores the projection.
    #
    # ``contents`` is an **explicit allow-list of 3D entities only**.
    # The ARKit-scenes example uses ``["$origin/**", "/world/**"]``
    # which is fine when ``/world/**`` is purely 3D content — but in
    # this app ``/world/`` also carries the 2D top-down archetypes
    # (``world/sim_path``, ``world/sim_dot``, ``world/track/**``,
    # ``world/ghost_path``, ``world/ghost_dot``). A
    # ``Spatial2DView`` rooted at a Pinhole DOES reproject 3D content
    # through the camera, but it ALSO renders any 2D archetypes
    # found in its contents as flat 2D overlays in the image plane,
    # using the archetype's own ``(x, y)`` directly as pixel/scene
    # coords. The lap top-down polyline at world ``(x, z)`` of
    # roughly ``(-400..+40, -1015..+215)`` gets dropped into the
    # chase image plane and appears as a small floating loop in the
    # upper portion of the frame — the elevation-bug report's
    # symptom ("trajectory polylines float high above the ground in
    # the sky area of the frame"). The fix is to exclude every 2D
    # archetype from the chase-cam ``contents`` and keep only the 3D
    # entities, which then go through the pinhole projection
    # normally and land on the road surface where they belong. The
    # ``$origin/**`` entry is still kept so any future image
    # overlays logged under the pinhole path show up.
    # Chase-cam allow-list: 3D entities ONLY. The 2D grass overlay
    # (logged as Boxes2D under world/env/grass2d) MUST be excluded so
    # it does not paint as a flat overlay on the chase image plane;
    # we therefore enumerate the specific 3D env entities rather than
    # wildcarding world/env/**.
    spatial_chase = rrb.Spatial2DView(
        origin=chase_cam_path,
        contents=[
            "$origin/**",
            f"+ /{world_root}/env/grass3d",
            f"+ /{world_root}/env/track_surface3d",
            f"+ /{world_root}/track3d/**",
            f"+ /{world_root}/sim_car/**",
            f"+ /{world_root}/ghost_car/**",
            f"+ /{world_root}/sim_path3d",
            f"+ /{world_root}/ghost_path3d",
        ],
        name="Chase cam",
    )
    telemetry_views = [
        rrb.TimeSeriesView(origin=root, name=root.split("/")[-1])
        for root in telemetry_roots
    ]
    spatial_row = rrb.Horizontal(
        spatial2d, spatial3d_world, spatial_chase
    )
    n_views = 3 + len(telemetry_views)
    if telemetry_views:
        bp = rrb.Blueprint(
            rrb.Vertical(
                spatial_row,
                rrb.Horizontal(*telemetry_views),
                row_shares=[3, 2],
            ),
            collapse_panels=True,
        )
    else:
        bp = rrb.Blueprint(spatial_row, collapse_panels=True)
    print(f"Blueprint OK: {n_views} views")
    return bp
