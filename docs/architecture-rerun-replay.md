# Architecture: Rerun lap replay (`viz/rerun_replay.py`)

Desktop visualization tool that replays a real reference lap (ghost) against
a simulated lap (sim) on a 2D top-down view plus a 3D world view and a
behind-the-car chase camera, using [Rerun](https://rerun.io) (`rerun-sdk`)
as the viewer.

Spec: ad-hoc Buddy spec embedded in the task ticket (no
`dev-planning/<feature>/spec.md`; this is a standalone tool that consumes
existing CSV artefacts and adds no production simulator code). Sibling docs:
`architecture-lap-simulation-stint-v2.md`,
`architecture-slip-model-phase4_2-v31-dp-planner.md`.

---

## What it does

`python -m viz.rerun_replay --track ... --ghost ... --sim ...` opens the
Rerun desktop viewer and shows:

- **Top-down 2D view.** Track centreline + left/right edges as static
  line strips on the AC `(x, z)` ground plane; ghost (grey) and sim
  (red) as `Points2D` logged at **top-level world paths**
  (`world/ghost_dot`, `world/sim_dot`) so they are NOT children of any
  per-tick `Transform3D`. Both are indexed against a single shared
  `sim_time` timeline so the viewer's scrubber moves both cars in
  lock-step. See "Do NOT log 2D archetypes inside a moving Transform3D
  subtree" below for the bug this layout prevents.
- **3D world view.** Same track + cars promoted to 3D. Track geometry
  uses the layout's per-sample elevation (AC `y`) so undulations show.
  Each car is a `Boxes3D` (~4 m x 1.2 m x 1.8 m, length x height x
  width) parented to a per-tick `Transform3D`; child entities (chase
  camera, future per-wheel slip arrows, etc.) inherit the pose for free.
- **Chase-cam view.** A static `Transform3D` (camera extrinsics:
  6 m behind, 2 m above, RDF rotation) and a separate child `Pinhole`
  (intrinsics: ~60 deg FOV, 1280x720) parented to the sim car's pose.
  The two archetypes live on **different entity paths**
  (`world/sim_car/chase_cam` carries the Transform3D, its child
  `world/sim_car/chase_cam/sensor` carries the Pinhole) because Rerun
  0.32's spatial-view code (`re_view_spatial::is_valid_space_for_content`)
  requires the 2D view's target frame to coincide with the pinhole's
  subspace root — when both archetypes are on the same path the view's
  target frame resolves to the parent's frame and every projected 3D
  entity is flagged with "3D visualizers require a pinhole at the origin
  of the 2D view." The blueprint exposes the pinhole path as a dedicated
  `Spatial2DView`; in Rerun 0.32 a 2D view at a pinhole origin
  reprojects the whole 3D scene through that camera, which is exactly
  the chase-cam behaviour we want. (`Spatial3DView` at the same origin
  shows only a free-orbit 3D scene centred near the pinhole and
  ignores the projection — that was the original bug.)
- **Synchronised scalar plots** for each lap's `speed_ms`, `speed_kmh`,
  and (when present) `gas` / `brake`. Steering is logged when present;
  the v3 sim traces currently omit it and the loader records that as a
  `missing_columns` caveat printed to stdout.
- **Faded static polylines** of each trace's reconstructed full path
  (both 2D and 3D), so the divergence between sim and reality is
  obvious even before scrubbing.

`rr.spawn()` is the default. `--no-spawn` writes a `.rrd` recording
instead (used for headless smoke tests; the file can be opened later
with `rerun <path>.rrd`). `--no-3d` disables the 3D scene and chase
camera entirely — the 2D pre-3D behaviour is preserved bit-for-bit.

---

## Why this architecture

**Rerun, not matplotlib / Plotly / Bokeh.** The task is fundamentally
spatial-plus-temporal: two moving entities on a 2D map plus aligned
scalar timeseries. Rerun's whole model — entity paths, timelines,
scrubbable archetypes — was designed for exactly this. The alternatives
all require hand-rolling either the time scrubber (matplotlib /
Plotly) or the spatial view (Bokeh), and none of them give synced
multi-pane layout for free.

**Two cars on one shared timeline, not two timelines.** Both traces
start at `t = 0` from the start/finish line. Aligning them on a single
`sim_time` timeline means the scrubber answers the user's actual
question — "where was the sim car when the real driver was here?" —
directly. Cross-timeline scrubbing in Rerun is awkward and would have
required two timelines plus a custom comparator.

**Position reconstructed from `distance_m`, not stored.** Neither the
ghost telemetry CSV nor the sim trace CSV carries world `(x, y, z)` per
sample. Both record arclength along the centreline (`distanceTraveled`
for the AC bridge, `distance_m` for the simulator). The track layout
CSV is the only artefact with world coordinates. The loader interpolates
`distance_m → (x, z)` against the layout to place each sample on the
map. This matches the existing validation pipeline's convention (see
`src/lap_estimator/validate.py`).

**Columnar `send_columns`, not per-tick `log` loops.** The ghost trace
is ~11 k samples and the sim trace is up to ~1.8 k. Two `log` calls per
sample would be ~25 k Python-side ops; `send_columns` ships each
channel as a single Arrow batch and runs in ~milliseconds.

**Edges built from centreline + width columns.** The layout CSV has
`width_left_m` and `width_right_m` but not pre-resolved edge `(x, z)`.
The loader rotates the local tangent 90 degrees to get the left-of-
travel normal and offsets the centreline by the per-sample width. This
is the same convention `prep/prep_track.py` uses when building those
width columns from the original AC pit-lane / ideal-line data.

**3D in a separate module (`viz/scene3d.py`).** All 3D-specific logic
(heading from successive samples, yaw quaternions, 3D track + car
logging, chase-cam transform, blueprint construction) lives in
`viz/scene3d.py`. Keeps `rerun_replay.py` under the 500-line ceiling
and makes the 3D extension easy to mute via the `--no-3d` flag without
touching the 2D code path. The 3D module imports `Track` and `LapTrace`
from `viz.loader` and has no dependency on the 2D logging functions.

**Heading from `arctan2(dz, dx)` with light smoothing, computed on the
unique-position sub-sequence.** Neither trace records yaw. The
simulator/ghost progress along arclength only, so the chase cam
needs heading reconstructed from successive `(x, z)` samples. A
centered moving-average (window 7) is applied after `np.unwrap` to
kill per-tick jitter without smearing real corner entries. Smoothing
is intentionally edge-anchored (pad-with-edge before convolution) so
`t = 0` doesn't snap to a half-window-shifted heading.

The v3 slip trace holds the same `distance_m` for several consecutive
ticks (the simulator's integrator step is slower than the trace
output rate; in the standard repro lap **68 % of consecutive samples
share the previous distance**, producing 3072 samples where both
`dx == 0` and `dz == 0` exactly). The naive `np.gradient` over the
full input would return `(0, 0)` at every duplicate and `arctan2(0,
0) = 0` would inject "facing East" into the smoother, sloshing the
yaw 90 degrees through every duplicate cluster — which on the
moving `world/sim_car` Transform3D appears as the chase camera and
car body whipping back and forth in the world-3D and chase-cam
panes. (User-visible symptom: piles of "crinkly red shapes" in the
3D world view and "scribbled squiggles" floating around the car
body in the chase view — the visualization-red-squiggle bug
report.)

The fix: compute heading on the *unique-position* sub-sequence (the
indices where `(x, z)` actually changes), then fan that value back
across all original samples that share each unique position via
`np.searchsorted`. The unwrap + smooth happens AFTER the fan-out so
the smoother sees a continuous signal across duplicates instead of a
square wave between the real heading and 0. After the fix, the
worst-case yaw step per tick is ~0.031 rad (1.8 deg) — down from
~0.89 rad (51 deg) before; consecutive-quaternion dot product stays
above 0.9999 across the whole lap.

**Chase cam as a static child transform with a split pinhole.** The
extrinsics (Transform3D — translation `(-6, 2, 0)` and the RDF→car-
local rotation quaternion) are logged at `world/sim_car/chase_cam`.
The intrinsics (Pinhole — focal length + resolution) are logged at the
**child** path `world/sim_car/chase_cam/sensor`. The Spatial2DView is
rooted at the sensor (the pinhole entity). Because the great-grand-
parent `world/sim_car` carries the per-tick pose, the whole rig
follows the car automatically — no per-frame chase logic, no chase-cam
state.

The Transform3D / Pinhole split is **not** stylistic; it is required
by Rerun 0.32's spatial-view validator
(`re_view_spatial::visualizers::utilities::transform_retrieval::
is_valid_space_for_content`). When a 3D visualizer is asked to draw
content inside a `Spatial2DView` whose origin has a Pinhole, the
validator checks `target_frame_pinhole_root == Some(target_frame)`.
If the Pinhole is co-located with a Transform3D on the same entity,
the view's target frame resolves to the parent frame of that entity,
so the equality fails and every world-frame 3D entity ends up flagged
with "3D visualizers require a pinhole at the origin of the 2D view"
in the viewer's per-view issues list. Putting the Pinhole on its own
child entity (no Transform3D there) puts the pinhole at the view's
target frame and the validator passes. This was traced down on
round 3 of the chase-cam pane bug; the underlying issue is
[rerun-io/rerun#6138](https://github.com/rerun-io/rerun/issues/6138)
— the "fixed" comment in that thread referred to a parent-visibility
regression, not the topology equality check above.

See the "AC-axis and Rerun-frame gotchas" section below for the
parity quirk this convention introduces.

**Static trajectory polylines are siblings of `world`, not children
of the cars.** Both the 2D `LineStrips2D` overview path and the 3D
`LineStrips3D` overview path are world-frame geometry — they must not
inherit a moving transform. They live at the top level under
`world/sim_path`, `world/sim_path3d`, `world/ghost_path`,
`world/ghost_path3d`. If they were logged as children of
`world/sim_car` (which carries the per-tick yaw + translation
`Transform3D`), they would be dragged around by the car's pose and
appear to float across the screen — that was the original bug. Only
entities that genuinely live in the car-local frame (the `Boxes3D`
body, the `chase_cam` pinhole rig, any future per-wheel arrows) belong
under `world/sim_car`.

**Trajectory polylines are deduplicated against consecutive
coincident points.** `viz.scene3d._dedup_consecutive` (and its 2D
sibling `viz.rerun_replay._dedup_consecutive_2d`) drop runs of
identical `(x, y, z)` (or `(x, z)`) before the polyline is logged.
Rerun's `LineStrips3D` is rendered as extruded cylindrical tubes;
zero-length segments between coincident vertices produce degenerate
end-caps with essentially random face normals, which the GPU then
expands into the "crinkly / branching red shapes" the
visualization-red-squiggle bug report described. The 2D dedup is
cheap defence-in-depth — `LineStrips2D` does not extrude tubes so
coincident vertices are visually harmless there, but feeding
~5 k zero-length segments to the viewer per lap wastes Arrow payload
(in the standard repro lap the dedup cuts the `sim_path3d` chunk
from 44 KiB to 14.5 KiB — a 3072-point reduction out of 7368 input
samples). The per-tick `Transform3D` on `world/sim_car` is *not*
deduped: each timestamp still ships a distinct translation +
quaternion, so the scrubber stays aligned to the timeline.

**Per-tick 2D car dots are also siblings of `world`, not children of
the cars.** The `Points2D` per-tick markers for "where is each car
right now in the top-down view" are logged at `world/ghost_dot` and
`world/sim_dot` (with sibling labels `world/ghost_dot_label`,
`world/sim_dot_label`). The same constraint as the trajectory
polylines applies: anything under `world/<role>_car/...` inherits the
per-tick `Transform3D`. A 2D archetype inside a moving subtree has
both its position and its 2D-plane orientation rewritten by the
cascade — the dot drifts off its absolute `(x, z)` coordinate AND the
plane gets tipped onto its edge by the per-tick yaw rotation around
+Y in Y-up world coords. In the user-visible reproduction this showed
as "no car dots in the top-down view" plus "a stray perpendicular
phantom track" appearing in the 3D world and chase-cam panes (the
edge-on 2D plane). Round-1 of this bug moved the trajectory
polylines; round-2 moved the per-tick dots. See the explicit gotcha
section below.

**Blueprint views use explicit allow-list `contents`, not the default
catch-all.** The top-down 2D view, the world 3D view, and the chase
cam view each declare their `contents` as an explicit list of entity
selectors. This is defence-in-depth against the "moving Transform3D
subtree" footgun: even if a future logger accidentally puts a 2D
archetype under `world/sim_car/...`, the top-down view will not pull
it (the allow-list doesn't include the car subtree). Conversely the
3D view's allow-list omits the 2D dot/label/path entities so they
can't be up-projected.

---

## Data flow

```
            +-----------------------------+
            | layout_<track>.csv          |
            | (centreline x,y,z + widths) |
            +-------------+---------------+
                          |
                  load_track()
                          |
                          v
+----------------+   +----+----+   +-----------------------+
| *_sim_telemetry|   |  Track  |   | *_sim_trace[_slip].csv|
| .csv (ghost)   |   |  object |   | (simulator output)    |
+-------+--------+   +----+----+   +-----------+-----------+
        |                 |                    |
        | load_ghost_     | xz lookup via      | load_sim_trace()
        |   trace()       |   distance_m       |
        v                 v                    v
   +---------+      (interp shared)       +---------+
   | LapTrace|<----------------------------| LapTrace|
   |  ghost  |                             |   sim   |
   +----+----+                             +----+----+
        |                                       |
        +--------+               +--------------+
                 v               v
         +-------------------------------+
         | rerun_replay.run()            |
         |   * declare RIGHT_HAND_Y_UP   |
         |     view coordinates          |
         |   * log 2D track + trajs      |
         |   * send_columns 2D car poses |
         |   * (3D path, when enabled:)  |
         |       scene3d.log_track_3d    |
         |       scene3d.log_trajectory  |
         |       scene3d.log_car_pose_   |
         |         stream (psi + quat    |
         |         per tick)             |
         |       scene3d.log_chase_camera|
         |         (extrinsics +         |
         |          child pinhole)       |
         |   * send_columns scalars      |
         |   * scene3d.build_blueprint   |
         |   * rr.send_blueprint(...,    |
         |       make_active=True)       |
         |   * rr.spawn() viewer         |
         +-------------------------------+
```

---

## File inventory

| Path | What it does | Why split |
| --- | --- | --- |
| `viz/__init__.py` | Package marker. | Keeps `python -m viz.rerun_replay` working. |
| `viz/loader.py` | CSV parsers + edge / position reconstruction. | Pure data ingestion, no Rerun dependency. Easy to unit-test or reuse from another viz front-end. |
| `viz/rerun_replay.py` | CLI + top-level Rerun orchestration: 2D logging (incl. S/F chequered flag), 3D logging dispatch, blueprint wiring. | All Rerun-stream entry points live here. ~510 lines (above the 500-line soft ceiling by a hair — the chequered-flag helper is tightly coupled to the 2D track logger and pulling it into a third module would be an artificial split). |
| `viz/scene3d.py` | 3D-specific logging (track/cars in 3D, S/F chequered flag in 3D, heading reconstruction, chase-cam transform + pinhole, blueprint builder). | Splits cleanly from the 2D path so `rerun_replay.py` stays focused and `--no-3d` can short-circuit the entire 3D pipeline without scattering branches through the call sites. ~700 lines (mostly docstrings; the 3D chequered-flag helper is ~110 lines including the orientation/rotation comment block). |
| `viz/requirements.txt` | Tool-local dependency list (`rerun-sdk>=0.32,<1.0`). | Matches the existing `web/requirements.txt` pattern — there is no repo-root manifest. |
| `docs/architecture-rerun-replay.md` | This file. | Sibling agents and future devs read `docs/architecture-*.md`. |

Nothing in `src/lap_estimator/` is touched. Nothing in
`src/lap_estimator/dynamics/` is read or written (a concurrent ArchDev
owns that directory).

---

## CSV columns the tool expects

### Track layout (`tracks_csv/<track>/layout_*.csv`)

Required: `distance_m, x, y, z, width_left_m, width_right_m`.
The 2D plane is `(x, z)`; `y` is vertical elevation in the AC world
frame. The layout is loaded with `viz.loader.load_track()`; it does not
go through `src/lap_estimator/track.py` because the viz tool only needs
geometry — not radii, speed limits, etc.

### Ghost telemetry (`*_sim_telemetry.csv`)

Required: `timestamp_ms, distanceTraveled, speedKmh`.
Optional scalars consumed if present: `gas, brake`.
Time is `(timestamp_ms - timestamp_ms[0]) / 1000`. World position is
interpolated from `distanceTraveled` against the layout centreline.

### Sim trace (`*_sim_trace.csv` and `*_sim_trace_slip.csv`)

Required: `distance_m, time_s, sim_speed_ms`.
Optional consumed: `sim_speed_kmh`, `gas, brake, steer`.
Time is `time_s - time_s[0]`. World position is interpolated as above.
The current v3 slip traces omit `gas / brake / steer`; this is reported
in stdout under `missing optional columns` and the matching scalar
plots are simply absent — the rest of the replay still works.

---

## How to run

```
pip install -r viz/requirements.txt

python -m viz.rerun_replay \
    --track tracks_csv/ks_nurburgring/layout_sprint_a.csv \
    --ghost tracks_csv/ks_nurburgring/layout_sprint_a__tomas_full_sim_telemetry.csv \
    --sim   tracks_csv/ks_nurburgring/layout_sprint_a__tomas_full_sim_trace_slip.csv
```

For a headless smoke that writes a recording instead of opening the
viewer:

```
python -m viz.rerun_replay ... --no-spawn --save .tmp/lap_replay.rrd
```

The recording can be opened later with `rerun .tmp/lap_replay.rrd`.

`--no-3d` disables the 3D scene and chase camera (top-down only):

```
python -m viz.rerun_replay ... --no-3d
```

### Viewer layout

`scene3d.build_blueprint` ships a default blueprint with three spatial
views in a horizontal row (top-down 2D | world 3D | chase cam) above a
row of `TimeSeriesView`s for each telemetry root. Tabs are not used —
all three spatial views are visible simultaneously so the user can
compare them while scrubbing.

`build_blueprint` prints `Blueprint OK: <n> views` to stdout on a
successful build (3 spatial + one timeseries per telemetry root, so 5
in the standard ghost+sim case). Smoke tests grep for that line.
`build_blueprint` is **not** wrapped in a broad `except` — a previous
revision was, which silently swallowed real failures (e.g. wrong
`contents=` shape on `Spatial2DView`) and degraded the user to the
viewer's auto-layout with no diagnostic. The only `None` return is when
`rerun.blueprint` cannot be imported at all (very old Rerun); any other
exception now propagates to the user with a real stack trace.

**Chase cam `contents` is an explicit allow-list of 3D entities
only, NOT a broad `/world/**`.** The canonical ARKit-scenes pattern
[`["$origin/**", "/world/**"]`](https://rerun.io/examples/spatial-computing/arkit_scenes)
works when `/world/` is purely 3D content. In this app `/world/` also
carries the 2D top-down archetypes (`world/sim_path`,
`world/sim_dot`, `world/track/**`, `world/ghost_path`,
`world/ghost_dot`, plus the `..._label` siblings). A
`Spatial2DView` rooted at a `Pinhole` reprojects 3D content through
the camera as expected, but it ALSO renders any 2D archetypes found
in its contents as flat 2D overlays in the image plane, using the
archetype's own `(x, y)` directly as scene coordinates. The lap
top-down polyline `LineStrips2D` at world `(x, z)` of roughly
`(-400 .. +40, -1015 .. +215)` gets dropped into the chase image
plane and appears as a small loop floating in the upper half of the
frame — that was the user-visible elevation bug ("trajectory
polylines float high above the ground in the sky area"). The
floating loop is the **2D top-down lap polyline** mis-rendered as a
2D overlay in the chase image, NOT the 3D `*_path3d` polyline
mis-projected. The chase view's `contents=`:

```
[
    "$origin/**",                  # reserves camera subtree for future image overlays
    "+ /world/track3d/**",
    "+ /world/sim_car/**",
    "+ /world/ghost_car/**",
    "+ /world/sim_path3d",
    "+ /world/ghost_path3d",
]
```

Same shape as the world-3D allow-list, minus the 2D entities (which
have no business in a camera projection anyway). Verification: an
`rrd print -vvv` of the recording shows the chase view's
`ViewContents:query` lists exactly these six entries, and the
in-viewer chase pane no longer shows the floating loop — every
polyline projects to the road surface where it belongs.

The world-3D view uses an explicit allow-list:

```
[
    "+ /world/track3d/**",
    "+ /world/sim_car/**",
    "+ /world/ghost_car/**",
    "+ /world/sim_path3d",
    "+ /world/ghost_path3d",
]
```

This explicitly excludes the top-level 2D dot/label/path entities
(`world/sim_dot`, `world/sim_path`, etc.) and anything outside
`/world/...` like `/telemetry/**` (which has its own
`TimeSeriesView`s). 2D archetypes inside a `Spatial3DView` are ignored
by the 3D visualizer in practice, but the explicit selector documents
intent and survives future Rerun behaviour drift.

The top-down 2D view's contents are a parallel allow-list of the
world-frame 2D entities (`/world/track/**`, `/world/sim_dot`,
`/world/sim_dot_label`, `/world/ghost_dot`, `/world/ghost_dot_label`,
`/world/sim_path`, `/world/ghost_path`). It **must not** pull from
`/world/sim_car/**` or `/world/ghost_car/**` — those subtrees carry
per-tick `Transform3D`s and pulling 2D content through them is the
root cause of the "missing top-down dots / perpendicular phantom
track" bug documented below.

**Blueprint is shipped with `make_active=True`, not just as a
default.** The recording is produced by calling `rr.save(path)` and
then `rr.send_blueprint(blueprint, make_active=True,
make_default=True)`. The earlier pattern — `rr.save(path,
default_blueprint=blueprint)` — emits a
`BlueprintActivationCommand(make_active: false, make_default: true)`,
which means the Rerun viewer keeps using any cached active blueprint
it already has for the `lap_replay` application id (cache lives at
`%APPDATA%/rerun/data/blueprints/lap_replay.rbl` on Windows). That
caching behaviour is by design — the user's manual viewer tweaks
must survive across runs — but it also pins any previously-shipped
broken blueprint until the user clicks "Reset Blueprint" in the
viewer. Calling `send_blueprint(..., make_active=True)` writes
`make_active: true` into the rrd, which the viewer treats as an
explicit "use this blueprint right now" command and overrides the
cache.

`rerun rrd print <path>.rrd | grep BlueprintActivation` is the
cheapest way to verify the right flag landed; you should see
`make_active: true, make_default: true`. For full blueprint
inspection, `rerun rrd stats <path>.rrd` still works — look for
non-zero counts of `ViewBlueprint:*`, `ContainerBlueprint:*`, and
`ViewportBlueprint:root_container` components.

To get the same three-view setup manually if you ever need to
reconstruct it inside the viewer (e.g. you opened a blueprint-less
recording):

1. Click "+ Add view" -> `Spatial 2D` with origin `world` and content
   query (one per line):
   ```
   + /world/track/**
   + /world/sim_dot
   + /world/sim_dot_label
   + /world/ghost_dot
   + /world/ghost_dot_label
   + /world/sim_path
   + /world/ghost_path
   ```
   -> "Top-down". Do NOT use the catch-all `/**` here; pulling content
   under `/world/sim_car/**` or `/world/ghost_car/**` will drag any 2D
   archetypes inside those subtrees through the per-tick `Transform3D`
   and either hide them or render the perpendicular-plane artefact.
2. "+ Add view" -> `Spatial 3D` with origin `world` and content query:
   ```
   + /world/track3d/**
   + /world/sim_car/**
   + /world/ghost_car/**
   + /world/sim_path3d
   + /world/ghost_path3d
   ```
   -> "World 3D".
3. "+ Add view" -> `Spatial 2D` with origin
   `world/sim_car/chase_cam/sensor` and content query
   `$origin/** /world/**` (two entries) -> "Chase cam". The origin must
   be the **pinhole entity** (the leaf with only `Pinhole`), not the
   parent path that carries the extrinsic `Transform3D` — see the
   "Chase cam as a static child transform with a split pinhole"
   subsection above for why. The view must be 2D, not 3D — the 2D view
   reprojects through the Pinhole, the 3D view does not.
4. Drag the three views into a horizontal row at the top.

### Visibility radii

Rerun's `radii` are in world units (metres). At the overview zoom level
where a ~1.7 km circuit fits in roughly 600-1000 CSS pixels, anything
below ~1 m is sub-pixel and disappears entirely. The radii are sized
for that overview-zoom regime, not for a physical-scale road-marking
view:

| Element | Radius (m) | Where |
| --- | --- | --- |
| Track centreline (2D + 3D) | 1.5 | `TRACK_CENTERLINE_RADIUS_M` in both modules |
| Track edges (2D + 3D) | 2.0 | `TRACK_EDGE_RADIUS_M` |
| Trajectory polyline (2D + 3D) | 2.5 | `TRAJECTORY_RADIUS_M` |
| Car dot (2D `Points2D`) | 4.0 | `CAR_RADIUS_M` |
| Start/finish flag | (see "S/F chequered flag" below) | n/a — sized in world metres from corridor width |

The 3D car body (`Boxes3D`) is still at physical scale (~4 x 1.2 x 1.8 m)
because zooming in for the chase-cam / world-3D view rapidly hits a
regime where physical scale is appropriate. The 2D `Points2D` dot is
purely a marker — its 4 m radius is ~2-3 CSS pixels at overview zoom
and grows on zoom-in, which is the correct visual behaviour for "a dot
that says where the car is right now".

**`TRAJECTORY_RADIUS_M` must be `>= max_segment_length`.** Rerun
renders `LineStrips3D` as extruded cylindrical tubes; consecutive
end-caps overlap into a continuous tube only when the tube diameter
is large enough to bridge each segment. The sim trace emits samples
roughly every 1.5 m of arclength (after `_dedup_consecutive` drops
the held-distance duplicates: 2320 verts over 3556 m of dedup'd
polyline, median segment 1.531 m, max ~1.6 m for non-degenerate
samples; a single ~4.7 m segment at the stationary-to-moving
transition is the only outlier). The earlier value `1.0 m` was
**below** that segment length, the end-caps did not overlap, and the
chase-cam pane showed the polyline as a string of beads — the
dotted-trajectory bug filed after the elevation-2D-leak fix. Setting
the radius to `2.5 m` gives ~57 % tube overlap on the typical 1.5 m
segment (diameter 5 m vs segment 1.5 m), which reads as a smooth
line at every camera zoom level. The single 4.7 m outlier still
shows a one-bead artefact at the lap start but is invisible at
playback speed. If a future sim trace ever has a typical segment
length >2.5 m (slower tick rate, longer track, etc.) the cure is
either to bump `TRAJECTORY_RADIUS_M` further or to interpolate the
dedup'd polyline to a finer spacing before logging — densification
is the cleaner long-term fix but pays a small memory cost (Option B
in the bug ticket; Option A — bump the radius — is currently
sufficient).

---

## Start/finish chequered flag

Replaces the original "single green dot" S/F marker (a `Points3D` /
`Points2D` of radius 3-6 m at `centerline_xyz[0]`) with a flat
chequered-flag grid laid on the ground across the full track corridor,
mirroring the painted start/finish stripe on a real circuit.

**3D pane** (`viz.scene3d._log_chequered_flag_3d`, entity
`world/track3d/start`):
- 8 lateral squares x 4 longitudinal squares = 32 `Boxes3D` in a
  single batch logged statically.
- Lateral axis: the corridor vector
  `track.right_xz[0] - track.left_xz[0]`, so the grid spans the
  resolved track edges (asymmetric `width_left_m` / `width_right_m`
  is preserved). On Nurburgring Sprint that's a 13.44 m wide grid,
  giving 1.68 m per square laterally.
- Longitudinal axis: the centreline tangent at distance=0 (first non-
  degenerate forward step), 4 rows of 1.25 m each, centred on the
  start (so the pattern straddles the S/F line ±2.5 m).
- Vertical extent: 2 x `SF_FLAG_SLAB_HALF_Y` = 0.10 m total thickness;
  centres are raised by `SF_FLAG_LIFT_M = 0.10 m` (sits above the
  asphalt surface at +0.05 m, still under the track lines at +0.15 m
  and the trajectory polylines at +0.20 m — see the "Environment
  overlay" section below for the full `LAYER_OFFSETS_M` ladder).
- Rotation: a single yaw quaternion about world +Y rotates each box's
  local +X onto the world lateral direction. The rotation angle is
  `theta = -arctan2(lat_hat.z, lat_hat.x)` — the negation comes from
  the same right-handed Y-up convention used by `_yaw_to_quat` for
  the car pose. The quaternion is `(0, sin(theta/2), 0, cos(theta/2))`
  and is broadcast to all 32 boxes via `Boxes3D(quaternions=[quat]*32)`.
- Fill: `fill_mode="solid"` so the squares render as filled rectangles
  rather than wireframes. Colours alternate via `(i_lon + i_lat) % 2`
  between `SF_FLAG_COLOR_WHITE = (245, 245, 245, 255)` and
  `SF_FLAG_COLOR_BLACK = (15, 15, 15, 255)`.

**2D top-down pane** (`viz.rerun_replay._log_chequered_flag_2d`, entity
`world/track/start`):
- Same 8x4 grid, same colour pattern, same lateral / longitudinal
  spacing, but axis-aligned in world `(x, z)` because Rerun 0.32's
  `Boxes2D` archetype has **no rotation field**. The half-extents are
  computed as
  `half_x = |lat_hat.x|*square_lat/2 + |tan_hat.x|*square_lon/2` and
  the symmetric expression for `half_z`, which gives the axis-aligned
  bounding box of the rotated square. At Nurburgring Sprint / GP the
  start tangent runs nearly along +Z (psi ~ 90 deg) so the axis-
  aligned bounding box equals the intended square to within ~0.1 deg.
- `Boxes2D` does not have `fill_mode` either. To get filled squares we
  set the per-box `radii` (stroke line width, world metres) to
  `min(half_x, half_z)` — large enough that the stroke covers the
  whole box without bleeding into the neighbour cell.
- A sibling entity `world/track/start_label` carries an invisible
  `Points2D` at `centerline_xz[0]` with an "S/F" text label so the
  pattern still reads as a start/finish marker at overview zoom where
  the squares themselves are sub-pixel.

**Limitations and future work.** The 2D pane's axis-aligned
approximation breaks down for tracks whose S/F line is rotated far
from the world axes (e.g. a hypothetical track with `psi ~ 45 deg`
would render parallelogram-ish squares). The simplest upgrade is to
emit a `LineStrips2D` per square (closed quad in rotated world
coordinates) and accept the outline-only fill, or to wait for a
future Rerun release that adds rotation + `fill_mode` to `Boxes2D`.
The 3D pane is robust at any orientation because `Boxes3D` supports
the quaternion natively.

The blueprint allow-lists already cover `world/track3d/**` (chase +
world-3D views) and `world/track/**` (top-down view), so the new
entities are picked up automatically without blueprint changes.

---

## Environment overlay (grass + asphalt surface)

Renders the world around the track as green grass and the track itself
as a grey paved surface so the scene reads as a real circuit rather
than wireframe lines on a black void. The trajectory polylines, S/F
flag, and car bodies are kept clearly visible on top via a layered
Y-offset stack rather than by ordering tricks alone.

**3D pane** (`viz.scene3d._log_environment_3d`, entities
`world/env/grass3d` and `world/env/track_surface3d`):
- **Grass**: a single 4-vertex / 2-triangle `Mesh3D` quad spanning
  `(track.x.min .. max, track.z.min .. max)` padded by `ENV_GRASS_PAD_M
  = 200 m` on each side. Y is fixed at
  `track.elevation_m.min() - ENV_GRASS_Y_DROP_M (= 1.0 m)` so the
  per-sample asphalt mesh sits cleanly above the grass even at the
  layout's lowest point. Colour `(60, 130, 60, 255)` is passed via
  `Mesh3D(albedo_factor=...)` so the whole quad is a uniform green.
  Winding is CCW from above (camera looking down -Y) to match Rerun's
  default front-face rule.
- **Track surface**: a triangle-strip ribbon between `track.left_xz`
  and `track.right_xz` with `Y = track.elevation_m + 0.05 m`. For each
  centreline sample `i` we emit two interleaved vertices
  (`left_i`, `right_i`) at the lifted elevation; each segment between
  consecutive samples becomes two triangles
  `(left_i, right_i, right_{i+1})` and
  `(left_i, right_{i+1}, left_{i+1})`. With Nurburgring Sprint's
  2325-sample centreline that's 4650 vertices and 9296 triangles —
  well under any GPU budget. Colour `(70, 70, 75, 255)` (asphalt
  grey, lightly darker than the existing centreline line).

**`LAYER_OFFSETS_M` ladder** (in `viz/scene3d.py`, metres above the
grass plane; central place to tune Y stacking):

| Layer            | Offset | Entity / archetype |
|------------------|--------|--------------------|
| `grass`          | 0.00 m | `world/env/grass3d`              (Mesh3D, large green quad) |
| `asphalt`        | 0.05 m | `world/env/track_surface3d`      (Mesh3D ribbon, grey)      |
| `sf_flag`        | 0.10 m | `world/track3d/start`            (32x Boxes3D chequered)    |
| `track_lines`    | 0.15 m | `world/track3d/{centerline,left,right}` (LineStrips3D)      |
| `trajectory`     | 0.20 m | `world/{sim,ghost}_path3d`       (LineStrips3D)             |

Each layer's offset is added on top of the layout's per-sample
elevation (so corners on slopes ride the slope correctly; the offset
is a parallel shift along +Y, not a flattening). 5 cm steps are far
above the GPU depth-buffer's float precision at typical viewer
distances, so z-fighting is impossible by construction. Polyline tube
radii in screen-space units (negative `radii` in Rerun) are unaffected
because the polyline's geometric Y doesn't change relative to the
camera distance.

**2D pane** (`viz.rerun_replay._log_environment_2d`, entity
`world/env/grass2d`):
- Single large green `Boxes2D` covering the same padded bbox as the
  3D grass quad. The 2D top-down view treats world `(x, z)` directly
  as scene coordinates so the rectangle in world metres aligns with
  everything else. Logged FIRST in `run()` so all later 2D archetypes
  (track lines, S/F flag, trajectories, car dots) paint on top.
- **No 2D asphalt surface.** `Spatial2DView` ignores `Mesh3D` archetypes
  and rendering a grey ribbon as many `Boxes2D` would mean thousands of
  axis-aligned boxes that drift from the actual track tangent — the
  visual quality would be worse than the existing line-only top-down
  layout. The 2D top-down keeps the original grey centreline + edge
  polylines as its sole "road" marker and uses the green grass purely
  for "field vs road" contrast.

**Blueprint allow-lists.** Updated to enumerate the env entities
precisely rather than wildcarding `world/env/**`, so each view picks
up only the archetypes it can render:
- Top-down 2D: `+ /world/env/grass2d` (the 3D Mesh3D entities under
  `world/env/**` are silently ignored by `Spatial2DView` but the
  explicit allow-list avoids per-entity "3D in 2D view" warnings).
- World 3D: `+ /world/env/grass3d`, `+ /world/env/track_surface3d`
  (the 2D `grass2d` is excluded for the symmetric reason).
- Chase cam (2D-through-Pinhole): `+ /world/env/grass3d`,
  `+ /world/env/track_surface3d` — same 3D entities only. The 2D
  `grass2d` MUST NOT be in this allow-list: a `Spatial2DView` rooted
  at a `Pinhole` renders `Boxes2D` as a flat overlay in the image
  plane using the box's `(x, z)` directly as pixel coords, which
  would paint a huge green square on top of the projected scene — the
  exact failure mode the existing chase-cam allow-list comment block
  documents for trajectory polylines.

---

## AC-axis and Rerun-frame gotchas

For future maintainers (and for sibling agents wiring per-wheel slip
arrows / debug overlays into the same scene):

- **AC world frame is `(x, y, z)` with `y` vertical.** The ground plane
  is `(x, z)`. The root entity declares
  `rr.ViewCoordinates.RIGHT_HAND_Y_UP` so the viewer's gizmos and
  camera controls agree with this. Don't change that without auditing
  every position log call below.
- **Heading is `arctan2(dz, dx)`.** Not `arctan2(dx, dz)`. The car's
  forward unit vector in world coords is `(cos psi, 0, sin psi)`.
- **Yaw rotation must be negated when converted to a Rerun quaternion.**
  A right-hand-rule rotation about +Y by angle `theta` maps
  `(1, 0, 0)` to `(cos theta, 0, -sin theta)`. To make the car's local
  +X (forward) land at `(cos psi, 0, sin psi)` we therefore need
  `theta = -psi` in the quaternion (`qy = sin(-psi/2)`, `qw =
  cos(-psi/2)`). See `_yaw_to_quat` in `viz/scene3d.py`.
- **Car-local frame is forward=+X, up=+Y, left=+Z.** Right-handed.
  "Left" (not "right") is +Z because `forward x up = +Z` by the
  right-hand rule.
- **Rerun's `Pinhole` defaults to RDF** (X=right, Y=down, Z=forward).
  Combining this with a right-handed Y-up world means the chase
  camera's "right" axis necessarily lines up with the car's **left**
  side. The screen's left/right are mirrored relative to the car's
  left/right — an unavoidable parity quirk. Inverting it would require
  flipping vertical (image upside down), which is worse. For the chase
  view it's invisible.
- **Chase-cam orientation quaternion is `(sqrt(2)/2, 0, sqrt(2)/2, 0)`.**
  Verified by `scipy.spatial.transform.Rotation.from_matrix`. Don't
  re-derive without checking determinant; a reflection (det = -1) will
  pass through Rerun but produce a wrong image.
- **Per-tick `Transform3D` is logged at `world/sim_car` (and
  `world/ghost_car`)**. The 3D car-body `Boxes3D`, the chase-cam
  extrinsics, and the chase-cam pinhole are all descendants of the
  bare `world/sim_car` path so they inherit the pose. The per-tick 2D
  `Points2D` car markers are NOT under that subtree — they live at
  top-level world paths `world/sim_dot` and `world/ghost_dot` (see
  next gotcha).
- **Pinhole is logged on a *child* of the extrinsics entity, not the
  same path.** `world/sim_car/chase_cam` carries the Transform3D
  (camera pose), `world/sim_car/chase_cam/sensor` carries the
  Pinhole (intrinsics). Co-locating them on a single path passes
  Rerun's archetype-level checks but fails the spatial visualizer's
  `is_valid_space_for_content` topology check — the view's target
  frame ends up one rung above the pinhole and every projected 3D
  entity is flagged. The Spatial2DView origin is the `.../sensor`
  child.
- **Do NOT log world-frame geometry under `world/sim_car` or
  `world/ghost_car`.** They both carry per-tick `Transform3D`s. Anything
  underneath them gets dragged along (3D archetypes obey the cascade;
  2D archetypes ignore it, but mixing makes the layout brittle to
  refactor). Static, world-anchored polylines and overlays go at
  sibling paths under `world/` directly — that's why the path overlays
  are at `world/sim_path`, `world/sim_path3d`, etc.
- **Track elevation is kept (`elevation_m`), not flattened.** Nurburgring
  Sprint runs `y ~ 32-65 m`. Flattening to `y = 0` would put the chase
  cam underground at the high points and floating above the road at the
  low points. The cars use the same per-sample elevation
  (`_elevation_at_distance(track, trace.distance_m)`).

---

## Do NOT log 2D archetypes inside a moving Transform3D subtree

This footgun cost two rounds of bug reports; documenting it explicitly
so future maintainers (and sibling agents) don't re-hit it.

**Rule.** Any entity logged under a path that carries a per-tick
`Transform3D` (today: `world/sim_car`, `world/ghost_car`) inherits
that transform on every tick. For 3D archetypes (`Boxes3D`,
`Points3D`, `Arrows3D`, `Pinhole`'s parent `Transform3D`, etc.) that
is the whole point — the car body, the future per-wheel slip arrows,
and the chase camera all *want* to be carried along by the car's
pose. For 2D archetypes (`Points2D`, `LineStrips2D`, `Arrows2D`) it
is a bug:

1. **Top-down drift / disappearance.** The 2D coordinates the
   archetype was logged with were the absolute world `(x, z)`. Once
   the parent Transform3D translates the car to its per-tick world
   position, the child 2D points are translated *again* by the same
   amount. The dot ends up at `2 * (x, z)` for that tick instead of
   `(x, z)`, which at lap-scale is far off-screen and the user sees
   "no dots in the top-down view".
2. **Perpendicular phantom plane in 3D.** Rerun interprets the 2D
   archetype's plane as the parent's local XY. The world frame is
   Y-up, so the world XY plane is already vertical (XZ is the ground
   plane). On top of that, the per-tick yaw rotation around +Y
   re-orients the 2D plane each tick. Stacked together you get the
   thin, edge-on, perpendicular-to-the-track strip the user reported.
3. **Chase-cam corruption.** The chase pane projects through the
   pinhole whose ancestor is the same `world/sim_car`. The misplaced
   2D archetype is inside that ancestor chain so it shows up in the
   chase view too, as scattered points or another edge-on plane.

**Layout.** Top-level world geometry (2D or 3D) goes at sibling paths
under `world/`:

```
world/                       (declares RIGHT_HAND_Y_UP, no transforms)
  env/                       (grass + asphalt env overlay)
    grass2d                  (static Boxes2D, 2D top-down only)
    grass3d                  (static Mesh3D quad, 3D + chase only)
    track_surface3d          (static Mesh3D ribbon, 3D + chase only)
  track/...                  (static 2D centreline + edges)
  track3d/...                (static 3D centreline + edges + S/F)
  ghost_path                 (static 2D LineStrips2D)
  ghost_path3d               (static 3D LineStrips3D)
  ghost_dot                  (per-tick 2D Points2D, time-indexed)
  ghost_dot_label            (static 2D label)
  ghost_car/                 (per-tick Transform3D)
    body                     (static Boxes3D, car-local)
  sim_path                   (static 2D LineStrips2D)
  sim_path3d                 (static 3D LineStrips3D)
  sim_dot                    (per-tick 2D Points2D, time-indexed)
  sim_dot_label              (static 2D label)
  sim_car/                   (per-tick Transform3D)
    body                     (static Boxes3D, car-local)
    chase_cam/               (static Transform3D, extrinsics)
      sensor                 (static Pinhole, intrinsics)
```

Mnemonic: **`*_car/...` is car-local, everything else under `world/`
is world-frame.** If you find yourself logging a `Points2D` or
`LineStrips2D` under `world/sim_car/...` or `world/ghost_car/...`,
stop and rename.

The blueprint allow-lists (top-down view: `+ /world/track/**`,
`+ /world/<role>_dot`, `+ /world/<role>_path`, etc.; world-3D view:
`+ /world/track3d/**`, `+ /world/<role>_car/**`,
`+ /world/<role>_path3d`; chase view: same 3D-only allow-list)
belt-and-braces the rule — even if a future logger violates it the
wrong view won't pick the entity up.

---

## Do NOT let 2D archetypes leak into the chase-cam `contents`

A second, distinct, 2D-vs-3D gotcha. The "moving Transform3D
subtree" rule above keeps the 2D `world/*_path` / `world/*_dot`
archetypes ANCHORED to absolute world `(x, z)` coords. That's good
for the top-down view, but **the chase pane is a `Spatial2DView`
rooted at the camera's pinhole** — and a `Spatial2DView` renders
LineStrips2D / Points2D it can reach **as flat 2D overlays in its
image plane**, regardless of whether the view's origin is a pinhole.
The (x, z) coords of the top-down lap polyline (range roughly
`(-400 .. +40, -1015 .. +215)` for Nurburgring Sprint) get dropped
straight into the chase image plane and the polyline appears as a
small floating loop hovering above the projected 3D scene — that
was the user-visible "trajectory polylines float high above the
ground in the sky area" elevation bug, but it's NOT an elevation
bug at all: the floating loop is the **2D top-down lap polyline**
rendered as a 2D overlay, not the 3D `*_path3d` polyline
mis-projected. The 3D `*_path3d` polyline data was correct
throughout — verified via `rerun rrd print -vvv` showing the first
vertex `[-4.652, 63.962, -764.065]` matched the first car body
`Transform3D:translation` to the bit.

**Fix.** The chase view's `contents=` is an **explicit allow-list
of 3D entities only**, mirroring the world-3D view:

```python
spatial_chase = rrb.Spatial2DView(
    origin="world/sim_car/chase_cam/sensor",   # pinhole entity
    contents=[
        "$origin/**",                  # future image overlays at the pinhole
        "+ /world/track3d/**",
        "+ /world/sim_car/**",
        "+ /world/ghost_car/**",
        "+ /world/sim_path3d",
        "+ /world/ghost_path3d",
    ],
    name="Chase cam",
)
```

A broad `/world/**` (the canonical ARKit-scenes pattern) does NOT
work here because our `/world/` subtree mixes 2D and 3D archetypes
by design — the same recording feeds both a top-down 2D view and a
chase 2D-through-pinhole view. The ARKit example only logs 3D
content under `/world/`, which is why the broad selector is safe
there.

If a future maintainer adds a new 2D archetype anywhere under
`/world/` (a new 2D label, a 2D arrow, a sector boundary line, ...),
they must NOT need to touch the chase allow-list — but they must
also make sure the new entity sits at a top-level world sibling
path that the chase allow-list doesn't include (i.e. NOT under
`world/track3d/...`, `world/*_car/...`, `world/sim_path3d`,
`world/ghost_path3d`). The mnemonic is the same as before:
**`*_car/...` is car-local, `*_path3d` / `track3d/...` are 3D world
geometry; everything else is 2D world geometry and stays out of the
chase view.**

---

## What to extend next

- **Tyre-slip arrows.** When the v3 sim grows per-wheel slip outputs
  into the trace CSV, log them as `rr.Arrows3D` rooted at static
  wheel-position offsets under `world/sim_car/wheel_<fl|fr|rl|rr>` —
  the per-tick car pose handles orientation automatically. (For the 2D
  top-down view, log a parallel `rr.Arrows2D` set at the car's world
  position.)
- **Racing-line overlay.** `layout_<track>_ideal_line.csv` has the same
  shape as the layout CSV and would slot in as a fourth `LineStrips2D`
  with a distinct colour.
- **Time-aligned re-scrub modes.** Today both cars play on shared
  wall-clock from `t = 0`. A distance-aligned scrub (cars synced by
  arclength instead of time) would answer "what speed was the sim
  doing at the same corner the ghost was at?" — that needs a second
  timeline keyed on `distance_m`.
- **Multi-driver comparison.** The CLI takes one ghost and one sim;
  generalising to a list of traces (each with its own colour and
  label) is a small change to `_log_car_positions` / `_log_scalars`
  and a `--trace LABEL=PATH` repeated flag.
