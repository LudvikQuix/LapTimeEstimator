# Complaint Log: Rerun lap replay viz (lap_replay_fix.rrd)

**Spec:** `docs/architecture-rerun-replay.md` + inline task brief
**Audit mode:** Live audit — recording loaded via file-drop into Rerun web viewer at `http://localhost:9090`, recording `C:/repos/LapTimeEstimator/.tmp/lap_replay_fix.rrd`
**Frontend files inspected (for root-cause tracing):** `viz/scene3d.py`, `viz/rerun_replay.py`, `viz/loader.py`
**Screenshots saved:** `.tmp/nitpick_*.png` (relative to repo root)

---

## Round 1 — 2026-05-22

### [CRITICAL] Chase-cam pane is missing from the layout — blueprint renders only 2 spatial views instead of 3

**Heuristic:** H1 — Visibility of system status
**Location:** `viz/scene3d.py:331–386` — `build_blueprint()` function; blueprint panel in viewer
**What happens:** The loaded recording shows exactly 2 spatial views (both labeled "world") in the viewport grid. The expected layout per spec is a 3-panel horizontal row: "Top-down" (2D, origin=world), "World 3D" (3D, origin=world), "Chase cam" (2D, origin=world/sim_car/chase_cam). The third "Chase cam" pane is absent. The blueprint panel on the left sidebar confirms two "world" entries and no chase-cam entry.
**What should happen:** Three spatial panes side by side: a top-down 2D track view, a world 3D view, and a chase-cam reprojection view that follows the sim car.
**Root cause layer:** code
**Reproduction:** Load `.tmp/lap_replay_fix.rrd` into the Rerun web viewer. Observe the blueprint panel (left sidebar): only 2 world views are listed. Scrub the timeline — only 2 spatial panes appear.
**Screenshot:** `.tmp/nitpick_16_reload.png` (t≈0, 1600×1000 viewport — blueprint tree shows 2 world entries, not 3)
**Likely root cause:** The `build_blueprint()` function in `viz/scene3d.py` is wrapped in a broad `except Exception: return None` handler (line 385). If `rrb.Spatial2DView(origin=chase_cam_path, contents=["/**"], ...)` raises on Rerun 0.32.2, the entire blueprint is silently discarded and the viewer auto-lays out with just whatever auto-discovery finds. Alternatively, the `contents=["/**"]` argument is not valid syntax in this version and causes the `try` block to fail. Either way the user gets a broken layout with no error message visible anywhere in the viewer UI.

---

### [CRITICAL] Left spatial pane shows no track geometry — the 3D world view is effectively empty

**Heuristic:** H1 — Visibility of system status
**Location:** `viz/scene3d.py:155–196` — `log_track_3d()` + `viz/rerun_replay.py:257` — `log_track_3d(track, TRACK_ROOT_3D)` call
**What happens:** The left spatial pane (approximately the first "world" panel in the auto-layout) shows a near-black or very dark reddish-brown background. Only a small "s/f" label and a single car dot are visible. The 3D track centreline, left edge, right edge, and start/finish marker (logged under `world/track3d/` at `viz/rerun_replay.py:257`) are invisible.
**What should happen:** The 3D world view should show the full Nürburgring Sprint A circuit as 3D `LineStrips3D` with elevation, with both cars (ghost grey, sim red) as oriented `Boxes3D` sitting on the track.
**Root cause layer:** unclear
**Reproduction:** Load the recording. Look at the left "world" pane. Observe that only the car position indicator is visible; no track geometry appears.
**Screenshot:** `.tmp/nitpick_16_reload.png` (left pane, dark background, single label visible)
**Likely root cause:** Two hypotheses: (1) Since the blueprint was silently discarded (see above), the viewer auto-generates a 2D spatial view for the `world` origin instead of a 3D view — a 2D spatial view ignores `LineStrips3D` entities, so the track disappears. (2) The viewer's auto-camera for the 3D view is pointed somewhere that doesn't include the Nürburgring's coordinate range (~x: -800 to +800m, z: -600 to +600m), and the default near/far clip or FOV misses the geometry. Confirming requires running the recording with a known-good blueprint.

---

### [CRITICAL] Two cars are not visually distinguishable on the top-down 2D view — only one dot appears at the start/finish area

**Heuristic:** H2 — Match between system and real world; H1 — Visibility
**Location:** `viz/rerun_replay.py:251–254` — `_log_car_positions()` calls for ghost and sim; `viz/scene3d.py` static trajectory logging
**What happens:** At t=0 (initial timeline position) the top-down 2D pane shows the track outline clearly but only a single small dot is visible near the start/finish line. As the timeline scrubs forward to ~14–19 seconds (observed in screenshots `.tmp/nitpick_06_after_play.png`, `.tmp/nitpick_07_try_expand.png`), the dot moves but remains a single indistinguishable point — not two separate dots for ghost (grey) and sim (red).
**What should happen:** Two separate colored dots: a light grey dot for the ghost (real Tomas lap) and a red dot for the sim car, at clearly different positions as the lap progresses (since the sim is ~16.6 s slower than real by spec, they should diverge progressively).
**Root cause layer:** unclear
**Reproduction:** Load recording, scrub timeline to t=30s–60s. Expect two separate distinctly-colored dots at different positions on the track. Observed: one dot (or two dots coincident/overlapping).
**Screenshot:** `.tmp/nitpick_08_1600wide.png` (center track pane, only one visible car marker)
**Likely root cause:** Either (a) the `CAR_RADIUS_M = 2.5` in `viz/rerun_replay.py:48` renders both cars at a radius that's tiny relative to the track scale, making them invisible until zoomed in; (b) both cars start at the same position at t=0 and diverge very slowly in the first seconds; or (c) the auto-zoomed view is scaled so the cars are sub-pixel. Cannot confirm which without running the app interactively at t=60s+ where divergence should be large.

---

### [CRITICAL] Static trajectory polylines for ghost and sim are not visible in the top-down 2D pane

**Heuristic:** H1 — Visibility of system status
**Location:** `viz/rerun_replay.py:247–248` — `_log_trajectory_static()` calls; entities `world/ghost_path` and `world/sim_path`
**What happens:** The faded static polylines (grey for ghost, red for sim) that should always show the full lap path as orientation aids are not visible in the top-down 2D pane. The center pane shows only a very faint track outline (likely the centreline/edge geometry from `world/track/`) and no visible faded path overlay for the car trajectories.
**What should happen:** Two faded static polylines (ghost at ~35% opacity grey, sim at ~35% opacity red) should be permanently visible on the track, showing the full reconstructed path for each car. This is critical for showing divergence between real and sim even when the timeline is paused.
**Root cause layer:** unclear
**Reproduction:** Load recording at t=0. The track pane should show the centreline plus two faded path overlays. Only the centreline is visible (faintly). No colored path overlays appear.
**Screenshot:** `.tmp/nitpick_16_reload.png` (center pane — track outline barely visible, no ghost/sim path polylines)
**Likely root cause:** The alpha=90 used for faded color (line 153 of `rerun_replay.py`: `faded = (color[0], color[1], color[2], 90)`) may be rendering too transparent at the automatic zoom level. Or the 2D view at `origin="world"` is auto-fitting to the track extent and the `LineStrips2D` radii of 0.4m are below the pixel threshold at that scale. Not confirmed without interactive zoom.

---

### [MAJOR] Blueprint falls back to auto-layout with no user-facing error — user has no indication something went wrong

**Heuristic:** H9 — Help users recognize, diagnose, and recover from errors
**Location:** `viz/scene3d.py:385` — `except Exception: return None` fallback
**What happens:** When `build_blueprint()` fails (for any reason), it silently returns `None`. `viz/rerun_replay.py:229` passes `default_blueprint=None` to `rr.init(...)` which causes the viewer to auto-generate its layout. The user sees a broken/incomplete layout with no message explaining what happened or how to fix it.
**What should happen:** Either (a) the blueprint error is propagated and printed to stdout alongside the other status messages that `run()` already emits; or (b) the blueprint succeeds and there's nothing to report.
**Root cause layer:** code
**Reproduction:** Any blueprint-construction failure (import error, API mismatch in the `rrb.*` call) will silently degrade to auto-layout. Run the script and observe that stdout shows normal "3D scene enabled" messages even when the layout is broken — there is no `"Blueprint construction failed"` line.

---

### [MAJOR] Top-down 2D pane shows correct track outline but the track geometry is extremely faint — the Nürburgring layout is barely perceptible

**Heuristic:** H8 — Aesthetic and minimalist design; H1 — Visibility
**Location:** `viz/rerun_replay.py:56–95` — `_log_track_static()`; centreline radius 0.15, edge radius 0.25
**What happens:** The top-down 2D pane shows the track outline but it is very faint against a light beige/grey background. The centreline and edges are nearly invisible at the auto-fit zoom level for a ~1.7 km circuit displayed in a ~640px-wide pane.
**What should happen:** The track outline should be immediately legible as a circuit shape. Edge lines and centreline should be clearly visible with good contrast against the background.
**Root cause layer:** ui-polish
**Reproduction:** Load recording and observe the center pane. The loop of the Sprint A circuit is discernible but very low contrast. At CSS zoom levels where the full circuit fits, 0.15m and 0.25m radii effectively disappear.
**Screenshot:** `.tmp/nitpick_16_reload.png` (center pane — faint track outline visible but very low contrast)
**Likely root cause:** The `radii` parameters (0.15m and 0.25m) are physically correct for road markings but too small for overview visualization. Rerun's `LineStrips2D` radii are in world units; at auto-fit zoom where 1.7 km fits in ~600 CSS pixels, 0.25m = ~0.09 CSS pixels — invisible. These should be at least 1–2m for the overview to be legible.

---

### [MAJOR] Telemetry plots lack axis labels — units are absent; user cannot tell if speed is m/s or km/h

**Heuristic:** H2 — Match between system and real world
**Location:** `viz/rerun_replay.py:160–178` — `_log_scalars()` function; entity names `speed_ms`, `speed_kmh`
**What happens:** The six telemetry panels across the top of the viewer show time-series plots with titles like `../ghost/speed_ms`, `../sim/speed_kmh`, etc. There are no y-axis labels and no unit annotations visible. The y-axis values are numbers only.
**What should happen:** Axis labels should indicate units: "Speed (m/s)" for `speed_ms` panels, "Speed (km/h)" for `speed_kmh` panels. The architecture doc says the spec expects 0–70 m/s ≈ 0–250 km/h. The two panels for speed are different units (m/s vs km/h) but visually they look the same to a user unfamiliar with the data source.
**Root cause layer:** ui-polish
**Reproduction:** Load recording, look at the top-row scalar plots. The axis tick values are the only guide; no unit labels appear anywhere on the plots.
**Screenshot:** `.tmp/nitpick_05_loaded_overview.png` (top-row telemetry panels, no axis unit labels visible)
**Note:** Rerun `TimeSeriesView` does not support custom axis labels in v0.32.2 — this may be a spec issue rather than a code issue. If Rerun can't render units, the panel names should at least include the unit (e.g., rename entity paths to include `(kmh)` or `(ms)` suffix). Current path names use opaque abbreviations.

---

### [MAJOR] Ghost and sim speed plots are in different units — `ghost/speed_ms` and `sim/speed_kmh` are NOT directly comparable

**Heuristic:** H2 — Match between system and real world; H1 — Visibility
**Location:** `viz/rerun_replay.py:271–272` — `_log_scalars()` called for ghost and sim traces
**What happens:** The ghost trace logs `speed_ms` (m/s) and `speed_kmh` (km/h). The sim trace logs `speed_ms` (m/s) and `speed_kmh` (km/h). However, looking at the top-row plots, the labels `ghost/speed_ms` and `sim/speed_kmh` appear in separate panes with different y-axis scales. A user comparing "sim/speed_kmh" to "ghost/speed_ms" would be comparing km/h to m/s and the curves would look completely mismatched.
**What should happen:** Panels intended for direct comparison (e.g., ghost speed vs sim speed) should use the same unit and ideally appear on the same plot (two traces) rather than separate panels with different scales.
**Root cause layer:** spec
**Reproduction:** Load recording. Compare the `ghost/speed_ms` panel (y-axis range ~0–40) with the `sim/speed_kmh` panel (y-axis range ~0–140). They appear to be different magnitudes — ghost in m/s vs sim in km/h — making side-by-side comparison misleading.
**Screenshot:** `.tmp/nitpick_05_loaded_overview.png` (top-row plots: second column is `ghost/gas`, third is `ghost/speed_kmh`, fourth is `ghost/speed_ms`, fifth is `sim/speed_kmh`, sixth is `sim/speed_ms` — note `ghost/speed_ms` ≈ 20-40 range while `sim/speed_kmh` ≈ 0-140)
**Note:** The spec (architecture doc) says "Synchronised scalar plots of speed (ghost vs sim)". Two separate panels with different units is a degenerate interpretation of "synchronised".

---

### [MINOR] `ghost/brake` panel shows tall narrow spikes instead of a smooth brake trace — likely an AC telemetry sampling artifact surfaced by the visualization

**Heuristic:** H1 — Visibility
**Location:** `viz/rerun_replay.py:160–178` — `_log_scalars()`; entity `telemetry/ghost/brake` (or `ghost/brake`)
**What happens:** The first panel in the top row (`../ghost/brake`) shows very narrow vertical spikes near the full-y range rather than a typical brake trace curve. The spikes appear at irregular intervals. The `gas` panel shows a normal-looking curve.
**What should happen:** Brake trace should show smooth application/release curves (0–1 range) that match the corners of the circuit, not narrow isolated spikes.
**Root cause layer:** unclear
**Reproduction:** Load recording. Observe the first panel (ghost brake). Very short, sharp spikes at several points in the timeline.
**Screenshot:** `.tmp/nitpick_05_loaded_overview.png` (top-left panel — narrow spikes visible)
**Note:** This could be a data artifact (brake quantization in AC telemetry, or the ghost lap's telemetry was recorded at lower frequency and `send_columns` is showing interpolated spikes). Cannot confirm root cause without running the viz interactively or inspecting the source CSV. Not a viz code bug per se, but the viz is surfacing a confusing data shape.

---

### [MINOR] No play button confirmation — timeline position indicator shows static time, no visual feedback that playback is running or paused

**Heuristic:** H1 — Visibility of system status
**Location:** Rerun viewer timeline controls at the bottom
**What happens:** The timeline shows a time value (e.g., "+14.846 600s") but there is no persistent visual indicator of whether playback is active or paused. The time changes during playback but stops changing without obvious indication when paused. The play/pause button state is very small in the bottom-left corner.
**What should happen:** A clear visual state indicator (e.g., play icon ▶ highlighted when playing, pause icon ⏸ when paused) should make the playback state immediately obvious at a glance.
**Root cause layer:** unclear
**Reproduction:** Press play, let it run, then pause. Determine at a glance whether playback is active or not.
**Screenshot:** `.tmp/nitpick_06_after_play.png` (bottom timeline area — no clear play/pause state visible)
**Note:** This is likely a Rerun viewer limitation (not something the recording author can fix). Filing as informational. Root cause is `unclear` — may be viewer behavior, not script behavior.

---

### [NIT] The recording is named `lap_replay` in the Sources panel, not `lap_replay_fix` — confusing when multiple recordings are present

**Heuristic:** H2 — Match between system and real world
**Location:** `viz/rerun_replay.py:223` — `rr.init("lap_replay", spawn=False, ...)` and line 229 — `rr.init("lap_replay", spawn=True, ...)`
**What happens:** The Sources panel shows the recording name as `lap_replay` regardless of the `--save` output filename (which was `lap_replay_fix.rrd`). If a user has loaded multiple recordings, they cannot distinguish the "fix attempt" recording from the original by name.
**What should happen:** The recording name passed to `rr.init()` should reflect the content/version, or the `--save` filename stem should be used as the recording name so `lap_replay_fix.rrd` shows as `lap_replay_fix` in the Sources panel.
**Root cause layer:** code
**Reproduction:** Load `lap_replay_fix.rrd`. Observe Sources panel: it shows `lap_replay`, not `lap_replay_fix`.
**Screenshot:** `.tmp/nitpick_04_after_drop.png` (Sources panel, left sidebar — shows `lap_replay`)
