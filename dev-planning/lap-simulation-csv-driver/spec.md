# Lap Simulation: CSV Track + Driver Config

**Status:** Draft (v1 + telemetry-fitting + sim-telemetry-emission + cross-track-validation + project-reorg addendum)
**Project:** LapTimeEstimator
**Branch:** feature/sc-71955/lap-simulation
**Created:** 2026-05-13
**Planned with:** Buddy

## 1. Summary

Today the lap-time simulator consumes a `Car` (parsed from AC data) and a `Track` built from hard-coded segments or a simple `{length, radius}` JSON. There is no notion of a driver, the AC-input → CSV preparation steps are scattered between root-level scripts (`decode_acd.py`, `decode_track.py`) and an untracked `Corner_Analysis/` workdir, and corner classification is bolted onto an exploratory analysis script that depends on telemetry merging.

This feature reshapes the tool around three first-class inputs — **car**, **track CSV**, **driver YAML** — and produces a lap-time estimate plus a trace artifact that can be visually validated against the AI reference speed already present in the CSV. The driver model is intentionally minimal in v1 (skill + optional consistency noise) so we get the input/output plumbing right before investing in richer driver behaviour.

The whole feature ships behind **two simulator CLI entry points** — `fit_driver.py` (derive a driver YAML from a real AC telemetry lap) and `lap.py` (run a sim on a track with a given driver, optionally cross-validating against a real lap on that same track via `--validate-against`). The "just run a sim" mode is the default of `lap.py`; the cross-track validation flow is the same script with one extra flag.

It also formalises the **preparation pipeline** (§16): two CLIs `prep/prep_car.py` and `prep/prep_track.py` that turn user-dropped AC content under `cars_in/` and `tracks_in/` into repo-friendly artefacts under `cars_csv/` and `tracks_csv/`. And it splits **corner analysis** (§17) out into a standalone, telemetry-free script `analysis/corner_analysis.py` that reads a track CSV plus the in-repo `tracks_config.json` and writes a canonical corner notation JSON next to the track.

**Pivot (2026-05-13):** instead of asking the user to hand-author `drivers/<name>.yaml`, the headline workflow becomes **deriving the driver YAML from a real Assetto Corsa telemetry lap** via the `fit_driver.py` tool (see §13). Hand-authored YAMLs are still supported — they just stop being the primary entry point.

**Closing the loop (2026-05-13):** after a sim run, the simulator also emits a **synthetic telemetry CSV** that matches the real AC log schema exactly (see §14). This makes the sim's output a drop-in replacement for real telemetry — downstream tools (notably `fit_driver.py`) can ingest sim output the same way they ingest real laps. A fit → sim → emit → re-fit round-trip becomes a strong self-consistency check on the whole pipeline.

**The actual point (2026-05-13):** the real reason this tool exists is **cross-track prediction with validation** (see §15). Fit a driver on Track A, predict their lap time on Track B (which they have never run in sim or in AC), then have the driver drive Track B in AC for ~10 laps to learn it, capture telemetry, and compare. Sections §13 and §14 are the plumbing; §15 is the headline use case.

**Reorg (2026-05-13):** the repository layout is locked. `cars_in/` and `tracks_in/` are user-dropped raw AC content (gitignored). `cars_csv/` and `tracks_csv/` are the outputs of the preparation scripts (tracked). Preparation scripts live under `prep/`, corner analysis lives under `analysis/`, simulator library code lives under `src/lap_estimator/`, and the two simulator CLIs (`lap.py`, `fit_driver.py`) stay at the repo root for discoverability. See §6.9 and §16/§17.

## 2. Goals

- Accept a **car data directory**, a **track CSV path**, and a **driver YAML path** as the three positional inputs to the sim CLI (`lap.py`).
- Consume the rich per-point track CSV format (ideal line preferred, centerline fallback) directly — no intermediate JSON conversion required.
- Apply a simple **driver model** (`skill_pct`, optional `consistency_sigma`) that scales the car's effective grip uniformly.
- Emit a per-point **trace CSV** and a **sim-vs-AI comparison PNG** next to the track input so the user can sanity-check results in Quix Cloud / locally.
- Preserve current stdout lap-time report format.
- Keep modules small (~500-line soft ceiling).
- Provide a `fit_driver.py` CLI that ingests an AC telemetry CSV and emits a driver YAML calibrated to that lap.
- Emit a **synthetic AC-schema telemetry CSV** alongside the trace output by default (configurable cadence, default 100 ms), so sim runs and real laps are interchangeable downstream.
- Ship cross-track validation as a **flag on `lap.py`** (`--validate-against <real_telemetry.csv>`), not as a third CLI: when present, `lap.py` additionally loads the real lap, prints a delta report, writes an overlay PNG and a per-bin delta CSV, and emits a GOOD/LOOSE/BAD verdict (§15). Two simulator CLIs total: `fit_driver.py` and `lap.py`.
- **(reorg)** Ship a **preparation pipeline** (§16): `prep/prep_car.py` (AC car folder → `cars_csv/<car>/data/`), `prep/prep_track.py` (AC track folder → `tracks_csv/<track>/layout_*.csv` rich CSVs). These wrap and replace the scattered logic currently in `decode_acd.py`, `decode_track.py`, and the untracked `Corner_Analysis/` workdir.
- **(reorg)** Ship a **corner analysis tool** (§17): `analysis/corner_analysis.py` reads a track CSV + `tracks_config.json` (in-repo) and writes a `<layout>_corners.json` corner notation file next to the CSV, plus the existing two visualisations. It is telemetry-free.
- **(reorg)** Lock the folder convention: `cars_in/`, `tracks_in/` for raw AC content (gitignored); `cars_csv/`, `tracks_csv/`, `drivers/` for tracked outputs; sample AC log committed under `samples/` (see §6.9).

## 3. Non-goals

- Driver line selection / line deviation (always uses the ideal line if present).
- Separate brake-aggression vs throttle-aggression parameters (v2 hook in §13.9).
- Reaction-time / lift-and-coast / fuel-saving driver behaviours.
- Tyre thermal model, ABS, traction control, weight transfer beyond what `car.py` already does.
- Hooking into `RequirementsForAbnormalityAnalysis.txt` checks — separate feature.
- Multi-lap / fuel-burn / tyre-wear simulation.
- Web UI. CLI only.
- Per-corner skill profile in v1 (v2 hook in §13.6).
- Fitting tyre, aero, or engine parameters from telemetry — only driver-skill scalars are fit; the car model is treated as ground truth.
- Per-track familiarity / learning-curve modelling in v1. v1 treats `skill_pct` and `consistency_sigma` as track-agnostic properties of the driver (§15 limitations). v2 candidate.
- No database, object store, remote artefact server, or networked service. All inputs (car data, track CSV, driver YAML, telemetry logs) are local files committed to or dropped into the repo. DuckDB / remote-storage integration is explicitly v2+.
- **(reorg)** The corner-analysis tool does **not** merge telemetry — it is purely track-derived. The telemetry-merge code path stays in `src/lap_estimator/telemetry.py` and is consumed only by the fit/validate flows.
- **(reorg)** `prep/prep_track.py` does **not** infer corner notation — that's `analysis/corner_analysis.py`'s job, run as a separate post-prep step.

## 4. User stories / scenarios

1. **Run a single lap.** User has decrypted car data in `cars_csv/bmw_1m/data/`, a track CSV at `tracks_csv/ks_nurburgring/layout_gp_a_ideal_line.csv`, and `drivers/pro.yaml`. They run `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_gp_a_ideal_line.csv drivers/pro.yaml` and see the lap-time report on stdout, plus a trace CSV and PNG written next to the track CSV.
2. **Compare two drivers, same car/track.** User runs the command twice with `drivers/pro.yaml` and `drivers/amateur.yaml`. Lap times differ in a way consistent with `skill_pct`. Trace files are named so they don't collide.
3. **Use a centerline-only track.** User points at a `layout_*.csv` with no `*_ideal_line.csv` sibling. Sim runs against the centerline and prints a notice in stdout.
4. **Consistency study.** User sets `consistency_sigma: 0.3` in the driver YAML. Sim performs N Monte-Carlo runs and prints `mean ± σ` instead of a single lap time. Trace CSV/PNG still come from a single representative run (the deterministic skill-only run).
5. **Legacy invocation (built-in track).** User runs `python lap.py cars_csv/bmw_1m monza drivers/pro.yaml`. The legacy hard-coded `monza` track still works; no AI-speed overlay (no CSV source).
6. **Fit a driver from telemetry.** User has an AC log at `samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv`. They run `python fit_driver.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv drivers/ludvik_nurburgring_sprint.yaml`. The tool writes the YAML, runs a validation sim, and prints `Real lap: 1:42.135 / Sim lap: 1:43.402 / Δ = +1.267 s (+1.2%)`.
7. **Loop-closure check.** User runs `lap.py` to produce a sim lap → sim emits a synthetic telemetry CSV → user feeds that synthetic CSV back into `fit_driver.py` against the same car/track. The recovered driver YAML's `skill_pct` matches the original within ~5% and the validation sim lap time is within ~1 s.
8. **Cross-track prediction and validation (headline workflow).** User has fit `drivers/ludvik_nurburgring_sprint.yaml` from a learned Nurburgring Sprint lap (Track A). They now want to know how Ludvík will perform on Brands Hatch (Track B), which they have not driven in AC yet.
   1. Run `python lap.py cars_csv/bmw_1m tracks_csv/brands_hatch/layout_indy_ideal_line.csv drivers/ludvik_nurburgring_sprint.yaml` → predicted lap time + sim telemetry CSV written next to the Brands Hatch track.
   2. User drives Brands Hatch in AC for ~10 laps until lap times stabilise (the "learned" lap), captures telemetry on a representative lap.
   3. Run `python lap.py cars_csv/bmw_1m tracks_csv/brands_hatch/layout_indy_ideal_line.csv drivers/ludvik_nurburgring_sprint.yaml --validate-against <real_brands_lap>.csv` → stdout delta report, overlay PNG, per-corner / per-bin delta table.
   4. Acceptance: |delta| < ~3 s on a 2–3 minute lap is "good"; >10% means either the fit is wrong or the driver has not learned the track yet.
9. **(reorg) Prepare a new car from raw AC content.** User copies `<AC install>/content/cars/bmw_1m` into `cars_in/bmw_1m/` and runs `python prep/prep_car.py cars_in/bmw_1m`. The script writes `cars_csv/bmw_1m/data/*.ini` and `*.lut`. The output is ready for `lap.py`.
10. **(reorg) Prepare a new track from raw AC content.** User copies `<AC install>/content/tracks/ks_nurburgring` into `tracks_in/ks_nurburgring/` and runs `python prep/prep_track.py tracks_in/ks_nurburgring`. The script writes `tracks_csv/ks_nurburgring/layout_<layout>.csv` and `tracks_csv/ks_nurburgring/layout_<layout>_ideal_line.csv` for each layout it finds, populated with the full column set from the README (`distance_m, segment_length_m, x, y, z, elevation_m, gradient_pct, radius_m, speed_ms, speed_kmh, width_left_m, width_right_m, width_total_m`).
11. **(reorg) Run corner analysis on a prepared track.** User runs `python analysis/corner_analysis.py tracks_csv/ks_nurburgring/layout_sprint_a.csv`. The script reads `tracks_config.json` for thresholds and colours, classifies corners, writes `tracks_csv/ks_nurburgring/layout_sprint_a_corners.json` (schema in §17.4), and emits the existing `..._corner_map.png` and `..._speed_vs_position.png` visualisations next to the track CSV. No telemetry input is required.

## 5. Proposed design

- Add a new `driver.py` module with a `Driver` dataclass (loaded from YAML) and a small `apply_to_car(car)` helper that returns a thin grip-scaling wrapper. Keep `car.py` untouched: scaling is done by composition, not mutation.
- Extend `track.py` with a `Track.from_csv(path)` classmethod that parses the rich CSV. The resulting `Track` exposes the same `to_points(ds)` API plus a parallel `to_ai_reference(ds)` returning the AI speed trace for overlay.
- Adjust `simulator.simulate(...)` so it takes an optional `driver` argument and returns a richer `SimResult` carrying the AI reference (when available), per-point time, and the **per-point binding-limit label** (`corner` / `accel` / `brake`) needed by sim-telemetry emission (§14).
- Move output artifact generation (trace CSV, comparison PNG) into a new `report.py` module so `simulator.py` stays focused on physics.
- `lap.py` is the single sim CLI: takes car / track / driver, runs sim (or Monte-Carlo), and orchestrates `report.write_trace_csv`, `report.write_comparison_plot`, and `sim_telemetry.write_synthetic_log` (unless `--no-telemetry`). When `--validate-against <real_telemetry.csv>` is supplied, `lap.py` additionally calls `validate.validate_lap(...)` and writes the overlay PNG + bins CSV (§15). The pre-existing `main.py` file is renamed to `lap.py` outright.
- Add a `telemetry.py` module that owns AC-log parsing and merging-with-track logic. Add a `driver_fit.py` module that consumes the merged frame plus a `Car` and emits driver parameters. Add a thin `fit_driver.py` CLI wrapper.
- Add a `sim_telemetry.py` module that takes a `SimResult` + total track length + cadence (ms) and emits a CSV that matches the AC telemetry schema exactly (§14).
- Add a `validate.py` module that runs a sim + diffs the result against a real AC telemetry CSV. Validation is exposed through `lap.py --validate-against` (§15). Reuses `telemetry.py`'s merge logic and `report.py`'s plotting helper.
- **(reorg)** Move all simulator library modules into a `src/lap_estimator/` package: `car.py`, `track.py`, `driver.py`, `driver_fit.py`, `simulator.py`, `telemetry.py`, `sim_telemetry.py`, `report.py`, `validate.py`. The two simulator CLIs (`lap.py`, `fit_driver.py`) stay at the repo root and import from the package.
- **(reorg)** Move the low-level AC decoders into `prep/`: `decode_acd.py` and `decode_track.py` move there from the repo root. Two new CLI wrappers, `prep/prep_car.py` and `prep/prep_track.py`, are the user-facing entry points and orchestrate the low-level decoders end-to-end from AC raw input to populated `cars_csv/` / `tracks_csv/` output. See §16.
- **(reorg)** Add an `analysis/` folder with `analysis/corner_analysis.py` — a rewrite of the existing `Corner_Analysis/corner_analysis.py` that is telemetry-free, reads `tracks_config.json` for thresholds/colours, and emits the canonical `<layout>_corners.json` notation file (schema in §17.4) plus the two existing PNG visualisations. See §17.

This shape keeps the physics core intact, isolates the driver concept behind a single small module, puts all I/O / plotting / synthetic-telemetry in dedicated modules so no single file pushes the 500-line soft cap, and makes the AC-input → CSV → analysis pipeline first-class and reproducible.

## 6. Sub-features / work breakdown

### 6.1 Driver module (new file: `src/lap_estimator/driver.py`)
- **What it does:** Loads a driver YAML, exposes `skill_pct` and `consistency_sigma`, and provides a function that wraps a `Car` so its lateral and longitudinal grip queries are scaled by `skill_pct` (and optionally perturbed per-point by Gaussian noise with `sigma_grip` derived from `consistency_sigma`).
- **Touchpoints:** new `src/lap_estimator/driver.py`; `src/lap_estimator/simulator.py` (accept driver, use wrapped car).
- **Approach:** `Driver.load(path) -> Driver`. `Driver.wrap(car, rng=None, noise=False) -> CarLike` returns an object that delegates to the underlying car but overrides `max_cornering_speed`, `max_braking_decel`, and `max_traction_force`'s grip-limited branch. Simplest impl: scale `tyre_grip_lateral` and `tyre_grip_longitudinal` results by `skill_pct * (1 + noise)` where `noise` is sampled per point (passed in by simulator).
- **Dependencies:** PyYAML (`pip install pyyaml`). Add to README requirements.
- **Owner:** ArchDev.

### 6.2 Driver YAML schema + example files
- **What it does:** Defines the v1 schema and ships two examples.
- **Touchpoints:** new `drivers/pro.yaml`, `drivers/amateur.yaml`.
- **Schema:**
  ```yaml
  name: Pro              # string, free-form
  skill_pct: 0.95        # 0.0 .. 1.0, required
  consistency_sigma: 0.0 # seconds (target), optional, default 0.0
  ```
- **Example values:**
  - `pro.yaml` — `skill_pct: 0.97`, `consistency_sigma: 0.1`
  - `amateur.yaml` — `skill_pct: 0.82`, `consistency_sigma: 0.6`
- **Owner:** ArchDev.

### 6.3 Track CSV loader (`src/lap_estimator/track.py` additions)
- **What it does:** Parses the rich per-point CSV into a structure usable by the simulator.
- **Touchpoints:** `src/lap_estimator/track.py`.
- **Approach:** Add `class TrackCSV` (or extend `Track`) with:
  - `from_csv(path) -> Track` — reads columns `distance_m`, `segment_length_m`, `radius_m`, `gradient_pct`, `elevation_m`, `speed_ms`. Stores them as numpy arrays.
  - `to_points(ds)` — for CSV-backed tracks, **resamples** the existing distance array to a uniform `ds` grid (np.interp on `radius_m`). For segment-backed tracks, keeps current behaviour.
  - `to_ai_reference(ds)` — same resampling for `speed_ms`. Returns `None` for segment-backed tracks.
  - `total_length_m` — exposed property (max of `distance_m`), needed by `sim_telemetry.py` for `normalizedCarPosition`.
  - `name` is inferred from the file stem (e.g. `layout_gp_a_ideal_line`).
- **Ideal-line fallback rule:** Driver passes a path directly, so no auto-fallback at this layer — fallback is handled in `lap.py` only if user passes a directory or a layout file and we detect an `_ideal_line.csv` sibling.
- **Dependencies:** none beyond numpy.
- **Owner:** ArchDev.

### 6.4 Simulator changes (`src/lap_estimator/simulator.py`)
- **What it does:** Accepts a driver; supports a single deterministic run and a Monte-Carlo mode for consistency.
- **Touchpoints:** `src/lap_estimator/simulator.py`.
- **Changes:**
  - `simulate(car, track, driver=None, ds=2.0, rng=None) -> SimResult` — driver is optional. If `None`, behaves as today (equivalent to `skill_pct=1.0, sigma=0`).
  - New `simulate_monte_carlo(car, track, driver, ds=2.0, n_runs=20, seed=0) -> MCResult` — runs N sims with per-point grip noise, returns lap-time mean/std plus one representative deterministic trace (skill-only, no noise) for plotting.
  - `SimResult` gains `times` (per-point cumulative time array), optional `ai_speeds` (m/s), and `limit_label` (per-point string in `{"corner","accel","brake"}` — the active binding pass at that point; used by §14).
  - Grip scaling lives entirely in the driver-wrapped car; the 3-pass core is unchanged. The label is derived during the 3-pass merge by checking which of `v_corner`, `v_forward`, `v_brake` is binding at each index.
- **Dependencies:** 6.1, 6.3.
- **Owner:** ArchDev.

### 6.5 Output artifacts (new file: `src/lap_estimator/report.py`)
- **What it does:** Writes trace CSV and comparison PNG; keeps stdout reporting.
- **Touchpoints:** new `src/lap_estimator/report.py`; move `print_report` from `simulator.py` to here.
- **Files written, adjacent to the input track CSV** (or cwd if track is built-in):
  - `<track_stem>_sim_trace.csv` with columns: `distance_m, sim_speed_ms, sim_speed_kmh, ai_speed_kmh, time_s`. `ai_speed_kmh` is empty when no AI reference.
  - `<track_stem>_sim_vs_ai.png` — matplotlib line plot: x=`distance_m`, y=km/h, two lines (`sim`, `ai`). Skip the plot if matplotlib is not installed; warn on stdout.
  - If driver name is set, include it in the file stem: `<track_stem>__<driver_name>_sim_trace.csv` so multiple driver runs don't overwrite each other.
- **Reusable helper:** expose `report.plot_speed_overlay(distances, series_dict, title, output_path)` so `lap.py --validate-against` (§15) can call it without copy-pasting matplotlib code. `series_dict` maps label → speed-in-km/h array. Existing `write_comparison_plot` becomes a thin wrapper over this helper.
- **Dependencies:** matplotlib (optional, already listed).
- **Owner:** ArchDev.

### 6.6 CLI rewiring (`lap.py`)
- **What it does:** Single sim CLI; new positional args, dispatch logic, and the optional cross-track validation flag.
- **Touchpoints:** new `lap.py` at the repo root (rename of the existing `main.py`). No separate `validate_lap.py` script exists.
- **CLI:**
  ```
  python lap.py <car_data_dir> <track> <driver_yaml> \
                [--ds 2.0] [--all-tracks] \
                [--no-plot] [--no-telemetry] [--telemetry-dt-ms 100] \
                [--validate-against <real_telemetry_csv>] [--bin-m 100] [--per-corner]
  ```
  - `<track>`: track CSV path (preferred), built-in name (`monza|spa|nurburgring|brands_hatch`), or legacy JSON path. Resolution order: file-exists-and-ends-`.csv` → CSV; file-exists-and-ends-`.json` → JSON; name in `BUILTIN_TRACKS` → built-in; else error.
  - `--all-tracks`: kept; runs against all built-in tracks (skips CSV/JSON path). Cheap to keep.
  - `--no-plot`: skip PNG generation (useful in CI / Quix Cloud headless). Also skips the validation overlay PNG when `--validate-against` is set.
  - `--no-telemetry`: skip synthetic telemetry CSV emission (§14). Default off (telemetry is emitted by default).
  - `--telemetry-dt-ms`: cadence in milliseconds for the synthetic telemetry CSV. Default 100.
  - `--validate-against <real_telemetry_csv>`: opt-in cross-track validation (§15). When present, after the normal sim outputs are emitted, `lap.py` calls `validate.validate_lap(...)` to compare the sim against `<real_telemetry_csv>`, prints the real/predicted/delta block + verdict, and writes the overlay PNG (unless `--no-plot`) and per-bin delta CSV next to the track CSV.
  - `--bin-m <int>`: per-distance-bin width in metres for the validation delta table. Default 100. Only meaningful with `--validate-against`. Ignored otherwise.
  - `--per-corner`: switch the validation delta table to corner-based bins (overrides `--bin-m`). Only meaningful with `--validate-against`. Ignored otherwise.
- **Owner:** ArchDev.

### 6.7 README / docs touch-up
- **What it does:** Reflect the new repo layout, the four CLIs (`prep/prep_car.py`, `prep/prep_track.py`, `analysis/corner_analysis.py`, plus `lap.py` / `fit_driver.py`), driver YAML schema, output artifacts (trace + plot + synthetic telemetry), the cross-track validation workflow as `lap.py --validate-against` (§15), and the corner notation JSON schema (§17.4).
- **Touchpoints:** `README.md`, `docs/AI_CONTEXT.md`.
- **Owner:** DocuGuy (after ArchDev lands code).

### 6.8 Package skeleton and module moves (reorg)
- **What it does:** Establishes the `src/lap_estimator/` package and physically relocates simulator library modules, low-level decoders, and the analysis script.
- **Touchpoints (moves):**
  - `car.py` → `src/lap_estimator/car.py`
  - `track.py` → `src/lap_estimator/track.py`
  - `simulator.py` → `src/lap_estimator/simulator.py`
  - `decode_acd.py` → `prep/decode_acd.py`
  - `decode_track.py` → `prep/decode_track.py`
  - `Corner_Analysis/corner_analysis.py` → `analysis/corner_analysis.py` (rewritten, telemetry-free; the original `Corner_Analysis/` folder is not deleted by ArchDev — user-managed scratch).
- **New files:** `src/lap_estimator/__init__.py`, `src/lap_estimator/driver.py`, `src/lap_estimator/driver_fit.py`, `src/lap_estimator/telemetry.py`, `src/lap_estimator/sim_telemetry.py`, `src/lap_estimator/report.py`, `src/lap_estimator/validate.py`, `prep/__init__.py`, `prep/prep_car.py`, `prep/prep_track.py`, `analysis/__init__.py`, `lap.py` (root, replaces `main.py`), `fit_driver.py` (root).
- **Import discipline:** root CLIs import from `lap_estimator` (e.g. `from lap_estimator.simulator import simulate`). `prep/` scripts are standalone and do **not** import from `lap_estimator` (they should be runnable even if the simulator deps are unhappy). `analysis/corner_analysis.py` may import `track.Track` for CSV loading but must not pull in the simulator chain.
- **Path bootstrapping:** the root CLIs and the `prep/` / `analysis/` scripts can run without `pip install -e .` by inserting `<repo>/src` onto `sys.path` at the top of each script (single line). Editable install is the preferred long-term path but is not a requirement for v1.
- **Owner:** ArchDev.

### 6.9 Folder convention and `.gitignore` policy (reorg)
- **What it does:** Locks in the repository layout and what is / isn't tracked.
- **Tracked-in-repo:**
  - `cars_csv/<car>/data/*.ini`, `*.lut` — outputs of `prep_car.py`.
  - `tracks_csv/<track>/layout_*.csv`, `tracks_csv/<track>/layout_*_ideal_line.csv` — outputs of `prep_track.py`.
  - `tracks_csv/<track>/layout_*_corners.json` — output of `analysis/corner_analysis.py`.
  - `tracks_csv/<track>/*.png` — visualisations from `analysis/corner_analysis.py` (overview, labeled, corner map, speed-vs-position).
  - `drivers/*.yaml` — example drivers (pro, amateur) and any fitted drivers the user wants to keep.
  - `tracks_config.json` at repo root — corner-classification thresholds and colours.
  - `samples/aclog/*.csv` — at least one committed AC telemetry log so acceptance criteria are reproducible (see "sample data" below).
  - `prep/`, `analysis/`, `src/lap_estimator/`, `lap.py`, `fit_driver.py`, `docs/`, `dev-planning/`.
- **Gitignored:**
  - `cars_in/*` and `tracks_in/*` (raw AC content — large, licensed, user-supplied). Already in `.gitignore`; keep `.gitkeep` markers so the folders exist post-clone.
  - `.tmp/` — scratch.
- **Sample data move:** the sample AC telemetry log currently at `Corner_Analysis/AClog/2026-04-07T135548_260Z_Tms_Lap2.csv` moves (or is copied) to `samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv` and is committed to the repo. All spec references that previously pointed to `Corner_Analysis/AClog/...` are updated to `samples/aclog/...`. The existing `Corner_Analysis/` folder remains user-managed scratch — ArchDev does not commit or delete it.
- **Owner:** ArchDev.

## 7. Data & interface contracts

### 7.1 Track CSV (input)
Required columns (others ignored): `distance_m`, `segment_length_m`, `radius_m`, `gradient_pct`, `elevation_m`, `speed_ms`. Rows assumed in distance order. `radius_m` values >= ~2000 treated as straight (existing convention). The full column set produced by `prep_track.py` and consumed by `corner_analysis.py` is documented in `README.md` and §16.4.

### 7.2 Driver YAML (input)
```yaml
name: string                  # required, used in file naming
skill_pct: float in [0,1]     # required; multiplier on tyre grip
consistency_sigma: float      # optional, seconds; >0 enables Monte-Carlo
source:                       # optional, populated by fit_driver.py
  telemetry_csv: string       # absolute or repo-relative path
  track_csv: string
  car_data_dir: string
  real_lap_time_s: float
  sim_lap_time_s: float
  delta_s: float
  fitted_at: ISO-8601 string
  fit_version: "1"
```
Validation: `0 < skill_pct <= 1`, `consistency_sigma >= 0`. Hard error on parse failure. `source` block is informational; the loader passes unknown sub-keys through unchanged.

### 7.3 Car data dir (input)
Unchanged. Either a directory containing `engine.ini` directly, or a parent with a `data/` subdirectory.

### 7.4 Trace CSV (output)
Columns, header row required:
```
distance_m, sim_speed_ms, sim_speed_kmh, ai_speed_kmh, time_s
```
`ai_speed_kmh` is empty for built-in / JSON tracks.

### 7.5 Comparison plot (output)
Single matplotlib figure, 1 axes, x-axis = `distance_m`, y-axis = `speed_kmh`, two labelled lines (`sim`, `ai`). Title = `<track_name> — <driver_name>`. Lap time annotated as a sub-title.

### 7.6 Stdout report
Same fields as today (Lap Time, Max/Min/Avg Speed, segment table for built-in tracks, 0-100/0-200/top speed). When CSV-backed and Monte-Carlo is enabled, replace "Lap Time" line with `Lap Time: <mean> ± <σ> (N=<n_runs>)`. Otherwise byte-compatible with today's output.

### 7.7 Synthetic telemetry CSV (output, new — see §14 for the algorithm)
Columns and column order exactly match the real AC log:
```
timestamp_ms,gas,brake,distanceTraveled,speedKmh,normalizedCarPosition
```
File path: `<track_dir>/<track_stem>__<driver_name>_sim_telemetry.csv` (or cwd for non-CSV tracks). Same name convention as the trace file.

### 7.8 Corner notation JSON (output of `analysis/corner_analysis.py`)
See §17.4 for the full schema. Path: `<track_dir>/<layout_stem>_corners.json`.

### 7.9 `tracks_config.json` (in-repo config, input to `analysis/corner_analysis.py`)
Lives at the repo root. Current fields (kept as-is — schema is informally frozen for v1):
```json
{
  "_comment": "Corner classification thresholds (meters) and colors used for track map and plot overlays.",
  "corner_thresholds": {
    "hairpin_max": 60,         // R < 60 m  -> hairpin
    "tight_max":  150,         // 60 <= R < 150 -> tight
    "sweeper_max": 400         // 150 <= R < 400 -> sweeper; R >= 400 -> straight
  },
  "colors": {
    "hairpin": "#f87171",
    "tight":   "#fb923c",
    "sweeper": "#fbbf24",
    "straight": "#34d399",
    "start_finish": "#ffffff",
    "marker":  "#fff8e1",
    "track_dot": "#ef4444"
  },
  "corner_labels": {
    "min_length_m": 20         // corners shorter than this are not labelled on the map
  },
  "default_track": "tracks_csv/ks_nurburgring/layout_sprint_a.csv"
}
```
Notes:
- The `default_track` field is convenience only — used when `corner_analysis.py` is invoked with no positional argument. **The path was updated from the legacy `tracks/...` prefix to `tracks_csv/...`** to match the locked folder convention (§6.9). ArchDev rewrites this field as part of the reorg commit.
- The thresholds in `corner_thresholds` are the same numbers currently hard-coded in `Corner_Analysis/corner_analysis.py` (`< 60`, `< 150`, `< 400`, `>= 400`). Centralising them here is the whole point of the config file.
- New fields may be added in v2 (e.g. per-track overrides). Loader must tolerate unknown keys.

## 8. Risks, constraints, and open questions

### Risks
- **CSV resampling vs current segment-resampling:** `to_points(ds)` for CSV tracks must interpolate `radius_m` not just replicate it; a naïve nearest-neighbour can spike at the resample boundaries. Mitigation: use `np.interp` on `1/radius_m` (curvature) so straight segments with `radius_m=2000` blend correctly through corner entries.
- **`skill_pct` as a uniform grip multiplier may be too punishing in slow corners**, since cornering speed scales with `sqrt(mu)`. A `skill_pct=0.5` halves grip → lap time slows by ~40% — that's a feature, not a bug, for v1, but worth noting before users start tuning.
- **`consistency_sigma` calibration:** the user specified it in seconds, but the noise is applied in grip-units per point. Mapping is heuristic in v1 (e.g. `sigma_grip = clip(consistency_sigma * 0.03, 0, 0.1)`). ArchDev should expose the conversion in the driver module so it's tweakable. Note it as approximate in CLI output.
- **PyYAML dependency:** new mandatory dep. Acceptable but add to README.
- **Synthetic telemetry realism:** the gas/brake reconstruction in §14 is heuristic — it's piecewise (0 or 1) on accel/brake-bound segments and partial-throttle on corner-bound segments. It will not look like a smooth human input trace. Documented as v1; v2 hook for a low-pass driver-input filter.
- **Driver portability across tracks (v1 limitation).** v1 treats `skill_pct` and `consistency_sigma` as track-agnostic properties of the driver. In practice a driver's effective skill on a brand-new track is **lower** than on a learned one — the "10-lap learning curve". A fit done on a familiar track will likely **overestimate** the driver's skill on a fresh track, producing optimistic predictions in §15. v1 documents this; v2 candidate is a per-track `familiarity` modifier or a `confidence` term that decays with track novelty.
- **(reorg) Track-preparation parity with the existing CSVs.** The committed `tracks_csv/ks_nurburgring/layout_*.csv` files were produced by some prior (partly manual / partly in `Corner_Analysis/`) pipeline that is not currently a single script. `prep_track.py` must reproduce that column set faithfully. Mitigation: ArchDev diffs the output of `prep_track.py` against the committed Nurburgring CSVs (same input AC folder) before accepting the script. Acceptance criterion §11.11 enforces this.
- **(reorg) Import-path fragility.** Moving simulator code under `src/lap_estimator/` without a `pip install -e .` setup means every CLI / script needs a one-line sys.path bootstrap. Risk of someone forgetting the bootstrap in a new script. Mitigation: standardise the bootstrap in a small `_bootstrap.py` shim or document it in `docs/AI_CONTEXT.md` as the required first line of any new entry-point script.

### Constraints
- One feature one branch (already on `feature/sc-71955/lap-simulation`).
- No file over ~500 lines. Estimated post-change sizes: `src/lap_estimator/car.py` ~256 (unchanged), `track.py` ~280 (loader added), `simulator.py` ~200, `driver.py` ~80, `report.py` ~170, `lap.py` ~150, `telemetry.py` ~140, `driver_fit.py` ~180, `fit_driver.py` ~60, `sim_telemetry.py` ~120, `validate.py` ~160, `prep/decode_acd.py` (unchanged from current root version), `prep/decode_track.py` (unchanged), `prep/prep_car.py` ~80, `prep/prep_track.py` ~200, `analysis/corner_analysis.py` ~350 (rewrite — was ~480 with telemetry; drops ~130 lines by removing the merged-with-telemetry path and the legacy `track_points.csv`/`track_meta.csv` base64 exports). All comfortably under the ceiling.

### Open questions
- Monte-Carlo `n_runs` default — proposed 20. User can override later via CLI flag (not in v1).
- Where exactly to place driver YAMLs — proposed `drivers/` at repo root. Locked unless user objects.
- Should `--all-tracks` also iterate drivers? Proposed: no, driver is single per invocation.
- Future v2 driver params (line deviation, brake/throttle split, reaction time) — schema should be forward-compatible (YAML parser ignores unknown keys with a warning).
- Per-track familiarity modelling — v2 candidate.
- Cross-track validation tolerance numbers — initial thresholds in §15 (|delta| < 3 s is good, >10% is bad) are guesses. Once 3+ validation runs exist, tighten them.
- **(reorg) `pip install -e .` or `sys.path` shim?** v1 picks the `sys.path` shim for zero-config runnability; long-term `pyproject.toml` + editable install is preferred. Not blocking v1.
- **(reorg) Should `prep_track.py` also auto-invoke `analysis/corner_analysis.py`?** v1 says no — they are two steps. The user can `&&` them on the command line. Auto-chaining is a convenience-only candidate for v2 (e.g. a `prep_all.py` umbrella script).

## 9. Alternatives considered

- **Driver model as full G-G-V-with-aggression-knobs.** Closer to the `ModelCreationSteps.txt` ideal, but heavy for v1 and obscures the input/output plumbing work. Deferred to v2.
- **Driver as a `skill_pct` knob baked into `Car` directly.** Simpler but pollutes `Car` with driving-skill concepts and makes it harder to add per-corner driver behaviour later. Rejected.
- **Convert CSV → JSON segments on the fly and reuse the existing pipeline.** Loses AI speed reference and gradient/elevation info. Rejected.
- **Drop the legacy built-in tracks entirely.** Cheaper to leave them; users already wire them. Keep as a fallback path with a soft-deprecation note in the README.
- **Skip the comparison PNG (CSV trace only).** PNG is the fastest QA signal; cheap to add. Kept.
- **Emit synthetic telemetry by sampling at uniform distance steps (no time resampling).** Simpler, but breaks the "drop-in real-AC-log" property — real AC logs are time-sampled at ~50 Hz. Rejected.
- **Three CLIs (`main.py` sim, `fit_driver.py` fit, `validate_lap.py` validate).** Collapsed into two CLIs on user feedback — validation became a `--validate-against` flag on `lap.py`.
- **Bake a per-track familiarity term into v1.** Premature: we have no calibration data for the learning curve yet. Rejected for v1.
- **(reorg) Flat top-level layout (no `src/lap_estimator/` package).** Considered — for a ~10-file prototype, a flat layout is defensible and simpler. Rejected because (a) several simulator modules now exist (`telemetry`, `sim_telemetry`, `driver_fit`, `validate`, `report`, `driver`) so the top-level was getting cluttered, and (b) a package boundary cleanly distinguishes "library code" (importable, reusable) from "scripts" (`prep/`, `analysis/`, root CLIs). If ArchDev finds the `sys.path` bootstrap too painful, falling back to a flat `src/` (no nested package) is acceptable — but the prep/analysis separation stays.
- **(reorg) Have `prep_track.py` also produce `<layout>_corners.json`.** Tempting (one command, one result) but couples geometry extraction to corner classification. If `tracks_config.json` changes, every track has to be re-prepped, not just re-classified. Keeping them separate makes corner-threshold tuning a fast inner loop. Rejected.
- **(reorg) Keep `Corner_Analysis/` as the canonical corner-analysis location.** Rejected — it's untracked, ambiguously named (overlaps with `corner_analysis.py`), and depends on a merged-with-telemetry CSV. Replaced by `analysis/corner_analysis.py`.

## 10. Migration

- The current `segments`-based `Track` (built-ins + `from_json`) is **retained as a legacy code path**. No removal in this feature.
- Add a one-line deprecation note at the top of `track.py`'s `BUILTIN_TRACKS` dict pointing users at CSV-backed tracks as the preferred input.
- The third positional CLI argument is **required** for `lap.py`. Old invocations like `python main.py cars_csv/bmw_1m monza` will fail with a clear error pointing at the new CLI.
- **Script rename:** the existing `main.py` is renamed to `lap.py` at the repo root.
- **(reorg) File moves** (ArchDev does these in a single commit, as part of the reorg):
  - `car.py`, `track.py`, `simulator.py` → `src/lap_estimator/`
  - `decode_acd.py`, `decode_track.py` → `prep/`
  - `main.py` → `lap.py` (rewritten per §6.6)
  - `Corner_Analysis/AClog/2026-04-07T135548_260Z_Tms_Lap2.csv` → `samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv` (copy and `git add`; do not delete the original — it's in untracked scratch).
  - `Corner_Analysis/corner_analysis.py` → `analysis/corner_analysis.py` (rewrite per §17, do not move the original; leave the user's scratch alone).
- **(reorg) `tracks_config.json`** is `git add`-ed (currently untracked per `git status`) with the `default_track` path updated to `tracks_csv/ks_nurburgring/layout_sprint_a.csv`.
- **(reorg) Existing CSVs under `tracks_csv/ks_nurburgring/`** remain in place. They are treated as canonical reference outputs of `prep_track.py` for the parity check in §11.11.

## 11. Acceptance criteria (for manual QA in Quix Cloud)

1. Running `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_gp_a_ideal_line.csv drivers/pro.yaml` succeeds, prints a lap time, writes `tracks_csv/ks_nurburgring/layout_gp_a_ideal_line__Pro_sim_trace.csv`, `..._sim_vs_ai.png`, and `..._sim_telemetry.csv`.
2. The comparison plot shows the sim speed line generally tracking the AI speed line (within ~10–20% on most of the lap) for a reasonably matched car. Large deviations are tolerable; the point is visual sanity.
3. Replacing `drivers/pro.yaml` with `drivers/amateur.yaml` gives a slower lap time and a visibly lower sim-speed line at corner apexes.
4. Setting `consistency_sigma: 0.5` in the driver YAML produces stdout in the form `Lap Time: 2:17.401 ± 0.487 (N=20)`.
5. Running with a `layout_*.csv` (no `_ideal_line`) succeeds; AI-speed line in the plot still comes from that file's `speed_ms` column.
6. Legacy invocation `python lap.py cars_csv/bmw_1m monza drivers/pro.yaml` still runs against the built-in Monza segments and prints a lap time (no plot generated, or plot with only the sim line). Synthetic telemetry CSV is still emitted (using the segment-derived total length).
7. `python lap.py ... --no-plot` runs without matplotlib installed.
8. `python lap.py ... --no-telemetry` skips the synthetic telemetry CSV; everything else still runs.
9. Driver YAML missing `skill_pct`, or with `skill_pct > 1.0`, fails fast with a clear error.
10. **Cross-track validation runs end-to-end as a `lap.py` flag.** `python lap.py cars_csv/bmw_1m tracks_csv/<track_b>/layout_*.csv drivers/<fitted_on_track_a>.yaml --validate-against <real_lap_on_track_b>.csv` succeeds, prints the real/predicted/delta block + verdict, writes the overlay PNG and per-bin delta CSV alongside the normal sim outputs.
11. **(reorg) `prep_track.py` parity check.** Running `python prep/prep_track.py tracks_in/ks_nurburgring` (after the user copies the AC folder into `tracks_in/`) produces `tracks_csv/ks_nurburgring/layout_sprint_a.csv` whose columns and row count match the committed reference file within a tight tolerance (column set identical; numeric columns equal to ≤1e-3 relative error per cell, ignoring trailing-row alignment). Equivalent check for `layout_sprint_a_ideal_line.csv`.
12. **(reorg) `prep_car.py` round-trip.** Running `python prep/prep_car.py cars_in/bmw_1m` (after the user copies the AC car folder in) produces `cars_csv/bmw_1m/data/` with the same `.ini` and `.lut` files as the existing reference. `lap.py` running on the freshly-produced `cars_csv/bmw_1m` matches the lap time from running on the pre-existing committed `cars_csv/bmw_1m` within numerical noise.
13. **(reorg) `corner_analysis.py` end-to-end.** Running `python analysis/corner_analysis.py tracks_csv/ks_nurburgring/layout_sprint_a.csv` succeeds, writes `tracks_csv/ks_nurburgring/layout_sprint_a_corners.json` (schema per §17.4), and emits `..._corner_map.png` and `..._speed_vs_position.png` next to the CSV. The JSON contains at least the corners visible in the existing `Corner_Analysis/track_corner_map.png` (manual visual check is acceptable), and every entry has `type` in `{hairpin, tight, sweeper, straight}`, `direction` in `{left, right, straight}`, and a sensible `min_radius_m`.
14. **(reorg) Corner analysis is telemetry-free.** `analysis/corner_analysis.py` runs with **no** AC telemetry file present (e.g. on a fresh clone, before any AC log is dropped in). No file in `samples/` or `Corner_Analysis/` is read.
15. **(reorg) `tracks_config.json` is the single source of truth for corner classification.** Changing `corner_thresholds.hairpin_max` from 60 to 40 in the config and re-running `corner_analysis.py` changes which corners get labelled `hairpin` vs `tight` in the output JSON. No corresponding edits to Python source are required.

## 12. References

- `docs/AI_CONTEXT.md` — Project structure, CSV column reference, physics summary.
- `README.md` — Current CLI and example output.
- `src/lap_estimator/simulator.py` — 3-pass simulator (kept).
- `src/lap_estimator/car.py` — Car physics (untouched).
- `src/lap_estimator/track.py` — Current segment-based track model (extended).
- `ModelCreationSteps.txt` — Long-term aspiration. Informs v2 roadmap.
- `RequirementsForAbnormalityAnalysis.txt` — Future requirements-checking hook; not addressed here.
- `Corner_Analysis/corner_analysis.py` — Source script that `analysis/corner_analysis.py` is rewritten from (telemetry-free) and that `src/lap_estimator/telemetry.py` lifts the merge logic from.
- `samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv` — Committed sample AC telemetry log; reference input for `fit_driver.py` and schema reference for §14 sim-telemetry emission. (Originally at `Corner_Analysis/AClog/…`; moved per §6.9 / §10.)
- `tracks_config.json` — Corner-classification thresholds and colours (§7.9).

---

## 13. Telemetry-driven driver fitting (addendum, v1)

### 13.1 Goal

Given a real AC telemetry CSV for a single lap on a known car/track, derive `skill_pct` and `consistency_sigma` so that the simulator's lap time (with that car + track + derived driver) lands close to the real lap time. The fit is **per-lap, per-car, per-track**; we are not building a portable "driver personality" object yet.

**Important — pick a learned lap, not lap 1.** Drive at least ~10 laps in AC on the source track before capturing the telemetry lap that gets fed to `fit_driver.py`. Pick a clean lap from after lap times have stabilised (typically the fastest of laps 8-15, or the median of the best 3). Lap 1 is almost always wrong. This is guidance, not enforced. (v2 candidate: `fit_driver.py` accepts a directory of laps and picks the fastest / median-of-best-3 automatically.)

### 13.2 Non-goals (fit tool)

- Fitting tyre / aero / engine parameters from telemetry — the AC car model is treated as ground truth.
- Per-corner skill profile (v2; see §13.9).
- Brake-vs-cornering skill split (v2).
- Combining multiple laps into one averaged driver (v2).
- Detecting / discarding off-track or invalid laps (out of scope).
- Live telemetry / streaming (file-based only).

### 13.3 CLI shape

```
python fit_driver.py <car_data_dir> <track_csv> <ac_telemetry_csv> <output_driver_yaml> \
                     [--ds 2.0] [--name <driver_name>] [--no-validate] [--no-plot]
```

- `<car_data_dir>` — same shape `lap.py` accepts (directory containing `engine.ini` or a parent with `data/`).
- `<track_csv>` — full track CSV (centerline or ideal line). The fit uses `distance_m` + `radius_m` from this file.
- `<ac_telemetry_csv>` — raw AC log with columns `timestamp_ms, gas, brake, distanceTraveled, speedKmh, normalizedCarPosition` (others ignored). **The same parser is used by `fit_driver.py` whether the file came from real AC telemetry or from the §14 synthetic emitter** — this is what enables the loop-closure check.
- `<output_driver_yaml>` — destination path; directories are created as needed.
- `--name` — overrides the auto-generated `name` field in the YAML. Default: derived from telemetry filename stem.
- `--no-validate` — skip the post-fit sim validation pass.
- `--no-plot` — skip plotting (passed through to the validation sim).

### 13.4 Input contract

**AC telemetry CSV** (confirmed columns from `samples/aclog/...`):

| Column | Unit | Used for |
|---|---|---|
| `timestamp_ms` | ms | Real lap time = `max - min` / 1000 |
| `gas` | 0..1 | Reserved (v2 brake/throttle split) |
| `brake` | 0..1 | Reserved (v2 brake/throttle split) |
| `distanceTraveled` | m | Merge key against track CSV `distance_m` |
| `speedKmh` | km/h | Converted to m/s for `v` |
| `normalizedCarPosition` | 0..1 | Sanity / corner labelling (not load-bearing) |

**Track CSV:** same contract as §7.1.

**Car dir:** same contract as §7.3.

### 13.5 Algorithm

All steps live in `driver_fit.fit_driver(car, track_df, telemetry_df) -> FitResult`. The CLI is a thin wrapper that loads inputs, calls this function, writes the YAML, and (unless `--no-validate`) runs the validation sim.

**Step 1 — Merge telemetry with track on distance.**
- Use `telemetry.merge_with_track(telemetry_df, track_df)` (the extracted helper from `Corner_Analysis/corner_analysis.py`).
- For each telemetry sample, look up the nearest track point by `distanceTraveled` ≈ `distance_m` (np.searchsorted + linear interp on `radius_m` and `gradient_pct`).
- Output: merged dataframe with `distance_m, speed_ms (=speedKmh/3.6), radius_m, gradient_pct, gas, brake, timestamp_ms`.

**Step 2 — Compute per-point observed lateral G.**
- `lat_g_obs[i] = v[i]**2 / radius_m[i] / 9.81` (in units of g).
- Points where `radius_m[i] >= STRAIGHT_THRESHOLD_M` (default **500 m**, matches `corner_analysis.py`) are flagged as straight and **excluded** from the skill fit.

**Step 3 — Compute the car's theoretical max lateral G at that speed.**
- `lat_g_max[i] = car.tyre_grip_lateral(v[i]) * (1 + downforce(v[i]) / (m*g))`.

**Step 4 — Per-point grip utilisation.**
- `util[i] = lat_g_obs[i] / lat_g_max[i]`.
- Clip to `[0.0, 1.2]`. Values > 1.0 indicate either the car model under-grips or the driver is using kerbs / runoff. Count them and emit a warning if more than 5% of cornering samples exceed 1.0.

**Step 5 — Aggregate into `skill_pct`.**
- **Decision (v1):** use the **85th percentile** of `util` across cornering samples, then clip to `(0, 1.0]`.
- `skill_pct = clip(percentile(util_cornering, 85), 0.05, 1.0)`.

**Step 6 — Aggregate into `consistency_sigma`.**
- `consistency_sigma_raw = stdev(util_cornering)`.
- Map to the YAML's "seconds" field: `consistency_sigma_seconds = round(consistency_sigma_raw / 0.03, 2)`.
- Clamp to `[0.0, 1.5]`.

**Step 7 — Worked example (illustrative):**
- Real lap time ≈ 1:42 s. Hairpin: 60 m radius, 75 km/h → `lat_g_obs ≈ 0.74 g`. `lat_g_max ≈ 1.05 g`. `util ≈ 0.70`. 85th percentile → `skill_pct ≈ 0.88`.

**Step 8 — Emit YAML.**
```yaml
name: <driver_name>
skill_pct: 0.88
consistency_sigma: 0.45
source:
  telemetry_csv: samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv
  track_csv: tracks_csv/ks_nurburgring/layout_sprint_a.csv
  car_data_dir: cars_csv/bmw_1m
  real_lap_time_s: 102.135
  sim_lap_time_s: null     # populated after validation, if run
  delta_s: null
  fitted_at: 2026-05-13T14:22:01Z
  fit_version: "1"
```

**Step 9 — Validation pass (default on).**
- Run a single-lap sim with the freshly-emitted YAML and print:
  ```
  Real lap: 1:42.135
  Sim lap:  1:43.402
  Delta:    +1.267 s  (+1.24%)
  ```
- Patch `source.sim_lap_time_s` and `source.delta_s`. If `|delta| > 3 s`, warn.

### 13.6 Output contract (YAML schema additions)

- `source` — optional object. Keys: `telemetry_csv`, `track_csv`, `car_data_dir`, `real_lap_time_s`, `sim_lap_time_s`, `delta_s`, `fitted_at`, `fit_version`. Hand-authored YAMLs leave it absent.
- **v2 hook:** `corners` — optional list of `{corner_id, skill_pct, consistency_sigma}` overrides. Reserved key.

### 13.7 Module changes

- **New:** `src/lap_estimator/telemetry.py` — owns:
  - `read_ac_log(path)` — parses the AC CSV.
  - `merge_with_track(telem, track_csv_path_or_track) -> merged_frame` — lifts the CSV+log merge logic.
- **New:** `src/lap_estimator/driver_fit.py` — owns `fit_driver(car, merged_frame, *, straight_threshold_m=500.0, util_percentile=85) -> FitResult`.
- **New:** `fit_driver.py` (CLI at repo root) — argparse, loads inputs, calls the library, writes YAML, runs validation sim, patches YAML.
- **Touched:** `src/lap_estimator/driver.py` — `Driver.load` tolerates the `source` block.
- **Untouched:** `car.py`, `simulator.py`, `report.py`.

### 13.8 `Corner_Analysis/` handling

The user's `Corner_Analysis/` folder is untracked scratch. The fit tool must **not** depend on it.

- ArchDev extracts the merge logic from `Corner_Analysis/corner_analysis.py` into `src/lap_estimator/telemetry.py`. The original file stays as-is — user scratch.
- Sample telemetry moves to `samples/aclog/*.csv` and is committed (§6.9 / §10).

### 13.9 Open questions / v2 candidates

- Per-corner skill profile, brake-vs-cornering split, multi-lap averaging, lap-to-lap consistency, percentile choice, skill ceiling clipping. (All v2.)

### 13.10 Acceptance criteria (fit tool)

1. Running `python fit_driver.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv drivers/ludvik_nurburgring_sprint.yaml` succeeds, writes a YAML that loads cleanly via `Driver.load`, prints the real-vs-sim delta block.
2. The YAML's `skill_pct` is in `(0, 1]` and `consistency_sigma` is in `[0, 1.5]`. `source` is fully populated.
3. Running `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/ludvik_nurburgring_sprint.yaml` produces the same sim lap time recorded in `source.sim_lap_time_s` (within numerical noise).
4. Validation delta `|sim - real|` < 5 s on the bundled Nurburgring Sprint lap.
5. `--no-validate` skips the sim and leaves `source.sim_lap_time_s` / `source.delta_s` as `null`.
6. Missing required telemetry column fails fast.
7. Non-overlapping distance ranges produce a clear error.
8. The fit tool does **not** read or write inside `Corner_Analysis/`.

---

## 14. Synthetic telemetry emission (addendum, v1)

### 14.1 Goal

After every sim run, emit a CSV that is **schema-identical to a real AC telemetry log** so downstream tools — first and foremost `fit_driver.py` — can consume sim output the same way they consume real laps.

### 14.2 Non-goals

- Human-realistic input traces. Reconstructed `gas`/`brake` are piecewise.
- Channels not in the AC sample (RPM, gear, steering, suspension, tyre temps).
- Wall-clock-anchored `timestamp_ms`. Sim telemetry starts at 0.
- Re-using sim's exact `ds` cadence. Output is **time-resampled** at fixed cadence (default 100 ms).
- Filtering / smoothing across samples.

### 14.3 Decisions baked in (Decisions block)

1. **Schema:** exact AC match — `timestamp_ms, gas, brake, distanceTraveled, speedKmh, normalizedCarPosition`.
2. **Cadence:** fixed 100 ms by default. CLI flag `--telemetry-dt-ms` overrides.
3. **Time origin:** `timestamp_ms` starts at 0.
4. **`normalizedCarPosition`:** `distance_m / track.total_length_m`, clipped to `[0, 1)`.
5. **Resampling:** linear interpolation in the time domain.
6. **`gas` / `brake` reconstruction:** derived from sim's per-point binding-limit label:
   - `accel` → `gas = 1.0, brake = 0.0`
   - `brake` → `gas = 0.0, brake = 1.0`
   - `corner` → `gas = required_drive_force / car.max_traction_force(v)`, `brake = 0`, clipped to `[0, 1]`.
7. **Emission is default-on.** `--no-telemetry` opts out.
8. **File path:** `<track_dir>/<track_stem>__<driver_name>_sim_telemetry.csv`.
9. **Module placement:** new `src/lap_estimator/sim_telemetry.py`. `simulator.py` is not extended with I/O.
10. **Low-pass filter on inputs:** deferred to v2.
11. **Two CLIs, not three.** Validation collapsed into `lap.py --validate-against`.
12. **Local-file-only prototype.** No DB / remote storage in v1.

### 14.4 CLI surface (additions to `lap.py`)

```
[--no-telemetry] [--telemetry-dt-ms 100]
```

### 14.5 Output schema (matches AC exactly)

```
timestamp_ms,gas,brake,distanceTraveled,speedKmh,normalizedCarPosition
```

### 14.6 Resampling algorithm

See original spec — unchanged. Walks the sim step→time map, builds a uniform output grid, linear-interp on distance and speed, nearest-neighbour on binding label, reconstructs gas/brake per §14.3 item 6.

### 14.7 Module: `src/lap_estimator/sim_telemetry.py`

```python
def write_synthetic_log(
    sim_result,
    car,
    track_total_length_m: float,
    output_path: str,
    *,
    telemetry_dt_ms: int = 100,
) -> None: ...
```

Pure function; no global state; numpy + stdlib `csv` only.

### 14.8 Loop-closure acceptance criterion

Fit a driver → emit sim telemetry → re-fit a driver from the synthetic telemetry. Pass criteria:
- `|skill_pct_loop - skill_pct_real| / skill_pct_real <= 0.05`.
- `|sim_lap_time_loop - sim_lap_time_real| <= 1.0` s.
- `skill_pct_loop` is not trivially 1.0.

### 14.9 Acceptance criteria (sim-telemetry emission)

1. Default `lap.py` produces `<track_stem>__<driver_name>_sim_telemetry.csv` next to the track CSV.
2. Header byte-matches `timestamp_ms,gas,brake,distanceTraveled,speedKmh,normalizedCarPosition`.
3. `timestamp_ms` starts at 0 and increments by exactly `--telemetry-dt-ms`.
4. `distanceTraveled` monotonic non-decreasing, ends within 1 m of `track.total_length_m`.
5. `normalizedCarPosition` stays in `[0, 1)`.
6. `gas` and `brake` never both > 0.
7. Trivial straight: `gas == 1.0, brake == 0.0`.
8. `--no-telemetry` skips the file.
9. `--telemetry-dt-ms 50` ≈ 2× rows; `200` ≈ ½× rows.
10. Loop closure passes on the bundled Nurburgring Sprint lap.

### 14.10 Open questions / v2 candidates

Low-pass filter, steering channel, extra channels, wall-clock timestamps, multi-lap output, cadence calibration. (All v2.)

---

## 15. Cross-track validation workflow (addendum, v1)

### 15.1 Goal — the actual point of the tool

Predict how a known driver (fit on Track A) will perform on a new track (Track B, same car), then validate against a real AC lap on Track B.

The CLI is `lap.py --validate-against` — **not** a separate script.

### 15.2 End-to-end workflow

1. Drive Track A in AC, capture a learned-lap telemetry.
2. `python fit_driver.py <car_dir> <track_a_csv> <real_lap_on_a>.csv drivers/<name>.yaml`.
3. Choose Track B.
4. `python lap.py <car_dir> <track_b_csv> drivers/<name>.yaml` → predicted lap.
5. Drive Track B in AC for ~10 laps, capture telemetry.
6. `python lap.py <car_dir> <track_b_csv> drivers/<name>.yaml --validate-against <real_lap_on_b>.csv`.

### 15.3 CLI shape

```
python lap.py <car_data_dir> <track_csv> <driver_yaml> \
              --validate-against <real_telemetry_csv> \
              [--ds 2.0] [--no-plot] [--bin-m 100] [--per-corner]
```

### 15.4 Algorithm

Lives in `validate.validate_lap(car, track, driver, sim_result, real_telem) -> ValidationResult`. `lap.py` calls it post-sim (the same sim result is reused — no second sim pass). Resample real telemetry onto sim's distance grid; bin by 100 m (or by corner spans if `--per-corner`); compute per-bin and headline deltas; categorise.

### 15.5 Outputs

**Stdout:**
```
Track:     tracks_csv/brands_hatch/layout_indy_ideal_line.csv
Driver:    drivers/ludvik_nurburgring_sprint.yaml  (fit on layout_sprint_a)
Real lap:  1:24.812
Sim lap:   1:26.301  (predicted)
Delta:     +1.489 s  (+1.76%)
Verdict:   GOOD    (|delta| < 3 s on a 84.8 s lap)
```

Verdicts: `GOOD` (|d| < 3 s AND |%| < 5), `LOOSE` (5–10 %), `BAD` (> 10 %).

**Files (next to track CSV):**
- `<track_stem>__<driver_name>_validation_overlay.png`.
- `<track_stem>__<driver_name>_validation_bins.csv`: `bin_start_m, bin_end_m, kind, t_sim_s, t_real_s, delta_s, v_avg_sim_kmh, v_avg_real_kmh`.

### 15.6 Limitations

v1 explicitly assumes `skill_pct` / `consistency_sigma` are track-agnostic. Practical consequence: unlearned Track B → negative delta (sim faster). v2 candidates: per-track familiarity modifier, track-similarity score, automatic learning-curve detection, validation-driven calibration.

### 15.7 Module placement

- **New:** `src/lap_estimator/validate.py` — `validate_lap(car, track, driver, sim_result, real_telem_path_or_frame) -> ValidationResult`. No I/O of its own.
- **No new CLI.** Invoked via `lap.py --validate-against`.
- **Reused:** `telemetry.py`, `simulator.py`, `report.py`.

### 15.8 Acceptance criteria (validate flow)

1. End-to-end flag invocation runs; stdout block + verdict; overlay PNG + bins CSV produced.
2. Verdict matches §15.5 thresholds.
3. Bins CSV well-formed; sums match lap times within 0.1 s.
4. Overlay PNG produced unless `--no-plot`.
5. Real-telemetry parsing reuses `telemetry.py`.
6. `--validate-against` is read-only on its inputs.
7. Without the flag, no validation outputs.
8. **Soft acceptance** — on the user's actual Nurburgring-fit → other-track run, `|delta_s|` < ~3 s on a 2–3 min lap.

### 15.9 Open questions

"Representative lap" definition; reverse-fit a Track B driver as a debugging signal; threshold tuning. (All v2.)

---

## 16. Preparation pipeline (addendum, v1 reorg)

### 16.1 Goal

Establish a single, scripted path from raw AC content under `cars_in/` and `tracks_in/` to repo-friendly CSV artefacts under `cars_csv/` and `tracks_csv/`. Replace the current scattered state (root-level decoders + untracked `Corner_Analysis/` workdir) with two clean CLIs.

### 16.2 Non-goals

- Re-encrypting or repackaging AC data — output is decrypted plain-text only.
- Modifying the AC source folders in place — `cars_in/` and `tracks_in/` are read-only as far as `prep` is concerned.
- Generating corner notation JSON — that is `analysis/corner_analysis.py`'s job (§17), not prep.
- Wrapping or hiding the low-level `decode_acd.py` / `decode_track.py` scripts — they remain directly runnable for advanced users (just relocated under `prep/`).
- Validating AC physics data (no `engine.ini` schema check etc.) — caller is trusted to drop a real AC folder.

### 16.3 `prep/prep_car.py`

**CLI:**
```
python prep/prep_car.py <cars_in_dir> [--output-root cars_csv]
```

- `<cars_in_dir>` — a folder under `cars_in/`, e.g. `cars_in/bmw_1m`. Must contain `data.acd`. The folder name (basename) is used as the decryption key.
- `--output-root` — root output dir. Default `cars_csv`. Files are written to `<output-root>/<car>/data/`.

**Behaviour:**
1. Validate that `<cars_in_dir>/data.acd` exists; error clearly if not.
2. Compute `car_name = basename(<cars_in_dir>)`.
3. Invoke `prep/decode_acd.decode_acd(...)` (function extracted from the current `decode_acd.py`'s `main`) with `(acd_path=<cars_in_dir>/data.acd, car_name=car_name, output_dir=<output-root>/<car>/data/)`.
4. Print a one-line summary: `Decrypted N files into <output-root>/<car>/data/`.

**Output:** `cars_csv/<car>/data/` populated with `*.ini` and `*.lut` files (same set as today's `decode_acd.py` produces).

**Module size:** ~80 lines (argparse + path validation + a couple of error messages). The actual decryption stays in `prep/decode_acd.py`.

### 16.4 `prep/prep_track.py`

**CLI:**
```
python prep/prep_track.py <tracks_in_dir> [--output-root tracks_csv] [--ds 1.0] [--layouts all|<name>,...]
```

- `<tracks_in_dir>` — a folder under `tracks_in/`, e.g. `tracks_in/ks_nurburgring`. Expected layout:
  - For a single-layout track: `<tracks_in_dir>/ai/fast_lane.ai` plus surface data (`<tracks_in_dir>/data/surfaces.ini`, etc.).
  - For multi-layout tracks (Nurburgring): `<tracks_in_dir>/<layout>/ai/fast_lane.ai`. Layouts are discovered by scanning subdirectories with an `ai/fast_lane.ai`.
- `--output-root` — default `tracks_csv`. Files are written to `<output-root>/<track>/`.
- `--ds` — distance-step in metres for the output CSV resampling. Default 1.0. Smaller = larger CSV but smoother.
- `--layouts` — comma-separated layout names to process; `all` (default) processes everything discovered.

**Behaviour:**
1. Detect layouts (as above). Error clearly if no `fast_lane.ai` is found anywhere.
2. For each layout `L`:
   - Parse the AI line via `prep/decode_track.parse_fast_lane(...)` (the existing low-level parser, refactored to expose a function — not just a CLI).
   - Compute the rich per-point CSV columns: `index, distance_m, segment_length_m, x, y, z, elevation_m, gradient_pct, radius_m, speed_ms, speed_kmh, width_left_m, width_right_m, width_total_m`.
     - `radius_m` from the curvature of the AI line (3-point circumcircle or savgol-smoothed first/second derivative). Cap straights at 2000 m (matches existing convention).
     - `gradient_pct` from `dy/distance` smoothed over a window (~10 m).
     - `width_*` from AC's `side_l` / `side_r` channels in `fast_lane.ai` (already present in the binary format).
     - `speed_ms` from the AI's reference speed channel; `speed_kmh = speed_ms * 3.6`.
   - Resample to a uniform `ds` grid in distance.
   - Write `tracks_csv/<track>/layout_<L>.csv` (centerline interpretation — the AI line **is** the reference path here, but the columns match what the rest of the pipeline expects). Optionally also write `tracks_csv/<track>/layout_<L>_ideal_line.csv` if AC exposes a separate ideal-line file (`pit_lane.ai` is not it — leave to ArchDev's judgement; if no separate file exists, ship a single `layout_<L>.csv` and document the omission).
3. Print a one-line summary per layout: `Wrote layout_<L>.csv (<rows> rows, <length> m)`.

**Output:** `tracks_csv/<track>/layout_<layout>.csv` (and optionally `_ideal_line.csv`) — full column set per §7.1 / README.

**Module size:** ~200 lines (multi-layout dispatch + column computation + CSV writing). Heavy lifting in `prep/decode_track.py` (parsing) and small helpers in `prep/_geometry.py` if needed.

### 16.5 `prep/decode_acd.py` and `prep/decode_track.py`

- Same code as today's root-level files, moved into `prep/`.
- Refactor: expose a callable function (`decode_acd(...)`, `parse_fast_lane(...)`) so `prep_car.py` and `prep_track.py` don't have to shell out or re-implement.
- Keep `if __name__ == "__main__":` blocks so they remain directly invocable as standalone scripts.

### 16.6 Prerequisites for adding a new car or track

**New car:**
1. Copy `<AC install>/content/cars/<car>` into `cars_in/<car>/`.
2. Run `python prep/prep_car.py cars_in/<car>`.
3. `cars_csv/<car>/` is ready for `lap.py`.

**New track:**
1. Copy `<AC install>/content/tracks/<track>` into `tracks_in/<track>/`.
2. Run `python prep/prep_track.py tracks_in/<track>`.
3. (Optional) Run `python analysis/corner_analysis.py tracks_csv/<track>/layout_<L>.csv` for each layout to produce corner-notation JSON.
4. `tracks_csv/<track>/` is ready for `lap.py` (the corners JSON is consumed by downstream tooling, not strictly required by `lap.py` itself).

### 16.7 Acceptance criteria (prep)

See §11.11 and §11.12. Headline checks:
- `prep_track.py` on a fresh `tracks_in/ks_nurburgring` produces `layout_sprint_a.csv` matching the committed reference within tight numeric tolerance.
- `prep_car.py` on a fresh `cars_in/bmw_1m` produces a `cars_csv/bmw_1m/data/` that `lap.py` can consume to reproduce the committed lap time.

---

## 17. Corner analysis (addendum, v1 reorg)

### 17.1 Goal

Turn a track CSV into a canonical, telemetry-free corner-notation JSON file plus two visualisations, using `tracks_config.json` as the single source of truth for classification thresholds and colours.

### 17.2 Non-goals

- Merging telemetry — the telemetry+track merge path lives in `src/lap_estimator/telemetry.py` and is consumed only by `fit_driver.py` and `validate.py`.
- Re-generating `track_points.csv`, `track_corners.csv`, or `track_meta.csv` with base64-embedded images (the legacy DuckDB-staging exports in the current `Corner_Analysis/corner_analysis.py`) — these are dropped from v1. The JSON-based notation file in §17.4 replaces them.
- Choosing the racing line — the analysis operates on whatever CSV the user passes in (centerline or ideal-line).
- Per-corner driver fitting (v2, in `driver_fit`).

### 17.3 CLI shape

```
python analysis/corner_analysis.py <track_csv> [--config tracks_config.json] [--no-plot] [--no-json]
```

- `<track_csv>` — required positional. Path to a track CSV produced by `prep_track.py`. If omitted, the script falls back to `tracks_config.json`'s `default_track` (currently `tracks_csv/ks_nurburgring/layout_sprint_a.csv`).
- `--config` — path to the classification config. Default: `tracks_config.json` at the repo root.
- `--no-plot` — skip the two PNGs.
- `--no-json` — skip writing the JSON (useful when iterating on visualisations).

### 17.4 Output schema — `<layout_stem>_corners.json`

Path: same directory as `<track_csv>`, file named `<csv_stem>_corners.json`. Example: `tracks_csv/ks_nurburgring/layout_sprint_a_corners.json`.

```json
{
  "track": "ks_nurburgring",
  "layout": "sprint_a",
  "source_csv": "tracks_csv/ks_nurburgring/layout_sprint_a.csv",
  "total_length_m": 3565.0,
  "config": {
    "hairpin_max_m": 60,
    "tight_max_m": 150,
    "sweeper_max_m": 400,
    "straight_threshold_m": 500
  },
  "generated_at": "2026-05-13T14:22:01Z",
  "version": "1",
  "corners": [
    {
      "id": 1,
      "type": "hairpin",
      "direction": "left",
      "distance_start_m": 412.3,
      "distance_end_m": 478.1,
      "length_m": 65.8,
      "min_radius_m": 28.5,
      "avg_radius_m": 41.2,
      "ai_min_speed_kmh": 54.0,
      "ai_avg_speed_kmh": 62.1
    }
  ]
}
```

Field semantics:
- `track` — basename of the parent directory of `<track_csv>`.
- `layout` — derived from the CSV stem by stripping the `layout_` prefix and `_ideal_line` suffix if present.
- `total_length_m` — `max(distance_m)`.
- `config` — the actual thresholds used (echoed from `tracks_config.json`'s `corner_thresholds`). `straight_threshold_m` is the value used to detect corner spans (default 500, kept hard-coded as a sane default with an optional override in `tracks_config.json` under `corner_thresholds.straight_threshold_max` — name is ArchDev's call; if added, document it).
- `corners[].type` — one of `hairpin | tight | sweeper | straight`. Classification is by `min_radius_m`:
  - `min_radius_m < hairpin_max_m` → `hairpin`
  - `hairpin_max_m <= min_radius_m < tight_max_m` → `tight`
  - `tight_max_m <= min_radius_m < sweeper_max_m` → `sweeper`
  - else → omitted (it's a straight; only true corner spans land in the array). v2 candidate: include straights as bookkeeping entries.
- `corners[].direction` — derived from the sign of the signed-radius computed by the analysis (or by looking at the y-axis component of the road normal). `left` if the road curves to the driver's left, `right` if right. `straight` only for entries with `type == straight` (not produced in v1 per the rule above, but the schema slot is reserved).
- `corners[].length_m` — `distance_end_m - distance_start_m`.
- `corners[].ai_min_speed_kmh` / `ai_avg_speed_kmh` — min/avg of the AI reference speed (`speed_kmh` column) over the corner span.

### 17.5 Algorithm

1. Load `<track_csv>` (read `distance_m`, `radius_m`, `speed_kmh`, plus `x, z` for visualisations).
2. Load `tracks_config.json` (or `--config`).
3. Smooth `radius_m` with a 25-sample (~25 m at `ds=1.0`) moving average — same as the existing script.
4. Walk the smoothed radius array; mark contiguous spans where `smooth_radius < straight_threshold_m` (default 500) and `length > 20` samples as corner candidates.
5. Merge adjacent candidates whose gap is < 1 % of total length (existing "merge 4+5" heuristic, generalised).
6. For each merged corner: compute `min_radius_m`, `avg_radius_m`, classify into `hairpin | tight | sweeper`, infer `direction` from signed curvature.
7. Emit the JSON per §17.4 unless `--no-json`.
8. Render two PNGs unless `--no-plot`:
   - `<csv_stem>_corner_map.png` — top-down `x,z` line, coloured per-point by severity (using `tracks_config.json` colours). Labels each corner with `T<id>`.
   - `<csv_stem>_speed_vs_position.png` — `distance_m` vs `speed_kmh`, with severity-coloured `axvspan` overlays per corner.
9. Print a stdout summary table (existing "NURBURGRING SPRINT – CORNER ANALYSIS" table, with the track name generalised).

### 17.6 Module layout

- `analysis/corner_analysis.py` — single file, ~350 lines. argparse + load + classify + JSON write + matplotlib. Imports `track.Track.from_csv` from `lap_estimator` (or re-implements a tiny CSV-reader if avoiding the package boundary is cleaner — ArchDev's call; both are fine, the simulator doesn't have to be importable for corner analysis to work).
- No new helpers required outside this file.

### 17.7 Visualisations (kept from existing script)

- **`<csv_stem>_corner_map.png`** — 2D track map, per-point coloured line, corner labels, severity legend. Same style as the existing `Curve_Analysis/track_corner_map.png` (dark background, coloured corner badges).
- **`<csv_stem>_speed_vs_position.png`** — speed-vs-distance line with coloured corner overlays. Same style as `Curve_Analysis/speed_vs_position.png`.

The legacy `track_points.csv`, `track_corners.csv`, and `track_meta.csv` (with base64 images) outputs of `Corner_Analysis/corner_analysis.py` are **not** reproduced — they were staging for an aborted DuckDB pipeline. If they're needed later, they can be regenerated from the JSON + PNGs by a separate exporter.

### 17.8 Decisions baked in

1. **Corner notation is JSON, not CSV.** A JSON object captures the nested config + corners array more naturally than two correlated CSVs.
2. **One JSON per layout, alongside the CSV.** Consumers find it by sibling-lookup; no central index.
3. **`tracks_config.json` is the source of truth for thresholds and colours.** No more hard-coded numbers in `corner_analysis.py`. Editing the config and re-running the script is the supported tuning workflow (acceptance §11.15).
4. **Telemetry-free.** No AC log dependency. Telemetry merging stays in `src/lap_estimator/telemetry.py`.
5. **Straight spans are not emitted as corner records in v1.** Only true corners (R < 400 m) appear in `corners[]`. Schema reserves `direction: straight` for v2.
6. **The 500 m `STRAIGHT_THRESHOLD_M` for corner-span detection is kept hard-coded as a default**, but `tracks_config.json` may override it (key TBD by ArchDev; not blocking).
7. **The legacy DuckDB-staging exports are dropped.** They were never consumed downstream in this repo.

### 17.9 Acceptance criteria (corner analysis)

See §11.13, §11.14, §11.15. Headline checks:
- Running on `tracks_csv/ks_nurburgring/layout_sprint_a.csv` produces a `_corners.json` that visually matches the existing `Corner_Analysis/track_corner_map.png` corners.
- Telemetry-free — runs without any AC log present.
- Editing thresholds in `tracks_config.json` changes the output JSON, without code edits.

### 17.10 Open questions / v2 candidates

- **Include straight spans as schema-typed entries** so consumers get a complete distance partition.
- **Per-track corner naming.** Real tracks have proper names ("Schwedenkreuz", "Karussell"). v2 could load a `<track>_corner_names.json` overlay and merge it in.
- **Confidence score per corner classification** — e.g. how clearly the `min_radius` falls inside the threshold band. Useful for the v2 driver-fit per-corner skill profile.
- **Auto-discover and process all layouts under a track folder** when a directory is passed instead of a CSV.

---

## Decisions block (locked in this revision)

1. **Folder convention is locked.** `cars_in/` / `tracks_in/` = user-dropped raw AC (gitignored). `cars_csv/` / `tracks_csv/` = tracked outputs of prep. `drivers/` = tracked driver YAMLs. `samples/aclog/` = tracked sample telemetry. (§6.9)
2. **Repo layout uses a `src/lap_estimator/` package** for simulator library code, with `prep/`, `analysis/`, and root-level CLIs (`lap.py`, `fit_driver.py`) as separate scopes. Path bootstrap via per-script `sys.path` insert until a `pyproject.toml` editable install lands. (§6.8)
3. **Two simulator CLIs** (`fit_driver.py`, `lap.py`) plus **three pipeline CLIs** (`prep/prep_car.py`, `prep/prep_track.py`, `analysis/corner_analysis.py`) — five total entry points. No separate `validate_lap.py`. (§14.3 item 11, §15.7)
4. **Corner notation lives in `<track_dir>/<layout_stem>_corners.json`** with the schema in §17.4. JSON, not CSV. Telemetry-free.
5. **`tracks_config.json` is the in-repo single source of truth for corner-classification thresholds and colours.** Loaded by `analysis/corner_analysis.py`. (§7.9, §17.8)
6. **Corner analysis is downstream of prep, not part of it.** Two independent steps. (§16.2, §17.1)
7. **Legacy DuckDB-staging CSVs (`track_points.csv`, `track_corners.csv`, `track_meta.csv`) are dropped.** The JSON notation file replaces them.
8. **Sample AC telemetry log moves to `samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv`** and is committed. The existing `Corner_Analysis/` folder is untouched (user-managed scratch). (§6.9, §10)
9. **`cars_in/*` and `tracks_in/*` stay gitignored** (already in `.gitignore`). `tracks_csv/`, `cars_csv/`, `drivers/`, `samples/`, `tracks_config.json`: tracked. (§6.9)
10. **`prep_track.py` does not auto-invoke corner analysis.** User chains the two commands. v2 candidate for a `prep_all.py` umbrella.
11. **`analysis/corner_analysis.py` is a rewrite, not an in-place edit** of the existing `Corner_Analysis/corner_analysis.py`. The latter stays as user scratch.
