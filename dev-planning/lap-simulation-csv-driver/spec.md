# Lap Simulation: CSV Track + Driver Config

**Status:** Draft (v1 + telemetry-fitting + sim-telemetry-emission + cross-track-validation + project-reorg addendum; v1.1 in-flight: driver JSON migration + driver-lag low-pass + trail-brake/throttle-ramp heuristic + two-lap tiled sim — §13/§14/§20; v2 MF4 telemetry output PLANNED — §18; v3 slip-based physics + tyre-state BACKLOG — §19)
**Project:** LapTimeEstimator
**Branch:** feature/sc-71955/lap-simulation
**Created:** 2026-05-13
**Planned with:** Buddy

## 1. Summary

Today the lap-time simulator consumes a `Car` (parsed from AC data) and a `Track` built from hard-coded segments or a simple `{length, radius}` JSON. There is no notion of a driver, the AC-input → CSV preparation steps are scattered between root-level scripts (`decode_acd.py`, `decode_track.py`) and an untracked `Corner_Analysis/` workdir, and corner classification is bolted onto an exploratory analysis script that depends on telemetry merging.

This feature reshapes the tool around three first-class inputs — **car**, **track CSV**, **driver JSON** — and produces a lap-time estimate plus a trace artifact that can be visually validated against the AI reference speed already present in the CSV. The driver model is intentionally minimal in v1 (skill + optional consistency noise) so we get the input/output plumbing right before investing in richer driver behaviour.

The whole feature ships behind **two simulator CLI entry points** — `fit_driver.py` (derive a driver JSON from a real AC telemetry lap) and `lap.py` (run a sim on a track with a given driver, optionally cross-validating against a real lap on that same track via `--validate-against`). The "just run a sim" mode is the default of `lap.py`; the cross-track validation flow is the same script with one extra flag.

It also formalises the **preparation pipeline** (§16): two CLIs `prep/prep_car.py` and `prep/prep_track.py` that turn user-dropped AC content under `cars_in/` and `tracks_in/` into repo-friendly artefacts under `cars_csv/` and `tracks_csv/`. And it splits **corner analysis** (§17) out into a standalone, telemetry-free script `analysis/corner_analysis.py` that reads a track CSV plus the in-repo `tracks_config.json` and writes a canonical corner notation JSON next to the track.

**Pivot (2026-05-13):** instead of asking the user to hand-author `drivers/<name>.json`, the headline workflow becomes **deriving the driver JSON from a real Assetto Corsa telemetry lap** via the `fit_driver.py` tool (see §13). Hand-authored configs are still supported — they just stop being the primary entry point.

**Closing the loop (2026-05-13):** after a sim run, the simulator also emits a **synthetic telemetry CSV** that matches the real AC log schema exactly (see §14). This makes the sim's output a drop-in replacement for real telemetry — downstream tools (notably `fit_driver.py`) can ingest sim output the same way they ingest real laps. A fit → sim → emit → re-fit round-trip becomes a strong self-consistency check on the whole pipeline.

**The actual point (2026-05-13):** the real reason this tool exists is **cross-track prediction with validation** (see §15). Fit a driver on Track A, predict their lap time on Track B (which they have never run in sim or in AC), then have the driver drive Track B in AC for ~10 laps to learn it, capture telemetry, and compare. Sections §13 and §14 are the plumbing; §15 is the headline use case.

**Reorg (2026-05-13):** the repository layout is locked. `cars_in/` and `tracks_in/` are user-dropped raw AC content (gitignored). `cars_csv/` and `tracks_csv/` are the outputs of the preparation scripts (tracked). Preparation scripts live under `prep/`, corner analysis lives under `analysis/`, simulator library code lives under `src/lap_estimator/`, and the two simulator CLIs (`lap.py`, `fit_driver.py`) stay at the repo root for discoverability. See §6.9 and §16/§17.

**Future directions (2026-05-13):** two longer-horizon backlog items are captured in §19 — a slip-based simulator that can represent drift / oversteer / understeer, and a tyre-state model (temp / wear / pressure) that modulates grip lap-over-lap. Both are research-scoped, not v1 work; §19 documents what AC already gives us for free, the architectural impact, and the recommended sequencing.

**v1.1 in-flight (2026-05-13):** four bundled changes layered on the shipped v1 plumbing — (A) driver config format moves from YAML to JSON (no fallback); (B) driver-lag 1st-order low-pass on emitted gas/brake; (C) trail-brake + throttle-ramp corner-shape heuristic applied to gas/brake before the low-pass; (D) two-lap "tiled" simulation always emitted (lap 1 standing, lap 2 flying), with a `lap` column on telemetry/trace CSVs and Monte-Carlo / validation / loop-closure all keying off lap 2. See §13, §14.3, §20.

## 2. Goals

- Accept a **car data directory**, a **track CSV path**, and a **driver JSON path** as the three positional inputs to the sim CLI (`lap.py`).
- Consume the rich per-point track CSV format (ideal line preferred, centerline fallback) directly — no intermediate JSON conversion required.
- Apply a simple **driver model** (`skill_pct`, optional `consistency_sigma`, `driver_tau_s`, `trail_brake_m`, `throttle_ramp_m`) that scales the car's effective grip uniformly and shapes the emitted gas/brake trace.
- Emit a per-point **trace CSV** and a **sim-vs-AI comparison PNG** next to the track input so the user can sanity-check results in Quix Cloud / locally.
- Preserve current stdout lap-time report format (extended for two-lap output — see §20).
- Keep modules small (~500-line soft ceiling).
- Provide a `fit_driver.py` CLI that ingests an AC telemetry CSV and emits a driver JSON calibrated to that lap.
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
- Multi-lap / fuel-burn / tyre-wear simulation. (v1.1 emits two laps for warm-up/flying-lap representativeness — see §20 — but does **not** model fuel burn or tyre wear; tyre state is constant across both laps.)
- Web UI. CLI only.
- Per-corner skill profile in v1 (v2 hook in §13.6).
- Fitting tyre, aero, or engine parameters from telemetry — only driver-skill scalars are fit; the car model is treated as ground truth.
- Per-track familiarity / learning-curve modelling in v1. v1 treats `skill_pct` and `consistency_sigma` as track-agnostic properties of the driver (§15 limitations). v2 candidate.
- No database, object store, remote artefact server, or networked service. All inputs (car data, track CSV, driver JSON, telemetry logs) are local files committed to or dropped into the repo. DuckDB / remote-storage integration is explicitly v2+.
- **(reorg)** The corner-analysis tool does **not** merge telemetry — it is purely track-derived. The telemetry-merge code path stays in `src/lap_estimator/telemetry.py` and is consumed only by the fit/validate flows.
- **(reorg)** `prep/prep_track.py` does **not** infer corner notation — that's `analysis/corner_analysis.py`'s job, run as a separate post-prep step.
- **MF4 output is v2 — see §18; v1 only emits CSV.**
- **Slip-based physics, drift / oversteer / understeer dynamics, and per-wheel thermal / wear / pressure state are v3 backlog — see §19. v1 keeps the point-mass 3-pass sim with isotropic μ.**

## 4. User stories / scenarios

1. **Run a single lap.** User has decrypted car data in `cars_csv/bmw_1m/data/`, a track CSV at `tracks_csv/ks_nurburgring/layout_gp_a_ideal_line.csv`, and `drivers/pro.json`. They run `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_gp_a_ideal_line.csv drivers/pro.json` and see the lap-time report on stdout (both laps — §20), plus a trace CSV and PNG written next to the track CSV.
2. **Compare two drivers, same car/track.** User runs the command twice with `drivers/pro.json` and `drivers/amateur.json`. Lap times differ in a way consistent with `skill_pct`. Trace files are named so they don't collide.
3. **Use a centerline-only track.** User points at a `layout_*.csv` with no `*_ideal_line.csv` sibling. Sim runs against the centerline and prints a notice in stdout.
4. **Consistency study.** User sets `consistency_sigma: 0.3` in the driver JSON. Sim performs N Monte-Carlo runs on **lap 2 only** (the flying lap; §20) and prints `mean ± σ` for lap 2. Trace CSV/PNG still come from a single representative run (the deterministic skill-only run).
5. **Legacy invocation (built-in track).** User runs `python lap.py cars_csv/bmw_1m monza drivers/pro.json`. The legacy hard-coded `monza` track still works; no AI-speed overlay (no CSV source).
6. **Fit a driver from telemetry.** User has an AC log at `samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv`. They run `python fit_driver.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv drivers/ludvik_nurburgring_sprint.json`. The tool writes the JSON, runs a validation sim, and prints `Real lap: 1:42.135 / Sim lap 2 (flying): 1:43.402 / Δ = +1.267 s (+1.2%)`.
7. **Loop-closure check.** User runs `lap.py` to produce a sim (two laps) → sim emits a synthetic telemetry CSV with `lap` column → user feeds that synthetic CSV back into `fit_driver.py` against the same car/track. The fitter picks **lap 2** by default (§20). The recovered driver JSON's `skill_pct` matches the original within ~5% and the validation sim lap time is within ~1 s.
8. **Cross-track prediction and validation (headline workflow).** User has fit `drivers/ludvik_nurburgring_sprint.json` from a learned Nurburgring Sprint lap (Track A). They now want to know how Ludvík will perform on Brands Hatch (Track B), which they have not driven in AC yet.
   1. Run `python lap.py cars_csv/bmw_1m tracks_csv/brands_hatch/layout_indy_ideal_line.csv drivers/ludvik_nurburgring_sprint.json` → predicted lap times (both laps) + sim telemetry CSV written next to the Brands Hatch track.
   2. User drives Brands Hatch in AC for ~10 laps until lap times stabilise (the "learned" lap), captures telemetry on a representative lap.
   3. Run `python lap.py cars_csv/bmw_1m tracks_csv/brands_hatch/layout_indy_ideal_line.csv drivers/ludvik_nurburgring_sprint.json --validate-against <real_brands_lap>.csv` → stdout delta report (real vs **sim lap 2**), overlay PNG, per-corner / per-bin delta table.
   4. Acceptance: |delta| < ~3 s on a 2–3 minute lap is "good"; >10% means either the fit is wrong or the driver has not learned the track yet.
9. **(reorg) Prepare a new car from raw AC content.** User copies `<AC install>/content/cars/bmw_1m` into `cars_in/bmw_1m/` and runs `python prep/prep_car.py cars_in/bmw_1m`. The script writes `cars_csv/bmw_1m/data/*.ini` and `*.lut`. The output is ready for `lap.py`.
10. **(reorg) Prepare a new track from raw AC content.** User copies `<AC install>/content/tracks/ks_nurburgring` into `tracks_in/ks_nurburgring/` and runs `python prep/prep_track.py tracks_in/ks_nurburgring`. The script writes `tracks_csv/ks_nurburgring/layout_<layout>.csv` and `tracks_csv/ks_nurburgring/layout_<layout>_ideal_line.csv` for each layout it finds, populated with the full column set from the README (`distance_m, segment_length_m, x, y, z, elevation_m, gradient_pct, radius_m, speed_ms, speed_kmh, width_left_m, width_right_m, width_total_m`).
11. **(reorg) Run corner analysis on a prepared track.** User runs `python analysis/corner_analysis.py tracks_csv/ks_nurburgring/layout_sprint_a.csv`. The script reads `tracks_config.json` for thresholds and colours, classifies corners, writes `tracks_csv/ks_nurburgring/layout_sprint_a_corners.json` (schema in §17.4), and emits the existing `..._corner_map.png` and `..._speed_vs_position.png` visualisations next to the track CSV. No telemetry input is required.
12. **(v1.1) Single-lap mode for fast sweeps.** User runs `python lap.py ... drivers/pro.json --single-lap` and gets only lap 1 (standing start), matching legacy v1 behaviour. Telemetry/trace CSVs omit the `lap` column when this flag is set (or carry `lap=1` only; ArchDev's call — schema must still validate).

## 5. Proposed design

- Add a new `driver.py` module with a `Driver` dataclass (loaded from JSON) and a small `apply_to_car(car)` helper that returns a thin grip-scaling wrapper. Keep `car.py` untouched: scaling is done by composition, not mutation. The dataclass also carries the three v1.1 fields (`driver_tau_s`, `trail_brake_m`, `throttle_ramp_m`) — they are consumed by `sim_telemetry.py`, not by the grip-scaling path.
- Extend `track.py` with a `Track.from_csv(path)` classmethod that parses the rich CSV. The resulting `Track` exposes the same `to_points(ds)` API plus a parallel `to_ai_reference(ds)` returning the AI speed trace for overlay.
- Adjust `simulator.simulate(...)` so it takes an optional `driver` argument and returns a richer `SimResult` carrying the AI reference (when available), per-point time, and the **per-point binding-limit label** (`corner` / `accel` / `brake`) needed by sim-telemetry emission (§14). The simulator also supports a two-lap tiled mode (§20) — same 3-pass core, segment list duplicated, output post-split into `lap 1` / `lap 2`.
- Move output artifact generation (trace CSV, comparison PNG) into a new `report.py` module so `simulator.py` stays focused on physics.
- `lap.py` is the single sim CLI: takes car / track / driver, runs sim (or Monte-Carlo on lap 2), and orchestrates `report.write_trace_csv`, `report.write_comparison_plot`, and `sim_telemetry.write_synthetic_log` (unless `--no-telemetry`). When `--validate-against <real_telemetry.csv>` is supplied, `lap.py` additionally calls `validate.validate_lap(...)` and writes the overlay PNG + bins CSV (§15). The pre-existing `main.py` file is renamed to `lap.py` outright.
- Add a `telemetry.py` module that owns AC-log parsing and merging-with-track logic. Add a `driver_fit.py` module that consumes the merged frame plus a `Car` and emits driver parameters. Add a thin `fit_driver.py` CLI wrapper.
- Add a `sim_telemetry.py` module that takes a `SimResult` + total track length + cadence (ms) and emits a CSV that matches the AC telemetry schema exactly (§14). v1.1: this module also applies the trail-brake/throttle-ramp heuristic (§14.3) and the driver-lag low-pass (§14.3) to the derived gas/brake before writing.
- Add a `validate.py` module that runs a sim + diffs the result against a real AC telemetry CSV. Validation is exposed through `lap.py --validate-against` (§15). Reuses `telemetry.py`'s merge logic and `report.py`'s plotting helper. v1.1: validation compares the real lap against sim **lap 2** (§20).
- **(reorg)** Move all simulator library modules into a `src/lap_estimator/` package: `car.py`, `track.py`, `driver.py`, `driver_fit.py`, `simulator.py`, `telemetry.py`, `sim_telemetry.py`, `report.py`, `validate.py`. The two simulator CLIs (`lap.py`, `fit_driver.py`) stay at the repo root and import from the package.
- **(reorg)** Move the low-level AC decoders into `prep/`: `decode_acd.py` and `decode_track.py` move there from the repo root. Two new CLI wrappers, `prep/prep_car.py` and `prep/prep_track.py`, are the user-facing entry points and orchestrate the low-level decoders end-to-end from AC raw input to populated `cars_csv/` / `tracks_csv/` output. See §16.
- **(reorg)** Add an `analysis/` folder with `analysis/corner_analysis.py` — a rewrite of the existing `Corner_Analysis/corner_analysis.py` that is telemetry-free, reads `tracks_config.json` for thresholds/colours, and emits the canonical `<layout>_corners.json` notation file (schema in §17.4) plus the two existing PNG visualisations. See §17.

This shape keeps the physics core intact, isolates the driver concept behind a single small module, puts all I/O / plotting / synthetic-telemetry in dedicated modules so no single file pushes the 500-line soft cap, and makes the AC-input → CSV → analysis pipeline first-class and reproducible.

## 6. Sub-features / work breakdown

### 6.1 Driver module (new file: `src/lap_estimator/driver.py`)
- **What it does:** Loads a driver JSON, exposes `skill_pct`, `consistency_sigma`, `driver_tau_s`, `trail_brake_m`, `throttle_ramp_m`, and provides a function that wraps a `Car` so its lateral and longitudinal grip queries are scaled by `skill_pct` (and optionally perturbed per-point by Gaussian noise with `sigma_grip` derived from `consistency_sigma`). The v1.1 fields (`driver_tau_s`, `trail_brake_m`, `throttle_ramp_m`) are **not** consumed by the grip-scaling path — they are read by `sim_telemetry.py` only (§14.3).
- **Touchpoints:** new `src/lap_estimator/driver.py`; `src/lap_estimator/simulator.py` (accept driver, use wrapped car); `src/lap_estimator/sim_telemetry.py` (read v1.1 fields).
- **Approach:** `Driver.load(path) -> Driver` (JSON parse via stdlib `json`). `Driver.wrap(car, rng=None, noise=False) -> CarLike` returns an object that delegates to the underlying car but overrides `max_cornering_speed`, `max_braking_decel`, and `max_traction_force`'s grip-limited branch. Simplest impl: scale `tyre_grip_lateral` and `tyre_grip_longitudinal` results by `skill_pct * (1 + noise)` where `noise` is sampled per point (passed in by simulator).
- **Dependencies:** stdlib `json` only. **PyYAML removed** (was previously a mandatory dep).
- **Owner:** ArchDev.

### 6.2 Driver JSON schema + example files
- **What it does:** Defines the v1.1 schema and ships two examples.
- **Touchpoints:** new `drivers/pro.json`, `drivers/amateur.json`.
- **Schema:**
  ```json
  {
    "name": "Pro",
    "skill_pct": 0.95,
    "consistency_sigma": 0.0,
    "driver_tau_s": 0.12,
    "trail_brake_m": 30.0,
    "throttle_ramp_m": 40.0
  }
  ```
  Field semantics:
  - `name` — string, free-form, required.
  - `skill_pct` — float in (0, 1], required. Multiplier on tyre grip.
  - `consistency_sigma` — float ≥ 0, optional, default 0.0. Seconds; >0 enables Monte-Carlo on lap 2 (§20).
  - `driver_tau_s` — float ≥ 0, optional, default 0.12. Seconds; 1st-order low-pass time constant applied to derived gas/brake before CSV write (§14.3). `0.0` bypasses the filter (legacy bang-bang behaviour).
  - `trail_brake_m` — float ≥ 0, optional, default 30.0. Metres; distance over which brake tapers 1.0 → 0.0 transitioning out of a braking-bound region (§14.3). `0.0` disables the heuristic.
  - `throttle_ramp_m` — float ≥ 0, optional, default 40.0. Metres; distance over which gas ramps corner-limited-partial → 1.0 after exit of a corner-bound region (§14.3). `0.0` disables the heuristic.
- **Example values:**
  - `pro.json` — `skill_pct: 0.97`, `consistency_sigma: 0.1`, defaults for v1.1 fields.
  - `amateur.json` — `skill_pct: 0.82`, `consistency_sigma: 0.6`, `driver_tau_s: 0.20`, `trail_brake_m: 45.0`, `throttle_ramp_m: 55.0` (amateur = laggier inputs, longer trail-brake, longer throttle-up).
- **YAML → JSON migration (v1.1):** Every existing `drivers/*.yaml` is rewritten as the equivalent `.json` and the `.yaml` is deleted. No loader-side YAML fallback. Files affected:
  - `drivers/pro.yaml` → `drivers/pro.json`
  - `drivers/amateur.yaml` → `drivers/amateur.json`
  - `drivers/tomas_nurburgring_sprint.yaml` → `drivers/tomas_nurburgring_sprint.json`
  - `drivers/ludvik_nurburgring_sprint.yaml` → `drivers/ludvik_nurburgring_sprint.json`
  - All `drivers/tomas_lap*.yaml` from the recent fit comparison → `.json`.
  ArchDev does the rewrite in the same commit that drops the PyYAML dep. The migration is a clean break: there is no transition window.
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
- **What it does:** Accepts a driver; supports a single deterministic run, a Monte-Carlo mode for consistency, and a two-lap tiled mode (§20).
- **Touchpoints:** `src/lap_estimator/simulator.py`.
- **Changes:**
  - `simulate(car, track, driver=None, ds=2.0, rng=None, two_lap=True) -> SimResult` — driver is optional. If `None`, behaves as today (equivalent to `skill_pct=1.0, sigma=0`). `two_lap=True` (default) tiles the segment list, runs both laps in one pass (lap 1 starts from rest, lap 2 starts at lap-1 final speed), and returns a `SimResult` with `lap_id` per-point.
  - New `simulate_monte_carlo(car, track, driver, ds=2.0, n_runs=20, seed=0) -> MCResult` — runs N two-lap sims with per-point grip noise; **Monte-Carlo statistics are computed on lap 2 only**. Returns lap-2 mean/std plus one representative deterministic trace (skill-only, no noise) covering both laps for plotting.
  - `SimResult` gains `times` (per-point cumulative time array, monotonic across both laps), optional `ai_speeds` (m/s), `limit_label` (per-point string in `{"corner","accel","brake"}`), and `lap_id` (per-point int in `{1, 2}`).
  - Grip scaling lives entirely in the driver-wrapped car; the 3-pass core is unchanged. The label is derived during the 3-pass merge by checking which of `v_corner`, `v_forward`, `v_brake` is binding at each index. Lap boundary handling: lap 2's `v_forward` initial condition is lap 1's final speed (not zero).
- **Dependencies:** 6.1, 6.3.
- **Owner:** ArchDev.

### 6.5 Output artifacts (new file: `src/lap_estimator/report.py`)
- **What it does:** Writes trace CSV and comparison PNG; keeps stdout reporting.
- **Touchpoints:** new `src/lap_estimator/report.py`; move `print_report` from `simulator.py` to here.
- **Files written, adjacent to the input track CSV** (or cwd if track is built-in):
  - `<track_stem>_sim_trace.csv` with columns: `lap, distance_m, sim_speed_ms, sim_speed_kmh, ai_speed_kmh, time_s`. `ai_speed_kmh` is empty when no AI reference. The `lap` column carries `1` for the first lap's rows and `2` for the second; with `--single-lap`, only lap 1 rows are emitted.
  - `<track_stem>_sim_vs_ai.png` — matplotlib line plot: x=`distance_m`, y=km/h, lap 2 sim line + ai line (lap 1 sim line optional, faded — ArchDev's call). Skip the plot if matplotlib is not installed; warn on stdout.
  - If driver name is set, include it in the file stem: `<track_stem>__<driver_name>_sim_trace.csv` so multiple driver runs don't overwrite each other.
- **Reusable helper:** expose `report.plot_speed_overlay(distances, series_dict, title, output_path)` so `lap.py --validate-against` (§15) can call it without copy-pasting matplotlib code. `series_dict` maps label → speed-in-km/h array. Existing `write_comparison_plot` becomes a thin wrapper over this helper.
- **Dependencies:** matplotlib (optional, already listed).
- **Owner:** ArchDev.

### 6.6 CLI rewiring (`lap.py`)
- **What it does:** Single sim CLI; new positional args, dispatch logic, the optional cross-track validation flag, and the v1.1 `--single-lap` opt-out.
- **Touchpoints:** new `lap.py` at the repo root (rename of the existing `main.py`). No separate `validate_lap.py` script exists.
- **CLI:**
  ```
  python lap.py <car_data_dir> <track> <driver_json> \
                [--ds 2.0] [--all-tracks] [--single-lap] \
                [--no-plot] [--no-telemetry] [--telemetry-dt-ms 100] \
                [--validate-against <real_telemetry_csv>] [--bin-m 100] [--per-corner]
  ```
  - `<track>`: track CSV path (preferred), built-in name (`monza|spa|nurburgring|brands_hatch`), or legacy JSON path. Resolution order: file-exists-and-ends-`.csv` → CSV; file-exists-and-ends-`.json` and is **not** a driver JSON → JSON track; name in `BUILTIN_TRACKS` → built-in; else error. (Driver-vs-track JSON disambiguation: driver JSONs always have a top-level `skill_pct` key — `track.from_json` rejects any input that looks like a driver and tells the user to swap positions.)
  - `<driver_json>`: path to a driver JSON. Required positional argument; `.json` extension.
  - `--all-tracks`: kept; runs against all built-in tracks (skips CSV/JSON path). Cheap to keep.
  - `--single-lap`: **v1.1.** Disables two-lap mode (§20). Emits lap 1 (standing start) only. Restores byte-compatible v1 stdout when combined with `consistency_sigma == 0`.
  - `--no-plot`: skip PNG generation (useful in CI / Quix Cloud headless). Also skips the validation overlay PNG when `--validate-against` is set.
  - `--no-telemetry`: skip synthetic telemetry CSV emission (§14). Default off (telemetry is emitted by default).
  - `--telemetry-dt-ms`: cadence in milliseconds for the synthetic telemetry CSV. Default 100.
  - `--validate-against <real_telemetry_csv>`: opt-in cross-track validation (§15). When present, after the normal sim outputs are emitted, `lap.py` calls `validate.validate_lap(...)` to compare the **sim's lap 2** (§20) against `<real_telemetry_csv>`, prints the real/predicted/delta block + verdict, and writes the overlay PNG (unless `--no-plot`) and per-bin delta CSV alongside the normal sim outputs.
  - `--bin-m <int>`: per-distance-bin width in metres for the validation delta table. Default 100. Only meaningful with `--validate-against`. Ignored otherwise.
  - `--per-corner`: switch the validation delta table to corner-based bins (overrides `--bin-m`). Only meaningful with `--validate-against`. Ignored otherwise.
- **Owner:** ArchDev.

### 6.7 README / docs touch-up
- **What it does:** Reflect the new repo layout, the four CLIs (`prep/prep_car.py`, `prep/prep_track.py`, `analysis/corner_analysis.py`, plus `lap.py` / `fit_driver.py`), driver JSON schema (v1.1), output artifacts (trace + plot + synthetic telemetry, both with `lap` column), the cross-track validation workflow as `lap.py --validate-against` (§15), the corner notation JSON schema (§17.4), and the two-lap default (§20).
- **Touchpoints:** `README.md`, `docs/AI_CONTEXT.md`.
- **Dependencies removed (v1.1):** `PyYAML` is no longer listed; the lap-estimator tool has zero optional-config dependencies beyond numpy / matplotlib.
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
  - `drivers/*.json` — example drivers (pro, amateur) and any fitted drivers the user wants to keep. **v1.1: `.json`, not `.yaml`.** No `.yaml` files remain under `drivers/` after the migration commit.
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

### 7.2 Driver JSON (input) — v1.1
```json
{
  "name": "string",
  "skill_pct": 0.95,
  "consistency_sigma": 0.0,
  "driver_tau_s": 0.12,
  "trail_brake_m": 30.0,
  "throttle_ramp_m": 40.0,
  "source": {
    "telemetry_csv": "string",
    "track_csv": "string",
    "car_data_dir": "string",
    "real_lap_time_s": 0.0,
    "sim_lap_time_s": 0.0,
    "delta_s": 0.0,
    "fitted_at": "ISO-8601 string",
    "fit_version": "1"
  }
}
```
Field semantics and defaults:
- `name` (string, required) — used in file naming.
- `skill_pct` (float in (0, 1], required) — multiplier on tyre grip.
- `consistency_sigma` (float ≥ 0, optional, default 0.0) — seconds; >0 enables Monte-Carlo on lap 2 (§20).
- `driver_tau_s` (float ≥ 0, optional, default **0.12**) — seconds; driver-lag low-pass time constant (§14.3).
- `trail_brake_m` (float ≥ 0, optional, default **30.0**) — metres; trail-brake taper distance (§14.3).
- `throttle_ramp_m` (float ≥ 0, optional, default **40.0**) — metres; throttle ramp-up distance (§14.3).
- `source` (object, optional, populated by `fit_driver.py`) — provenance block.

Validation: `0 < skill_pct <= 1`, `consistency_sigma >= 0`, `driver_tau_s >= 0`, `trail_brake_m >= 0`, `throttle_ramp_m >= 0`. Hard error on parse failure. `source` block is informational; the loader passes unknown sub-keys through unchanged. **Format: JSON only — no YAML fallback in v1.1.**

### 7.3 Car data dir (input)
Unchanged. Either a directory containing `engine.ini` directly, or a parent with a `data/` subdirectory.

### 7.4 Trace CSV (output)
Columns, header row required:
```
lap, distance_m, sim_speed_ms, sim_speed_kmh, ai_speed_kmh, time_s
```
`ai_speed_kmh` is empty for built-in / JSON tracks. `lap` is `1` or `2` in default (two-lap) mode; with `--single-lap`, only lap 1 rows are emitted (the column is still present and constant `1` — keeps the schema stable for downstream consumers).

### 7.5 Comparison plot (output)
Single matplotlib figure, 1 axes, x-axis = `distance_m`, y-axis = `speed_kmh`, lines for lap 2 sim + ai (lap 1 sim optional/faded — ArchDev's call). Title = `<track_name> — <driver_name>`. Both lap times annotated as a sub-title.

### 7.6 Stdout report
Same fields as today (Max/Min/Avg Speed, segment table for built-in tracks, 0-100/0-200/top speed). Lap-time line is v1.1-extended to show both laps:
```
Lap 1 (standing): 1:48.6   |   Lap 2 (flying): 1:46.2 ± 0.06 (N=20)
```
Monte-Carlo applies to lap 2 only. With `--single-lap`, falls back to the v1 single-line format (byte-compatible with v1 when `consistency_sigma == 0`).

### 7.7 Synthetic telemetry CSV (output, new — see §14 for the algorithm)
Columns and column order match AC's log schema, with the v1.1 `lap` column appended (so existing AC-schema consumers can ignore it):
```
timestamp_ms,gas,brake,distanceTraveled,speedKmh,normalizedCarPosition,lap
```
`lap` is `1` or `2` in default mode; with `--single-lap`, `lap` is constant `1`. `timestamp_ms` is monotonic and continues across the lap boundary (lap 2's first sample's `timestamp_ms` = lap 1's last sample's `timestamp_ms` + `--telemetry-dt-ms`). `distanceTraveled` resets to 0 at the start of lap 2; `normalizedCarPosition` likewise wraps. File path: `<track_dir>/<track_stem>__<driver_name>_sim_telemetry.csv` (or cwd for non-CSV tracks). Same name convention as the trace file.

### 7.8 Corner notation JSON (output of `analysis/corner_analysis.py`)
See §17.4 for the full schema. Path: `<track_dir>/<layout_stem>_corners.json`.

### 7.9 `tracks_config.json` (in-repo config, input to `analysis/corner_analysis.py`)
Lives at the repo root. Current fields (kept as-is — schema is informally frozen for v1):
```json
{
  "_comment": "Corner classification thresholds (meters) and colors used for track map and plot overlays.",
  "corner_thresholds": {
    "hairpin_max": 60,
    "tight_max":  150,
    "sweeper_max": 400
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
    "min_length_m": 20
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
- **Synthetic telemetry realism (improved in v1.1 but still heuristic):** the gas/brake reconstruction in §14.3 layers a trail-brake / throttle-ramp shape on top of the limit-label rules, then runs a 1st-order low-pass with `driver_tau_s` default 120 ms. Resulting traces look human-plausible at the 100 ms cadence (no infinitely-thin spikes, visible corner-entry/exit shapes) but still aren't a true motor-control model. v2 hook: a full driver-input model with reaction time and prediction.
- **Driver-lag default tuning:** `driver_tau_s=0.12` was chosen as a reasonable human pedal-modulation time constant (user's initial 20 ms guess was below the motor-reflex floor and gave near-zero smoothing at 100 ms sampling — bumped on Buddy recommendation, user accepted). Documented here so future tuners know the rationale.
- **Lap-2 fairness vs lap-1 representativeness:** v1.1 makes lap 2 (the flying lap) the headline output (MC stats, validation target, loop-closure default). Lap 1 (standing) is mostly a warm-up artefact and is included for completeness, not comparison. Documented in §20 so consumers don't accidentally compare a real flying lap against sim lap 1.
- **Driver portability across tracks (v1 limitation).** v1 treats `skill_pct` and `consistency_sigma` as track-agnostic properties of the driver. In practice a driver's effective skill on a brand-new track is **lower** than on a learned one — the "10-lap learning curve". A fit done on a familiar track will likely **overestimate** the driver's skill on a fresh track, producing optimistic predictions in §15. v1 documents this; v2 candidate is a per-track `familiarity` modifier or a `confidence` term that decays with track novelty.
- **(reorg) Track-preparation parity with the existing CSVs.** The committed `tracks_csv/ks_nurburgring/layout_*.csv` files were produced by some prior (partly manual / partly in `Corner_Analysis/`) pipeline that is not currently a single script. `prep_track.py` must reproduce that column set faithfully. Mitigation: ArchDev diffs the output of `prep_track.py` against the committed Nurburgring CSVs (same input AC folder) before accepting the script. Acceptance criterion §11.11 enforces this.
- **(reorg) Import-path fragility.** Moving simulator code under `src/lap_estimator/` without a `pip install -e .` setup means every CLI / script needs a one-line sys.path bootstrap. Risk of someone forgetting the bootstrap in a new script. Mitigation: standardise the bootstrap in a small `_bootstrap.py` shim or document it in `docs/AI_CONTEXT.md` as the required first line of any new entry-point script.

### Constraints
- One feature one branch (already on `feature/sc-71955/lap-simulation`).
- No file over ~500 lines. Estimated post-change sizes: `src/lap_estimator/car.py` ~256 (unchanged), `track.py` ~280 (loader added), `simulator.py` ~240 (two-lap mode added), `driver.py` ~100 (JSON loader + v1.1 fields), `report.py` ~180 (lap column added), `lap.py` ~170 (--single-lap added), `telemetry.py` ~140, `driver_fit.py` ~190 (lap-2 default added), `fit_driver.py` ~60, `sim_telemetry.py` ~180 (heuristic + low-pass + lap column added), `validate.py` ~170 (lap-2-target added), `prep/decode_acd.py` (unchanged), `prep/decode_track.py` (unchanged), `prep/prep_car.py` ~80, `prep/prep_track.py` ~200, `analysis/corner_analysis.py` ~350. All comfortably under the ceiling.

### Open questions
- Monte-Carlo `n_runs` default — proposed 20. User can override later via CLI flag (not in v1).
- Where exactly to place driver JSONs — proposed `drivers/` at repo root. Locked unless user objects.
- Should `--all-tracks` also iterate drivers? Proposed: no, driver is single per invocation.
- Future v2 driver params (line deviation, brake/throttle split, reaction time) — schema should be forward-compatible (JSON parser ignores unknown keys with a warning).
- Per-track familiarity modelling — v2 candidate.
- Cross-track validation tolerance numbers — initial thresholds in §15 (|delta| < 3 s is good, >10% is bad) are guesses. Once 3+ validation runs exist, tighten them.
- **(reorg) `pip install -e .` or `sys.path` shim?** v1 picks the `sys.path` shim for zero-config runnability; long-term `pyproject.toml` + editable install is preferred. Not blocking v1.
- **(reorg) Should `prep_track.py` also auto-invoke `analysis/corner_analysis.py`?** v1 says no — they are two steps. The user can `&&` them on the command line. Auto-chaining is a convenience-only candidate for v2 (e.g. a `prep_all.py` umbrella script).
- **(v1.1) Should lap 1 also get MC?** v1.1 says no — lap 1 is deterministic (standing-start dynamics are dominated by traction/launch, not driver consistency); applying noise there muddies the lap-1 vs lap-2 comparison. Revisit if a user explicitly wants standing-start variance.

## 9. Alternatives considered

- **Driver model as full G-G-V-with-aggression-knobs.** Closer to the `ModelCreationSteps.txt` ideal, but heavy for v1 and obscures the input/output plumbing work. Deferred to v2.
- **Driver as a `skill_pct` knob baked into `Car` directly.** Simpler but pollutes `Car` with driving-skill concepts and makes it harder to add per-corner driver behaviour later. Rejected.
- **Convert CSV → JSON segments on the fly and reuse the existing pipeline.** Loses AI speed reference and gradient/elevation info. Rejected.
- **Drop the legacy built-in tracks entirely.** Cheaper to leave them; users already wire them. Keep as a fallback path with a soft-deprecation note in the README.
- **Skip the comparison PNG (CSV trace only).** PNG is the fastest QA signal; cheap to add. Kept.
- **Emit synthetic telemetry by sampling at uniform distance steps (no time resampling).** Simpler, but breaks the "drop-in real-AC-log" property — real AC logs are time-sampled at ~50 Hz. Rejected.
- **Three CLIs (`main.py` sim, `fit_driver.py` fit, `validate_lap.py` validate).** Collapsed into two CLIs on user feedback — validation became a `--validate-against` flag on `lap.py`.
- **Bake a per-track familiarity term into v1.** Premature: we have no calibration data for the learning curve yet. Rejected for v1.
- **(v1.1) Keep YAML and add a JSON loader alongside.** Rejected — the rest of the repo (tracks_config, corner JSON, sim telemetry CSV/PNG) is already JSON; YAML's comment + nesting advantages don't justify the PyYAML dep on a five-field flat schema. Clean break is simpler than dual-format support.
- **(v1.1) `driver_tau_s` default of 0.02 s (user's initial proposal).** Rejected — at 100 ms sampling, α = 0.02/(0.02+0.1) = 0.17 versus the bang-bang signal, smoothing is barely visible. Worse, 20 ms is below the documented human motor-reflex floor (~80–120 ms). Bumped to 0.12 s on Buddy recommendation; user accepted.
- **(v1.1) Apply low-pass before the trail-brake heuristic.** Rejected — the low-pass would smear the binary limit-label transitions before the heuristic can read them, so the heuristic's geometry (taper / ramp distances) would be applied to already-smoothed edges and produce a double-smoothed shape with the wrong total taper distance. Order is `heuristic → low-pass` (§14.3).
- **(v1.1) Emit two separate CSV files for the two laps.** Rejected — a single CSV with a `lap` column matches the (future) lake-style schema and avoids forcing every downstream consumer to handle a pair of files. AC-schema consumers that don't know about the `lap` column can ignore it (extra trailing column).
- **(v1.1) Run MC on both laps.** Rejected — lap 1 is standing-start; its dominant variance is launch-traction (not driver consistency), and the MC noise model is grip-utilisation-keyed. Restricting MC to lap 2 keeps the noise model honest. (Future: a launch-variance term on lap 1 if/when traction-loss is modelled.)
- **(reorg) Flat top-level layout (no `src/lap_estimator/` package).** Considered — for a ~10-file prototype, a flat layout is defensible and simpler. Rejected because (a) several simulator modules now exist (`telemetry`, `sim_telemetry`, `driver_fit`, `validate`, `report`, `driver`) so the top-level was getting cluttered, and (b) a package boundary cleanly distinguishes "library code" (importable, reusable) from "scripts" (`prep/`, `analysis/`, root CLIs). If ArchDev finds the `sys.path` bootstrap too painful, falling back to a flat `src/` (no nested package) is acceptable — but the prep/analysis separation stays.
- **(reorg) Have `prep_track.py` also produce `<layout>_corners.json`.** Tempting (one command, one result) but couples geometry extraction to corner classification. If `tracks_config.json` changes, every track has to be re-prepped, not just re-classified. Keeping them separate makes corner-threshold tuning a fast inner loop. Rejected.
- **(reorg) Keep `Corner_Analysis/` as the canonical corner-analysis location.** Rejected — it's untracked, ambiguously named (overlaps with `corner_analysis.py`), and depends on a merged-with-telemetry CSV. Replaced by `analysis/corner_analysis.py`.

## 10. Migration

- The current `segments`-based `Track` (built-ins + `from_json`) is **retained as a legacy code path**. No removal in this feature.
- Add a one-line deprecation note at the top of `track.py`'s `BUILTIN_TRACKS` dict pointing users at CSV-backed tracks as the preferred input.
- The third positional CLI argument is **required** for `lap.py`. Old invocations like `python main.py cars_csv/bmw_1m monza` will fail with a clear error pointing at the new CLI.
- **Script rename:** the existing `main.py` is renamed to `lap.py` at the repo root.
- **(v1.1) YAML → JSON driver migration.** ArchDev rewrites every existing `drivers/*.yaml` as the equivalent `.json` and deletes the `.yaml`. No loader fallback. PyYAML is removed from the project's requirements (README + any `requirements.txt` / `pyproject.toml`). See §6.2 for the file list.
- **(reorg) File moves** (ArchDev does these in a single commit, as part of the reorg):
  - `car.py`, `track.py`, `simulator.py` → `src/lap_estimator/`
  - `decode_acd.py`, `decode_track.py` → `prep/`
  - `main.py` → `lap.py` (rewritten per §6.6)
  - `Corner_Analysis/AClog/2026-04-07T135548_260Z_Tms_Lap2.csv` → `samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv` (copy and `git add`; do not delete the original — it's in untracked scratch).
  - `Corner_Analysis/corner_analysis.py` → `analysis/corner_analysis.py` (rewrite per §17, do not move the original; leave the user's scratch alone).
- **(reorg) `tracks_config.json`** is `git add`-ed (currently untracked per `git status`) with the `default_track` path updated to `tracks_csv/ks_nurburgring/layout_sprint_a.csv`.
- **(reorg) Existing CSVs under `tracks_csv/ks_nurburgring/`** remain in place. They are treated as canonical reference outputs of `prep_track.py` for the parity check in §11.11.

## 11. Acceptance criteria (for manual QA in Quix Cloud)

1. Running `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_gp_a_ideal_line.csv drivers/pro.json` succeeds, prints both lap times (§20), writes `tracks_csv/ks_nurburgring/layout_gp_a_ideal_line__Pro_sim_trace.csv` (with `lap` column), `..._sim_vs_ai.png`, and `..._sim_telemetry.csv` (with `lap` column).
2. The comparison plot shows the sim speed line generally tracking the AI speed line (within ~10–20% on most of the lap) for a reasonably matched car. Large deviations are tolerable; the point is visual sanity.
3. Replacing `drivers/pro.json` with `drivers/amateur.json` gives a slower lap time and a visibly lower sim-speed line at corner apexes.
4. Setting `consistency_sigma: 0.5` in the driver JSON produces stdout in the form `Lap 1 (standing): 2:18.103   |   Lap 2 (flying): 2:17.401 ± 0.487 (N=20)`. MC stats apply to lap 2 only.
5. Running with a `layout_*.csv` (no `_ideal_line`) succeeds; AI-speed line in the plot still comes from that file's `speed_ms` column.
6. Legacy invocation `python lap.py cars_csv/bmw_1m monza drivers/pro.json` still runs against the built-in Monza segments and prints lap times (no plot generated, or plot with only the sim line). Synthetic telemetry CSV is still emitted (using the segment-derived total length, both laps).
7. `python lap.py ... --no-plot` runs without matplotlib installed.
8. `python lap.py ... --no-telemetry` skips the synthetic telemetry CSV; everything else still runs.
9. Driver JSON missing `skill_pct`, or with `skill_pct > 1.0`, fails fast with a clear error. Missing v1.1 fields (`driver_tau_s` / `trail_brake_m` / `throttle_ramp_m`) **default cleanly** — not an error.
10. **Cross-track validation runs end-to-end as a `lap.py` flag.** `python lap.py cars_csv/bmw_1m tracks_csv/<track_b>/layout_*.csv drivers/<fitted_on_track_a>.json --validate-against <real_lap_on_track_b>.csv` succeeds, compares the real lap against **sim lap 2**, prints the real/predicted/delta block + verdict, writes the overlay PNG and per-bin delta CSV alongside the normal sim outputs.
11. **(reorg) `prep_track.py` parity check.** Running `python prep/prep_track.py tracks_in/ks_nurburgring` (after the user copies the AC folder into `tracks_in/`) produces `tracks_csv/ks_nurburgring/layout_sprint_a.csv` whose columns and row count match the committed reference file within a tight tolerance (column set identical; numeric columns equal to ≤1e-3 relative error per cell, ignoring trailing-row alignment). Equivalent check for `layout_sprint_a_ideal_line.csv`.
12. **(reorg) `prep_car.py` round-trip.** Running `python prep/prep_car.py cars_in/bmw_1m` (after the user copies the AC car folder in) produces `cars_csv/bmw_1m/data/` with the same `.ini` and `.lut` files as the existing reference. `lap.py` running on the freshly-produced `cars_csv/bmw_1m` matches the lap time from running on the pre-existing committed `cars_csv/bmw_1m` within numerical noise.
13. **(reorg) `corner_analysis.py` end-to-end.** Running `python analysis/corner_analysis.py tracks_csv/ks_nurburgring/layout_sprint_a.csv` succeeds, writes `tracks_csv/ks_nurburgring/layout_sprint_a_corners.json` (schema per §17.4), and emits `..._corner_map.png` and `..._speed_vs_position.png` next to the CSV. The JSON contains at least the corners visible in the existing `Corner_Analysis/track_corner_map.png` (manual visual check is acceptable), and every entry has `type` in `{hairpin, tight, sweeper, straight}`, `direction` in `{left, right, straight}`, and a sensible `min_radius_m`.
14. **(reorg) Corner analysis is telemetry-free.** `analysis/corner_analysis.py` runs with **no** AC telemetry file present (e.g. on a fresh clone, before any AC log is dropped in). No file in `samples/` or `Corner_Analysis/` is read.
15. **(reorg) `tracks_config.json` is the single source of truth for corner classification.** Changing `corner_thresholds.hairpin_max` from 60 to 40 in the config and re-running `corner_analysis.py` changes which corners get labelled `hairpin` vs `tight` in the output JSON. No corresponding edits to Python source are required.
16. **(v1.1) Driver-lag smoothing visible at τ=120 ms.** With `driver_tau_s: 0.12` (default) and `--telemetry-dt-ms 100`, the emitted `gas` and `brake` columns produce **no run of more than 5 consecutive samples stuck at exactly 0.0 or exactly 1.0 during a corner-exit window** (defined as the 4 s after the last sample where `limit_label == "brake"` or `"corner"`). With `driver_tau_s: 0.0`, the legacy bang-bang behaviour returns and this criterion does not apply.
17. **(v1.1) Trail-brake heuristic produces a visible linear taper.** With `trail_brake_m: 30.0` (default), a plot of `brake` vs `distance_m` over the last 30 m before each backward-pass-limited → corner-limited transition shows a roughly linear taper from 1.0 to 0.0 (within ±0.05 of a perfect line, allowing for the subsequent low-pass smoothing). With `trail_brake_m: 0.0`, the heuristic is bypassed and `brake` drops bang-bang.
18. **(v1.1) Two-lap default emits two laps; lap 2 faster within `consistency_sigma`.** Running `lap.py` with any of the bundled `drivers/*.json` produces a telemetry CSV whose `lap` column contains exactly `{1, 2}` and whose lap-2 time is **less than or equal to lap-1 time within `consistency_sigma`** (i.e. `lap_2_time ≤ lap_1_time + consistency_sigma` — the flying lap is always at least as fast as the standing lap modulo MC noise). The `--single-lap` flag reduces output to lap 1 only.
19. **(v1.1) YAML → JSON migration leaves no `.yaml` files in `drivers/`.** After ArchDev's reorg/migration commit, `git ls-files drivers/ | grep '\.yaml$'` returns no results. `drivers/` contains only `.json` files plus optional `.gitkeep`. PyYAML is no longer listed in the project's requirements.

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

**v1.1 note on sim-emitted telemetry.** When `fit_driver.py` consumes a sim telemetry CSV produced by `lap.py` (e.g. for the loop-closure check, §14.8), the file contains **both** laps. The fitter picks **lap 2** by default — it's the like-for-like flying-lap analogue of a real learned lap. A `--lap {1,2}` override is allowed for debugging; otherwise lap 2 is implicit.

### 13.2 Non-goals (fit tool)

- Fitting tyre / aero / engine parameters from telemetry — the AC car model is treated as ground truth.
- Per-corner skill profile (v2; see §13.9).
- Brake-vs-cornering skill split (v2).
- Combining multiple laps into one averaged driver (v2).
- Detecting / discarding off-track or invalid laps (out of scope).
- Live telemetry / streaming (file-based only).

### 13.3 CLI shape

```
python fit_driver.py <car_data_dir> <track_csv> <ac_telemetry_csv> <output_driver_json> \
                     [--ds 2.0] [--name <driver_name>] [--no-validate] [--no-plot] [--lap {1,2}]
```

- `<car_data_dir>` — same shape `lap.py` accepts (directory containing `engine.ini` or a parent with `data/`).
- `<track_csv>` — full track CSV (centerline or ideal line). The fit uses `distance_m` + `radius_m` from this file.
- `<ac_telemetry_csv>` — raw AC log with columns `timestamp_ms, gas, brake, distanceTraveled, speedKmh, normalizedCarPosition` (others ignored). **The same parser is used by `fit_driver.py` whether the file came from real AC telemetry or from the §14 synthetic emitter** — this is what enables the loop-closure check.
- `<output_driver_json>` — destination path; directories are created as needed. `.json` extension.
- `--name` — overrides the auto-generated `name` field in the JSON. Default: derived from telemetry filename stem.
- `--no-validate` — skip the post-fit sim validation pass.
- `--no-plot` — skip plotting (passed through to the validation sim).
- `--lap {1,2}` — when the telemetry CSV has a `lap` column (sim-emitted, §14.5), pick this lap. Default: `2`. Ignored on real AC logs (no `lap` column).

### 13.4 Input contract

**AC telemetry CSV** (confirmed columns from `samples/aclog/...`):

| Column | Unit | Used for |
|---|---|---|
| `timestamp_ms` | ms | Real lap time = `max - min` / 1000 (per lap if `lap` column present) |
| `gas` | 0..1 | Reserved (v2 brake/throttle split) |
| `brake` | 0..1 | Reserved (v2 brake/throttle split) |
| `distanceTraveled` | m | Merge key against track CSV `distance_m` |
| `speedKmh` | km/h | Converted to m/s for `v` |
| `normalizedCarPosition` | 0..1 | Sanity / corner labelling (not load-bearing) |
| `lap` (sim-emitted only) | int | Lap selector (§14.5); absent in real AC logs |

**Track CSV:** same contract as §7.1.

**Car dir:** same contract as §7.3.

### 13.5 Algorithm

All steps live in `driver_fit.fit_driver(car, track_df, telemetry_df) -> FitResult`. The CLI is a thin wrapper that loads inputs, calls this function, writes the JSON, and (unless `--no-validate`) runs the validation sim.

**Step 1 — Merge telemetry with track on distance.**
- Use `telemetry.merge_with_track(telemetry_df, track_df)` (the extracted helper from `Corner_Analysis/corner_analysis.py`).
- If the input has a `lap` column, filter to the chosen lap (default 2) before merge.
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
- Map to the JSON's "seconds" field: `consistency_sigma_seconds = round(consistency_sigma_raw / 0.03, 2)`.
- Clamp to `[0.0, 1.5]`.

**Step 7 — Worked example (illustrative):**
- Real lap time ≈ 1:42 s. Hairpin: 60 m radius, 75 km/h → `lat_g_obs ≈ 0.74 g`. `lat_g_max ≈ 1.05 g`. `util ≈ 0.70`. 85th percentile → `skill_pct ≈ 0.88`.

**Step 8 — Emit JSON.**
```json
{
  "name": "<driver_name>",
  "skill_pct": 0.88,
  "consistency_sigma": 0.45,
  "driver_tau_s": 0.12,
  "trail_brake_m": 30.0,
  "throttle_ramp_m": 40.0,
  "source": {
    "telemetry_csv": "samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv",
    "track_csv": "tracks_csv/ks_nurburgring/layout_sprint_a.csv",
    "car_data_dir": "cars_csv/bmw_1m",
    "real_lap_time_s": 102.135,
    "sim_lap_time_s": null,
    "delta_s": null,
    "fitted_at": "2026-05-13T14:22:01Z",
    "fit_version": "1"
  }
}
```
The v1.1 fields (`driver_tau_s`, `trail_brake_m`, `throttle_ramp_m`) are emitted at their defaults — v1.1 does not fit them from telemetry (would require a real driver-input model, deferred to v2). Hand-tunable post-fit.

**Step 9 — Validation pass (default on).**
- Run a two-lap sim with the freshly-emitted JSON and print:
  ```
  Real lap:           1:42.135
  Sim lap 2 (flying): 1:43.402
  Delta:              +1.267 s  (+1.24%)
  ```
- Patch `source.sim_lap_time_s` (the sim lap 2 time) and `source.delta_s`. If `|delta| > 3 s`, warn.

### 13.6 Output contract (JSON schema additions)

- `source` — optional object. Keys: `telemetry_csv`, `track_csv`, `car_data_dir`, `real_lap_time_s`, `sim_lap_time_s`, `delta_s`, `fitted_at`, `fit_version`. Hand-authored JSONs leave it absent.
- **v2 hook:** `corners` — optional list of `{corner_id, skill_pct, consistency_sigma}` overrides. Reserved key.

### 13.7 Module changes

- **New:** `src/lap_estimator/telemetry.py` — owns:
  - `read_ac_log(path)` — parses the AC CSV. Detects and exposes a `lap` column when present.
  - `merge_with_track(telem, track_csv_path_or_track) -> merged_frame` — lifts the CSV+log merge logic.
- **New:** `src/lap_estimator/driver_fit.py` — owns `fit_driver(car, merged_frame, *, straight_threshold_m=500.0, util_percentile=85) -> FitResult`.
- **New:** `fit_driver.py` (CLI at repo root) — argparse, loads inputs, calls the library, writes JSON, runs validation sim, patches JSON. Uses stdlib `json` (no PyYAML).
- **Touched:** `src/lap_estimator/driver.py` — `Driver.load` parses JSON, tolerates the `source` block, defaults the v1.1 fields when absent.
- **Untouched:** `car.py`, `simulator.py`, `report.py`.

### 13.8 `Corner_Analysis/` handling

The user's `Corner_Analysis/` folder is untracked scratch. The fit tool must **not** depend on it.

- ArchDev extracts the merge logic from `Corner_Analysis/corner_analysis.py` into `src/lap_estimator/telemetry.py`. The original file stays as-is — user scratch.
- Sample telemetry moves to `samples/aclog/*.csv` and is committed (§6.9 / §10).

### 13.9 Open questions / v2 candidates

- Per-corner skill profile, brake-vs-cornering split, multi-lap averaging, lap-to-lap consistency, percentile choice, skill ceiling clipping. (All v2.)
- **(v1.1)** Fit `driver_tau_s` from telemetry by measuring the lag between throttle-pedal step inputs and the corresponding longitudinal-G response. Requires real-AC `gas` / `brake` traces (currently reserved in the schema; not load-bearing). v2 candidate.
- **(v1.1)** Fit `trail_brake_m` / `throttle_ramp_m` from telemetry by measuring the actual taper distances around each corner-entry / corner-exit transition. Same data dependency as the previous bullet.

### 13.10 Acceptance criteria (fit tool)

1. Running `python fit_driver.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv drivers/ludvik_nurburgring_sprint.json` succeeds, writes a JSON that loads cleanly via `Driver.load`, prints the real-vs-sim-lap-2 delta block.
2. The JSON's `skill_pct` is in `(0, 1]`, `consistency_sigma` is in `[0, 1.5]`, and the v1.1 fields are present at defaults (`driver_tau_s: 0.12`, `trail_brake_m: 30.0`, `throttle_ramp_m: 40.0`). `source` is fully populated.
3. Running `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/ludvik_nurburgring_sprint.json` produces the same sim lap-2 time recorded in `source.sim_lap_time_s` (within numerical noise).
4. Validation delta `|sim_lap2 - real|` < 5 s on the bundled Nurburgring Sprint lap.
5. `--no-validate` skips the sim and leaves `source.sim_lap_time_s` / `source.delta_s` as `null`.
6. Missing required telemetry column fails fast.
7. Non-overlapping distance ranges produce a clear error.
8. The fit tool does **not** read or write inside `Corner_Analysis/`.
9. **(v1.1)** When the input telemetry CSV has a `lap` column, the fitter consumes lap 2 by default; `--lap 1` overrides.

---

## 14. Synthetic telemetry emission (addendum, v1 + v1.1)

### 14.1 Goal

After every sim run, emit a CSV that is **schema-identical to a real AC telemetry log** (plus a trailing `lap` column — v1.1) so downstream tools — first and foremost `fit_driver.py` — can consume sim output the same way they consume real laps.

### 14.2 Non-goals

- Human-realistic input traces. Reconstructed `gas`/`brake` are heuristic — see §14.3.
- Channels not in the AC sample (RPM, gear, steering, suspension, tyre temps).
- Wall-clock-anchored `timestamp_ms`. Sim telemetry starts at 0.
- Re-using sim's exact `ds` cadence. Output is **time-resampled** at fixed cadence (default 100 ms).

### 14.3 Gas/brake derivation pipeline (v1.1 — replaces v1's simple piecewise rule)

The emitted `gas` / `brake` traces are built in **three layers**, in this exact order:

1. **Layer 1 — limit-label rule (base / unchanged from v1).** Per sample, look up the simulator's per-point binding-limit label and emit:
   - `accel` → `gas = 1.0, brake = 0.0`
   - `brake` → `gas = 0.0, brake = 1.0`
   - `corner` → `gas = required_drive_force / car.max_traction_force(v)`, `brake = 0.0`, clipped to `[0, 1]`.

   This produces a piecewise trace with sharp transitions at every limit-label change.

2. **Layer 2 — corner-shape heuristic (v1.1, applied after Layer 1, before Layer 3).** Walks the `limit_label` sequence by distance and applies two linear ramps:
   - **Trail-brake taper.** Find every transition where `limit_label` leaves a `brake` region (i.e. transitions `brake → corner` or `brake → accel`). For the `trail_brake_m` metres **immediately preceding** the transition, replace the constant `brake = 1.0` with a linear taper from `1.0` at distance `(transition - trail_brake_m)` down to `0.0` at the transition point. The taper is applied in the distance domain (not time), interpolated onto the output time grid.
   - **Throttle ramp-up.** Find every transition where `limit_label` leaves a `corner` region into `accel` (i.e. `corner → accel`). For the `throttle_ramp_m` metres **immediately following** the transition, replace the constant `gas = 1.0` with a linear ramp from `corner_exit_gas_value` (the corner-region partial-throttle at the transition point) up to `1.0` at distance `(transition + throttle_ramp_m)`. Linear in distance.
   - Either field set to `0.0` disables that heuristic (no-op). Both default-on with the §6.2 defaults.
   - **Order matters within Layer 2:** trail-brake first, then throttle-ramp. They operate on disjoint regions, so order is not load-bearing, but ArchDev should apply them in the order listed for clarity.

3. **Layer 3 — driver-lag low-pass (v1.1, applied after Layer 2, before CSV write).** Run a 1st-order IIR low-pass filter independently on `gas` and `brake`:
   ```
   y[n] = y[n-1] + α · (x[n] - y[n-1])
   α   = dt / (driver_tau_s + dt)
   ```
   where `dt` is the output sample interval in seconds (e.g. 0.1 s for the default 100 ms cadence). Initial condition `y[0] = x[0]`.
   - `driver_tau_s = 0.0` bypasses the filter (Layer 2 output passes through unchanged → legacy bang-bang behaviour when `trail_brake_m = throttle_ramp_m = 0.0` as well).
   - Default `driver_tau_s = 0.12 s` chosen as a human pedal-modulation time constant (motor-reflex floor ≈ 80–120 ms; user accepted Buddy's recommendation over an initial 20 ms guess that was below the reflex floor and gave near-zero smoothing at 100 ms sampling).

**Order summary:** `limit-label rule → corner-shape heuristic → driver-lag low-pass → CSV write`. The heuristic shapes the corner geometry while the limit-label transitions are still sharp (so taper/ramp distances are meaningful); the low-pass then smooths the residual edges (transition between linear ramps and their neighbouring constant regions, and any other sharp features). Inverting the order would smear the limit-label transitions before the heuristic can read them, producing double-smoothed shapes with the wrong total taper distance.

**Worked example (single corner-entry / mid-corner / corner-exit window):**

Suppose the simulator binds as: `accel` (straight) → `brake` (50 m braking zone) → `corner` (slow apex, 80 m) → `accel` (exit straight). Sample interval `dt = 0.1 s` (10 Hz). Driver defaults: `tau=0.12`, `trail=30`, `ramp=40`. Corner-region partial throttle ≈ `0.35`.

| Stage | Approaching brake zone | Last 30 m of brake zone | Apex (mid-corner) | First 40 m after corner | Far past corner |
|---|---|---|---|---|---|
| Layer 1 (limit-label) | `gas=1.0, brake=0.0` | `gas=0.0, brake=1.0` | `gas=0.35, brake=0.0` | `gas=1.0, brake=0.0` | `gas=1.0, brake=0.0` |
| Layer 2 (heuristic) | `gas=1.0, brake=0.0` | `brake` tapers `1.0 → 0.0` linearly over the 30 m; `gas=0.0` | `gas=0.35, brake=0.0` | `gas` ramps `0.35 → 1.0` linearly over the 40 m; `brake=0.0` | `gas=1.0, brake=0.0` |
| Layer 3 (low-pass τ=0.12) | unchanged (already constant) | smoother taper, residual lag ≈ 120 ms; brake never **exactly** 1.0 or 0.0 mid-taper | small lag carried in from the brake → corner transition; `gas` settles to `≈0.35` within ~0.3 s | smoother ramp; `gas` never **exactly** 0.35 or 1.0 mid-ramp | unchanged (constant) |

Net effect on the emitted CSV: brake-zone traces look like a half-second of saturation then a soft taper over the last ~3 s of approach; corner-exit gas comes up over ~0.4–0.5 s instead of instantaneously; nothing is stuck at exactly 0 or 1 for more than ~0.3 s through the entry/exit transitions. Acceptance §11.16 / §11.17 enforce this quantitatively.

### 14.4 Decisions baked in (v1 + v1.1)

1. **Schema:** AC schema columns plus a trailing `lap` column — `timestamp_ms, gas, brake, distanceTraveled, speedKmh, normalizedCarPosition, lap`. AC-schema-only consumers ignore the trailing column.
2. **Cadence:** fixed 100 ms by default. CLI flag `--telemetry-dt-ms` overrides.
3. **Time origin:** `timestamp_ms` starts at 0 **and continues monotonically across the lap boundary** (§20). Lap 2's first sample's `timestamp_ms` = lap 1's last sample's `timestamp_ms + telemetry_dt_ms`.
4. **`normalizedCarPosition`:** `(distance_m_within_current_lap) / track.total_length_m`, clipped to `[0, 1)`. Resets at the lap boundary.
5. **`distanceTraveled`:** resets to 0 at the start of lap 2.
6. **Resampling:** linear interpolation in the time domain.
7. **`gas` / `brake` reconstruction:** the three-layer pipeline in §14.3 (limit-label → corner-shape heuristic → driver-lag low-pass).
8. **Emission is default-on.** `--no-telemetry` opts out.
9. **File path:** `<track_dir>/<track_stem>__<driver_name>_sim_telemetry.csv` — **single file with two laps**, not two files. Matches lake-style schema.
10. **Module placement:** new `src/lap_estimator/sim_telemetry.py`. `simulator.py` is not extended with I/O.
11. **Driver-input model (v1.1):** the trail-brake/throttle-ramp heuristic + low-pass replace the deferred-to-v2 "low-pass filter on inputs" hook. Further model upgrades (reaction time, prediction, separate brake-vs-throttle skill) remain v2.
12. **Two CLIs, not three.** Validation collapsed into `lap.py --validate-against`.
13. **Local-file-only prototype.** No DB / remote storage in v1.

### 14.5 CLI surface (additions to `lap.py`)

```
[--no-telemetry] [--telemetry-dt-ms 100] [--single-lap]
```

`--single-lap` (v1.1, §20) reduces output to lap 1; the `lap` column is still present (constant `1`).

### 14.6 Output schema (matches AC plus `lap` column)

```
timestamp_ms,gas,brake,distanceTraveled,speedKmh,normalizedCarPosition,lap
```

### 14.7 Resampling algorithm

Walks the sim step→time map (lap 1 then lap 2, with monotonic time), builds a uniform output grid per lap (timestamps continue across), linear-interp on distance and speed, nearest-neighbour on binding label, reconstructs gas/brake per §14.3.

### 14.8 Module: `src/lap_estimator/sim_telemetry.py`

```python
def write_synthetic_log(
    sim_result,
    car,
    driver,                              # v1.1: needed for tau / trail / ramp
    track_total_length_m: float,
    output_path: str,
    *,
    telemetry_dt_ms: int = 100,
) -> None: ...
```

Pure function; no global state; numpy + stdlib `csv` only. Implements all three layers of §14.3 internally.

### 14.9 Loop-closure acceptance criterion

Fit a driver → emit sim telemetry (two laps) → re-fit a driver from the synthetic telemetry's **lap 2**. Pass criteria:
- `|skill_pct_loop - skill_pct_real| / skill_pct_real <= 0.05`.
- `|sim_lap_time_loop - sim_lap_time_real| <= 1.0` s (both measured on lap 2).
- `skill_pct_loop` is not trivially 1.0.

### 14.10 Acceptance criteria (sim-telemetry emission)

1. Default `lap.py` produces `<track_stem>__<driver_name>_sim_telemetry.csv` next to the track CSV.
2. Header byte-matches `timestamp_ms,gas,brake,distanceTraveled,speedKmh,normalizedCarPosition,lap`.
3. `timestamp_ms` starts at 0 and increments by exactly `--telemetry-dt-ms`, monotonically across the lap boundary (no reset).
4. `distanceTraveled` monotonic non-decreasing **within each lap**, resets to ~0 at the start of lap 2, ends within 1 m of `track.total_length_m` for each lap.
5. `normalizedCarPosition` stays in `[0, 1)` and resets at the lap boundary.
6. `gas` and `brake` never both > 0.05 (allowing for low-pass overshoot near transitions).
7. Trivial straight: `gas ≈ 1.0, brake ≈ 0.0` (after low-pass settles).
8. `--no-telemetry` skips the file.
9. `--telemetry-dt-ms 50` ≈ 2× rows; `200` ≈ ½× rows. Filter coefficient α adjusts accordingly (per §14.3 Layer 3 formula).
10. Loop closure passes on the bundled Nurburgring Sprint lap (re-fitting against lap 2 of the synthetic log).
11. **(v1.1)** With `driver_tau_s = 0` and `trail_brake_m = 0` and `throttle_ramp_m = 0`, output matches the v1 bang-bang behaviour byte-for-byte (modulo the new `lap` column).

### 14.11 Open questions / v2 candidates

Full driver-input motor-control model (reaction time, prediction, mistakes), steering channel, extra channels, wall-clock timestamps, multi-lap (>2) output, cadence calibration. (All v2.)

---

## 15. Cross-track validation workflow (addendum, v1)

### 15.1 Goal — the actual point of the tool

Predict how a known driver (fit on Track A) will perform on a new track (Track B, same car), then validate against a real AC lap on Track B.

The CLI is `lap.py --validate-against` — **not** a separate script. v1.1: validation compares the real lap against **sim lap 2** (the flying lap, §20).

### 15.2 End-to-end workflow

1. Drive Track A in AC, capture a learned-lap telemetry.
2. `python fit_driver.py <car_dir> <track_a_csv> <real_lap_on_a>.csv drivers/<name>.json`.
3. Choose Track B.
4. `python lap.py <car_dir> <track_b_csv> drivers/<name>.json` → predicted lap times (both laps).
5. Drive Track B in AC for ~10 laps, capture telemetry.
6. `python lap.py <car_dir> <track_b_csv> drivers/<name>.json --validate-against <real_lap_on_b>.csv` → real-vs-sim-lap-2 delta.

### 15.3 CLI shape

```
python lap.py <car_data_dir> <track_csv> <driver_json> \
              --validate-against <real_telemetry_csv> \
              [--ds 2.0] [--no-plot] [--bin-m 100] [--per-corner] [--single-lap]
```

`--single-lap` (v1.1) makes validation compare against lap 1 instead of lap 2. Default (no flag) targets lap 2.

### 15.4 Algorithm

Lives in `validate.validate_lap(car, track, driver, sim_result, real_telem) -> ValidationResult`. `lap.py` calls it post-sim (the same sim result is reused — no second sim pass). Selects **lap 2** from the sim result by default; resample real telemetry onto sim lap-2's distance grid; bin by 100 m (or by corner spans if `--per-corner`); compute per-bin and headline deltas; categorise.

### 15.5 Outputs

**Stdout:**
```
Track:               tracks_csv/brands_hatch/layout_indy_ideal_line.csv
Driver:              drivers/ludvik_nurburgring_sprint.json  (fit on layout_sprint_a)
Real lap:            1:24.812
Sim lap 2 (flying):  1:26.301  (predicted)
Delta:               +1.489 s  (+1.76%)
Verdict:             GOOD    (|delta| < 3 s on a 84.8 s lap)
```

Verdicts: `GOOD` (|d| < 3 s AND |%| < 5), `LOOSE` (5–10 %), `BAD` (> 10 %).

**Files (next to track CSV):**
- `<track_stem>__<driver_name>_validation_overlay.png`.
- `<track_stem>__<driver_name>_validation_bins.csv`: `bin_start_m, bin_end_m, kind, t_sim_s, t_real_s, delta_s, v_avg_sim_kmh, v_avg_real_kmh`.

### 15.6 Limitations

v1 explicitly assumes `skill_pct` / `consistency_sigma` are track-agnostic. Practical consequence: unlearned Track B → negative delta (sim faster). v2 candidates: per-track familiarity modifier, track-similarity score, automatic learning-curve detection, validation-driven calibration.

### 15.7 Module placement

- **New:** `src/lap_estimator/validate.py` — `validate_lap(car, track, driver, sim_result, real_telem_path_or_frame, *, target_lap=2) -> ValidationResult`. No I/O of its own.
- **No new CLI.** Invoked via `lap.py --validate-against`.
- **Reused:** `telemetry.py`, `simulator.py`, `report.py`.

### 15.8 Acceptance criteria (validate flow)

1. End-to-end flag invocation runs; stdout block + verdict; overlay PNG + bins CSV produced.
2. Verdict matches §15.5 thresholds.
3. Bins CSV well-formed; sums match the lap-2 lap time within 0.1 s.
4. Overlay PNG produced unless `--no-plot`.
5. Real-telemetry parsing reuses `telemetry.py`.
6. `--validate-against` is read-only on its inputs.
7. Without the flag, no validation outputs.
8. **Soft acceptance** — on the user's actual Nurburgring-fit → other-track run, `|delta_s|` < ~3 s on a 2–3 min lap (real vs sim lap 2).
9. **(v1.1)** Default validation targets sim lap 2; `--single-lap` switches to sim lap 1 (warning printed: "comparing real flying lap against sim standing lap").

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

## 18. MF4 telemetry output (v2, PLANNED — not implemented)

### 18.1 Status

**PLANNED for v2.0. NOT implemented in v1.** This section captures the design so ArchDev can pick it up cleanly when the user gives the go-ahead. v1 ships CSV-only (§14); MF4 is purely additive — no v1 behaviour changes.

### 18.2 Goal & motivation

Emit the same sim telemetry that §14 writes as CSV **also** as an **ASAM MDF v4** file (`.mf4`) — the de-facto automotive measurement standard. Two reasons:

1. **Bridge while the DB is offline.** The user's longer-term plan is a time-series DB (DuckDB / similar) for telemetry. Until that lands, MF4 is a portable, well-defined container that survives indefinitely on disk and ETLs cleanly into any time-series store later.
2. **Tool interoperability.** MF4 is consumable by `asammdf` (Python), MATLAB, Vector CANape, ETAS INCA, and most OEM telemetry workbenches. Emitting MF4 makes the simulator's output useful well beyond this repo without inventing a new format.

The driver request: *"OK, until database is on. Think about creating MF4 file with telemetry, we will need for further experiments."* — this section is the answer.

### 18.3 Non-goals (v2.0)

- **Replacing CSV.** Both formats are emitted side-by-side. CSV remains the default; MF4 is opt-in.
- **CAN-bus signals.** No CAN frame channels, no DBC mapping. Plain numeric channels only.
- **Multi-source / multi-rate channels.** Single source, single rate — all channels share one master time channel.
- **Compressed / encrypted MF4.** Default uncompressed `.mf4`. asammdf's compression flag is a v2.1 candidate.
- **Human-realistic input traces.** Same gas/brake reconstruction as §14 — the MF4 doesn't fix realism, it just packages the same data differently.
- **Read-side MF4 ingest in v2.0.** Deferred to v2.1 (§18.9).

### 18.4 Decisions baked in

These are the calls made now; list-form so they're easy to challenge later.

1. **Format: ASAM MDF v4 (`.mf4`).** Industry standard for automotive measurement. Parquet considered and rejected (not native to automotive tooling). HDF5 considered and rejected (less common in the AC / motorsport-telemetry community than MF4).
2. **Library: `asammdf` (PyPI, MIT-licensed, actively maintained).** Used purely as an optional dependency, gated behind the `--mf4` CLI flag. If `asammdf` is not importable and the user passed `--mf4`, the CLI fails with exit code 2 and the message `MF4 output requires asammdf; install with: pip install asammdf`.
3. **Scope: emit alongside CSV, not instead of it.** The same `SimResult` feeds both `sim_telemetry.write_synthetic_log` (CSV) and the new MF4 emitter. No CSV behaviour changes.
4. **Output path:** `<track_dir>/<track_stem>__<driver_name>_sim_telemetry.mf4` — sits next to the existing `_sim_telemetry.csv`, same stem.
5. **Channel set (v2.0 minimum):** match the CSV columns one-to-one, plus master time channel:

   | Channel | Unit | Dtype | Notes |
   |---|---|---|---|
   | `time` | s | float64 | Master channel (timestamp_ms / 1000.0) |
   | `speed_ms` | m/s | float64 | Primary speed (sim's native unit) |
   | `speed_kmh` | km/h | float32 | Convenience, derived |
   | `distance_m` | m | float64 | = `distanceTraveled` in CSV |
   | `gas` | 0..1 | float32 | Reconstructed per §14.3 item 6 |
   | `brake` | 0..1 | float32 | Reconstructed per §14.3 item 6 |
   | `normalized_position` | 0..1 | float32 | = `normalizedCarPosition` in CSV |

   Optional **nice-to-have** extras if trivial to derive at emit time (do not block v2.0 if costly):
   - `lat_g` (g) — `v² / radius_m / 9.81` from sim's per-point state.
   - `long_g` (g) — derived from `dv/dt`.
   - `rpm` (1/min) — from sim's gear + speed model, if exposed.
   - `gear` (int) — from sim's gear selection, if exposed.

   These four are flagged as "include if the data is already on `SimResult`; otherwise punt to v2.1".

6. **CLI surface change on `lap.py`:**
   - `--mf4` — boolean flag, default **off**. Off keeps v1 byte-compatible behaviour.
   - `--mf4-dt-ms <int>` — optional cadence override for the MF4 file. Default: same value as `--telemetry-dt-ms`. Letting the two diverge is rare but cheap to support.
   - No new flag added if `--no-telemetry` is set — `--no-telemetry` already suppresses all telemetry emission. Combination `--no-telemetry --mf4` is a CLI error.
7. **Read-side (v2.1, deferred):** `fit_driver.py` and `lap.py --validate-against` should eventually accept `.mf4` real-telemetry inputs in addition to `.csv`. Motivation: MoTeC, AiM, RaceCapture, and most professional logger tooling export MF4. Adding a thin `asammdf`-backed reader in `telemetry.read_ac_log` (dispatched by file extension) makes the tool useful to non-AC users. Deferred from v2.0 to keep that release small.
8. **Metadata block (file header / global comment):** the MF4 must carry the same provenance the fitted-driver JSON carries:
   - Source car name and `cars_csv` path.
   - Source track name, layout, and `tracks_csv` path.
   - Driver name + `skill_pct` + `consistency_sigma`.
   - Generator: `LapTimeEstimator <git_short_sha or "dev">`.
   - Generation timestamp (ISO-8601 UTC).
   - Cross-reference: the matching `_sim_telemetry.csv` path (relative to the MF4 file's directory).

   Exact placement (file-level comment, channel-group comment, or both) is ArchDev's call within `asammdf`'s API surface; the constraint is that it must round-trip through `asammdf.MDF.load(...)` and be inspectable from the Python API without parsing custom strings.
9. **Acceptance criteria (v2.0):**
   - `python lap.py ... --mf4` produces `<stem>_sim_telemetry.mf4` next to `<stem>_sim_telemetry.csv`.
   - Opening the `.mf4` with `asammdf.MDF(...)`: channel count matches §18.4 item 5 (master + 6 required, plus any of the 4 optionals that landed); sample count equals the CSV row count; `max(speed_kmh)` from the MF4 equals `max(speedKmh)` from the CSV to within 1 km/h.
   - Without `asammdf` installed, `python lap.py ... --mf4` fails with exit code 2 and the install-hint message (§18.4 item 2).
   - CSV emission unchanged when `--mf4` is omitted (regression check on §14.9).
   - `--no-telemetry --mf4` is rejected by argparse with a clear error.
10. **Non-goals for v2 (re-stated for the Decisions block):** no CAN signals, no multi-source channels, no per-channel sample-rate variation, no compression, no MF4 read-side in v2.0. Plain single-rate single-source numeric channels only.

### 18.5 Module placement

- **New file:** `src/lap_estimator/sim_telemetry_mf4.py`.
  - Single public function: `write_synthetic_mf4(sim_result, car, track_total_length_m, output_path, *, telemetry_dt_ms=100, metadata: dict | None = None) -> None`.
  - Imports `asammdf` lazily inside the function so the module is importable on systems without `asammdf` installed (only the call fails).
  - Reuses the same time-resampling helper as `sim_telemetry.py` — extracted into a small `_resample.py` if duplication gets annoying. ArchDev's call.
- **`sim_telemetry.py` stays unchanged.** The MF4 emitter is a sibling, not a refactor.
- **`lap.py`** gains the two flags and a one-line dispatch: if `--mf4`, also call `write_synthetic_mf4` after the CSV write.

### 18.6 Read-side (v2.1, deferred)

When the user is ready, extend `src/lap_estimator/telemetry.py` so `read_ac_log(path)` dispatches by extension:

- `.csv` → existing parser.
- `.mf4` → new `asammdf`-backed reader that maps channels back to the AC-schema column names (`timestamp_ms, gas, brake, distanceTraveled, speedKmh, normalizedCarPosition`).

This is the inverse of §18.5 and unlocks `fit_driver.py` / `--validate-against` for MoTeC / AiM-style MF4 inputs. Out of scope for v2.0.

### 18.7 Open questions

- **`asammdf` as a hard dependency vs optional?** v2.0 keeps it optional (per §18.4 item 2). If the user wants `--mf4` to be the default, we'd promote it to a hard dep — change `README.md` requirements and drop the install-hint branch. Worth revisiting once MF4 has been used in anger.
- **Metadata convention.** `asammdf` supports arbitrary string comments and a structured `header` block, but there's no widely-adopted convention for "this MF4 came from a simulator". Following DBC (CAN database) or ASAP2 (calibration) feels like overkill for the channel set in §18.4; v2.0 uses plain key/value strings in the file-level comment. Flag for revisit if downstream tooling needs more structure.
- **Channel naming convention.** The table in §18.4 uses snake_case + `_unit` suffixes (e.g. `speed_ms`, `speed_kmh`). Matches AC's natural-keys but diverges from common automotive practice (e.g. `Speed_kph`, `Throttle_pct`). v2.0 picks snake_case for one-to-one CSV mapping; revisit if `asammdf`-using consumers complain.
- **Should the file extension `.mf4` vary?** ASAM also defines `.mdf` (v3) and the umbrella `.dat`. v2.0 uses `.mf4` exclusively (v4 is what `asammdf` produces by default).

### 18.8 References

- ASAM MDF v4 spec — `https://www.asam.net/standards/detail/mdf/`.
- `asammdf` library — `https://github.com/danielhrisca/asammdf`.
- §14 — Synthetic telemetry CSV emission (the source of truth for cadence, gas/brake reconstruction, and sample alignment).
- §7.7 — AC-schema CSV columns that the MF4 channels map to one-to-one.

---

## 19. Backlog — complex physics & tyre damage (research, v3)

### 19.1 Status

**BACKLOG / research.** Neither item below is scheduled. This section is the answer to the user's "can Buddy in the meantime dig some info, how we can do?" — it captures the recommended approach, what AC already exposes, the architectural impact, and a sequencing recommendation against the v2 (§18) work. **No code changes follow from this section in the current branch.** v1 stays point-mass with isotropic μ; v2 adds MF4 + tyre-state-as-grip-modifier; v3 is the slip-based rebuild.

### 19.2 Item A — Slip-based tyre + drift / oversteer / understeer dynamics

#### 19.2.1 Motivation

The current sim (`src/lap_estimator/simulator.py`) is a distance-stepped 3-pass point-mass. `car.tyre_grip_lateral(v)` returns a single μ_y; lateral and longitudinal grip are treated independently and there is no notion of slip. As a result the sim cannot represent:

- understeer (front axle saturates first → car ploughs straight on)
- oversteer / drift (rear axle saturates first → yaw moment, big slip-angle)
- the friction-ellipse trade between braking and turning
- weight transfer transients on entry / mid-corner / exit

These are the differentiators between a fast lap and a slow one for any driver beyond the grip limit. None of them are visible to a point-mass.

#### 19.2.2 Recommended approach — Pacejka "Magic Formula"

The industry-standard tyre model is Pacejka's Magic Formula (MF), in either the "MF5.2" simple form or the modern "MF6.x / MF-Tyre/MF-Swift" form. The minimum viable surface for a lap-time sim:

- **Inputs per tyre:** vertical load `Fz`, slip angle `α`, slip ratio `κ`, camber (optional in v3.0).
- **Outputs per tyre:** lateral force `Fy(α, Fz)`, longitudinal force `Fx(κ, Fz)`, aligning moment `Mz` (optional).
- **Combined slip:** friction ellipse `(Fx/Fx_max)² + (Fy/Fy_max)² ≤ 1` (or full MF combined-slip if we go that far).

This replaces `car.tyre_grip_lateral(v)` / `car.tyre_grip_longitudinal(v)` with a per-tyre force lookup, parameterised by `(α, κ, Fz)` rather than `v`.

#### 19.2.3 Architectural implication — distance-stepped sim cannot host this

The 3-pass distance-stepped solver assumes one decision per `ds`, no state carried between points beyond `v`. Slip angle, slip ratio, yaw rate, body slip, per-wheel angular velocity, suspension travel — these are **states that evolve in time, not in distance**, and they depend on inputs (steer, throttle, brake) that a driver chooses, not on instantaneous grip limits.

The v3 sim is therefore a **time-domain ODE-integrated vehicle model**:

- **State vector (minimum):** `(x, y, ψ, v_x, v_y, ω_yaw, ω_wheel_FL, ω_wheel_FR, ω_wheel_RL, ω_wheel_RR)` — 10 states. Plus a driver-control state (steer angle, throttle, brake) updated by a control loop.
- **Integrator:** RK4 at fixed `dt` (1–5 ms). Sensitive at very low speed (tyre relaxation length matters) — may need an implicit step or a low-speed regularisation.
- **Driver becomes a control loop**, not a grip-utilisation %. Minimum control architecture: PID on lateral error vs the racing line, look-ahead preview distance, slip-target setpoint, separate throttle / brake control loops. This is essentially what AC's AI does — there is prior art.
- **Sim lives alongside the v1 point-mass**, not replacing it: the point-mass is fast (sub-second per lap) and good for sweeps; the slip-based sim is slow (seconds–minutes per lap depending on integrator) and good for behavioural fidelity. Two `simulate()` entry points, same `SimResult` shape (mostly).

#### 19.2.4 AC signals that make this feasible (from `C:/repos/SensorNotation/sensor_dictionary_merged.json`)

AC's shared-memory dump already exposes most of the channels needed to **validate** a slip-based sim (we'd not be flying blind). Exact keys from the dictionary:

**Body-frame kinematics:**
- `velocity_x`, `velocity_y`, `velocity_z` — world-frame velocity.
- `localVelocity_x`, `localVelocity_y`, `localVelocity_z` — body-frame velocity. **`localVelocity_x` / `localVelocity_z` give body-slip angle directly** (`β = atan2(v_y_body, v_x_body)`).
- `localAngularVel_x`, `localAngularVel_y`, `localAngularVel_z` — yaw / pitch / roll rates. **`localAngularVel_y` is yaw rate** (the headline drift / oversteer signal).
- `accG_x`, `accG_y`, `accG_z` — body-frame accelerations in g.
- `heading`, `pitch`, `roll` — body orientation.

**Driver inputs:**
- `steerAngle`, `gas`, `brake`, `clutch`, `gear`.

**Per-wheel state (FL/FR/RL/RR):**
- `wheelSlipFL/FR/RL/RR` — AC's combined slip metric. **Note: not pure α or κ** — it's a derived `sqrt(α² + κ²)` style number. Useful for validation, not directly invertible to (α, κ).
- `wheelLoadFL/FR/RL/RR` — vertical load `Fz` per wheel. This is the ground truth weight-transfer signal we currently approximate via `cg_front` and downforce.
- `wheelAngularSpeedFL/FR/RL/RR` — ω_wheel per corner. Combined with `localVelocity_x` and tyre radius, gives slip ratio `κ` directly.
- `suspensionTravelFL/FR/RL/RR` — suspension state.
- `camberRADFL/FR/RL/RR` — current camber in radians.
- `tyreContactPoint{FL,FR,RL,RR}_{x,y,z}` — contact patch position in world frame.
- `tyreContactNormal{FL,FR,RL,RR}_{x,y,z}` — surface normal at contact patch (encodes road camber/banking).
- `tyreContactHeading{FL,FR,RL,RR}_{x,y,z}` — wheel heading at contact patch. **`steerAngle` plus this gives the tyre heading, which combined with body velocity gives slip angle `α` per wheel.**

This is enough to **derive ground-truth `(α, κ, Fz)` per wheel** from a real AC lap without needing to instrument AC itself. The fit pipeline becomes: log → compute per-wheel `(α, κ, Fz, Fx, Fy)` → fit MF coefficients to AC's behaviour for our `tyres.ini` compound.

**Coverage check vs the original ask:** the user asked specifically about `steerAngle`, slip angle/ratio, `accG_x/y/z`, tyre temps, tyre pressures, tyre wear, suspension travel, contact patch. All present except a directly-named `slipAngle` / `slipRatio` channel — AC exposes them indirectly via `wheelSlipFL/FR/RL/RR` (combined) plus the contact-frame channels above, which is sufficient to reconstruct both. Worth flagging: AC's `wheelSlipFL` semantics are not documented as pure Pacejka inputs; ArchDev should sanity-check by comparing reconstructed α/κ against a known constant-radius corner before committing to the model.

#### 19.2.5 Pacejka coefficient sourcing — `tyres.ini` is not directly compatible

AC's `tyres.ini` (verified against `cars_csv/bmw_1m/tyres.ini`) uses a **simplified internal model** that is **not Pacejka MF**. The relevant parameters AC exposes per compound (front/rear sections):

- `DY0`, `DY1` — peak lateral grip coefficient and load sensitivity (currently parsed by `car.py` line 75–80).
- `DX0`, `DX1` — peak longitudinal grip coefficient and load sensitivity (parsed).
- `SPEED_SENSITIVITY` — grip falloff with speed (parsed).
- `FRICTION_LIMIT_ANGLE` — slip angle (degrees) at which peak `Fy` occurs. **This is the AC equivalent of MF's `B*C*D` curvature — gives us α_peak directly.** Example values: 8.28° front / 8.09° rear for the bmw_1m street compound.
- `XMU` — friction at very high slip (saturation tail).
- `FALLOFF_LEVEL`, `FALLOFF_SPEED` — describe how `Fy` decays past the peak (MF's `E` shape parameter is analogous).
- `LS_EXPY`, `LS_EXPX` — non-linear load exponents (Pacejka's `pCx2 / pCy2` family).
- `DY_REF`, `DX_REF`, `FZ0` — reference load + reference μ. These let us non-dimensionalise.
- `RELAXATION_LENGTH` — tyre lag (matters for low-speed solver stability).
- `CAMBER_GAIN`, `DCAMBER_0`, `DCAMBER_1` — camber sensitivity.
- `FLEX`, `FLEX_GAIN`, `PRESSURE_FLEX_GAIN` — tyre carcass flex (couples slip with pressure).
- `BRAKE_DX_MOD`, `CX_MULT` — longitudinal asymmetries.

Conversion strategy (proposed):
1. **Fit MF5.2 coefficients to AC's behaviour empirically.** Drive a known set of test manoeuvres in AC (constant-radius cornering at different speeds, straight-line braking at different decelerations, combined load corner-on-brake), log the channels in §19.2.4, derive per-wheel `(α, κ, Fz, Fx, Fy)`, fit Pacejka. This is the robust path.
2. **Bootstrap from `tyres.ini`** for an initial guess: use `DY0` for MF's `D_y`, `FRICTION_LIMIT_ANGLE` to derive `B_y * C_y`, `FALLOFF_LEVEL` for `E_y`, `LS_EXPY` for load sensitivity. Same for longitudinal. This gives us a sensible starting point but won't match AC perfectly — fitting (step 1) closes the gap.
3. **Don't bother trying to map AC's per-field semantics directly to MF.** Two different model families; the math doesn't line up cleanly. The fit approach is faster than the algebra.

#### 19.2.6 Driver model implications — v3 schema, not v1

The current v1 driver JSON (`name`, `skill_pct`, `consistency_sigma`, plus the v1.1 `driver_tau_s` / `trail_brake_m` / `throttle_ramp_m` heuristics) is meaningless for a slip-based sim. A slip-based driver needs at minimum:

- **Preview distance** (how far ahead the driver looks at the racing line). Calibrated from telemetry by correlating steer with look-ahead curvature.
- **Lateral PID gains** (`Kp`, `Ki`, `Kd` on lateral error vs the line).
- **Slip-target setpoint** (`α_target` — how close to peak slip the driver is willing to operate). This is the actual "skill" knob — a pro runs at α_peak ± 1°, an amateur stays well inside.
- **Brake-release rate** (trail-braking aggression).
- **Throttle-application rate** (corner-exit traction handling).
- **Reaction-time delay** (low-pass on inputs).
- **Mistake / consistency model** (occasional setpoint overshoot, modelled as a noise process on α_target).

This is a **v3 driver schema** — incompatible with v1's two scalars. Migration: v1 JSONs continue to work with the point-mass sim; v3 introduces `drivers_v3/<name>.json` with the slip-driver schema. The fit tool (`fit_driver.py`) gets a `--model {point-mass, slip}` flag.

#### 19.2.7 Effort estimate

**Multi-week build, not a tweak.** Headline scope:

- Vehicle dynamics core (state + ODE integrator + force aggregation): ~1 week.
- Tyre model (Pacejka MF + load + camber + relaxation): ~1 week, plus a coefficient-fit pass against AC test laps (~3–5 days of driving + post-processing).
- Driver control loop (preview + PID + slip-setpoint + mistakes): ~1 week.
- Integration into `lap.py` (model-flag dispatch, `SimResult` extension, plot updates): ~2–3 days.
- Validation against real laps (compare lat/long G traces, yaw rate, slip angle — not just lap time): ~1 week.

Total: ~5–6 weeks of focused work, longer in evenings. Realistically a one-quarter project, not a sprint. Worth doing properly when we get there; not worth half-building.

#### 19.2.8 Risks and open questions

- **ODE solver stability at low speed.** Tyre force models go undefined as `v → 0` (slip angle uses `atan2(v_y, v_x)` which is fine but slip ratio uses `(ω*r - v) / v` which divides by zero). Standard mitigations: regularise denominator below ~3 m/s, switch to a low-speed kinematic model under threshold, or use the MF-Tyre low-speed extension. Decide before implementation.
- **Pacejka coefficient fitting from AC test laps is non-trivial.** AC's internal model has asymmetries (`BRAKE_DX_MOD`, `CX_MULT`, camber gain) that classical MF5.2 doesn't capture. Either fit to MF6 (more parameters, harder regression) or accept a 5–10% residual error in combined-slip cases. Document the choice.
- **Validation strategy.** Lap time alone is too coarse a metric for a slip-based sim — two very different cars can produce the same lap time with completely different lat/long G distributions. Validation must compare distributions: G-G diagram coverage, yaw-rate-vs-curvature correlation, slip-angle histograms at the apex. Same telemetry channels as §19.2.4.
- **Computational cost.** RK4 at 1 ms on 10 states for a 100-second lap is 1M solver steps. Pure-Python numpy will be slow (~10–60 s per lap). Realistic options: numba JIT (preferred — keep the code in Python), Cython, or accept the runtime cost for v3 and optimise later. The point-mass stays as the fast path for sweeps.
- **Open question — re-fit existing v1 drivers or start fresh?** The v1 `skill_pct` doesn't translate to slip-target α. Probably start fresh in v3, but document a translation heuristic (e.g. `α_target_initial = α_peak * skill_pct`) so users have a starting point.
- **Open question — do we model the diff?** AC's `drivetrain.ini` has `POWER` / `COAST` lock parameters (currently ignored by `car.py`). Diff behaviour is a significant oversteer / understeer modulator. Plausibly v3.1; v3.0 can ship with an open diff or a fixed lock %.
- **Open question — track surface variation.** `surfaceGrip` channel exists in AC (`sensor_dictionary_merged.json` line 1982); `tyreContactNormal*` gives banking. Both are v3.1 niceties — v3.0 assumes flat, uniform grip.

### 19.3 Item B — Tyre damage from type, temperature, wear, and pressure

#### 19.3.1 Motivation

Current sim uses fresh-tyre grip on every lap. In reality grip evolves with:
- **Compound** (street vs semislicks vs slicks — different μ_peak, different optimal-temp window, different wear rate).
- **Tread temperature** (cold = no grip; optimal = peak; hot = degraded).
- **Wear** (lap-over-lap loss of compound thickness).
- **Pressure** (under-inflated = oversize contact patch but mushy; over-inflated = small contact patch and bouncing).

Without these, the sim can't represent fade across a stint, can't predict tyre strategy, and gives the same lap time on lap 1 and lap 25 — which is wrong for any car with non-trivial tyre dynamics.

#### 19.3.2 Recommended approach — per-wheel state, multiplicative grip modifier

**Per-wheel state, integrated over the lap (and across laps in a stint):**
- `T_core` (°C) — bulk tread temperature.
- `wear_km` (km of compound consumed) — accumulating wear.
- `P` (psi) — current hot pressure (cold pressure + thermal expansion).

**Per-wheel grip multiplier** (applied to whatever the underlying tyre model produces — point-mass μ in v1/v2, MF peak `D` in v3):

```
grip_mult(wheel) = thermal_curve(T_core) * wear_curve(wear_km) * pressure_curve(P)
```

All three curves are **already in AC's `tyres.ini`** — we just need to parse and apply them.

#### 19.3.3 AC data we already have (verified against `cars_csv/bmw_1m/tyres.ini`)

**Thermal curve** — already referenced by AC's `tyres.ini`:
- `[THERMAL_FRONT].PERFORMANCE_CURVE = tcurve_street.lut` (and `tcurve_semis.lut` for semislicks). LUT format: `temperature_C | grip_multiplier`. Sits next to `tyres.ini` in the car data folder.
- `FRICTION_K` — fraction of slip energy that becomes heat (drives temp rise rate).
- `ROLLING_K` — rolling-resistance heat contribution.
- `CORE_TRANSFER`, `INTERNAL_CORE_TRANSFER`, `SURFACE_TRANSFER`, `PATCH_TRANSFER` — heat-transfer rates between tread surface, core, inner air, road.
- `COOL_FACTOR` — convective cooling rate.

**Wear curve:**
- `[FRONT].WEAR_CURVE = street_front.lut` (and `street_rear.lut`, `semislicks_front.lut`, etc.). Format per `docs/AI_CONTEXT.md`: `wear_km | grip_multiplier`.
- `[VIRTUALKM].USE_LOAD = 1` — wear scaled by load (slip-energy-based wear, not just distance).

**Pressure curve** — present in `tyres.ini` as a parametric model, not a LUT:
- `PRESSURE_STATIC` — cold reference pressure (psi).
- `PRESSURE_IDEAL` — hot pressure for peak grip (psi). bmw_1m street: 42/43 front/rear.
- `PRESSURE_D_GAIN` — fractional grip loss per psi off ideal. **This is the headline pressure-vs-grip parameter** (0.004 for bmw_1m street → 0.4% grip loss per psi deviation; quadratic in (P - P_ideal) by AC's docs).
- `PRESSURE_SPRING_GAIN` — extra spring rate per psi (affects suspension behaviour, not direct grip).
- `PRESSURE_FLEX_GAIN` — extra carcass flex per psi (couples into slip-angle behaviour — matters more for v3 slip model).
- `PRESSURE_RR_GAIN` — rolling resistance vs pressure (affects long-run lap-time consistency).

**Compound switching:**
- `[FRONT]` (compound 0) vs `[FRONT_1]` (compound 1, semislicks) vs `[FRONT_2]` (slicks if present). `[COMPOUND_DEFAULT].INDEX` picks the default.
- bmw_1m has at least two compounds visible in `tyres.ini`: street (DY0=1.28, P_ideal=42) and semislicks (DY0=1.31, P_ideal=33).

**Shared-memory telemetry channels** for validation (from `sensor_dictionary_merged.json`):
- `tyreCoreTemperature` — not in the merged dictionary directly; instead AC exposes `tyreTempIFL/IFR/IRL/IRR` (inner), `tyreTempMFL/...` (middle), `tyreTempOFL/...` (outer) — surface temps across the contact patch. Bulk core temp is approximated as the middle, or derivable from inner/outer/middle averaging.
- `tyreTempFL/FR/RL/RR` — single-value tread temp per wheel (likely an aggregate of I/M/O).
- `tyreWearFL/FR/RL/RR` — per-wheel wear state (0..100 or 0..1, check before use).
- `wheelsPressureFL/FR/RL/RR` — current hot pressure per wheel (psi).
- `tyreDirtyLevelFL/...` — pickup / marbles (not relevant for v2, future).
- `tyreCompound` — currently mounted compound name (validation only — the user's setup decides this).
- `aidTireRate` — game's tyre-wear multiplier (relevant if the user's AC session has wear scaled differently from 100%).

This is more than enough to **fit and validate** thermal / wear / pressure curves against real laps.

#### 19.3.4 Compatibility — fits inside the existing point-mass sim

**Item B does NOT require the slip-based simulator (Item A).** It can be added to the current point-mass 3-pass sim as a **per-segment grip modifier** with minor changes:

- Extend `Car` (or wrap it in a `TyreState` adapter) so `tyre_grip_lateral(v, lap_progress, wheel_state)` and `tyre_grip_longitudinal(...)` accept an optional per-wheel state. Default to fresh-tyre behaviour when no state is supplied.
- Add a `TyreState` class with `(T_core, wear_km, P)` per wheel. Integrate temperature each `ds`:
  - Heat input: `FRICTION_K * slip_power` (approximated from `lat_g` and `long_g`) + `ROLLING_K * v²`.
  - Heat loss: `COOL_FACTOR * (T_core - T_ambient)` + transfer to inner air.
  - Wear: `dW/dx = slip_energy * compound_constant` (from `WEAR_CURVE` derivative or fit).
  - Pressure: `P = P_cold + thermal_expansion(T_core)` — simple linear or AC's gain formula.
- Apply `grip_mult` from the three LUTs at each `ds`.

**This is the small, low-risk version.** It changes one return value in `Car`'s grip queries (multiplied by the per-wheel `grip_mult`) and adds a per-lap state struct. No solver changes; the 3-pass still works.

**Effort: days, not weeks.** Realistic scope:
- Parse the three LUT / parametric curves from `tyres.ini` and adjacent `.lut` files: ~1 day.
- `TyreState` class + integration step at each `ds`: ~2 days.
- Hook into `simulator.py`: ~1 day.
- Validation against bundled lap (T_core trace, pressure trace, wear progression): ~2 days.
- Documentation + acceptance criteria: ~1 day.

Roughly **one week of evening work**. Ships independently of v3.

#### 19.3.5 Driver-fit implications

`skill_pct` derived from telemetry depends on which lap you chose. Currently §13 guidance says "pick a learned lap, ~laps 8–15". But:
- A lap on **cold tyres** (lap 1–3) → driver looks worse than they are (grip cap is lower than what the v1 sim assumes from fresh-spec `DY0`).
- A lap on **heat-faded tyres** (lap 25+ on hot day) → same.
- A lap at **off-ideal pressure** → same in either direction.

Two mitigations for v2:

1. **Auto-detect tyre state** from the telemetry: if `tyreTempFL/...` and `wheelsPressureFL/...` channels are present in the AC log (they're shared-memory channels, not always logged to file — depends on user's logger config), use them to compute a per-lap `grip_state_at_lap_start` and feed it into the sim. The driver fit then operates on the lap with the correct grip ceiling.
2. **`source.tyre_state` block in the driver JSON.** If the fit's telemetry includes the channels, populate; otherwise mark as `unknown`. Downstream consumers (e.g. `lap.py --validate-against`) can warn when comparing a cold-tyres fit against a hot-tyres sim.

This is a v2 addition, not v1: requires §19.3.4's tyre-state plumbing.

#### 19.3.6 Open questions (Item B)

- **Multi-lap stint sim — needed?** Single-lap-with-fixed-state is enough for headline use cases. Multi-lap (degradation over 20 laps for race-pace prediction) is the next step up — same machinery, just runs N times with `TyreState` carried over and a `lap_strategy.csv` output. Strategic relevance is high for race-pace work; v2.1 candidate.
- **Tyre temperature inputs from real telemetry vs simulated cold start.** When running a pure prediction (no telemetry input), what's the initial `T_core`? Options: assume warm (60–80°C for street), expose as a CLI flag (`--tyre-temp 70`), or model a 1–2 lap warm-up curve and tag the lap accordingly.
- **Flat-spotted / blistered / grained tyres.** AC models all three (`GRAIN_GAIN`, `BLISTER_GAIN` in `[THERMAL_FRONT]`). Out of scope for v2 — defer to v3 alongside the slip-based sim, where slip energy is the natural driver of these phenomena.
- **AC's `aidTireRate` scaling.** If the user's AC session has tyre wear multiplier ≠ 100%, the wear curve derived from a real lap is scaled. Either ask the user to log with tyre rate = 100% (default), or expose `--tyre-rate <pct>` and unscale.
- **Sessions, not laps.** A "stint" is a multi-lap concept. The current `lap.py` is single-lap. v2.1 candidate: `stint.py` CLI that runs N laps and produces a per-lap report.

### 19.4 Recommended sequencing

Three release stages, in order:

#### v1 (shipped, this branch)

Point-mass 3-pass sim + `skill_pct`/`consistency_sigma` driver + AC-schema CSV emission + cross-track validation + prep / corner-analysis reorg. The plumbing release. Tyres are isotropic μ scaled by speed sensitivity; one fresh-tyre grip number per lap. Driver is a grip-utilisation %.

#### v2 (small follow-ups, weeks of evening work, can ship independently)

- §18 — MF4 telemetry output (additive, lap.py `--mf4` flag).
- §19.3 — Tyre-state model in the point-mass sim. `TyreState(T_core, wear_km, P)` per wheel, integrated each `ds`, multiplicative grip modifier. Parses the AC LUTs (`tcurve_*.lut`, `*_front.lut`, `*_rear.lut`) and pressure-curve params in `tyres.ini` (`PRESSURE_IDEAL`, `PRESSURE_D_GAIN`) that v1 currently ignores.
- §19.3.5 — Telemetry-driven tyre-state detection in `fit_driver.py`; optional `source.tyre_state` block in the driver JSON.
- v2.1 candidate — multi-lap stint sim (`stint.py` umbrella).

Critically: **v2 keeps the point-mass solver**. No new vehicle dynamics, no new driver schema, no re-validation of the existing tooling. Drop-in additive features.

#### v3 (large rebuild, multi-week project)

- §19.2 — Slip-based time-domain ODE sim. Pacejka MF tyre model with combined slip. New driver schema (preview + PID + α_target + reaction time). Lives alongside the point-mass — both sims selectable via `lap.py --model {point-mass, slip}`.
- Pacejka coefficient fit from AC test laps. Validation against G-G distributions and yaw-rate traces, not just lap time.
- §19.3 tyre-state model reuses its plumbing under the new vehicle model — the LUTs and grip multipliers are model-agnostic.
- v3.1 candidates — diff modelling, road banking, surface grip variation, blistering / graining.

**Why this order:** v2 is cheap, ships independently, and unlocks two real research questions (tyre management strategy, MF4-based ETL into a future DB). v3 is the expensive rebuild and shouldn't block v2.

### 19.5 References

- Pacejka, H. (2012). *Tire and Vehicle Dynamics*, 3rd ed. — the canonical Magic Formula textbook. ISBN 978-0-08-097016-5.
- ASAM-XIL / OpenSCENARIO community: Magic Formula MF6.x parameters and reference test procedures.
- Wikipedia overview of Pacejka MF — `https://en.wikipedia.org/wiki/Hans_B._Pacejka` (links to MF references; useful as a quick sanity-check, not a substitute for the textbook).
- `wheelSlipFL/FR/RL/RR` AC shared-memory channel semantics — confirm against the `sensor_dictionary_merged.json` entry (line 884 onwards) and AC's `ksgame.dll` shared-memory header before treating them as Pacejka inputs.
- `cars_csv/bmw_1m/tyres.ini` — concrete reference for the AC parameter set this section quotes.
- `C:/repos/SensorNotation/sensor_dictionary_merged.json` — full AC shared-memory channel dictionary; ~220 keys, all per-wheel state channels named above are present.
- §18 — adjacent v2 work item (MF4 output). §19 and §18 are independent and can be sequenced either way.

---

## 20. Two-lap "tiled" simulation (addendum, v1.1)

### 20.1 Goal

Every `lap.py` invocation emits **two consecutive laps** in one shot: lap 1 from rest (standing start), lap 2 starting at lap 1's end-of-lap speed (flying start). This makes the headline lap-time output representative of a real flying lap (lap 2) while keeping a standing-start reference (lap 1) in the same artefact. Loop-closure (§14.9) and cross-track validation (§15) both key off lap 2.

### 20.2 Non-goals

- Multi-lap (>2) simulation — out of scope for v1.1. v2 candidate (`stint.py`, §19.3.6).
- Fuel burn, tyre wear, brake fade across laps — tyre state is constant across both laps.
- Driver learning / per-lap skill adjustment.
- Per-lap weather / track-temp variation.

### 20.3 Decisions baked in

1. **Two laps, always (default-on).** `--single-lap` is the explicit opt-out.
2. **Lap 1 starts from rest; lap 2 starts at lap 1's end-of-lap-1 speed.** `v_forward[0]` for lap 2 ≠ 0.
3. **Implementation: tile the segment list twice.** Reuses the existing 3-pass simulator unchanged at the algorithmic level. The only adjustments are (a) carry forward the lap-1 final speed as lap 2's initial speed in the forward pass, and (b) post-split the output by distance into lap 1 and lap 2 halves.
4. **`SimResult.lap_id` is a per-point int array** with values `1` (first half) and `2` (second half). With `--single-lap`, all values are `1`.
5. **Single combined CSV output** (telemetry + trace), with a trailing `lap` column. No file splitting. Matches lake-style schema; AC-schema-only consumers can ignore the column.
6. **Timestamps continue monotonically across the lap boundary.** Lap 2's first sample = lap 1's last sample + `telemetry_dt_ms`.
7. **`distanceTraveled` and `normalizedCarPosition` reset at the lap boundary.** Per-lap-relative semantics (matches what a real AC two-lap log would look like if AC reset distance per lap).
8. **Monte-Carlo applies to lap 2 only.** Lap 1 is deterministic (standing-start variance is dominated by launch traction, not driver consistency — and the v1 MC model is grip-utilisation noise, which is the wrong vehicle for launch variance).
9. **Stdout reports both lap times.** Format in §7.6 / §20.5.
10. **`--validate-against` compares the real lap against sim lap 2.** Like-for-like flying-lap comparison. `--single-lap` forces comparison against lap 1 with a warning.
11. **Loop-closure (§14.9) picks lap 2 of the synthetic telemetry by default.** `fit_driver.py --lap` overrides.

### 20.4 CLI surface (additions to `lap.py`)

```
[--single-lap]
```

When set: only lap 1 is simulated and emitted. Output schema unchanged (the `lap` column is constant `1`). All other defaults preserved.

### 20.5 Stdout report (extended from §7.6)

Two-lap (default):
```
Lap 1 (standing): 1:48.612
Lap 2 (flying):   1:46.231 ± 0.061 (N=20)
```

With MC disabled (`consistency_sigma == 0`):
```
Lap 1 (standing): 1:48.612
Lap 2 (flying):   1:46.231
```

With `--single-lap` (legacy v1-compatible):
```
Lap Time: 1:48.612
```

(Bytecompatible with v1 when `consistency_sigma == 0` and `--single-lap` is set.)

### 20.6 Algorithm

1. Build the per-point segment list as today (from track CSV or built-in segments).
2. **Tile:** concatenate the segment list with itself, producing a 2× distance grid (`distance_m` in the second half is `track.total_length_m + original_distance`).
3. Run the 3-pass simulator on the tiled grid. Lap 1's forward pass starts at `v=0`; lap 2's forward pass continues from the speed at the lap-1/lap-2 boundary (no reset).
4. **Split:** post-process the resulting per-point arrays into `lap_id` based on `distance_m < total_length_m`. Times in lap 2 stay relative to the sim start (monotonic across the boundary).
5. **For telemetry emission** (§14): build a single time grid across both laps, time-resample distance/speed/labels as usual, derive gas/brake per §14.3 over the whole grid (the heuristic and low-pass operate on the concatenated trace — taper/ramp at the start/end of each lap are handled by the limit-label transitions just like any other transition). Emit one CSV with `lap` column.
6. **For MC**: only the lap-2 segment of each MC run contributes to the lap-2 mean/std. Lap 1 is computed once, deterministically, with `skill_pct` and no noise.

### 20.7 Acceptance criteria

See §11.18 (lap column + lap-2-not-slower-than-lap-1-within-sigma). Headline checks:
- Default `lap.py` invocation emits a `lap` column on telemetry and trace CSVs with values exactly `{1, 2}`.
- Lap 2 time ≤ lap 1 time + `consistency_sigma` for all bundled drivers.
- `timestamp_ms` is strictly monotonic across the entire CSV (no reset at the lap boundary).
- `distanceTraveled` and `normalizedCarPosition` reset at the lap boundary.
- `--single-lap` reduces output to lap 1 only; the `lap` column is constant `1`.
- MC stats apply to lap 2 only; lap 1 has no ± term in stdout.
- `--validate-against` compares against lap 2 by default; with `--single-lap` falls back to lap 1 with a warning.
- `fit_driver.py` on a synthetic two-lap telemetry CSV picks lap 2 by default; `--lap 1` overrides.

### 20.8 Module touchpoints

- `src/lap_estimator/simulator.py` — `simulate(..., two_lap=True)` default; tile + lap_id post-split; lap-2 forward-pass initial speed.
- `src/lap_estimator/sim_telemetry.py` — handles the cross-boundary time grid, resets distance/normalized-position at the boundary, emits the `lap` column.
- `src/lap_estimator/report.py` — trace CSV gains `lap` column; stdout report extended per §20.5.
- `src/lap_estimator/validate.py` — selects lap 2 from `SimResult` by default (`target_lap=2`).
- `src/lap_estimator/driver_fit.py` — filters input telemetry to a single lap when a `lap` column is present (default lap 2).
- `lap.py` — `--single-lap` flag.
- `fit_driver.py` — `--lap {1,2}` flag.

### 20.9 Open questions / v2 candidates

- **Should lap 1 be optionally suppressed from the telemetry CSV** while still being used to seed lap 2's initial speed? (i.e. emit only lap 2, but with a proper flying-start.) v2 candidate. v1.1 always emits both for traceability.
- **Multi-lap (>2) stint mode.** Same machinery, N copies of the segment list, per-lap state evolution (tyre temp / wear / fuel — §19.3). v2.1.
- **Lap-1 standing-start variance.** Currently deterministic; could model launch traction as a noise source. Deferred — needs a launch model (`car.py` doesn't currently distinguish first-gear traction-limit from the steady-state corner-exit gas trace).

---

## Decisions block (locked in this revision)

1. **Folder convention is locked.** `cars_in/` / `tracks_in/` = user-dropped raw AC (gitignored). `cars_csv/` / `tracks_csv/` = tracked outputs of prep. `drivers/` = tracked driver JSONs. `samples/aclog/` = tracked sample telemetry. (§6.9)
2. **Repo layout uses a `src/lap_estimator/` package** for simulator library code, with `prep/`, `analysis/`, and root-level CLIs (`lap.py`, `fit_driver.py`) as separate scopes. Path bootstrap via per-script `sys.path` insert until a `pyproject.toml` editable install lands. (§6.8)
3. **Two simulator CLIs** (`fit_driver.py`, `lap.py`) plus **three pipeline CLIs** (`prep/prep_car.py`, `prep/prep_track.py`, `analysis/corner_analysis.py`) — five total entry points. No separate `validate_lap.py`. (§14.4 item 12, §15.7)
4. **Corner notation lives in `<track_dir>/<layout_stem>_corners.json`** with the schema in §17.4. JSON, not CSV. Telemetry-free.
5. **`tracks_config.json` is the in-repo single source of truth for corner-classification thresholds and colours.** Loaded by `analysis/corner_analysis.py`. (§7.9, §17.8)
6. **Corner analysis is downstream of prep, not part of it.** Two independent steps. (§16.2, §17.1)
7. **Legacy DuckDB-staging CSVs (`track_points.csv`, `track_corners.csv`, `track_meta.csv`) are dropped.** The JSON notation file replaces them.
8. **Sample AC telemetry log moves to `samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv`** and is committed. The existing `Corner_Analysis/` folder is untouched (user-managed scratch). (§6.9, §10)
9. **`cars_in/*` and `tracks_in/*` stay gitignored** (already in `.gitignore`). `tracks_csv/`, `cars_csv/`, `drivers/`, `samples/`, `tracks_config.json`: tracked. (§6.9)
10. **`prep_track.py` does not auto-invoke corner analysis.** User chains the two commands. v2 candidate for a `prep_all.py` umbrella.
11. **`analysis/corner_analysis.py` is a rewrite, not an in-place edit** of the existing `Corner_Analysis/corner_analysis.py`. The latter stays as user scratch.
12. **MF4 telemetry output is v2, PLANNED, not v1.** Format = ASAM MDF v4 via `asammdf` (optional dependency); emitted alongside CSV when `--mf4` is passed to `lap.py`; channel set mirrors the AC-schema CSV one-to-one. Read-side MF4 ingest is v2.1. (§18)
13. **Slip-based physics (drift / oversteer / understeer) and per-wheel tyre-state (temp / wear / pressure) are v3 / v2 backlog research items, not v1.** v3 = slip-based ODE sim + Pacejka tyre model + new driver schema (multi-week build). v2 = tyre-state-as-grip-modifier inside the existing point-mass sim (one week of evening work). Both items are scoped in §19, with AC signal coverage, architectural impact, and risks documented. v1 stays point-mass with isotropic μ. (§19)
14. **(v1.1-A) Driver config format is JSON, not YAML.** `drivers/*.json` exclusively; no YAML loader fallback. PyYAML dropped from project requirements. Rationale: rest of the repo (tracks_config, corners JSON, sim telemetry CSV/PNG) is already JSON; driver schema is flat enough that YAML's comment + nesting advantages don't justify the dep. Migration: every existing `drivers/*.yaml` is rewritten as `.json` and the `.yaml` deleted in a single commit. (§6.2, §7.2, §10)
15. **(v1.1-B) Driver-lag 1st-order low-pass on emitted gas/brake.** New driver field `driver_tau_s` (seconds, default `0.12`). Applied independently to `gas` and `brake` as Layer 3 of the §14.3 pipeline, after the corner-shape heuristic, before CSV write. `α = dt / (τ + dt)`. `τ = 0` bypasses the filter. Default 0.12 s chosen as a human pedal-modulation time constant (motor-reflex floor ≈ 80–120 ms; user accepted Buddy recommendation over an initial 20 ms guess that gave near-zero smoothing at 100 ms sampling). (§13.6/§7.2, §14.3 Layer 3)
16. **(v1.1-C) Trail-brake + throttle-ramp heuristic on emitted gas/brake.** New driver fields `trail_brake_m` (default `30.0`) and `throttle_ramp_m` (default `40.0`). Applied as Layer 2 of the §14.3 pipeline — between the limit-label rule and the driver-lag low-pass. Order is load-bearing: heuristic shapes the corner geometry on sharp limit-label transitions, then the low-pass smooths residual edges. Inverting the order would smear the limit-label transitions before the heuristic can read them. Either field set to `0.0` disables that heuristic. (§7.2, §14.3 Layer 2)
17. **(v1.1-D) Two-lap "tiled" simulation, always default.** Every `lap.py` invocation simulates lap 1 (standing start) and lap 2 (flying start from lap-1 end-of-lap speed) and emits both. Single combined telemetry/trace CSV with a `lap` column; monotonic `timestamp_ms` across the boundary; `distanceTraveled` and `normalizedCarPosition` reset per lap. Monte-Carlo applies to lap 2 only; lap 1 is deterministic. `--validate-against` compares the real lap against sim lap 2. Loop-closure picks sim lap 2 by default. `--single-lap` CLI flag is the explicit opt-out (legacy v1 behaviour, lap 1 only). (§20, §7.4, §7.6, §7.7, §15.4)
