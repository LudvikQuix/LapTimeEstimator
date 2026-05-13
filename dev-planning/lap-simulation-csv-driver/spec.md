# Lap Simulation: CSV Track + Driver Config

**Status:** Draft (v1 + telemetry-fitting + sim-telemetry-emission + cross-track-validation + project-reorg addendum; v1.1 in-flight: driver JSON migration + driver-lag low-pass + trail-brake/throttle-ramp heuristic + two-lap tiled sim + multi-lap-mandatory fit — §13/§14/§20; v1.2 in-flight: driver-profile dynamic-signal measurement from telemetry + 10 ms sim cadence default — §13/§14; v2 MF4 telemetry output PLANNED — §18; v3 slip-based physics + tyre-state BACKLOG — §19)
**Project:** LapTimeEstimator
**Branch:** feature/sc-71955/lap-simulation
**Created:** 2026-05-13
**Planned with:** Buddy

## 1. Summary

Today the lap-time simulator consumes a `Car` (parsed from AC data) and a `Track` built from hard-coded segments or a simple `{length, radius}` JSON. There is no notion of a driver, the AC-input → CSV preparation steps are scattered between root-level scripts (`decode_acd.py`, `decode_track.py`) and an untracked `Corner_Analysis/` workdir, and corner classification is bolted onto an exploratory analysis script that depends on telemetry merging.

This feature reshapes the tool around three first-class inputs — **car**, **track CSV**, **driver JSON** — and produces a lap-time estimate plus a trace artifact that can be visually validated against the AI reference speed already present in the CSV. The driver model is intentionally minimal in v1 (skill + optional consistency noise) so we get the input/output plumbing right before investing in richer driver behaviour.

The whole feature ships behind **two simulator CLI entry points** — `fit_driver.py` (derive a driver JSON from real AC telemetry, **multi-lap mandatory** — see §13) and `lap.py` (run a sim on a track with a given driver, optionally cross-validating against a real lap on that same track via `--validate-against`). The "just run a sim" mode is the default of `lap.py`; the cross-track validation flow is the same script with one extra flag.

It also formalises the **preparation pipeline** (§16): two CLIs `prep/prep_car.py` and `prep/prep_track.py` that turn user-dropped AC content under `cars_in/` and `tracks_in/` into repo-friendly artefacts under `cars_csv/` and `tracks_csv/`. And it splits **corner analysis** (§17) out into a standalone, telemetry-free script `analysis/corner_analysis.py` that reads a track CSV plus the in-repo `tracks_config.json` and writes a canonical corner notation JSON next to the track.

**Pivot (2026-05-13):** instead of asking the user to hand-author `drivers/<name>.json`, the headline workflow becomes **deriving the driver JSON from real Assetto Corsa telemetry** via the `fit_driver.py` tool (see §13). Hand-authored configs are still supported — they just stop being the primary entry point.

**Closing the loop (2026-05-13):** after a sim run, the simulator also emits a **synthetic telemetry CSV** that matches the real AC log schema exactly (see §14). This makes the sim's output a drop-in replacement for real telemetry — downstream tools (notably `fit_driver.py`) can ingest sim output the same way they ingest real laps. A fit → sim → emit → re-fit round-trip becomes a strong self-consistency check on the whole pipeline.

**The actual point (2026-05-13):** the real reason this tool exists is **cross-track prediction with validation** (see §15). Fit a driver on Track A, predict their lap time on Track B (which they have never run in sim or in AC), then have the driver drive Track B in AC for ~10 laps to learn it, capture telemetry, and compare. Sections §13 and §14 are the plumbing; §15 is the headline use case.

**Reorg (2026-05-13):** the repository layout is locked. `cars_in/` and `tracks_in/` are user-dropped raw AC content (gitignored). `cars_csv/` and `tracks_csv/` are the outputs of the preparation scripts (tracked). Preparation scripts live under `prep/`, corner analysis lives under `analysis/`, simulator library code lives under `src/lap_estimator/`, and the two simulator CLIs (`lap.py`, `fit_driver.py`) stay at the repo root for discoverability. See §6.9 and §16/§17.

**Future directions (2026-05-13):** two longer-horizon backlog items are captured in §19 — a slip-based simulator that can represent drift / oversteer / understeer, and a tyre-state model (temp / wear / pressure) that modulates grip lap-over-lap. Both are research-scoped, not v1 work; §19 documents what AC already gives us for free, the architectural impact, and the recommended sequencing.

**v1.1 in-flight (2026-05-13):** five bundled changes layered on the shipped v1 plumbing — (A) driver config format moves from YAML to JSON (no fallback); (B) driver-lag 1st-order low-pass on emitted gas/brake; (C) trail-brake + throttle-ramp corner-shape heuristic applied to gas/brake before the low-pass; (D) two-lap "tiled" simulation always emitted (lap 1 standing, lap 2 flying), with a `lap` column on telemetry/trace CSVs and Monte-Carlo / validation / loop-closure all keying off lap 2; (E) **driver-fit requires ≥2 laps and pools cornering samples across them** — single-lap fits are hard-errors (§13). See §13, §14.3, §20.

**v1.2 in-flight (2026-05-13):** three bundled extensions on top of v1.1 — (F) `fit_driver.py` now **measures the dynamic driver-profile signals from telemetry** instead of taking the hand-defaults: `driver_tau_s`, `trail_brake_m`, `throttle_ramp_m` plus two new statistics `pedal_press_rate_per_s` and `steering_aggression_deg_per_s`. All five live under a new `profile.dynamic` sub-object in `drivers/<name>.json`. Hand-defaults stay as fallbacks when measurements are not extractable. (G) **Lap-selection rule** for the fitter: take the **5–10 newest** laps for that driver (fallback to all if fewer than 5, hard minimum of 2). Newness comes from input ordering or the new `--newest N` flag combined with `--laps-glob`. (H) **Default `--telemetry-dt-ms` raised from 100 to 10** for sim emission — 10× larger files, accepted for the dramatically better driver-lag smoothing fidelity at the new α = 10 / (120 + 10) ≈ 0.077. See §13.11, §13.12, §14.

## 2. Goals

- Accept a **car data directory**, a **track CSV path**, and a **driver JSON path** as the three positional inputs to the sim CLI (`lap.py`).
- Consume the rich per-point track CSV format (ideal line preferred, centerline fallback) directly — no intermediate JSON conversion required.
- Apply a simple **driver model** (`skill_pct`, optional `consistency_sigma`, `driver_tau_s`, `trail_brake_m`, `throttle_ramp_m`) that scales the car's effective grip uniformly and shapes the emitted gas/brake trace.
- Emit a per-point **trace CSV** and a **sim-vs-AI comparison PNG** next to the track input so the user can sanity-check results in Quix Cloud / locally.
- Preserve current stdout lap-time report format (extended for two-lap output — see §20).
- Keep modules small (~500-line soft ceiling).
- Provide a `fit_driver.py` CLI that ingests **two or more** AC telemetry CSVs (variadic positional, plus a `--laps-glob` shortcut) and emits a single driver JSON calibrated to pooled cornering samples from all laps. Single-lap fits are rejected with a clear error — see §13.
- Emit a **synthetic AC-schema telemetry CSV** alongside the trace output by default (configurable cadence, **default 10 ms in v1.2** — see §14), so sim runs and real laps are interchangeable downstream.
- Ship cross-track validation as a **flag on `lap.py`** (`--validate-against <real_telemetry.csv>`), not as a third CLI: when present, `lap.py` additionally loads the real lap, prints a delta report, writes an overlay PNG and a per-bin delta CSV, and emits a GOOD/LOOSE/BAD verdict (§15). Two simulator CLIs total: `fit_driver.py` and `lap.py`.
- **(v1.2)** Measure dynamic driver-profile signals (`driver_tau_s`, `trail_brake_m`, `throttle_ramp_m`, plus the new `pedal_press_rate_per_s` and `steering_aggression_deg_per_s`) from telemetry inside `fit_driver.py`, replacing hand-defaults. See §13.11 / §13.12.
- **(v1.2)** Default lap selection is the **5–10 newest laps**; fall back to all available if fewer than 5; hard minimum 2 (existing §13 guard). See §13.12.
- **(reorg)** Ship a **preparation pipeline** (§16).
- **(reorg)** Ship a **corner analysis tool** (§17).
- **(reorg)** Lock the folder convention (§6.9).

## 3. Non-goals

- Driver line selection / line deviation (always uses the ideal line if present).
- Separate brake-aggression vs throttle-aggression parameters (v2 hook in §13.9).
- Reaction-time / lift-and-coast / fuel-saving driver behaviours.
- Tyre thermal model, ABS, traction control, weight transfer beyond what `car.py` already does.
- Hooking into `RequirementsForAbnormalityAnalysis.txt` checks — separate feature.
- Multi-lap / fuel-burn / tyre-wear simulation. (v1.1 emits two laps for warm-up/flying-lap representativeness — see §20 — but does **not** model fuel burn or tyre wear.)
- Web UI. CLI only.
- Per-corner skill profile in v1 (v2 hook in §13.6).
- Fitting tyre, aero, or engine parameters from telemetry — only driver-skill scalars are fit; the car model is treated as ground truth.
- Per-track familiarity / learning-curve modelling in v1.
- No database, object store, remote artefact server, or networked service.
- **(reorg)** The corner-analysis tool does **not** merge telemetry.
- **(reorg)** `prep/prep_track.py` does **not** infer corner notation.
- **MF4 output is v2 — see §18; v1 only emits CSV.**
- **Slip-based physics, drift / oversteer / understeer dynamics, and per-wheel thermal / wear / pressure state are v3 backlog — see §19.**
- **(v1.2)** Consuming `pedal_press_rate_per_s` / `steering_aggression_deg_per_s` inside the simulator — they are recorded as profile statistics only for future v1.3 work. The currently-consumed dynamic fields remain `driver_tau_s`, `trail_brake_m`, `throttle_ramp_m`.

## 4. User stories / scenarios

1. **Run a single lap.** User has decrypted car data in `cars_csv/bmw_1m/data/`, a track CSV, and `drivers/pro.json`. They run `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_gp_a_ideal_line.csv drivers/pro.json` and see the lap-time report on stdout (both laps — §20), plus a trace CSV and PNG written next to the track CSV.
2. **Compare two drivers, same car/track.** User runs the command twice with `drivers/pro.json` and `drivers/amateur.json`.
3. **Use a centerline-only track.** User points at a `layout_*.csv` with no `*_ideal_line.csv` sibling.
4. **Consistency study.** User sets `consistency_sigma: 0.3` → sim performs N Monte-Carlo runs on **lap 2 only**.
5. **Legacy invocation (built-in track).** User runs `python lap.py cars_csv/bmw_1m monza drivers/pro.json`.
6. **Fit a driver from multi-lap telemetry.** User has six AC laps at `samples/aclog/Tomas_Lap{1..6}.csv` (lap 1 has a standing-start prefix; lap 6 is unfinished). They run `python fit_driver.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json samples/aclog/Tomas_Lap1.csv samples/aclog/Tomas_Lap2.csv samples/aclog/Tomas_Lap3.csv samples/aclog/Tomas_Lap4.csv samples/aclog/Tomas_Lap5.csv samples/aclog/Tomas_Lap6.csv` (or equivalently `--laps-glob "samples/aclog/Tomas_Lap*.csv"`). The tool pools cornering samples across all six laps, writes `drivers/tomas.json`, runs a validation sim, and prints the multi-lap delta block (real-laps-used / mean-real / sim-lap-2 / delta — §13.5 step 9).
7. **Loop-closure check.** Sim → synthetic telemetry CSV (two laps) → fed back into `fit_driver.py`. Since the fitter requires ≥2 laps, the synthetic CSV's `lap` column (carrying `1` and `2`) is split per-lap and both are passed to the fitter, or the user runs the sim twice with different seeds and pools both lap-2s.
8. **Cross-track prediction and validation (headline workflow).** Fit on Track A → predict on Track B → drive Track B in AC for ~10 laps → validate.
9. **(reorg) Prepare a new car / track from raw AC content.**
10. **(reorg) Run corner analysis on a prepared track.**
11. **(v1.1) Single-lap mode for fast sweeps.** User runs `python lap.py ... drivers/pro.json --single-lap`.
12. **(v1.2) Pick newest laps automatically.** User has 30 telemetry CSVs in `samples/aclog/` and runs `python fit_driver.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json --laps-glob "samples/aclog/Tomas_*.csv" --newest 10`. The tool ranks the glob hits by mtime (newest first), keeps the top 10, and runs the multi-lap fit.
13. **(v1.2) Four-layout sweep across ks_nurburgring.** After fitting `drivers/tomas.json` on the freshest 5–10 Sprint A laps, the user runs `lap.py` once for each of `layout_gp_a.csv`, `layout_gp_b.csv`, `layout_sprint_a.csv`, `layout_sprint_b.csv` at `--telemetry-dt-ms 10` (2 laps by default) and compares the predicted lap times.

## 5. Proposed design

- Add a new `driver.py` module with a `Driver` dataclass (loaded from JSON) and a small `apply_to_car(car)` helper. The dataclass also carries the three v1.1 fields and (v1.2) a `profile` block.
- Extend `track.py` with a `Track.from_csv(path)` classmethod.
- Adjust `simulator.simulate(...)` so it takes an optional `driver` argument and supports two-lap tiled mode (§20).
- Move output artifact generation into a new `report.py` module.
- `lap.py` is the single sim CLI.
- Add a `telemetry.py` module that owns AC-log parsing and merging-with-track logic. **(v1.2)** It also accepts the broader AC channel set, including `steerAngle` when present; missing optional channels degrade gracefully with a warning.
- Add a `driver_fit.py` module that consumes a list of merged frames (one per lap) and pools their cornering samples to fit driver parameters. **(v1.2)** It also runs the new `profile_dynamics.py` helper to measure `driver_tau_s`, `trail_brake_m`, `throttle_ramp_m`, `pedal_press_rate_per_s`, `steering_aggression_deg_per_s` from leading-edge / ramp / steering-rate statistics on the same merged frames. Add a thin `fit_driver.py` CLI wrapper that accepts variadic telemetry paths, `--laps-glob`, and `--newest N`.
- Add a `profile_dynamics.py` module (v1.2 — see §13.11). Single file, ~250 lines.
- Add a `sim_telemetry.py` module (§14).
- Add a `validate.py` module (§15).
- **(reorg)** Move modules into `src/lap_estimator/`, `prep/`, `analysis/`.

## 6. Sub-features / work breakdown

### 6.1 Driver module (new file: `src/lap_estimator/driver.py`)
- Loads driver JSON; exposes `skill_pct`, `consistency_sigma`, `driver_tau_s`, `trail_brake_m`, `throttle_ramp_m`; provides grip-scaling wrapper.
- **(v1.2)** Tolerates the new `profile` block (`profile.dynamic.*`). `Driver.load` reads `driver_tau_s`/`trail_brake_m`/`throttle_ramp_m` from `profile.dynamic` first if present, falls back to top-level keys, then to hand-defaults. `pedal_press_rate_per_s` and `steering_aggression_deg_per_s` are loaded onto the dataclass but **not consumed** by the simulator in v1.2 — they are statistics only.
- `Driver.load(path) -> Driver` via stdlib `json`. **PyYAML removed.**
- Owner: ArchDev.

### 6.2 Driver JSON schema + example files
- Schema in §7.2 (includes v1.2 `profile` block). Examples: `pro.json` (`skill_pct: 0.97`), `amateur.json` (`skill_pct: 0.82`).
- YAML → JSON migration: clean break, single commit. Files: `pro`, `amateur`, `tomas_*`, `ludvik_*`.
- Owner: ArchDev.

### 6.3 Track CSV loader (`src/lap_estimator/track.py` additions)
- `from_csv(path) -> Track`, `to_points(ds)`, `to_ai_reference(ds)`, `total_length_m`.
- Owner: ArchDev.

### 6.4 Simulator changes (`src/lap_estimator/simulator.py`)
- `simulate(car, track, driver=None, ds=2.0, rng=None, two_lap=True) -> SimResult`.
- New `simulate_monte_carlo(...)` — MC stats on lap 2 only.
- `SimResult` gains `times`, `ai_speeds`, `limit_label`, `lap_id`.
- Owner: ArchDev.

### 6.5 Output artifacts (new file: `src/lap_estimator/report.py`)
- Writes `<track_stem>_sim_trace.csv` (with `lap` column) and `<track_stem>_sim_vs_ai.png`.
- Exposes `plot_speed_overlay(...)` helper.
- Owner: ArchDev.

### 6.6 CLI rewiring (`lap.py`)
- See §13.3 for `fit_driver.py` CLI; `lap.py` CLI:
  ```
  python lap.py <car_data_dir> <track> <driver_json> \
                [--ds 2.0] [--all-tracks] [--single-lap] \
                [--no-plot] [--no-telemetry] [--telemetry-dt-ms 10] \
                [--validate-against <real_telemetry_csv>] [--bin-m 100] [--per-corner]
  ```
  **(v1.2)** `--telemetry-dt-ms` default is now `10` (was `100` in v1.1). Old behaviour reproducible with `--telemetry-dt-ms 100`.
- Owner: ArchDev.

### 6.7 README / docs touch-up
- Reflect new repo layout, four CLIs, driver JSON schema (v1.1 + v1.2 `profile` block), outputs, validation, corner notation, two-lap default, multi-lap mandatory fit, 10 ms telemetry default, `--newest N` flag.
- `PyYAML` removed.
- Owner: DocuGuy.

### 6.8 Package skeleton and module moves (reorg)
- See file moves listed in §10.
- Owner: ArchDev.

### 6.9 Folder convention and `.gitignore` policy (reorg)
- Tracked: `cars_csv/`, `tracks_csv/`, `drivers/*.json`, `tracks_config.json`, `samples/aclog/*.csv`, `prep/`, `analysis/`, `src/lap_estimator/`, root CLIs, `docs/`, `dev-planning/`.
- Gitignored: `cars_in/*`, `tracks_in/*`, `.tmp/`.
- Owner: ArchDev.

### 6.10 Profile-dynamics module (new file: `src/lap_estimator/profile_dynamics.py`) — v1.2
- Exposes `measure_dynamics(merged_frames: list, *, dt_floor_s: float = 0.005) -> ProfileDynamics`.
- Computes `driver_tau_s`, `trail_brake_m`, `throttle_ramp_m`, `pedal_press_rate_per_s`, `steering_aggression_deg_per_s` (see §13.11 for algorithm).
- Owner: ArchDev.

## 7. Data & interface contracts

### 7.1 Track CSV (input)
Required columns: `distance_m`, `segment_length_m`, `radius_m`, `gradient_pct`, `elevation_m`, `speed_ms`.

### 7.2 Driver JSON (input) — v1.1 + v1.2 profile block
```json
{
  "name": "string",
  "skill_pct": 0.95,
  "consistency_sigma": 0.0,
  "driver_tau_s": 0.12,
  "trail_brake_m": 30.0,
  "throttle_ramp_m": 40.0,
  "profile": {
    "dynamic": {
      "driver_tau_s": 0.14,
      "trail_brake_m": 28.5,
      "throttle_ramp_m": 47.0,
      "pedal_press_rate_per_s": 6.4,
      "steering_aggression_deg_per_s": 312.0,
      "measured": {
        "driver_tau_s": true,
        "trail_brake_m": true,
        "throttle_ramp_m": true,
        "pedal_press_rate_per_s": true,
        "steering_aggression_deg_per_s": false
      },
      "sample_counts": {
        "pedal_leading_edges": 184,
        "brake_taper_segments": 22,
        "throttle_ramp_segments": 24,
        "steering_samples": 198432
      }
    }
  },
  "source": {
    "telemetry_csvs": ["string", "..."],
    "track_csv": "string",
    "car_data_dir": "string",
    "n_laps": 0,
    "n_finished_laps": 0,
    "real_lap_times_s": [0.0],
    "real_lap_time_s": 0.0,
    "pooled_sample_count": 0,
    "sim_lap_time_s": 0.0,
    "delta_s": 0.0,
    "lap_selection": {
      "rule": "newest-5-to-10",
      "candidates_considered": 0,
      "selected_count": 0,
      "selected_sources": ["..."]
    },
    "fitted_at": "ISO-8601 string",
    "fit_version": "2"
  }
}
```
Field semantics:
- `name` (string, required).
- `skill_pct` (float in (0, 1], required).
- `consistency_sigma` (float ≥ 0, optional, default 0.0).
- Top-level `driver_tau_s` / `trail_brake_m` / `throttle_ramp_m` remain for backward compatibility with hand-authored JSON and pre-v1.2 fits. When `profile.dynamic.*` of the same name is present, **the profile value wins** (consumed by sim). Hand-defaults: 0.12 / 30.0 / 40.0.
- `profile.dynamic.*` (v1.2, optional, populated by `fit_driver.py`):
  - `driver_tau_s` — measured median time-to-50 % on gas/brake leading edges (seconds). Fallback 0.12.
  - `trail_brake_m` — measured median distance over which brake decays from 0.8 → 0.1 entering corner-limited segments (metres). Fallback 30.0.
  - `throttle_ramp_m` — measured median distance over which gas rises from corner-limited partial → 0.95+ exiting corners (metres). Fallback 40.0.
  - `pedal_press_rate_per_s` — measured median dy/dt on gas+brake leading edges (per-second). Statistic only — not consumed by the v1.2 simulator. Fallback `null` if no clean edges.
  - `steering_aggression_deg_per_s` — measured 95th-percentile |d(steerAngle)/dt| over the pooled lap data (deg/s). Requires `steerAngle` channel; if absent, this field is `null` and `measured.steering_aggression_deg_per_s = false`. Statistic only.
  - `measured.*` — booleans flagging whether each field came from real telemetry (`true`) or fell back to a hand-default / null (`false`).
  - `sample_counts.*` — diagnostic counts of how many edges / segments / samples contributed to each measurement.
- `source` (object, optional, populated by `fit_driver.py`) — full schema in §13.5 step 8. v1.1 multi-lap fields plus v1.2 `lap_selection` sub-block (see §13.12). `fit_version` bumps from `"1"` to `"2"` when the profile block is populated.

Validation: as today, plus:
- If `profile.dynamic.driver_tau_s` exists and is outside `[0.0, 1.0]`, fail fast.
- If `profile.dynamic.trail_brake_m` exists and is negative, fail fast.
- If `profile.dynamic.throttle_ramp_m` exists and is negative, fail fast.
- The other two are statistics — out-of-range values warn but do not fail.

**Format: JSON only.**

### 7.3 Car data dir (input)
Unchanged.

### 7.4 Trace CSV (output)
`lap, distance_m, sim_speed_ms, sim_speed_kmh, ai_speed_kmh, time_s`.

### 7.5 Comparison plot (output)
Single matplotlib figure.

### 7.6 Stdout report
Two-lap format: `Lap 1 (standing): 1:48.6   |   Lap 2 (flying): 1:46.2 ± 0.06 (N=20)`.

### 7.7 Synthetic telemetry CSV (output, new — see §14)
`timestamp_ms,gas,brake,distanceTraveled,speedKmh,normalizedCarPosition,lap`.

### 7.8 Corner notation JSON
See §17.4.

### 7.9 `tracks_config.json`
Schema in original spec; thresholds + colours.

## 8. Risks, constraints, and open questions

### Risks
- CSV resampling vs current segment-resampling.
- `skill_pct` as uniform grip multiplier.
- `consistency_sigma` calibration is heuristic.
- Synthetic telemetry realism (v1.1 better, still heuristic).
- Driver-lag default tuning.
- Lap-2 fairness vs lap-1 representativeness.
- Driver portability across tracks (v1 limitation).
- **(v1.1 — fit) Pooling-vs-averaging tradeoff.** Pooling samples across laps means a lap with 4000 cornering samples contributes 8× more weight than a 500-sample short-stint lap. This is the intended behaviour — high-data laps should dominate — but documented so future tuners know percentile estimates can be skewed by a single dominant lap. v2 candidate: a per-lap weight cap.
- **(v1.2) Leading-edge detection robustness.** Real pedal traces are noisy. A naïve d/dt > threshold rule will catch micro-corrections as "edges". Mitigation: hysteresis band (require pedal to dip below 0.05 before the next rising edge counts) and minimum step height (rise from ≤ 0.1 to ≥ 0.5 within 200 ms). Edges below the minimum are discarded. If <10 clean edges are found across the pool, fall back to the 0.12 s default and set `measured.driver_tau_s = false`.
- **(v1.2) Trail-brake / throttle-ramp segment selection.** These are measured around the boundaries of corner-limited segments. The boundary set comes from the simulator's per-point binding-limit label run against the real telemetry's speed trace — i.e. we need to re-classify each real-lap point with the limit rule to know which segments are "corner-limited" and where their entry/exit are. Risk: if classification is off, segments are mis-selected. Mitigation: a simple speed-minimum-based corner detector inside `profile_dynamics.py` that does not depend on the simulator (find local minima of `speedKmh`, take ±2 s window around each, treat that as a corner-limited segment). Use this for v1.2; switch to the simulator's limit label in a v1.3 cleanup.
- **(v1.2) `steerAngle` channel availability.** Older AC logs may not contain it. The fitter must not fail when missing — set to null + measured=false and continue.
- **(v1.2) 10 ms telemetry file size.** A 1:45 lap × 2 laps × 100 Hz ≈ 21 000 rows per file × 6 numeric columns + lap = ~1.5 MB raw, ~600 KB compressed. Per-driver-per-track-per-fit-run order of magnitude. Accepted.
- **(reorg) Track-preparation parity.**
- **(reorg) Import-path fragility.**

### Constraints
- ~500-line soft cap. `driver_fit.py` ~260 (multi-lap pooling + profile-dynamics integration), `profile_dynamics.py` ~250, `fit_driver.py` ~100 (variadic CLI + glob + `--newest`). All under ceiling.

### Open questions
- MC `n_runs` default 20.
- `--all-tracks` does not iterate drivers.
- Per-track familiarity — v2.
- Cross-track validation tolerance numbers.
- **(v1.1 — fit) Per-lap skill variance as a separate signal.** A future "consistency-from-lap-to-lap" metric (variance of per-lap 85th percentiles) could complement the per-point `consistency_sigma`. Two independent consistency channels: within-lap (current model) and between-lap (new). Not building it now — recorded in §13.9 as a v2 candidate.
- **(v1.2)** Should `pedal_press_rate_per_s` feed into a v1.3 motor-control model that replaces the current first-order low-pass with a slew-rate-limited filter? Captured in §13.13.
- **(v1.2)** Should `steering_aggression_deg_per_s` translate into a corner-entry yaw-input model (relevant only once §19.2 slip-based sim lands)? Captured in §19.2.6.

## 9. Alternatives considered

- Full G-G-V driver model (deferred).
- `skill_pct` inside `Car` (rejected).
- CSV → JSON segments on the fly (rejected).
- Drop legacy tracks (rejected — kept as fallback).
- Skip comparison PNG (kept).
- Synthetic telemetry by distance (rejected — time-resampled instead).
- Three CLIs (collapsed to two).
- Per-track familiarity in v1 (rejected).
- (v1.1) Keep YAML alongside JSON (rejected).
- (v1.1) `driver_tau_s=0.02` (rejected — too small).
- (v1.1) Low-pass before heuristic (rejected — order matters).
- (v1.1) Two separate CSVs per lap (rejected — one with `lap` column).
- (v1.1) MC on both laps (rejected — lap 2 only).
- **(v1.1 — fit) Average per-lap `skill_pct` values across laps** (compute a separate 85th percentile per lap, then mean them). Rejected — laps with fewer cornering samples (unfinished, short layouts) would be weighted equally with full laps, drowning out the higher-quality samples. **Pooling all cornering samples into one array and taking the 85th percentile of the pool is statistically more robust** — sample weight is proportional to data quantity. See §13.5.
- **(v1.1 — fit) Single-lap fit with stricter "pick a learned lap" guidance.** Rejected — user-facing constraint that has repeatedly produced silently-wrong results when users picked the wrong lap. Mandatory ≥2-lap input makes the constraint enforceable. See §13.1.
- **(v1.1 — fit) Exclude lap 1 (out-lap) by rule.** Rejected — the existing §13 trim logic (drop pre-start-line portion via ncp-wrap detection) already handles the standing-start prefix. The trimmed in-lap remainder contains valid cornering samples and there is no reason to throw them away. With multi-lap mandatory, no single lap is load-bearing anyway.
- **(v1.2) Keep hand-defaults for tau / trail-brake / throttle-ramp.** Rejected — once we have enough real telemetry per driver (≥2 laps required, typically 5–10), the hand-defaults systematically misrepresent driver-specific input shapes. Measured values give a personalised dynamic profile at zero extra cost. Hand-defaults remain as the explicit fallback when measurement fails.
- **(v1.2) Put `pedal_press_rate_per_s` / `steering_aggression_deg_per_s` at the top level of the driver JSON.** Rejected — they are *statistics*, not consumed parameters. Grouping them under `profile.dynamic` keeps consumed fields visually separable and signals intent to future readers.
- **(v1.2) Default `--telemetry-dt-ms` of 50.** Rejected — at τ = 120 ms, α(50 ms) ≈ 0.29 vs α(10 ms) ≈ 0.077. The 10 ms value gives smooth, AC-realistic traces; 50 ms still leaves visible staircasing on sharp transitions. Disk-size delta between 10 ms and 50 ms is irrelevant at our volumes.
- **(v1.2) Lap-selection rule "use every lap supplied".** Rejected — once a driver has been running a stint for an hour, the most-recent laps are most representative of their current familiarity / tyre state. Capping at 5–10 newest avoids early-stint laps dragging the profile.
- (reorg) Flat layout (rejected).
- (reorg) `prep_track.py` produces corners JSON (rejected).
- (reorg) Keep `Corner_Analysis/` (rejected).

## 10. Migration

- Legacy `segments`-based `Track` retained.
- `lap.py` requires third positional arg.
- `main.py` → `lap.py`.
- (v1.1) YAML → JSON driver migration.
- (v1.1) `fit_driver.py` CLI signature changes from single-CSV positional to **variadic CSV positionals** (`<car> <track> <output> <lap1.csv> <lap2.csv> [<lap3.csv> ...]`). Existing single-lap invocations now fail with the "≥2 laps" error (acceptance §11.20). User-facing change is documented in README and in the `fit_driver.py --help` text.
- **(v1.2)** Driver JSONs produced by v1.1 (no `profile` block) continue to load. `Driver.load` falls back to top-level `driver_tau_s`/`trail_brake_m`/`throttle_ramp_m` when `profile.dynamic` is absent.
- **(v1.2)** Default `--telemetry-dt-ms` raised from 100 to 10. Existing scripts that depended on 100 ms output must pass `--telemetry-dt-ms 100` explicitly. README updated.
- **(v1.2)** `fit_driver.py` gains `--newest N` (default 10) and applies the lap-selection rule (§13.12) when more than 10 candidate files are supplied (positional or via `--laps-glob`). Existing invocations with ≤10 inputs are unaffected.
- (reorg) File moves as before.
- (reorg) `tracks_config.json` added.
- (reorg) Existing CSVs remain as canonical reference.

## 11. Acceptance criteria (for manual QA in Quix Cloud)

1. `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_gp_a_ideal_line.csv drivers/pro.json` succeeds.
2. Comparison plot generally tracks AI speed.
3. `pro` vs `amateur` produces slower lap times for amateur.
4. `consistency_sigma: 0.5` produces MC stats on lap 2 only.
5. Centerline-only `layout_*.csv` works.
6. Legacy `monza` invocation works.
7. `--no-plot` runs without matplotlib.
8. `--no-telemetry` skips telemetry CSV.
9. Driver JSON validation: missing `skill_pct` or `skill_pct > 1.0` fails; missing v1.1 fields default cleanly; missing v1.2 `profile` block loads with top-level fallbacks.
10. Cross-track validation runs end-to-end against sim lap 2.
11. (reorg) `prep_track.py` parity check.
12. (reorg) `prep_car.py` round-trip.
13. (reorg) `corner_analysis.py` end-to-end.
14. (reorg) Corner analysis is telemetry-free.
15. (reorg) `tracks_config.json` is single source of truth.
16. (v1.1) Driver-lag smoothing visible at τ=120 ms.
17. (v1.1) Trail-brake heuristic produces visible linear taper.
18. (v1.1) Two-lap default emits two laps; lap 2 faster within `consistency_sigma`.
19. (v1.1) YAML → JSON migration leaves no `.yaml` in `drivers/`.
20. **(v1.1 — fit) Single-lap input is rejected.** `python fit_driver.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/out.json samples/aclog/one_lap.csv` exits non-zero with a clear error message containing the text "fit requires ≥2 laps; see §13". No output JSON is written.
21. **(v1.1 — fit) Six-lap pool produces a multi-lap JSON.** Fitting on Tomas's six AC laps (`samples/aclog/Tomas_Lap{1..6}.csv`) — where lap 1 has a standing-start prefix that gets trimmed and its in-lap remainder spans ≥0.9 NCP (counted as finished), laps 2–5 are fully-finished flying laps (each NCP span ≥0.9), and lap 6 is unfinished (NCP span < 0.9) — produces `drivers/tomas.json` whose `source` block has `n_laps == 6`, `n_finished_laps == 5`, `real_lap_times_s` array of length 5 (only finished laps), `real_lap_time_s` equal to the mean of those five times, `pooled_sample_count > 16000` (loose bound — exact value implementation-dependent), and valid `skill_pct` / `consistency_sigma` computed from the pooled cornering-sample array.
22. **(v1.2 — profile) Measured dynamics within reasonable bounds.** Fitting on Tomas's 6 lake laps produces `drivers/tomas.json` whose `profile.dynamic` satisfies: `driver_tau_s ∈ [0.05, 0.30]`, `trail_brake_m ∈ [15, 80]`, `throttle_ramp_m ∈ [20, 100]`, `pedal_press_rate_per_s > 1.0`. `measured.driver_tau_s`, `measured.trail_brake_m`, `measured.throttle_ramp_m`, `measured.pedal_press_rate_per_s` are all `true`. (Loose bounds — the point is "it produced a measured value, not a default.") `steering_aggression_deg_per_s` is `> 0` when `steerAngle` is present in the input CSVs, else `null` with `measured.steering_aggression_deg_per_s = false`.
23. **(v1.2 — telemetry cadence) 10 ms row count.** Telemetry CSV emitted at `--telemetry-dt-ms 10` has within ±10 % of 10× the row count of the same sim run emitted at `--telemetry-dt-ms 100`. Header bytes are identical.
24. **(v1.2 — ks_nurburgring sweep) Four-layout run.** With `drivers/tomas.json` produced from the freshest 5–10 Sprint A laps, `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_<L>.csv drivers/tomas.json --telemetry-dt-ms 10` succeeds for `<L> ∈ {gp_a, gp_b, sprint_a, sprint_b}` and prints two lap times each (lap 1 standing, lap 2 flying).
25. **(v1.2 — lap selection) `--newest N` picks the freshest N by mtime.** With 12 CSVs in `samples/aclog/Tomas_*.csv`, `python fit_driver.py ... --laps-glob "samples/aclog/Tomas_*.csv" --newest 10` selects the 10 newest-by-mtime files and emits a `source.lap_selection.selected_count == 10`, `selected_sources` listing those 10 paths, and `candidates_considered == 12`. With 4 CSVs available and `--newest 10`, all 4 are used and `selected_count == 4`. With 1 CSV, the fitter still errors on the ≥2 guard.

## 12. References

- `docs/AI_CONTEXT.md`, `README.md`, `src/lap_estimator/*.py`, `ModelCreationSteps.txt`, `Corner_Analysis/corner_analysis.py`, `samples/aclog/2026-04-07T135548_260Z_Tms_Lap2.csv`, `tracks_config.json`.

---

## 13. Telemetry-driven driver fitting (addendum, v1 + v1.1 multi-lap mandatory + v1.2 profile-dynamics)

### 13.1 Goal

Given **two or more** real AC telemetry CSVs for laps on a known car/track, derive `skill_pct` and `consistency_sigma` so that the simulator's lap time (with that car + track + derived driver) lands close to the mean of the supplied real laps. **(v1.2)** Additionally, measure the dynamic profile fields (`driver_tau_s`, `trail_brake_m`, `throttle_ramp_m`, `pedal_press_rate_per_s`, `steering_aggression_deg_per_s`) from the same telemetry instead of taking hand-defaults. The fit is **per-driver-stint, per-car, per-track**; we are not building a portable "driver personality" object yet.

**Hard requirement (v1.1):** the fit requires **≥2 laps**. A single-lap input is rejected at CLI parse time with a clear error ("`fit requires ≥2 laps; see §13`"). No fallback / no override flag. Rationale: a single lap is too noisy a signal — driver consistency, tyre warm-up state, traffic, and one-off mistakes all dominate per-lap variance. Pooling samples across multiple laps is statistically more robust than averaging per-lap skill values (see §13.5 step 5). Unfinished laps and out-laps still contribute usefully — the only invalid input is "one lap".

**Lap-selection rule (v1.2):** when more than 10 candidate laps are supplied (via positional CSVs, `--laps-glob`, or both — when interaction is allowed in a future iteration), keep only the **5–10 newest** by file mtime. The `--newest N` flag (default 10) sets the upper bound. If fewer than `N` candidates are available, use all of them (subject to the ≥2 minimum). See §13.12.

**Out-lap (lap 1) handling.** The existing §13.5 step 1 trim logic (drop the pre-start-line portion via `normalizedCarPosition` wrap-detection) is retained. The trimmed in-lap remainder is included in the pool — there is no longer a "use a learned lap, not lap 1" rule. With multi-lap mandatory, no single lap is load-bearing, so the previous out-lap advisory is superseded.

**v1.1 note on sim-emitted telemetry.** When `fit_driver.py` consumes a sim telemetry CSV produced by `lap.py`, the file contains both laps (lap 1 standing + lap 2 flying) with a `lap` column. The fitter splits on the `lap` column and treats each lap as one of the ≥2 required input laps. A `--lap {1,2}` override exists for debugging single-lap selection inside a sim file, but it implies "select that lap and require at least one more telemetry input alongside it".

### 13.2 Non-goals (fit tool)

- Fitting tyre / aero / engine parameters from telemetry.
- Per-corner skill profile (v2).
- Brake-vs-cornering skill split (v2).
- Detecting / discarding off-track or invalid laps (out of scope — user curates the input list).
- Live telemetry / streaming.
- A v2 "track familiarity" or per-track confidence term (§15 hook).
- **(v1.2)** Consuming `pedal_press_rate_per_s` / `steering_aggression_deg_per_s` inside the v1.2 simulator. They are statistics only; v1.3 picks them up.

### 13.3 CLI shape

```
python fit_driver.py <car_data_dir> <track_csv> <output_driver_json> \
                     [<lap1.csv> <lap2.csv> [<lap3.csv> ...]] \
                     [--ds 2.0] [--name <driver_name>] [--no-validate] [--no-plot] \
                     [--lap {1,2}] [--laps-glob "<pattern>"] [--newest N]
```

- `<car_data_dir>` — same shape `lap.py` accepts.
- `<track_csv>` — full track CSV.
- `<output_driver_json>` — destination path; directories created as needed.
- `<lap1.csv> <lap2.csv> [<lap3.csv> ...]` — **variadic positional**; **two or more required** *after* lap selection is applied. Each is a raw AC log (or a sim-emitted CSV containing the `lap` column). Single-CSV input is a hard error at argparse-validation time: `error: fit requires ≥2 laps; see §13`.
- `--laps-glob "<pattern>"` — alternative to listing CSVs individually. Expands the glob at runtime; after `--newest` filtering, the resolved list must still contain ≥2 files or the same error fires. Mutually exclusive with positional CSVs: either pass positionals OR `--laps-glob`, not both (argparse error if both). Example: `--laps-glob "samples/aclog/Tomas_Lap*.csv"`.
- **(v1.2)** `--newest N` — integer cap on the number of candidate laps to keep, ranked newest-first by file mtime. Default `10`. Minimum effective value is `2` (enforced by the ≥2 guard, not by argparse). Applied uniformly to positionals or to the `--laps-glob` expansion. If the candidate list has ≤ `N` files, all of them are used.
- `--name` — overrides the auto-generated `name`. Default: derived from the **first** telemetry filename stem with the lap suffix stripped.
- `--no-validate` — skip the post-fit sim validation pass.
- `--no-plot` — skip plotting.
- `--lap {1,2}` — when an input telemetry CSV has a `lap` column (sim-emitted, §14.5), select this lap from each such file. Default: `2`. Ignored on real AC logs with no `lap` column. Each sim-emitted CSV still counts as **one** input file regardless of how many laps it contains internally — to feed both laps of a sim file in, pass it twice with different `--lap` values, or split it outside the fitter.

### 13.4 Input contract

**AC telemetry CSV** (confirmed columns from `samples/aclog/...`):

| Column | Unit | Used for |
|---|---|---|
| `timestamp_ms` | ms | Per-lap real time = `max - min` / 1000; pedal-edge timing; steering-rate timing |
| `gas` | 0..1 | (v1.2) Leading-edge tau + pedal-press-rate + throttle-ramp measurement |
| `brake` | 0..1 | (v1.2) Leading-edge tau + pedal-press-rate + trail-brake measurement |
| `distanceTraveled` | m | Merge key against track CSV `distance_m`; (v1.2) ramp-length measurement |
| `speedKmh` | km/h | Converted to m/s for `v` |
| `normalizedCarPosition` | 0..1 | Lap-finished detection (≥0.9 span = finished) + out-lap trim |
| `steerAngle` (v1.2, optional) | rad or deg (units detected from range) | Steering-aggression measurement; absent → null |
| `lap` (sim-emitted only) | int | Lap selector when present |

**(v1.2) `telemetry.read_ac_log`** is extended to accept the broader AC channel set. `steerAngle` is opportunistically loaded if present; missing values emit a single warning per file ("`steerAngle absent — steering aggression will be unmeasured`") and the column is filled with `NaN`. Other AC channels listed in §19.2.4 are not required by v1.2 but the reader does not reject them — extra columns are passed through to the merged frame so v1.3 can pick them up without another schema change.

**Track CSV:** §7.1.
**Car dir:** §7.3.

### 13.5 Algorithm

All steps live in `driver_fit.fit_driver(car, track_df, telemetry_dfs: list) -> FitResult`. The CLI is a thin wrapper that loads the variadic input list, applies lap selection (§13.12), calls this function, writes the JSON, and (unless `--no-validate`) runs the validation sim.

**Pre-flight: lap-count guard.**
- If `len(telemetry_dfs) < 2`, raise `ValueError("fit requires ≥2 laps; see §13")`. The CLI catches this and exits non-zero with the same message.

**Per-lap pipeline (steps 1–4 run independently for each input lap):**

**Step 1 — Out-lap trim and merge with track on distance.**
- Detect out-lap by checking whether the lap's `normalizedCarPosition` series starts above some threshold (e.g. > 0.05) and wraps through zero — that's a standing-start prefix that crosses the start line mid-CSV. If detected, drop the pre-wrap portion.
- For sim-emitted CSVs with a `lap` column, filter to the chosen lap (`--lap`, default 2) before merge.
- Use `telemetry.merge_with_track(telem, track_df)`.
- Output: merged dataframe with `distance_m, speed_ms, radius_m, gradient_pct, gas, brake, timestamp_ms, normalizedCarPosition` (plus `steerAngle` and any other passthrough columns when present).

**Step 2 — Per-point observed lateral G.**
- `lat_g_obs[i] = v[i]**2 / radius_m[i] / 9.81`.
- Points where `radius_m[i] >= STRAIGHT_THRESHOLD_M` (default 500 m) are flagged as straight and **excluded** from the skill fit.

**Step 3 — Theoretical max lateral G.**
- `lat_g_max[i] = car.tyre_grip_lateral(v[i]) * (1 + downforce(v[i]) / (m*g))`.

**Step 4 — Per-point grip utilisation.**
- `util[i] = lat_g_obs[i] / lat_g_max[i]`.
- Clip to `[0.0, 1.2]`. Warning if > 5% of cornering samples exceed 1.0.
- Tag each lap with `finished = (ncp_span >= 0.9)` where `ncp_span = max(ncp) - min(ncp)` over the (post-trim) lap. Finished laps additionally contribute `real_lap_time_s = (max(timestamp_ms) - min(timestamp_ms)) / 1000` to the per-lap times list.

**Step 5 — Pool across laps, aggregate.**
- **Pool all cornering-sample `util` values from all laps into one big array** (`pooled_util`).
- `skill_pct = clip(percentile(pooled_util, 85), 0.05, 1.0)`.
- `consistency_sigma_util = stdev(pooled_util)`.
- `consistency_sigma_seconds = round(consistency_sigma_util / 0.03, 2)`, clamped to `[0.0, 1.5]`. (Existing heuristic mapping.)
- **Rationale (locked):** pooling — not per-lap-skill averaging — is statistically more robust. A 4000-sample lap contributes proportionally more weight than a 500-sample lap, which is the right behaviour: more data → more weight, automatically. Averaging per-lap percentiles would treat a short stint and a full lap as equals, drowning out the high-quality samples. The pooled-percentile interpretation is straightforward: "across the pooled cornering history, the driver achieved at least this utilisation in 85% of samples". See §9 for the rejected alternative.

**Step 5b — (v1.2) Measure dynamic profile.**
- Call `profile_dynamics.measure_dynamics(merged_frames)` (the same per-lap merged frames as steps 1–4). See §13.11 for the algorithm.
- Returns a `ProfileDynamics` with `driver_tau_s`, `trail_brake_m`, `throttle_ramp_m`, `pedal_press_rate_per_s`, `steering_aggression_deg_per_s`, plus per-field `measured` booleans and `sample_counts`.
- Each measured-true field overrides the hand-default and is written under `profile.dynamic.*`. Each measured-false field falls back to the hand-default (or `null` for the two statistics-only fields) and still emits the `profile.dynamic` block — readers can inspect `measured.*` to know what came from where.

**Step 6 — Worked example (illustrative, multi-lap):**
- Six laps, pooled cornering samples ≈ 18 000 (lap 1 trimmed contributes ~2 100; laps 2–5 contribute ~3 200 each; lap 6 unfinished contributes ~1 600). 85th percentile of pool → `skill_pct ≈ 0.88`. Stdev → `consistency_sigma_seconds ≈ 0.42`. Profile-dynamics across the same pool yields `driver_tau_s ≈ 0.14`, `trail_brake_m ≈ 28.5`, `throttle_ramp_m ≈ 47.0`, `pedal_press_rate_per_s ≈ 6.4`, `steering_aggression_deg_per_s ≈ 312` (steerAngle present).

**Step 7 — Pre-fit summary (printed before validation):**
```
Laps loaded:      6   (5 finished, 1 unfinished)
Pooled samples:   17834 cornering points
Mean real lap time (finished): 1:42.418
Profile dynamics: τ=0.140 s  trail=28.5 m  ramp=47.0 m  press=6.4/s  steer95=312°/s
```

**Step 8 — Emit JSON.**
```json
{
  "name": "tomas",
  "skill_pct": 0.88,
  "consistency_sigma": 0.42,
  "driver_tau_s": 0.14,
  "trail_brake_m": 28.5,
  "throttle_ramp_m": 47.0,
  "profile": {
    "dynamic": {
      "driver_tau_s": 0.14,
      "trail_brake_m": 28.5,
      "throttle_ramp_m": 47.0,
      "pedal_press_rate_per_s": 6.4,
      "steering_aggression_deg_per_s": 312.0,
      "measured": {
        "driver_tau_s": true,
        "trail_brake_m": true,
        "throttle_ramp_m": true,
        "pedal_press_rate_per_s": true,
        "steering_aggression_deg_per_s": true
      },
      "sample_counts": {
        "pedal_leading_edges": 184,
        "brake_taper_segments": 22,
        "throttle_ramp_segments": 24,
        "steering_samples": 198432
      }
    }
  },
  "source": {
    "telemetry_csvs": [
      "samples/aclog/Tomas_Lap1.csv",
      "samples/aclog/Tomas_Lap2.csv",
      "samples/aclog/Tomas_Lap3.csv",
      "samples/aclog/Tomas_Lap4.csv",
      "samples/aclog/Tomas_Lap5.csv",
      "samples/aclog/Tomas_Lap6.csv"
    ],
    "track_csv": "tracks_csv/ks_nurburgring/layout_sprint_a.csv",
    "car_data_dir": "cars_csv/bmw_1m",
    "n_laps": 6,
    "n_finished_laps": 5,
    "real_lap_times_s": [102.135, 101.998, 102.402, 102.310, 103.244],
    "real_lap_time_s": 102.418,
    "pooled_sample_count": 17834,
    "sim_lap_time_s": null,
    "delta_s": null,
    "lap_selection": {
      "rule": "newest-5-to-10",
      "candidates_considered": 6,
      "selected_count": 6,
      "selected_sources": [
        "samples/aclog/Tomas_Lap1.csv",
        "samples/aclog/Tomas_Lap2.csv",
        "samples/aclog/Tomas_Lap3.csv",
        "samples/aclog/Tomas_Lap4.csv",
        "samples/aclog/Tomas_Lap5.csv",
        "samples/aclog/Tomas_Lap6.csv"
      ]
    },
    "fitted_at": "2026-05-13T14:22:01Z",
    "fit_version": "2"
  }
}
```
- Top-level `driver_tau_s` / `trail_brake_m` / `throttle_ramp_m` mirror `profile.dynamic.*` (or fall back to hand-defaults when measured-false) for backward compatibility with v1.1 readers.
- `telemetry_csvs` (list) replaces the v1 scalar `telemetry_csv`. Always populated by the fitter even when only 2 laps are supplied.
- `n_laps` — total **selected** input laps (length of `telemetry_csvs` after `--newest` filtering).
- `n_finished_laps` — count of laps whose post-trim `ncp_span >= 0.9`.
- `real_lap_times_s` (list) — per-lap real lap times in seconds, **only for finished laps**, in the order the laps were supplied. Length equals `n_finished_laps`.
- `real_lap_time_s` (scalar) — mean of `real_lap_times_s`. Kept under this exact key name for backward compatibility with downstream readers (`lap.py --validate-against` and any pre-v1.1 driver-JSON consumer that read this field).
- `pooled_sample_count` — total cornering samples used in the percentile (after straight-exclusion and out-lap trim).
- `sim_lap_time_s` — populated by the validation sim (lap 2); `null` when `--no-validate`.
- `delta_s` — `sim_lap_time_s - real_lap_time_s` (mean real vs sim lap 2); `null` when `--no-validate`.
- `lap_selection` (v1.2) — block recording how the input list was filtered. `rule` is the constant `"newest-5-to-10"` for v1.2. `candidates_considered` counts the pre-filter pool size; `selected_count` is the post-filter count (== `n_laps`); `selected_sources` is the post-filter list (== `telemetry_csvs`).
- `fit_version` — `"2"` when `profile` is populated; `"1"` for legacy v1.1 fits.

**Step 9 — Validation pass (default on, post-fit).**
Run a two-lap sim with the freshly-emitted JSON and print:
```
Real laps used:                6  (5 finished)
Mean real lap time (finished): 1:42.418
Sim lap 2 (flying):            1:43.402
Delta vs mean real:            +0.984 s  (+0.96%)
Verdict:                       GOOD
```
- Verdicts use the existing thresholds (unchanged): `GOOD` (|d| < 3 s AND |%| < 5), `LOOSE` (5–10 %), `BAD` (> 10 %). Thresholds compare the sim lap 2 against the **mean** of the finished real lap times.
- Patches `source.sim_lap_time_s` and `source.delta_s` (where `delta_s = sim_lap2_time - mean_real`).
- If `|delta| > 3 s`, the verdict line surfaces it as `LOOSE` or `BAD` rather than as a separate warning.

### 13.6 Output contract (JSON schema additions)

- `profile.dynamic` — (v1.2) populated by `fit_driver.py`. Full schema in §7.2 / §13.5 step 8.
- `source` — optional object; full key list in §13.5 step 8.
- **v2 hook:** `corners` — optional list of per-corner overrides. Reserved key.

### 13.7 Module changes

- **New / updated:** `src/lap_estimator/telemetry.py` — `read_ac_log(path)` and `merge_with_track(...)`. (v1.2) Accepts `steerAngle` opportunistically; passes through unknown columns.
- **New / updated:** `src/lap_estimator/driver_fit.py` — `fit_driver(car, track_df, telemetry_dfs: list, *, straight_threshold_m=500.0, util_percentile=85) -> FitResult`. Owns the lap-count guard, per-lap pipeline, pooling, dynamic-profile invocation (v1.2), and `FitResult` shape.
- **New (v1.2):** `src/lap_estimator/profile_dynamics.py` — `measure_dynamics(merged_frames) -> ProfileDynamics`. See §13.11.
- **New / updated:** `fit_driver.py` (CLI at repo root) — argparse with variadic positionals, `--laps-glob`, `--newest N` (v1.2), lap-count guard at parse time, loads inputs, applies lap selection, calls library, writes JSON, runs validation sim, patches JSON.
- **Touched:** `src/lap_estimator/driver.py` — `Driver.load` tolerates the new `source` keys and the v1.2 `profile.dynamic` block.
- **Untouched:** `car.py`, `simulator.py`, `report.py`.

### 13.8 `Corner_Analysis/` handling

User-managed scratch — not depended on by the fit tool. Sample telemetry lives under `samples/aclog/`.

### 13.9 Open questions / v2 candidates

- Per-corner skill profile, brake-vs-cornering split, percentile choice, skill ceiling clipping.
- **Per-lap skill variance as a separate signal.** Compute the 85th percentile per lap as well as the pooled percentile, and surface the spread of per-lap values as an independent "consistency-from-lap-to-lap" metric. Two consistency channels would then live in the driver schema: `consistency_sigma` (within-lap, from pooled stdev — current model) and `consistency_lap_to_lap` (between-lap, new). Useful for distinguishing "always 88% util" from "ranges 75–95% util across laps". Not building it now — recorded here so the v2 schema can add the field forward-compatibly.
- **Per-lap weighting controls.** Currently sample-count-weighted by construction (pooling). A future option could let the user weight laps explicitly (e.g. discount short stints, boost a known-clean lap).
- ~~**(v1.1)** Fit `driver_tau_s` from telemetry by measuring throttle-to-G lag.~~ **Closed in v1.2 — see §13.11.**
- ~~**(v1.1)** Fit `trail_brake_m` / `throttle_ramp_m` from corner-entry / corner-exit telemetry.~~ **Closed in v1.2 — see §13.11.**

### 13.10 Acceptance criteria (fit tool)

1. Running `python fit_driver.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json samples/aclog/Tomas_Lap1.csv samples/aclog/Tomas_Lap2.csv samples/aclog/Tomas_Lap3.csv samples/aclog/Tomas_Lap4.csv samples/aclog/Tomas_Lap5.csv samples/aclog/Tomas_Lap6.csv` succeeds, writes a JSON that loads cleanly via `Driver.load`, prints the multi-lap delta block (§13.5 step 9).
2. JSON's `skill_pct ∈ (0, 1]`, `consistency_sigma ∈ [0, 1.5]`, **(v1.2)** `profile.dynamic` populated with measured-true booleans on the four pedal/ramp fields, `source` fully populated per §13.5 step 8.
3. Running `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json` reproduces `source.sim_lap_time_s` within numerical noise.
4. Validation delta `|sim_lap2 - mean_real|` < 5 s on the bundled multi-lap Nurburgring Sprint pool.
5. `--no-validate` skips the sim; `source.sim_lap_time_s` / `source.delta_s` stay `null`.
6. Missing required telemetry column → fail fast. **(v1.2)** Missing optional `steerAngle` → warn and continue; `profile.dynamic.steering_aggression_deg_per_s` becomes `null` with `measured.steering_aggression_deg_per_s = false`.
7. Non-overlapping distance ranges → clear error.
8. Tool does not read or write inside `Corner_Analysis/`.
9. **(v1.1) Single-lap input rejected.** See §11.20.
10. **(v1.1) Multi-lap JSON shape.** See §11.21.
11. **(v1.1) `--laps-glob` shorthand.** `python fit_driver.py <car> <track> <out> --laps-glob "samples/aclog/Tomas_Lap*.csv"` produces an output JSON byte-identical to listing the six files positionally (modulo the `fitted_at` timestamp and modulo v1.2 lap-selection metadata).
12. **(v1.1) `--laps-glob` resolving to <2 files** errors out with the same "≥2 laps" message; no JSON written.
13. **(v1.1) Mixing positional CSVs with `--laps-glob`** is rejected at argparse-validation time.
14. **(v1.2) `--newest N` cap.** See §11.25.
15. **(v1.2) Profile-dynamics fields measured within plausible bounds.** See §11.22.

### 13.11 Profile-dynamics algorithm (v1.2 — new)

Lives in `src/lap_estimator/profile_dynamics.py`. Single entry point:

```python
def measure_dynamics(
    merged_frames: list,        # list of per-lap DataFrames from §13.5 step 1
    *,
    rising_edge_low: float = 0.10,
    rising_edge_high: float = 0.50,
    min_rise_window_s: float = 0.20,
    pedal_hysteresis_low: float = 0.05,
    taper_high: float = 0.80,
    taper_low: float = 0.10,
    ramp_low: float = 0.40,      # lower bound for "corner-exit partial"
    ramp_high: float = 0.95,
    corner_detect_speed_window_s: float = 2.0,
    steering_percentile: float = 95.0,
) -> ProfileDynamics: ...
```

`ProfileDynamics` is a dataclass:

```
driver_tau_s: float | None
trail_brake_m: float | None
throttle_ramp_m: float | None
pedal_press_rate_per_s: float | None
steering_aggression_deg_per_s: float | None
measured: dict[str, bool]
sample_counts: dict[str, int]
```

**Step A — Leading-edge detection (for `driver_tau_s` and `pedal_press_rate_per_s`).**

For each lap, for each of `gas` and `brake`:
1. Compute hysteresis-armed rising edges: walk the trace; the channel is "armed" when it has been below `pedal_hysteresis_low` (0.05). Once armed, a sample crossing `rising_edge_low` (0.10) starts an edge candidate. The candidate completes when the channel reaches `rising_edge_high` (0.50). If the channel does not reach 0.5 within `min_rise_window_s` (200 ms), discard the candidate and re-arm when it dips below 0.05 again.
2. For each completed edge:
   - `t_50` = time at which the channel first crossed 0.5.
   - `t_start` = time at which the channel first crossed 0.10 (start of edge).
   - `tau_candidate = t_50 - t_start`. This is the time-to-50 % from the toe-press start.
   - `slope_candidate = (y[t_50] - y[t_start]) / (t_50 - t_start)`. This is the per-second press rate (units: 1/s, i.e. fraction-per-second).
3. Pool `tau_candidate` and `slope_candidate` across both channels and all laps.

`driver_tau_s = median(tau_candidate_pool)` (clamped to `[0.02, 0.50]`).
`pedal_press_rate_per_s = median(slope_candidate_pool)`.
If fewer than 10 edges total across the pool → both fields fallback (`driver_tau_s = 0.12`, `pedal_press_rate_per_s = null`) and `measured.*` for each is `false`.

**Step B — Corner-limited segment detection (shared by trail-brake and throttle-ramp).**

For each lap:
1. Smooth `speedKmh` with a moving-average over `corner_detect_speed_window_s` (2 s window).
2. Find local minima of the smoothed speed trace whose value is below `0.85 * lap_max_speed`. Each minimum is the "apex" of a corner-limited segment.
3. For each minimum:
   - `entry_distance_m` = `distance_m` at the apex.
   - Walk backward in distance until either `brake` drops below `taper_low` or a previous segment's exit was passed — call this `brake_release_distance_m`.
   - Walk forward in distance until `gas` reaches `ramp_high` or another segment's apex is reached — call this `throttle_full_distance_m`.

**Step C — Trail-brake distance (for `trail_brake_m`).**

For each detected segment:
1. Find the last sample before the apex at which `brake >= taper_high` (0.80) — call this `t_brake_peak`.
2. Find the first sample after `t_brake_peak` at which `brake <= taper_low` (0.10) — call this `t_brake_off`.
3. `trail_brake_candidate = distance_m[t_brake_off] - distance_m[t_brake_peak]` (clamped to ≥ 0).
4. Discard candidates where peak brake never reached `taper_high` (driver didn't actually firm-brake) or where the taper window exceeds 200 m (likely a misdetection).

`trail_brake_m = median(trail_brake_candidate_pool)` (clamped to `[5.0, 150.0]`).
If fewer than 5 candidates → fallback to 30.0 with `measured.trail_brake_m = false`.

**Step D — Throttle-ramp distance (for `throttle_ramp_m`).**

For each detected segment:
1. Find the first sample after the apex at which `gas >= ramp_low` (0.40) — call this `t_throttle_partial`.
2. Find the first subsequent sample at which `gas >= ramp_high` (0.95) — call this `t_throttle_full`.
3. `throttle_ramp_candidate = distance_m[t_throttle_full] - distance_m[t_throttle_partial]` (clamped to ≥ 0).
4. Discard candidates where the channel never reaches `ramp_high` within the segment, or where the ramp window exceeds 250 m.

`throttle_ramp_m = median(throttle_ramp_candidate_pool)` (clamped to `[5.0, 200.0]`).
If fewer than 5 candidates → fallback to 40.0 with `measured.throttle_ramp_m = false`.

**Step E — Steering aggression (for `steering_aggression_deg_per_s`).**

If no merged frame contains a `steerAngle` column with non-NaN values → return `null`, `measured = false`.
Otherwise:
1. For each lap, compute `d_steer = diff(steerAngle) / diff(timestamp_s)` (per-sample finite difference).
2. Convert to degrees-per-second if input units look like radians (heuristic: max |steerAngle| < 5 → radians, else degrees). Record the unit decision in `sample_counts.steering_unit_detected` (string `"rad"` or `"deg"`).
3. Pool `|d_steer|` across all laps.
4. `steering_aggression_deg_per_s = percentile(pool, 95)` (clamped to `[0.0, 5000.0]`).

**Step F — Assemble result.**
- `measured.<field>` is `true` iff the corresponding measurement path succeeded (≥ minimum candidates, channel present).
- `sample_counts.pedal_leading_edges` = total edges across both channels and all laps.
- `sample_counts.brake_taper_segments` = number of trail-brake candidates retained.
- `sample_counts.throttle_ramp_segments` = number of throttle-ramp candidates retained.
- `sample_counts.steering_samples` = total non-NaN samples in pooled steering trace.

### 13.12 Lap-selection rule (v1.2 — new)

**Rationale.** Once a driver has accumulated more than 10 laps in a stint, the oldest laps are typically less representative of their current familiarity, tyre state, and dialled-in line. Capping the fit at the newest 5–10 laps preserves data quality without forcing the user to manually pick.

**Algorithm.**
1. Collect candidate paths. If positional CSVs are supplied, that list **is** the candidate set. If `--laps-glob` is supplied, expand the glob to produce the candidate set.
2. Rank the candidate set by file modification time, newest first. Ties broken by lexicographic filename ordering.
3. If `len(candidates) > --newest N` (default 10): truncate to the first `N`.
4. If `len(candidates) < 2`: error out with the `"fit requires ≥2 laps; see §13"` message.
5. Otherwise: use 2 ≤ `len(selected)` ≤ `N`. The hard minimum of 2 is enforced; if the user runs `--newest 1`, argparse rejects.

**Logging.** The fitter prints, before the pre-fit summary:
```
Lap selection: 12 candidates → kept 10 newest (range 2026-05-09 14:22 .. 2026-05-13 09:48)
```

**JSON record.** `source.lap_selection` captures the rule name, candidate count, selected count, and selected source paths in newest-first order. See §13.5 step 8.

### 13.13 Open questions / v3 candidates (v1.2-specific)

- **Replace the first-order low-pass with a slew-rate-limited filter parameterised by `pedal_press_rate_per_s`.** The current low-pass smooths edges uniformly; a slew-rate limiter would more faithfully reproduce "driver hits the brake at X /s" behaviour. Captured for v1.3.
- **Steering channel as a sim input.** Once §19.2 (slip-based sim) lands, `steering_aggression_deg_per_s` becomes a direct input to the corner-entry yaw model. Until then it is a statistic only.
- **Per-corner profile-dynamics overrides.** Currently we measure global medians; a future v2 hook could surface per-corner-type ramp distances (hairpin vs sweeper) under `profile.dynamic.by_corner_type`.

---

## 14. Synthetic telemetry emission (addendum, v1 + v1.1 + v1.2)

### 14.1 Goal

After every sim run, emit a CSV that is **schema-identical to a real AC telemetry log** (plus a trailing `lap` column — v1.1) so downstream tools — first and foremost `fit_driver.py` — can consume sim output the same way they consume real laps.

### 14.2 Non-goals

- Human-realistic input traces.
- Channels not in the AC sample.
- Wall-clock-anchored `timestamp_ms`.
- Re-using sim's exact `ds` cadence.

### 14.3 Gas/brake derivation pipeline (v1.1 — replaces v1's simple piecewise rule)

The emitted `gas` / `brake` traces are built in **three layers**, in this exact order:

1. **Layer 1 — limit-label rule (unchanged from v1).** Per sample, look up the simulator's per-point binding-limit label and emit:
   - `accel` → `gas = 1.0, brake = 0.0`
   - `brake` → `gas = 0.0, brake = 1.0`
   - `corner` → `gas = required_drive_force / car.max_traction_force(v)`, `brake = 0.0`, clipped to `[0, 1]`.

2. **Layer 2 — corner-shape heuristic (v1.1).**
   - **Trail-brake taper.** For `trail_brake_m` metres preceding a `brake → corner|accel` transition, linearly taper `brake` from 1.0 down to 0.0.
   - **Throttle ramp-up.** For `throttle_ramp_m` metres after a `corner → accel` transition, linearly ramp `gas` from corner-exit partial value up to 1.0.
   - Either field set to `0.0` disables that heuristic.

3. **Layer 3 — driver-lag low-pass (v1.1).** 1st-order IIR low-pass on `gas` and `brake`:
   ```
   y[n] = y[n-1] + α · (x[n] - y[n-1])
   α   = dt / (driver_tau_s + dt)
   ```
   - `driver_tau_s = 0.0` bypasses the filter.
   - Default `driver_tau_s = 0.12 s`.
   - **(v1.2)** At the new default `--telemetry-dt-ms 10`, `α = 0.010 / (0.120 + 0.010) ≈ 0.077` — i.e. the smoothing carries 92 % of the previous output forward each step, producing visibly smoother traces than the v1.1 100 ms default (`α = 0.455`). With a measured τ from `profile.dynamic.driver_tau_s` (typically 0.10–0.20 s for human drivers) the same shape applies.

**Order summary:** `limit-label rule → corner-shape heuristic → driver-lag low-pass → CSV write`.

### 14.4 Decisions baked in (v1 + v1.1 + v1.2)

1. **Schema:** AC schema + `lap` column.
2. **Cadence:** **(v1.2)** 10 ms default (was 100 ms in v1.1). Override via `--telemetry-dt-ms`.
3. **Time origin:** monotonic across lap boundary.
4. **`normalizedCarPosition`** resets at lap boundary.
5. **`distanceTraveled`** resets to 0 at lap 2 start.
6. **Resampling:** linear interp in time.
7. **Gas/brake reconstruction:** three-layer pipeline (§14.3).
8. **Emission default-on.**
9. **File path:** single file with two laps.
10. **Module:** `sim_telemetry.py`.
11. **Driver-input model (v1.1):** trail-brake / throttle-ramp + low-pass replaces deferred-to-v2 hook.
12. **Two CLIs, not three.**
13. **Local-file-only prototype.**
14. **(v1.2) Driver-input values sourced from `profile.dynamic.*` when present, falling back to top-level driver fields, falling back to hand-defaults.** The simulator does not care which path was taken; `Driver.load` resolves them at load time.

### 14.5 CLI surface (additions to `lap.py`)

```
[--no-telemetry] [--telemetry-dt-ms 10] [--single-lap]
```

**(v1.2)** `--telemetry-dt-ms` default is `10`. Set to `100` to reproduce v1.1 output volume.

### 14.6 Output schema

```
timestamp_ms,gas,brake,distanceTraveled,speedKmh,normalizedCarPosition,lap
```

### 14.7 Resampling algorithm

Walks sim step→time map, builds uniform output grid per lap (timestamps continue across), linear-interp distance/speed, nearest-neighbour binding label, reconstructs gas/brake per §14.3.

### 14.8 Module: `src/lap_estimator/sim_telemetry.py`

```python
def write_synthetic_log(
    sim_result,
    car,
    driver,
    track_total_length_m: float,
    output_path: str,
    *,
    telemetry_dt_ms: int = 10,
) -> None: ...
```

**(v1.2)** Default kwarg value bumped from 100 to 10 to match the CLI default.

### 14.9 Loop-closure acceptance criterion

Fit a driver from ≥2 laps → emit sim telemetry → re-fit using sim's lap 1 + lap 2 as the two required inputs. Pass criteria:
- `|skill_pct_loop - skill_pct_real| / skill_pct_real <= 0.05`.
- `|sim_lap_time_loop - sim_lap_time_real| <= 1.0` s.
- `skill_pct_loop` is not trivially 1.0.

### 14.10 Acceptance criteria (sim-telemetry emission)

1. Default `lap.py` produces telemetry CSV.
2. Header byte-matches.
3. `timestamp_ms` monotonic across boundary.
4. `distanceTraveled` resets per lap.
5. `normalizedCarPosition` in [0, 1), resets at boundary.
6. `gas` and `brake` never both > 0.05.
7. Straight: `gas ≈ 1.0, brake ≈ 0.0`.
8. `--no-telemetry` skips file.
9. `--telemetry-dt-ms` scales row count.
10. Loop closure passes (per §14.9 — re-fits using both laps of the sim telemetry as the two required `fit_driver.py` inputs).
11. **(v1.1)** With `driver_tau_s = 0`, `trail_brake_m = 0`, `throttle_ramp_m = 0`, output matches v1 bang-bang byte-for-byte (modulo `lap` column).
12. **(v1.2)** Default `--telemetry-dt-ms` is 10. See §11.23.

### 14.11 Open questions / v2 candidates

Full motor-control model, steering channel, extra channels, wall-clock timestamps, multi-lap (>2) output, cadence calibration.

---

## 15. Cross-track validation workflow (addendum, v1)

### 15.1 Goal — the actual point of the tool

Predict how a known driver (fit on Track A) will perform on a new track (Track B, same car), then validate against a real AC lap on Track B.

The CLI is `lap.py --validate-against`. v1.1: validation compares the real lap against **sim lap 2**.

### 15.2 End-to-end workflow

1. Drive Track A in AC for ≥2 laps; capture telemetry.
2. `python fit_driver.py <car_dir> <track_a_csv> drivers/<name>.json <lap1.csv> <lap2.csv> [...]`.
3. Choose Track B.
4. `python lap.py <car_dir> <track_b_csv> drivers/<name>.json` → predicted lap times.
5. Drive Track B in AC; capture telemetry.
6. `python lap.py <car_dir> <track_b_csv> drivers/<name>.json --validate-against <real_lap_on_b>.csv`.

**(v1.2) End-to-end workflow box for the ks_nurburgring sweep:**

1. Pull the 5–10 newest laps for `<driver>` on Sprint A.
2. `python fit_driver.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/<driver>.json --laps-glob "samples/aclog/<driver>_*.csv" --newest 10` → produces `drivers/<driver>.json` with enriched `profile.dynamic`.
3. For each `<L> ∈ {gp_a, gp_b, sprint_a, sprint_b}`:
   `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_<L>.csv drivers/<driver>.json --telemetry-dt-ms 10` → two-lap prediction + 10 ms telemetry per layout.
4. Compare predicted lap times across the four layouts. Once real laps exist for the other three layouts, validate via `--validate-against`.

### 15.3 CLI shape

```
python lap.py <car_data_dir> <track_csv> <driver_json> \
              --validate-against <real_telemetry_csv> \
              [--ds 2.0] [--no-plot] [--bin-m 100] [--per-corner] [--single-lap]
```

### 15.4 Algorithm

Lives in `validate.validate_lap(...)`. Selects **lap 2** by default; resamples real onto sim lap-2 distance grid; bins; computes deltas; categorises.

### 15.5 Outputs

**Stdout:**
```
Track:               tracks_csv/brands_hatch/layout_indy_ideal_line.csv
Driver:              drivers/ludvik_nurburgring_sprint.json  (fit on layout_sprint_a, 6 laps)
Real lap:            1:24.812
Sim lap 2 (flying):  1:26.301  (predicted)
Delta:               +1.489 s  (+1.76%)
Verdict:             GOOD
```

Verdicts: GOOD (|d| < 3 s AND |%| < 5), LOOSE (5–10 %), BAD (> 10 %).

**Files:** `<track_stem>__<driver_name>_validation_overlay.png`, `<track_stem>__<driver_name>_validation_bins.csv`.

### 15.6 Limitations

v1 assumes `skill_pct` / `consistency_sigma` are track-agnostic. v2 candidates as before.

### 15.7 Module placement

- **New:** `src/lap_estimator/validate.py`.
- **No new CLI.**
- **Reused:** `telemetry.py`, `simulator.py`, `report.py`.

### 15.8 Acceptance criteria (validate flow)

1. End-to-end flag invocation runs.
2. Verdict matches thresholds.
3. Bins CSV well-formed.
4. Overlay PNG produced unless `--no-plot`.
5. Real-telemetry parsing reuses `telemetry.py`.
6. `--validate-against` is read-only.
7. Without flag, no validation outputs.
8. **Soft acceptance:** |delta_s| < ~3 s on a 2–3 min lap.
9. **(v1.1)** Default targets sim lap 2; `--single-lap` switches to lap 1 with warning.

### 15.9 Open questions

"Representative lap" definition; reverse-fit; threshold tuning.

---

## 16. Preparation pipeline (addendum, v1 reorg)

### 16.1 Goal

Single scripted path from raw AC content under `cars_in/` and `tracks_in/` to repo-friendly CSV artefacts.

### 16.2 Non-goals

- Re-encrypting AC data.
- Modifying AC source folders.
- Generating corner JSON (that's §17).
- Wrapping low-level decoders.
- Validating AC physics data.

### 16.3 `prep/prep_car.py`

```
python prep/prep_car.py <cars_in_dir> [--output-root cars_csv]
```

Behaviour: validate `data.acd` exists, derive car name, invoke `decode_acd`, write to `cars_csv/<car>/data/`.

### 16.4 `prep/prep_track.py`

```
python prep/prep_track.py <tracks_in_dir> [--output-root tracks_csv] [--ds 1.0] [--layouts all|<name>,...]
```

Behaviour: detect layouts via `ai/fast_lane.ai`, compute rich per-point CSV columns, resample to uniform `ds` grid, write `tracks_csv/<track>/layout_<L>.csv`.

### 16.5 `prep/decode_acd.py` and `prep/decode_track.py`

Existing code moved into `prep/`, refactored to expose callable functions; `__main__` blocks retained.

### 16.6 Prerequisites for adding a new car or track

**New car:** copy AC folder → `cars_in/<car>/` → `python prep/prep_car.py cars_in/<car>` → done.
**New track:** copy → `tracks_in/<track>/` → `python prep/prep_track.py tracks_in/<track>` → (optional) corner analysis → done.

### 16.7 Acceptance criteria (prep)

See §11.11, §11.12.

---

## 17. Corner analysis (addendum, v1 reorg)

### 17.1 Goal

Turn a track CSV into a canonical, telemetry-free corner-notation JSON plus two visualisations, using `tracks_config.json` as source of truth.

### 17.2 Non-goals

- Merging telemetry.
- Legacy DuckDB-staging CSVs.
- Choosing racing line.
- Per-corner driver fitting (v2).

### 17.3 CLI shape

```
python analysis/corner_analysis.py <track_csv> [--config tracks_config.json] [--no-plot] [--no-json]
```

### 17.4 Output schema — `<layout_stem>_corners.json`

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

Field semantics as before — `track`, `layout`, `total_length_m`, `config`, `corners[].type` classification, `direction` from signed curvature, `length_m`, `ai_*_speed_kmh` over corner span.

### 17.5 Algorithm

1. Load CSV (radius_m, speed_kmh, x, z).
2. Load `tracks_config.json`.
3. Smooth `radius_m` with 25-sample moving average.
4. Mark contiguous spans where `smooth_radius < straight_threshold_m`.
5. Merge adjacent candidates.
6. Compute `min_radius_m`, `avg_radius_m`, classify, infer direction.
7. Emit JSON unless `--no-json`.
8. Render two PNGs unless `--no-plot`.
9. Print stdout summary table.

### 17.6 Module layout

`analysis/corner_analysis.py` — single file ~350 lines.

### 17.7 Visualisations (kept from existing script)

`<csv_stem>_corner_map.png`, `<csv_stem>_speed_vs_position.png`. Legacy DuckDB-staging CSVs dropped.

### 17.8 Decisions baked in

1. Corner notation is JSON, not CSV.
2. One JSON per layout, alongside CSV.
3. `tracks_config.json` is source of truth.
4. Telemetry-free.
5. Straight spans not emitted as corner records in v1.
6. 500 m STRAIGHT_THRESHOLD_M default, config-overridable.
7. Legacy exports dropped.

### 17.9 Acceptance criteria (corner analysis)

See §11.13, §11.14, §11.15.

### 17.10 Open questions / v2 candidates

Include straight spans; per-track corner naming; confidence score per corner; auto-discover all layouts.

---

## 18. MF4 telemetry output (v2, PLANNED — not implemented)

### 18.1 Status

**PLANNED for v2.0. NOT implemented in v1.** v1 ships CSV-only (§14); MF4 is purely additive.

### 18.2 Goal & motivation

Emit sim telemetry as ASAM MDF v4 (`.mf4`) alongside CSV. Two reasons: bridge until time-series DB lands; tool interoperability (asammdf, MATLAB, Vector CANape, ETAS INCA).

### 18.3 Non-goals (v2.0)

- Replacing CSV.
- CAN-bus signals.
- Multi-source / multi-rate channels.
- Compressed / encrypted MF4.
- Human-realistic input traces.
- Read-side MF4 ingest in v2.0 (deferred to v2.1).

### 18.4 Decisions baked in

1. Format: ASAM MDF v4 (`.mf4`).
2. Library: `asammdf` (PyPI, MIT). Optional dep behind `--mf4` flag.
3. Scope: emit alongside CSV.
4. Output path: `<track_dir>/<track_stem>__<driver_name>_sim_telemetry.mf4`.
5. **Channel set (v2.0 minimum):** master `time` + `speed_ms`, `speed_kmh`, `distance_m`, `gas`, `brake`, `normalized_position`. Optional extras: `lat_g`, `long_g`, `rpm`, `gear`.
6. **CLI surface change:** `--mf4` (default off), `--mf4-dt-ms` optional. `--no-telemetry --mf4` is an error.
7. Read-side (v2.1): extension-dispatched parser in `telemetry.read_ac_log`.
8. **Metadata block:** source car/track/driver, generator name + git sha, timestamp, CSV cross-reference.
9. **Acceptance criteria (v2.0):** MF4 file produced next to CSV; channel/sample counts match; `asammdf` missing → exit code 2 with install hint; CSV emission unchanged when flag omitted; `--no-telemetry --mf4` rejected.
10. Non-goals re-stated: no CAN, no multi-rate, no compression, no read-side in v2.0.

### 18.5 Module placement

- **New file:** `src/lap_estimator/sim_telemetry_mf4.py` with `write_synthetic_mf4(...)`. Lazy `asammdf` import.
- `sim_telemetry.py` stays unchanged.
- `lap.py` gains two flags + one-line dispatch.

### 18.6 Read-side (v2.1, deferred)

`telemetry.read_ac_log(path)` dispatches by extension: `.csv` → existing parser; `.mf4` → `asammdf`-backed reader mapping channels back to AC-schema names.

### 18.7 Open questions

`asammdf` as hard dep vs optional; metadata convention; channel naming convention; file extension `.mf4` vs `.mdf` vs `.dat`.

### 18.8 References

ASAM MDF v4 spec; `asammdf` library; §14; §7.7.

---

## 19. Backlog — complex physics & tyre damage (research, v3)

### 19.1 Status

**BACKLOG / research.** Not scheduled. v1 stays point-mass; v2 adds MF4 + tyre-state-as-grip-modifier; v3 is the slip-based rebuild.

### 19.2 Item A — Slip-based tyre + drift / oversteer / understeer dynamics

#### 19.2.1 Motivation

Current sim is point-mass; cannot represent understeer, oversteer/drift, friction-ellipse, weight transfer transients.

#### 19.2.2 Recommended approach — Pacejka "Magic Formula"

MF5.2 or MF6.x. Inputs `Fz, α, κ, camber`; outputs `Fy, Fx, Mz`; combined slip via friction ellipse.

#### 19.2.3 Architectural implication — distance-stepped sim cannot host this

v3 sim is **time-domain ODE-integrated vehicle model**. State vector ~10 (body + per-wheel ω). RK4 at 1–5 ms. Driver becomes a control loop (preview + PID + slip-target). Sim lives alongside point-mass.

#### 19.2.4 AC signals that make this feasible

From `sensor_dictionary_merged.json`:
- Body-frame kinematics: `velocity_*`, `localVelocity_*`, `localAngularVel_*`, `accG_*`, `heading/pitch/roll`.
- Driver inputs: `steerAngle, gas, brake, clutch, gear`.
- Per-wheel: `wheelSlip*`, `wheelLoad*`, `wheelAngularSpeed*`, `suspensionTravel*`, `camberRAD*`, `tyreContactPoint*`, `tyreContactNormal*`, `tyreContactHeading*`.

Enough to **derive ground-truth (α, κ, Fz) per wheel** from real AC laps. Fit pipeline: log → compute per-wheel (α, κ, Fz, Fx, Fy) → fit MF coefficients.

#### 19.2.5 Pacejka coefficient sourcing

`tyres.ini` uses a simplified internal model — not directly Pacejka-compatible. Relevant fields: `DY0/DY1, DX0/DX1, SPEED_SENSITIVITY, FRICTION_LIMIT_ANGLE, XMU, FALLOFF_LEVEL/SPEED, LS_EXPY/LS_EXPX, DY_REF/DX_REF/FZ0, RELAXATION_LENGTH, CAMBER_GAIN, DCAMBER_0/1, FLEX*, PRESSURE_FLEX_GAIN, BRAKE_DX_MOD, CX_MULT`. Strategy: fit MF empirically to AC behaviour from test manoeuvres; bootstrap from `tyres.ini` for initial guess.

#### 19.2.6 Driver model implications — v3 schema

v1 driver JSON meaningless for slip-based sim. v3 driver needs: preview distance, lateral PID gains, slip-target setpoint (the "skill" knob), brake-release rate, throttle-application rate, reaction-time delay, mistake/consistency model. New `drivers_v3/` schema. `fit_driver.py --model {point-mass, slip}` flag.

**(v1.2 hook)** `profile.dynamic.steering_aggression_deg_per_s` measured in v1.2 becomes a direct seed for the v3 driver's corner-entry yaw-input model. v3 spec should pick this up rather than re-measuring.

#### 19.2.7 Effort estimate

~5–6 weeks focused. Vehicle dynamics ~1 wk; tyre model ~1 wk + coefficient-fit pass; driver loop ~1 wk; integration ~2–3 days; validation ~1 wk.

#### 19.2.8 Risks and open questions

ODE stability at low speed; Pacejka fitting non-trivial; validation must compare distributions (G-G, yaw-rate, slip-angle), not just lap time; computational cost (numba JIT preferred); re-fit existing v1 drivers vs start fresh; diff modelling (v3.1); surface variation (v3.1).

### 19.3 Item B — Tyre damage from type, temperature, wear, and pressure

#### 19.3.1 Motivation

Current sim uses fresh-tyre grip every lap. Reality evolves with compound, temperature, wear, pressure.

#### 19.3.2 Recommended approach — per-wheel state, multiplicative grip modifier

Per-wheel state: `T_core, wear_km, P`. `grip_mult(wheel) = thermal_curve(T_core) * wear_curve(wear_km) * pressure_curve(P)`. All three curves already in AC's `tyres.ini`.

#### 19.3.3 AC data we already have

- **Thermal:** `tcurve_*.lut`, `FRICTION_K, ROLLING_K, CORE_TRANSFER, COOL_FACTOR`.
- **Wear:** `*_front.lut`, `*_rear.lut`, `VIRTUALKM.USE_LOAD`.
- **Pressure:** `PRESSURE_STATIC, PRESSURE_IDEAL, PRESSURE_D_GAIN, PRESSURE_SPRING_GAIN, PRESSURE_FLEX_GAIN, PRESSURE_RR_GAIN`.
- **Compounds:** `[FRONT], [FRONT_1], [FRONT_2]`, `[COMPOUND_DEFAULT].INDEX`.
- **Telemetry:** `tyreTempIFL/MFL/OFL`, `tyreTempFL`, `tyreWearFL`, `wheelsPressureFL`, `tyreDirtyLevelFL`, `tyreCompound`, `aidTireRate`.

#### 19.3.4 Compatibility — fits inside the existing point-mass sim

Item B does **not** require Item A. Extend `Car` with optional per-wheel state; integrate `T_core, wear_km, P` each `ds`; apply `grip_mult` from LUTs. Effort: ~1 week of evening work.

#### 19.3.5 Driver-fit implications

`skill_pct` depends on tyre state. v2 mitigations: auto-detect tyre state from telemetry; `source.tyre_state` block in driver JSON.

#### 19.3.6 Open questions (Item B)

Multi-lap stint sim; cold-start initial T_core; flat-spotted/blistered/grained (v3); `aidTireRate` scaling; sessions vs laps.

### 19.4 Recommended sequencing

- **v1 (this branch):** point-mass + skill_pct + AC-schema CSV + validation + reorg.
- **v2 (weeks of evening work, ships independently):** §18 MF4 output; §19.3 tyre-state model in point-mass; §19.3.5 telemetry-driven tyre-state detection; v2.1 candidate `stint.py`.
- **v3 (multi-week project):** §19.2 slip-based ODE sim with Pacejka; v3.1 candidates diff, banking, surface grip, blistering.

### 19.5 References

Pacejka textbook; ASAM-XIL; Wikipedia overview; AC `sensor_dictionary_merged.json`; `cars_csv/bmw_1m/tyres.ini`; §18.

---

## 20. Two-lap "tiled" simulation (addendum, v1.1)

### 20.1 Goal

Every `lap.py` invocation emits two consecutive laps: lap 1 from rest, lap 2 from lap-1 end-speed. Loop-closure (§14.9) and validation (§15) key off lap 2.

### 20.2 Non-goals

- Multi-lap (>2).
- Fuel burn, tyre wear, brake fade across laps.
- Driver learning per-lap.
- Per-lap weather.

### 20.3 Decisions baked in

1. Two laps, always (default-on).
2. Lap 1 from rest; lap 2 from lap-1 end speed.
3. Tile segment list twice; 3-pass simulator unchanged.
4. `SimResult.lap_id` per-point int array.
5. Single combined CSV with `lap` column.
6. Monotonic timestamps across boundary.
7. `distanceTraveled` / `normalizedCarPosition` reset at boundary.
8. MC applies to lap 2 only.
9. Stdout reports both lap times.
10. `--validate-against` targets sim lap 2.
11. Loop-closure picks lap 2 by default.

### 20.4 CLI surface (additions to `lap.py`)

```
[--single-lap]
```

### 20.5 Stdout report (extended from §7.6)

Two-lap (default):
```
Lap 1 (standing): 1:48.612
Lap 2 (flying):   1:46.231 ± 0.061 (N=20)
```

`--single-lap`:
```
Lap Time: 1:48.612
```

### 20.6 Algorithm

1. Build per-point segment list.
2. Tile: concatenate with itself.
3. Run 3-pass simulator on tiled grid (lap 2's forward pass starts from lap-1 final speed).
4. Split into `lap_id` based on `distance_m < total_length_m`.
5. For telemetry: build single time grid, time-resample, derive gas/brake per §14.3, emit one CSV with `lap` column.
6. For MC: only lap-2 contributes to mean/std.

### 20.7 Acceptance criteria

See §11.18. Headline: `lap` column has `{1, 2}`; lap 2 ≤ lap 1 + `consistency_sigma`; `timestamp_ms` monotonic; `distanceTraveled` resets at boundary; `--single-lap` reduces to lap 1.

### 20.8 Module touchpoints

`simulator.py`, `sim_telemetry.py`, `report.py`, `validate.py`, `driver_fit.py`, `lap.py`, `fit_driver.py`.

### 20.9 Open questions / v2 candidates

Suppress lap 1 from CSV; multi-lap (>2) stint; lap-1 standing-start variance.

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
18. **(v1.1-E) Multi-lap fit mandatory (≥2 laps). Sample pooling, not skill averaging. Unfinished laps accepted. Out-lap trim retained but lap 1 no longer excluded by rule.** `fit_driver.py` rejects single-lap input at parse time with the error "fit requires ≥2 laps; see §13". CLI shape is variadic positional (`<car> <track> <out> <lap1.csv> <lap2.csv> [<lap3.csv> ...]`) plus a `--laps-glob` shorthand. Algorithm pools all cornering-sample grip-utilisation values across laps into one array; `skill_pct` = 85th percentile of the pool; `consistency_sigma_util` = stdev of the pool (mapped to seconds via the existing `/0.03` heuristic). Pooling is statistically more robust than averaging per-lap percentiles — sample weight is proportional to data quantity. Unfinished laps (NCP span < 0.9 post-trim) still contribute cornering samples; only their real lap time is excluded from the finished-lap mean. The existing out-lap trim (drop pre-start-line portion via NCP-wrap detection) is retained; the trimmed remainder is included in the pool. Output JSON's `source` block grows: `telemetry_csvs[]`, `n_laps`, `n_finished_laps`, `real_lap_times_s[]`, `pooled_sample_count`; `real_lap_time_s` is kept as the mean of finished-lap times for backward compatibility. Validation prints `Real laps used / Mean real lap time / Sim lap 2 / Delta vs mean real / Verdict`. (§13.1, §13.3, §13.5, §11.20, §11.21)
19. **(v1.2-F) Driver-profile dynamic signals (tau, taper, ramp, pedal-press-rate, steering-aggression) now measured from telemetry, not hand-defaulted.** `fit_driver.py` runs `profile_dynamics.measure_dynamics(merged_frames)` after the cornering-sample pool step and writes the results under a new `profile.dynamic.*` block in the driver JSON. Hand-defaults (0.12 / 30.0 / 40.0) remain as fallbacks when a measurement cannot be extracted (too few clean edges / no `steerAngle` channel). Each measured field carries a `measured` boolean so consumers can tell at a glance which path was taken. `pedal_press_rate_per_s` and `steering_aggression_deg_per_s` are recorded as statistics only; the v1.2 simulator does not consume them (v1.3 hook). Top-level `driver_tau_s` / `trail_brake_m` / `throttle_ramp_m` continue to exist for backward compatibility and mirror the profile values. (§7.2, §13.5 step 5b, §13.11, §11.22)
20. **(v1.2-G) Lap-selection rule: 5–10 newest, fallback to all if fewer than 5, hard minimum 2.** When `fit_driver.py` is handed more than 10 candidate laps (positional or `--laps-glob`), it keeps only the 10 newest by file mtime. The cap is configurable via `--newest N` (default 10). Fewer than 10 candidates → use all. Fewer than 2 → existing hard error fires. The pre-fit summary prints the selection (`12 candidates → kept 10`); the JSON's `source.lap_selection` block records the rule, candidate count, selected count, and selected source paths. (§13.12, §11.25)
21. **(v1.2-H) Default `--telemetry-dt-ms` raised from 100 to 10.** Sim telemetry CSV emitted at 100 Hz by default (was 10 Hz). 10× larger files; accepted for the dramatically improved driver-lag smoothing fidelity (α = 0.010 / (0.120 + 0.010) ≈ 0.077 vs 0.455 at 100 ms). Old behaviour reproducible with `--telemetry-dt-ms 100`. README and `lap.py --help` updated. (§14.3 Layer 3, §14.4 item 2, §14.5, §11.23)
