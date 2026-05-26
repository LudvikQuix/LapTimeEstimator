# Lap Simulation: CSV Track + Driver Config

**Status:** Draft (v1 + telemetry-fitting + sim-telemetry-emission + cross-track-validation + project-reorg addendum; v1.1 in-flight: driver JSON migration + trail-brake/throttle-ramp heuristic + two-lap tiled sim + multi-lap-mandatory fit — §13/§14/§20; v1.2 in-flight: driver-profile dynamic-signal measurement from telemetry + 10 ms sim cadence default — §13/§14; v1.2.1: IIR driver-lag low-pass removed from sim emission — §14.3 / Decisions item 22; **v2 in-flight: per-wheel tyre state + multi-lap stint sim + inverse PSI solver — §21 / Decisions item 23; v2 compound-aware tyres.ini parsing — §21.11 / Decisions item 24; v1.3 in-flight: asymmetric pressure model — grip-only penalty above IDEAL, drag-only penalty below IDEAL — §21.3 / Decisions item 25; v2.0.1 in-flight: stint-lap velocity continuity — every lap N≥2 starts at lap N-1's v_end — §21.4 / Decisions item 26**; v2 MF4 telemetry output PLANNED — §18; v3 slip-based physics BACKLOG — §19)
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

**v1.1 in-flight (2026-05-13):** five bundled changes layered on the shipped v1 plumbing — (A) driver config format moves from YAML to JSON (no fallback); (B) ~~driver-lag 1st-order low-pass on emitted gas/brake~~ **REMOVED in v1.2.1 — see Decisions item 22**; (C) trail-brake + throttle-ramp corner-shape heuristic applied to gas/brake; (D) two-lap "tiled" simulation always emitted (lap 1 standing, lap 2 flying), with a `lap` column on telemetry/trace CSVs and Monte-Carlo / validation / loop-closure all keying off lap 2; (E) **driver-fit requires ≥2 laps and pools cornering samples across them** — single-lap fits are hard-errors (§13). See §13, §14.3, §20.

**v1.2 in-flight (2026-05-13):** three bundled extensions on top of v1.1 — (F) `fit_driver.py` now **measures the dynamic driver-profile signals from telemetry** instead of taking the hand-defaults: `driver_tau_s`, `trail_brake_m`, `throttle_ramp_m` plus two new statistics `pedal_press_rate_per_s` and `steering_aggression_deg_per_s`. All five live under a new `profile.dynamic` sub-object in `drivers/<name>.json`. Hand-defaults stay as fallbacks when measurements are not extractable. `driver_tau_s` is **measured but no longer consumed** by the sim (v1.2.1 — see §14.3 and Decisions item 22). (G) **Lap-selection rule** for the fitter: take the **5–10 newest** laps for that driver (fallback to all if fewer than 5, hard minimum of 2). Newness comes from input ordering or the new `--newest N` flag combined with `--laps-glob`. (H) **Default `--telemetry-dt-ms` raised from 100 to 10** for sim emission — 10× larger files, accepted for cleaner edge geometry in downstream consumers. See §13.11, §13.12, §14.

**v1.2.1 in-flight (2026-05-14):** the §14.3 emission pipeline drops the IIR driver-lag low-pass (Layer 3). Brake points in racing are spatial cues, not stimulus-driven reactions; the only physical lag (motor execution + pedal mechanics, ~20–50 ms) is sub-sample at 50 Hz and well below the v1.2 10 ms cadence. `driver_tau_s` is retained in the schema and in `profile_dynamics.py` measurements as a **driver statistic only** — useful for characterising how fast a driver presses pedals — but not consumed by the simulator. See Decisions item 22.

**v2 in-flight (2026-05-14):** the simulator gains **per-wheel tyre state (temperature, wear, pressure) evolving across a multi-lap stint**, plus an **inverse-solver CLI mode** that recommends cold-PSI setup values to hit a target wear at a target lap. Architecturally this is **option 2.5** — the existing 3-pass `simulator.py` and single-scalar grip envelope are retained; per-wheel state is computed offline between laps and fed back as a scaled `mu_x` / `mu_y` for the next lap's solver pass. Per-wheel forces, yaw dynamics, and Pacejka stay in v3 (§19 unchanged). v2 is sized as ~1–2 weeks of evening work; v3 is a multi-week rebuild. Full algorithm, schema additions, CLI shapes, calibration approach, and acceptance criteria in §21. See Decisions item 23.

**v2 compound-aware parsing (2026-05-14):** `car.py` now parses **all compounds** defined in `tyres.ini` (un-suffixed sections = compound 0, `_1` suffix = compound 1, `_2` = compound 2, …) instead of only the un-suffixed Compound 0. Each compound carries its own `NAME`, `SHORT_NAME`, per-axle `PRESSURE_IDEAL`, `PRESSURE_STATIC`, `PRESSURE_D_GAIN`, `WEAR_CURVE` LUT, `DY0/DY1/DX0/DX1/DY_REF/DX_REF`, and `[THERMAL_*]` `PERFORMANCE_CURVE`. Setup JSON's `compound` field selects which compound the sim uses (case-insensitive match against `NAME` or `SHORT_NAME`); a new `--compound <name>` CLI flag on `lap.py` overrides that; and `fit_driver.py` auto-selects from the telemetry's `tyreCompound` column. `f_pressure_grip`, `f_pressure_drag`, `f_temp`, and `f_wear` all become per-compound functions; default cold pressures fall back to the active compound's `PRESSURE_STATIC` (not `PRESSURE_IDEAL`). Motivation: Tomas's real lap on Semislicks at 26 psi cold was being scored against Street's `PRESSURE_IDEAL=42`, putting `f_pressure` at the 0.30 grip floor and tanking the sim. See §21.11 and Decisions item 24.

**v1.3 pressure-model asymmetry (2026-05-14):** the previous v2 `f_pressure(p) = 1 - PRESSURE_D_GAIN · (p - PRESSURE_IDEAL)²` was a symmetric quadratic that penalised grip on *both* sides of `PRESSURE_IDEAL`. Physically wrong on the low-PSI side: under-pressure increases the contact patch (more or equal mechanical grip) but increases sidewall flex + rolling resistance (more drag). The model was double-counting the low-PSI penalty in the grip term where it should have been in the drag term. v1.3 splits the single function into two: `f_pressure_grip(p)` (one-sided quadratic, penalises *over*-pressure only; `1.0` below IDEAL) and `f_pressure_drag(p)` (linear under-pressure penalty + small over-pressure benefit, modulating aero drag + rolling resistance in the straights). `combined_grip_envelope` now returns `(mu_x_scale, mu_y_scale, drag_scale)`; `drag_scale` threads into the 3-pass solver's drag-force computation. Two new hand-default constants `k_drag = 0.5` and `k_drag_reduction = 0.1` live on the active `Compound`. Calibration from telemetry is deferred to v1.4. See §21.3 and Decisions item 25.

**v2.0.1 stint-lap velocity continuity (2026-05-15):** the v2 multi-lap stint loop in `simulate_stint` was calling `simulate(..., two_lap=False)` once per lap, which means **every lap was a standing start at v=0**. The v1.1 two-lap tiled fast path (`simulate(..., two_lap=True)`) does tile the segments and produces a flying lap 2, but the N≥3 lap loop never inherited that property. Fix: `simulate(...)` gains a kwarg-only `v_initial: float = 0.0`; `simulate_stint`'s lap loop carries `v_prev_end = lap_result.speeds[-1]` across laps and passes it as `v_initial` for laps 2..N. Lap 1 always passes `v_initial=0.0`. The `n_laps == 2 && !measured` two-lap-tile fast path is unchanged (already produces continuous velocity via a single solver pass). Universal rule: lap 1 from rest, lap N from end-of-(N-1). See §21.4 and Decisions item 26.

## 2. Goals

- Accept a **car data directory**, a **track CSV path**, and a **driver JSON path** as the three positional inputs to the sim CLI (`lap.py`).
- Consume the rich per-point track CSV format (ideal line preferred, centerline fallback) directly — no intermediate JSON conversion required.
- Apply a simple **driver model** (`skill_pct`, optional `consistency_sigma`, `trail_brake_m`, `throttle_ramp_m`) that scales the car's effective grip uniformly and shapes the emitted gas/brake trace. `driver_tau_s` is retained in the schema as a measured statistic but is **not consumed** by the sim (v1.2.1).
- Emit a per-point **trace CSV** and a **sim-vs-AI comparison PNG** next to the track input so the user can sanity-check results in Quix Cloud / locally.
- Preserve current stdout lap-time report format (extended for two-lap output — see §20; extended further for multi-lap stint output — see §21.5).
- Keep modules small (~500-line soft ceiling).
- Provide a `fit_driver.py` CLI that ingests **two or more** AC telemetry CSVs (variadic positional, plus a `--laps-glob` shortcut) and emits a single driver JSON calibrated to pooled cornering samples from all laps. Single-lap fits are rejected with a clear error — see §13. **(v2)** When the input telemetry contains per-wheel state channels (§21.6), also calibrate the four tyre-thermal/wear knobs (`k_friction`, `h`, `C_thermal`, `k_wear`) into a `tyre_calibration` block on the driver JSON.
- Emit a **synthetic AC-schema telemetry CSV** alongside the trace output by default (configurable cadence, **default 10 ms in v1.2** — see §14), so sim runs and real laps are interchangeable downstream. **(v2)** The synthetic CSV gains 12 per-wheel state columns when multi-lap stint mode is engaged — see §21.4.
- Ship cross-track validation as a **flag on `lap.py`** (`--validate-against <real_telemetry.csv>`), not as a third CLI: when present, `lap.py` additionally loads the real lap, prints a delta report, writes an overlay PNG and a per-bin delta CSV, and emits a GOOD/LOOSE/BAD verdict (§15). Two simulator CLIs total: `fit_driver.py` and `lap.py`.
- **(v1.2)** Measure dynamic driver-profile signals (`driver_tau_s`, `trail_brake_m`, `throttle_ramp_m`, plus the new `pedal_press_rate_per_s` and `steering_aggression_deg_per_s`) from telemetry inside `fit_driver.py`, replacing hand-defaults. See §13.11 / §13.12. **(v1.2.1)** `driver_tau_s` is measured but recorded as a driver statistic — not consumed by the simulator.
- **(v1.2)** Default lap selection is the **5–10 newest laps**; fall back to all available if fewer than 5; hard minimum 2 (existing §13 guard). See §13.12.
- **(v2)** Run **multi-lap stint simulation** (`--laps N`, N ∈ [1, 50], default 2 for back-compat) with per-wheel state (temperature, wear, pressure) evolving end-of-lap → start-of-next-lap. Per-lap stdout summary + 12 new telemetry columns. See §21.
- **(v2)** Provide an **inverse-PSI solver** (`lap.py --solve-pressure-for-wear <pct> --at-lap <N>` or `setup-recommend` sub-command) that bisects cold-PSI per wheel to hit a target wear at a target lap. See §21.5.
- **(v2)** Introduce a `setups/` directory of cold-PSI / ambient setup configs. See §21.7.
- **(v2)** Parse **all compounds** from `tyres.ini` into `car.compounds: list[Compound]` and select active compound by setup-JSON `compound` field, `--compound` CLI flag, or telemetry's `tyreCompound`. `f_pressure_grip`/`f_pressure_drag`/`f_temp`/`f_wear` are per-compound. See §21.11.
- **(v1.3)** Replace the symmetric pressure-grip quadratic with an **asymmetric pressure model**: grip penalty active only above `PRESSURE_IDEAL` (over-pressure → smaller contact patch); drag penalty active only below `PRESSURE_IDEAL` (under-pressure → rolling resistance + sidewall flex). `combined_grip_envelope` returns three scalars `(mu_x_scale, mu_y_scale, drag_scale)`. See §21.3.
- **(v2.0.1)** Make stint laps velocity-continuous: lap 1 starts at v=0; lap N ≥ 2 starts at lap N-1's end-of-lap speed. Threaded via a new `v_initial` kwarg on `simulate(...)`. See §21.4 and Decisions item 26.
- **(reorg)** Ship a **preparation pipeline** (§16).
- **(reorg)** Ship a **corner analysis tool** (§17).
- **(reorg)** Lock the folder convention (§6.9).

## 3. Non-goals

- Driver line selection / line deviation (always uses the ideal line if present).
- Separate brake-aggression vs throttle-aggression parameters (v2 hook in §13.9).
- Reaction-time / lift-and-coast / fuel-saving driver behaviours. **(v1.2.1) Note:** classical stimulus-driven reaction-time models are explicitly out of scope. Racing brake points are spatial cues; the only physical lag (~20–50 ms motor execution) is sub-sample at the v1.2 10 ms cadence and not worth modelling.
- ~~Tyre thermal model, ABS, traction control, weight transfer beyond what `car.py` already does.~~ **(v2 amendment)** A coarse per-wheel tyre thermal + wear + pressure model lands in v2 (§21). ABS / traction control / true weight-transfer transients remain out of scope.
- Hooking into `RequirementsForAbnormalityAnalysis.txt` checks — separate feature.
- ~~Multi-lap / fuel-burn / tyre-wear simulation.~~ **(v2 amendment)** Multi-lap stint simulation with per-wheel tyre wear lands in v2 (§21). Fuel burn is still out of scope. The v1.1 two-lap tiled mode (§20) is preserved as the default for `--laps 2`.
- Web UI. CLI only.
- Per-corner skill profile in v1 (v2 hook in §13.6).
- Fitting tyre, aero, or engine parameters from telemetry — only driver-skill scalars are fit. **(v2 amendment)** The four tyre-thermal/wear calibration knobs (`k_friction`, `h`, `C_thermal`, `k_wear`) are fit when measured per-wheel state is available; aero and engine parameters remain ground truth.
- Per-track familiarity / learning-curve modelling in v1.
- No database, object store, remote artefact server, or networked service.
- **(reorg)** The corner-analysis tool does **not** merge telemetry.
- **(reorg)** `prep/prep_track.py` does **not** infer corner notation.
- **MF4 output is v2 — see §18; v1 only emits CSV.**
- **Slip-based physics, drift / oversteer / understeer dynamics, per-wheel forces, yaw dynamics, Pacejka, slip-based control-loop drivers are v3 backlog — see §19.** v2 keeps the single-grip-envelope architecture; per-wheel state modulates `mu_x` / `mu_y` only, not per-wheel forces (§21.3).
- **(v2)** Tyre puncture, catastrophic failure modes, marble pickup, track-evolution grip, heat soak across pit stops, multi-stint sessions. All out of scope. v2 is one stint, no pit stop.
- **(v1.2)** Consuming `pedal_press_rate_per_s` / `steering_aggression_deg_per_s` inside the simulator — they are recorded as profile statistics only for future v1.3 work.
- **(v1.2.1)** Consuming `driver_tau_s` inside the simulator. It is recorded as a statistic in the driver JSON for back-compat and future use (e.g. a v1.3 slew-rate-limited driver model), but the §14.3 emission pipeline does not apply any low-pass smoothing based on it. The currently-consumed dynamic fields are `trail_brake_m` and `throttle_ramp_m` only.
- **(v2)** Per-compound calibration of the four tyre-thermal/wear knobs (`k_friction`, `h`, `C_thermal`, `k_wear`). They are physical heating/wear constants in v2 and apply across compounds. Per-compound knob tables are a v1.3 candidate. See §21.11.
- **(v2)** Compound mid-stint switching (pit stop with fresh compound change). Out of scope for v2; v1.3 candidate. See §21.11.
- **(v1.3)** Calibrating `k_drag` and `k_drag_reduction` from telemetry. They are hand-defaults in v1.3 (`k_drag = 0.5`, `k_drag_reduction = 0.1`). Telemetry-driven fit (lap-time-vs-pressure sweep against measured AC data) is a v1.4 candidate. See §21.3 and §21.10.
- **(v1.3)** Modelling an under-pressure *grip bonus* (larger contact patch). Real but small; v1.3 keeps `f_pressure_grip(p < IDEAL) = 1.0` for simplicity. v1.4 candidate. See §21.3.
- **(v2.0.1)** Modelling per-lap pit-lane deceleration or starting-grid spawn semantics. The "v_initial" threading is purely about continuity between consecutive flying laps within the same stint — it does not represent any real-world inter-lap event (no pit stop, no SC, no formation lap). See §21.4 and Decisions item 26.

## 4. User stories / scenarios

1. **Run a single lap.** User has decrypted car data in `cars_csv/bmw_1m/data/`, a track CSV, and `drivers/pro.json`. They run `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_gp_a_ideal_line.csv drivers/pro.json` and see the lap-time report on stdout (both laps — §20), plus a trace CSV and PNG written next to the track CSV.
2. **Compare two drivers, same car/track.** User runs the command twice with `drivers/pro.json` and `drivers/amateur.json`.
3. **Use a centerline-only track.** User points at a `layout_*.csv` with no `*_ideal_line.csv` sibling.
4. **Consistency study.** User sets `consistency_sigma: 0.3` → sim performs N Monte-Carlo runs on **lap 2 only**.
5. **Legacy invocation (built-in track).** User runs `python lap.py cars_csv/bmw_1m monza drivers/pro.json`.
6. **Fit a driver from multi-lap telemetry.** User has six AC laps at `samples/aclog/Tomas_Lap{1..6}.csv`. They run `python fit_driver.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json samples/aclog/Tomas_Lap1.csv samples/aclog/Tomas_Lap2.csv samples/aclog/Tomas_Lap3.csv samples/aclog/Tomas_Lap4.csv samples/aclog/Tomas_Lap5.csv samples/aclog/Tomas_Lap6.csv` (or `--laps-glob "samples/aclog/Tomas_Lap*.csv"`).
7. **Loop-closure check.** Sim → synthetic telemetry CSV (two laps) → fed back into `fit_driver.py`.
8. **Cross-track prediction and validation (headline workflow).** Fit on Track A → predict on Track B → drive Track B in AC → validate.
9. **(reorg) Prepare a new car / track from raw AC content.**
10. **(reorg) Run corner analysis on a prepared track.**
11. **(v1.1) Single-lap mode for fast sweeps.** User runs `python lap.py ... drivers/pro.json --single-lap`.
12. **(v1.2) Pick newest laps automatically.** User has 30 telemetry CSVs and runs with `--laps-glob ... --newest 10`.
13. **(v1.2) Four-layout sweep across ks_nurburgring.**
14. **(v2) "How long until 80%?".** User runs `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json --laps 20` and reads the per-lap stdout block to find the lap at which the limiting wheel crosses 80% wear.
15. **(v2) "Give me a setup for 50% at lap 12".** User runs `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json --solve-pressure-for-wear 0.50 --at-lap 12` and gets a recommended four-PSI setup back, plus a verification table showing predicted wear/temp/pressure at each lap when re-run forward with that setup.
16. **(v2) Uniform-pressure setup.** Same as 15 but with `--uniform-pressure` so all four wheels get the same recommended cold PSI (simpler answer the user can dial into AC's tyre app).
17. **(v2) Calibrate thermal/wear knobs from lake telemetry.** User runs `fit_driver.py` against Tomas's six laps where the lake CSVs include `tyreTempFL/...`, `tyreWearFL/...`, `wheelsPressureFL/...`. The fitter fits the four calibration knobs alongside `skill_pct` and writes them under `tyre_calibration` in the driver JSON. Without per-wheel state in the telemetry, the fitter writes default knobs and a `measured: false` flag.
18. **(v2) Run sim on Semislicks compound.** Tomas's real lap was on Semislicks (`tyreCompound="Semislicks (SM)"`, 26 psi cold). User runs `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json --compound Semislicks --pressure FL=26,FR=26,RL=26,RR=26`. The sim picks the Semislicks `PRESSURE_IDEAL=33/34`, `PRESSURE_D_GAIN=0.0045`, `semislicks_{front,rear}.lut` wear curves, and `[THERMAL_FRONT_1]`/`[THERMAL_REAR_1]` thermal curves — so 26 psi is now within the Semislicks operating envelope rather than collapsing `f_pressure` to the floor.
19. **(v2) Fit driver from compound-tagged telemetry.** Same as 6, but the telemetry rows carry `tyreCompound="Semislicks (SM)"`. `fit_driver.py` auto-selects the Semislicks compound for the calibration and the `lat_g_max` envelope; written into the driver JSON's `source.compound` for traceability.
20. **(v1.3) Low-PSI sanity.** With calibrated Tomas + Semislicks + ambient 26 °C, a 3-lap sim at 26 psi cold should produce lap times within ±2 s of the 33 psi (IDEAL) baseline — *not* the 6 s deficit the symmetric v2 model produced. Cause for the residual gap is rolling-resistance drag, not grip collapse.
21. **(v2.0.1) Flying-lap pace from lap 2 onward.** User runs `python lap.py ... --laps 3` and observes that lap 2 and lap 3 are both substantially faster than lap 1 (which started from a standing v=0). Pre-v2.0.1, lap 2 and lap 3 also stood up at v=0 — masking the actual stint pace and making the per-lap stdout block look uniformly slow. Post-v2.0.1, lap 2..N inherit lap (N-1)'s end-of-lap speed and look like flying laps. See §11.47 and §11.48.

## 5. Proposed design

- Add a new `driver.py` module with a `Driver` dataclass (loaded from JSON) and a small `apply_to_car(car)` helper. The dataclass also carries the three v1.1 fields and (v1.2) a `profile` block. **(v2)** Plus a `tyre_calibration` block (§21.6).
- Extend `track.py` with a `Track.from_csv(path)` classmethod.
- Adjust `simulator.simulate(...)` so it takes an optional `driver` argument and supports two-lap tiled mode (§20). **(v2)** Add a new `simulate_stint(car, track, driver, *, n_laps, setup, calibration)` wrapper that loops `simulate(...)` lap-by-lap and threads per-wheel state through (§21.4). **(v1.3)** Threads `drag_scale` from `combined_grip_envelope` into the 3-pass drag-force computation. **(v2.0.1)** `simulate(...)` gains a `v_initial: float = 0.0` kwarg; `simulate_stint` threads the previous lap's `v_end` into the next lap's `v_initial`.
- Move output artifact generation into a new `report.py` module. **(v2)** Add a per-lap stint summary helper.
- `lap.py` is the single sim CLI. **(v2)** Gains `--laps N`, `--setup <path>`, per-wheel pressure flags, `--solve-pressure-for-wear`, `--at-lap`, `--target-wheel`, `--uniform-pressure`, `--compound <name>`.
- Add a `telemetry.py` module that owns AC-log parsing and merging-with-track logic. **(v1.2)** Accepts `steerAngle` opportunistically. **(v2)** Accepts per-wheel state channels (`wheelsPressureFL/...`, `tyreTempFL/...`, `tyreWearFL/...`, `wheelLoadFL/...`, `tyreCompound`) — see §21.6.
- Add a `driver_fit.py` module. **(v2)** Adds an optional `fit_tyre_calibration(merged_frames, car) -> TyreCalibration` step. **(v2)** Detects telemetry `tyreCompound` and selects the matching `car.compounds[i]` for the fit.
- Add a `profile_dynamics.py` module (v1.2 — see §13.11).
- Add a `sim_telemetry.py` module (§14). **(v1.2.1)** Two-layer pipeline. **(v2)** Gains 12 per-wheel state columns.
- Add a `validate.py` module (§15).
- **(v2)** Add a `tyre_state.py` module that owns the per-wheel state struct, the per-segment update (slip energy attribution, thermal ODE, wear, pressure), and the scalar-grip-envelope reduction (§21.3). Single file, ~400 lines. Per-compound functions are looked up via the active `Compound`. **(v1.3)** Adds `f_pressure_grip` / `f_pressure_drag` split; `combined_grip_envelope` returns a 3-tuple.
- **(v2)** Add a `setup.py` module that loads `setups/<car>_<scenario>.json` and resolves it against `--pressure FL=... FR=...` CLI overrides + `tyres.ini` `PRESSURE_STATIC` defaults of the active compound.
- **(v2)** Add a `solve_setup.py` module that owns the inverse-PSI bisection (§21.5).
- **(v2)** Extend `car.py` to parse **all** compound sections (un-suffixed = compound 0, `_1`, `_2`, …) and store them as `car.compounds: list[Compound]` plus `car.default_compound_index`. See §21.11.
- **(reorg)** Move modules into `src/lap_estimator/`, `prep/`, `analysis/`.

## 6. Sub-features / work breakdown

### 6.1 Driver module (`src/lap_estimator/driver.py`)
- Loads driver JSON; exposes `skill_pct`, `consistency_sigma`, `driver_tau_s`, `trail_brake_m`, `throttle_ramp_m`; provides grip-scaling wrapper. **(v1.2.1)** `driver_tau_s` informational only.
- **(v1.2)** Tolerates `profile.dynamic.*`.
- **(v2)** Tolerates `tyre_calibration` block (§21.6). `Driver.load` populates the four calibration scalars onto the dataclass, with defaults when absent or `measured: false`.
- `Driver.load(path) -> Driver` via stdlib `json`.
- Owner: ArchDev.

### 6.2 Driver JSON schema + example files
- Schema in §7.2. v1.2 profile block. **(v2)** `tyre_calibration` block.
- Examples: `pro.json`, `amateur.json`.
- Owner: ArchDev.

### 6.3 Track CSV loader (`src/lap_estimator/track.py` additions)
- `from_csv(path) -> Track`, `to_points(ds)`, `to_ai_reference(ds)`, `total_length_m`.
- Owner: ArchDev.

### 6.4 Simulator changes (`src/lap_estimator/simulator.py`)
- `simulate(car, track, driver=None, ds=2.0, rng=None, two_lap=True, *, v_initial=0.0, drag_scale=1.0) -> SimResult`.
- `simulate_monte_carlo(...)`.
- **(v2)** `simulate_stint(car, track, driver, *, n_laps, setup, calibration, ds=2.0) -> StintResult` — loops `simulate(..., two_lap=False)` once per lap, after each lap calls `tyre_state.update_per_lap(...)` and rescales `mu_x` / `mu_y` for the next lap. **(v2.0.1)** The lap loop also carries `v_prev_end = lap_result.speeds[-1]` and passes it as `v_initial` into the next lap's `simulate(...)` call. Lap 1 passes `v_initial=0.0` (standing start). Implementation note: keep the existing `simulate(...)` function's default behaviour byte-compat (`v_initial=0.0` default). Single-lap regression is preserved.
- **(v1.3)** `simulate(...)` signature gains an internal `drag_scale: float = 1.0` parameter (kwarg, defaults preserve byte-compat). The 3-pass solver's drag-force computation (`F_drag = 0.5 · ρ · Cd · A · v²` plus any rolling-resistance term in `car.drag_force(v)` / equivalent) is multiplied by `drag_scale` at the natural seam — see §21.3 step "Drag plumbing". `simulate_stint` reads the third element of `combined_grip_envelope`'s return and threads it into the next lap's `simulate(..., drag_scale=...)`.
- **(v2.0.1)** `simulate(...)` gains a kwarg-only `v_initial: float = 0.0`. When `v_initial > 0`, the forward pass's first velocity sample is `v_initial` instead of the standing-start default (0). Backward pass and corner-limit pass are unchanged (they don't depend on `v_initial`). Edge case: if `v_initial >= v_corner_max(starting segment)`, the forward pass's next-step velocity-cap clamp drops it to `v_corner_max` naturally — no extra guard needed; documented for clarity. Default `v_initial=0.0` preserves byte-compat with v1.3 and earlier.
- `SimResult` unchanged. `StintResult` new dataclass — see §21.4.
- Owner: ArchDev.

### 6.5 Output artifacts (`src/lap_estimator/report.py`)
- Writes `<track_stem>_sim_trace.csv` and `<track_stem>_sim_vs_ai.png`.
- **(v2)** When `StintResult` is supplied, writes a per-lap summary CSV (`<track_stem>_stint_summary.csv`) with one row per lap: `lap, lap_time_s, wearFL, wearFR, wearRL, wearRR, tempFL_C, ..., pressureFL_psi, ...`. Stdout printer gains the per-lap block (§21.5).
- Owner: ArchDev.

### 6.6 CLI rewiring (`lap.py`)
- See §13.3 for `fit_driver.py`. `lap.py` CLI:
  ```
  python lap.py <car_data_dir> <track> <driver_json> \
                [--ds 2.0] [--all-tracks] [--single-lap] \
                [--no-plot] [--no-telemetry] [--telemetry-dt-ms 10] \
                [--validate-against <real_telemetry_csv>] [--bin-m 100] [--per-corner] \
                [--laps N] [--setup <path>] [--pressure FL=31,FR=31,RL=29,RR=29] \
                [--ambient-temp-c 25.0] [--compound <name>] \
                [--solve-pressure-for-wear <pct> --at-lap <N> \
                 [--target-wheel min|max|avg|FL|FR|RL|RR] [--uniform-pressure]]
  ```
- **(v2)** `--laps` defaults to `2` (back-compat with §20's two-lap default). `--single-lap` is shorthand for `--laps 1`. Mutually exclusive with `--single-lap` when used together (argparse error).
- **(v2)** `--solve-pressure-for-wear` triggers inverse-solver mode; suppresses normal per-lap output and prints the recommendation table instead. Requires `--at-lap`.
- **(v2)** `--compound <name>` overrides setup-JSON's `compound`. Case-insensitive match against `Compound.name` or `Compound.short_name`. Unknown → argparse error listing available compounds.
- Owner: ArchDev.

### 6.7 README / docs touch-up
- v2 sections: stint sim, inverse solver, setup files, calibration block, **compound-aware parsing**. **(v1.3)** Note the asymmetric pressure model and `drag_scale` plumbing. **(v2.0.1)** Note the stint-lap velocity-continuity rule.
- Owner: DocuGuy.

### 6.8 Package skeleton and module moves (reorg)
- Owner: ArchDev.

### 6.9 Folder convention and `.gitignore` policy (reorg)
- Tracked: `cars_csv/`, `tracks_csv/`, `drivers/*.json`, `tracks_config.json`, `samples/aclog/*.csv`, `prep/`, `analysis/`, `src/lap_estimator/`, root CLIs, `docs/`, `dev-planning/`, **(v2) `setups/*.json`**.
- Gitignored: `cars_in/*`, `tracks_in/*`, `.tmp/`.
- Owner: ArchDev.

### 6.10 Profile-dynamics module (v1.2)
- Owner: ArchDev.

### 6.11 Tyre-state module (`src/lap_estimator/tyre_state.py`) — v2 NEW
- Exposes `TyreState` (per-wheel: `temp_C`, `wear_pct`, `pressure_psi`, plus shared `cumulative_slip_energy_J`), `update_per_segment(state, segment_info, car, compound, calibration, ambient_temp_C)`, `update_per_lap(state, segment_states_iter, car, compound, calibration, ambient_temp_C)`, and `combined_grip_envelope(state, compound) -> (mu_x_scale, mu_y_scale, drag_scale)` **(v1.3: 3-tuple)**.
- Implements slip-energy attribution, thermal ODE, wear integration, pressure ideal-gas, and per-wheel-grip → scalar reduction per §21.3. **(v2)** `f_temp`, `f_wear`, `f_pressure_grip`, `f_pressure_drag` are looked up off the passed `Compound` — see §21.11. **(v1.3)** `f_pressure` is split into grip and drag halves; see §21.3.
- Single file, ~400 lines.
- Owner: ArchDev.

### 6.12 Setup module (`src/lap_estimator/setup.py`) — v2 NEW
- Exposes `Setup` dataclass (`pressures_psi: dict[str, float]`, `ambient_temp_C: float`, `compound: str | None`), `Setup.load(path)`, `Setup.from_cli(pressure_str, ambient_str, compound_str, car)`, `Setup.default_for_car(car, compound)` (reads active compound's `PRESSURE_STATIC` from `tyres.ini`).
- Resolution precedence: explicit `--pressure ...` flag > `--setup <path>` > active compound's `PRESSURE_STATIC` (per axle).
- Compound resolution precedence: `--compound` flag > setup-JSON `compound` > telemetry `tyreCompound` (when called from fitter) > `car.default_compound_index`.
- ~120 lines.
- Owner: ArchDev.

### 6.13 Inverse-solver module (`src/lap_estimator/solve_setup.py`) — v2 NEW
- Exposes `solve_pressure_for_wear(car, track, driver, *, target_wear, target_lap, target_wheel, uniform, compound, calibration, ambient_temp_C, n_laps_max) -> SolveResult`.
- Bisection per-wheel (or uniform) over `[20.0, 50.0]` psi; tolerance ±1% wear or ±0.5 psi resolution; cap at 12 iterations per wheel.
- Calls `simulate_stint(...)` per candidate set; reads `StintResult.tyre_state_history` at lap `target_lap`.
- ~250 lines.
- Owner: ArchDev.

### 6.14 Car module compound parsing (`src/lap_estimator/car.py`) — v2 EXTENSION
- Scan `tyres.ini` for compound sections: un-suffixed (`[FRONT]`, `[REAR]`, `[THERMAL_FRONT]`, `[THERMAL_REAR]`) = compound 0; `_1`, `_2`, … = additional compounds.
- For each compound build a `Compound` dataclass: `index`, `name`, `short_name`, per-axle `pressure_ideal_psi`, `pressure_static_psi`, `pressure_d_gain`, `wear_curve_lut` (front + rear), `dy0/dy1/dx0/dx1/dy_ref/dx_ref` per axle, `thermal_lut` (front + rear `PERFORMANCE_CURVE`). **(v1.3)** Plus two hand-default scalars `k_drag` (default 0.5) and `k_drag_reduction` (default 0.1) — see §21.3 / §21.11. These are *not* parsed from `tyres.ini` in v1.3 (no AC source for them); they live on `Compound` so a future v1.4 fitter can vary them per-compound. For v1.3 they are constants set at `Compound` construction.
- `Car.compounds: list[Compound]` indexed 0..N-1.
- `Car.default_compound_index: int` — honour `[COMPOUND_DEFAULT]` section's `INDEX` value if present, otherwise 0.
- `Car.find_compound(name_or_short: str) -> Compound | None` — case-insensitive match against both `name` and `short_name`. Also strip trailing parenthetical short-name from telemetry strings (e.g. `"Semislicks (SM)"` → matches `"Semislicks"`).
- Owner: ArchDev.

## 7. Data & interface contracts

### 7.1 Track CSV (input)
Unchanged. `distance_m`, `segment_length_m`, `radius_m`, `gradient_pct`, `elevation_m`, `speed_ms`.

### 7.2 Driver JSON (input) — v1.1 + v1.2 profile block + v2 tyre_calibration
```json
{
  "name": "string",
  "skill_pct": 0.95,
  "consistency_sigma": 0.0,
  "driver_tau_s": 0.12,
  "trail_brake_m": 30.0,
  "throttle_ramp_m": 40.0,
  "profile": { "dynamic": { "...": "see §13.5 step 8" } },
  "tyre_calibration": {
    "k_friction": 1.0,
    "h": 50.0,
    "C_thermal": 5000.0,
    "k_wear": 1.0e-7,
    "measured": false,
    "source": {
      "telemetry_csvs": ["..."],
      "compound": null,
      "fit_rmse_temp_C": null,
      "fit_rmse_wear_pct": null,
      "fit_rmse_pressure_psi": null,
      "fitted_at": "ISO-8601"
    }
  },
  "source": { "...": "see §13.5 step 8" }
}
```
**v2 `tyre_calibration` field semantics:**
- `k_friction` (float, default 1.0) — multiplier on slip-energy-to-heat conversion. Units: dimensionless. Bounded `[0.1, 10.0]`. **Compound-agnostic in v2** — physical heating constant; per-compound table is a v1.3 candidate.
- `h` (float, default 50.0) — Newton-cooling coefficient (W/K) per tyre. Bounded `[5.0, 500.0]`. **Compound-agnostic.**
- `C_thermal` (float, default 5000.0) — per-tyre heat capacity (J/K). Bounded `[500.0, 50000.0]`. **Compound-agnostic.**
- `k_wear` (float, default 1.0e-7) — wear rate per unit slip energy per Joule, modulated by `f_temp_penalty`. Bounded `[1e-9, 1e-4]`. **Compound-agnostic.** Per-compound wear differences come from the `WEAR_CURVE` LUT, not from `k_wear`.
- `measured` (bool, default `false`) — `true` iff the four knobs were fit against per-wheel lake telemetry (§21.6). When `false`, the values are hand-defaults.
- `source.compound` (string, optional) — the compound name used for the fit (matches telemetry's `tyreCompound`).
- `source.*` — diagnostic block populated by the fitter when `measured = true`.

Backward compat: pre-v2 driver JSONs without `tyre_calibration` load cleanly; `Driver.load` injects defaults with `measured: false`.

### 7.3–7.9 unchanged.

### 7.10 Setup JSON (input, **v2 NEW**) — `setups/<car>_<scenario>.json`
```json
{
  "car": "bmw_1m",
  "name": "default",
  "pressures_psi": {"FL": 31.0, "FR": 31.0, "RL": 29.0, "RR": 29.0},
  "ambient_temp_C": 25.0,
  "compound": "Street",
  "notes": "Default street setup; PRESSURE_STATIC from tyres.ini compound 0"
}
```
- `car` (string, required) — must match the car-data-dir basename (validation).
- `name` (string, required) — free-form scenario tag.
- `pressures_psi` (object, optional) — keys exactly `FL`, `FR`, `RL`, `RR`; values in `[20.0, 50.0]`. **When absent, the active compound's `PRESSURE_STATIC` is used per axle (cold-pressure default — *not* `PRESSURE_IDEAL`, which is the hot-grip target).**
- `ambient_temp_C` (float, optional, default 25.0) — used as initial tyre temperature and as `T_cold` for pressure ideal-gas evolution.
- `compound` (string, optional) — selects which compound from `car.compounds` the sim uses. Matched case-insensitively against `Compound.name` or `Compound.short_name`; also strips trailing parenthetical short-name (e.g. `"Semislicks (SM)"` matches `"Semislicks"`). When absent, the car's `default_compound_index` is used. Unknown compound → load-time error listing available compounds. `f_pressure_grip`, `f_pressure_drag`, `f_temp`, and `f_wear` all key off the resolved compound (§21.11).
- `notes` (string, optional).

**Compound resolution precedence** (high → low): `--compound` CLI flag > setup-JSON `compound` > telemetry `tyreCompound` (fitter only) > `car.default_compound_index`. Logged on stdout: `Compound: <name> (idx <i>) | source: <cli|setup|telemetry|car-default>`.

### 7.11 Stint summary CSV (output, **v2 NEW**) — `<track_stem>_stint_summary.csv`
Header: `lap,lap_time_s,tempFL_C,tempFR_C,tempRL_C,tempRR_C,wearFL_pct,wearFR_pct,wearRL_pct,wearRR_pct,pressureFL_psi,pressureFR_psi,pressureRL_psi,pressureRR_psi`. One row per lap. Wear values are 0..100 (100=fresh). Written by `report.py` when `StintResult` is supplied.

### 7.12 Telemetry CSV extension (**v2 NEW columns**)
The §14.6 schema gains 12 trailing columns (always emitted in stint mode, even when `--laps 2`):
```
...,lap,tempFL,tempFR,tempRL,tempRR,wearFL,wearFR,wearRL,wearRR,pressureFL,pressureFR,pressureRL,pressureRR
```
Units: temp in °C, wear in 0..100 (100=fresh), pressure in PSI. Per-sample values are forward-filled from the per-segment state (§21.4). For `--laps 1` runs without state evolution, these columns are still emitted but constant at the initial setup values.

## 8. Risks, constraints, and open questions

(All existing v1.1 / v1.2 / v1.2.1 risks retained — abbreviated here.)

### v2-specific risks
- **(v2) Calibration overfit to a single driver / track.** The four knobs (`k_friction`, `h`, `C_thermal`, `k_wear`) are fit against one stint's measured per-wheel evolution. They may not generalise to other tracks or drivers. Mitigation: record `source.fit_rmse_*` so a future user can spot bad fits. v3 candidate: per-compound knob tables.
- **(v2) Slip-energy estimate is coarse.** We have no per-wheel forces — slip energy is reverse-engineered from `v²/R` lat-g and a hand-coded load-transfer coefficient (`k_load ≈ 0.3/g`). Real AC slip energy includes wheel slip, camber, tyre flex, etc. The calibration step partially absorbs the error into the four knobs. Risk: the absolute values of temperature and wear may be off; the *evolution shape* should be close. Acceptance criteria (§11.27–28) target absolute deviation against measured ± 5–10%, which is loose enough to tolerate the coarseness.
- **(v2) Single-grip-envelope simplification masks balance.** Front-axle limiting vs rear-axle limiting tyre changes oversteer/understeer in reality. Our scalar reduction (`0.5 × (front_axle_min + rear_axle_min)`) loses that information. Document in §21.3 as a known v3 hook.
- **(v2) Inverse solver may have multiple solutions.** Per-wheel bisection assumes monotonic wear-vs-PSI relationship in the search range. Mitigation: bisection over `[20, 50]` psi; if bisection fails to bracket the target, surface a clear "could not find a setup hitting <target>" error. Add a coarse 5-psi-step grid scan as the bisection seed to reduce sensitivity to initial bracket.
- **(v2) Per-wheel solve makes physical sense but ergonomically the user dials four numbers into AC manually.** Pit-stop apps usually let drivers set per-wheel pressures, so this is fine; `--uniform-pressure` is the simpler answer.
- **(v2) `tyre_calibration` and `skill_pct` are coupled in the fit.** Tyre wear and driver utilisation both modulate observed cornering grip. Mitigation: fit `tyre_calibration` first against the *measured* per-wheel evolution (purely physical signal — no skill confound), then fit `skill_pct` against cornering-sample util with the calibrated `f_temp/f_wear/f_pressure_grip` already accounted for in `lat_g_max`. Flag in §13/§21.6 step ordering.
- **(v2) Lake schema requires extending `telemetry.py`'s reader.** Risk is low — 12 columns added — but the merger needs to handle absence gracefully (real AC sessions without per-wheel state must still fit `skill_pct` alone with `tyre_calibration.measured = false`).
- **(v2) Telemetry `tyreCompound` string format varies.** AC writes the long form `"Semislicks (SM)"` mid-session but some loggers strip to `"SM"` or `"Semislicks"`. Mitigation: `Car.find_compound` matches case-insensitively against both `name` and `short_name`, and strips the trailing parenthetical. Unknown compound after stripping → warn, fall back to `car.default_compound_index`. See §21.11.

### v1.3-specific risks
- **(v1.3) `k_drag` and `k_drag_reduction` are hand-defaults, not fit from data.** Real lap-time-vs-pressure curves will tell us the right values, but v1.3 ships with `k_drag = 0.5` and `k_drag_reduction = 0.1`. Mitigation: acceptance criteria §11.37–§11.39 only require the *sign* and *order of magnitude* of the asymmetry (low-PSI side flatter than high-PSI side, low-PSI deficit within ±2 s of IDEAL); we are not claiming numerical accuracy on `drag_scale`. v1.4 candidate: fit `k_drag` against a Tomas pressure sweep.
- **(v1.3) Drag plumbing seam is fuzzy in current `simulator.py`.** The 3-pass solver may not expose drag as a single multipliable knob. Mitigation: ArchDev finds the natural seam (most likely in the function that computes `F_aero` and rolling resistance for the next velocity step). If a single `drag_force(v)` exists on `Car`, wrap it. If drag is inlined into the solver, hoist it into a helper first. The change must preserve §11.30 (single-lap regression within ±0.1 s when `drag_scale == 1.0`).
- **(v1.3) Asymmetric grip model could surprise an existing tuned `skill_pct`.** Drivers fit pre-v1.3 had their `skill_pct` absorb the spurious low-PSI grip penalty. Re-fitting on the same telemetry will yield a slightly different `skill_pct`. Document as a *correction* in §13 risks for future tuners (mirrors the existing pre-v2/v2 note).
- **(v1.3) Under-pressure grip bonus ignored.** Larger contact patch at low PSI gives a small real grip *bonus* (not just "no penalty"). v1.3 takes `f_pressure_grip(p < IDEAL) = 1.0` for simplicity — see §3 non-goals. Lap times under PSI will therefore be modelled slightly slower than reality (because drag dominates and grip parity is assumed). Acceptable for v1.3; v1.4 candidate.

### v2.0.1-specific risks
- **(v2.0.1) `v_initial` interacts with corner-limit pass.** The forward pass starts at `v_initial`, but the corner-limit pass independently caps velocity at every segment by `sqrt(mu_y · g · R)`. If `v_initial > v_corner_max(segment 0)`, the natural minimum operation between forward and corner-limit passes drops the resulting velocity to `v_corner_max` on the very first step. No extra clamp needed; documented in §21.4. Risk is low — in practice, lap N-1's end-of-lap speed is typically below the start-line-segment's `v_corner_max` (otherwise lap N-1 itself wouldn't have hit that speed at the previous lap's start). One edge case: if the track CSV is laid out such that the start/finish line is the apex of a tight corner *and* the previous lap ended on a long straight, `v_initial` would exceed the cap and clip. Acceptable — matches what would happen if the driver braked into the line.
- **(v2.0.1) `simulate_stint` lap-loop ordering matters.** The grip-envelope computation, the `v_initial=v_prev_end` thread, and the per-lap update must happen in the right order: (1) compute grip envelope from current state; (2) run `simulate(...)` with current state's `mu_*_scale`, `drag_scale`, and `v_initial=v_prev_end`; (3) update state using the resulting lap arrays; (4) set `v_prev_end = lap_result.speeds[-1]` for the next iteration. Lap 1 starts with `v_prev_end = 0.0`. Mitigation: codify in §21.4 algorithm steps and in §11.47/§11.48 acceptance.

### v2-specific constraints
- ~500-line soft cap. `tyre_state.py` ~400, `solve_setup.py` ~250, `setup.py` ~120, `car.py` compound section ~150 added — all under ceiling. `simulator.py` gains `simulate_stint` (~80 lines) + `v_initial` plumbing (~10 lines) — total ~400, under ceiling.

### v2 open questions
- Should `target_wheel` default be `max` (most-worn wheel — usually outer-driven) or `avg`? §21.5 picks `max` because the user's verbatim use case ("wear will reach 80%") implies the limiting wheel; document the choice.
- `k_wear` units are awkward (per-Joule). Consider a more user-meaningful surrogate (`wear_pct_per_km_at_ref_load`) in v2.1.
- Calibration step needs a sensible loss function — RMSE on `tyreWear*` vs simulated `wear_pct` is the primary; secondary RMSE on `tyreTemp*` vs `temp_C`. Equal weights? Lap-by-lap or full-stint?

## 9. Alternatives considered

(All v1.1/v1.2/v1.2.1 entries retained — abbreviated.)

**v2-specific alternatives:**
- **(v2) Full v3 rebuild (slip-based ODE + Pacejka) instead of bolted-on tyre state.** Rejected for v2 — too large (multi-week build, §19), and the v2 use cases ("how many laps to 80%?", "what PSI for 50% at lap 12?") do not require per-wheel forces or yaw dynamics. The single-grip-envelope architecture is sufficient. v3 stays in §19 backlog.
- **(v2) Per-axle (front/rear) tyre state, not per-wheel.** Rejected — the user's verbatim use cases mention "tyres" (plural) and AC's lake telemetry already gives us per-wheel state for free. Per-axle would throw away the FL/FR and RL/RR asymmetry that matters for outer/inner wear in long stints.
- **(v2) Couple per-wheel state directly into the 3-pass solver instead of post-lap scaling.** Rejected — would require deep changes to `simulator.py` and break the back-compat regression criterion (§11.30). Post-lap scaling is the minimal change that delivers the use case.
- **(v2) Skip the inverse solver; ask the user to bisect manually.** Rejected — the user verbatim asked for "give me pressure in PSI so my tyres will have 50% after 12 laps". Inverse solver is the primary v2 deliverable.
- **(v2) Per-wheel cold pressures default to AC's `PRESSURE_IDEAL` not `PRESSURE_STATIC`.** Rejected for v2 — `PRESSURE_IDEAL` is the *hot* target the tyre should reach after warm-up; `PRESSURE_STATIC` is the realistic cold-spawn default that matches what drivers actually dial into the AC tyre app before going out. (Earlier draft had this inverted; corrected when adding compound-aware parsing — see §21.11.)
- **(v2) Numerical optimisation (Nelder-Mead, gradient-free) for the inverse solve instead of bisection.** Rejected — bisection is simpler, robust, and per-wheel one-dimensional. Multi-wheel coupled optimisation is a v2.1 candidate.
- **(v2) Parse only the first compound from `tyres.ini` (the un-suffixed sections).** Rejected — caused the live regression described in §1's "v2 compound-aware parsing" note (Tomas's Semislicks lap at 26 psi was scored against Street's `PRESSURE_IDEAL=42`). Full compound parsing has a clean shape (`car.compounds: list[Compound]`) and modest implementation cost. Per-compound *calibration knob tables* remain out of scope (v1.3).

**v1.3-specific alternatives:**
- **(v1.3) Keep the symmetric `f_pressure` quadratic; accept the low-PSI grip penalty as a "modeller's licence".** Rejected — the live regression on Tomas's 26 psi Semislicks lap produced a 6 s deficit (1:53.46 vs 1:47.33) that is physically nonsense. Real-world under-pressure produces *more* mechanical grip, not less. Keeping the symmetric model would force future fits of `skill_pct` to absorb the bug, propagating the error across drivers.
- **(v1.3) Move the entire pressure response into the drag term (no grip penalty at all).** Rejected — over-pressure *does* reduce grip (smaller contact patch is real and observable in AC at 44 psi cold, where Tomas's lap drops to 2:51). The asymmetric split preserves the over-pressure grip penalty (the part that was right) while fixing the under-pressure side (the part that was wrong).
- **(v1.3) Add a low-PSI grip *bonus* (`f_pressure_grip(p < IDEAL) > 1.0`).** Deferred to v1.4 — real but small; introducing it now adds another hand-default constant with no telemetry to fit it against. v1.3 ships with `f_pressure_grip(p < IDEAL) = 1.0` (no bonus, no penalty) and revisits in v1.4.
- **(v1.3) Apply `drag_scale` only to aero drag, not rolling resistance.** Rejected — rolling resistance is the dominant physical contributor at low PSI (sidewall flex → hysteresis loss → heat → speed loss). Splitting them adds complexity for no benefit; one combined `drag_scale` multiplier on the total drag term is simpler and physically defensible.

**v2.0.1-specific alternatives:**
- **(v2.0.1) Tile N laps into a single solver pass (extend the §20 two-lap tiling to N-lap tiling).** Rejected — the two-lap tile path only works because the grip envelope is constant across the two laps (no tyre-state update). Extending to N≥3 would either (a) require the tile to be regenerated each lap with the new grip envelope (in which case it's just N independent passes again — equivalent to the loop) or (b) accept a constant grip envelope across the whole stint (defeats the entire v2 per-lap state-evolution purpose). The loop-with-v_initial approach is the minimal change that delivers continuous velocity across N laps while preserving the lap-by-lap grip-envelope update.
- **(v2.0.1) Spawn lap N from `v_corner_max(segment 0)` instead of `lap_(N-1).speeds[-1]`.** Rejected — would not represent the actual continuous motion of a stint, and would cap every lap's start at the start/finish-line corner's grip-limited speed (typically slower than a flying lap's actual end-of-lap speed on tracks where the line crosses on a straight). The "end-of-previous-lap speed" rule is the physically correct one.
- **(v2.0.1) Add a parameter `v_initial_mode = {standing, flying, custom}` to `simulate_stint`.** Rejected — over-engineered. The universal rule "lap 1 from 0, lap N≥2 from prev v_end" covers every realistic stint case. The two-lap fast path already handles `--laps 2` byte-compat. If a future use case needs a custom initial velocity (e.g. modelling a rolling start), `simulate(..., v_initial=...)` is directly callable at the API level.

## 10. Migration

(All existing migration notes retained.)

- **(v2)** Driver JSONs without `tyre_calibration` continue to load. `Driver.load` injects defaults (`measured: false`). Existing fitter runs without per-wheel-state telemetry leave the block absent; the sim falls back to defaults.
- **(v2)** Default `--laps` is `2`, matching v1.1 two-lap behaviour byte-for-byte (acceptance §11.30). Existing scripts that pass `--single-lap` continue to work.
- **(v2)** New `setups/` directory under repo root. Tracked. README updated.
- **(v2)** `lap.py` gains a `--solve-pressure-for-wear` mode that suppresses normal output. Standard invocations (without that flag) are unchanged.
- **(v2)** `car.py` compound parsing is additive: cars with a single compound (no `_1` sections) produce `car.compounds == [Compound(index=0, name=..., ...)]` and `default_compound_index == 0`. Setup JSONs without `compound` and CLI invocations without `--compound` keep their pre-compound-aware behaviour (use the default compound). Pre-v2 `tyres.ini` parsing that read `[FRONT]/[REAR]` directly is replaced with `car.compounds[car.default_compound_index].front/rear`. Any callers of the old direct attributes (`car.pressure_ideal_front`, etc.) need to be updated to go through the active compound — see §21.11 migration notes.
- **(v1.3)** `combined_grip_envelope` return arity changes from 2 to 3. All callers in `simulator.py` / `simulate_stint` must unpack three values. Pre-v1.3 driver JSONs are unaffected (no schema change). Pre-v1.3 `skill_pct` values fit against the symmetric `f_pressure` may need re-fitting against the asymmetric model; document in §13 risks. `drag_scale == 1.0` at IDEAL pressure preserves §11.30 single-lap regression.
- **(v2.0.1)** `simulate(...)` gains a kwarg-only `v_initial: float = 0.0`. Default preserves byte-compat with all existing callers (v1.3 and earlier). Existing `simulate_monte_carlo`, `simulate(..., two_lap=True)`, and any direct external callers are unaffected unless they explicitly pass `v_initial`. `simulate_stint`'s lap loop is the only internal caller that passes a non-zero `v_initial` (and only for laps 2..N). §11.30 and §11.31 acceptance are preserved.

## 11. Acceptance criteria (for manual QA in Quix Cloud)

(Items 1–26 retained from v1/v1.1/v1.2/v1.2.1 — full text in prior revisions; abbreviated here for brevity.)

1–26. *(unchanged)*

27. **(v2 — stint sim end-to-end wear)** A 12-lap stint sim of Tomas on Sprint A, with initial pressures set from the cold-PSI estimate of his lake telemetry's first-lap-warmup `wheelsPressureFL/...` (back-solved via the ideal-gas relation against measured `tyreTempFL/...`), produces predicted end-of-stint `wearFL/FR/RL/RR` within **±5% absolute** of the lake-measured `tyreWearFL/FR/RL/RR` at the same lap. Test fixture: `samples/aclog/Tomas_Lap{1..12}.csv` (or equivalent stint from the lake). Calibration knobs from the fitter (`tyre_calibration.measured == true`).
28. **(v2 — stint sim pressure evolution)** Same 12-lap stint produces predicted per-lap `pressureFL/FR/RL/RR` matching lake-measured `wheelsPressureFL/...` within **±10%** at each lap.
29. **(v2 — inverse solver hits target wear)** `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json --solve-pressure-for-wear 0.50 --at-lap 12` returns four PSI values such that re-running `python lap.py ... --laps 12 --pressure FL=...,FR=...,RL=...,RR=...` predicts wear at lap 12 within **±1% absolute** of 50% on at least the **target wheel** (default `max`, i.e. most-worn). With `--uniform-pressure`, the returned single PSI is applied to all four wheels and the same ±1% target is required on the most-worn wheel.
30. **(v2 — single-lap regression safety)** `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json --laps 1` reproduces the v1.2.1 lap-1 time within **±0.1 s** for the same driver JSON and track CSV. (Guards against the stint wrapper accidentally perturbing the existing `simulate(...)` path.) **(v1.3 amendment)** At IDEAL pressure (Semislicks 33 psi cold), `drag_scale` resolves to 1.0 and the result is byte-equivalent to v2. **(v2.0.1 amendment)** `v_initial=0.0` default preserves byte-compat — lap 1 is always a standing start, and `--laps 1` only runs lap 1, so `v_initial` is never overridden.
31. **(v2 — `--laps 2` byte-compat)** `python lap.py ... --laps 2` produces lap-1 and lap-2 times within **±0.05 s** of the v1.2.1 two-lap default. Telemetry CSV header includes the 12 new state columns; their values are constant across `lap == 1` and `lap == 2` and equal to the setup defaults (no state evolution at `n_laps == 2` when `tyre_calibration.measured == false`). **(v2.0.1 amendment)** The `--laps 2` fast path still delegates to `simulate(..., two_lap=True)` (single solver pass with continuous v across the tile), so `v_initial` is not exercised here. Byte-compat preserved.
32. **(v2 — setup CLI precedence)** Running with `--setup setups/bmw_1m_default.json --pressure FL=33` overrides only FL; FR/RL/RR stay at the setup file's values. Running with neither flag picks the active compound's `PRESSURE_STATIC` from `tyres.ini` per axle. Logged on stdout: `Setup: <source> | FL=... FR=... RL=... RR=... | ambient=...°C`.
33. **(v2 — per-lap stdout block)** `--laps 5` prints a five-line block:
    ```
    Lap 1: 1:46.213 | wear FL=98% FR=97% RL=95% RR=94% | temp avg 76°C | pressure avg 31.4 psi
    Lap 2: 1:46.198 | ...
    ...
    Lap 5: 1:46.402 | wear FL=87% FR=86% RL=78% RR=77% | temp avg 82°C | pressure avg 33.1 psi
    ```
34. **(v2 — compound parsing, multi-compound car)** `Car.from_dir("cars_csv/bmw_1m")` loads two compounds: `car.compounds[0].name == "Street"`, `car.compounds[0].short_name == "ST"`, `car.compounds[1].name == "Semislicks"`, `car.compounds[1].short_name == "SM"`. `car.compounds[0].pressure_ideal_front == 42.0`, `car.compounds[1].pressure_ideal_front == 33.0`. `car.compounds[0].pressure_d_gain == 0.004`, `car.compounds[1].pressure_d_gain == 0.0045`. Wear LUTs and thermal LUTs resolve to the per-compound files (`street_*.lut` vs `semislicks_*.lut`; `[THERMAL_FRONT]` vs `[THERMAL_FRONT_1]`). `car.default_compound_index == 0` (no `[COMPOUND_DEFAULT]` section in the BMW M1 `tyres.ini`). **(v1.3 amendment)** Both compounds carry `k_drag == 0.5` and `k_drag_reduction == 0.1` (hand-defaults; not parsed from `tyres.ini`).
35. **(v2 — `--compound` CLI selects Semislicks)** `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json --compound Semislicks --pressure FL=26,FR=26,RL=26,RR=26 --laps 1` runs without `f_pressure_grip` collapsing to the 0.30 grip floor (i.e. the resulting `mu_y_scale` from `combined_grip_envelope` is `== 1.0` at lap-1 because 26 psi is below Semislicks IDEAL=33 → `f_pressure_grip` returns 1.0 on the under-pressure side). With `--compound Street` and the same 26 psi, `f_pressure_grip` is at the floor (26 < 42 → wait, Street IDEAL=42, so 26 is also under-pressure → also returns 1.0 in v1.3; this part of the regression-of-the-bug test is now subsumed by §11.37). Stdout logs `Compound: Semislicks (idx 1) | source: cli`.
36. **(v2 — fitter auto-selects compound from telemetry)** `python fit_driver.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json samples/aclog/Tomas_Lap*.csv` against CSVs containing `tyreCompound="Semislicks (SM)"` auto-selects `car.compounds[1]` (Semislicks) for the calibration fit and for the `lat_g_max` envelope used in the skill-percentile step. The written driver JSON has `tyre_calibration.source.compound == "Semislicks"`. With telemetry whose `tyreCompound` value is `"Foobar"` (not in `car.compounds`), the fitter prints a warning and falls back to `car.compounds[car.default_compound_index]`.
37. **(v1.3 — low-PSI lap time within ±2 s of IDEAL)** With Semislicks compound, calibrated Tomas driver, ambient 26 °C, `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json --compound Semislicks --laps 3 --pressure FL=26,FR=26,RL=26,RR=26 --ambient-temp-c 26.0` produces an average lap time that is **≤ baseline + 2 s** (or *faster*) compared to the same run at `--pressure FL=33,...` (Semislicks IDEAL). Pre-v1.3 (symmetric quadratic) produced 1:53.46 vs 1:47.33 (6 s deficit). Post-v1.3 acceptance: the gap is in `[-1 s, +2 s]` — low-PSI side may be slightly slower due to drag, may be slightly faster, must not be massively slower.
38. **(v1.3 — over-pressure grip penalty preserved)** With Semislicks compound, calibrated Tomas driver, ambient 26 °C, `python lap.py ... --compound Semislicks --laps 3 --pressure FL=44,FR=44,RL=44,RR=44` continues to produce a slow lap time (current pre-v1.3 baseline: 2:51.33). Post-v1.3: still > IDEAL lap time by `≥ 30 s`. The over-pressure grip penalty (`f_pressure_grip(p > IDEAL) < 1.0`) is preserved end-to-end.
39. **(v1.3 — pressure sensitivity sweep is asymmetric)** A sweep across cold pressures `{22, 26, 29, 33, 37, 40, 44}` psi (Semislicks, all four wheels uniform, `--laps 3`, calibrated Tomas) produces a lap-time curve whose **low-PSI side (22–33) is flatter than the high-PSI side (33–44)**. Quantitatively: `(lap_time(22) - lap_time(33)) < (lap_time(44) - lap_time(33))`, and ideally the low-PSI side stays within ~5 s of IDEAL while the high-PSI side balloons by tens of seconds. Demonstrates the asymmetric model is wired through end-to-end.

47. **(v2.0.1 — 3-lap stint produces flying lap 2 and lap 3)** `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas.json --laps 3 --compound Semislicks --pressure FL=33,FR=33,RL=33,RR=33 --ambient-temp-c 26.0` (fresh tyres, no calibration / `tyre_calibration.measured == false`) produces lap times where lap 2 and lap 3 are **both** within **±0.3 s of each other** and each is **at least 2 s faster than lap 1**. Pre-v2.0.1, lap 2 and lap 3 were both standing-start laps with the same lap-1 pace (no flying-lap benefit). Post-v2.0.1, laps 2 and 3 inherit the previous lap's `v_end` and run at flying-lap pace. The stdout block (§11.33) shows the speed-up visibly.
48. **(v2.0.1 — multi-lap stint lap 2 matches two-lap-tile lap 2)** A 5-lap stint's lap 2 time (`python lap.py ... --laps 5 --compound Semislicks --pressure FL=33,...,RR=33` with `tyre_calibration.measured == false`) matches the same configuration's `--laps 2` lap-2 time within **±0.05 s**. Both paths see the same `v_initial` for lap 2 — the two-lap tile's continuous solver pass produces it implicitly, the N≥3 loop produces it explicitly via `v_prev_end = lap_1.speeds[-1]`. The two paths should converge on identical lap-2 dynamics when no state evolution intervenes (`measured == false`). Larger tolerance (±0.05 s rather than byte-equal) accounts for floating-point ordering differences between a single solver pass over a tiled grid vs two separate solver passes.

## 12. References

`docs/AI_CONTEXT.md`, `README.md`, `src/lap_estimator/*.py`, `cars_csv/bmw_1m/tyres.ini`, `samples/aclog/*.csv`, `tracks_config.json`. **(v2)** AC `tyres.ini` `WEAR_CURVE`, `PRESSURE_IDEAL`, `PRESSURE_STATIC`, `PRESSURE_D_GAIN`, `[FRONT_n]`/`[REAR_n]`/`[THERMAL_FRONT_n]`/`[THERMAL_REAR_n]` compound sections, optional `[COMPOUND_DEFAULT]` `INDEX`, `[THERMAL_FRONT/REAR].PERFORMANCE_CURVE` (`tcurve_*.lut`). Lake schema: `reference_ac_telemetry_schema.md` (per-wheel state columns + `tyreCompound` confirmed 2026-05-14).

---

## 13. Telemetry-driven driver fitting (addendum, v1 + v1.1 multi-lap mandatory + v1.2 profile-dynamics + v2 tyre-calibration)

*(All v1/v1.1/v1.2/v1.2.1 content retained verbatim from prior revisions — see prior revision §13.1 through §13.13. The full algorithm is unchanged; v2 adds one optional step.)*

### 13.14 Tyre-state calibration step (v2 — new, optional)

**Goal.** When the input telemetry CSVs contain per-wheel state channels (`tyreTempFL/FR/RL/RR`, `tyreWearFL/FR/RL/RR`, `wheelsPressureFL/FR/RL/RR`; optionally `wheelLoadFL/...` for slip-energy attribution), fit the four tyre-calibration knobs (`k_friction`, `h`, `C_thermal`, `k_wear`) so the simulated state evolution matches the measured trace across all input laps. When the channels are absent, skip the step and emit `tyre_calibration.measured = false` with hand-defaults.

**Ordering inside `fit_driver(...)`.** The calibration step runs **before** the `skill_pct` percentile (§13.5 step 5) so that the cornering-grip-utilisation calculation in step 4 can use the calibrated `f_temp/f_wear/f_pressure_grip` envelope in `lat_g_max`. This decouples tyre-state confounds from driver skill — flagged in §13 risks and §8 v2 constraints.

**Compound resolution (v2).** Before fitting, the fitter reads `tyreCompound` from the merged telemetry frame (most common value across all rows wins). It calls `car.find_compound(value)` to resolve to a `Compound`. On match, the calibration step and the `lat_g_max` envelope use that compound's `f_pressure_grip/f_pressure_drag/f_temp/f_wear`. On miss (unknown value), warn `Tyre compound "<value>" not found in car.compounds — falling back to default "<default>"` and use `car.compounds[car.default_compound_index]`. The resolved compound name is written under `tyre_calibration.source.compound`.

**Algorithm.**
1. **Detect per-wheel state availability.** Inspect the union of columns across all input frames. Required for calibration: all four `tyreTemp*` and all four `tyreWear*`. `wheelsPressure*` is desirable but not required (ideal-gas relation can fill it). If required channels absent, set `tyre_calibration.measured = false`, populate defaults, skip to step 5b.
2. **Per-frame, per-segment, derive simulated state under candidate knobs.** Walk the merged frame; at each `(distance_m, v, radius)`, compute slip-energy attribution per §21.3, then advance per-wheel `temp_C`, `wear_pct`, `pressure_psi` **using the resolved compound's curves**. The candidate knobs are `(k_friction, h, C_thermal, k_wear)`.
3. **Fit loss = weighted RMSE.** `L = w_T · RMSE(temp_C - tyreTemp_*) + w_W · RMSE(wear_pct - tyreWear_*) + w_P · RMSE(pressure_psi - wheelsPressure_*)`. Weights `w_T = 1.0`, `w_W = 5.0` (wear is the user-facing target), `w_P = 0.5`.
4. **Minimise via SciPy `minimize(method='Nelder-Mead')`** over `log10` of each of the four knobs (positive-only). Initial guess: the defaults. Bounds enforced by clipping inside the loss function. Max 100 iterations.
5. **Record under `tyre_calibration`** (§7.2) with `measured = true`, `source.fit_rmse_*` filled, `source.fitted_at` set, `source.compound` set to the resolved compound name.
5b. **No-measurement fallback.** Defaults written, `measured = false`, log line: `Tyre calibration: per-wheel state channels absent — using hand-defaults`.

**JSON output.** Added under the top-level `tyre_calibration` key (§7.2).

**Acceptance.** §11.27, §11.28 require `tyre_calibration.measured = true` and exercise the calibration end-to-end. §11.30 covers the no-measurement path. §11.36 covers compound auto-selection.

---

## 14. Synthetic telemetry emission (v1 + v1.1 + v1.2 + v1.2.1 + v2)

*(All v1/v1.1/v1.2/v1.2.1 content retained from prior revisions. v2 adds the 12 per-wheel state columns.)*

### 14.6 Output schema (v2 update)
```
timestamp_ms,gas,brake,distanceTraveled,speedKmh,normalizedCarPosition,lap,tempFL,tempFR,tempRL,tempRR,wearFL,wearFR,wearRL,wearRR,pressureFL,pressureFR,pressureRL,pressureRR
```
Units: temp °C; wear 0..100 (100=fresh); pressure PSI. Values forward-filled from per-segment state. Constant across the lap when `n_laps == 1` and `tyre_calibration.measured == false`.

### 14.12 v2 telemetry-emission acceptance criteria
14. **(v2)** Stint mode (`--laps N` with N ≥ 1) emits the 12 per-wheel state columns. At `--laps 1` without calibration, values are constant at setup defaults. At `--laps 12` with calibration, values evolve monotonically (wear strictly decreasing, temp warming then plateauing, pressure rising with temp).

---

## 15. Cross-track validation workflow (addendum, v1)

*(Unchanged.)*

---

## 16. Preparation pipeline (addendum, v1 reorg)

*(Unchanged.)*

---

## 17. Corner analysis (addendum, v1 reorg)

*(Unchanged.)*

---

## 18. MF4 telemetry output (v2, PLANNED — not implemented)

*(Unchanged.)*

---

## 19. Backlog — complex physics & tyre damage (research, v3)

*(Unchanged. Note: §19.3 "Item B — tyre damage" is **partially superseded** by §21 in v2. §19.3 retained as research scope for the deeper integration that lands in v3 alongside slip-based physics. The v2 implementation in §21 is the coarse, single-grip-envelope, post-lap-scaling version of the same idea.)*

---

## 20. Two-lap "tiled" simulation (addendum, v1.1)

*(Unchanged. v2 note: with `--laps 2` and `tyre_calibration.measured == false`, behaviour is byte-equivalent to v1.2.1 modulo the 12 new constant telemetry columns — §11.31. The stint wrapper in §21.4 detects `n_laps == 2 && !measured` and delegates straight to the existing `simulate(..., two_lap=True)` path for back-compat.)*

**(v2.0.1 universal-rule note, 2026-05-15):** the §20 two-lap tile is one of *two* mechanisms in the codebase that produce continuous velocity across laps. The other is the §21.4 multi-lap loop with `v_initial` threading. The universal rule across both: **lap 1 starts at v=0; lap N≥2 starts at the end-of-lap velocity of lap N-1**. The two-lap tile achieves this *implicitly* via a single solver pass over a doubled segment list — the forward pass naturally carries velocity across the start-finish boundary. The multi-lap loop achieves it *explicitly* via `v_prev_end = lap_result.speeds[-1]` threaded into the next iteration's `simulate(..., v_initial=v_prev_end)` call. Both paths exist because the two-lap tile only works when the grip envelope is constant across the two laps (no tyre-state update between them); for N≥3 with per-lap state evolution, the loop is required. The §11.31 / §11.48 acceptance criteria check that the two paths agree on lap-2 dynamics when no state evolution intervenes.

---

## 21. Per-wheel tyre state & stint simulation (v2 — NEW)

### 21.1 Goal

Two concrete user use cases, verbatim:
> "I have to estimate after how many laps tyres will have e.g. 80%."
> "Driver telling: give me pressure in PSI so my tyres will have 50% after 12 laps."

Deliver these by adding **per-wheel tyre state (temperature, wear, pressure)** that evolves across a multi-lap stint, plus an **inverse-PSI solver** that recommends cold setup pressures for a target wear at a target lap.

**Architectural posture (option 2.5, not v3).** The existing 3-pass `simulator.py` and its single-scalar grip envelope are retained unchanged. Per-wheel state is computed **between laps**, then reduced to a scalar grip multiplier that scales `mu_x`/`mu_y` for the next lap's solver pass. Per-wheel forces, yaw dynamics, Pacejka, and slip-based control-loop drivers stay in v3 (§19).

### 21.2 Non-goals

- Per-wheel forces, yaw dynamics, Pacejka tyre model — all v3 (§19).
- Tyre puncture / catastrophic failure modes.
- Marble pickup, track-evolution grip, surface-rubber rollout.
- Heat soak across pit stops, multi-stint sessions. v2 is one stint, no pit stop.
- Slip-based control-loop drivers (§19.2.6) — v3.
- ~~Per-compound thermal/wear LUT switching beyond first-compound default.~~ **(v2 amendment, 2026-05-14):** per-compound `f_pressure_grip`/`f_pressure_drag`/`f_temp`/`f_wear` LUT switching is now **in scope** for v2 — see §21.11. Out of scope: per-compound *calibration knob tables* (`k_friction`/`h`/`C_thermal`/`k_wear` stay compound-agnostic) and compound *mid-stint switching* (pit-stop compound change). Both are v1.3 candidates.
- True weight-transfer transients (load builds up over time-domain integrations). v2 uses a quasi-static load-transfer coefficient (`k_load`).

### 21.3 Algorithm — per-wheel state evolution

State per wheel `w ∈ {FL, FR, RL, RR}`:
- `temp_C[w]` — bulk tyre temperature (Celsius). Initial = `setup.ambient_temp_C` (or 25°C default).
- `wear_pct[w]` — 0..100, 100 = fresh, 0 = bald. Initial = 100.
- `pressure_psi[w]` — measured cold pressure. Initial = setup value (per-wheel from `setups/*.json` OR `--pressure FL=...,FR=...,RL=...,RR=...` OR active compound's `PRESSURE_STATIC` per axle).

Plus diagnostic accumulators:
- `cumulative_slip_energy_J[w]` — running sum across the stint.
- `T_cold_K[w]` — pressure ideal-gas reference (set once at stint start; equals `ambient_temp_C + 273.15`).

**Active compound (v2).** The per-segment update receives a `Compound` reference (see §21.11). All curves (`f_pressure_grip`, `f_pressure_drag`, `f_temp`, `f_wear`, wear-LUT, thermal-LUT) are looked up off that `Compound`, not off `car` directly.

**Per-segment update (called once per simulator segment `(distance_m, v, radius)`):**

1. **Slip-energy attribution.**
   - Estimate lateral acceleration: `lat_g = v² / (R · g)` if `R < STRAIGHT_THRESHOLD_M`, else 0.
   - Estimate longitudinal acceleration from the simulator's binding label (corner/accel/brake): `long_g = car.max_accel(v)/g` for accel; `-car.max_braking_decel(v)/g` for brake; 0 for corner.
   - Per-wheel normal-load multiplier from load transfer:
     - `lat_factor[outer_wheels] = 1 + k_load · |lat_g|`; `lat_factor[inner_wheels] = 1 - k_load · |lat_g|`. Outer/inner determined by sign of `radius` (left vs right corner — derived from track CSV signed radius). `k_load = 0.30` (hand-default; v2.1 candidate to make per-car).
     - Longitudinal transfer: `long_factor[front] = 1 - k_long · long_g` for accel (weight shifts rearward); `long_factor[rear] = 1 + k_long · long_g`. For brake, signs flip. `k_long = 0.20` (hand-default).
   - Per-wheel normal load: `Fz[w] = (m·g + downforce) · load_share[w]`, where `load_share[w]` integrates `cg_front`, `lat_factor`, `long_factor` and is normalised to sum to 1.0 across the four wheels.
   - Driven-axle longitudinal energy carried by RWD = rear pair / FWD = front pair / AWD = all four (read `drive_type` from `Car`).
   - Brake longitudinal energy split by `car.brake_front_share` (read from `brakes.ini`).
   - **Slip-energy per wheel per segment:** `dE[w] = k_slip · Fz[w] · |a[w]| · dt`, where `a[w]` is the per-wheel longitudinal acceleration (0 for non-driven/non-braked wheels), and lateral contribution adds `Fz[w] · |lat_g| · |v| · dt`. Time step `dt = segment_length / v`. `k_slip` is a scalar calibration that rolls into `k_friction` — we don't separate them in v2.
2. **Temperature update (per wheel).**
   - `P_heat[w] = k_friction · dE[w] / dt` (Watts).
   - `dT/dt[w] = (P_heat[w] - h · (temp_C[w] - ambient_temp_C)) / C_thermal`.
   - Euler step: `temp_C[w] += dT/dt[w] · dt`.
3. **Wear update (per wheel).**
   - `f_temp_penalty(T)` — sharply rising above optimal. Use the **active compound's** `thermal_lut` (the `[THERMAL_FRONT_n]/[THERMAL_REAR_n].PERFORMANCE_CURVE`); the LUT itself is grip-vs-temp; invert to a wear penalty as `penalty = 1 / max(grip(T), 0.3)`. Optimal-temp window inferred from LUT peak. Front vs rear use the relevant axle's LUT.
   - `dwear/dt[w] = k_wear · dE[w] / dt · f_temp_penalty(temp_C[w])` (units: pct/s).
   - `wear_pct[w] -= dwear/dt[w] · dt` (clamped to `[0, 100]`).
4. **Pressure update (per wheel).**
   - `pressure_psi[w] = pressure_cold_psi[w] · (temp_C[w] + 273.15) / T_cold_K[w]`. Ideal-gas, isovolumetric. Pure function of current temp; recomputed each segment.

**Per-lap state passthrough.** End-of-lap state (per-wheel `temp_C`, `wear_pct`, `pressure_psi`) becomes start-of-lap-(N+1). No reset between laps.

**Scalar grip envelope reduction (called once per lap, before the next lap's 3-pass solver):**

1. **Per-wheel grip multiplier** (cornering envelope — what `mu_x`, `mu_y` scale by):
   `g[w] = f_temp(temp_C[w]) · f_wear(wear_pct[w]) · f_pressure_grip(pressure_psi[w])`.
   - `f_temp(T)` — from the **active compound's** `thermal_lut` (axle-appropriate).
   - `f_wear(w_pct)` — from the **active compound's** `wear_curve_lut` (axle-appropriate).
   - **`f_pressure_grip(p_psi)`** (v1.3 — asymmetric):
     - `p ≥ PRESSURE_IDEAL` (over-pressure side): `f_pressure_grip(p) = 1 - PRESSURE_D_GAIN · (p - PRESSURE_IDEAL)²`, clamped to `[0.30, 1.0]`. (Smaller contact patch → less grip — physically real.)
     - `p < PRESSURE_IDEAL` (under-pressure side): `f_pressure_grip(p) = 1.0`. (No grip penalty. Larger contact patch in reality gives a small bonus; v1.3 ignores the bonus — see §3 non-goals.)
     - Uses the **active compound's** `PRESSURE_IDEAL` (axle-appropriate) and `PRESSURE_D_GAIN`.

2. **Per-wheel drag multiplier** (longitudinal drag envelope — aero drag + rolling resistance on straights):
   `d[w] = f_pressure_drag(pressure_psi[w])`.
   - **`f_pressure_drag(p_psi)`** (v1.3 — asymmetric):
     - `p < PRESSURE_IDEAL` (under-pressure side): `f_pressure_drag(p) = 1 + k_drag · max((PRESSURE_IDEAL - p) / PRESSURE_IDEAL, 0)`. More drag (sidewall flex + rolling resistance). At `p = 0` hypothetically, drag is `1 + k_drag`. Hand-default `k_drag = 0.5`.
     - `p ≥ PRESSURE_IDEAL` (over-pressure side): `f_pressure_drag(p) = 1 - k_drag_reduction · (p - PRESSURE_IDEAL) / PRESSURE_IDEAL`. Tiny benefit on straights (less rolling resistance from a stiffer, smaller patch). Hand-default `k_drag_reduction = 0.10`.
     - Final clamp: `f_pressure_drag(p) ∈ [0.85, 1.50]`.
     - Constants live on `Compound` (see §21.11 / §6.14). Compound-agnostic in v1.3 (same 0.5 / 0.10 for both Street and Semislicks); per-compound values are a v1.4 candidate.

3. **Front-axle / rear-axle grip:** `g_front = min(g[FL], g[FR])`; `g_rear = min(g[RL], g[RR])`.
4. **Combined scalar grip:** `g_combined = 0.5 · (g_front + g_rear)`. **Documented simplification:** equal-weight axle average loses balance information (oversteer/understeer requires yaw dynamics — see v3 §19.2). Acceptable for "how many laps to 80%?" / "what PSI for 50%?" use cases.
5. **Combined scalar drag:** `drag_scale = mean(d[FL], d[FR], d[RL], d[RR])`. Equal-weight four-wheel average. Drag is a single-vehicle quantity (the whole car experiences one drag force); per-wheel pressures average naturally since all four contribute to rolling resistance.
6. **Return:** `combined_grip_envelope(state, compound) -> (mu_x_scale, mu_y_scale, drag_scale)` where `mu_x_scale = mu_y_scale = g_combined`.

**Solver wiring (v1.3 — Drag plumbing).** The 3-pass solver in `simulator.py` consumes `(mu_x_scale, mu_y_scale, drag_scale)`:
- `mu_x_scale` / `mu_y_scale` scale `car.tyre_dy0_*` / `tyre_dx0_*` as in v2 (unchanged).
- `drag_scale` multiplies the total drag force the solver applies during forward and backward velocity passes. Natural seam: if `Car` exposes a `drag_force(v)` or `total_drag(v)` helper, wrap it with `lambda v: drag_scale * car.drag_force(v)`. If drag is inlined into the solver's velocity-update loop (mixed aero + rolling-resistance term), refactor the smallest possible function around it so the multiplier lands in one place. The change must preserve §11.30: at `drag_scale == 1.0` (which is what IDEAL pressure produces) the lap time is byte-equivalent to v2.

**Rationale for the asymmetric split (v1.3 motivation).** The pre-v1.3 symmetric quadratic `f_pressure(p) = 1 - PRESSURE_D_GAIN · (p - IDEAL)²` penalised grip on both sides. Physically wrong on the low side: under-pressure produces a *larger* contact patch (more mechanical grip), but slower lap times come from *higher rolling resistance and sidewall flex* (drag). The pre-v1.3 model double-counted — penalising grip when the real penalty is in drag. The fix is the split above: grip term penalty active only above IDEAL; drag term penalty active only below IDEAL. Live regression: Tomas's Semislicks lap at 26 psi (under-pressure, IDEAL=33) scored at 1:53.46 vs 1:47.33 at IDEAL — a 6 s deficit that the asymmetric model reduces to ~0–2 s (the residual is drag, which is physically expected and small at 26 psi). Over-pressure stays slow (44 psi → 2:51) because the grip term still penalises it.

### 21.4 Multi-lap stint loop

**CLI flag (`lap.py`):**
- `--laps N` (int, default `2`, range `[1, 50]`).
- `--laps 1` is equivalent to legacy `--single-lap`.
- `--laps 2` is byte-compatible with the v1.1 default (§11.31).
- `--laps N` for N ≥ 3 engages stint mode.

**Velocity-continuity rule (v2.0.1 — universal).** Every stint-mode run obeys: **lap 1 starts at v = 0 (standing start); lap N for N ≥ 2 starts at lap N-1's end-of-lap velocity**. The rule applies to every stint regardless of `n_laps`, calibration state, or compound. There are two implementation paths in the codebase, both consistent with the rule:

1. **Two-lap tile fast path (`n_laps == 2 && !measured`).** Delegates to `simulate(..., two_lap=True)`. The tile is a single solver pass over a segment list shifted by `lap_length`; the forward pass naturally carries velocity across the start-finish boundary. No explicit `v_initial` threading is needed — the continuous velocity emerges from the single pass. Preserves byte-compat with v1.1 (§11.31).
2. **Multi-lap loop (every other case).** `simulate_stint` runs `simulate(...)` N times. Lap 1 is called with `v_initial=0.0` (default). Each subsequent lap is called with `v_initial=lap_prev.speeds[-1]`. This delivers the same velocity continuity per-lap that the tile achieves all-at-once. The two paths are tested for agreement by §11.31 (lap 2 vs v1.1 two-lap tile) and §11.48 (5-lap stint's lap 2 vs 2-lap stint's lap 2, both with no state evolution).

**v_end definition.** "lap N-1's end-of-lap velocity" is `lap_result.speeds[-1]` — the velocity at the **last sample** of the per-point arrays for lap N-1. Not interpolated to the start-finish line; the per-point grid is dense enough that the last sample is effectively at the line.

**Edge case (over-cap).** If `v_initial >= v_corner_max(starting segment)`, the corner-limit pass's clamp will drop the forward-pass first-step velocity to `v_corner_max` naturally. No explicit guard is added in `simulate(...)`. In practice, this never fires for a coherent stint — lap N-1's end-of-lap speed is bounded by the corner-limit pass that *generated* lap N-1, and the start-finish-line segment's `v_corner_max` is the same value in lap N. The only way to trigger it is if the active compound changes (out of scope for v2 — see §21.2) or if the grip envelope drops enough between laps that the previous-lap end speed is now above the new lap's corner cap (theoretically possible with extreme wear / temperature drops — in which case the natural clamp is the right behaviour).

**`simulator.simulate_stint(car, track, driver, *, n_laps, setup, compound, calibration, ds=2.0) -> StintResult`:**
1. Initialise `TyreState` from `setup.pressures_psi`, `setup.ambient_temp_C`. Bind `compound` (the resolved active compound from §21.11). Initialise `v_prev_end = 0.0` (lap 1 is standing start).
2. **Lap loop.** For `lap in 1..n_laps`:
   a. Compute current scalar grip envelope `(mu_x_scale, mu_y_scale, drag_scale)` from `TyreState` and `compound` (§21.3 reduction).
   b. Scale `car`'s `tyre_dy0_*`/`tyre_dx0_*` by `mu_*_scale` — using the active compound's `dy0/dy1/dx0/dx1` as the base, *not* the un-suffixed defaults (wrapper, not in-place — preserves `car` immutability).
   c. **(v2.0.1)** Run `simulate(scaled_car, track, driver, ds=ds, two_lap=False, drag_scale=drag_scale, v_initial=v_prev_end)` → one-lap `SimResult`. **Lap 1: `v_prev_end == 0.0`** (standing start). **Lap N ≥ 2: `v_prev_end == lap_(N-1).speeds[-1]`** (continuous with previous lap). **(v1.3)** `drag_scale` is threaded into the 3-pass solver as described in §21.3 "Solver wiring".
   d. Walk the per-point arrays and update `TyreState` segment-by-segment (§21.3 steps 1–4) using the active `compound`. Record per-lap-end state snapshot.
   e. Append per-lap-end state and lap-time to `StintResult`.
   f. **(v2.0.1)** Set `v_prev_end = lap_result.speeds[-1]` for the next iteration.
3. Return `StintResult`.

**`StintResult` dataclass** (in `simulator.py`):
```python
@dataclass
class StintResult:
    n_laps: int
    setup: Setup
    compound: Compound                  # active compound for the stint (v2)
    calibration: TyreCalibration
    lap_times_s: list[float]            # length n_laps
    tyre_state_history: list[TyreState] # length n_laps + 1 (initial + end-of-each-lap)
    per_lap_sim_results: list[SimResult] # for telemetry emission
```

**Telemetry emission (12 new columns — §7.12).** `sim_telemetry.write_synthetic_log` is extended to accept a `StintResult` (alternative to `SimResult`). The per-sample state values are forward-filled from the per-segment update — the writer walks segments in time order, updating state, and emits the current state alongside each row.

### 21.5 Inverse-PSI solver

**CLI shape (added to `lap.py`):**
```
python lap.py <car> <track> <driver> \
              --solve-pressure-for-wear <pct> --at-lap <N> \
              [--target-wheel min|max|avg|FL|FR|RL|RR] \
              [--uniform-pressure] \
              [--setup <path>] [--ambient-temp-c 25.0] [--compound <name>]
```

`<pct>` is in `[0.0, 1.0]` (0.50 = 50% wear). `<N>` is target lap. Default `--target-wheel` is `max` (most-worn wheel — typically the outer-driven wheel). `--compound` follows the same precedence rules as in normal sim mode (§7.10).

**Algorithm (`solve_setup.solve_pressure_for_wear`):**

1. **Coarse seed scan.** For `seed_psi in {22, 27, 32, 37, 42, 47}`, run `simulate_stint(..., n_laps=N, setup=Setup(pressures={all: seed_psi}, ambient=..., compound=...))`. Record `target_wear_observed[seed_psi] = aggregate(wear_pct_at_lap_N, target_wheel)`. `aggregate` reduces 4 wheels → 1 number per the user's `--target-wheel` choice.
2. **Bracket.** Find adjacent `(seed_lo, seed_hi)` where `target_wear_observed` straddles the target. If no bracket found across the seed grid, error: `could not find a setup hitting <pct>% wear at lap <N> in the PSI range [20, 50]`.
3. **Bisection.** Per wheel (or single, if `--uniform-pressure`), bisect `[seed_lo, seed_hi]` for ≤ 12 iterations. Each iteration calls `simulate_stint(...)` with the candidate pressures.
4. **Per-wheel bisection scheme (when not `--uniform-pressure`).** Bisect each wheel independently in turn (4 outer loops × ≤ 12 inner iterations = ≤ 48 stint sims worst case). Wheels are bisected in order `[FL, FR, RL, RR]`; after each wheel converges, its pressure is fixed and the next wheel bisects against the residual error. (Greedy coordinate descent — converges in practice on monotonic-wear-vs-PSI assumptions.)
5. **Convergence.** Stop when `|target_wear_observed - <pct>| ≤ 0.01` (1% absolute) OR `|psi_hi - psi_lo| ≤ 0.5`.
6. **Output:** four PSI values (or one if `--uniform-pressure`), plus a verification stint sim's per-lap state history.

**Stdout output:**
```
Recommended setup for 50% wear at lap 12 on layout_sprint_a:
  Compound: Semislicks (idx 1) | source: cli
  FL = 31.2 psi  (cold)
  FR = 30.8 psi  (cold)
  RL = 28.4 psi  (cold)
  RR = 28.1 psi  (cold)  <-- target wheel (max wear)

Verification — predicted state under recommended setup:
  Lap  Time     Wear FL/FR/RL/RR              Temp avg  Pressure avg
  ----  ------  ------------------------------ --------  ------------
   1   1:46.213 98% / 97% / 95% / 94%         76 °C     31.0 psi
   2   1:46.198 96% / 94% / 89% / 88%         81 °C     32.4 psi
   ...
  12   1:47.804 78% / 76% / 53% / 50% <-      89 °C     34.1 psi

Verdict: target hit (RR=50.0% ± 0.1%).
```

### 21.6 Lake integration & fit

**Extending `telemetry.read_ac_log`:**
- Opportunistically read: `tyreCompound` (str), `wheelsPressureFL/FR/RL/RR` (PSI), `tyreTempFL/FR/RL/RR` (°C, core), `tyreWearFL/FR/RL/RR` (0..100, 100=fresh), `wheelLoadFL/FR/RL/RR` (N, optional).
- Missing columns warn-and-continue; emit a single warning per file (`per-wheel state columns absent — tyre_calibration will use defaults`).
- All extra columns passed through to the merged frame (downstream tools can inspect them).

**Fit integration (in `fit_driver.fit_driver(...)` — §13.14):**
- Compound resolution from telemetry (most-common `tyreCompound`) runs **before** the calibration step.
- Calibration runs **before** `skill_pct` percentile.
- Calibrated `f_temp/f_wear/f_pressure_grip` envelope (from the resolved compound) feeds into the `lat_g_max` calculation (§13.5 step 3) so `skill_pct` does not absorb tyre-state confounds. **(v1.3)** `f_pressure_drag` does *not* enter `lat_g_max` — it's a straight-line drag term, not a cornering grip term.
- New driver-JSON field `tyre_calibration` (§7.2). `tyre_calibration.source.compound` records the resolved compound name.

**Note on `skill_pct` tightening.** v2 calibration removes tyre-state confounds from the cornering-grip-utilisation calculation. Expect measured `skill_pct` to shift slightly across pre-v2/v2 fits of the same driver on the same telemetry — this is a *correction*, not a regression. Document in §13 risks for future tuners. **(v1.3)** A second, smaller shift occurs across pre-v1.3/v1.3 fits because the under-pressure grip penalty is removed from `f_pressure_grip`. Same nature — a correction.

### 21.7 Setup config

**Directory:** `setups/` at repo root. Tracked.

**Example file `setups/bmw_1m_default.json`:**
```json
{
  "car": "bmw_1m",
  "name": "default",
  "ambient_temp_C": 25.0,
  "compound": "Street"
}
```

Note: when `pressures_psi` is omitted, the active compound's `PRESSURE_STATIC` is used per axle. For Street that yields FL/FR/RL/RR = 35.0 psi; for Semislicks 28.0 psi.

**CLI resolution precedence** (high → low):
1. `--pressure FL=...,FR=...,RL=...,RR=...` (per-wheel override).
2. `--setup <path>` `pressures_psi` field.
3. Active compound's `PRESSURE_STATIC` per axle (cold default).

Per-wheel-override merges with `--setup` — unmentioned wheels keep setup-file values (or fall through to compound `PRESSURE_STATIC`). `--ambient-temp-c <C>` always overrides. `--compound <name>` always overrides setup-JSON `compound`.

**Logged on stdout:** `Setup: <source> | FL=... FR=... RL=... RR=... | ambient=...°C | compound=<name>`.

### 21.8 Module touchpoints

- **New:** `src/lap_estimator/tyre_state.py` (~400 lines), `src/lap_estimator/setup.py` (~120 lines), `src/lap_estimator/solve_setup.py` (~250 lines).
- **Touched:** `src/lap_estimator/car.py` (+`Compound` dataclass, +multi-compound parsing — §6.14, ~150 lines added), `src/lap_estimator/simulator.py` (+`simulate_stint`, +`StintResult`, **(v1.3)** +`drag_scale` kwarg threading, **(v2.0.1)** +`v_initial` kwarg + lap-loop `v_prev_end` threading), `src/lap_estimator/sim_telemetry.py` (+12 columns), `src/lap_estimator/report.py` (+per-lap printer, +stint-summary CSV), `src/lap_estimator/telemetry.py` (+per-wheel-state channel reads, +`tyreCompound` passthrough), `src/lap_estimator/driver_fit.py` (+`fit_tyre_calibration` step, +compound auto-select), `src/lap_estimator/driver.py` (+`tyre_calibration` load), `lap.py` (+CLI flags + dispatch to stint / solver modes, +`--compound`).
- **Untouched:** `track.py`, `prep/*`, `analysis/*`, `validate.py`, `profile_dynamics.py`.

### 21.9 Acceptance criteria (v2)

See §11.27–§11.36 and §14.12 item 14. **(v1.3)** §11.37–§11.39 added. **(v2.0.1)** §11.47–§11.48 added.

### 21.10 Open questions / v2.1 / v1.4 candidates

- `k_load`, `k_long` as per-car parameters (vs hand-coded defaults).
- Multi-stint with pit-stop heat-soak between stints.
- Wear-rate units in user-friendly form (`wear_pct_per_km_at_ref_load` instead of `k_wear` per-Joule).
- Numerical optimisation (Nelder-Mead, gradient-free) for coupled per-wheel inverse solve, replacing greedy coordinate descent.
- Per-corner `target_wheel` (different wheel limits different corners).
- Calibration loss function weights (`w_T`, `w_W`, `w_P`) as tunables, not hand-defaults.
- Surface `cumulative_slip_energy_J` per wheel in the stint summary for diagnostic.
- **(v1.4) `k_drag` and `k_drag_reduction` calibration from telemetry.** Run a pressure sweep in AC at fixed driver/track/compound, capture lap times at e.g. {22, 26, 29, 33, 37, 40, 44} psi, and fit `k_drag` against the under-IDEAL slope of lap-time-vs-pressure. Same for `k_drag_reduction` on the over-IDEAL side (small effect; expect noisy fit). Extend `fit_driver.py` with a `--fit-drag-knobs` flag that walks a stint at varying pressures and minimises lap-time RMSE.
- **(v1.4) Asymmetric grip on the under-pressure side.** Current `f_pressure_grip(p < IDEAL) = 1.0` is conservative. Reality has a small bonus (larger contact patch → ~2–5% more lateral grip in `[IDEAL - 5, IDEAL]` psi). Hand-default `k_grip_bonus = 0.03` over `(IDEAL - 5 psi, IDEAL)` is a candidate.
- **(v1.4) Per-compound `k_drag` / `k_drag_reduction`.** Real (slicks vs road tyres flex very differently), but unmeasured in v1.3. Field exists on `Compound`; values are constants for v1.3.

### 21.11 Compound-aware tyres.ini parsing (v2 — NEW)

**Motivation.** AC's `tyres.ini` may define multiple compounds. For the BMW M1: compound 0 (`Street`, `ST`) and compound 1 (`Semislicks`, `SM`) with very different `PRESSURE_IDEAL` (42/43 vs 33/34), `PRESSURE_STATIC` (35 vs 28), `PRESSURE_D_GAIN` (0.004 vs 0.0045), and different `WEAR_CURVE` / `[THERMAL_*]` LUTs. The v2 sim currently parses only the un-suffixed sections (compound 0 / Street). Tomas's real lap was on Semislicks at 26 psi cold; scored against Street's `PRESSURE_IDEAL=42`, `f_pressure` collapses to its 0.30 floor and the sim tanks. The fix is to parse all compounds and select the active one from setup / CLI / telemetry.

**`Compound` dataclass** (lives in `car.py`):
```python
@dataclass(frozen=True)
class Compound:
    index: int                          # 0, 1, 2, ...
    name: str                           # "Street", "Semislicks"
    short_name: str                     # "ST", "SM"
    pressure_ideal_front: float         # psi
    pressure_ideal_rear: float
    pressure_static_front: float        # psi (cold)
    pressure_static_rear: float
    pressure_d_gain: float              # scalar (front-rear-shared for simplicity; see note)
    dy0_front: float; dy1_front: float
    dx0_front: float; dx1_front: float
    dy_ref_front: float; dx_ref_front: float
    dy0_rear: float; dy1_rear: float
    dx0_rear: float; dx1_rear: float
    dy_ref_rear: float; dx_ref_rear: float
    wear_curve_front: LUT               # parsed wear LUT
    wear_curve_rear: LUT
    thermal_lut_front: LUT              # PERFORMANCE_CURVE from [THERMAL_FRONT_n]
    thermal_lut_rear: LUT
    # v1.3 — drag-pressure constants (hand-defaults, not parsed from tyres.ini)
    k_drag: float = 0.5
    k_drag_reduction: float = 0.10
```

Note on `pressure_d_gain`: in BMW M1 `tyres.ini` it's shared across `[FRONT]/[REAR]`; if a future car splits it, extend to `pressure_d_gain_front/rear`.

Note on `k_drag` / `k_drag_reduction`: not parsed from `tyres.ini` (AC has no source for them). Set at `Compound` construction with hand-defaults. Compound-agnostic in v1.3 (both Street and Semislicks get 0.5 / 0.10); per-compound calibration is v1.4 (§21.10).

**Section discovery.** `Car.from_dir` scans `tyres.ini` for any of `[FRONT]`, `[FRONT_n]`, `[REAR]`, `[REAR_n]`, `[THERMAL_FRONT]`, `[THERMAL_FRONT_n]`, `[THERMAL_REAR]`, `[THERMAL_REAR_n]` where `n ∈ {1, 2, 3, ...}` (un-suffixed = index 0, suffix `_k` = index `k`). For each `n` discovered across all four section families, build a `Compound(index=n, ...)`. If any of the four families is missing for a given `n`, raise a clear parse error: `tyres.ini compound index <n>: missing section [<FAMILY>_<n>] (got: [FRONT_<n>], [REAR_<n>], ...)`.

**Default compound selection.** If `tyres.ini` has a `[COMPOUND_DEFAULT]` section with `INDEX=<n>`, use that; otherwise `default_compound_index = 0`.

**`Car.find_compound(query: str) -> Compound | None`:** case-insensitive match against `name` and `short_name`. Strips trailing parenthetical short-name (regex `\s*\([^)]+\)\s*$`) so `"Semislicks (SM)"` matches `"Semislicks"`. Returns `None` on miss.

**Compound resolution precedence** (high → low) at sim invocation:
1. `--compound <name>` CLI flag on `lap.py`.
2. Setup-JSON `compound` field.
3. Telemetry's `tyreCompound` (only inside `fit_driver.py`).
4. `car.compounds[car.default_compound_index]`.

On unknown name at level 1 or 2: argparse error / load-time error listing available compound names. On unknown at level 3: warn and fall through to level 4. The resolved compound is logged on stdout: `Compound: <name> (idx <i>) | source: <cli|setup|telemetry|car-default>`.

**Per-compound function plumbing.** `tyre_state.py`'s `update_per_segment`, `update_per_lap`, and `combined_grip_envelope` all take a `Compound` parameter and look up `f_pressure_grip/f_pressure_drag/f_temp/f_wear` off it. `simulate_stint` resolves the active compound once at entry and threads it through. The default `dy0/dy1/dx0/dx1/dy_ref/dx_ref` baseline used for `mu_x`/`mu_y` scaling is also drawn from the active compound (not from un-suffixed defaults).

**Cold-pressure default.** When `setup.pressures_psi` is absent, fall back to the **active compound's `PRESSURE_STATIC`** per axle. `PRESSURE_STATIC` is the cold-pressure target (what a driver dials in pre-session); `PRESSURE_IDEAL` is the hot-grip-peak target (what the tyre should reach after warm-up). Earlier v2 drafts had this inverted — corrected here.

**Calibration knobs stay compound-agnostic in v2.** `tyre_calibration.{k_friction, h, C_thermal, k_wear}` are physical heating/wear constants and apply across compounds. Per-compound knob tables are a v1.3 candidate — flagged in §3 non-goals and in the backlog notes below. The fitter records `tyre_calibration.source.compound` for traceability when fitting, but the knobs themselves are single-valued. **(v1.3)** Same for `k_drag` / `k_drag_reduction` — they live on `Compound` but are compound-agnostic constants in v1.3 (per-compound calibration is v1.4).

**Backward compat.** Cars with a single un-suffixed compound block (no `_1` sections) parse to `car.compounds == [Compound(index=0, name=<from NAME or "default">, ...)]`. Setup JSONs without `compound` and invocations without `--compound` continue to work — they pick `car.default_compound_index == 0`. Any callers of the deprecated direct attributes (`car.pressure_ideal_front`, `car.pressure_d_gain`, etc.) are migrated to go through `car.compounds[idx]`; if a quick-shim is needed, expose properties on `Car` that delegate to `car.compounds[car.default_compound_index]` for one release.

**Acceptance.** §11.34 (parsing both compounds), §11.35 (Semislicks at 26 psi sim runs without grip-floor collapse — now subsumed by v1.3's §11.37), §11.36 (fitter auto-selects compound from telemetry).

**Backlog (v1.3 / v1.4 candidates — not building now):**
- Per-compound calibration knob tables (`k_friction`/`h`/`C_thermal`/`k_wear` differ by compound — different thermal mass, different friction characteristics).
- Compound mid-stint switching (pit stop with fresh tyres on a different compound). Requires a pit-stop event in `simulate_stint`'s lap loop and a state-transfer model (new compound → fresh wear/temp/pressure).
- **(v1.4)** Per-compound `k_drag` / `k_drag_reduction` calibration from telemetry pressure sweeps. See §21.10.

---

## Decisions block (locked in this revision)

1. **Folder convention is locked.** `cars_in/` / `tracks_in/` = user-dropped raw AC (gitignored). `cars_csv/` / `tracks_csv/` = tracked outputs of prep. `drivers/` = tracked driver JSONs. `samples/aclog/` = tracked sample telemetry. **(v2)** `setups/` = tracked setup JSONs. (§6.9)
2. **Repo layout uses a `src/lap_estimator/` package.** (§6.8)
3. **Two simulator CLIs + three pipeline CLIs.** (§14.4 item 12, §15.7)
4. **Corner notation lives in `<track_dir>/<layout_stem>_corners.json`.**
5. **`tracks_config.json` is the in-repo single source of truth for corner-classification thresholds.**
6. **Corner analysis is downstream of prep, not part of it.**
7. **Legacy DuckDB-staging CSVs are dropped.**
8. **Sample AC telemetry log lives in `samples/aclog/`.**
9. **`cars_in/*` and `tracks_in/*` stay gitignored.**
10. **`prep_track.py` does not auto-invoke corner analysis.**
11. **`analysis/corner_analysis.py` is a rewrite.**
12. **MF4 telemetry output is v2 PLANNED.** (§18)
13. **Slip-based physics, per-wheel forces, yaw dynamics are v3 backlog.** (§19) *(v2 amendment: per-wheel tyre **state** — temp/wear/pressure — lands in v2 via §21 using single-grip-envelope post-lap scaling. Per-wheel **forces** stay in v3.)*
14. **(v1.1-A) Driver config format is JSON.**
15. ~~**(v1.1-B) Driver-lag IIR low-pass.**~~ **Superseded by item 22.**
16. **(v1.1-C) Trail-brake + throttle-ramp heuristic.**
17. **(v1.1-D) Two-lap "tiled" simulation default.**
18. **(v1.1-E) Multi-lap fit mandatory.**
19. **(v1.2-F) Driver-profile dynamic signals measured from telemetry.** (v1.2.1 amendment: `driver_tau_s` joins statistic-only set.)
20. **(v1.2-G) Lap-selection rule: 5–10 newest.**
21. **(v1.2-H) Default `--telemetry-dt-ms` = 10.**
22. **(v1.2.1) Driver-lag IIR low-pass removed from sim telemetry emission.** Supersedes item 15.
23. **(v2) Per-wheel tyre state, multi-lap stint sim, inverse PSI solver — option 2.5, not v3.** The existing 3-pass `simulator.py` and single-grip-envelope architecture are retained. Per-wheel state (`temp_C`, `wear_pct`, `pressure_psi` per FL/FR/RL/RR) evolves between laps via slip-energy attribution (§21.3) using a coarse load-transfer model with hand-coded `k_load = 0.30` and `k_long = 0.20`. Four calibration knobs (`k_friction`, `h`, `C_thermal`, `k_wear`) live in the driver JSON under `tyre_calibration` and are fit against measured lake telemetry (`tyreTemp*`, `tyreWear*`, `wheelsPressure*`) by `fit_driver.py` when those channels are present (§13.14). When absent, hand-defaults are used and `measured: false` is flagged. The scalar grip envelope reduction is `0.5 · (min(g_FL, g_FR) + min(g_RL, g_RR))` — a documented simplification that loses oversteer/understeer balance (v3 hook §19.2). CLI gains `--laps N` (default 2 for back-compat with §20; range [1, 50]), `--setup <path>`, `--pressure FL=...`, `--ambient-temp-c`, plus the inverse-solver flags `--solve-pressure-for-wear <pct> --at-lap <N> [--target-wheel min|max|avg|FL|...] [--uniform-pressure]`. Inverse solver is greedy per-wheel bisection over [20, 50] psi seeded by a 6-point grid scan, tolerance ±1% wear or ±0.5 psi. Default target-wheel is `max` (most-worn). Setups directory `setups/` is tracked; resolution precedence `--pressure` > `--setup` > active compound's `PRESSURE_STATIC` from `tyres.ini` (per axle). Telemetry CSV gains 12 trailing per-wheel state columns (always emitted in stint mode; constant when no calibration). Stint summary CSV `<track_stem>_stint_summary.csv` written next to track. Single-lap regression preserved (§11.30). Per-wheel forces, yaw dynamics, Pacejka, slip-based control-loop drivers, tyre puncture, marbles, multi-stint/pit-stop heat soak — all stay in v3 (§19) or out-of-scope. (§21, §11.27–§11.36, §13.14, §14.12 item 14, §7.2 `tyre_calibration`, §7.10 setup JSON schema, §7.11 stint summary CSV, §7.12 telemetry column extension.)
24. **(v2) Compound-aware `tyres.ini` parsing.** `Car` holds `compounds: list[Compound]` (one entry per `[FRONT_n]/[REAR_n]/[THERMAL_FRONT_n]/[THERMAL_REAR_n]` quadruple; un-suffixed = index 0). Each `Compound` carries `name`, `short_name`, per-axle `pressure_ideal`, `pressure_static`, `pressure_d_gain`, wear LUT, thermal LUT, and `dy0/dy1/dx0/dx1/dy_ref/dx_ref`. `default_compound_index` honours `[COMPOUND_DEFAULT]` `INDEX` if present, else 0. Active-compound resolution precedence: `--compound` CLI > setup-JSON `compound` > telemetry `tyreCompound` (fitter only) > `default_compound_index`. Matching is case-insensitive against `name` or `short_name` and strips trailing parenthetical (so `"Semislicks (SM)"` matches). `f_pressure_grip`/`f_pressure_drag`/`f_temp`/`f_wear` and `mu_x`/`mu_y` baselines all key off the active compound. Cold-pressure default falls back to active compound's `PRESSURE_STATIC` per axle (not `PRESSURE_IDEAL`). `tyre_calibration` knobs (`k_friction`/`h`/`C_thermal`/`k_wear`) stay compound-agnostic in v2 — per-compound knob tables and compound mid-stint switching are v1.3 backlog. Driver JSON `tyre_calibration.source.compound` records the compound the fit was performed against. CLI flag `--compound <name>` added to `lap.py` (normal sim + inverse solver modes). (§21.11, §11.34–§11.36, §7.10 setup JSON `compound` semantics, §6.14, §13.14 compound resolution.)
25. **(v1.3) Pressure penalty asymmetric.** Grip penalty active only above `PRESSURE_IDEAL` (over-pressure → smaller contact patch → less grip). Drag penalty active only below `PRESSURE_IDEAL` (under-pressure → rolling resistance + sidewall flex → more drag). Symmetric quadratic from v2 removed as physically incorrect (double-counted the low-PSI loss in the grip term where it belonged in the drag term). The single `f_pressure(p)` is split into two compound-attached functions: `f_pressure_grip(p)` (one-sided quadratic above IDEAL, clamp `[0.30, 1.0]`; `1.0` below IDEAL) and `f_pressure_drag(p)` (linear penalty below IDEAL with hand-default `k_drag = 0.5`; small linear benefit above IDEAL with hand-default `k_drag_reduction = 0.10`; clamp `[0.85, 1.50]`). `combined_grip_envelope` return arity goes from 2-tuple to 3-tuple: `(mu_x_scale, mu_y_scale, drag_scale)`. `drag_scale` is threaded into the 3-pass solver and multiplies the total drag force (aero + rolling resistance) per lap. `k_drag` and `k_drag_reduction` live on `Compound` but are compound-agnostic in v1.3 (hand-defaults; not parsed from `tyres.ini`); per-compound calibration is v1.4. Calibration from telemetry deferred — v1.4 candidate that fits the constants against an AC pressure sweep. Under-pressure grip bonus (real but small) ignored in v1.3 — `f_pressure_grip(p < IDEAL) = 1.0` (no bonus, no penalty); v1.4 candidate. Acceptance §11.37 (low-PSI lap time within ±2 s of IDEAL — fixes the Tomas Semislicks 26 psi regression), §11.38 (over-pressure grip penalty preserved at 44 psi), §11.39 (asymmetric pressure sweep curve). (§21.3, §11.37–§11.39, §6.14, §6.11.)
26. **(v2.0.1) All laps in a stint are continuous in velocity: lap 1 from rest, lap N from v_end of lap N-1.** Closes the standing-start gap between the two-lap fast path (already continuous via single-pass tile) and the multi-lap loop (was standing-each-lap pre-v2.0.1). `simulate(...)` gains a kwarg-only `v_initial: float = 0.0`; when positive, the forward pass starts at that velocity instead of zero. Backward and corner-limit passes are unaffected — the corner-limit cap naturally clamps an over-cap `v_initial` on the first step. `simulate_stint`'s lap loop carries `v_prev_end = lap_result.speeds[-1]` across iterations: lap 1 passes `v_initial=0.0` (standing), laps 2..N pass `v_initial=v_prev_end`. The `n_laps == 2 && !measured` two-lap-tile fast path is unchanged (already produces continuous velocity via a single solver pass over a doubled segment list) — both paths exist because the tile only works when the grip envelope is constant across laps, while the multi-lap loop also handles per-lap state evolution. Universal rule applies to ALL stint-mode runs, n_laps ∈ [1, 50]. Acceptance: §11.30 / §11.31 preserved (single-lap regression and `--laps 2` byte-compat both rely on the `v_initial=0.0` default); new §11.47 (3-lap stint, lap 2 and lap 3 both flying, similar times, ≥2 s faster than lap 1); new §11.48 (5-lap stint lap-2 time matches 2-lap stint lap-2 time within ±0.05 s — same `v_initial` for lap 2 across both paths when no state evolution intervenes). (§21.4, §11.47–§11.48, §6.4, §20 universal-rule note, §10 migration.)
