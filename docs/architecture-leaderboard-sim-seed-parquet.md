# Leaderboard seed parquets for QuixLake `ac_telemetry`

One-paragraph overview
----------------------

This pipeline produces Hive-partitioned parquet files that sidestep the
DCM / Quix-Streams enrichment path and seed the QuixLake `ac_telemetry`
table directly from our local lap simulator. The user uploads the
`.tmp/quix_seed/` tree into the lake's S3 bucket; Iceberg/Hive
auto-discovery picks the partitions up; the leaderboard SQL then sees a
synthetic "Pro / Expert / Tomas / Amateur / Novice / Learner" field on
four Nurburgring layouts with the BMW 1M.

Why this architecture
---------------------

We need leaderboard data quickly and the DCM enrichment pipeline is
blocked on configuration changes outside our control. The lake itself
accepts any well-formed parquet that matches the table schema (186
columns, 8 Hive partitions). Our sim already produces a per-tick trace
CSV; reshaping it into a 186-column parquet is a one-shot ops task.
Trade-offs we accept:

* **Sparse rows.** Only ~10 of the 186 columns are populated from the
  sim: partition keys, `timestamp_ms`, `distanceTraveled`, `speedKmh`,
  `normalizedCarPosition`, lap-progress fields, and `tyreCompound`.
  Everything else is null. That is fine for a leaderboard query that
  only needs `iLastTime` per (driver, experiment).
* **`skill_pct` alone barely moves lap time.** All six driver JSONs
  inherit Tomas's measured calibration (Pacejka, `driver_tau_s`,
  `trail_brake_m`, `throttle_ramp_m`, pedal-press rate). v3-reactive
  doesn't tie `skill_pct` to the controller, so lap deltas between the
  six profiles are ~0.2 s on Sprint A. This is a known limitation; the
  six profiles still provide *distinct* leaderboard rows because the
  partition key differs. If we ever need a real spread, we have to
  perturb the calibration fields (slower brake taper, longer throttle
  ramp, higher tau) rather than just `skill_pct`.
* **Layout encoding through `experiment`.** The lake's `track`
  partition value is `ks_nurburgring` for both Sprint and GP layouts,
  so we discriminate them through `experiment` instead. Each layout
  gets its own experiment name (`LeaderboardSeed_sim__layout_sprint_a`,
  etc.). The leaderboard SQL groups by experiment.
* **URL-encoded session_id on disk.** Windows rejects `:` in directory
  names; the colons in the ISO-8601 session_id are percent-escaped to
  `%3A`. Hive/Iceberg readers tolerate either form. If the upload
  tool requires literal colons, rename at upload time or sync through
  a helper.

Data flow
---------

```
drivers/tomas.json
    |
    | (clone × 6, override `name` and `skill_pct`)
    v
drivers/{pro|expert|tomas|amateur|novice|learner}_sim.json
    |
    | python lap.py cars_csv/bmw_1m
    |    tracks_csv/ks_nurburgring/<layout>_ideal_line.csv
    |    drivers/<driver>.json
    |    --model slip --controller reactive --single-lap --no-plot
    |    --inertia-zz 2400 --chicane-safety-mult 0.85
    v
tracks_csv/ks_nurburgring/<layout>_ideal_line__<DriverName>_sim_trace_slip.csv
    |
    | .tmp/build_seed_parquets.py
    | + .tmp/quix_seed/_lake_schema_ac_telemetry.json (186-col schema)
    v
.tmp/quix_seed/environment=.../test_rig=.../experiment=.../
    driver=.../track=ks_nurburgring/carModel=bmw_1m/
    session_id=2026-05-25T00%3A00%3A00Z/lap=1/data.parquet
```

The schema fetch lives at `GET <QUIXLAKE_URL>/schema?table=ac_telemetry`
(no `DESCRIBE` etc -- the SQL endpoint is read-only-SELECT-only).
The shape of that response is cached at
`.tmp/quix_seed/_lake_schema_ac_telemetry.json` so we don't re-hit the
lake on every run.

File inventory
--------------

Created (this feature only — non-repo, all under `.tmp/` per CLAUDE.md):

* `.tmp/build_seed_parquets.py` — the parquet builder. Reads sim trace
  CSVs, materialises a 186-column pyarrow Table per (driver, layout),
  writes one `data.parquet` per partition.
* `.tmp/quix_seed/_lake_schema_ac_telemetry.json` — cached lake schema.
* `.tmp/quix_seed/MANIFEST.md` — run summary, upload instructions,
  verification SQL.
* `.tmp/quix_seed/_sim_logs/*.{stdout,stderr}.log` — per-run sim
  console output (kept for forensics on the gp_b abort).
* `.tmp/quix_seed/environment=.../...` — 24 partition directories,
  each containing one `data.parquet` (~4.9 MB total).

Created in the repo (committed-eligible):

* `drivers/pro_sim.json`, `drivers/expert_sim.json`,
  `drivers/tomas_sim.json`, `drivers/amateur_sim.json`,
  `drivers/novice_sim.json`, `drivers/learner_sim.json` — six driver
  profiles cloned from `drivers/tomas.json` with `skill_pct` set to
  `{1.00, 0.85, 1.00, 0.60, 0.30, 0.10}` and `name` set to
  `{Pro, Expert, Tomas, Amateur, Novice, Learner}`. The Tomas
  calibration (Pacejka, measured tau / trail / throttle ramp) is
  carried verbatim.
* `docs/architecture-leaderboard-sim-seed-parquet.md` (this file).

No production code paths were modified. `lap.py`, `src/lap_estimator/*`,
and `viz/` are untouched.

Integration points
------------------

* **Input:** `python lap.py ... --model slip --controller reactive
  --single-lap` produces the sim trace CSV with columns
  `lap, distance_m, sim_speed_ms, sim_speed_kmh, ai_speed_kmh,
  time_s`. The builder consumes only `distance_m`, `sim_speed_kmh`,
  `time_s`.
* **Output:** parquet files in the partition layout the lake's
  Iceberg catalog expects. The exact partition spec is
  `(environment, test_rig, experiment, driver, track, carModel,
  session_id, lap)`. Embedding the partition keys as columns inside
  the parquet (in addition to the directory path) makes auto-discovery
  safe across readers.
* **Schema source of truth:** the lake's `/schema` HTTP endpoint. If
  the lake's schema evolves, re-fetch the JSON and re-run the
  builder; no code changes needed because we materialise the table
  generically from the field list.

Lap-progress field semantics (`currentTime / lastTime / bestTime`)
------------------------------------------------------------------

The lake stores these as strings in `M:SS:mmm` format (no zero-pad on
minutes, two-digit seconds, three-digit milliseconds). Sentinel for
"not yet set" is `-:--:---` (matches the in-game UI). We populate:

* `currentTime` -- per-row, `format(timestamp_ms)`.
* `lastTime / bestTime` -- `-:--:---` for every row of an in-progress
  lap; on the **final row of a completed lap** we stamp them with the
  lap's total time. `completedLaps` jumps from 0 to 1 on that same
  row. Aborted laps (`completed=False`) keep `lastTime / bestTime`
  at sentinel for *all* rows so the leaderboard SQL's
  `MIN(iLastTime)` aggregate ignores them.
* The numeric mirrors `iCurrentTime / iLastTime / iBestTime` (int64
  ms) follow the same rule but use NULL instead of `-1` for unset.

Re-running
----------

```powershell
# 1. Refresh the lake schema cache
python -c "import httpx, json, os; ..."   # see .tmp/build_seed_parquets.py

# 2. Re-run all 24 sims (~25 min wall-clock on 3 workers)
python .tmp/run_seed_sims.py    # this script is captured inline in the
                                 # session; reuse the snippet from
                                 # .tmp/quix_seed/_sim_logs/ as needed.

# 3. Rebuild parquets
python .tmp/build_seed_parquets.py
```

Upload
------

```bash
aws s3 cp .tmp/quix_seed/ \
  s3://quixdatalaketest/ac_telemetry/ \
  --recursive \
  --exclude 'MANIFEST.md' --exclude '_lake_schema_*' \
  --exclude '_sim_logs/*' --exclude 'build_seed_parquets*' \
  --acl bucket-owner-full-control
```

After upload, hit
`POST <QUIXLAKE_URL>/tables/ac_telemetry/refresh` (or
`/refresh-manifest`) so the catalog picks up the new partitions.

Verification SQL
----------------

```sql
SELECT driver,
       experiment,
       carModel,
       MIN(iLastTime) AS best_ms
FROM ac_telemetry
WHERE experiment LIKE 'LeaderboardSeed_sim__%'
  AND completedLaps >= 1
GROUP BY driver, experiment, carModel
ORDER BY experiment, best_ms
```

Aborted runs (currently every gp_b combination) are filtered by the
`completedLaps >= 1` predicate.

Known caveats
-------------

1. **gp_b OffTrackError at 337 m.** The Nurburgring GP-B ideal line
   has a corner geometry the reactive controller can't keep on-track
   with the current `chicane-safety-mult=0.85`. Six gp_b parquets are
   written with `completedLaps=0` (246 rows each, ~4.9 s of running).
   If a complete gp_b is needed later, tune the chicane safety or
   regenerate a smoother ideal line.
2. **Schema drift.** The cached schema is a snapshot. If the lake's
   `ac_telemetry` table gets columns added/removed, re-fetch via the
   `/schema` endpoint and re-run.
3. **`skill_pct` is cosmetic for v3-reactive.** As discussed above,
   the six leaderboard rows look near-identical because the
   calibration fields are identical. If a meaningful spread matters
   downstream, perturb tau / trail / throttle ramp on the slower
   drivers.
4. **No body-frame motion / load / tyre-temp columns.** The
   reactive-controller sim trace doesn't expose those; a future
   leaderboard rendering that wants per-tick load or tyre temp would
   need to switch to `--telemetry-dt-ms 20` output (which writes a
   richer `_sim_telemetry.csv`) and extend the builder to read it.
