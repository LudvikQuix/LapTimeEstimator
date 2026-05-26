# Leaderboard seed via Quix Streams + self-published config seed

## Summary

Streams 24 sim runs (6 drivers x 4 layouts) into the `ac_telemetry`
Iceberg/Hive lake using the Quix Streams SDK only. There is no S3 upload, no
DCM REST call, and no parquet build step. The script publishes a tiny config
seed to `ac-telemetry-config` and the raw per-tick telemetry rows to
`ac-telemetry-raw`; the lake's `join_lookup` enrichment writes both into the
correct Hive partition automatically.

This replaces the prior approach (`.tmp/build_seed_parquets.py` + manual
`aws s3 cp`), which remains as the offline fallback if the lake sink
deployment is unhealthy.

## Why this works

`ac-quix-bridge/ac-telemetry-lake/main.py` builds the streaming dataframe as:

```python
sdf = app.dataframe(topic=app.topic(os.environ["input"], key_deserializer="str"))

config_lookup = QuixConfigurationService(
    topic=config_topic, app_config=app.config,
)
sdf = sdf.join_lookup(
    lookup=config_lookup,
    fields={
        "test_id":     ...jsonpath="$.test_id",       type="experiment",
        "environment": ...jsonpath="$.environment",   type="experiment",
        "test_rig":    ...jsonpath="$.test_rig",      type="experiment",
        "experiment":  ...jsonpath="$.experiment_id", type="experiment",
        "driver":      ...jsonpath="$.driver",        type="experiment",
        "carModel":    ...jsonpath="$.carModel",      type="session",
        "track":       ...jsonpath="$.track",         type="session",
    },
)
sdf = sdf.fill(completedLaps=-1)
sdf["lap"] = sdf["completedLaps"] + 1
sdf.sink(blob_sink)
```

`QuixConfigurationService` is a Kafka-backed lookup. It keys configs by the
Kafka message-key on the config topic. Any process that publishes to
`ac-telemetry-config` with the same key produces a valid seed; the DCM REST
form was a convenience layer, not the only path.

The Hive partition columns (per the lake's `app.yaml`) are:

```
environment, test_rig, experiment, driver, track, carModel, session_id, lap
```

`join_lookup` populates the first six. `session_id` and `lap` are NOT in the
lookup spec; they come straight from the message body. `lap` is computed by
the lake as `completedLaps + 1`, so to keep all rows of a single lap inside
the same `lap=1` partition we send `completedLaps=0` for every row.

## Architecture

```
                                        +-----------------------------+
                                        |   stream_sim_to_quix.py     |
                                        |                             |
                  (1) 24 config seeds   |  for each (driver, layout): |
       +--- key=sim_<driver>_<layout> --+    publish 1 config row     |
       |    value={ experiment_id,     |    publish N telemetry rows  |
       |            environment,       |                              |
       |            test_rig,          +-----------------------------+
       |            driver, carModel,                |
       |            track, test_id }                 |
       |                                              |
       v                                              v
+-------------------+                  +----------------------------+
| ac-telemetry-     |                  | ac-telemetry-raw           |
| config (Kafka)    |                  | (Kafka, 1 partition)       |
+-------------------+                  +----------------------------+
       |                                              |
       |  QuixConfigurationService (key-keyed)        |
       +------------+    +----------------------------+
                    v    v
            +---------------------------+
            | sdf.join_lookup(...)      |
            |   adds 6 partition cols   |
            +---------------------------+
                    |
                    v
            +---------------------------+
            | sdf["lap"] = completedLaps+1
            +---------------------------+
                    |
                    v
            +---------------------------+
            | QuixTSDataLakeSink        |
            |   batch -> S3 parquet     |
            |   Hive-partitioned by:    |
            |   environment / test_rig  |
            |   / experiment / driver   |
            |   / track / carModel      |
            |   / session_id / lap      |
            +---------------------------+
                    |
                    v
            +---------------------------+
            | Iceberg catalog (REST)    |
            | ac_telemetry table        |
            +---------------------------+
```

## Key design choices

### Layout goes into `experiment_id`, not `track`

The lake's `track` partition collapses both Sprint and GP layouts to a single
value (`ks_nurburgring`). To keep layouts distinct in the leaderboard we
follow the parquet builder's convention and scope the layout inside the
experiment field: each run's `experiment_id` is
`LeaderboardSeed_sim__<layout>` (e.g. `LeaderboardSeed_sim__layout_sprint_a`).
The Iceberg `experiment` column carries the layout-scoped value as well.

### `completedLaps=0` on every row, including the final row

`sdf["lap"] = sdf["completedLaps"] + 1`. If we stamped `completedLaps=1` on
the final row of the lap (as the parquet builder does), that one row would
land in the `lap=2` partition under the sink-based path. We therefore keep
`completedLaps=0` for every row and signal lap completion only via the
lap-time columns (`iLastTime`, `lastTime`, `bestTime`, `iBestTime`), which
are NULL on every row except the final row of a completed lap.

The leaderboard SQL filter becomes `WHERE iLastTime IS NOT NULL` (or
equivalently `WHERE lap_time_str <> '-:--:---'`), which is the same effective
filter as `WHERE completedLaps >= 1` but partition-clean.

### Aborted laps included verbatim

`layout_gp_b` is unfinishable with the current v3 controller (sim aborts
around ~5 s / 71 m). We still publish those ~246 rows per driver so the
leaderboard UI can render an "ABORTED" placeholder. `iLastTime` stays NULL
for those runs, so the SQL above filters them out automatically.

### Kafka message key is the join key

`QuixConfigurationService` is keyed by Kafka message-key. The config seed
payload contains the partition values (`environment`, `test_rig`, etc.) but
does NOT need to repeat the key. Our chosen key format `sim_<driver>_<layout>`
is purely a discriminator; the lake never reads it back.

### `session_id` lives in the message body

`session_id` is a Hive partition column but is NOT in the lake's `join_lookup`
spec. We therefore include it in every telemetry message body as an ISO-8601
string (`2026-05-25T00:00:00Z`). The lake's sink reads it from the body and
slots it into the partition path automatically.

## File inventory

- `scripts/stream_sim_to_quix.py` (new). Single-file CLI with three modes:
  - default: publish all 24 runs to live Kafka.
  - `--dry-run`: build and log messages without importing `quixstreams`.
  - `--limit N`: only the first N runs (useful for smoke testing one run
    before the full dispatch).
- `requirements.txt` (unchanged). `quixstreams` is already installed in the
  developer environment; no production code depends on it (the script lives
  outside `src/lap_estimator/`), so we did not pin it in requirements. Add
  `quixstreams>=3.20,<4.0` if/when this script becomes part of an automated
  pipeline.

## Integration with neighbouring features

- **Parquet-build fallback** (`.tmp/build_seed_parquets.py`): unchanged. Both
  paths target the same Iceberg table and the same partition shape (apart
  from the `completedLaps` final-row caveat described above). The sim trace
  CSVs (`tracks_csv/ks_nurburgring/<layout>_ideal_line__<Driver>_sim_trace_slip.csv`)
  are the single source of truth shared by both paths.
- **Web UI** (`docs/architecture-web-ui.md`): the leaderboard view queries
  `ac_telemetry` for `experiment LIKE 'LeaderboardSeed_sim%'` and per-driver
  `MIN(iLastTime)`. No change required.
- **v3 slip simulator**: untouched. We consume its existing sim-trace CSV
  outputs only.

## Operational notes

### How to dispatch

```bash
# Smoke test (one run, no live publish):
python scripts/stream_sim_to_quix.py --dry-run --limit 1

# Smoke test (one run, real publish):
python scripts/stream_sim_to_quix.py --limit 1

# Full dispatch (24 runs, ~127K rows):
python scripts/stream_sim_to_quix.py
```

### Verification SQL

```sql
SELECT experiment,
       driver,
       MIN(iLastTime) AS best_ms,
       COUNT(*)       AS rows
FROM ac_telemetry
WHERE experiment LIKE 'LeaderboardSeed_sim%'
  AND iLastTime IS NOT NULL
GROUP BY experiment, driver
ORDER BY experiment, best_ms;
```

### Known dependencies on lake-side health

The script relies on the `quixdev-acquixbridge-testmanager` workspace's lake
sink (deployment of `ac-quix-bridge/ac-telemetry-lake`) being healthy and
consuming `ac-telemetry-raw`. If the sink is scaled to zero or stuck, our
messages land in Kafka but never reach S3. There is no producer-side signal
for this; verify by polling `/tables/ac_telemetry/refresh` and watching
`file_count` move.

If the lake sink is down, fall back to
`python .tmp/build_seed_parquets.py` + the `aws s3 cp` step documented in
its MANIFEST. The catalog auto-discovery is the same for either path.

### Stray-parquet hazard

The `ac_telemetry` table in the dev catalog has a partition-mismatched
`data.parquet` at the table root (left over from an unrelated debug session).
Queries that scan all partitions return:

```
Binder Error: Hive partition mismatch between file ".../ac_telemetry/data.parquet"
and ".../ac_telemetry/environment=.../lap=1/data_*.parquet"
```

Queries that filter on at least one partition column (e.g. `WHERE driver = ...`
or `WHERE experiment LIKE '...'`) silently skip the broken file because the
catalog's partition predicate excludes it. The leaderboard SQL above always
filters on `experiment`, so it is unaffected. The stray file should be
removed from S3 when convenient.
