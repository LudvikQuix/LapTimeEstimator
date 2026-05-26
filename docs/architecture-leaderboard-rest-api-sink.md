# Leaderboard parquet upload via QuixLake REST API

## One-paragraph overview

This document captures the REST-API procedure that lands locally-built
Hive-partitioned parquet files into the QuixLake `ac_telemetry` table
without touching the S3 bucket directly. The neighbouring doc
`architecture-leaderboard-sim-seed-parquet.md` describes *how* the 24
seed parquets are produced; this doc describes *how* they are
delivered. The pipeline is: POST `/files/upload` for each parquet, POST
`/refresh-manifest` once, then verify with POST `/query`. No S3
credentials are required — the lake service signs S3 itself.

## Why this architecture

The `.env` ships S3 keys for `quixdatalaketest`, but the actual catalog
target lives in a sibling bucket (`quixdevbucket`) we do not have keys
for. The lake's own `/files/upload` endpoint proxies the write through
its service identity, so it is the only viable path from this
workstation. The REST API also exposes the manifest-refresh and SQL
endpoints we need for end-to-end verification, so the entire upload +
discover + query cycle is one HTTP surface.

## Lake REST endpoints used

Base URL: `https://quixlake-quixdev-quixlakev2-dev.deployments-dev.quix.io`

Auth: `Authorization: Bearer ${QUIX_LAKE_TOKEN}` (token in `.env`).

| Method | Path | Purpose |
|---|---|---|
| POST | `/files/upload?table=<t>&path=<hive-path>` | Multipart upload, field name **`files`** (plural). 200 = stored. |
| POST | `/refresh-manifest?table=<t>` | Re-scans S3 under the table prefix and re-registers files. Synchronous. |
| POST | `/tables/{t}/refresh` | Equivalent; supports `async=true`. We call both for belt-and-braces. |
| POST | `/query` | Body: SQL as `text/plain`. Returns CSV (default) or JSON with `?explain=true`. |
| GET | `/files/list?table=<t>` | Top-level catalog listing — handy to confirm rogue root-level files. |
| GET | `/partitions?table=<t>` | Tree view; surfaces `environment=__None__` for unpartitioned root files. |
| DELETE | `/delete?table=<t>&mode=partitions&partitions=<p>` | Drops a partition; used to clean orphans (operator-only). |

The OpenAPI spec is fetchable at `/openapi.json` (no auth required).

## Partition path convention

The deployed lake stores `session_id` with **literal `:`** in the S3
key (verified against existing g29 data:
`session_id=2026-04-15T10:02:25.041Z`). The local Hive tree on Windows
URL-encodes the colon as `%3A` because NTFS forbids `:` in file names.
The upload script decodes `%3A` back to `:` before sending the `path`
query parameter — otherwise the lake's Hive auto-discovery would see
two different `session_id` values for the same logical session.

Partition order (matches `_lake_schema_ac_telemetry.json`):

```
environment / test_rig / experiment / driver / track / carModel / session_id / lap
```

## Data flow

```
local file        derive partition path       POST /files/upload
.tmp/quix_seed/*  ─── strip 'k=v' parents ──> ?table=ac_telemetry      ┐
data.parquet      (decode %3A -> :)           &path=<hive>             │
                                              multipart files=...      │
                                                                       ▼
                                              lake service writes to S3
                                              quixdevbucket/.../ac_telemetry/<hive>/data.parquet
                                                                       │
                                              (loop 24x)               │
                                                                       ▼
                                              POST /refresh-manifest
                                              ?table=ac_telemetry
                                              => file_count grows by N
                                                                       │
                                                                       ▼
                                              POST /query
                                              (SQL in text/plain body)
                                              => CSV result
```

## File inventory

Created:

* `docs/architecture-leaderboard-rest-api-sink.md` (this file).
* `.tmp/lake_upload.py` — one-shot upload + refresh + verify script.
  Reusable for re-seeding. Reads `.env` directly.
* `.tmp/lake_probe.py` — read-only health/schema probe.
* `.tmp/lake_list.py` — partition listing utility.
* `.tmp/lake_verify.py` — verification via `read_parquet()` glob
  (workaround for the orphan unpartitioned file at the table root —
  see Caveats).

Unchanged:

* All production code under `src/lap_estimator/` and `viz/` — this
  task is pure ops.

## Integration with neighbouring features

* **Upstream:** `architecture-leaderboard-sim-seed-parquet.md` describes
  how `build_seed_parquets.py` produces the 24-file tree under
  `.tmp/quix_seed/`. This doc starts from that tree.
* **Downstream:** The frontend leaderboard (web UI work in
  `architecture-web-ui.md`) queries `ac_telemetry` via the lake's
  `/query` endpoint. With these 24 partitions live, the leaderboard
  shows six sim drivers across three completed Nurburgring layouts.
* **Sibling:** `architecture-leaderboard-stream-via-config-seed.md`
  describes the streaming alternative that was blocked on DCM
  configuration; this REST sink is the unblocked path.

## Caveats

1. **Orphan root file blocks `FROM ac_telemetry`.** A pre-existing
   unpartitioned `data.parquet` (320 kB, ~0.3 MB) sits at
   `s3://quixdevbucket/.../ac_telemetry/data.parquet` and registers as
   the partition `environment=(null)` (URL-safe form
   `environment=__None__`). DuckDB's Hive binder refuses to mix
   partitioned and unpartitioned files in the same `read_parquet`
   call, so every `SELECT ... FROM ac_telemetry` returns a Binder
   Error until the orphan is removed. Verification therefore uses
   explicit `read_parquet('s3://.../environment=*/.../*.parquet',
   hive_partitioning=1)`. Once the orphan is cleared via
   `DELETE /delete?table=ac_telemetry&mode=partitions&partitions=environment=__None__`
   followed by a manifest refresh, the simple `FROM ac_telemetry`
   form will work and downstream queries do not need the workaround.
2. **Multipart field name.** The OpenAPI parameter docs do not name
   the multipart field. The server expects **`files`** (plural),
   confirmed by 400 error body `No files provided. Use multipart
   form with 'files' field.` after a `file` attempt.
3. **Token expiry.** `QUIX_LAKE_TOKEN` in `.env` expires in May 2026
   (~8 weeks of headroom from upload date). Re-seeding past that
   point requires a fresh token from the Quix portal.
4. **Idempotency.** Re-running the upload script overwrites the
   destination object in-place (same key), so reruns are safe; the
   `file_count` in the refresh response will not grow.

## Verification queries

Leaderboard (best lap per driver+layout, only completed laps):

```sql
SELECT experiment, driver, MIN(iLastTime) AS best_ms, COUNT(*) AS n
FROM read_parquet(
  's3://quixdevbucket/quixdev-acquixbridge-testmanager/data-lake/time-series/'
  'ac_telemetry/environment=*/test_rig=*/experiment=LeaderboardSeed_sim__*/**/*.parquet',
  hive_partitioning=1
)
WHERE completedLaps >= 1
GROUP BY experiment, driver
ORDER BY experiment, driver
```

Expected: 18 rows (6 drivers x 3 completed layouts). gp_b is absent
because all six gp_b runs aborted (`completedLaps = 0`).

Per-partition row count (sanity check against `MANIFEST.md`):

```sql
SELECT experiment, driver, COUNT(*) AS rows
FROM read_parquet('...glob...', hive_partitioning=1)
GROUP BY experiment, driver
ORDER BY experiment, driver
```

Expected total: 126,899 rows across all 24 partitions.
