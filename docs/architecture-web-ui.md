# Architecture: web UI (`lap-estimator-web`)

Spec: `dev-planning/lap-simulation-csv-driver/spec-section-22-web-ui.md`
(spec §22, v3-aux). Sibling reference:
`ac-quix-bridge/telemetry-comparison/{main.py, partition_walker.py,
partition_filter.py, static/}`. Prior docs:
`architecture-config-pipeline.md`, `architecture-lap-simulation-stint-v2.md`.

---

## What it does

`web/` ships a FastAPI single-page web UI that wraps every CLI surface
(`lap.py`, `fit_driver.py`, `solve_setup.py`, `validate.py`) plus the
QuixLake data lake into a five-tabbed app. It is deployable as a Quix
Cloud service alongside `ac-quix-bridge/telemetry-comparison` and shares
its env conventions (`QUIXLAKE_URL`, `QUIX_LAKE_TOKEN`). All compute runs
in-process via direct `import` from `src.lap_estimator.*` wrapped in
`asyncio.to_thread(...)`; nothing is spawned as a subprocess.

The five tabs are: Sim, Driver (view + fit), Stint Solver, Validate, Lake.
The first four call the matching POST endpoint and render the in-memory
artefacts (`text/csv` streams) through Plotly.js. The Lake tab walks the
QuixLake partition tree to surface (driver, car, track, session, lap)
tuples the user has actually driven.

---

## Why this architecture

**FastAPI + Tailwind CDN + Plotly CDN + vanilla ES modules, no Node.**
Mirrors the proven `telemetry-comparison` shape exactly. Zero build step,
zero npm dependency, zero JS toolchain. The trade-off is a slightly less
ergonomic frontend than a React app, but the Sim/Driver/Stint pages are
form-and-table heavy with one chart per tab, and the cost of a JS
framework would have dwarfed the value for this audience size.

**In-process import, not subprocess.** A `python -m lap.py ...` subprocess
spawn would have cost ~500 ms of interpreter startup per request. With
in-process invocation a Sim run is <1 s (cold) and ~200 ms warm. The
trade-off is that a runaway `simulate_stint` blocks one worker thread until
it returns; this is acceptable because sim runtimes are bounded (~5 s) and
the workspace is single-user.

**SSE for `/api/fit_driver` only.** Fitter runs take 10-60 s and need
stage-by-stage progress to feel responsive. All other endpoints are
synchronous JSON. The SSE pump uses a worker thread + `queue.Queue` rather
than `asyncio.Queue` because the underlying `fit_driver_pipeline` is a
plain Python generator (it has to be — it calls SciPy/NumPy synchronously)
and you can't iterate a sync generator from an async coroutine without
that thread + queue bridge.

**Per-request Arrow/CSV content negotiation, no startup probe.** The
revised lake server supports Arrow on demand; spec §22.C originally called
for a startup probe + cached transport flag, but the negotiated header
approach is simpler and lets the lake itself decide per-query. The
`Accept: application/vnd.apache.arrow.stream, text/csv;q=0.5` header on
every `POST /query` covers both fast path (Arrow stream into pyarrow) and
fallback (CSV via `pd.read_csv`). `LAKE_ARROW_FORCE=0|1` overrides for
ops debugging. This deviates from the spec's "probe at startup, cache" —
see "Deviations from spec" below.

**Lake transport split between library and web layer.** `web/lake_client.py`
holds the async-httpx client used by `/api/lake/*` routes. The library-side
`src/lap_estimator/lake_loader.py` does its own synchronous httpx call so
the `fit_driver.py --from-lake` CLI works without spinning up the web
service. Both consult the same env vars and use the same content-negotiation
header set. The duplication is ~30 LoC and explicitly accepted to keep the
library independent of the web package.

**Refresh-from-lake overwrites `drivers/<name>.json` in place.** No staging
directory, no merge tooling. The container's filesystem is ephemeral, so
the user is told to pull the file locally and commit it. The UI banner on
the Driver tab calls this out after every successful fit.

---

## Data flow

### Sim path

```
Browser sim.js
  POST /api/sim                                (JSON body)
    -> web.sim_routes.post_sim
    -> web.sim_runner.run_sim (asyncio.to_thread)
       -> Car(_car_path) + Track.from_csv + Driver.load
       -> resolve_compound, resolve_setup
       -> lap_estimator.simulator.simulate_stint
       -> _stint_to_telemetry_df (via sim_telemetry.write_synthetic_log
                                  -> tempfile -> pd.read_csv)
       -> _stint_to_trace_df
       -> _stint_to_summary_df
       -> jobs.new_job("sim") stores DataFrames keyed by UUID
    <- {job_id, lap_times_s, ...csv_urls}
  GET /api/sim/result/{job_id}/telemetry.csv   (Plotly fetches CSV)
    -> jobs.require(job_id).telemetry_df -> StreamingResponse(text/csv)
```

The telemetry round-trip through a tempfile is intentional: `sim_telemetry.
write_synthetic_log` already handles the 12 v2 per-wheel state columns,
the `lap` column, and the gas/brake reconstruction. Reimplementing it as a
pandas-direct emitter would have duplicated ~200 LoC of the simulator's
output contract. The temp file is unlinked immediately after read.

### Fit-from-lake path (SSE)

```
Browser driver.js
  POST /api/fit_driver  (Accept: text/event-stream)
    -> web.fit_routes.post_fit_driver (StreamingResponse)
       -> web.sim_runner.run_fit_driver (async generator)
          -> spawn worker thread
             -> Car/Track/Driver setup
             -> lap_estimator.driver_fit.fit_driver_pipeline (sync gen)
                -> [lake mode]   load_laps_from_lake
                                   (HTTP POST /query, Arrow or CSV decode)
                                   yield lake_query_started, lake_query_done
                -> [local mode]  read_ac_log + merge_with_track per file
                                   yield merged_done
                -> fit_driver (library) -> FitResult
                                   yield skill_fit_done
                                   yield tyre_calibration_done
                -> simulate(...) for sim_lap_time
                                   yield validation_done
                -> json.dump payload
                                   yield written
          <- main thread pumps stages onto queue.Queue
       <- async generator yields ("stage", payload), ("written", ...), ("done", ...)
    SSE: event: stage / data: {...}
    SSE: ... event: written / data: {...path...}
    SSE: event: done
```

Heartbeats (`: keep-alive`) are emitted every 30 s so a Quix Cloud ingress
or nginx reverse-proxy doesn't kill the idle TCP connection. The client
ignores them (they're SSE comments).

### Lake tree path

```
Browser lake.js
  GET /api/lake/tree
    -> web.lake_routes.get_tree
    -> (cache 60 s unless ?force=1)
    -> partition_walker._walk_partition_tree (recursive, asyncio.gather
                                              fan-out per level)
       -> 7 levels of S3 LIST against QuixLake's /partitions endpoint
    -> _shape_tree: flat session dicts -> nested driver/car/track/sessions
    -> lake_client.query_with_transport("SELECT 1") for the transport tag
    <- {drivers: [...], transport: "arrow"|"csv", cache: "miss"}
```

`fitted_locally` flag is computed by intersecting lake-driver names with
the lowercase stems of `drivers/*.json`. Stem-match also catches
case-insensitive variants and bidirectional `tomas` ↔ `tomas_full` cases.

---

## File inventory

### New files (web service, ~1100 LoC Python + ~600 LoC JS)

| Path | Lines (approx) | Purpose |
|------|---------------:|---------|
| `web/__init__.py` | 5 | Package marker. |
| `web/config.py` | 60 | Env-var loader + repo-root resolution. |
| `web/lake_client.py` | 100 | Per-request Arrow/CSV `/query` dispatch + health probe. |
| `web/partition_walker.py` | 130 | VENDORED from telemetry-comparison; one import swap. |
| `web/partition_filter.py` | 50 | VENDORED from telemetry-comparison; verbatim. |
| `web/jobs.py` | 70 | In-memory job dict with 60-min TTL eviction. |
| `web/sim_runner.py` | 340 | `asyncio.to_thread` wrappers for sim/solve/validate + SSE pump for fit_driver. |
| `web/sim_routes.py` | 110 | `/api/sim`, `/api/solve`, `/api/validate`, artefact streams. |
| `web/fit_routes.py` | 90 | `/api/fit_driver` SSE route. |
| `web/lake_routes.py` | 180 | `/api/lake/tree`, `/api/lake/laps`. |
| `web/main.py` | 240 | FastAPI app + cars/tracks/drivers enumeration + SPA shell. |
| `web/templates/index.html` | 40 | Jinja-rendered SPA shell; loads Tailwind + Plotly CDNs. |
| `web/static/styles.css` | 45 | Minimal Tailwind overrides. |
| `web/static/modules/common.js` | 165 | Shared fetch/SSE/tab helpers. |
| `web/static/modules/sim.js` | 200 | Sim tab + Plotly stacked subplots. |
| `web/static/modules/driver.js` | 180 | Driver view + fit-new-driver SSE consumer. |
| `web/static/modules/stint.js` | 110 | Stint solver tab. |
| `web/static/modules/validate.js` | 110 | Validate tab + per-bin overlay. |
| `web/static/modules/lake.js` | 70 | Lake tree drill-down. |
| `web/requirements.txt` | 7 | Pinned web-only deps. |
| `web/Dockerfile` | 20 | Python 3.11-slim base, port 8080. |
| `web/app.yaml` | 30 | Quix Cloud deployment manifest. |

### Library additions

| Path | Lines | Purpose |
|------|------:|---------|
| `src/lap_estimator/lake_loader.py` | 220 | `load_laps_from_lake` synchronous lake transport with Arrow fast path and CSV fallback. Lazy-imports `httpx` + `pyarrow`. |

### Library modifications

| Path | Change | Why |
|------|--------|-----|
| `src/lap_estimator/driver_fit.py` | Added `FitStage` dataclass + `fit_driver_pipeline(...)` generator (~250 LoC append). The existing `fit_driver(...)` single-step function is unchanged. | The new generator is the SSE event source and the home of the fit-orchestration that used to live only in `fit_driver.py` CLI. The CLI's `_run_from_lake` consumes the same generator for byte-identical behaviour between CLI and SSE paths. |
| `fit_driver.py` | Added `--from-lake driver=...,car=...,track=...,n=N` flag; new `_run_from_lake` helper consumes `fit_driver_pipeline` and prints stage names. The legacy CSV / `--laps-glob` path is unchanged. | Spec §22.B.4 — CLI parity with the UI's lake mode. |

### Non-changes (byte-equivalent preserved)

`simulator.py`, `solve_setup.py`, `validate.py`, `car.py`, `track.py`,
`driver.py`, `setup.py`, `sim_telemetry.py`, `report.py`, `telemetry.py`,
`profile_dynamics.py`, `tyre_state.py`, `lap.py` CLI. Acceptance
§11.1–§11.39 untouched.

---

## Integration points

- **Driver-JSON shape (§7.2)** stays exactly the same. The web fit path
  writes the same payload the CLI fit path does, via shared
  `_build_payload` in `driver_fit.py`.
- **Compound resolution (§21.11)** uses `setup.resolve_compound(car, cli_name=...)`
  unchanged. The web layer maps the `compound` request field into
  `cli_name`, so the precedence stays: web → CLI → setup → telemetry → car-default.
- **Pressure resolution (§21.7)** uses `setup.resolve_setup(...)`. The web
  layer translates `{FL,FR,RL,RR}` JSON into the legacy `"FL=33,FR=33,..."`
  string the CLI accepts. Same defaults, same overrides.
- **Tracks/cars enumeration** is filesystem-based against `cars_csv/` and
  `tracks_csv/`. Sim-output files (the `layout_*__*` siblings) are excluded
  from the tracks list by the `"__" in name` check in `_list_tracks`.
- **Driver enumeration** reads `drivers/*.json`, surfaces `skill_pct`,
  `fit_version`, and `tyre_calibration.source.compound` in the summary.

## Deviations from spec

1. **No Arrow startup probe.** Per the prompt update, the lake server now
   supports Arrow per-request, so `web/lake_client.py` does
   content-type dispatch on every call rather than a one-shot probe.
   `/api/health` derives `lake: arrow|csv|unreachable` from a tiny
   `SELECT 1` at probe time. `LAKE_ARROW_FORCE=0|1` overrides remain.
2. **`fit_driver(...)` in the library is unchanged** — it still returns
   `FitResult`. The spec text suggested converting it to a generator, but
   the natural seam was to add a higher-level `fit_driver_pipeline(...)`
   generator that *calls* `fit_driver(...)` and yields stages around it.
   This keeps the library's typed FitResult contract intact and avoids
   touching `_distances_overlap` checks and CLI-only helpers. CLI consumers
   of `driver_fit.fit_driver` (e.g. tests) are unaffected.

## How to extend

- **New tab.** Add a new `<button data-tab="newtab">` to `index.html`,
  a new `<section id="tab-newtab">` panel, and a new
  `static/modules/newtab.js` that listens on the `tab-shown` event for
  its own name. Mirror the boot-once pattern from `sim.js`.
- **New endpoint.** Add the handler to whichever router fits semantically;
  if it's a long-running compute, wrap it via `asyncio.to_thread`. The
  in-memory `jobs.py` dict is fine for any new artefact stream up to ~50 MB.
- **New lake table or column.** Update `lake_loader._TELEM_COLUMNS` and
  re-run a smoke fit against the lake. The Arrow path doesn't require any
  schema config; pandas materialisation is automatic.

---

## Acceptance criteria status (§11.40–§11.46)

| § | Criterion | Status |
|---|-----------|--------|
| §11.40 | `/api/cars`, `/api/tracks`, `/api/drivers` enumerate correctly. | Verified locally. `/api/cars` returns `[{name:"bmw_1m", compounds:[Street, Semislicks]}]`; `/api/tracks` returns one entry with the 4 committed `ks_nurburgring/layout_*` layouts (and their `_ideal_line` siblings); `/api/drivers` returns the 10 committed JSONs. |
| §11.41 | `POST /api/sim` Tomas/Sprint A Semislicks lap-2 within ±0.05s of CLI. | Verified: API 107.548 s vs CLI 107.548 s (exact match). |
| §11.42 | SSE fit emits required stages, writes driver JSON loadable via `Driver.load`. | Verified end-to-end against local CSV samples. Lake mode wired identically (same `fit_driver_pipeline`) but not exercised against a live lake — see "Deviations" / verification notes. |
| §11.43 | `/api/lake/tree` parity with telemetry-comparison `/api/sessions`. | Wired via the vendored `partition_walker`. Not exercised without live lake creds. |
| §11.44 | SPA navigation: 5 tabs without reload, Tailwind classes apply. | Verified by serving the SPA shell and probing each tab module endpoint. |
| §11.45 | Arrow probe via `/api/health`. | Implemented per the revised per-request dispatch (not startup probe). `/api/health` reports `arrow`/`csv`/`unreachable`. |
| §11.46 | `fit_driver.py --from-lake` parity with SSE flow. | Both consume the same `fit_driver_pipeline` generator, so identical inputs produce byte-identical JSON output. Not exercised end-to-end without live lake creds. |
