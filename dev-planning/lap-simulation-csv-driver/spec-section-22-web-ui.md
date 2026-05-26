# Spec §22 — Web UI (v3-aux)

**Parent spec:** `dev-planning/lap-simulation-csv-driver/spec.md`
**Status:** Draft (v3-aux, additive — does not modify v2/v1.3 simulator behaviour)
**Created:** 2026-05-15
**Planned with:** Buddy

This file is an additive section of the main spec. It should be appended verbatim
at the end of `spec.md` (after §21.11 backlog, before the "Decisions block"), and
the §6.1 dependency addendum below should be merged into §6 of the main spec.

---

## §6.1.dep (addendum to §6 sub-features) — Web UI dependency surface

The v3-aux Web UI (§22) introduces a new module dependency surface that does **not**
belong to the core `lap_estimator` library — it lives in a new top-level `web/`
directory. Library code stays Python-stdlib + NumPy + (optional) Matplotlib +
SciPy (for the v2 calibration Nelder-Mead). The web service adds:

| Package | Version pin | Used by | Why |
|---|---|---|---|
| `fastapi` | `>=0.110,<1.0` | `web/main.py` | ASGI HTTP framework; mirrors `telemetry-comparison`. |
| `uvicorn[standard]` | `>=0.27,<1.0` | `web/main.py` | ASGI server; `[standard]` brings `uvloop`/`httptools`. |
| `httpx` | `>=0.27,<1.0` | `web/lake_client.py` | Async HTTP client for QuixLake `/query` and `/partitions`. |
| `pyarrow` | `>=15.0,<20.0` | `web/lake_client.py` | Arrow-stream decode of `/query` Arrow responses (preferred fast path). |
| `pandas` | `>=2.0,<3.0` | `web/lake_client.py`, `src/lap_estimator/lake_loader.py` | DataFrame interchange with `fit_driver` and `telemetry.py`. |
| `jinja2` | `>=3.1,<4.0` | `web/main.py` | Renders the SPA shell template `web/templates/index.html`. |

These pins live in `web/requirements.txt`, NOT in the repo-root requirements
(library code stays leaner). The `lake_loader.py` shim in §22.B.3 below is the
only library-side module that takes a soft dependency on `pyarrow` + `pandas`;
it imports lazily and degrades to CSV-via-`pd.read_csv` when Arrow is unavailable.
`pandas` was already an effective dependency of `fit_driver` via the merged
telemetry frame, so this is not a new library-side cost.

Tailwind, Plotly.js, and any icons are loaded via CDN from
`web/static/index.html` — **no Node, npm, or front-end build step.** Mirrors
the `ac-quix-bridge/telemetry-comparison` pattern exactly.

---

## §22 Web UI (v3-aux)

### 22.1 Goal

Replace the CLI-only workflow with a single-page web UI that wraps the existing
`lap.py` / `fit_driver.py` / `solve_setup.py` / `validate.py` surfaces and the
QuixLake data lake into one tabbed interface. The UI is **additive** — the CLIs
keep working unchanged. The UI is deployable as a Quix Cloud service alongside
the existing `ac-quix-bridge/telemetry-comparison` and shares its auth/env
conventions.

The UI is named `lap-estimator-web` for Quix Cloud purposes and lives in
`web/` at the repo root. Sized at ~1500 LoC total across all Python + JS files.

### 22.2 Non-goals

- User authentication on the UI (workspace-only access, identical to
  `telemetry-comparison`).
- Live Quix Streams ingestion (the UI consumes only static lake parquet/CSV).
- Pit-stop / mid-stint compound switching (v3 backlog; §21.11).
- Mobile-first responsive design (Tailwind's default breakpoints are enough;
  desktop is the target form factor).
- MF4 telemetry export from the UI (§18 backlog).
- Editing track CSVs, car ini files, setup JSONs, or tracks_config.json via UI.
- Authoring new driver JSONs from scratch (use the "Fit new driver" path; no
  hand-authored JSON form in v3-aux).
- Persisting written `drivers/<name>.json` across container restarts — see §22.5
  for the deploy-shape caveat.

### 22.3 Locked architectural choices

1. **Option 1** — standalone **FastAPI + Tailwind + Plotly.js + vanilla ES
   modules**. No React. No Node build step. New `web/` directory. New Quix
   Cloud service.
2. **In-process invocation** of the simulator / solver / fitter via direct
   `import` from `src.lap_estimator.*`, wrapped in `asyncio.to_thread(...)`. No
   subprocess spawn, no separate worker. Imports the same functions the CLIs
   import.
3. **Refresh-from-lake output policy:** the "Fit new driver" path overwrites
   `drivers/<name>.json` in place. No side directory, no staging. The user is
   responsible for `git add` / `git commit` / `git push` afterwards.
4. **Auth model:** mirror `ac-quix-bridge/telemetry-comparison` exactly. Same
   env vars `QUIX_LAKE_TOKEN` and `QUIXLAKE_URL`. No new auth code.
5. **Charts:** Plotly.js via CDN. Five stacked subplots for sim telemetry
   (speed / gas / brake / temp / pressure). For validation, overlay real vs
   sim on speed + gas + brake with a per-bin delta panel.
6. **Lake transport:** Apache Arrow over `/query` when the server supports it
   (`Accept: application/vnd.apache.arrow.stream`); fall back to CSV. Probe at
   startup, cache the result, log the chosen path.

### 22.4 Lake-as-source-of-truth (locked answer #3 implications)

The lake is a **growing** dataset: Tomas, Ludvik, and Daniel run new sessions
on different cars and tracks over time. The UI must NOT assume that every
(driver, car, track) tuple has a local `drivers/<name>.json`. Concretely:

- The **Driver** tab supports two modes: load an existing fitted driver JSON,
  OR fit a new driver on demand from the lake.
- The **Sim** tab's driver picker shows two sections:
  1. Locally fitted drivers (from `drivers/*.json`).
  2. Lake-available-but-not-yet-fitted drivers, with a "Fit then run" shortcut
     that chains the two actions.
- The **Lake** tab is the canonical listing of (driver, car, track, session,
  lap) tuples that have actually been driven, derived from the QuixLake
  partition tree via the `partition_walker.py` pattern.

The car list (`/api/cars`) and track list (`/api/tracks`) are local filesystem
enumerations (the lake doesn't have car data files; it has telemetry). A new
car/track requires the user to run `prep/prep_car.py` / `prep/prep_track.py`
first — this is intentional and documented in the UI's empty states.

### 22.5 Deploy shape (Quix Cloud)

The UI ships as a containerised FastAPI service. Shape mirrors
`ac-quix-bridge/telemetry-comparison`:

- Container exposes port **8080**.
- `web/Dockerfile` — Python 3.11-slim base; `pip install -r requirements.txt`;
  `COPY . /app`; `CMD uvicorn main:app --host 0.0.0.0 --port 8080`.
- `web/app.yaml` — Quix Cloud deployment config: name `lap-estimator-web`,
  resource hints (1 CPU, 1 GB RAM is sufficient — sim+solve are <5 s, fit is
  10–60 s and runs in a thread).
- Env vars consumed at startup:
  | Var | Required | Default | Purpose |
  |---|---|---|---|
  | `QUIXLAKE_URL` | yes | — | QuixLake base URL, e.g. `https://lake.<env>.quix.io`. |
  | `QUIX_LAKE_TOKEN` | yes | — | Bearer token, sent as `Authorization: Bearer <token>`. |
  | `LAP_ESTIMATOR_REPO_ROOT` | no | `/app` | Filesystem root for `cars_csv/`, `tracks_csv/`, `drivers/`, `setups/`. Override for local dev. |
  | `LOGLEVEL` | no | `INFO` | Standard Python `logging` level. |
  | `LAKE_ARROW_FORCE` | no | unset | `1`=force Arrow, `0`=force CSV, unset=auto-probe at startup. |

- Filesystem persistence caveat: `drivers/<name>.json` writes from the "Fit
  new driver" action are persisted **only inside the running container**. They
  are lost on redeploy / restart. The user re-pushes from local dev when they
  want a fit to survive. This is consciously accepted; the UI surfaces it in
  the success banner ("Saved to container. Pull the file locally and commit to
  persist.").

### 22.6 Module / file layout

Target total: ~1500 LoC. Layout:

```
web/
├── main.py                  # FastAPI app + all /api routes (~450 LoC)
├── lake_client.py           # Arrow-or-CSV /query wrapper (~150 LoC)
├── partition_walker.py      # COPIED from telemetry-comparison (~80 LoC, vendored)
├── partition_filter.py      # COPIED from telemetry-comparison (~50 LoC, vendored)
├── config.py                # Env-var loader (mirrors sister service) (~40 LoC)
├── sim_runner.py            # asyncio.to_thread wrappers around lap_estimator (~150 LoC)
├── jobs.py                  # In-memory job store + SSE event queues (~100 LoC)
├── requirements.txt         # see §6.1.dep
├── Dockerfile
├── app.yaml                 # Quix Cloud deployment config
├── templates/
│   └── index.html           # Jinja shell — loads Tailwind CDN + module entrypoint
└── static/
    ├── styles.css           # Minimal overrides on top of Tailwind (~50 LoC)
    └── modules/
        ├── common.js        # fetch helpers, SSE helper, tab switcher (~120 LoC)
        ├── sim.js           # Sim tab logic + Plotly stacked subplots (~180 LoC)
        ├── driver.js        # Driver tab (view + fit-from-lake) (~180 LoC)
        ├── stint.js         # Stint solver tab (~100 LoC)
        ├── validate.js      # Validation tab + overlay plot (~120 LoC)
        └── lake.js          # Lake browser tab (partition tree) (~100 LoC)
```

`partition_walker.py` and `partition_filter.py` are **vendored copies** from
`ac-quix-bridge/telemetry-comparison`. Out of scope: factoring them into a
shared library. The copy is cheap (~130 LoC combined) and the sister service
is the proven reference. If they diverge over time we revisit; v1.4 candidate.

Library-side touchpoints (added under `src/lap_estimator/`):
- `src/lap_estimator/lake_loader.py` (~150 LoC) — see §22.B.3.

`fit_driver.py` CLI gains a `--from-lake` flag — see §22.B.4.

### 22.A — HTTP API surface

All routes namespaced under `/api/`. JSON request/response unless noted.
Errors follow FastAPI default `{"detail": "..."}` with appropriate 4xx/5xx.

| Method | Path | Body / Query | Response | Purpose |
|---|---|---|---|---|
| GET | `/` | — | HTML | Jinja-rendered SPA shell. Mounts `/static/`. |
| GET | `/api/health` | — | `{"ok": true, "lake": "arrow"|"csv"|"unreachable"}` | Liveness + lake-transport probe state. |
| GET | `/api/cars` | — | `["bmw_1m", ...]` | Enumerate `cars_csv/<car>/data/tyres.ini` presence. |
| GET | `/api/tracks` | — | `[{"track": "ks_nurburgring", "layouts": ["sprint_a", "gp_a_ideal_line", ...]}]` | Enumerate `tracks_csv/<track>/layout_*.csv`. |
| GET | `/api/drivers` | — | `[{"name": "tomas_full", "path": "drivers/tomas_full.json", "skill_pct": 1.0, "fit_version": "2", "compound": "Semislicks"}]` | Enumerate `drivers/*.json` with summary metadata. Cached by mtime. |
| GET | `/api/driver/{name}` | — | full parsed JSON (§7.2) | Single-driver detail for the Driver tab viewer. |
| POST | `/api/sim` | see §22.A.1 | see §22.A.1 | Run a sim stint, return artefact URLs. |
| POST | `/api/solve` | see §22.A.2 | see §22.A.2 | Inverse PSI solver. |
| POST | `/api/validate` | see §22.A.3 | see §22.A.3 | Sim vs real-lap overlay. |
| POST | `/api/fit_driver` | see §22.A.4 | SSE stream | Fit a driver JSON; streams stage events. |
| GET | `/api/lake/tree` | — | partition-tree JSON (see §22.A.5) | Driver→car→track→session→lap hierarchy from QuixLake. |
| GET | `/api/lake/laps` | `?driver=&car=&track=&limit=` | `[{"session_id": ..., "lap": ..., "lap_time_s": ..., "started_at": ...}]` | Flat lap list for the fit-driver picker. |
| GET | `/api/sim/result/{job_id}/telemetry.csv` | — | `text/csv` (chunked if >10 MB) | Streams the in-memory telemetry DataFrame for Plotly to fetch. |
| GET | `/api/sim/result/{job_id}/trace.csv` | — | `text/csv` | Streams the per-point trace. |
| GET | `/api/sim/result/{job_id}/plot.png` | — | `image/png` | Optional static plot (back-compat with CLI's `_sim_vs_ai.png`). |
| GET | `/api/sim/result/{job_id}/overlay.png` | — | `image/png` | Validation overlay PNG (only for `/api/validate` jobs). |
| GET | `/api/sim/result/{job_id}/stint_summary.csv` | — | `text/csv` | Per-lap wear/temp/pressure block (stint mode). |

Job results live in an in-memory dict (`web/jobs.py`) keyed by UUID, with a
60-minute TTL. Sufficient for single-user workspace use; no Redis required.

#### §22.A.1 `POST /api/sim`

**Request:**
```json
{
  "car": "bmw_1m",
  "track": "ks_nurburgring/layout_sprint_a",
  "driver": "tomas_full",
  "compound": "Semislicks",          // optional; falls through resolution precedence
  "pressures_psi": {"FL": 26, "FR": 26, "RL": 26, "RR": 26},  // optional
  "ambient_temp_c": 26.0,            // optional, default 25.0
  "n_laps": 3,                       // required, [1, 50]
  "ds": 2.0,                         // optional
  "telemetry_dt_ms": 10              // optional
}
```

**Behaviour:** generate a `job_id` (UUID), call
`sim_runner.run_sim(...)` (which calls `simulate_stint(...)` under
`asyncio.to_thread`). Cache the resulting `StintResult` + telemetry DataFrame
in `jobs.py`. Sim runs are <5 s; the request blocks until done (no SSE).

**Response:**
```json
{
  "job_id": "9c4f...",
  "lap_times_s": [108.2, 107.6, 107.4],
  "telemetry_csv_url": "/api/sim/result/9c4f.../telemetry.csv",
  "trace_csv_url": "/api/sim/result/9c4f.../trace.csv",
  "stint_summary_csv_url": "/api/sim/result/9c4f.../stint_summary.csv",
  "plot_png_url": "/api/sim/result/9c4f.../plot.png",
  "compound_resolved": "Semislicks (idx 1)",
  "compound_source": "cli"
}
```

#### §22.A.2 `POST /api/solve`

**Request:**
```json
{
  "car": "bmw_1m",
  "track": "ks_nurburgring/layout_sprint_a",
  "driver": "tomas_full",
  "compound": "Semislicks",
  "target_wear": 0.50,
  "target_lap": 12,
  "target_wheel": "max",       // optional, default "max"
  "uniform_pressure": false,   // optional
  "ambient_temp_c": 26.0
}
```

**Response:**
```json
{
  "job_id": "...",
  "recommended_pressures_psi": {"FL": 30.5, "FR": 30.2, "RL": 28.8, "RR": 29.1},
  "iterations": 8,
  "converged": true,
  "verification_stint": [
    {"lap": 1, "lap_time_s": 108.1, "wear_max_pct": 6.3, "...": "..."},
    ...
    {"lap": 12, "lap_time_s": 108.9, "wear_max_pct": 50.4, "...": "..."}
  ]
}
```

Backed by `solve_setup.solve_pressure_for_wear(...)` under
`asyncio.to_thread`. <10 s typical (bisection cap 12 iter × ~0.5 s per sim).

#### §22.A.3 `POST /api/validate`

**Request:**
```json
{
  "car": "bmw_1m",
  "track": "ks_nurburgring/layout_sprint_a",
  "driver": "tomas_full",
  "real_source": {
    "kind": "local",                                    // "local" | "lake"
    "path": "samples/aclog/Tomas_Lap2.csv"              // when kind=local
  },
  // OR
  "real_source": {
    "kind": "lake",
    "driver": "tomas",
    "car": "bmw_1m",
    "track": "ks_nurburgring",
    "session_id": "20260514T...",
    "lap": 2
  },
  "compound": "Semislicks",
  "bin_m": 100,
  "per_corner": false
}
```

**Response:**
```json
{
  "job_id": "...",
  "real_lap_time_s": 107.56,
  "sim_lap_time_s": 107.42,
  "delta_s": -0.14,
  "verdict": "GOOD",                              // GOOD | LOOSE | BAD
  "overlay_png_url": "/api/sim/result/.../overlay.png",
  "validation_bins_csv_url": "/api/sim/result/.../validation_bins.csv",
  "telemetry_csv_url": "/api/sim/result/.../telemetry.csv"
}
```

Backed by `validate.validate_lap(...)`. Threshold values for GOOD/LOOSE/BAD
come from the v1 spec §15 unchanged.

#### §22.A.4 `POST /api/fit_driver` (SSE)

This is the only long-running endpoint (10–60 s). It returns
`Content-Type: text/event-stream` and emits stage events as the fitter
progresses. Implementation: `fit_driver(...)` is called as a generator inside
the thread; each yielded stage becomes an SSE `event:` line.

**Request:**
```json
{
  "car": "bmw_1m",
  "track": "ks_nurburgring/layout_sprint_a",
  "driver_name": "ludvik",
  "source": {
    "kind": "lake",
    "driver": "ludvik",
    "car": "bmw_1m",
    "track": "ks_nurburgring",
    "n_newest": 10                  // optional, default 10
  },
  // OR
  "source": {
    "kind": "local",
    "csv_paths": ["samples/aclog/Ludvik_Lap1.csv", ...]
  },
  "overwrite": true                 // confirms overwrite of existing drivers/<name>.json
}
```

**SSE event stream** (`text/event-stream`, 30 s heartbeat to keep proxies
happy):

```
event: stage
data: {"stage": "lake_query_started", "ts": "..."}

event: stage
data: {"stage": "lake_query_done", "ts": "...", "n_laps": 10, "transport": "arrow"}

event: stage
data: {"stage": "merged_done", "ts": "...", "rows": 87532}

event: stage
data: {"stage": "skill_fit_done", "ts": "...", "skill_pct": 0.91}

event: stage
data: {"stage": "tyre_calibration_done", "ts": "...", "rmse_temp_C": 4.8, "measured": true}

event: stage
data: {"stage": "validation_done", "ts": "...", "real_lap_time_s": 108.3, "sim_lap_time_s": 107.9}

event: written
data: {"path": "drivers/ludvik.json", "fit_version": "2"}

event: done
data: {"job_id": "...", "driver_url": "/api/driver/ludvik"}

: keep-alive
```

Errors emit `event: error\ndata: {"detail": "..."}` then close.

Minimum stage events required for acceptance §11.42: `lake_query_done`,
`merged_done`, `skill_fit_done`, `tyre_calibration_done`, `written`.

#### §22.A.5 `GET /api/lake/tree`

Returns the QuixLake partition tree walked via `partition_walker.py`. Same
shape as `telemetry-comparison`'s `/api/sessions` for the same lake instance.

```json
{
  "drivers": [
    {
      "driver": "tomas",
      "cars": [
        {
          "car": "bmw_1m",
          "tracks": [
            {
              "track": "ks_nurburgring",
              "sessions": [
                {"session_id": "20260514T...", "lap_count": 6, "fitted_locally": true}
              ]
            }
          ]
        }
      ]
    },
    {"driver": "ludvik", "cars": [...], "fitted_locally": false}
  ],
  "lake_url": "...",
  "transport": "arrow"
}
```

`fitted_locally` is computed by intersecting the lake driver names against
`drivers/*.json` filenames (case-insensitive stem match).

### 22.B — In-process invocation contracts

ArchDev wires these via direct imports — no subprocess, no CLI re-shelling. All
heavy calls go through `asyncio.to_thread(...)` so FastAPI's event loop stays
responsive.

#### §22.B.1 Functions to import

| Endpoint | Function | Module |
|---|---|---|
| `/api/sim` | `simulate_stint(car, track, driver, *, n_laps, setup, calibration, ds, compound)` | `src.lap_estimator.simulator` |
| `/api/solve` | `solve_pressure_for_wear(car, track, driver, *, target_wear, target_lap, target_wheel, uniform, compound, calibration, ambient_temp_C, n_laps_max)` | `src.lap_estimator.solve_setup` |
| `/api/validate` | `validate_lap(car, track, driver, real_csv_path, *, bin_m, per_corner, compound)` | `src.lap_estimator.validate` |
| `/api/fit_driver` | `fit_driver(car_dir, track_csv, output_json, lap_csvs, *, name, validate, plot, lap, newest, from_lake=None)` | `src.lap_estimator.driver_fit` |
| All routes | `Car.from_dir(car_dir)`, `Track.from_csv(path)`, `Driver.load(path)`, `Setup.load(path)` | `src.lap_estimator.{car,track,driver,setup}` |

`fit_driver(...)` is upgraded to a **generator** that yields stage events
(strings or dicts) so SSE can consume incrementally. Existing CLI callers wrap
it in a `for _ in fit_driver(...): pass` loop to consume eagerly. This is a
small backward-compatible change documented in §22.B.5.

#### §22.B.2 Threading wrappers (`web/sim_runner.py`)

```python
async def run_sim(req: SimRequest) -> SimResponse:
    return await asyncio.to_thread(_sim_sync, req)

def _sim_sync(req: SimRequest) -> SimResponse:
    car = Car.from_dir(_repo_path("cars_csv", req.car))
    track = Track.from_csv(_repo_path("tracks_csv", req.track + ".csv"))
    driver = Driver.load(_repo_path("drivers", req.driver + ".json"))
    setup = Setup.from_request(req)                      # builds from req fields
    result = simulate_stint(car, track, driver,
                            n_laps=req.n_laps,
                            setup=setup,
                            calibration=driver.tyre_calibration,
                            ds=req.ds,
                            compound=req.compound)
    return _materialise(result)                          # writes job artefacts into jobs.py
```

Same pattern for `run_solve`, `run_validate`. For `run_fit_driver` use an
async generator that pumps the underlying sync generator across a
`queue.Queue` in a worker thread — see `web/sim_runner.py` skeleton in the
implementation brief.

#### §22.B.3 New library module `src/lap_estimator/lake_loader.py`

Exposes one function:

```python
def load_laps_from_lake(
    driver: str,
    car: str,
    track: str,
    n_newest: int = 10,
    *,
    lake_url: str | None = None,
    token: str | None = None,
) -> list[pd.DataFrame]:
    """
    Query QuixLake for the newest `n_newest` finished laps for the given
    (driver, car, track) tuple. Each returned DataFrame is one lap, columns
    matching the existing CSV path that fit_driver consumes (telemetry.py's
    expected schema).

    Out-lap trim and lap selection are delegated to existing helpers in
    telemetry.py — this module only handles the lake transport.
    """
```

**Implementation notes:**
- Uses `web/lake_client.py` for the HTTP+Arrow probe. To avoid `web/` being a
  dependency of library code, `lake_loader.py` imports `httpx` + `pyarrow`
  directly and re-implements the small `query_sql()` helper. ~80 LoC.
- SQL: `SELECT * FROM telemetry WHERE driver = ? AND carModel = ? AND track = ? ORDER BY recorded_at DESC` then bucket rows into laps by `currentLap` / `lap` column transitions. Reuses the same lap-bucketing helper that `partition_walker` uses in the sister service.
- Returns the same shape `fit_driver(...)` already expects when given a list of CSV-parsed DataFrames.

#### §22.B.4 `fit_driver.py` CLI gains `--from-lake`

```
python fit_driver.py <car_dir> <track_csv> <output.json> \
    --from-lake driver=<name>,car=<name>,track=<name>,n=<int> \
    [other flags]
```

When `--from-lake` is present, positional CSV paths and `--laps-glob` must
NOT be supplied (argparse mutual-exclusion). The CLI calls
`lake_loader.load_laps_from_lake(...)` and feeds the returned frames into
`fit_driver(...)` exactly as if they had been parsed from local CSVs.

This keeps the CLI useful for terminal users on the same dataset the UI
sees, and is the only library-side behaviour change for v3-aux.

#### §22.B.5 `fit_driver(...)` becomes a generator

Existing signature returns a written-JSON path. New signature:

```python
def fit_driver(...) -> Iterator[FitStage]:
    yield FitStage("lake_query_started", ...)
    ...
    yield FitStage("merged_done", rows=...)
    ...
    yield FitStage("skill_fit_done", skill_pct=...)
    ...
    yield FitStage("tyre_calibration_done", rmse_temp_C=..., measured=...)
    ...
    yield FitStage("written", path=output_json)
```

`FitStage` is a small dataclass with `name: str` and an arbitrary `**kwargs`
payload (serialisable to JSON).

CLI back-compat: `fit_driver.py` consumes the generator with
`stage = None; for stage in fit_driver(...): print(f"[{stage.name}] {stage.payload}")`.

### 22.C — Lake transport probe (Arrow vs CSV)

At service startup, `web/lake_client.py` issues one probe request to
`{QUIXLAKE_URL}/query` with `Accept: application/vnd.apache.arrow.stream` and
a trivial `SELECT 1`. Outcomes:

| Status | Content-Type | Decision |
|---|---|---|
| 200 | `application/vnd.apache.arrow.stream` | Use Arrow → `pyarrow.ipc.RecordBatchStreamReader` → `pa.Table` → `.to_pandas()`. |
| 200 | `text/csv` (server downgraded) | Use CSV via `pd.read_csv(io.StringIO(resp.text))`. |
| 406 | — | Re-issue without the Arrow accept header; use CSV. |
| Other | — | Log error; mark lake unreachable; `/api/lake/*` returns 503 until probe re-attempts. |

The decision is cached for the process lifetime and surfaced on
`GET /api/health`. Override via `LAKE_ARROW_FORCE=1`/`0`. Arrow is ~3–5×
faster on large lap pulls and avoids the entire CSV-parsing path (date
parsing, string→float, etc).

### 22.D — Tab UX specifications

Each tab is one ES module under `web/static/modules/`. All five share the
header (logo, tab strip, env indicator showing `arrow|csv|offline`) rendered by
`web/templates/index.html`. Tab switching is client-side (no full reload).

#### §22.D.1 Sim tab (`sim.js`)

**Pickers:**
- Car: `<select>` populated from `/api/cars`.
- Track: cascaded `<select>` (track → layout) from `/api/tracks`.
- Driver: grouped `<select>` with two `<optgroup>`s:
  1. "Fitted locally" — names from `/api/drivers`.
  2. "In lake, not fitted" — names from `/api/lake/tree` minus the local set.
     Selecting one of these surfaces a "Fit then run" button that POSTs
     `/api/fit_driver` first (SSE), then `/api/sim`.
- Compound: `<select>` populated from the chosen car's `tyres.ini` compounds
  (served as `compounds` array on `/api/cars` response — see addendum).
- Pressures: four numeric inputs (FL/FR/RL/RR psi) + "uniform" checkbox.
- Ambient temp: numeric input (°C).
- Laps: numeric input [1, 50].

**Run** button → `POST /api/sim`. Spinner. On response, render five stacked
Plotly subplots (speed, gas+brake, temp 4-wheel, wear 4-wheel, pressure
4-wheel) fed from `telemetry_csv_url`. Below the plot: per-lap stdout block
table fed from `stint_summary_csv_url`.

#### §22.D.2 Driver tab (`driver.js`)

**Mode toggle:** "View existing" / "Fit new driver".

**View existing:**
- Driver `<select>` from `/api/drivers`.
- On select, fetch `/api/driver/{name}` and render the driver JSON in two
  panels: a key-stat summary (skill_pct, consistency_sigma, trail_brake_m,
  throttle_ramp_m, fit_version, compound, lap-time delta) and a JSON tree
  viewer for the full payload.

**Fit new driver:**
- Name input (text; will become `drivers/<name>.json`).
- Car `<select>` (required) — defaults to the most recent lake driver's car.
- Track `<select>` (required).
- Source `<select>`: "From lake (newest N)" / "From local CSVs".
  - Lake mode: driver `<select>` populated from `/api/lake/tree`, plus a
    "newest N" numeric input (default 10).
  - Local mode: file-list textarea (one path per line, paths relative to
    `LAP_ESTIMATOR_REPO_ROOT`).
- "Overwrite existing" checkbox (required if `drivers/<name>.json` already
  exists; otherwise the POST 409s).

**Run** button → opens SSE to `POST /api/fit_driver`. A progress strip
renders one chip per stage; final stage flips the strip green and shows the
"Saved" banner with the post-fit summary (sim_lap_time_s, real_lap_time_s,
delta_s) and the persistence caveat.

#### §22.D.3 Stint solver tab (`stint.js`)

- Pickers: car / track / driver / compound (same as Sim tab).
- Inputs: target wear (slider 0.1–0.95), target lap (numeric 2–50),
  target wheel (`max`/`min`/`avg`/`FL`/`FR`/`RL`/`RR`), uniform-pressure
  checkbox, ambient temp.
- Run → `POST /api/solve`. Result panel: recommended pressures table +
  verification stint mini-plot (per-lap wear of the target wheel).

#### §22.D.4 Validate tab (`validate.js`)

- Pickers: car / track / driver / compound.
- Real-lap source: "Local CSV" (file picker against `samples/aclog/`) or
  "Lake" (cascaded session→lap select using `/api/lake/laps`).
- Bin width (numeric, default 100 m) + "per corner" checkbox.
- Run → `POST /api/validate`. Renders the overlay PNG inline, plus three
  Plotly panels: speed-vs-distance overlay (sim+real), gas/brake overlay,
  per-bin delta bars. Verdict shown as a GOOD/LOOSE/BAD chip.

#### §22.D.5 Lake tab (`lake.js`)

- One big drill-down tree fed from `/api/lake/tree`: driver → car → track →
  session → lap-count.
- Each driver row has a "Fit this driver" button that jumps to the Driver
  tab with the picker pre-filled.
- Each session row has a "Validate against this lap" button that jumps to
  the Validate tab pre-filled.
- Refresh button issues a fresh `/api/lake/tree` (the response is cached
  in-memory for 60 s otherwise).

### 22.E — Telemetry rendering contract

Sim runs hold their telemetry DataFrame in `web/jobs.py` keyed by `job_id`.
The DataFrame is identical to the CSV that `sim_telemetry.write_synthetic_log`
would have written, including the 12 v2 per-wheel state columns (§7.12) and
the `lap` column.

`GET /api/sim/result/{job_id}/telemetry.csv`:
- If the DataFrame serialises to <10 MB, return as a single `text/csv`
  response.
- Otherwise, stream via FastAPI's `StreamingResponse` with chunked transfer
  encoding (one chunk per 1000 rows). Plotly.js's CSV loader handles chunked
  responses transparently.

Plotly subplot layout (Sim tab):
1. Speed (km/h) vs distance (m).
2. Gas + brake (0..1) vs distance — two traces on same axis.
3. Tyre temp (FL/FR/RL/RR, °C) vs distance — four traces.
4. Tyre wear (FL/FR/RL/RR, %, 100=fresh) vs distance — four traces.
5. Tyre pressure (FL/FR/RL/RR, psi) vs distance — four traces.

X-axes are linked; zoom on any panel zooms all. `lap` column drives a colour
fade so multi-lap stints are visually distinguishable. Pattern matches the
existing `scratch_plot.py` matplotlib layout, made interactive.

Validation overlay (Validate tab):
1. Speed: sim trace + real trace.
2. Gas: sim trace + real trace.
3. Brake: sim trace + real trace.
4. Per-bin delta (s) bars — bin width from request.

### 22.F — Caching, hot-reload, SSE keep-alive

- **`/api/drivers` and `/api/cars` mtime check.** Each request stats the
  underlying files; if mtimes haven't changed since the last cache build,
  serve cached JSON. Otherwise rebuild. This makes "drop a new driver JSON
  on disk" visible immediately without restarts.
- **`/api/lake/tree`** is cached in-memory for 60 s. Manual refresh via the
  Lake tab's "Refresh" button passes a `?force=1` query param that bypasses
  the cache.
- **SSE keep-alive:** every 30 s emit a `: keep-alive` line on open SSE
  streams so reverse proxies (Quix Cloud's ingress) don't kill idle
  connections.
- **Sim job artefact TTL:** 60 minutes. Eviction on next access after expiry.

### 22.G — Out of scope (explicit)

- UI auth (workspace-only, mirrors sister service).
- Live Quix Streams ingestion.
- Pit-stop / mid-stint compound switching.
- Mobile-first responsive.
- MF4 export from UI.
- Editing track CSVs / car ini files / tracks_config.json via UI.
- Hand-authoring driver JSONs via a form (only "Fit new driver" is
  supported).
- Multi-user concurrent fit jobs sharing the same `drivers/<name>.json`
  (last-write-wins is accepted).
- Persistent server-side storage of sim results (in-memory 60 min TTL only).

### 22.H — Open questions resolved

1. **Cache strategy:** filesystem mtime check on each `/api/drivers` and
   `/api/cars` request. Lake tree cached 60 s.
2. **SSE keep-alive:** 30 s heartbeat.
3. **Telemetry-CSV streaming:** chunked transfer for >10 MB.
4. **Compound list per car:** `/api/cars` returns `[{name, compounds: [...]}]`
   so the UI doesn't need a second round-trip when populating the compound
   picker.
5. **Driver name collision on fit:** require explicit `overwrite: true` in the
   request body; respond 409 otherwise. UI surfaces a confirmation modal.
6. **Lake-driver matched to local-driver:** stem match (case-insensitive,
   `drivers/<name>.json` ↔ lake `driver` partition value). Edge cases
   (Tomas vs tomas vs TOMAS_FULL) resolved by canonicalising to lowercase.

### 22.I — Acceptance criteria (added to §11)

- **§11.40 — basic enumeration.** `GET /api/cars` returns `["bmw_1m"]` (the
  only car currently in `cars_csv/`). `GET /api/tracks` returns one
  `{"track": "ks_nurburgring", "layouts": [...]}` entry with the four
  committed layouts (`sprint_a`, plus the three current `ks_nurburgring/layout_*.csv`
  siblings). `GET /api/drivers` returns the current set of
  `drivers/*.json` filenames as objects with summary fields.

- **§11.41 — sim parity with CLI.** `POST /api/sim` with body
  `{car: "bmw_1m", track: "ks_nurburgring/layout_sprint_a", driver: "tomas_full",
  compound: "Semislicks", pressures_psi: {FL:33,FR:33,RL:34,RR:34}, ambient_temp_c: 26,
  n_laps: 2}` returns `lap_times_s[1]` within **±0.05 s** of the equivalent
  `python lap.py cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv drivers/tomas_full.json --compound Semislicks --pressure FL=33,FR=33,RL=34,RR=34 --ambient-temp-c 26 --laps 2` invocation.

- **§11.42 — fit-from-lake end-to-end.** `POST /api/fit_driver` with body
  `{car: "bmw_1m", track: "ks_nurburgring/layout_sprint_a", driver_name: "ludvik_test",
  source: {kind: "lake", driver: "ludvik", car: "bmw_1m", track: "ks_nurburgring", n_newest: 10}, overwrite: true}`
  (against a lake instance that has ≥ 10 Ludvik laps) produces an SSE stream
  containing at minimum the events `lake_query_done`, `merged_done`,
  `skill_fit_done`, `tyre_calibration_done`, `written`. After completion
  `drivers/ludvik_test.json` exists on disk, loads cleanly via
  `Driver.load(...)`, has `fit_version == "2"` and a populated
  `tyre_calibration.source.compound` field.

- **§11.43 — lake tree parity.** `GET /api/lake/tree` returns the same set of
  `(driver, car, track, session_id)` tuples as
  `telemetry-comparison`'s `GET /api/sessions` against the same lake URL +
  token, modulo response-shape (the §22.A.5 nested object vs sister
  service's flat session list).

- **§11.44 — SPA navigation.** UI loads at `http://localhost:8080/`, all five
  tabs render without page reload (verified by `window.performance.navigation`
  counters or equivalent), Tailwind CDN classes apply (verified by computing
  CSS on a known utility class).

- **§11.45 — Arrow probe.** On startup against a QuixLake instance that
  supports Arrow, `GET /api/health` returns `{"lake": "arrow"}`. Setting
  `LAKE_ARROW_FORCE=0` forces CSV and `/api/health` returns `{"lake": "csv"}`.
  Against an unreachable lake, returns `{"lake": "unreachable"}` and
  `/api/lake/*` endpoints return 503.

- **§11.46 — fit-from-lake CLI parity.** `python fit_driver.py
  cars_csv/bmw_1m tracks_csv/ks_nurburgring/layout_sprint_a.csv
  drivers/ludvik_test.json --from-lake driver=ludvik,car=bmw_1m,track=ks_nurburgring,n=10`
  produces a JSON that, when its `skill_pct`, `consistency_sigma`, and
  `tyre_calibration.*` fields are compared to the JSON produced by §11.42's
  SSE flow against the same lake, matches **within 1e-6 relative tolerance**.
  (Same code path, two different drivers — output must be identical.)

### 22.J — Open questions deferred to ArchDev

- Exact natural seam in `simulator.py` for the in-process `simulate_stint`
  import call from `web/sim_runner.py` — should the wrapper accept an
  already-constructed `Car` / `Track` / `Driver` or take string IDs?
  Recommendation: take pre-constructed dataclasses (ArchDev decides path
  resolution in `sim_runner.py`).
- Whether `partition_walker.py` should be vendored or factored. Vendor for
  v3-aux; revisit in v1.4 if it drifts.
- Whether to ship a `tests/web/` suite. Per CLAUDE.md "I run QA manually in
  Quix Cloud; skip Tester by default" — no automated test suite for v3-aux.

### 22.K — References

- `ac-quix-bridge/telemetry-comparison/main.py`, `partition_walker.py`,
  `partition_filter.py`, `static/` — the proven sister-service pattern.
- `docs/architecture-config-pipeline.md` — the 5-source config flow the UI
  visualises.
- `README.md` §CLIs — the CLI surfaces being wrapped.
- `lap.py`, `fit_driver.py` — module-level functions to import.
- §13, §14, §15, §21 — algorithm specs underpinning the API endpoints.
- Decisions item 23 (v2 stint sim), item 24 (compound-aware parsing), item
  25 (asymmetric pressure model).

### 22.L — Decisions item (proposed, to add to Decisions block)

**26. (v3-aux) Web UI is a new top-level `web/` directory deploying as a
Quix Cloud service.** FastAPI + Tailwind CDN + Plotly.js + vanilla ES
modules. In-process import of `lap_estimator` (no subprocess). SSE for
long-running `fit_driver`. Lake transport auto-probes Arrow vs CSV. Mirrors
`ac-quix-bridge/telemetry-comparison`'s env vars (`QUIX_LAKE_TOKEN`,
`QUIXLAKE_URL`) and deploy shape. Five tabs (Sim, Driver, Stint, Validate,
Lake). New library module `src/lap_estimator/lake_loader.py` lets
`fit_driver` ingest from the lake (CLI gains `--from-lake`). `fit_driver(...)`
becomes a generator yielding stage events. CLI surface (`lap.py`,
`fit_driver.py`) unchanged for back-compat. Acceptance §11.40–§11.46.
(§22, §6.1.dep.)
