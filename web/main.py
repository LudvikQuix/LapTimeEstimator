"""FastAPI app for the lap-estimator-web service (spec §22).

Mounted routes:
  - GET  /                       SPA shell (Jinja-rendered index.html)
  - GET  /api/health             Liveness + lake transport probe
  - GET  /api/cars               Enumerate cars_csv/<car>/
  - GET  /api/tracks             Enumerate tracks_csv/<track>/layout_*.csv
  - GET  /api/drivers            Enumerate drivers/*.json
  - GET  /api/driver/{name}      Full driver JSON
  - POST /api/sim                Run a sim stint
  - POST /api/solve              Inverse-PSI solver
  - POST /api/validate           Sim vs real lap overlay
  - POST /api/fit_driver         SSE: fit a driver from lake or local CSVs
  - GET  /api/lake/tree          Lake partition tree
  - GET  /api/lake/laps          Flat lap list for picker
  - GET  /api/sim/result/{job_id}/{artefact}    Sim artefact streams

Subroutes live in sibling routers: `sim_routes`, `lake_routes`, `fit_routes`.
"""

from __future__ import annotations

import json
import logging
import os
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from starlette.requests import Request

from . import config, fit_routes, jobs, lake_client, lake_routes, sim_routes

logging.basicConfig(level=config.LOGLEVEL)
for _name in ("uvicorn", "uvicorn.error", "uvicorn.access"):
    logging.getLogger(_name).propagate = False
logging.getLogger("httpx").setLevel(logging.WARNING)
logging.getLogger("httpcore").setLevel(logging.WARNING)
logger = logging.getLogger(__name__)

app = FastAPI(title="Lap Time Estimator Web")
app.include_router(sim_routes.router)
app.include_router(fit_routes.router)
app.include_router(lake_routes.router)

templates = Jinja2Templates(directory=str(config.TEMPLATES_DIR))


# ---------------------------------------------------------------------------
# Local-filesystem enumeration endpoints (cars, tracks, drivers)
# ---------------------------------------------------------------------------


_CACHE: dict[str, dict] = {
    "cars": {"mtime": -1.0, "body": None},
    "tracks": {"mtime": -1.0, "body": None},
    "drivers": {"mtime": -1.0, "body": None},
}


def _dir_mtime(p: Path) -> float:
    if not p.exists():
        return 0.0
    # Use the directory's mtime; child file mtimes propagate via mtime of dir
    # on most filesystems when entries are added/removed. For in-place edits
    # we also include the max mtime of immediate children.
    try:
        m = p.stat().st_mtime
        for child in p.iterdir():
            if child.is_file():
                m = max(m, child.stat().st_mtime)
            elif child.is_dir():
                m = max(m, child.stat().st_mtime)
        return m
    except OSError:
        return 0.0


def _list_cars() -> list[dict]:
    """Enumerate cars by detecting `engine.ini` directly under cars_csv/<name>/."""
    from lap_estimator.car import Car
    out = []
    if not config.CARS_DIR.is_dir():
        return out
    for entry in sorted(config.CARS_DIR.iterdir()):
        if not entry.is_dir():
            continue
        if not (entry / "engine.ini").is_file():
            continue
        try:
            car = Car(str(entry))
            compounds = [
                {"name": c.name, "short_name": c.short_name, "index": c.index}
                for c in car.compounds
            ]
            out.append({
                "name": entry.name,
                "compounds": compounds,
                "default_compound_index": car.default_compound_index,
            })
        except Exception as e:  # noqa: BLE001
            logger.warning("Failed to load car %s: %s", entry.name, e)
            out.append({"name": entry.name, "compounds": [], "error": str(e)})
    return out


def _list_tracks() -> list[dict]:
    """Enumerate tracks: each `tracks_csv/<track>/layout_*.csv` is a layout.

    Filters out sim-output siblings (the `layout_xxx__*` files emitted by
    `lap.py`) -- those carry per-driver telemetry/trace data, not track
    definitions. The `_ideal_line` variants are kept because they're real
    alternate layout CSVs (different racing line).
    """
    out = []
    if not config.TRACKS_DIR.is_dir():
        return out
    for entry in sorted(config.TRACKS_DIR.iterdir()):
        if not entry.is_dir():
            continue
        layouts = []
        for csv_path in sorted(entry.glob("layout_*.csv")):
            name = csv_path.stem
            # Sim outputs use a `__` separator (double underscore) in the stem
            # (e.g. `layout_sprint_a__tomas_sim_trace`). Real layout CSVs use
            # single underscores only (`layout_sprint_a`, `layout_sprint_a_ideal_line`).
            if "__" in name:
                continue
            short = name[len("layout_"):] if name.startswith("layout_") else name
            layouts.append({"name": short, "full": name})
        if layouts:
            out.append({"track": entry.name, "layouts": layouts})
    return out


def _list_drivers() -> list[dict]:
    """Enumerate drivers/*.json with summary metadata."""
    out = []
    if not config.DRIVERS_DIR.is_dir():
        return out
    for json_path in sorted(config.DRIVERS_DIR.glob("*.json")):
        try:
            with open(json_path) as f:
                raw = json.load(f)
            compound = None
            if isinstance(raw.get("tyre_calibration"), dict):
                src = raw["tyre_calibration"].get("source") or {}
                compound = src.get("compound") if isinstance(src, dict) else None
            out.append({
                "name": json_path.stem,
                "path": f"drivers/{json_path.name}",
                "skill_pct": float(raw.get("skill_pct", 0.0)),
                "fit_version": (
                    raw.get("source", {}).get("fit_version")
                    if isinstance(raw.get("source"), dict) else None
                ),
                "compound": compound,
            })
        except Exception as e:  # noqa: BLE001
            logger.warning("Failed to read %s: %s", json_path, e)
            out.append({"name": json_path.stem, "path": str(json_path), "error": str(e)})
    return out


def _cached(name: str, builder, root: Path):
    """Serve cached JSON when the underlying dir's mtime hasn't moved."""
    cur = _dir_mtime(root)
    slot = _CACHE[name]
    if slot["body"] is not None and slot["mtime"] == cur:
        return slot["body"]
    body = builder()
    slot["body"] = body
    slot["mtime"] = cur
    return body


@app.get("/api/cars")
def get_cars():
    return _cached("cars", _list_cars, config.CARS_DIR)


@app.get("/api/tracks")
def get_tracks():
    return _cached("tracks", _list_tracks, config.TRACKS_DIR)


@app.get("/api/drivers")
def get_drivers():
    return _cached("drivers", _list_drivers, config.DRIVERS_DIR)


@app.get("/api/driver/{name}")
def get_driver(name: str):
    if not name or "/" in name or "\\" in name or ".." in name:
        raise HTTPException(status_code=400, detail="Invalid driver name")
    path = config.DRIVERS_DIR / f"{name}.json"
    if not path.is_file():
        raise HTTPException(status_code=404, detail=f"Driver not found: {name}")
    try:
        with open(path) as f:
            return JSONResponse(content=json.load(f))
    except Exception as e:  # noqa: BLE001
        raise HTTPException(status_code=500, detail=f"Failed to load: {e}") from e


# ---------------------------------------------------------------------------
# Health
# ---------------------------------------------------------------------------


@app.get("/api/health")
async def health():
    """Liveness + lake transport probe. Never fails the request."""
    lake = await lake_client.health_check()
    return {
        "ok": True,
        "lake": lake,
        "jobs_in_memory": jobs.count(),
        "repo_root": str(config.REPO_ROOT),
    }


# ---------------------------------------------------------------------------
# SPA shell + static mount
# ---------------------------------------------------------------------------


app.mount("/static", StaticFiles(directory=str(config.STATIC_DIR)), name="static")


@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    return templates.TemplateResponse(
        request,
        "index.html",
        {"title": "Lap Time Estimator"},
    )


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "web.main:app",
        host="0.0.0.0",
        port=int(os.getenv("PORT", "8080")),
        reload=bool(os.getenv("RELOAD", "")),
    )
