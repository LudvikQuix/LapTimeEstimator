"""/api/lake/tree, /api/lake/laps routes.

The tree walk uses the vendored `partition_walker` (S3 LIST per level); the
laps endpoint uses a SQL query through `lake_client`.
"""

from __future__ import annotations

import logging
import time
from pathlib import Path

import httpx
from fastapi import APIRouter, HTTPException

from . import config, lake_client
from .partition_walker import _walk_partition_tree

logger = logging.getLogger(__name__)
router = APIRouter()

_TREE_CACHE: dict = {"ts": 0.0, "data": None, "transport": None}


def _local_driver_stems() -> set[str]:
    """Lowercase stems of `drivers/*.json`."""
    out: set[str] = set()
    if not config.DRIVERS_DIR.is_dir():
        return out
    for p in config.DRIVERS_DIR.glob("*.json"):
        out.add(p.stem.lower())
    return out


def _shape_tree(flat_sessions: list[dict]) -> dict:
    """Reshape the flat partition-walker output into the nested §22.A.5 form."""
    local = _local_driver_stems()
    drivers: dict = {}
    for s in flat_sessions:
        d_name = s.get("driver")
        if not d_name:
            continue
        d_entry = drivers.setdefault(d_name, {
            "driver": d_name,
            "fitted_locally": d_name.lower() in local
                              or any(d_name.lower() in stem or stem in d_name.lower()
                                     for stem in local),
            "cars": {},
        })
        car_name = s.get("carModel", "?")
        c_entry = d_entry["cars"].setdefault(car_name, {
            "car": car_name,
            "tracks": {},
        })
        t_name = s.get("track", "?")
        t_entry = c_entry["tracks"].setdefault(t_name, {
            "track": t_name,
            "sessions": [],
        })
        t_entry["sessions"].append({
            "session_id": s.get("session_id", "?"),
            "environment": s.get("environment"),
            "test_rig": s.get("test_rig"),
            "experiment": s.get("experiment"),
            "lap_count": len(s.get("laps", [])),
            "laps": s.get("laps", []),
        })

    # Convert nested dicts to lists for stable JSON output.
    drivers_list = []
    for d_name, d_entry in sorted(drivers.items()):
        cars_list = []
        for c_name, c_entry in sorted(d_entry["cars"].items()):
            tracks_list = []
            for t_name, t_entry in sorted(c_entry["tracks"].items()):
                t_entry["sessions"].sort(
                    key=lambda s: s["session_id"], reverse=True
                )
                tracks_list.append(t_entry)
            cars_list.append({"car": c_name, "tracks": tracks_list})
        drivers_list.append({
            "driver": d_name,
            "fitted_locally": d_entry["fitted_locally"],
            "cars": cars_list,
        })
    return {"drivers": drivers_list}


@router.get("/api/lake/tree")
async def get_tree(force: int = 0):
    """Return the partition tree under §22.A.5 shape. Cached 60 s by default."""
    now = time.time()
    if (
        not force
        and _TREE_CACHE["data"] is not None
        and now - _TREE_CACHE["ts"] < config.LAKE_TREE_TTL_SECONDS
    ):
        cached = dict(_TREE_CACHE["data"])
        cached["cache"] = "hit"
        cached["transport"] = _TREE_CACHE["transport"]
        return cached

    try:
        config.require_lake_env()
    except RuntimeError as e:
        raise HTTPException(status_code=503, detail=str(e)) from e

    try:
        sessions = await _walk_partition_tree("", 0, None)
    except httpx.HTTPStatusError as e:
        raise HTTPException(
            status_code=502,
            detail=f"Lake /partitions returned {e.response.status_code}",
        ) from e
    except httpx.TimeoutException as e:
        raise HTTPException(
            status_code=504, detail=f"Lake /partitions timed out: {e}"
        ) from e
    except Exception as e:  # noqa: BLE001
        logger.exception("Lake tree walk failed")
        raise HTTPException(
            status_code=500, detail=f"{type(e).__name__}: {e}"
        ) from e

    # Determine transport by sending one tiny query through lake_client.
    transport = "unknown"
    try:
        _, transport = await lake_client.query_with_transport("SELECT 1 AS one")
    except Exception:  # noqa: BLE001
        transport = "unknown"

    body = _shape_tree(sessions)
    body["lake_url"] = config.QUIXLAKE_URL
    body["transport"] = transport
    body["cache"] = "miss"
    _TREE_CACHE["data"] = body
    _TREE_CACHE["ts"] = now
    _TREE_CACHE["transport"] = transport
    return body


@router.get("/api/lake/laps")
async def get_laps(driver: str, car: str, track: str, limit: int = 50):
    """Return a flat list of (session_id, lap, lap_time_s) tuples.

    Backed by a SQL query: min/max timestamp per (session, lap) -> lap_time.
    """
    try:
        config.require_lake_env()
    except RuntimeError as e:
        raise HTTPException(status_code=503, detail=str(e)) from e

    # Safe interpolation: filter chars in the same allowlist the partition
    # filter uses (alphanumerics + _-.: + space).
    for v in (driver, car, track):
        if not v or any(c not in "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-.: " for c in v):
            raise HTTPException(status_code=400, detail=f"Invalid value: {v!r}")
    if not 1 <= int(limit) <= 1000:
        raise HTTPException(status_code=400, detail="limit must be 1..1000")

    sql = (
        f"SELECT session_id, lap, "
        f"MIN(timestamp_ms) AS t_start, MAX(timestamp_ms) AS t_end "
        f"FROM {config.TABLE_NAME} "
        f"WHERE driver = '{driver}' AND carModel = '{car}' AND track = '{track}' "
        f"GROUP BY session_id, lap "
        f"ORDER BY session_id DESC, lap ASC "
        f"LIMIT {int(limit)}"
    )
    try:
        df = await lake_client.query(sql)
    except httpx.HTTPStatusError as e:
        raise HTTPException(
            status_code=502, detail=f"Lake /query returned {e.response.status_code}"
        ) from e
    except Exception as e:  # noqa: BLE001
        logger.exception("Lake laps query failed")
        raise HTTPException(
            status_code=500, detail=f"{type(e).__name__}: {e}"
        ) from e

    out = []
    for _, row in df.iterrows():
        out.append({
            "session_id": str(row.get("session_id", "")),
            "lap": int(row["lap"]) if row.get("lap") is not None else None,
            "lap_time_s": round((float(row["t_end"]) - float(row["t_start"])) / 1000.0, 3),
            "started_at": str(row.get("session_id", "")),
        })
    return out
