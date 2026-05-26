"""/api/fit_driver SSE route."""

from __future__ import annotations

import json
import logging
import os

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse

from . import config, fit_runner

logger = logging.getLogger(__name__)
router = APIRouter()


def _sse_format(event: str, data: dict | str) -> bytes:
    """Format one SSE message. `data` is JSON-encoded when not already a str."""
    if isinstance(data, dict):
        data_str = json.dumps(data, default=str)
    else:
        data_str = str(data)
    return f"event: {event}\ndata: {data_str}\n\n".encode("utf-8")


def _sse_comment(text: str) -> bytes:
    return f": {text}\n\n".encode("utf-8")


@router.post("/api/fit_driver")
async def post_fit_driver(req: dict):
    """SSE endpoint: stream fit pipeline stage events.

    Body shape per spec §22.A.4. Required: car, track, driver_name, source,
    overwrite. The response is `text/event-stream`. Errors emit
    `event: error` then close.
    """
    # Validate the bare minimum up front so we can 400 cleanly.
    for key in ("car", "track", "driver_name", "source"):
        if key not in req:
            raise HTTPException(status_code=400, detail=f"missing field '{key}'")
    driver_name = str(req["driver_name"])
    overwrite = bool(req.get("overwrite", False))
    target = config.DRIVERS_DIR / f"{driver_name}.json"
    if target.exists() and not overwrite:
        raise HTTPException(
            status_code=409,
            detail=(
                f"drivers/{driver_name}.json exists; "
                "pass overwrite:true to replace."
            ),
        )
    # Ensure drivers dir exists (Docker volume mount or fresh repo).
    os.makedirs(config.DRIVERS_DIR, exist_ok=True)

    async def _stream():
        try:
            async for kind, payload in fit_runner.run_fit_driver(req):
                if kind == "heartbeat":
                    yield _sse_comment("keep-alive")
                elif kind == "stage":
                    yield _sse_format("stage", payload)
                elif kind == "written":
                    yield _sse_format("written", payload)
                elif kind == "done":
                    yield _sse_format("done", payload)
                elif kind == "error":
                    yield _sse_format("error", payload)
                    return
        except Exception as e:  # noqa: BLE001 - last-resort SSE error path
            logger.exception("fit_driver SSE crashed")
            yield _sse_format("error", {"detail": f"{type(e).__name__}: {e}"})

    return StreamingResponse(
        _stream(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "X-Accel-Buffering": "no",  # tell nginx not to buffer
        },
    )
