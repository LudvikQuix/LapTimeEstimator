"""SSE pump for /api/fit_driver (spec §22.B.2).

`fit_driver_pipeline(...)` is a synchronous generator; FastAPI's event loop
cannot iterate it directly without blocking. This module drives the
pipeline in a worker thread, shuttles `FitStage` events through a
`queue.Queue`, and exposes `run_fit_driver(...)` as an async generator the
SSE route can `async for` over.
"""

from __future__ import annotations

import asyncio
import queue
import sys
import threading
import time
from pathlib import Path

# Make `src/` importable for in-process calls (mirrors sim_runner).
_SRC = Path(__file__).resolve().parent.parent / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from lap_estimator.car import Car  # noqa: E402
from lap_estimator.driver_fit_pipeline import fit_driver_pipeline  # noqa: E402
from lap_estimator.track import Track  # noqa: E402

from . import config  # noqa: E402

_FIT_SENTINEL = object()


async def run_fit_driver(req: dict):
    """Async generator yielding ('stage'|'written'|'done'|'heartbeat'|'error', payload).

    Args:
        req: API request body. See spec §22.A.4 for shape.

    Yields:
        Tuples consumed by `fit_routes.post_fit_driver` and serialised as SSE.
    """
    q: queue.Queue = queue.Queue()
    err_holder: dict = {}
    thread = threading.Thread(
        target=_fit_worker,
        args=(req, q, err_holder),
        daemon=True,
    )
    thread.start()
    last_heartbeat = time.time()
    while True:
        try:
            item = q.get(timeout=0.5)
        except queue.Empty:
            # Heartbeat every 30 s so proxies keep the SSE alive.
            if time.time() - last_heartbeat > 30.0:
                last_heartbeat = time.time()
                yield ("heartbeat", {})
            if not thread.is_alive() and q.empty():
                break
            await asyncio.sleep(0.05)
            continue
        if item is _FIT_SENTINEL:
            break
        last_heartbeat = time.time()
        yield item
    if err_holder.get("error"):
        yield ("error", {"detail": err_holder["error"]})


def _car_path(car_name: str) -> str:
    base = config.CARS_DIR / car_name
    if not base.is_dir():
        raise FileNotFoundError(f"Unknown car: {car_name}")
    return str(base)


def _track_path(track_arg: str) -> str:
    base = config.TRACKS_DIR / (track_arg + ".csv")
    if not base.is_file():
        raise FileNotFoundError(f"Unknown track: {track_arg}")
    return str(base)


def _fit_worker(req: dict, q: queue.Queue, err_holder: dict) -> None:
    """Worker thread: drive the pipeline and shuttle stages onto the queue."""
    try:
        car_name = req["car"]
        track_arg = req["track"]
        driver_name = req["driver_name"]
        overwrite = bool(req.get("overwrite", False))
        source = req["source"]
        car = Car(_car_path(car_name))
        track = Track.from_csv(_track_path(track_arg))
        track.source_path = _track_path(track_arg)
        output_json = str(config.DRIVERS_DIR / f"{driver_name}.json")

        if source["kind"] == "lake":
            lake_args = {
                "driver": source["driver"],
                "car": source["car"],
                "track": source["track"],
                "n_newest": int(source.get("n_newest", 10)),
                "lake_url": config.QUIXLAKE_URL,
                "token": config.QUIX_LAKE_TOKEN,
            }
            csv_paths = None
        elif source["kind"] == "local":
            csv_paths = [
                str((config.REPO_ROOT / p).resolve())
                for p in source.get("csv_paths", [])
            ]
            if len(csv_paths) < 2:
                raise ValueError(
                    f"fit_driver local mode requires >=2 csv_paths, got {len(csv_paths)}"
                )
            lake_args = None
        else:
            raise ValueError(f"Unknown source.kind={source['kind']!r}")

        for stage in fit_driver_pipeline(
            car, track,
            output_json=output_json,
            name=driver_name,
            csv_paths=csv_paths,
            lake_args=lake_args,
            do_validate=True,
            overwrite=overwrite,
        ):
            evt_payload = {"stage": stage.name, **stage.payload}
            if stage.name == "written":
                q.put(("written", stage.payload))
            else:
                q.put(("stage", evt_payload))
        q.put(("done", {
            "driver_url": f"/api/driver/{driver_name}",
        }))
    except Exception as e:  # noqa: BLE001 - thread boundary
        err_holder["error"] = f"{type(e).__name__}: {e}"
    finally:
        q.put(_FIT_SENTINEL)
