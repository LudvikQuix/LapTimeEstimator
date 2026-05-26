"""Walk QuixLake's partition tree via the native /partitions endpoint.

VENDORED from ac-quix-bridge/telemetry-comparison/partition_walker.py per
spec §22.6. The only edit vs the source is the `import config` line below
(swapped to `from . import config`).

Used by /api/lake/tree to enumerate sessions without a SQL GROUP BY over
Parquet. Each level of the tree is one S3 LIST (~150 ms); sibling calls fan
out via asyncio.gather so total latency is roughly D x single-call.

Config values are read off the `config` module at call time so tests can
monkeypatch `config.QUIXLAKE_URL` / `config.QUIX_LAKE_TOKEN` to simulate
missing vars. The http client and semaphore are module-level so the
connection pool + TLS handshake are amortized across the walk.
"""

from __future__ import annotations

import asyncio

import httpx

from . import config

PARTITION_COLS = [
    "environment",
    "test_rig",
    "experiment",
    "driver",
    "track",
    "carModel",
    "session_id",
]

# Reused across all /partitions calls so a single TLS handshake + connection
# pool is amortized over the entire tree walk. Creating a new AsyncClient
# per call was costing ~30ms of TLS setup each. Kept module-level so FastAPI's
# autoreload doesn't need a lifespan handler; httpx.AsyncClient is safe to
# reuse across requests.
_http_client = httpx.AsyncClient(
    timeout=30.0,
    limits=httpx.Limits(max_connections=50, max_keepalive_connections=20),
)

# Caps the number of in-flight QuixLake requests per process.
_LAKE_CONCURRENCY = 20
_lake_semaphore: asyncio.Semaphore | None = None


def _get_lake_semaphore() -> asyncio.Semaphore:
    """Lazy-init so the Semaphore binds to the event loop that's actually
    running (avoids `got Future attached to a different loop` under some
    test configurations)."""
    global _lake_semaphore
    if _lake_semaphore is None:
        _lake_semaphore = asyncio.Semaphore(_LAKE_CONCURRENCY)
    return _lake_semaphore


async def _list_partition_children(path: str) -> list[str]:
    """Return the immediate child partition names under `path`."""
    config.require_lake_env()
    params = (
        {"table": config.TABLE_NAME, "path": path}
        if path
        else {"table": config.TABLE_NAME}
    )
    async with _get_lake_semaphore():
        response = await _http_client.get(
            f"{config.QUIXLAKE_URL}/partitions",
            params=params,
            headers={"Authorization": f"Bearer {config.QUIX_LAKE_TOKEN}"},
        )
    response.raise_for_status()
    return [p["name"] for p in response.json().get("partitions", [])]


async def _walk_partition_tree(
    path: str, depth: int, filters: dict[str, str] | None = None
) -> list[dict]:
    """Recursively walk the partition tree under `path`. At the leaf
    (session_id level) attaches a `laps` list by listing the lap=N
    sub-partitions so the frontend gets sessions + their laps in one round trip.

    Fan-out at each level is parallelized via asyncio.gather.
    """
    if depth == len(PARTITION_COLS):
        session: dict = {}
        for part in path.split("/"):
            if "=" in part:
                k, v = part.split("=", 1)
                session[k] = v
        lap_names = await _list_partition_children(path)
        laps: list[int] = []
        for name in lap_names:
            if name.startswith("lap="):
                try:
                    laps.append(int(name[len("lap=") :]))
                except ValueError:
                    continue
        session["laps"] = sorted(laps)
        return [session]

    children = await _list_partition_children(path)
    if not children:
        return []

    if filters:
        col = PARTITION_COLS[depth]
        wanted = filters.get(col)
        if wanted:
            target = f"{col}={wanted}"
            children = [c for c in children if c == target]

    next_paths = [f"{path}/{child}" if path else child for child in children]
    subtrees = await asyncio.gather(
        *(_walk_partition_tree(p, depth + 1, filters) for p in next_paths)
    )
    return [s for sublist in subtrees for s in sublist]
