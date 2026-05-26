"""QuixLake /query client with per-request Arrow-or-CSV content negotiation.

Spec §22.C (revised): the lake server now supports Arrow on demand, so we
just send `Accept: application/vnd.apache.arrow.stream, text/csv;q=0.5` on
every request and dispatch on the response Content-Type. No startup probe.

LAKE_ARROW_FORCE=1 -> Arrow only (raise on miss).
LAKE_ARROW_FORCE=0 -> CSV only (omit Arrow accept).
unset -> negotiate.
"""

from __future__ import annotations

import io
import logging

import httpx
import pandas as pd

from . import config

logger = logging.getLogger(__name__)

# Shared async client for QuixLake /query calls (TLS + pool amortisation).
_http: httpx.AsyncClient | None = None


def _client() -> httpx.AsyncClient:
    global _http
    if _http is None:
        _http = httpx.AsyncClient(
            timeout=120.0,
            limits=httpx.Limits(max_connections=50, max_keepalive_connections=20),
        )
    return _http


def _accept_header() -> str:
    """Build the Accept header per LAKE_ARROW_FORCE."""
    force = config.LAKE_ARROW_FORCE
    if force == "1":
        return "application/vnd.apache.arrow.stream"
    if force == "0":
        return "text/csv"
    return "application/vnd.apache.arrow.stream, text/csv;q=0.5"


def _decode(content: bytes, content_type: str) -> tuple[pd.DataFrame, str]:
    """Decode the lake response into a DataFrame; return (df, transport_tag)."""
    ctype = (content_type or "").lower()
    if "arrow" in ctype:
        try:
            import pyarrow as pa  # lazy import
            import pyarrow.ipc  # noqa: F401  -- registers the reader
        except ImportError as e:
            raise RuntimeError(
                "Lake returned Arrow but pyarrow is not installed; "
                "install pyarrow or set LAKE_ARROW_FORCE=0."
            ) from e
        table = pa.ipc.open_stream(content).read_all()
        return table.to_pandas(), "arrow"
    # Default: CSV (text/csv or anything else).
    return pd.read_csv(io.BytesIO(content)), "csv"


async def query(sql: str) -> pd.DataFrame:
    """POST a SQL string to QuixLake's /query endpoint; return a DataFrame.

    Per-request content negotiation: ask for Arrow first, fall back to CSV
    if the server downgrades. Raises HTTPStatusError on 4xx/5xx so callers
    can map to FastAPI HTTPException.
    """
    df, _ = await query_with_transport(sql)
    return df


async def query_with_transport(sql: str) -> tuple[pd.DataFrame, str]:
    """Same as `query` but also returns the transport tag ("arrow"|"csv")."""
    config.require_lake_env()
    headers = {
        "Authorization": f"Bearer {config.QUIX_LAKE_TOKEN}",
        "Content-Type": "text/plain",
        "Accept": _accept_header(),
        "Accept-Encoding": "gzip",
    }
    r = await _client().post(
        f"{config.QUIXLAKE_URL}/query",
        content=sql,
        headers=headers,
    )
    r.raise_for_status()
    ctype = r.headers.get("Content-Type", "")
    df, transport = _decode(r.content, ctype)
    return df, transport


async def health_check() -> str:
    """Tiny probe used by /api/health.

    Returns "arrow" | "csv" | "unreachable". Never raises.
    """
    try:
        config.require_lake_env()
    except RuntimeError:
        return "unreachable"
    try:
        _, transport = await query_with_transport("SELECT 1 AS one")
        return transport
    except Exception as e:  # noqa: BLE001 - probe must never propagate
        logger.warning("Lake health probe failed: %s", e)
        return "unreachable"
