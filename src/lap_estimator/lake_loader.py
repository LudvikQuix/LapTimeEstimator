"""Load real telemetry laps from QuixLake for the driver fitter (spec §22.B.3).

`load_laps_from_lake(driver, car, track, n_newest)` queries QuixLake for the
newest `n_newest` finished laps for the (driver, car, track) tuple and returns
a list of merged-with-track DataFrames in the same shape `driver_fit.fit_driver`
already consumes (one frame per lap, dict-of-arrays via numpy).

Implementation notes
--------------------
- Imports `httpx` and `pyarrow` lazily so the rest of the library stays
  stdlib + numpy. Error messages are explicit when those packages are
  missing.
- SQL is `SELECT ... FROM <table> WHERE driver=... AND carModel=... AND
  track=... ORDER BY recorded_at DESC` (per spec §22.B.3). Then rows are
  bucketed into laps by the `lap` column (Hive partition value).
- `lake_url` and `token` default to the env vars consumed by `web/config.py`
  so the CLI (`fit_driver.py --from-lake`) can run without spinning up the
  web service.
- Output is built by reusing `telemetry.merge_with_track` after synthesising
  a telem-dict that matches `read_ac_log`'s schema.
"""

from __future__ import annotations

import io
import os
import sys

import numpy as np

# These columns mirror `telemetry.REQUIRED_COLUMNS` + the v2 per-wheel state
# channels (spec §21.6) + tyreCompound (§21.11). We request them explicitly
# so the lake doesn't return all 200+ AC channels.
_TELEM_COLUMNS = (
    "timestamp_ms",
    "gas",
    "brake",
    "distanceTraveled",
    "speedKmh",
    "normalizedCarPosition",
    "steerAngle",
    "tyreCompound",
    "tyreTempFL", "tyreTempFR", "tyreTempRL", "tyreTempRR",
    "tyreWearFL", "tyreWearFR", "tyreWearRL", "tyreWearRR",
    "wheelsPressureFL", "wheelsPressureFR", "wheelsPressureRL", "wheelsPressureRR",
    # v3 (spec §23.7.1): Pacejka-fit prerequisites — additive to the v2
    # column list. Lake schema already exposes these AC channels; the
    # fitter falls back to "measured: false" when any are absent (per-lap).
    "wheelLoadFL", "wheelLoadFR", "wheelLoadRL", "wheelLoadRR",
    "wheelAngularSpeedFL", "wheelAngularSpeedFR",
    "wheelAngularSpeedRL", "wheelAngularSpeedRR",
    "localVelocity_x", "localVelocity_y", "localVelocity_z",
    "localAngularVel_x", "localAngularVel_y", "localAngularVel_z",
    "accG_x", "accG_y", "accG_z",
    "tyreContactHeadingFL_x", "tyreContactHeadingFL_y", "tyreContactHeadingFL_z",
    "tyreContactHeadingFR_x", "tyreContactHeadingFR_y", "tyreContactHeadingFR_z",
    "tyreContactHeadingRL_x", "tyreContactHeadingRL_y", "tyreContactHeadingRL_z",
    "tyreContactHeadingRR_x", "tyreContactHeadingRR_y", "tyreContactHeadingRR_z",
    "wheelSlipFL", "wheelSlipFR", "wheelSlipRL", "wheelSlipRR",
)
# Partition / metadata columns we also need for lap bucketing.
_PARTITION_COLUMNS = ("lap",)

_DEFAULT_TABLE = "ac_telemetry"


def _lazy_imports():
    """Import httpx + pyarrow (the latter optional). Returns (httpx, pa_or_None)."""
    try:
        import httpx as _httpx  # noqa: WPS433
    except ImportError as e:
        raise RuntimeError(
            "lake_loader requires `httpx` (pip install httpx). "
            "Install it or run fit_driver against local CSVs instead."
        ) from e
    try:
        import pyarrow as _pa  # noqa: WPS433
        import pyarrow.ipc  # noqa: F401, WPS433
    except ImportError:
        _pa = None
    return _httpx, _pa


def _resolve_env(lake_url: str | None, token: str | None) -> tuple[str, str, str]:
    url = lake_url or os.getenv("QUIXLAKE_URL")
    tok = token or os.getenv("QUIX_LAKE_TOKEN")
    if not url or not tok:
        raise RuntimeError(
            "lake_loader: missing QUIXLAKE_URL or QUIX_LAKE_TOKEN. "
            "Set them in the environment or pass lake_url=/token= explicitly."
        )
    table = os.getenv("TABLE_NAME", _DEFAULT_TABLE)
    return url, tok, table


def _decode_response(content: bytes, content_type: str, pa):
    """Decode an Arrow stream or CSV bytes blob into a pandas DataFrame."""
    import pandas as pd  # local import

    ctype = (content_type or "").lower()
    if "arrow" in ctype:
        if pa is None:
            raise RuntimeError(
                "Lake returned Arrow but pyarrow is not installed; "
                "install pyarrow."
            )
        return pa.ipc.open_stream(content).read_all().to_pandas()
    return pd.read_csv(io.BytesIO(content))


def _query_sql(sql: str, *, lake_url: str | None, token: str | None):
    """Synchronous /query helper, used by both the CLI and the web SSE path.

    Returns a pandas DataFrame. The web layer uses `web.lake_client.query`
    (async) for its own routes; this helper exists so the library can be
    called from synchronous CLI code without requiring an event loop.
    """
    httpx, pa = _lazy_imports()
    url, tok, _table = _resolve_env(lake_url, token)
    accept = "application/vnd.apache.arrow.stream, text/csv;q=0.5"
    if os.getenv("LAKE_ARROW_FORCE") == "0":
        accept = "text/csv"
    elif os.getenv("LAKE_ARROW_FORCE") == "1":
        accept = "application/vnd.apache.arrow.stream"
    r = httpx.post(
        f"{url}/query",
        content=sql,
        headers={
            "Authorization": f"Bearer {tok}",
            "Content-Type": "text/plain",
            "Accept": accept,
            "Accept-Encoding": "gzip",
        },
        timeout=180.0,
    )
    r.raise_for_status()
    return _decode_response(r.content, r.headers.get("Content-Type", ""), pa)


def _select_columns_sql(table: str, where_clause: str) -> str:
    cols = ", ".join(_TELEM_COLUMNS + _PARTITION_COLUMNS)
    return (
        f"SELECT {cols} FROM {table} "
        f"{where_clause} "
        f"ORDER BY session_id DESC, lap DESC"
    )


def _bucket_into_laps(df, *, n_newest: int):
    """Group `df` rows into per-lap DataFrames, return the newest `n_newest`.

    The lake returns rows ordered by `recorded_at DESC`; within a lap the
    rows are mixed timestamp orders, so we sort each per-lap subset
    ascending by `timestamp_ms` before returning. Each lap is keyed by
    `(session_id?, lap)` where session_id may not be present in the
    SELECT — in that case we just key by `lap` and rely on the partition
    filter having already pinned us to one (driver, car, track) tuple.
    """
    if df is None or df.empty:
        return []
    if "lap" not in df.columns:
        raise RuntimeError(
            "Lake response missing required `lap` column. "
            "Verify the lake schema includes the partition value."
        )
    # Group rows into laps. Cast lap to int for sane sort.
    try:
        df = df.copy()
        df["lap"] = df["lap"].astype(int)
    except Exception:  # noqa: BLE001
        pass
    # Sort within each lap by timestamp ascending.
    df = df.sort_values(["lap", "timestamp_ms"], ascending=[False, True])
    per_lap = []
    for lap_value, sub in df.groupby("lap", sort=False):
        # Drop laps with too few rows (out-laps that never crossed S/F).
        if len(sub) < 50:
            continue
        per_lap.append((int(lap_value), sub.reset_index(drop=True)))
    # Newest-first by lap number (lap N is the latest within a session).
    per_lap.sort(key=lambda kv: -kv[0])
    return per_lap[:n_newest]


def _df_to_telem_dict(sub):
    """Convert a per-lap DataFrame into the dict-of-arrays shape `read_ac_log`
    produces, so `merge_with_track` can ingest it unchanged.
    """
    out: dict = {}
    for col in _TELEM_COLUMNS + _PARTITION_COLUMNS:
        if col not in sub.columns:
            continue
        series = sub[col]
        if col == "tyreCompound":
            out[col] = series.astype(object).to_numpy()
            continue
        if col == "lap":
            out[col] = series.astype(int).to_numpy()
            continue
        # Coerce to float64. Non-numeric -> NaN.
        try:
            out[col] = series.astype(float).to_numpy()
        except (TypeError, ValueError):
            out[col] = np.asarray(series.values, dtype=object)
    return out


def load_laps_from_lake(
    driver: str,
    car: str,
    track: str,
    n_newest: int = 10,
    *,
    lake_url: str | None = None,
    token: str | None = None,
    track_obj=None,
) -> list:
    """Query QuixLake for the newest `n_newest` finished laps; return merged frames.

    Args:
        driver: Lake `driver` partition value (case-sensitive).
        car: Lake `carModel` partition value.
        track: Lake `track` partition value.
        n_newest: How many newest laps to return. Default 10.
        lake_url, token: Override env vars.
        track_obj: A loaded `Track` (CSV-backed). REQUIRED so we can merge
            telemetry with the per-point track radii. The CLI path resolves
            this from the positional `track_csv` arg; the web/sim path
            passes the already-loaded Track.

    Returns:
        list of merged dicts (one per lap) in the shape `driver_fit.fit_driver`
        consumes.

    Raises:
        RuntimeError: missing env vars, missing pyarrow when server sent Arrow,
            or no matching laps in the lake.
    """
    if track_obj is None:
        raise RuntimeError(
            "load_laps_from_lake: track_obj is required (load the Track CSV "
            "before calling this function)."
        )
    if n_newest < 1:
        raise ValueError(f"n_newest must be >= 1, got {n_newest}")

    # Lazy import the telemetry merger to avoid forcing pyarrow on `lap.py`.
    from .telemetry import merge_with_track  # local

    url, tok, table = _resolve_env(lake_url, token)
    where = (
        f"WHERE driver = '{driver}' "
        f"AND carModel = '{car}' "
        f"AND track = '{track}'"
    )
    sql = _select_columns_sql(table, where)
    print(f"  lake query: SELECT ... WHERE driver={driver}, car={car}, track={track}",
          file=sys.stderr)
    df = _query_sql(sql, lake_url=url, token=tok)
    print(f"  lake returned {len(df)} rows across {df['lap'].nunique() if 'lap' in df.columns else '?'} laps",
          file=sys.stderr)
    bucketed = _bucket_into_laps(df, n_newest=n_newest)
    if not bucketed:
        raise RuntimeError(
            f"lake_loader: no laps with >=50 rows found for "
            f"driver={driver!r}, car={car!r}, track={track!r}."
        )

    merged_frames = []
    for lap_value, sub in bucketed:
        telem = _df_to_telem_dict(sub)
        merged = merge_with_track(telem, track_obj)
        merged_frames.append(merged)
        print(f"  lap {lap_value}: merged {len(merged.get('timestamp_ms', []))} rows", file=sys.stderr)
    return merged_frames
