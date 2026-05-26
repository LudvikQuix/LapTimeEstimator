"""Build WHERE clauses from partition column values.

VENDORED from ac-quix-bridge/telemetry-comparison/partition_filter.py per
spec §22.6. Unmodified (no internal imports to swap).

Used by /api/telemetry to filter DuckDB queries on the Hive-partitioned
Parquet files. The allowlist regex prevents SQL injection via `{val}`
interpolation.
"""

from __future__ import annotations

import re

_SAFE_PARTITION_VALUE = re.compile(r"^[A-Za-z0-9_\-.: ]+$")


def _build_partition_filter(**kwargs) -> str:
    """Build a WHERE clause from partition column values.

    Skips empty strings. Uses CAST for session_id to handle DuckDB timestamp
    normalization vs Hive partition format.

    Raises ValueError on any string value that doesn't match
    `_SAFE_PARTITION_VALUE`.
    """
    clauses = []
    for col, val in kwargs.items():
        if val is None or val == "":
            continue
        if isinstance(val, int):
            clauses.append(f"{col} = {val}")
            continue
        if not _SAFE_PARTITION_VALUE.fullmatch(str(val)):
            raise ValueError(f"Invalid character in {col}: {val!r}")
        if col == "session_id":
            prefix = val.replace("T", " ").rstrip("Z").rstrip("0").rstrip(".")
            escaped = (
                prefix.replace("\\", "\\\\")
                .replace("%", "\\%")
                .replace("_", "\\_")
            )
            clauses.append(
                f"CAST(session_id AS VARCHAR) LIKE '{escaped}%' ESCAPE '\\'"
            )
        else:
            clauses.append(f"{col} = '{val}'")
    return ("WHERE " + " AND ".join(clauses)) if clauses else ""
