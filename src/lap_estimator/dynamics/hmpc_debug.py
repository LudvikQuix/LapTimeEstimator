"""HMPC diagnostic CSV writers (spec §23.4.6.5).

Two trace files are produced when ``--hmpc-debug-trace`` is set:

- ``hmpc_outer_trace_<track>_<driver>.csv`` — one row per outer
  planner solve. Columns: ``t_solve, status, t_solve_ms, sqp_iters,
  s_start, s_end, v_start, v_ref_min, n_ref_p95``.
- ``hmpc_inner_trace_<track>_<driver>.csv`` — one row per inner
  tick. Columns: ``t, s, v_x, outer_age_ticks, outer_fired,
  v_ref_at_s, n_ref_at_s, thr_inner_emit, brk_inner_emit, tier,
  inner_solve_ms``.

The writers are intentionally **append-once at end of run** (no
streaming). This keeps the per-tick path free of disk-write contention
and is consistent with v3.2 / v3.3 patterns. Files are overwritten on
each run.
"""

from __future__ import annotations

import csv
import logging
import os
from typing import Iterable, Mapping

log = logging.getLogger(__name__)


def write_inner_trace_csv(path: str, rows: Iterable[Mapping[str, object]]) -> None:
    """Write the inner-tick trace CSV.

    Parameters
    ----------
    path : str
        Output file path. Parent directory created if missing.
    rows : iterable of dict-likes
        Per-tick records. The first row's keys define the column order.
    """
    rows_list = list(rows)
    if not rows_list:
        log.info("HMPC inner trace: no rows to write at %s", path)
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    keys = list(rows_list[0].keys())
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=keys)
        writer.writeheader()
        for r in rows_list:
            writer.writerow({k: r.get(k, "") for k in keys})
    log.info("HMPC inner trace written: %s (%d rows)", path, len(rows_list))


def write_outer_trace_csv(path: str, rows: Iterable[Mapping[str, object]]) -> None:
    """Write the outer-solve trace CSV (one row per outer solve)."""
    rows_list = list(rows)
    if not rows_list:
        log.info("HMPC outer trace: no rows to write at %s", path)
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    keys = list(rows_list[0].keys())
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=keys)
        writer.writeheader()
        for r in rows_list:
            writer.writerow({k: r.get(k, "") for k in keys})
    log.info("HMPC outer trace written: %s (%d rows)", path, len(rows_list))


__all__ = ["write_inner_trace_csv", "write_outer_trace_csv"]
