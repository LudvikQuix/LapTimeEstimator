"""In-memory job store for sim/solve/validate artefacts (spec §22).

A `Job` carries the result DataFrames + summary metadata. Eviction is lazy:
on each access we drop entries older than `JOB_TTL_SECONDS`. Sufficient for
single-user workspace use; no Redis required.
"""

from __future__ import annotations

import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any

import pandas as pd

from . import config


@dataclass
class Job:
    job_id: str
    kind: str  # "sim" | "solve" | "validate" | "fit"
    created_at: float
    # Per-kind payload buckets. None when not applicable.
    telemetry_df: pd.DataFrame | None = None
    trace_df: pd.DataFrame | None = None
    stint_summary_df: pd.DataFrame | None = None
    validation_bins_df: pd.DataFrame | None = None
    plot_png: bytes | None = None
    overlay_png: bytes | None = None
    summary: dict[str, Any] = field(default_factory=dict)


_jobs: dict[str, Job] = {}
_lock = threading.Lock()


def _evict_expired_locked() -> None:
    now = time.time()
    ttl = config.JOB_TTL_SECONDS
    expired = [jid for jid, j in _jobs.items() if now - j.created_at > ttl]
    for jid in expired:
        del _jobs[jid]


def new_job(kind: str) -> Job:
    """Create + register a new Job. Returns the Job (caller fills payload)."""
    job = Job(job_id=uuid.uuid4().hex, kind=kind, created_at=time.time())
    with _lock:
        _evict_expired_locked()
        _jobs[job.job_id] = job
    return job


def get(job_id: str) -> Job | None:
    with _lock:
        _evict_expired_locked()
        return _jobs.get(job_id)


def require(job_id: str) -> Job:
    job = get(job_id)
    if job is None:
        raise KeyError(f"Job {job_id} not found or expired (>{config.JOB_TTL_SECONDS}s)")
    return job


def drop(job_id: str) -> None:
    with _lock:
        _jobs.pop(job_id, None)


def count() -> int:
    with _lock:
        _evict_expired_locked()
        return len(_jobs)
