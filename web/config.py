"""Central config for the lap-estimator-web service (spec §22.5).

Mirrors `ac-quix-bridge/telemetry-comparison`'s convention: read env vars at
module-import time, expose them as attributes. Consumers read attributes off
this module so tests can monkeypatch values.
"""
from __future__ import annotations

import os
from pathlib import Path

try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:  # python-dotenv is optional in non-local deploys
    pass

# Web service base dir (this file's parent).
BASE_DIR = Path(__file__).parent
STATIC_DIR = BASE_DIR / "static"
TEMPLATES_DIR = BASE_DIR / "templates"

# Repo root: where cars_csv/, tracks_csv/, drivers/, setups/ live.
# Default is the repo root one level up. Quix Cloud overrides to /app.
REPO_ROOT = Path(os.getenv("LAP_ESTIMATOR_REPO_ROOT", BASE_DIR.parent)).resolve()
CARS_DIR = REPO_ROOT / "cars_csv"
TRACKS_DIR = REPO_ROOT / "tracks_csv"
DRIVERS_DIR = REPO_ROOT / "drivers"
SETUPS_DIR = REPO_ROOT / "setups"
SAMPLES_DIR = REPO_ROOT / "samples"

# QuixLake transport. Mirrors telemetry-comparison.
TABLE_NAME = os.getenv("TABLE_NAME", "ac_telemetry")
QUIXLAKE_URL = os.getenv("QUIXLAKE_URL")
QUIX_LAKE_TOKEN = os.getenv("QUIX_LAKE_TOKEN")

# Logging.
LOGLEVEL = os.getenv("LOGLEVEL", "INFO")

# Sim job artefact TTL (seconds). 60 min per spec §22.A.
JOB_TTL_SECONDS = 60 * 60

# Lake tree cache TTL (seconds). 60 s per spec §22.F.
LAKE_TREE_TTL_SECONDS = 60

# Arrow probe override. unset=auto via per-request content-type dispatch.
LAKE_ARROW_FORCE = os.getenv("LAKE_ARROW_FORCE")


def require_lake_env() -> None:
    """Raise RuntimeError if lake env vars are missing.

    Called by lake-touching endpoints; lets the rest of the API stay usable
    in offline / local-only mode (e.g. running the Sim tab without the lake).
    """
    missing = [
        name
        for name, val in (
            ("QUIXLAKE_URL", QUIXLAKE_URL),
            ("QUIX_LAKE_TOKEN", QUIX_LAKE_TOKEN),
        )
        if not val
    ]
    if missing:
        raise RuntimeError(
            f"Missing required env var(s): {', '.join(missing)}. "
            "Set them in .env or the environment before starting the service."
        )
