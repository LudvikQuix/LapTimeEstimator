"""Stream the leaderboard seed sim runs to QuixLake via Quix Streams.

For each (driver, layout) pair we:
  1. Publish a config seed to topic ``ac-telemetry-config`` keyed by
     ``sim_<driver>_<layout>``. The lake's ``join_lookup`` (see
     ``ac-quix-bridge/ac-telemetry-lake/main.py``) reads this config by the
     Kafka message key and joins it onto matching telemetry rows.
  2. Stream the per-tick telemetry rows from the existing
     ``tracks_csv/ks_nurburgring/<layout>_ideal_line__<Driver>_sim_trace_slip.csv``
     CSV (the same files the offline parquet builder uses) to topic
     ``ac-telemetry-raw``, keyed identically.

The lake's ``app.yaml`` declares::

    HIVE_COLUMNS=environment,test_rig,experiment,driver,track,carModel,
                  session_id,lap
    TIMESTAMP_COLUMN=timestamp_ms

``environment``, ``test_rig``, ``experiment``, ``driver``, ``track`` and
``carModel`` come from the config join. ``session_id`` and ``lap`` are NOT in
the join spec, so we must put them in the telemetry body.

``lap`` is computed by the lake as ``completedLaps + 1`` (with NaN filled to
``-1``). We therefore keep ``completedLaps=0`` on every row to keep all rows in
``lap=1``. Lap-time fields (``iLastTime``, ``lastTime``, ``bestTime``,
``iBestTime``) are stamped only on the final row of a completed lap, so the
leaderboard SQL can filter ``WHERE iLastTime IS NOT NULL``. This intentionally
differs from the parquet builder which sets ``completedLaps=1`` on the final
row -- that variant produces a stray row in the ``lap=2`` partition when the
sink-based path is used (the parquet builder bypasses the sink).

Run::

    python scripts/stream_sim_to_quix.py            # full dispatch (6 x 4 = 24)
    python scripts/stream_sim_to_quix.py --dry-run  # build messages, skip publish
    python scripts/stream_sim_to_quix.py --limit 1  # one (driver, layout) only

Environment overrides (otherwise defaults below)::

    Quix__Sdk__Token, Quix__Portal__Api, Quix__Workspace__Id
    CONFIG_TOPIC=ac-telemetry-config
    RAW_TOPIC=ac-telemetry-raw
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path

# ---------------------------------------------------------------------------
# Defaults from the spec / prior diagnostic. Env vars take precedence.
# ---------------------------------------------------------------------------

os.environ.setdefault(
    "Quix__Sdk__Token", "sdk-1828e90c13cc465c9b62918259d1d2b5"
)
os.environ.setdefault("Quix__Portal__Api", "https://portal-api.dev.quix.io")
os.environ.setdefault(
    "Quix__Workspace__Id", "quixdev-acquixbridge-testmanager"
)

CONFIG_TOPIC_NAME = os.environ.get("CONFIG_TOPIC", "ac-telemetry-config")
RAW_TOPIC_NAME = os.environ.get("RAW_TOPIC", "ac-telemetry-raw")

REPO = Path(__file__).resolve().parent.parent
TRACK_DIR = REPO / "tracks_csv" / "ks_nurburgring"

# ---------------------------------------------------------------------------
# Run matrix -- mirrors .tmp/build_seed_parquets.py so the streaming output
# matches the offline parquet fallback row-for-row.
# ---------------------------------------------------------------------------

DRIVERS: list[tuple[str, str, float]] = [
    ("pro_sim", "Pro", 1.00),
    ("expert_sim", "Expert", 0.85),
    ("tomas_sim", "Tomas", 1.00),
    ("amateur_sim", "Amateur", 0.60),
    ("novice_sim", "Novice", 0.30),
    ("learner_sim", "Learner", 0.10),
]

LAYOUTS = ["layout_sprint_a", "layout_sprint_b", "layout_gp_a", "layout_gp_b"]

ENVIRONMENT = "prague_office"
TEST_RIG = "sim"
EXPERIMENT_BASE = "LeaderboardSeed_sim"
TRACK_PARTITION = "ks_nurburgring"
CAR_MODEL = "bmw_1m"
SESSION_ID_ISO = "2026-05-25T00:00:00Z"
TYRE_COMPOUND = "Semislicks (SM)"
TICK_INTERVAL_MS = 20  # sim trace is 50 Hz
LAP_COMPLETE_FRACTION = 0.95


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


@dataclass
class RunSpec:
    driver_key: str       # partition value, e.g. ``pro_sim``
    driver_display: str   # filename token, e.g. ``Pro``
    layout: str           # e.g. ``layout_sprint_a``
    skill_pct: float

    @property
    def key(self) -> str:
        return f"sim_{self.driver_key}_{self.layout}"

    @property
    def experiment_id(self) -> str:
        # Layout scoped into experiment because the ``track`` partition is
        # always ``ks_nurburgring`` for both Sprint and GP layouts.
        return f"{EXPERIMENT_BASE}__{self.layout}"

    @property
    def trace_csv(self) -> Path:
        return (
            TRACK_DIR
            / f"{self.layout}_ideal_line__{self.driver_display}"
            f"_sim_trace_slip.csv"
        )

    @property
    def ideal_line_csv(self) -> Path:
        return TRACK_DIR / f"{self.layout}_ideal_line.csv"


def all_runs() -> list[RunSpec]:
    runs: list[RunSpec] = []
    for driver_key, driver_display, skill in DRIVERS:
        for layout in LAYOUTS:
            runs.append(
                RunSpec(driver_key, driver_display, layout, skill)
            )
    return runs


def format_lap_time(ms: int) -> str:
    """Lake's ``M:SS:mmm`` string format (no leading zero on minutes)."""
    if ms < 0:
        return "-:--:---"
    minutes = ms // 60_000
    seconds = (ms // 1000) % 60
    millis = ms % 1000
    return f"{minutes}:{seconds:02d}:{millis:03d}"


def build_config_payload(run: RunSpec) -> dict:
    """Config seed matching the lake's ``join_lookup`` JSON-path spec.

    From ``ac-telemetry-lake/main.py``::

        $.test_id, $.environment, $.test_rig, $.experiment_id, $.driver
            -> type=experiment
        $.carModel, $.track
            -> type=session

    The lookup is keyed by Kafka message-key, not by a body field.
    """
    return {
        "test_id": run.experiment_id,
        "environment": ENVIRONMENT,
        "test_rig": TEST_RIG,
        "experiment_id": run.experiment_id,
        "driver": run.driver_key,
        "carModel": CAR_MODEL,
        "track": TRACK_PARTITION,
    }


def expected_layout_length_m(layout: str) -> float:
    csv_path = TRACK_DIR / f"{layout}_ideal_line.csv"
    if not csv_path.exists():
        raise FileNotFoundError(csv_path)
    last_dist = 0.0
    with csv_path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                last_dist = float(row["distance_m"])
            except (KeyError, ValueError):
                continue
    return last_dist


def build_telemetry_rows(run: RunSpec) -> tuple[list[dict], dict]:
    """Read the sim trace CSV and yield per-tick telemetry messages.

    Returns (rows, meta). ``meta`` carries lap stats for logging /
    verification.
    """
    if not run.trace_csv.exists():
        raise FileNotFoundError(run.trace_csv)

    expected_m = expected_layout_length_m(run.layout)

    rows: list[dict] = []
    final_dist = 0.0
    final_time_s = 0.0
    with run.trace_csv.open() as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            dist_m = float(row["distance_m"])
            speed_kmh = float(row["sim_speed_kmh"])
            time_s = float(row["time_s"])
            timestamp_ms = i * TICK_INTERVAL_MS
            icurrent_ms = int(round(time_s * 1000.0))

            norm = 0.0
            if expected_m > 0:
                norm = max(0.0, min(1.0, dist_m / expected_m))

            msg = {
                # --- body fields the lake reads directly into Hive columns ---
                "timestamp_ms": timestamp_ms,
                "session_id": SESSION_ID_ISO,
                "completedLaps": 0,           # keep all rows in lap=1
                # --- per-tick telemetry ---
                "distanceTraveled": dist_m,
                "speedKmh": speed_kmh,
                "normalizedCarPosition": norm,
                "currentTime": format_lap_time(icurrent_ms),
                "iCurrentTime": icurrent_ms,
                "lastTime": "-:--:---",
                "bestTime": "-:--:---",
                "iLastTime": None,
                "iBestTime": None,
                "tyreCompound": TYRE_COMPOUND,
            }
            rows.append(msg)
            final_dist = dist_m
            final_time_s = time_s

    completed = expected_m > 0 and final_dist >= LAP_COMPLETE_FRACTION * expected_m
    lap_time_ms = int(round(final_time_s * 1000.0)) if completed else -1

    if completed and rows:
        last = rows[-1]
        last["iLastTime"] = lap_time_ms
        last["iBestTime"] = lap_time_ms
        last["lastTime"] = format_lap_time(lap_time_ms)
        last["bestTime"] = format_lap_time(lap_time_ms)

    meta = {
        "rows": len(rows),
        "completed": completed,
        "lap_time_ms": lap_time_ms if completed else None,
        "lap_time_str": format_lap_time(lap_time_ms) if completed else None,
        "final_dist_m": final_dist,
        "expected_dist_m": expected_m,
    }
    return rows, meta


# ---------------------------------------------------------------------------
# Publish
# ---------------------------------------------------------------------------


def publish_all(runs: list[RunSpec], dry_run: bool) -> dict:
    """Publish config seeds + telemetry for every run. Returns a summary."""
    summary = {
        "config_messages": 0,
        "telemetry_messages": 0,
        "completed_laps": 0,
        "aborted_laps": 0,
        "per_run": [],
    }

    if dry_run:
        producer_ctx = _NullProducer()
    else:
        # Import here so ``--dry-run`` works without the dependency.
        from quixstreams import Application  # type: ignore

        app = Application(
            consumer_group=os.environ.get(
                "CONSUMER_GROUP", "leaderboard_seed_publisher"
            ),
        )
        config_topic = app.topic(
            CONFIG_TOPIC_NAME,
            value_serializer="json",
            key_serializer="str",
        )
        raw_topic = app.topic(
            RAW_TOPIC_NAME,
            value_serializer="json",
            key_serializer="str",
        )
        producer_ctx = _LiveProducer(app, config_topic, raw_topic)

    with producer_ctx as producer:
        for run in runs:
            cfg = build_config_payload(run)
            rows, meta = build_telemetry_rows(run)

            producer.produce_config(run.key, cfg)
            summary["config_messages"] += 1

            for msg in rows:
                producer.produce_raw(run.key, msg)
            summary["telemetry_messages"] += len(rows)

            if meta["completed"]:
                summary["completed_laps"] += 1
            else:
                summary["aborted_laps"] += 1

            summary["per_run"].append(
                {
                    "key": run.key,
                    "rows": meta["rows"],
                    "completed": meta["completed"],
                    "lap_time": meta["lap_time_str"] or "ABORTED",
                }
            )

            print(
                f"  {run.driver_key:12s} {run.layout:17s} "
                f"rows={meta['rows']:5d} "
                f"completed={'Y' if meta['completed'] else 'N'} "
                f"lap={meta['lap_time_str'] or '----'}",
                flush=True,
            )

    return summary


# ---------------------------------------------------------------------------
# Producer wrappers (so --dry-run doesn't import quixstreams).
# ---------------------------------------------------------------------------


class _NullProducer:
    def __enter__(self):
        print("[dry-run] no messages will be published", flush=True)
        return self

    def __exit__(self, *exc):  # noqa: D401
        return False

    def produce_config(self, key: str, value: dict) -> None:  # noqa: D401
        pass

    def produce_raw(self, key: str, value: dict) -> None:  # noqa: D401
        pass


class _LiveProducer:
    def __init__(self, app, config_topic, raw_topic) -> None:
        self._app = app
        self._config_topic = config_topic
        self._raw_topic = raw_topic
        self._producer = None

    def __enter__(self):
        self._producer = self._app.get_producer()
        self._producer.__enter__()
        return self

    def __exit__(self, exc_type, exc, tb):
        return self._producer.__exit__(exc_type, exc, tb)

    def _serialize(self, topic, key: str, value: dict):
        # quixstreams 3.x serializes via topic.serialize before produce.
        return topic.serialize(key=key, value=value)

    def produce_config(self, key: str, value: dict) -> None:
        msg = self._serialize(self._config_topic, key, value)
        self._producer.produce(
            topic=self._config_topic.name,
            key=msg.key,
            value=msg.value,
        )

    def produce_raw(self, key: str, value: dict) -> None:
        msg = self._serialize(self._raw_topic, key, value)
        self._producer.produce(
            topic=self._raw_topic.name,
            key=msg.key,
            value=msg.value,
        )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Build messages and print stats, but do not publish.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Process only the first N runs (0 = all 24).",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Optional path to dump the publish summary as JSON.",
    )
    args = parser.parse_args(argv)

    runs = all_runs()
    if args.limit > 0:
        runs = runs[: args.limit]

    missing = [r for r in runs if not r.trace_csv.exists()]
    if missing:
        for r in missing:
            print(f"MISSING TRACE: {r.trace_csv}", file=sys.stderr)
        return 2

    print(
        f"Workspace: {os.environ.get('Quix__Workspace__Id')}  "
        f"Portal: {os.environ.get('Quix__Portal__Api')}",
        flush=True,
    )
    print(
        f"Config topic: {CONFIG_TOPIC_NAME}  "
        f"Raw topic: {RAW_TOPIC_NAME}  "
        f"Runs: {len(runs)}  "
        f"dry_run={args.dry_run}",
        flush=True,
    )

    summary = publish_all(runs, dry_run=args.dry_run)

    print(
        "\nSummary: "
        f"{summary['config_messages']} config + "
        f"{summary['telemetry_messages']} telemetry rows "
        f"({summary['completed_laps']} completed / "
        f"{summary['aborted_laps']} aborted)",
        flush=True,
    )

    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(json.dumps(summary, indent=2))
        print(f"Wrote summary -> {args.summary_json}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
