"""In-process wrappers around lap_estimator (spec §22.B.2).

All heavy compute runs in a worker thread via `asyncio.to_thread(...)` so
FastAPI's event loop stays responsive. The `fit_driver` path is async-
generator: a worker thread iterates the sync pipeline, pumps `FitStage`
objects through a `queue.Queue`, and the async generator yields them.
"""

from __future__ import annotations

import asyncio
import math
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd

# Make `src/` importable for in-process calls.
_SRC = Path(__file__).resolve().parent.parent / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from lap_estimator.car import Car  # noqa: E402
from lap_estimator.driver import Driver  # noqa: E402
from lap_estimator.setup import (  # noqa: E402
    resolve_compound,
    resolve_setup,
)
from lap_estimator.sim_telemetry import write_synthetic_log  # noqa: E402
from lap_estimator.simulator import simulate, simulate_stint  # noqa: E402
from lap_estimator.solve_setup import solve_pressure_for_wear  # noqa: E402
from lap_estimator.track import Track  # noqa: E402
from lap_estimator.validate import validate_lap  # noqa: E402

from . import config, jobs  # noqa: E402


def _car_path(car_name: str) -> str:
    """Resolve `car_name` -> car data dir. Cars live under cars_csv/<name>/."""
    base = config.CARS_DIR / car_name
    if not base.is_dir():
        raise FileNotFoundError(f"Unknown car: {car_name} (looked under {base})")
    # AC car-data is a flat dir of *.ini + *.lut files.
    return str(base)


def _track_path(track_arg: str) -> str:
    """Resolve `track_arg` (e.g. 'ks_nurburgring/layout_sprint_a') -> CSV path."""
    base = config.TRACKS_DIR / (track_arg + ".csv")
    if not base.is_file():
        raise FileNotFoundError(f"Unknown track: {track_arg} (looked at {base})")
    return str(base)


def _driver_path(driver_name: str) -> str:
    base = config.DRIVERS_DIR / f"{driver_name}.json"
    if not base.is_file():
        raise FileNotFoundError(
            f"Unknown driver: {driver_name} (looked at {base})"
        )
    return str(base)


# ---------------------------------------------------------------------------
# /api/sim
# ---------------------------------------------------------------------------


async def run_sim(req: dict) -> dict:
    """Async wrapper for `simulate_stint`. Returns the API response dict."""
    return await asyncio.to_thread(_sim_sync, req)


def _sim_sync(req: dict) -> dict:
    car_name = req["car"]
    track_arg = req["track"]
    driver_name = req["driver"]
    n_laps = int(req["n_laps"])
    if not 1 <= n_laps <= 50:
        raise ValueError(f"n_laps must be in [1, 50], got {n_laps}")
    ds = float(req.get("ds") or 2.0)
    telemetry_dt_ms = int(req.get("telemetry_dt_ms") or 10)
    pressures_psi = req.get("pressures_psi") or {}
    ambient_temp_c = req.get("ambient_temp_c")
    compound_name = req.get("compound")
    # v3 spec §23.4.2: the Sim-tab Model dropdown sends a `model` field.
    # Phase 3+: slip model dispatches to dynamics.simulate_stint_slip.
    model = req.get("model") or "point-mass"
    if model not in ("point-mass", "slip"):
        raise ValueError(f"model must be one of point-mass|slip, got {model!r}")

    car = Car(_car_path(car_name))
    track = Track.from_csv(_track_path(track_arg))
    driver = Driver.load(_driver_path(driver_name))
    calibration = driver.get_tyre_calibration()

    compound, compound_source = resolve_compound(car, cli_name=compound_name)

    # Build the setup from the request. We piggyback on `resolve_setup` for
    # the cold-pressure defaults (compound's PRESSURE_STATIC).
    pressure_str = (
        ",".join(f"{w}={float(pressures_psi[w])}" for w in pressures_psi)
        if pressures_psi else None
    )
    setup = resolve_setup(
        car, None, pressure_str, ambient_temp_c, compound=compound,
    )

    slip_extras: dict = {}
    if model == "slip":
        # v3 Phase 4: real preview-target slip-aware driver controller. The
        # SlipSimResult carries util_p85 / slip_target_rad / MC stats; we
        # surface those in the response payload below.
        from lap_estimator.dynamics import simulate_slip as _simulate_slip
        slip_result = _simulate_slip(
            car, track, driver,
            setup=setup, compound=compound,
            n_laps=n_laps, ds=ds,
        )
        stint = _build_slip_stint_shim(slip_result, setup, calibration, compound)
        slip_extras = {
            "util_p85": round(float(slip_result.util_p85), 4),
            "slip_target_deg": round(math.degrees(float(slip_result.slip_target_rad)), 3),
            "used_ghost": bool(slip_result.used_ghost),
            "fallback_front_to_rear": bool(slip_result.fallback_front_to_rear),
            "mc_n_runs": int(slip_result.mc_n_runs),
            "mc_sigma_s": round(float(slip_result.mc_sigma_s), 4),
        }
    else:
        stint = simulate_stint(
            car, track, driver,
            n_laps=n_laps, setup=setup, calibration=calibration,
            compound=compound, ds=ds,
        )

    # Build artefact DataFrames into a job slot.
    job = jobs.new_job("sim")
    job.telemetry_df = _stint_to_telemetry_df(
        stint, car, driver, track, telemetry_dt_ms=telemetry_dt_ms,
    )
    job.trace_df = _stint_to_trace_df(stint)
    job.stint_summary_df = _stint_to_summary_df(stint)
    job.summary = {
        "n_laps": n_laps,
        "lap_times_s": [round(float(t), 3) for t in stint.lap_times_s],
        "compound_resolved": f"{compound.name} (idx {compound.index})",
        "compound_source": compound_source,
        "setup_line": setup.fmt_line(),
    }

    response = {
        "job_id": job.job_id,
        "lap_times_s": job.summary["lap_times_s"],
        "telemetry_csv_url": f"/api/sim/result/{job.job_id}/telemetry.csv",
        "trace_csv_url": f"/api/sim/result/{job.job_id}/trace.csv",
        "stint_summary_csv_url": f"/api/sim/result/{job.job_id}/stint_summary.csv",
        "plot_png_url": f"/api/sim/result/{job.job_id}/plot.png",
        "compound_resolved": job.summary["compound_resolved"],
        "compound_source": compound_source,
        "setup_line": job.summary["setup_line"],
    }
    response.update(slip_extras)
    return response


def _build_slip_stint_shim(slip_result, setup, calibration, compound):
    """Wrap a :class:`SlipSimResult` in a StintResult-compatible shape.

    Lets ``_stint_to_telemetry_df`` / ``_stint_to_trace_df`` /
    ``_stint_to_summary_df`` / ``write_synthetic_log`` consume the v3
    output without branching. Per-lap tyre state is held constant in
    Phase 3 (no in-lap thermal evolution); telemetry will show flat
    wear/temp/pressure traces — Phase 5 layers per-lap evolution back in.
    """
    from lap_estimator.simulator import StintResult
    from lap_estimator.tyre_state import TyreState
    initial_state = TyreState.from_setup(setup)
    history = [initial_state.copy() for _ in range(slip_result.n_laps + 1)]
    # Constant-state per-point arrays for telemetry emission.
    per_point_states = []
    for k in range(slip_result.n_laps):
        n_lap = int(np.sum(slip_result.lap_id == (k + 1))) if slip_result.lap_id is not None else 0
        if n_lap <= 0:
            continue
        per_point_states.append(_slip_constant_state_arrays(initial_state, n_lap))
    return StintResult(
        n_laps=slip_result.n_laps,
        setup=setup,
        calibration=calibration,
        lap_times_s=list(slip_result.lap_times_s),
        tyre_state_history=history,
        per_lap_sim_results=[slip_result] * slip_result.n_laps,
        per_point_states=per_point_states,
        compound=compound,
    )


def _slip_constant_state_arrays(state, n_pts):
    """Build the constant-state per-point dict the telemetry writer expects."""
    return {
        "temp_C": {w: np.full(n_pts, state.temp_C[w]) for w in ("FL", "FR", "RL", "RR")},
        "wear_pct": {w: np.full(n_pts, state.wear_pct[w]) for w in ("FL", "FR", "RL", "RR")},
        "pressure_psi": {w: np.full(n_pts, state.pressure_psi[w]) for w in ("FL", "FR", "RL", "RR")},
    }


def _stint_to_telemetry_df(stint, car, driver, track, *, telemetry_dt_ms: int):
    """Run sim_telemetry to a temp CSV, then parse it back as a DataFrame.

    `sim_telemetry.write_synthetic_log` writes to a path; we round-trip
    through a temp file rather than reimplement its emitter logic.
    """
    import tempfile
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".csv", delete=False, newline=""
    ) as tmp:
        tmp_path = tmp.name
    try:
        write_synthetic_log(
            stint, car, driver, track.total_length_m, tmp_path,
            telemetry_dt_ms=telemetry_dt_ms,
        )
        df = pd.read_csv(tmp_path)
    finally:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
    return df


def _stint_to_trace_df(stint) -> pd.DataFrame:
    """Per-point trace CSV (per the v1.1 trace schema)."""
    # Concatenate per-lap SimResults' arrays.
    rows = []
    for k, lap_result in enumerate(stint.per_lap_sim_results):
        # Back-compat: when stint used the legacy two-lap fast path, the same
        # SimResult is repeated twice across per_lap_sim_results -- skip the
        # duplicate to avoid 2x rows.
        if k > 0 and lap_result is stint.per_lap_sim_results[k - 1]:
            continue
        n = len(lap_result.distances)
        ai = lap_result.ai_speeds
        lap_id = lap_result.lap_id if lap_result.lap_id is not None else np.full(n, k + 1)
        for i in range(n):
            rows.append({
                "lap": int(lap_id[i]) if lap_result.lap_id is not None else k + 1,
                "distance_m": round(float(lap_result.distances[i]), 3),
                "sim_speed_ms": round(float(lap_result.speeds[i]), 4),
                "sim_speed_kmh": round(float(lap_result.speeds[i]) * 3.6, 4),
                "ai_speed_kmh": (
                    "" if ai is None else round(float(ai[i]) * 3.6, 4)
                ),
                "time_s": (
                    round(float(lap_result.times[i]), 4)
                    if len(lap_result.times) else 0.0
                ),
            })
    return pd.DataFrame(rows)


def _stint_to_summary_df(stint) -> pd.DataFrame:
    """Per-lap stint summary block: lap, time, wear/temp/pressure aggregates."""
    rows = []
    for k in range(stint.n_laps):
        end = stint.tyre_state_history[k + 1]
        rows.append({
            "lap": k + 1,
            "lap_time_s": round(float(stint.lap_times_s[k]), 3),
            "wear_FL": round(float(end.wear_pct["FL"]), 2),
            "wear_FR": round(float(end.wear_pct["FR"]), 2),
            "wear_RL": round(float(end.wear_pct["RL"]), 2),
            "wear_RR": round(float(end.wear_pct["RR"]), 2),
            "temp_avg_C": round(end.avg_temp_C(), 2),
            "pressure_avg_psi": round(end.avg_pressure_psi(), 2),
        })
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# /api/solve
# ---------------------------------------------------------------------------


async def run_solve(req: dict) -> dict:
    return await asyncio.to_thread(_solve_sync, req)


def _solve_sync(req: dict) -> dict:
    car = Car(_car_path(req["car"]))
    track = Track.from_csv(_track_path(req["track"]))
    driver = Driver.load(_driver_path(req["driver"]))
    calibration = driver.get_tyre_calibration()
    compound, _src = resolve_compound(car, cli_name=req.get("compound"))
    target_wear = float(req["target_wear"])
    target_lap = int(req["target_lap"])
    target_wheel = str(req.get("target_wheel") or "max")
    uniform = bool(req.get("uniform_pressure") or False)
    ambient = float(req.get("ambient_temp_c") or 25.0)

    sr = solve_pressure_for_wear(
        car, track, driver,
        target_wear=target_wear,
        target_lap=target_lap,
        target_wheel=target_wheel,
        uniform=uniform,
        calibration=calibration,
        ambient_temp_C=ambient,
        compound=compound,
    )

    verification = []
    for k in range(sr.verification_stint.n_laps):
        end = sr.verification_stint.tyre_state_history[k + 1]
        verification.append({
            "lap": k + 1,
            "lap_time_s": round(float(sr.verification_stint.lap_times_s[k]), 3),
            "wear_FL": round(float(end.wear_pct["FL"]), 2),
            "wear_FR": round(float(end.wear_pct["FR"]), 2),
            "wear_RL": round(float(end.wear_pct["RL"]), 2),
            "wear_RR": round(float(end.wear_pct["RR"]), 2),
            "temp_avg_C": round(end.avg_temp_C(), 2),
            "pressure_avg_psi": round(end.avg_pressure_psi(), 2),
        })

    job = jobs.new_job("solve")
    job.stint_summary_df = pd.DataFrame(verification)
    job.summary = {
        "target_wear": target_wear,
        "target_lap": target_lap,
        "iterations": sr.iterations,
        "converged": bool(sr.converged),
    }

    return {
        "job_id": job.job_id,
        "recommended_pressures_psi": {
            k: round(float(v), 2) for k, v in sr.recommended_psi.items()
        },
        "target_wheel_resolved": sr.target_wheel_resolved,
        "iterations": int(sr.iterations),
        "converged": bool(sr.converged),
        "observed_wear": round(float(sr.observed_wear), 4),
        "verification_stint": verification,
        "verification_csv_url": f"/api/sim/result/{job.job_id}/stint_summary.csv",
    }


# ---------------------------------------------------------------------------
# /api/validate
# ---------------------------------------------------------------------------


async def run_validate(req: dict) -> dict:
    return await asyncio.to_thread(_validate_sync, req)


def _validate_sync(req: dict) -> dict:
    car = Car(_car_path(req["car"]))
    track = Track.from_csv(_track_path(req["track"]))
    driver = Driver.load(_driver_path(req["driver"]))
    calibration = driver.get_tyre_calibration()
    compound, _src = resolve_compound(car, cli_name=req.get("compound"))

    bin_m = int(req.get("bin_m") or 100)
    per_corner = bool(req.get("per_corner") or False)

    # Build a 2-lap sim for the validation overlay (target lap 2 -- flying).
    sim_result = simulate(car, track, driver, ds=2.0, two_lap=True)

    real = req["real_source"]
    if real["kind"] == "local":
        real_path = str((config.REPO_ROOT / real["path"]).resolve())
    elif real["kind"] == "lake":
        # Materialise a single lap from the lake to a temp CSV.
        from lap_estimator.lake_loader import load_laps_from_lake
        frames = load_laps_from_lake(
            driver=str(real["driver"]),
            car=str(real["car"]),
            track=str(real["track"]),
            n_newest=int(real.get("lap") or 1),
            track_obj=track,
        )
        # Pick the matching lap by index (1-based).
        idx = int(real.get("lap") or 1) - 1
        frame = frames[max(0, min(idx, len(frames) - 1))]
        real_path = _merged_to_tempcsv(frame)
    else:
        raise ValueError(f"validate: real_source.kind must be 'local' or 'lake', got {real['kind']!r}")

    vr = validate_lap(
        car, track, sim_result, real_path,
        bin_m=bin_m, per_corner=per_corner, target_lap=2,
    )

    bins_rows = []
    for b in vr.bins:
        bins_rows.append({
            "bin_start_m": round(float(b["bin_start_m"]), 2),
            "bin_end_m": round(float(b["bin_end_m"]), 2),
            "kind": b["kind"],
            "t_sim_s": round(float(b["t_sim_s"]), 4),
            "t_real_s": round(float(b["t_real_s"]), 4),
            "delta_s": round(float(b["delta_s"]), 4),
            "v_avg_sim_kmh": round(float(b["v_avg_sim_kmh"]), 2),
            "v_avg_real_kmh": round(float(b["v_avg_real_kmh"]), 2),
        })

    job = jobs.new_job("validate")
    job.validation_bins_df = pd.DataFrame(bins_rows)
    job.summary = {
        "real_lap_time_s": round(float(vr.real_lap_time_s), 3),
        "sim_lap_time_s": round(float(vr.sim_lap_time_s), 3),
        "delta_s": round(float(vr.delta_s), 3),
        "verdict": vr.verdict,
    }

    return {
        "job_id": job.job_id,
        "real_lap_time_s": round(float(vr.real_lap_time_s), 3),
        "sim_lap_time_s": round(float(vr.sim_lap_time_s), 3),
        "delta_s": round(float(vr.delta_s), 3),
        "delta_pct": round(float(vr.delta_pct), 3),
        "verdict": vr.verdict,
        "validation_bins_csv_url": f"/api/sim/result/{job.job_id}/validation_bins.csv",
    }


def _merged_to_tempcsv(merged: dict) -> str:
    """Write a merged-frame dict back to a CSV in AC-log columns for validate_lap."""
    import tempfile
    df = pd.DataFrame({
        "timestamp_ms": merged["timestamp_ms"],
        "gas": merged["gas"],
        "brake": merged["brake"],
        "distanceTraveled": merged["distance_m"],
        "speedKmh": merged["speedKmh"],
        "normalizedCarPosition": merged["normalizedCarPosition"],
    })
    # Drop NaNs in any required col.
    df = df.dropna()
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".csv", delete=False, newline="",
    )
    df.to_csv(tmp.name, index=False)
    tmp.close()
    return tmp.name


# The SSE pump for /api/fit_driver lives in `fit_runner.py` to keep this
# module under the 500-line soft cap. `fit_routes.py` imports from there.
