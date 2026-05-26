"""High-level fit orchestration generator (spec §22.B.5, v3-aux).

`fit_driver_pipeline(...)` is the single source of truth for the driver-
fitting flow. It yields one `FitStage` per milestone so both the CLI
(`fit_driver.py --from-lake` / positional CSV mode) and the web SSE
endpoint (`web/sim_runner.py`) can consume the same events. Splitting it
out of `driver_fit.py` keeps that module focused on the math
(`fit_driver` -> `FitResult`) and this one on plumbing.
"""

from __future__ import annotations

import datetime as _dt
import json as _json
import os as _os
from dataclasses import dataclass

# Lazy import inside the pipeline to break a circular import: `driver_fit.py`
# re-exports `FitStage` + `fit_driver_pipeline` from this module for back-
# compat. Hoisting `fit_driver` to module level would re-enter
# `driver_fit.py` during its own import.


@dataclass
class FitStage:
    """One stage event from `fit_driver_pipeline`.

    `payload` is a plain dict serialisable to JSON; the web layer uses it
    directly as the SSE `data:` line. Stage names per spec §22.A.4:
    lake_query_started, lake_query_done, merged_done, skill_fit_done,
    tyre_calibration_done, validation_done, written.
    """
    name: str
    payload: dict


def fit_driver_pipeline(
    car,
    track,
    *,
    output_json: str,
    name: str | None = None,
    csv_paths: list[str] | None = None,
    lake_args: dict | None = None,
    do_validate: bool = True,
    overwrite: bool = False,
    ds: float = 2.0,
    lap_choice: int = 2,
):
    """High-level fit pipeline; yields `FitStage` events as it progresses.

    Inputs are mutually exclusive: pass either `csv_paths` (list of local
    AC telemetry CSV paths) OR `lake_args` (``{"driver": ..., "car": ...,
    "track": ..., "n_newest": N}``).

    Side effect: writes `output_json` on the `written` stage.
    """
    if (csv_paths is None) == (lake_args is None):
        raise ValueError(
            "fit_driver_pipeline: pass exactly one of csv_paths or lake_args."
        )

    if _os.path.exists(output_json) and not overwrite:
        raise FileExistsError(
            f"{output_json} already exists; pass overwrite=True to replace."
        )

    merged_frames, sources, lake_stages = _load_frames(track, csv_paths, lake_args, lap_choice)
    yield from lake_stages

    yield FitStage("merged_done", {
        "n_laps": len(merged_frames),
        "rows": int(sum(len(f.get("timestamp_ms", [])) for f in merged_frames)),
    })

    if len(merged_frames) < 2:
        raise ValueError("fit requires >=2 laps; see spec §13")

    # Math. Local import (circular-import workaround — see module header).
    from .driver_fit import fit_driver
    fit = fit_driver(car, merged_frames, track=track)
    yield FitStage("skill_fit_done", {
        "skill_pct": round(float(fit.skill_pct), 6),
        "consistency_sigma": round(float(fit.consistency_sigma), 6),
        "util_p85": round(float(fit.util_p85), 6),
        "n_corner_samples": int(fit.n_corner_samples),
        "n_laps": int(fit.n_laps),
    })
    tc = fit.tyre_calibration
    yield FitStage("tyre_calibration_done", {
        "measured": bool(tc.measured) if tc is not None else False,
        "rmse_temp_C": _round_maybe(fit.tyre_calibration_rmse.get("rmse_temp_C"), 4)
            if fit.tyre_calibration_rmse else None,
        "rmse_wear_pct": _round_maybe(fit.tyre_calibration_rmse.get("rmse_wear_pct"), 4)
            if fit.tyre_calibration_rmse else None,
        "rmse_pressure_psi": _round_maybe(fit.tyre_calibration_rmse.get("rmse_pressure_psi"), 4)
            if fit.tyre_calibration_rmse else None,
        "compound": fit.compound_name,
    })

    # Build payload.
    derived_name = name or _derive_name(output_json)
    profile_block, top_level = _profile_payload(fit.profile)
    payload = _build_payload(
        fit, derived_name, sources,
        track_csv=getattr(track, "source_path", None) or "",
        car_data_dir=getattr(car, "data_dir", ""),
        profile_block=profile_block,
        top_level=top_level,
    )

    if do_validate:
        yield from _run_validation(car, track, fit, derived_name, top_level, payload, ds)

    # Write.
    _os.makedirs(_os.path.dirname(output_json) or ".", exist_ok=True)
    with open(output_json, "w") as f:
        _json.dump(payload, f, indent=2)
        f.write("\n")
    yield FitStage("written", {
        "path": output_json.replace("\\", "/"),
        "fit_version": payload["source"]["fit_version"],
        "name": derived_name,
    })


def _load_frames(track, csv_paths, lake_args, lap_choice):
    """Return (merged_frames, source_tags, [pre-merge stages])."""
    if lake_args is not None:
        stage_started = FitStage("lake_query_started", {
            "driver": lake_args.get("driver"),
            "car": lake_args.get("car"),
            "track": lake_args.get("track"),
            "n_newest": int(lake_args.get("n_newest", 10)),
        })
        from .lake_loader import load_laps_from_lake
        merged_frames = load_laps_from_lake(
            driver=str(lake_args["driver"]),
            car=str(lake_args["car"]),
            track=str(lake_args["track"]),
            n_newest=int(lake_args.get("n_newest", 10)),
            lake_url=lake_args.get("lake_url"),
            token=lake_args.get("token"),
            track_obj=track,
        )
        sources = [
            f"lake://{lake_args['driver']}/{lake_args['car']}/{lake_args['track']}/lap{i + 1}"
            for i in range(len(merged_frames))
        ]
        stage_done = FitStage("lake_query_done", {
            "n_laps": len(merged_frames),
            "rows": int(sum(len(f.get("timestamp_ms", [])) for f in merged_frames)),
        })
        return merged_frames, sources, [stage_started, stage_done]

    from .telemetry import filter_to_lap, merge_with_track, read_ac_log
    merged_frames = []
    sources = []
    for path in csv_paths:
        telem = read_ac_log(path)
        if "lap" in telem:
            telem = filter_to_lap(telem, lap_choice)
        merged = merge_with_track(telem, track)
        merged_frames.append(merged)
        sources.append(path.replace("\\", "/"))
    return merged_frames, sources, []


def _run_validation(car, track, fit, derived_name, top_level, payload, ds):
    """Re-sim with the freshly-fit driver and capture lap-2 time. Yields stage."""
    from .driver import Driver
    from .simulator import simulate
    driver_obj = Driver(
        name=derived_name,
        skill_pct=fit.skill_pct,
        consistency_sigma=fit.consistency_sigma,
        driver_tau_s=top_level["driver_tau_s"],
        trail_brake_m=top_level["trail_brake_m"],
        throttle_ramp_m=top_level["throttle_ramp_m"],
        raw=payload,
    )
    result = simulate(car, track, driver_obj, ds=ds, two_lap=True)
    sim_lap_s = float(result.lap2_time if result.two_lap else result.lap_time)
    payload["source"]["sim_lap_time_s"] = round(sim_lap_s, 3)
    if fit.real_lap_time_s is not None:
        payload["source"]["delta_s"] = round(sim_lap_s - float(fit.real_lap_time_s), 3)
    yield FitStage("validation_done", {
        "real_lap_time_s": (
            round(float(fit.real_lap_time_s), 3)
            if fit.real_lap_time_s is not None else None
        ),
        "sim_lap_time_s": round(sim_lap_s, 3),
        "delta_s": (
            round(sim_lap_s - float(fit.real_lap_time_s), 3)
            if fit.real_lap_time_s is not None else None
        ),
    })


def _round_maybe(value, places):
    if value is None:
        return None
    return round(float(value), places)


def _derive_name(output_path):
    base = _os.path.splitext(_os.path.basename(output_path))[0]
    return base.replace(":", "_").replace(".", "_")


def _norm(path):
    return path.replace("\\", "/")


def _profile_payload(profile):
    """Translate ProfileDynamics -> JSON `profile.dynamic` + top-level mirrors."""
    from .driver import (
        DEFAULT_DRIVER_TAU_S,
        DEFAULT_THROTTLE_RAMP_M,
        DEFAULT_TRAIL_BRAKE_M,
    )
    dynamic_block = {
        "driver_tau_s": _round_maybe(profile.driver_tau_s, 4),
        "trail_brake_m": _round_maybe(profile.trail_brake_m, 2),
        "throttle_ramp_m": _round_maybe(profile.throttle_ramp_m, 2),
        "pedal_press_rate_per_s": _round_maybe(profile.pedal_press_rate_per_s, 4),
        "steering_aggression_deg_per_s": _round_maybe(
            profile.steering_aggression_deg_per_s, 2
        ),
        "measured": dict(profile.measured),
        "sample_counts": dict(profile.sample_counts),
    }
    top_level = {
        "driver_tau_s": (
            profile.driver_tau_s
            if profile.measured.get("driver_tau_s") and profile.driver_tau_s is not None
            else DEFAULT_DRIVER_TAU_S
        ),
        "trail_brake_m": (
            profile.trail_brake_m
            if profile.measured.get("trail_brake_m") and profile.trail_brake_m is not None
            else DEFAULT_TRAIL_BRAKE_M
        ),
        "throttle_ramp_m": (
            profile.throttle_ramp_m
            if profile.measured.get("throttle_ramp_m") and profile.throttle_ramp_m is not None
            else DEFAULT_THROTTLE_RAMP_M
        ),
    }
    return {"dynamic": dynamic_block}, top_level


def _build_payload(fit, name, sources, *, track_csv, car_data_dir,
                   profile_block, top_level):
    """Assemble the driver-JSON payload from a FitResult."""
    tc = fit.tyre_calibration
    rmse = fit.tyre_calibration_rmse or {}
    tyre_calibration_block = {
        "k_friction": round(float(tc.k_friction), 6) if tc is not None else 1.0,
        "h": round(float(tc.h), 4) if tc is not None else 50.0,
        "C_thermal": round(float(tc.C_thermal), 2) if tc is not None else 5000.0,
        "k_wear": float(tc.k_wear) if tc is not None else 1.0e-7,
        "measured": bool(tc.measured) if tc is not None else False,
        "source": {
            "telemetry_csvs": [_norm(s) for s in sources],
            "compound": fit.compound_name,
            "fit_rmse_temp_C": _round_maybe(rmse.get("rmse_temp_C"), 4),
            "fit_rmse_wear_pct": _round_maybe(rmse.get("rmse_wear_pct"), 4),
            "fit_rmse_pressure_psi": _round_maybe(rmse.get("rmse_pressure_psi"), 4),
            "fitted_at": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        },
    }
    return {
        "name": name,
        "skill_pct": round(float(fit.skill_pct), 4),
        "consistency_sigma": round(float(fit.consistency_sigma), 4),
        "driver_tau_s": round(float(top_level["driver_tau_s"]), 4),
        "trail_brake_m": round(float(top_level["trail_brake_m"]), 2),
        "throttle_ramp_m": round(float(top_level["throttle_ramp_m"]), 2),
        "profile": profile_block,
        "tyre_calibration": tyre_calibration_block,
        "source": {
            "telemetry_csvs": [_norm(s) for s in sources],
            "track_csv": _norm(track_csv) if track_csv else "",
            "car_data_dir": _norm(car_data_dir) if car_data_dir else "",
            "n_laps": fit.n_laps,
            "n_finished_laps": fit.n_finished_laps,
            "real_lap_times_s": list(fit.real_lap_times_s),
            "real_lap_time_s": fit.real_lap_time_s,
            "pooled_sample_count": fit.pooled_sample_count,
            "sim_lap_time_s": None,
            "delta_s": None,
            "fitted_at": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "fit_version": "2",
        },
    }
