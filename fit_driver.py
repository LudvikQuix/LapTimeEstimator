#!/usr/bin/env python3
"""Fit a driver JSON from real AC telemetry laps on a known car/track.

v1.1 + v1.2 (spec §13.1, §13.5, §13.11, §13.12):
  - Variadic positional CSVs OR `--laps-glob`; >=2 laps required.
  - `--newest N` (default 10) caps the candidate pool to the N newest-by-mtime
    files, lex-tiebreak. Spec §13.12.
  - Pools cornering samples across laps; 85th percentile -> skill_pct,
    scaled stdev -> consistency_sigma.
  - Measures the v1.2 dynamic profile (driver_tau_s, trail_brake_m,
    throttle_ramp_m, pedal_press_rate_per_s, steering_aggression_deg_per_s)
    via `profile_dynamics.measure_dynamics`. Spec §13.11.
  - Validation sim runs two laps; reported `sim_lap_time_s` is lap 2 (the
    flying-lap analogue of a real learned lap).
"""
from __future__ import annotations

import _bootstrap  # noqa: F401

import argparse
import datetime as _dt
import glob as _glob
import json
import os
import sys

from lap_estimator.car import Car
from lap_estimator.driver import (
    DEFAULT_DRIVER_TAU_S,
    DEFAULT_THROTTLE_RAMP_M,
    DEFAULT_TRAIL_BRAKE_M,
    Driver,
)
from lap_estimator.driver_fit import fit_driver
from lap_estimator.simulator import simulate
from lap_estimator.telemetry import (
    filter_to_lap,
    merge_with_track,
    read_ac_log,
)
from lap_estimator.track import Track


LAP_SELECTION_RULE = "newest-5-to-10"
LAP_GUARD_MESSAGE = "fit requires >=2 laps; see §13"


def find_car_data(car_path):
    if os.path.isfile(os.path.join(car_path, "engine.ini")):
        return car_path
    inner = os.path.join(car_path, "data")
    if os.path.isfile(os.path.join(inner, "engine.ini")):
        return inner
    raise FileNotFoundError(f"No engine.ini under {car_path}")


def main():
    parser = argparse.ArgumentParser(description="Fit driver JSON from AC telemetry")
    parser.add_argument("car", help="Car data directory")
    parser.add_argument("track", help="Track CSV path")
    parser.add_argument("output", help="Output driver JSON path")
    parser.add_argument("telemetry", nargs="*",
                        help="AC telemetry CSV files (>=2 required after lap selection)")
    parser.add_argument("--laps-glob", default=None,
                        help="Glob pattern; expanded to the candidate-lap list. "
                             "Mutually exclusive with positional CSVs.")
    parser.add_argument("--newest", type=int, default=10,
                        help="Keep at most N newest-by-mtime candidate laps "
                             "(default 10, minimum effective 2).")
    parser.add_argument("--ds", type=float, default=2.0)
    parser.add_argument("--name", default=None)
    parser.add_argument("--no-validate", action="store_true")
    parser.add_argument("--no-plot", action="store_true")  # reserved
    parser.add_argument("--lap", type=int, default=2, choices=(1, 2),
                        help="When an input CSV has a `lap` column (sim-emitted), "
                             "filter to this lap before fitting. Default 2.")
    args = parser.parse_args()

    if args.newest < 2:
        parser.error("--newest must be >= 2 (the >=2 laps guard would always trip).")

    if args.laps_glob and args.telemetry:
        parser.error("--laps-glob is mutually exclusive with positional telemetry CSVs.")

    if args.laps_glob:
        candidates = sorted(_glob.glob(args.laps_glob))
    else:
        candidates = list(args.telemetry)
    if not candidates:
        parser.error("No telemetry CSVs supplied (positional or --laps-glob).")

    selected, range_str = _apply_lap_selection(candidates, args.newest)
    if len(selected) < 2:
        print(f"ERROR: {LAP_GUARD_MESSAGE}", file=sys.stderr)
        sys.exit(2)

    print(
        f"Lap selection: {len(candidates)} candidates -> "
        f"kept {len(selected)} newest{range_str}"
    )

    data_dir = find_car_data(args.car)
    car = Car(data_dir)
    track = Track.from_csv(args.track)

    merged_frames, real_lap_times_inputs, steer_present_any = _load_and_merge(
        selected, track, args.lap
    )
    if not steer_present_any:
        print("  warn: no input CSV contained `steerAngle`; "
              "steering_aggression_deg_per_s will be null.")

    for path, merged in zip(selected, merged_frames):
        if not _distances_overlap(merged, track):
            print(f"ERROR: {path}: distance range does not overlap track.",
                  file=sys.stderr)
            sys.exit(2)

    try:
        fit = fit_driver(car, merged_frames, track=track)
    except ValueError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(2)

    if fit.n_over_unity / max(1, fit.n_corner_samples) > 0.05:
        print(f"  warn: {fit.n_over_unity}/{fit.n_corner_samples} samples have util > 1.0; "
              "car under-grips or kerbs / runoff are being used.")

    name = args.name or _derive_name(args.output)
    profile_block, top_level_dynamics = _profile_payload(fit.profile)

    # v2: tyre_calibration block (spec §7.2 / §13.14 / §21.11).
    tc = fit.tyre_calibration
    tyre_calibration_block = {
        "k_friction": round(float(tc.k_friction), 6) if tc is not None else 1.0,
        "h": round(float(tc.h), 4) if tc is not None else 50.0,
        "C_thermal": round(float(tc.C_thermal), 2) if tc is not None else 5000.0,
        "k_wear": float(tc.k_wear) if tc is not None else 1.0e-7,
        "measured": bool(tc.measured) if tc is not None else False,
        "source": {
            "telemetry_csvs": [_norm(p) for p in selected],
            "compound": fit.compound_name,
            "fit_rmse_temp_C": round(fit.tyre_calibration_rmse.get("rmse_temp_C"), 4)
                if fit.tyre_calibration_rmse else None,
            "fit_rmse_wear_pct": round(fit.tyre_calibration_rmse.get("rmse_wear_pct"), 4)
                if fit.tyre_calibration_rmse else None,
            "fit_rmse_pressure_psi": round(fit.tyre_calibration_rmse.get("rmse_pressure_psi"), 4)
                if fit.tyre_calibration_rmse else None,
            "fitted_at": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        },
    }
    if fit.compound_name is not None:
        print(
            f"Tyre compound: {fit.compound_name} (source: {fit.compound_source})"
        )
    if tc is None or not tc.measured:
        print("Tyre calibration: per-wheel state channels absent - using hand-defaults")

    payload = {
        "name": name,
        "skill_pct": round(fit.skill_pct, 4),
        "consistency_sigma": round(fit.consistency_sigma, 4),
        "driver_tau_s": round(top_level_dynamics["driver_tau_s"], 4),
        "trail_brake_m": round(top_level_dynamics["trail_brake_m"], 2),
        "throttle_ramp_m": round(top_level_dynamics["throttle_ramp_m"], 2),
        "profile": profile_block,
        "tyre_calibration": tyre_calibration_block,
        "source": {
            "telemetry_csvs": [_norm(p) for p in selected],
            "track_csv": _norm(args.track),
            "car_data_dir": _norm(args.car),
            "n_laps": fit.n_laps,
            "n_finished_laps": fit.n_finished_laps,
            "real_lap_times_s": list(fit.real_lap_times_s),
            "real_lap_time_s": fit.real_lap_time_s,
            "pooled_sample_count": fit.pooled_sample_count,
            "sim_lap_time_s": None,
            "delta_s": None,
            "lap_selection": {
                "rule": LAP_SELECTION_RULE,
                "candidates_considered": len(candidates),
                "selected_count": len(selected),
                "selected_sources": [_norm(p) for p in selected],
            },
            "fitted_at": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "fit_version": "2",
        },
    }

    _print_prefit_summary(fit)

    sim_lap_s = None
    if not args.no_validate:
        d = Driver(
            name=name,
            skill_pct=fit.skill_pct,
            consistency_sigma=fit.consistency_sigma,
            driver_tau_s=top_level_dynamics["driver_tau_s"],
            trail_brake_m=top_level_dynamics["trail_brake_m"],
            throttle_ramp_m=top_level_dynamics["throttle_ramp_m"],
            raw=payload,
        )
        result = simulate(car, track, d, ds=args.ds, two_lap=True)
        sim_lap_s = result.lap2_time if result.two_lap else result.lap_time
        payload["source"]["sim_lap_time_s"] = round(sim_lap_s, 3)
        if fit.real_lap_time_s is not None:
            payload["source"]["delta_s"] = round(sim_lap_s - fit.real_lap_time_s, 3)

    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    with open(args.output, "w") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")
    print(f"Wrote: {args.output}")

    _print_post_validation_report(fit, sim_lap_s, real_lap_times_inputs)


def _apply_lap_selection(candidates, newest):
    """Apply the v1.2 lap-selection rule: top-N newest-by-mtime, lex tiebreak.

    Returns (selected_paths_newest_first, range_str_for_log).
    """
    decorated = []
    for path in candidates:
        try:
            mtime = os.path.getmtime(path)
        except OSError as e:
            raise SystemExit(f"ERROR: cannot stat {path}: {e}") from e
        decorated.append((mtime, path))
    # Sort: newest mtime first; lex on path for ties (ascending).
    decorated.sort(key=lambda t: (-t[0], t[1]))
    selected = [p for _, p in decorated[:newest]]
    if not selected:
        return selected, ""
    newest_mtime = max(_get_mtime(p) for p in selected)
    oldest_mtime = min(_get_mtime(p) for p in selected)
    range_str = (
        f" (range {_fmt_mtime(oldest_mtime)}..{_fmt_mtime(newest_mtime)})"
    )
    return selected, range_str


def _get_mtime(path):
    return os.path.getmtime(path)


def _fmt_mtime(ts):
    return _dt.datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M")


def _load_and_merge(paths, track, lap_choice):
    """Read each CSV, trim out-lap, optionally filter sim `lap` column, merge."""
    merged_frames = []
    real_lap_times = []
    steer_present_any = False
    for path in paths:
        telem = read_ac_log(path)
        if "lap" in telem:
            try:
                telem = filter_to_lap(telem, lap_choice)
                print(f"  {path}: has 'lap' column; filtered to lap={lap_choice}")
            except ValueError as e:
                print(f"ERROR: {e}", file=sys.stderr)
                sys.exit(2)
        telem = _trim_outlap(telem)
        merged = merge_with_track(telem, track)
        merged_frames.append(merged)
        if "steerAngle" in merged:
            steer_present_any = True
        # Per-CSV lap time captured for the post-report (regardless of finished flag).
        ts = telem["timestamp_ms"]
        if len(ts) >= 2:
            real_lap_times.append(float((ts.max() - ts.min()) / 1000.0))
    return merged_frames, real_lap_times, steer_present_any


def _trim_outlap(telem):
    """Drop pre-start-line prefix on a standing-start lap.

    Detects the wrap-through-zero of `normalizedCarPosition`: when the trace
    starts above 0.05 and later wraps below 0.05, drop everything up to (and
    including) that wrap. Idempotent on flying laps.
    """
    ncp = telem.get("normalizedCarPosition")
    if ncp is None or len(ncp) < 5:
        return telem
    if float(ncp[0]) <= 0.05:
        return telem
    # Find the first index where ncp drops back near zero (wrap).
    wrap_idx = -1
    for i in range(1, len(ncp)):
        if ncp[i] < 0.05 and ncp[i - 1] > 0.5:
            wrap_idx = i
            break
    if wrap_idx < 0:
        return telem
    out = {}
    for k, v in telem.items():
        out[k] = v[wrap_idx:]
    return out


def _profile_payload(profile):
    """Translate a `ProfileDynamics` into the JSON `profile.dynamic` block and
    the top-level `driver_tau_s/trail_brake_m/throttle_ramp_m` mirror values.
    """
    dynamic_block = {
        "driver_tau_s": _maybe_round(profile.driver_tau_s, 4),
        "trail_brake_m": _maybe_round(profile.trail_brake_m, 2),
        "throttle_ramp_m": _maybe_round(profile.throttle_ramp_m, 2),
        "pedal_press_rate_per_s": _maybe_round(profile.pedal_press_rate_per_s, 4),
        "steering_aggression_deg_per_s": _maybe_round(
            profile.steering_aggression_deg_per_s, 2
        ),
        "measured": dict(profile.measured),
        "sample_counts": dict(profile.sample_counts),
    }
    # Top-level mirror: measured value if measured-true, else hand-default.
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


def _maybe_round(value, places):
    if value is None:
        return None
    return round(float(value), places)


def _print_prefit_summary(fit):
    p = fit.profile
    print(
        f"Laps loaded:      {fit.n_laps}   "
        f"({fit.n_finished_laps} finished, {fit.n_laps - fit.n_finished_laps} unfinished)"
    )
    print(f"Pooled samples:   {fit.pooled_sample_count} cornering points")
    if fit.real_lap_time_s is not None:
        print(f"Mean real lap time (finished): {_fmt(fit.real_lap_time_s)}")
    if p is not None:
        press = (
            f"{p.pedal_press_rate_per_s:.2f}/s"
            if p.pedal_press_rate_per_s is not None else "n/a"
        )
        steer = (
            f"{p.steering_aggression_deg_per_s:.0f}d/s"
            if p.steering_aggression_deg_per_s is not None else "n/a"
        )
        tau = f"{p.driver_tau_s:.3f}s" if p.driver_tau_s is not None else "n/a"
        trail = f"{p.trail_brake_m:.1f}m" if p.trail_brake_m is not None else "n/a"
        ramp = f"{p.throttle_ramp_m:.1f}m" if p.throttle_ramp_m is not None else "n/a"
        print(
            f"Profile dynamics: tau={tau}  trail={trail}  ramp={ramp}  "
            f"press={press}  steer95={steer}"
        )


def _print_post_validation_report(fit, sim_lap_s, real_lap_times_inputs):
    print()
    print(f"Real laps used:                {fit.n_laps}  "
          f"({fit.n_finished_laps} finished)")
    if fit.real_lap_time_s is not None:
        print(f"Mean real lap time (finished): {_fmt(fit.real_lap_time_s)}")
    if sim_lap_s is not None:
        print(f"Sim lap 2 (flying):            {_fmt(sim_lap_s)}")
        if fit.real_lap_time_s is not None:
            delta = sim_lap_s - fit.real_lap_time_s
            pct = (
                delta / fit.real_lap_time_s * 100.0
                if fit.real_lap_time_s > 0 else 0.0
            )
            print(f"Delta vs mean real:            {delta:+.3f} s  ({pct:+.2f}%)")
            print(f"Verdict:                       {_verdict(delta, pct)}")
    print(f"skill_pct         = {fit.skill_pct:.4f}")
    print(f"consistency_sigma = {fit.consistency_sigma:.4f}")
    print(f"util_p85          = {fit.util_p85:.4f}")
    print(f"util_stdev        = {fit.util_stdev:.4f}")
    print(f"n_corner_samples  = {fit.n_corner_samples}")


def _verdict(delta_s, pct):
    if abs(delta_s) < 3.0 and abs(pct) < 5.0:
        return "GOOD"
    if abs(pct) < 10.0:
        return "LOOSE"
    return "BAD"


def _distances_overlap(merged, track):
    d = merged["distance_m"]
    if len(d) < 2:
        return False
    span = d.max() - d.min()
    return span > 0.5 * track.total_length_m


def _derive_name(output_path):
    base = os.path.splitext(os.path.basename(output_path))[0]
    return base.replace(":", "_").replace(".", "_")


def _norm(path):
    return path.replace("\\", "/")


def _fmt(seconds):
    m = int(seconds // 60)
    s = seconds - m * 60
    return f"{m}:{s:06.3f}"


if __name__ == "__main__":
    main()
