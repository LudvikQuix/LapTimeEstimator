#!/usr/bin/env python3
"""Lap Time Estimator - simulator CLI (car / track / driver).

Run a single sim, optionally cross-validate against a real AC telemetry lap.
"""
from __future__ import annotations

import _bootstrap  # noqa: F401  (puts src/ on sys.path)

import argparse
import os
import sys

from lap_estimator.car import Car
from lap_estimator.driver import Driver
from lap_estimator.report import (
    build_output_stem,
    plot_speed_overlay,
    write_comparison_plot,
    write_trace_csv,
)
from lap_estimator.sim_telemetry import write_synthetic_log
from lap_estimator.simulator import (
    print_report,
    simulate,
    simulate_monte_carlo,
)
from lap_estimator.track import BUILTIN_TRACKS, Track
from lap_estimator.validate import validate_lap, write_bins_csv


def find_car_data(car_path):
    """Resolve a car data dir. Accepts either the dir containing engine.ini or a parent."""
    if os.path.isfile(os.path.join(car_path, "engine.ini")):
        return car_path
    data_dir = os.path.join(car_path, "data")
    if os.path.isdir(data_dir) and os.path.isfile(os.path.join(data_dir, "engine.ini")):
        return data_dir
    raise FileNotFoundError(f"No car data (engine.ini) found in {car_path}")


def resolve_track(arg):
    """Return (track_obj, source_kind, source_path).

    source_kind in {'csv', 'json', 'builtin'}; source_path is the input string.
    """
    if os.path.isfile(arg) and arg.lower().endswith(".csv"):
        return Track.from_csv(arg), "csv", arg
    if os.path.isfile(arg) and arg.lower().endswith(".json"):
        return Track.from_json(arg), "json", arg
    if arg in BUILTIN_TRACKS:
        return BUILTIN_TRACKS[arg](), "builtin", arg
    raise ValueError(
        f"Track '{arg}' not found. Provide a .csv, .json, or one of: "
        f"{', '.join(BUILTIN_TRACKS)}"
    )


def main():
    parser = argparse.ArgumentParser(description="Lap Time Estimator")
    parser.add_argument("car", help="Path to car data directory")
    parser.add_argument("track", help="Track CSV path, JSON path, or built-in name")
    parser.add_argument("driver", help="Path to driver YAML")
    parser.add_argument("--ds", type=float, default=2.0)
    parser.add_argument("--all-tracks", action="store_true",
                        help="Run on all built-in tracks (ignores positional track)")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--no-telemetry", action="store_true")
    parser.add_argument("--telemetry-dt-ms", type=int, default=100)
    parser.add_argument("--validate-against", default=None,
                        help="Path to real AC telemetry CSV for cross-track validation.")
    parser.add_argument("--bin-m", type=int, default=100)
    parser.add_argument("--per-corner", action="store_true")
    args = parser.parse_args()

    data_dir = find_car_data(args.car)
    car = Car(data_dir)
    driver = Driver.load(args.driver)
    print(f"\nLoaded: {car}")
    print(f"Driver: {driver.name} (skill={driver.skill_pct:.2f}, sigma={driver.consistency_sigma:.2f})")

    if args.all_tracks:
        tracks = [(name, BUILTIN_TRACKS[name](), "builtin", name) for name in BUILTIN_TRACKS]
    else:
        track, kind, src = resolve_track(args.track)
        tracks = [(track.name, track, kind, src)]

    for _name, track, kind, src in tracks:
        print(f"\nSimulating {track.name}...")
        mc = None
        if driver.consistency_sigma > 0:
            mc = simulate_monte_carlo(car, track, driver, ds=args.ds)
            result = mc.representative
        else:
            result = simulate(car, track, driver, ds=args.ds)

        print_report(car, track, result, driver=driver, mc=mc)

        stem = build_output_stem(src if kind == "csv" else _name,
                                 driver.name,
                                 is_csv_track=(kind == "csv"))
        trace_path = f"{stem}_sim_trace.csv"
        write_trace_csv(result, trace_path)
        print(f"  Wrote trace: {trace_path}")

        if not args.no_plot:
            plot_path = f"{stem}_sim_vs_ai.png"
            ok = write_comparison_plot(
                result, track.name, driver.name, plot_path,
                lap_time_label=(f"{mc.mean_lap_time:.3f}s ± {mc.std_lap_time:.3f} (N={mc.n_runs})"
                                if mc is not None else result.lap_time_str),
            )
            if ok:
                print(f"  Wrote plot:  {plot_path}")

        if not args.no_telemetry:
            tel_path = f"{stem}_sim_telemetry.csv"
            write_synthetic_log(
                result, car, track.total_length_m, tel_path,
                telemetry_dt_ms=args.telemetry_dt_ms,
            )
            print(f"  Wrote telemetry: {tel_path}")

        if args.validate_against:
            if kind != "csv":
                print("ERROR: --validate-against requires a CSV-backed track.", file=sys.stderr)
                sys.exit(2)
            vr = validate_lap(
                car, track, result, args.validate_against,
                bin_m=args.bin_m, per_corner=args.per_corner,
            )
            _print_validation(vr, src, args.driver)
            bins_path = f"{stem}_validation_bins.csv"
            write_bins_csv(vr, bins_path)
            print(f"  Wrote bins: {bins_path}")
            if not args.no_plot:
                _plot_validation_overlay(
                    car, track, result, args.validate_against,
                    f"{stem}_validation_overlay.png", driver.name,
                )


def _print_validation(vr, track_path, driver_path):
    print("\n--- VALIDATION ---")
    print(f"Track:     {track_path}")
    print(f"Driver:    {driver_path}")
    print(f"Real lap:  {_fmt(vr.real_lap_time_s)}")
    print(f"Sim lap:   {_fmt(vr.sim_lap_time_s)}  (predicted)")
    print(f"Delta:     {_signed(vr.delta_s)} s  ({_signed_pct(vr.delta_pct)})")
    print(f"Verdict:   {vr.verdict}")


def _plot_validation_overlay(car, track, sim_result, real_telem_path, output_path, driver_name):
    from lap_estimator.telemetry import merge_with_track, read_ac_log
    telem = read_ac_log(real_telem_path)
    merged = merge_with_track(telem, track)
    import numpy as np
    # Resample real speed onto sim distance grid
    d = sim_result.distances
    real_kmh = np.interp(d, merged["distance_m"], merged["speedKmh"])
    series = {
        "sim": sim_result.speeds * 3.6,
        "real": real_kmh,
    }
    plot_speed_overlay(d, series,
                       title=f"{track.name} - {driver_name} (validation)",
                       output_path=output_path)
    print(f"  Wrote overlay: {output_path}")


def _fmt(seconds):
    m = int(seconds // 60)
    s = seconds - m * 60
    return f"{m}:{s:06.3f}"


def _signed(x):
    return f"{x:+.3f}"


def _signed_pct(x):
    return f"{x:+.2f}%"


if __name__ == "__main__":
    main()
