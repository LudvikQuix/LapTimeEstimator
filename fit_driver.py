#!/usr/bin/env python3
"""Fit a driver YAML from a real AC telemetry lap on a known car/track."""
from __future__ import annotations

import _bootstrap  # noqa: F401

import argparse
import datetime as _dt
import os
import sys

import yaml

from lap_estimator.car import Car
from lap_estimator.driver import Driver
from lap_estimator.driver_fit import fit_driver
from lap_estimator.simulator import simulate
from lap_estimator.telemetry import lap_time_seconds, merge_with_track, read_ac_log
from lap_estimator.track import Track


def find_car_data(car_path):
    if os.path.isfile(os.path.join(car_path, "engine.ini")):
        return car_path
    inner = os.path.join(car_path, "data")
    if os.path.isfile(os.path.join(inner, "engine.ini")):
        return inner
    raise FileNotFoundError(f"No engine.ini under {car_path}")


def main():
    parser = argparse.ArgumentParser(description="Fit driver YAML from AC telemetry")
    parser.add_argument("car", help="Car data directory")
    parser.add_argument("track", help="Track CSV path")
    parser.add_argument("telemetry", help="AC telemetry CSV")
    parser.add_argument("output", help="Output driver YAML path")
    parser.add_argument("--ds", type=float, default=2.0)
    parser.add_argument("--name", default=None)
    parser.add_argument("--no-validate", action="store_true")
    parser.add_argument("--no-plot", action="store_true")  # reserved
    args = parser.parse_args()

    data_dir = find_car_data(args.car)
    car = Car(data_dir)
    track = Track.from_csv(args.track)
    telem = read_ac_log(args.telemetry)

    if not _distances_overlap(telem, track):
        print("ERROR: telemetry distanceTraveled does not overlap track distance_m.",
              file=sys.stderr)
        sys.exit(2)

    merged = merge_with_track(telem, track)
    fit = fit_driver(car, merged)
    real_lap_s = lap_time_seconds(telem)

    if fit.n_over_unity / max(1, fit.n_corner_samples) > 0.05:
        print(f"  warn: {fit.n_over_unity}/{fit.n_corner_samples} samples have util > 1.0; "
              "car under-grips or kerbs / runoff are being used.")

    name = args.name or _derive_name(args.telemetry)

    payload = {
        "name": name,
        "skill_pct": round(fit.skill_pct, 4),
        "consistency_sigma": round(fit.consistency_sigma, 4),
        "source": {
            "telemetry_csv": _norm(args.telemetry),
            "track_csv": _norm(args.track),
            "car_data_dir": _norm(args.car),
            "real_lap_time_s": round(real_lap_s, 3),
            "sim_lap_time_s": None,
            "delta_s": None,
            "fitted_at": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "fit_version": "1",
        },
    }

    sim_lap_s = None
    if not args.no_validate:
        d = Driver(name=name, skill_pct=fit.skill_pct,
                   consistency_sigma=fit.consistency_sigma, raw=payload)
        result = simulate(car, track, d, ds=args.ds)
        sim_lap_s = result.lap_time
        payload["source"]["sim_lap_time_s"] = round(sim_lap_s, 3)
        payload["source"]["delta_s"] = round(sim_lap_s - real_lap_s, 3)

    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    with open(args.output, "w") as f:
        yaml.safe_dump(payload, f, sort_keys=False)
    print(f"Wrote: {args.output}")

    print()
    print(f"Real lap: {_fmt(real_lap_s)}")
    if sim_lap_s is not None:
        delta = sim_lap_s - real_lap_s
        pct = (delta / real_lap_s * 100.0) if real_lap_s > 0 else 0.0
        print(f"Sim lap:  {_fmt(sim_lap_s)}")
        print(f"Delta:    {delta:+.3f} s  ({pct:+.2f}%)")
        if abs(delta) > 3.0:
            print("  warn: |delta| > 3 s; review fit quality.")
    print(f"skill_pct        = {fit.skill_pct:.4f}")
    print(f"consistency_sigma = {fit.consistency_sigma:.4f}")
    print(f"util_p85          = {fit.util_p85:.4f}")
    print(f"util_stdev        = {fit.util_stdev:.4f}")
    print(f"n_corner_samples  = {fit.n_corner_samples}")


def _distances_overlap(telem, track):
    d = telem["distanceTraveled"]
    if len(d) < 2:
        return False
    span = d.max() - d.min()
    return span > 0.5 * track.total_length_m


def _derive_name(telemetry_path):
    base = os.path.splitext(os.path.basename(telemetry_path))[0]
    return base.replace(":", "_").replace(".", "_")


def _norm(path):
    return path.replace("\\", "/")


def _fmt(seconds):
    m = int(seconds // 60)
    s = seconds - m * 60
    return f"{m}:{s:06.3f}"


if __name__ == "__main__":
    main()
