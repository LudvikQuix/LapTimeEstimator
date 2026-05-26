#!/usr/bin/env python3
"""Fit Pacejka coefficients into a driver JSON (v3 slip-model, spec §23.7).

Distinct workflow from ``fit_driver.py`` (the v2 fitter). This CLI runs the
five-stage algorithm in ``pacejka_fit.py`` and writes (or merges) the
``pacejka_calibration`` block onto the existing driver JSON, preserving
every other top-level field.

Usage::

    fit_slip.py <car_dir> <track_csv> <driver_json> \
        [--from-lake driver=...,car=...,track=...,n=N] \
        [--from-csvs <path>...] \
        [--lake-url ...] [--lake-token ...] \
        [--compound <name>] \
        [--require-cv-pass]

Either ``--from-lake`` or ``--from-csvs`` is required.
"""
from __future__ import annotations

import _bootstrap  # noqa: F401

import argparse
import json
import sys

from lap_estimator.dynamics.pacejka_fit import (
    fit_pacejka_from_laps,
    merge_into_driver_json,
)
from lap_estimator.track import Track


def _parse_from_lake_arg(spec: str) -> dict:
    """Parse 'driver=...,car=...,track=...,n=...' into a kwargs dict."""
    out = {"driver": None, "car": None, "track": None, "n_newest": 5}
    for token in spec.split(","):
        token = token.strip()
        if not token:
            continue
        if "=" not in token:
            raise SystemExit(f"ERROR: --from-lake token '{token}' missing '='")
        k, v = token.split("=", 1)
        k = k.strip().lower()
        v = v.strip()
        if k == "n":
            try:
                out["n_newest"] = int(v)
            except ValueError as e:
                raise SystemExit(f"ERROR: --from-lake n='{v}' is not an int") from e
        elif k in ("driver", "car", "track"):
            out[k] = v
        else:
            raise SystemExit(f"ERROR: unknown --from-lake key '{k}'")
    missing = [k for k in ("driver", "car", "track") if not out[k]]
    if missing:
        raise SystemExit(f"ERROR: --from-lake missing required keys: {missing}")
    return out


def _load_laps_from_lake(args, track):
    from lap_estimator.lake_loader import load_laps_from_lake
    lake_args = _parse_from_lake_arg(args.from_lake)
    return load_laps_from_lake(
        driver=lake_args["driver"],
        car=lake_args["car"],
        track=lake_args["track"],
        n_newest=lake_args["n_newest"],
        lake_url=args.lake_url,
        token=args.lake_token,
        track_obj=track,
    )


def _load_laps_from_csvs(args, track):
    from lap_estimator.telemetry import merge_with_track, read_ac_log
    laps = []
    for path in args.from_csvs:
        telem = read_ac_log(path)
        merged = merge_with_track(telem, track)
        laps.append(merged)
    return laps


def _print_block_summary(block: dict) -> None:
    src = block["source"]
    print()
    print("=== Pacejka calibration ===")
    for axle in ("front", "rear"):
        lat = block[axle]["lateral"]
        lon = block[axle]["longitudinal"]
        print(
            f"  {axle:5s} LAT  B={lat['B']:6.2f}  C={lat['C']:5.2f}  "
            f"D_per_Fz={lat['D_per_Fz']:5.3f}  E={lat['E']:+5.2f}"
        )
        print(
            f"  {axle:5s} LONG B={lon['B']:6.2f}  C={lon['C']:5.2f}  "
            f"D_per_Fz={lon['D_per_Fz']:5.3f}  E={lon['E']:+5.2f}"
        )
    print(f"  ellipse_exponent     = {block['friction_ellipse_exponent']}")
    print(f"  RMSE Fy (lat)        = {src['rmse_lat_pct']:.2f}%  "
          f"[front {src.get('rmse_lat_front_pct', float('nan')):.2f}%, "
          f"rear {src.get('rmse_lat_rear_pct', float('nan')):.2f}%]")
    print(f"  RMSE Fx (long)       = {src['rmse_long_pct']:.2f}%  "
          f"[front {src.get('rmse_long_front_pct', float('nan')):.2f}%, "
          f"rear {src.get('rmse_long_rear_pct', float('nan')):.2f}%]")
    print(f"  cv_passed            = {src['cv_passed']}")
    print(f"  samples              = {src['n_samples']}")
    print(f"  compound             = {src['compound']}")


def main():
    parser = argparse.ArgumentParser(
        description="Fit Pacejka coefficients into a driver JSON (v3 slip model)."
    )
    parser.add_argument("car", help="Car data directory (with car.ini/tyres.ini)")
    parser.add_argument("track", help="Track CSV path")
    parser.add_argument("driver", help="Driver JSON path (read + merged in place)")
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--from-lake", default=None,
                     help="Lake query spec: 'driver=...,car=...,track=...,n=N'. "
                          "Uses QUIXLAKE_URL + QUIX_LAKE_TOKEN env vars unless overridden.")
    src.add_argument("--from-csvs", nargs="+", default=None,
                     help="One or more local AC log CSV paths.")
    parser.add_argument("--lake-url", default=None)
    parser.add_argument("--lake-token", default=None)
    parser.add_argument("--compound", default=None,
                        help="Override compound name (otherwise read from telemetry).")
    parser.add_argument("--require-cv-pass", action="store_true",
                        help="Exit non-zero if cross-validation thresholds fail.")
    args = parser.parse_args()

    track = Track.from_csv(args.track)

    if args.from_lake:
        laps = _load_laps_from_lake(args, track)
    else:
        laps = _load_laps_from_csvs(args, track)

    print(f"Loaded {len(laps)} laps; running 5-stage Pacejka fit...")
    try:
        block = fit_pacejka_from_laps(
            laps,
            car_data_dir=args.car,
            compound_name=args.compound,
        )
    except ValueError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(2)

    _print_block_summary(block)

    if args.require_cv_pass and not block["source"]["cv_passed"]:
        print("ERROR: cross-validation failed; not writing block.", file=sys.stderr)
        sys.exit(3)

    merge_into_driver_json(args.driver, block)
    print(f"\nWrote pacejka_calibration into: {args.driver}")

    # Sanity-print: confirm v2 fields preserved.
    with open(args.driver, "r", encoding="utf-8") as f:
        data = json.load(f)
    preserved = sorted(k for k in data.keys() if k != "pacejka_calibration")
    print(f"Preserved top-level keys: {preserved}")


if __name__ == "__main__":
    main()
