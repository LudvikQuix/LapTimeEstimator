#!/usr/bin/env python3
"""Lap Time Estimator - simulator CLI (car / track / driver).

v2 (spec §21): multi-lap stint mode (`--laps N`, default 2 for back-compat),
inverse-PSI solver (`--solve-pressure-for-wear`), per-wheel cold-pressure setup
files (`--setup`, `--pressure`, `--ambient-temp-c`). Legacy single-lap and
two-lap behaviour is preserved (§11.30, §11.31).
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
    print_per_lap_block,
    write_comparison_plot,
    write_stint_summary_csv,
    write_trace_csv,
)
from lap_estimator.setup import resolve_compound, resolve_setup
from lap_estimator.sim_telemetry import write_synthetic_log
from lap_estimator.simulator import (
    print_report,
    simulate,
    simulate_monte_carlo,
    simulate_stint,
)
from lap_estimator.solve_setup import solve_pressure_for_wear
from lap_estimator.track import BUILTIN_TRACKS, Track
from lap_estimator.validate import validate_lap, write_bins_csv


def find_car_data(car_path):
    if os.path.isfile(os.path.join(car_path, "engine.ini")):
        return car_path
    data_dir = os.path.join(car_path, "data")
    if os.path.isdir(data_dir) and os.path.isfile(os.path.join(data_dir, "engine.ini")):
        return data_dir
    raise FileNotFoundError(f"No car data (engine.ini) found in {car_path}")


def resolve_track(arg):
    """Return (track_obj, source_kind, source_path)."""
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
    parser.add_argument("driver", help="Path to driver JSON (v1.1: JSON only, no YAML)")
    parser.add_argument("--ds", type=float, default=2.0)
    parser.add_argument("--all-tracks", action="store_true",
                        help="Run on all built-in tracks (ignores positional track)")
    parser.add_argument("--single-lap", action="store_true",
                        help="Alias for --laps 1. Mutually exclusive with --laps.")
    parser.add_argument("--laps", type=int, default=None,
                        help="Number of laps in the stint (1..50, default 2). "
                             "--laps 1 is back-compat single-lap; --laps 2 matches "
                             "the v1.1 two-lap default byte-for-byte (§11.31); "
                             "--laps N for N>=3 engages stint mode (§21.4).")
    parser.add_argument("--setup", default=None,
                        help="Path to a setup JSON (see setups/<car>_*.json).")
    parser.add_argument("--pressure", default=None,
                        help="Per-wheel cold pressure override: "
                             "FL=31,FR=31,RL=29,RR=29 (subset allowed).")
    parser.add_argument("--ambient-temp-c", type=float, default=None,
                        help="Ambient temperature (°C). Overrides setup file. Default 25.")
    parser.add_argument("--compound", default=None,
                        help="Active tyre compound name or short-name (e.g. Semislicks or SM). "
                             "Overrides setup-JSON `compound`. Case-insensitive; trailing "
                             "parenthetical short-name is stripped (so 'Semislicks (SM)' "
                             "matches). Unknown name -> argparse error.")
    parser.add_argument("--solve-pressure-for-wear", type=float, default=None,
                        help="Inverse-solver mode: target wear (0..1) at --at-lap.")
    parser.add_argument("--at-lap", type=int, default=None,
                        help="Target lap for --solve-pressure-for-wear (required with that flag).")
    parser.add_argument("--target-wheel", default="max",
                        choices=("max", "min", "avg", "FL", "FR", "RL", "RR"),
                        help="Aggregator for --solve-pressure-for-wear (default max).")
    parser.add_argument("--uniform-pressure", action="store_true",
                        help="Solve a single PSI applied to all 4 wheels.")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--no-telemetry", action="store_true")
    parser.add_argument("--telemetry-dt-ms", type=int, default=10,
                        help="Cadence (ms) of the synthetic telemetry CSV. "
                             "v1.2 default is 10 (was 100); pass 100 to revert.")
    parser.add_argument("--validate-against", default=None,
                        help="Path to real AC telemetry CSV for cross-track validation.")
    parser.add_argument("--bin-m", type=int, default=100)
    parser.add_argument("--per-corner", action="store_true")
    args = parser.parse_args()

    if args.single_lap and args.laps is not None:
        parser.error("--single-lap is an alias for --laps 1; do not pass both.")
    if args.single_lap:
        n_laps = 1
    elif args.laps is not None:
        n_laps = int(args.laps)
        if not 1 <= n_laps <= 50:
            parser.error("--laps must be in [1, 50].")
    else:
        n_laps = 2  # v1.1 default (back-compat).

    data_dir = find_car_data(args.car)
    car = Car(data_dir)
    driver = Driver.load(args.driver)
    calibration = driver.get_tyre_calibration()

    print(f"\nLoaded: {car}")
    print(
        f"Driver: {driver.name} "
        f"(skill={driver.skill_pct:.2f}, sigma={driver.consistency_sigma:.2f}, "
        f"tau={driver.driver_tau_s:.2f}s, trail={driver.trail_brake_m:.0f}m, "
        f"ramp={driver.throttle_ramp_m:.0f}m)"
    )
    if calibration.measured:
        print(
            f"Tyre calibration: MEASURED (k_friction={calibration.k_friction:.3f}, "
            f"h={calibration.h:.1f}, C_thermal={calibration.C_thermal:.0f}, "
            f"k_wear={calibration.k_wear:.2e})"
        )
    else:
        print("Tyre calibration: defaults (measured=false; "
              "per-wheel state channels absent from fit telemetry)")

    # Compound resolution (spec §21.11). Two-phase: load any setup-JSON peek
    # for its `compound` field, then resolve via CLI > setup > car-default.
    setup_for_compound_peek = None
    if args.setup:
        from lap_estimator.setup import Setup as _Setup
        try:
            setup_for_compound_peek = _Setup.load(args.setup)
        except Exception as e:  # pragma: no cover -- bad path / json surfaces below
            parser.error(f"Could not load --setup {args.setup}: {e}")
    try:
        compound, compound_source = resolve_compound(
            car,
            cli_name=args.compound,
            setup=setup_for_compound_peek,
        )
    except ValueError as e:
        parser.error(str(e))
    print(f"Compound: {compound.name} (idx {compound.index}) | source: {compound_source}")

    # Solve mode dispatch.
    if args.solve_pressure_for_wear is not None:
        if args.at_lap is None:
            parser.error("--solve-pressure-for-wear requires --at-lap")
        _run_solver(args, car, data_dir, driver, calibration, compound)
        return

    # Resolve setup (only used in stint mode; legacy single-lap path is unaffected).
    setup = resolve_setup(car, args.setup, args.pressure, args.ambient_temp_c,
                          compound=compound)
    print(setup.fmt_line())

    if args.all_tracks:
        tracks = [(name, BUILTIN_TRACKS[name](), "builtin", name) for name in BUILTIN_TRACKS]
    else:
        track, kind, src = resolve_track(args.track)
        tracks = [(track.name, track, kind, src)]

    for _name, track, kind, src in tracks:
        _run_one_track(args, car, driver, calibration, setup, track, kind, src,
                       n_laps=n_laps, compound=compound)


def _run_one_track(args, car, driver, calibration, setup, track, kind, src,
                   *, n_laps, compound=None):
    label = "single-lap" if n_laps == 1 else (
        "two-lap" if n_laps == 2 and not calibration.measured else f"stint ({n_laps} laps)"
    )
    print(f"\nSimulating {track.name} ({label})...")

    # Engage stint mode for N>=3 OR calibration.measured (so per-wheel state evolves).
    # Also engage when the user supplied an explicit setup/compound/pressure -- those
    # only have an effect through the stint loop's per-segment state update. (Spec
    # §11.35: --compound + --pressure at --laps 1 must affect lap-1 grip envelope.)
    user_supplied_state_overrides = bool(
        getattr(args, "setup", None)
        or getattr(args, "pressure", None)
        or getattr(args, "compound", None)
        or getattr(args, "ambient_temp_c", None)
    )
    use_stint = (
        n_laps >= 3
        or calibration.measured
        or n_laps == 2
        or user_supplied_state_overrides
    )

    if use_stint:
        stint = simulate_stint(
            car, track, driver,
            n_laps=n_laps, setup=setup, calibration=calibration,
            compound=compound, ds=args.ds,
        )
        # For lap-time stdout + plot, use the LAST lap's SimResult as the headline.
        result = stint.per_lap_sim_results[-1]
        # n_laps==2 back-compat: result is the v1.1 two-lap SimResult.
        if n_laps == 1:
            print(f"  Lap 1: {_fmt(stint.lap_times_s[0])}")
        elif n_laps == 2 and not calibration.measured:
            # Legacy print path -- back-compat with v1.2.1.
            print_report(car, track, result, driver=driver)
        else:
            # Per-lap block (spec §11.33).
            print_per_lap_block(stint)
    else:
        # Single-lap legacy path.
        stint = None
        mc = None
        if driver.consistency_sigma > 0:
            mc = simulate_monte_carlo(car, track, driver, ds=args.ds, two_lap=False)
            result = mc.representative
        else:
            result = simulate(car, track, driver, ds=args.ds, two_lap=False)
        print_report(car, track, result, driver=driver, mc=mc)

    stem = build_output_stem(src if kind == "csv" else track.name,
                             driver.name,
                             is_csv_track=(kind == "csv"))
    trace_path = f"{stem}_sim_trace.csv"
    write_trace_csv(result, trace_path)
    print(f"  Wrote trace: {trace_path}")

    if stint is not None and n_laps != 1 and not (n_laps == 2 and not calibration.measured):
        summary_path = f"{stem}_stint_summary.csv"
        write_stint_summary_csv(stint, summary_path)
        print(f"  Wrote stint summary: {summary_path}")

    if not args.no_plot:
        plot_path = f"{stem}_sim_vs_ai.png"
        lap_label = result.lap2_time_str if result.two_lap else result.lap_time_str
        ok = write_comparison_plot(
            result, track.name, driver.name, plot_path,
            lap_time_label=lap_label,
        )
        if ok:
            print(f"  Wrote plot:  {plot_path}")

    if not args.no_telemetry:
        tel_path = f"{stem}_sim_telemetry.csv"
        if stint is not None:
            write_synthetic_log(
                stint, car, driver, track.total_length_m, tel_path,
                telemetry_dt_ms=args.telemetry_dt_ms,
            )
        else:
            write_synthetic_log(
                result, car, driver, track.total_length_m, tel_path,
                telemetry_dt_ms=args.telemetry_dt_ms,
            )
        print(f"  Wrote telemetry: {tel_path}")

    if args.validate_against:
        if kind != "csv":
            print("ERROR: --validate-against requires a CSV-backed track.",
                  file=sys.stderr)
            sys.exit(2)
        target_lap = 2 if result.two_lap else 1
        if not result.two_lap:
            print("  warn: single-lap result; comparing real (flying) lap against "
                  "sim lap 1 (standing).")
        vr = validate_lap(
            car, track, result, args.validate_against,
            bin_m=args.bin_m, per_corner=args.per_corner, target_lap=target_lap,
        )
        _print_validation(vr, src, args.driver)
        bins_path = f"{stem}_validation_bins.csv"
        write_bins_csv(vr, bins_path)
        print(f"  Wrote bins: {bins_path}")
        if not args.no_plot:
            _plot_validation_overlay(
                car, track, result, args.validate_against,
                f"{stem}_validation_overlay.png", driver.name,
                target_lap=target_lap,
            )


def _run_solver(args, car, data_dir, driver, calibration, compound=None):
    """Inverse-PSI solver entry point (spec §21.5)."""
    track, kind, _src = resolve_track(args.track)
    target_wear = float(args.solve_pressure_for_wear)
    if not 0.0 <= target_wear <= 1.0:
        print("ERROR: --solve-pressure-for-wear must be in [0.0, 1.0].", file=sys.stderr)
        sys.exit(2)
    ambient = args.ambient_temp_c if args.ambient_temp_c is not None else 25.0
    print(
        f"\nSolver: target {target_wear * 100:.1f}% wear at lap {args.at_lap} "
        f"on {track.name} (wheel={args.target_wheel}, "
        f"{'uniform' if args.uniform_pressure else 'per-wheel'}) ..."
    )
    try:
        sr = solve_pressure_for_wear(
            car, track, driver,
            target_wear=target_wear,
            target_lap=int(args.at_lap),
            target_wheel=args.target_wheel,
            uniform=bool(args.uniform_pressure),
            calibration=calibration,
            ambient_temp_C=ambient,
            ds=args.ds,
            compound=compound,
        )
    except ValueError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(2)

    print(f"\nRecommended setup for {target_wear * 100:.1f}% wear at lap "
          f"{args.at_lap} on {track.name}:")
    for w in ("FL", "FR", "RL", "RR"):
        marker = "  <-- target wheel" if w == sr.target_wheel_resolved else ""
        print(f"  {w} = {sr.recommended_psi[w]:.1f} psi  (cold){marker}")
    print()
    print("Verification - predicted state under recommended setup:")
    print(f"  {'Lap':>3} {'Time':>9}  {'Wear FL/FR/RL/RR (%)':30}  "
          f"{'Temp avg':>9}  {'Pressure avg':>12}")
    print(f"  {'-' * 3} {'-' * 9}  {'-' * 30}  {'-' * 9}  {'-' * 12}")
    for k in range(sr.verification_stint.n_laps):
        end = sr.verification_stint.tyre_state_history[k + 1]
        lap_time = sr.verification_stint.lap_times_s[k]
        wears = f"{end.wear_pct['FL']:.0f}/{end.wear_pct['FR']:.0f}/{end.wear_pct['RL']:.0f}/{end.wear_pct['RR']:.0f}"
        print(
            f"  {k + 1:>3} {_fmt(lap_time):>9}  {wears:>30}  "
            f"{end.avg_temp_C():>6.1f} °C  {end.avg_pressure_psi():>8.1f} psi"
        )
    print()
    obs_pct = sr.observed_wear * 100.0
    target_pct = sr.target_wear * 100.0
    verdict = "target hit" if sr.converged else "target NOT hit (within tolerance)"
    print(f"Verdict: {verdict} "
          f"({sr.target_wheel_resolved}={obs_pct:.1f}% vs target {target_pct:.1f}%, "
          f"delta {obs_pct - target_pct:+.1f}%).")


def _print_validation(vr, track_path, driver_path):
    print("\n--- VALIDATION ---")
    print(f"Track:     {track_path}")
    print(f"Driver:    {driver_path}")
    print(f"Real lap:  {_fmt(vr.real_lap_time_s)}")
    label = f"Sim lap {vr.target_lap}" + (
        " (flying)" if vr.target_lap == 2 else " (standing)"
    )
    print(f"{label}: {_fmt(vr.sim_lap_time_s)}  (predicted)")
    print(f"Delta:     {_signed(vr.delta_s)} s  ({_signed_pct(vr.delta_pct)})")
    print(f"Verdict:   {vr.verdict}")


def _plot_validation_overlay(car, track, sim_result, real_telem_path, output_path,
                             driver_name, *, target_lap=2):
    from lap_estimator.telemetry import merge_with_track, read_ac_log
    import numpy as np
    telem = read_ac_log(real_telem_path)
    merged = merge_with_track(telem, track)
    if sim_result.lap_id is not None and sim_result.two_lap:
        mask = sim_result.lap_id == target_lap
        d = sim_result.distances[mask]
        sim_kmh = sim_result.speeds[mask] * 3.6
    else:
        d = sim_result.distances
        sim_kmh = sim_result.speeds * 3.6
    real_kmh = np.interp(d, merged["distance_m"], merged["speedKmh"])
    series = {"sim": sim_kmh, "real": real_kmh}
    plot_speed_overlay(
        d, series,
        title=f"{track.name} - {driver_name} (validation, sim lap {target_lap})",
        output_path=output_path,
    )
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
