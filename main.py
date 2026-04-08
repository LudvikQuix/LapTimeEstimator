#!/usr/bin/env python3
"""Lap Time Estimator - Point-mass simulation using Assetto Corsa car data."""
import argparse
import sys
import os

from car import Car
from track import BUILTIN_TRACKS, Track
from simulator import simulate, print_report


def find_car_data(car_path):
    """Find the data directory for a car."""
    if os.path.isfile(os.path.join(car_path, 'engine.ini')):
        return car_path
    data_dir = os.path.join(car_path, 'data')
    if os.path.isdir(data_dir):
        return data_dir
    raise FileNotFoundError(f"No car data found in {car_path}")


def main():
    parser = argparse.ArgumentParser(description="Lap Time Estimator")
    parser.add_argument('car', help='Path to car data directory')
    parser.add_argument('track', help=f'Track name ({", ".join(BUILTIN_TRACKS)}) or path to track JSON')
    parser.add_argument('--ds', type=float, default=2.0, help='Distance step in meters (default: 2.0)')
    parser.add_argument('--all-tracks', action='store_true', help='Run on all built-in tracks')

    args = parser.parse_args()

    # Load car
    data_dir = find_car_data(args.car)
    car = Car(data_dir)
    print(f"\nLoaded: {car}")

    if args.all_tracks:
        tracks = [(name, fn()) for name, fn in BUILTIN_TRACKS.items()]
    elif args.track in BUILTIN_TRACKS:
        tracks = [(args.track, BUILTIN_TRACKS[args.track]())]
    elif os.path.isfile(args.track):
        t = Track.from_json(args.track)
        tracks = [(t.name, t)]
    else:
        print(f"Unknown track '{args.track}'. Available: {', '.join(BUILTIN_TRACKS)}")
        sys.exit(1)

    for name, track in tracks:
        print(f"\nSimulating {track.name}...")
        result = simulate(car, track, ds=args.ds)
        print_report(car, track, result)


if __name__ == '__main__':
    main()
