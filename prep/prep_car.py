#!/usr/bin/env python3
"""Prep a car: decode AC `data.acd` into `cars_csv/<car>/data/`."""
from __future__ import annotations

import argparse
import os
import sys

# Bring the sibling decoder into scope. prep/ is intentionally standalone -
# no dependency on the lap_estimator package.
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from decode_acd import decode_acd  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description="Decrypt an AC car's data.acd into cars_csv/")
    parser.add_argument("cars_in_dir", help="e.g. cars_in/bmw_1m")
    parser.add_argument("--output-root", default="cars_csv")
    args = parser.parse_args()

    cars_in = args.cars_in_dir.rstrip("/\\")
    acd_path = os.path.join(cars_in, "data.acd")
    if not os.path.isfile(acd_path):
        print(f"ERROR: {acd_path} not found.", file=sys.stderr)
        sys.exit(2)

    car_name = os.path.basename(os.path.abspath(cars_in))
    output_dir = os.path.join(args.output_root, car_name, "data")
    os.makedirs(output_dir, exist_ok=True)

    files = decode_acd(acd_path, car_name, output_dir)
    print(f"Decrypted {len(files)} files into {output_dir}")


if __name__ == "__main__":
    main()
