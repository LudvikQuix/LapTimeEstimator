#!/usr/bin/env python3
"""Decode Assetto Corsa .acd car data archives.

Usage:
    python decode_acd.py <path/to/data.acd> <car_folder_name> [output_dir]

Example:
    python decode_acd.py cars/bmw_1m/data.acd bmw_1m cars/bmw_1m/data/
"""
import struct
import os
import sys


def generate_key(car_name):
    """Generate decryption key string from the car folder name."""
    s = car_name.lower()
    n = len(s)
    o = [ord(c) for c in s]

    # Part 1: sum of char codes
    p1 = sum(o) % 256

    # Part 2: alternating multiply/subtract
    num = 0
    i = 0
    while i < n - 1:
        num = num * o[i] - o[i + 1]
        i += 2
    p2 = num % 256

    # Part 3: multiply, divide, offset with stride 3
    num = 0
    i = 1
    while i < n - 3:
        num = num * o[i]
        num = int(num / (o[i + 1] + 27))
        num = num + (-27 - o[i - 1])
        i += 3
    p3 = num % 256

    # Part 4: subtract from 5763
    num = 0x1683
    for i in range(1, n):
        num = num - o[i]
    p4 = num % 256

    # Part 5: multiply pairs + 22 with stride 4
    num = 0x42
    i = 1
    while i < n - 4:
        num = num * (o[i] + 15) * (o[i - 1] + 15) + 22
        i += 4
    p5 = num % 256

    # Part 6: subtract even-indexed chars from 101
    num = 0x65
    i = 0
    while i < n - 2:
        num = num - o[i]
        i += 2
    p6 = num % 256

    # Part 7: modulo even-indexed chars from 171
    num = 0xAB
    i = 0
    while i < n - 2:
        num = num % o[i]
        i += 2
    p7 = num % 256

    # Part 8: divide and add from 171
    num = 0xAB
    for i in range(n - 1):
        num = int(num / o[i]) + o[i + 1]
    p8 = num % 256

    return f"{p1}-{p2}-{p3}-{p4}-{p5}-{p6}-{p7}-{p8}"


def unpack_acd(acd_path, car_name, output_dir):
    """Decrypt and extract all files from a .acd archive.

    Args:
        acd_path: Path to the .acd file
        car_name: Car folder name (used to derive decryption key)
        output_dir: Directory to write extracted files

    Returns:
        List of (filename, size) tuples for extracted files
    """
    os.makedirs(output_dir, exist_ok=True)

    key_str = generate_key(car_name)
    key = [ord(c) for c in key_str]
    key_len = len(key_str)

    with open(acd_path, 'rb') as f:
        data = f.read()

    pos = 0

    # Check for version-2 header marker
    first_int = struct.unpack_from('<i', data, 0)[0]
    if first_int == -1111:
        pos = 8

    files = []
    while pos < len(data):
        if pos + 4 > len(data):
            break

        name_len = struct.unpack_from('<i', data, pos)[0]
        pos += 4

        if name_len <= 0 or name_len > 1000 or pos + name_len > len(data):
            break

        filename = data[pos:pos + name_len].decode('utf-8', errors='replace')
        pos += name_len

        if pos + 4 > len(data):
            break

        content_size = struct.unpack_from('<i', data, pos)[0]
        pos += 4

        if content_size < 0 or pos + content_size * 4 > len(data):
            break

        # Decrypt: each byte stored as int32, subtract cycling key
        content = bytearray()
        for i in range(content_size):
            val = struct.unpack_from('<I', data, pos)[0] & 0xFF
            pos += 4
            content.append((val - key[i % key_len]) & 0xFF)

        out_path = os.path.join(output_dir, filename)
        with open(out_path, 'wb') as fout:
            fout.write(content)
        files.append((filename, len(content)))

    return files


# Public alias preferred by spec / new prep_car.py
decode_acd = unpack_acd


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    acd_path = sys.argv[1]
    car_name = sys.argv[2]
    output_dir = sys.argv[3] if len(sys.argv) > 3 else os.path.join(os.path.dirname(acd_path), 'data')

    print(f"Decrypting: {acd_path}")
    print(f"Car name:   {car_name}")
    print(f"Output:     {output_dir}")
    print(f"Key:        {generate_key(car_name)}")
    print()

    files = unpack_acd(acd_path, car_name, output_dir)

    for name, size in files:
        print(f"  {name:40s} {size:>8d} bytes")
    print(f"\nExtracted {len(files)} files")


if __name__ == '__main__':
    main()
