#!/usr/bin/env python3
"""
Measure the motion-sensor report rate and the sensor timestamps the kernel
derives from the DS4 report (MSC_TIMESTAMP), and compare them with host
wall-clock time. A healthy stream has MSC_TIMESTAMP deltas that match the
wall-clock deltas; a constant MSC_TIMESTAMP means the report's timestamp
field is not filled in.

Usage: ds4_imu_rate.py [--secs N] [--dev PATH]
"""
import argparse
import glob
import statistics
import sys
import time

from evdev import InputDevice, ecodes


def find_device():
    for path in sorted(glob.glob("/dev/input/by-id/*Sony*event-if00")):
        return path
    return None


def summarize(name, deltas):
    if not deltas:
        print(f"{name}: no samples")
        return
    print(f"{name}: n={len(deltas)} mean={statistics.mean(deltas):.0f}us "
          f"median={statistics.median(deltas):.0f}us min={min(deltas)}us max={max(deltas)}us")


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("--secs", type=float, default=3.0, help="measurement duration")
    parser.add_argument("--dev", help="motion sensors event device (default: auto-detect)")
    args = parser.parse_args()

    path = args.dev or find_device()
    if not path:
        sys.exit("DS4 motion sensors device not found")
    dev = InputDevice(path)
    print(f"{dev.name} ({path}), measuring for {args.secs}s - move the controller a bit")

    msc, wall = [], []
    end = time.monotonic() + args.secs
    for ev in dev.read_loop():
        if ev.type == ecodes.EV_MSC and ev.code == ecodes.MSC_TIMESTAMP:
            msc.append(ev.value)
            wall.append(int(ev.timestamp() * 1e6))
        if time.monotonic() > end:
            break

    msc_d = [(b - a) & 0xFFFFFFFF for a, b in zip(msc, msc[1:])]
    wall_d = [b - a for a, b in zip(wall, wall[1:])]
    if len(wall) > 1:
        print(f"report rate: {len(wall) / ((wall[-1] - wall[0]) / 1e6):.0f} Hz")
    summarize("wall-clock delta  ", wall_d)
    summarize("MSC_TIMESTAMP delta", msc_d)
    if msc and len(set(msc)) == 1:
        print(f"MSC_TIMESTAMP is constant ({msc[0]}): report timestamp field is not filled in")
    elif len(wall) > 1:
        ratio = ((msc[-1] - msc[0]) & 0xFFFFFFFF) / (wall[-1] - wall[0])
        print(f"sensor time / wall time = {ratio:.3f} (1.000 is ideal)")


if __name__ == "__main__":
    main()
