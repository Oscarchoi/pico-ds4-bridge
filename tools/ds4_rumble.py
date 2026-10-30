#!/usr/bin/env python3
"""
Play a rumble effect on the bridge's emulated DS4 through the kernel
force-feedback API (the same path games use), so the request travels
host -> USB output report 0x05 -> Pico -> Bluetooth -> controller.

Usage: ds4_rumble.py [--strong 0-65535] [--weak 0-65535] [--ms N] [--dev PATH]
"""
import argparse
import glob
import sys
import time

from evdev import InputDevice, ecodes, ff


def find_device():
    for path in sorted(glob.glob("/dev/input/by-id/*Sony*event-joystick")):
        return path
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("--strong", type=int, default=0xC000, help="strong (left) motor, 0-65535")
    parser.add_argument("--weak", type=int, default=0xC000, help="weak (right) motor, 0-65535")
    parser.add_argument("--ms", type=int, default=1500, help="effect duration in ms")
    parser.add_argument("--dev", help="event device path (default: auto-detect)")
    args = parser.parse_args()

    path = args.dev or find_device()
    if not path:
        sys.exit("DS4 event-joystick device not found")

    dev = InputDevice(path)
    if ecodes.EV_FF not in dev.capabilities():
        sys.exit(f"{path} has no force-feedback support")

    effect = ff.Effect(
        ecodes.FF_RUMBLE, -1, 0,
        ff.Trigger(0, 0),
        ff.Replay(args.ms, 0),
        ff.EffectType(ff_rumble_effect=ff.Rumble(strong_magnitude=args.strong, weak_magnitude=args.weak)),
    )
    effect_id = dev.upload_effect(effect)
    print(f"{dev.name} ({path}): strong={args.strong} weak={args.weak} for {args.ms}ms")
    try:
        dev.write(ecodes.EV_FF, effect_id, 1)
        time.sleep(args.ms / 1000 + 0.2)
    finally:
        dev.erase_effect(effect_id)
        dev.close()


if __name__ == "__main__":
    main()
