#!/usr/bin/env python3
"""Cycle one XL431 through centered angles, independently of the MIDI demo."""

import argparse
from pathlib import Path
import sys
import time

# Allow running this example directly from a source checkout without installing.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from camera_dynamixel import CameraBridge, ServoError


ANGLES = (0, 90, 180, -90, -180, 0)


def run(args: argparse.Namespace) -> None:
    bridge = CameraBridge(
        camera_ip=args.camera_ip,
        host_ip=args.host_ip,
        timeout=args.timeout,
        retries=args.retries,
    )
    servo = bridge.servo(args.id)
    initial = servo.validate()
    if abs(initial.goal_position - initial.present_position) > args.start_tolerance:
        raise ServoError(
            "Goal and present positions differ by more than "
            f"{args.start_tolerance} counts; refusing torque enable"
        )

    torque_was_enabled = initial.torque_enabled
    print(
        f"Servo {initial.servo_id}, model {initial.model_number}, "
        f"limits {initial.minimum_position}..{initial.maximum_position}, "
        f"position {initial.present_position}, torque "
        f"{'on' if torque_was_enabled else 'off'}"
    )
    print(
        "Angle mapping: -180 -> minimum, 0 -> midpoint, +180 -> maximum. "
        f"Cycles: {args.cycles}."
    )

    try:
        if not torque_was_enabled:
            servo.enable_torque()
            held_position = servo.position
            if abs(held_position - initial.present_position) > args.start_tolerance:
                raise ServoError(
                    "Position shifted too far when torque was enabled; "
                    "stopping the demo"
                )
            servo.move_to_position(held_position)
        for cycle in range(1, args.cycles + 1):
            for angle in ANGLES:
                target = servo.move_to_angle(angle, timeout=args.move_timeout, tolerance=args.tolerance)
                print(
                    f"cycle {cycle}/{args.cycles}: angle {angle:+4d} deg "
                    f"-> goal {target}, present {servo.position}",
                    flush=True,
                )
                if args.dwell:
                    time.sleep(args.dwell)
    finally:
        try:
            servo.move_to_angle(0, timeout=args.move_timeout, tolerance=args.tolerance)
            print(f"returned to 0 deg, present {servo.position}", flush=True)
        finally:
            if not torque_was_enabled:
                servo.disable_torque()
                print("torque restored off", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--id", type=int, default=102, help="DYNAMIXEL ID (default: 102)")
    parser.add_argument("--camera-ip", default="10.0.144.35")
    parser.add_argument("--host-ip", default="10.0.144.2")
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--dwell", type=float, default=0.0, help="pause after each angle, seconds (default: no extra pause)")
    parser.add_argument("--move-timeout", type=float, default=30.0)
    parser.add_argument("--tolerance", type=int, default=32)
    parser.add_argument("--start-tolerance", type=int, default=256)
    parser.add_argument("--timeout", type=float, default=0.45)
    parser.add_argument("--retries", type=int, default=10)
    args = parser.parse_args()
    if args.cycles < 1:
        parser.error("--cycles must be at least 1")
    if args.dwell < 0:
        parser.error("--dwell cannot be negative")
    run(args)


if __name__ == "__main__":
    main()
