#!/usr/bin/env python3
"""Put one or more UniArmL1 motors into zero-damping mode."""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from robot_control.arm.uniarm_l1_bus import UnitreeMotorSDK


def main() -> None:
    parser = argparse.ArgumentParser(description="Disable selected UniArmL1 motors")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--ids", type=int, nargs="+", default=[5], help="Motor IDs to disable")
    parser.add_argument("--seconds", type=float, default=5.0, help="How long to keep sending commands")
    parser.add_argument("--status", type=int, default=1, help="Motor status/mode byte field to send")
    args = parser.parse_args()

    sdk = UnitreeMotorSDK(motor_ids=args.ids, port=args.port)
    deadline = time.monotonic() + args.seconds

    print(
        f"Sending zero-damping commands to {args.ids} on {args.port} "
        f"for {args.seconds:.1f}s (status={args.status})"
    )
    print("Try moving the motor while this is running. Press Ctrl+C to stop.")
    try:
        while time.monotonic() < deadline:
            for motor_id in args.ids:
                sdk.write_control_packet(
                    motor_id=motor_id,
                    status=args.status,
                    kp=0.0,
                    kd=0.0,
                    torque=0.0,
                    speed=0.0,
                    pos=0.0,
                    timeout_enable=0,
                )
                sdk.read()
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        sdk.close()
        print("Done.")


if __name__ == "__main__":
    main()
