#!/usr/bin/env python3
"""Monitor selected UniArmL1 motor feedback in zero-damping mode."""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from robot_control.arm.uniarm_l1_bus import UnitreeMotorSDK


def main() -> None:
    parser = argparse.ArgumentParser(description="Monitor UniArmL1 motor feedback")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM1")
    parser.add_argument("--ids", type=int, nargs="+", default=[5], help="Motor IDs to monitor")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--hz", type=float, default=20.0)
    args = parser.parse_args()

    sdk = UnitreeMotorSDK(motor_ids=args.ids, port=args.port)
    period = 1.0 / args.hz
    deadline = time.monotonic() + args.seconds

    print(f"Monitoring motors {args.ids} on {args.port} for {args.seconds:.1f}s")
    print("Move the joint by hand if this is the leader arm. Press Ctrl+C to stop.")

    try:
        while time.monotonic() < deadline:
            values = []
            for motor_id in args.ids:
                sdk.write_control_packet(
                    motor_id=motor_id,
                    status=1,
                    kp=0.0,
                    kd=0.0,
                    torque=0.0,
                    speed=0.0,
                    pos=0.0,
                    timeout_enable=0,
                )
                feedback = sdk.read()
                st = sdk.motor_states[motor_id]
                if feedback is None:
                    values.append(f"M{motor_id}: MISS")
                else:
                    values.append(
                        f"M{motor_id}: raw={st.OutPos:7.1f} "
                        f"rad={st.OutPos_rad:7.3f} spd={st.speed_rads:7.3f} "
                        f"err={st.MError} warn={st.ExFlag}"
                    )
            print(" | ".join(values))
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        sdk.close()


if __name__ == "__main__":
    main()
