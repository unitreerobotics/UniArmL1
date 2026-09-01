#!/usr/bin/env python3
"""Benchmark UniArmL1 motor bus round-trip timing."""

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from robot_control.arm.uniarm_l1_bus import UnitreeMotorSDK


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    idx = min(len(values) - 1, max(0, round((len(values) - 1) * pct)))
    return sorted(values)[idx]


def main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark UniArmL1 motor bus timing")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--ids", type=int, nargs="+", default=list(range(8)), help="Motor IDs to poll")
    parser.add_argument("--rounds", type=int, default=200, help="Number of polling rounds")
    args = parser.parse_args()

    sdk = UnitreeMotorSDK(motor_ids=args.ids, port=args.port)
    round_ms: list[float] = []
    per_motor_ms = {motor_id: [] for motor_id in args.ids}
    misses = {motor_id: 0 for motor_id in args.ids}

    print(f"Benchmarking {args.port}, ids={args.ids}, rounds={args.rounds}")
    try:
        for round_idx in range(args.rounds):
            t0 = time.perf_counter()
            for motor_id in args.ids:
                tm0 = time.perf_counter()
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
                result = sdk.read()
                dtm = (time.perf_counter() - tm0) * 1000.0
                per_motor_ms[motor_id].append(dtm)
                if result is None:
                    misses[motor_id] += 1
            round_ms.append((time.perf_counter() - t0) * 1000.0)

            if (round_idx + 1) % 50 == 0:
                print(f"  {round_idx + 1}/{args.rounds} rounds")
    finally:
        sdk.close()

    print("\nRound timing:")
    print(f"  mean={statistics.mean(round_ms):.2f} ms")
    print(f"  p50 ={statistics.median(round_ms):.2f} ms")
    print(f"  p95 ={percentile(round_ms, 0.95):.2f} ms")
    print(f"  max ={max(round_ms):.2f} ms")
    print(f"  rate={1000.0 / statistics.mean(round_ms):.1f} Hz")

    print("\nPer motor:")
    for motor_id in args.ids:
        times = per_motor_ms[motor_id]
        print(
            f"  M{motor_id}: mean={statistics.mean(times):.2f} ms, "
            f"p95={percentile(times, 0.95):.2f} ms, "
            f"max={max(times):.2f} ms, misses={misses[motor_id]}"
        )


if __name__ == "__main__":
    main()
