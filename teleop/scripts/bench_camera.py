#!/usr/bin/env python3
"""Benchmark OpenCV camera read stability for common formats."""

from pathlib import Path
import argparse
import time

import cv2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("camera", type=int, help="OpenCV camera index, e.g. 2")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fourcc", nargs="*", default=["MJPG", "YUYV", "none"])
    return parser.parse_args()


def bench_one(camera_id: int, fourcc: str, width: int, height: int, fps: int, seconds: float) -> None:
    cap = cv2.VideoCapture(f"/dev/video{camera_id}")
    if not cap.isOpened():
        print(f"/dev/video{camera_id} {fourcc}: open failed")
        return

    if fourcc.lower() != "none":
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    actual_w = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
    actual_h = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    actual_code = int(cap.get(cv2.CAP_PROP_FOURCC))
    actual_fourcc = "".join(chr((actual_code >> 8 * i) & 0xFF) for i in range(4))

    ok_count = 0
    fail_count = 0
    shapes: set[tuple[int, int]] = set()
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        ok, frame = cap.read()
        if ok and frame is not None:
            ok_count += 1
            h, w = frame.shape[:2]
            shapes.add((w, h))
        else:
            fail_count += 1

    elapsed = time.monotonic() - t0
    cap.release()
    rate = ok_count / elapsed if elapsed > 0 else 0.0
    print(
        f"/dev/video{camera_id} req={fourcc:4s} actual={actual_fourcc!r} "
        f"{actual_w}x{actual_h}@{actual_fps:.1f}: ok={ok_count}, fail={fail_count}, "
        f"rate={rate:.1f} Hz, shapes={sorted(shapes)}"
    )


def main() -> None:
    args = parse_args()
    for fourcc in args.fourcc:
        bench_one(args.camera, fourcc, args.width, args.height, args.fps, args.seconds)


if __name__ == "__main__":
    main()
