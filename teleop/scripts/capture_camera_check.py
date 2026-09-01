#!/usr/bin/env python3
"""Capture diagnostic frames for checking camera color channel/order."""

from pathlib import Path
import argparse
import sys
import time

import cv2

sys.path.insert(0, str(Path(__file__).parent.parent))

from image_server.opencv.camera_opencv import OpenCVCamera
from image_server.opencv.configuration_opencv import ColorMode, OpenCVCameraConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("camera", type=int, help="OpenCV camera index, e.g. 0 or 2")
    parser.add_argument("--out-dir", default="/tmp/uniarm_camera_check")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--wb-temperature", type=int, default=None)
    parser.add_argument("--no-auto-wb", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    raw_cap = cv2.VideoCapture(args.camera)
    if not raw_cap.isOpened():
        raise SystemExit(f"Failed to open raw camera {args.camera}")

    raw_cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    raw_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    raw_cap.set(cv2.CAP_PROP_FPS, args.fps)
    if args.no_auto_wb:
        raw_cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    if args.wb_temperature is not None:
        raw_cap.set(cv2.CAP_PROP_WB_TEMPERATURE, args.wb_temperature)

    raw_frame = None
    for _ in range(10):
        ok, frame = raw_cap.read()
        if ok:
            raw_frame = frame
        time.sleep(0.03)
    raw_cap.release()

    if raw_frame is None:
        raise SystemExit(f"Failed to read raw camera {args.camera}")

    raw_path = out_dir / f"camera{args.camera}_raw_bgr.jpg"
    cv2.imwrite(str(raw_path), raw_frame)

    config = OpenCVCameraConfig(
        index_or_path=args.camera,
        fps=args.fps,
        width=args.width,
        height=args.height,
        color_mode=ColorMode.RGB,
        auto_wb=not args.no_auto_wb,
        wb_temperature=args.wb_temperature,
    )
    camera = OpenCVCamera(config)
    camera.connect(warmup=True)
    rgb_frame = camera.read()
    camera.disconnect()

    rgb_saved_as_bgr_path = out_dir / f"camera{args.camera}_pipeline_rgb_saved_correctly.jpg"
    rgb_saved_wrong_path = out_dir / f"camera{args.camera}_pipeline_rgb_saved_without_conversion.jpg"
    cv2.imwrite(str(rgb_saved_as_bgr_path), cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(rgb_saved_wrong_path), rgb_frame)

    print(f"Saved raw BGR frame: {raw_path}")
    print(f"Saved pipeline RGB frame with correct jpg conversion: {rgb_saved_as_bgr_path}")
    print(f"Saved intentionally wrong RGB-as-BGR frame: {rgb_saved_wrong_path}")


if __name__ == "__main__":
    main()
