#!/usr/bin/env python3
"""Capture one diagnostic frame from each camera and build a labeled overview."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import argparse
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from image_server.opencv.camera_opencv import OpenCVCamera
from image_server.opencv.configuration_opencv import Cv2Rotation, OpenCVCameraConfig


CAMERA_ROTATIONS = {
    "0": Cv2Rotation.NO_ROTATION,
    "90": Cv2Rotation.ROTATE_90,
    "180": Cv2Rotation.ROTATE_180,
    "270": Cv2Rotation.ROTATE_270,
    "-90": Cv2Rotation.ROTATE_270,
}


@dataclass
class CameraSpec:
    name: str
    device_id: int
    rotation: Cv2Rotation


def parse_camera_spec(spec: str) -> CameraSpec:
    parts = spec.split(":")
    if len(parts) not in (2, 3):
        raise ValueError("camera spec must be 'name:id' or 'name:id:rotation'")
    name, device_id = parts[:2]
    rotation = Cv2Rotation.NO_ROTATION
    if len(parts) == 3:
        rotation = CAMERA_ROTATIONS.get(parts[2])
        if rotation is None:
            raise ValueError("rotation must be 0/90/180/270/-90")
    return CameraSpec(name=name, device_id=int(device_id), rotation=rotation)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture and label camera snapshots")
    parser.add_argument(
        "-c",
        "--camera",
        nargs="+",
        required=True,
        help="Camera specs like head:0 wrist:1:180",
    )
    parser.add_argument("--out-dir", default="./camera_check_outputs")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--camera-fourcc", type=str, default="MJPG")
    parser.add_argument("--no-auto-wb", action="store_true")
    parser.add_argument("--wb-temperature", type=int, default=None)
    return parser.parse_args()


def capture_frame(spec: CameraSpec, args: argparse.Namespace) -> np.ndarray:
    config = OpenCVCameraConfig(
        index_or_path=Path(f"/dev/video{spec.device_id}"),
        fps=args.fps,
        width=args.width,
        height=args.height,
        rotation=spec.rotation,
        fourcc=None if args.camera_fourcc.lower() == "none" else args.camera_fourcc,
        auto_wb=not args.no_auto_wb,
        wb_temperature=args.wb_temperature,
    )
    cam = OpenCVCamera(config)
    cam.connect(warmup=True)
    try:
        frame = cam.read()
    finally:
        cam.disconnect()
    return frame


def annotate(frame: np.ndarray, title: str) -> np.ndarray:
    canvas = frame.copy()
    overlay_h = 32
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], overlay_h), (0, 0, 0), thickness=-1)
    cv2.putText(
        canvas,
        title,
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return canvas


def pad_to_height(image: np.ndarray, height: int) -> np.ndarray:
    if image.shape[0] == height:
        return image
    pad = height - image.shape[0]
    return cv2.copyMakeBorder(image, 0, pad, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))


def main() -> None:
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    specs = [parse_camera_spec(spec) for spec in args.camera]
    snapshots: list[tuple[CameraSpec, np.ndarray]] = []

    for spec in specs:
        frame = capture_frame(spec, args)
        title = f"{spec.name} /dev/video{spec.device_id} rot={spec.rotation.value}"
        annotated = annotate(frame, title)
        snapshots.append((spec, annotated))
        out_path = out_dir / f"{spec.name}_video{spec.device_id}.jpg"
        cv2.imwrite(str(out_path), cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
        print(f"Saved {out_path}")

    if not snapshots:
        print("No camera snapshots captured.")
        return

    max_h = max(frame.shape[0] for _, frame in snapshots)
    padded = [pad_to_height(frame, max_h) for _, frame in snapshots]
    overview = np.hstack(padded)
    overview_path = out_dir / "overview.jpg"
    cv2.imwrite(str(overview_path), cv2.cvtColor(overview, cv2.COLOR_RGB2BGR))
    print(f"Saved {overview_path}")


if __name__ == "__main__":
    main()
