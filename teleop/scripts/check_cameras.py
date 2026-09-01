#!/usr/bin/env python3
"""List OpenCV-readable /dev/video devices."""

from pathlib import Path
import sys

import cv2

sys.path.insert(0, str(Path(__file__).parent.parent))

from image_server.opencv.camera_opencv import OpenCVCamera


def main() -> None:
    devices = sorted(Path("/dev").glob("video*"))
    if not devices:
        print("No /dev/video* devices found.")
        return

    print("--- OpenCV camera probe ---")
    try:
        print("\nDefault stream profiles:")
        for info in OpenCVCamera.find_cameras():
            profile = info["default_stream_profile"]
            print(
                f"{info['id']}: {profile['width']}x{profile['height']} "
                f"fps={profile['fps']:.1f} fourcc={profile['fourcc']!r}"
            )
        print()
    except Exception as exc:
        print(f"Could not query default profiles: {exc}\n")

    for dev in devices:
        cap = cv2.VideoCapture(str(dev))
        opened = cap.isOpened()
        ok, frame = cap.read() if opened else (False, None)
        if ok and frame is not None:
            h, w = frame.shape[:2]
            print(f"{dev}: OK, frame={w}x{h}")
        else:
            print(f"{dev}: not readable")
        cap.release()


if __name__ == "__main__":
    main()
