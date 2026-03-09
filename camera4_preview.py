#!/usr/bin/env python3
import argparse
import cv2
import numpy as np


def _open_camera(index: int) -> cv2.VideoCapture | None:
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if cap.isOpened():
        return cap
    cap.release()

    # fallback to default backend if V4L2 failed
    cap = cv2.VideoCapture(index)
    if cap.isOpened():
        return cap

    cap.release()
    return None


def zoom_frame(frame: np.ndarray, zoom_factor: float, focus: tuple[float, float]) -> np.ndarray:
    """Digitally zoom in by cropping around focus and resizing back to original size."""
    if zoom_factor <= 1.0:
        return frame

    h, w = frame.shape[:2]
    crop_w = max(1, int(w / zoom_factor))
    crop_h = max(1, int(h / zoom_factor))
    focus_x = np.clip(focus[0], 0.0, 1.0)
    focus_y = np.clip(focus[1], 0.0, 1.0)
    center_x = int(focus_x * w)
    center_y = int(focus_y * h)

    x1 = np.clip(center_x - crop_w // 2, 0, w - crop_w)
    y1 = np.clip(center_y - crop_h // 2, 0, h - crop_h)
    cropped = frame[y1 : y1 + crop_h, x1 : x1 + crop_w]
    return cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)


def main(cam2_zoom: float, cam2_focus: tuple[float, float]):
    cap1 = _open_camera(2)
    cap2 = _open_camera(4)
    cap3 = _open_camera(6)

    if cap1 is None or cap2 is None or cap3 is None:
        print("Failed to open camera index 0, 4, or 6 (V4L2 and default backends)")
        return

    # Request MJPEG to reduce USB bandwidth usage.
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    for cap in (cap1, cap2, cap3):
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

    print("Streaming camera 0, 4, and 6. Press 'q' to quit.")
    while True:
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()
        ret3, frame3 = cap3.read()

        if not ret1 or not ret2 or not ret3:
            print("Failed to grab frame")
            break

        frame1 = zoom_frame(frame1, cam2_zoom, cam2_focus)

        # Show feeds in a single row.
        combined = np.hstack((frame1, frame2, frame3))
        cv2.imshow("USB Cameras 0 | 4 | 6", combined)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap1.release()
    cap2.release()
    cap3.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Preview three USB cameras with optional zoom on camera index 2.")
    parser.add_argument(
        "--cam2-zoom",
        type=float,
        default=1.0,
        help="Digital zoom factor for camera index 2 (must be >= 1.0).",
    )
    parser.add_argument(
        "--cam2-focus-x",
        type=float,
        default=0.5,
        help="Horizontal focus (0.0 left, 1.0 right) for camera 2 zoom crop.",
    )
    parser.add_argument(
        "--cam2-focus-y",
        type=float,
        default=0.5,
        help="Vertical focus (0.0 top, 1.0 bottom) for camera 2 zoom crop.",
    )
    args = parser.parse_args()
    if args.cam2_zoom < 1.0:
        raise SystemExit("--cam2-zoom must be >= 1.0")
    main(args.cam2_zoom, (args.cam2_focus_x, args.cam2_focus_y))
