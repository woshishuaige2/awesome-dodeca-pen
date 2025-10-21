# Code/Computer_vision/run.py
import cv2
import numpy as np
import time
from pathlib import Path
import sys

# Make relative imports work when imported as a module
BASE_DIR = Path(__file__).resolve().parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

import src.DoDecahedronUtils as dodecapen
import src.Tracker as tracker

# Intrinsics (unchanged from your file)
CAM_DIR = BASE_DIR / "camera_matrix"
COLOR_CAM_MATRIX_PATH = CAM_DIR / "color_cam_matrix.npy"
COLOR_CAM_DIST_PATH   = CAM_DIR / "color_cam_dist.npy"
color_cam_matrix = np.load(COLOR_CAM_MATRIX_PATH)
color_cam_dist   = np.load(COLOR_CAM_DIST_PATH)

# >>> shared state the bridge reads <<<
# shape (1,12): [tx,ty,tz, r00 r01 r02 r10 r11 r12 r20 r21 r22]
object_pose: np.ndarray | None = None

def _publish_pose(obj_1x12: np.ndarray) -> None:
    """Make the latest pose visible to dodeca_bridge in-process."""
    global object_pose
    object_pose = obj_1x12

def start(headless: bool = True, cam_index: int = 0) -> None:
    """
    Main vision loop.
    Continuously captures frames, runs marker tracking, converts pose to (1,12),
    and publishes it to the shared global 'object_pose' for EKF fusion.
    """
    global object_pose

    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"[CV] Could not open camera index {cam_index}")
        return

    if not headless:
        w, h = int(cap.get(3)), int(cap.get(4))
        cv2.namedWindow("RGB-D Live Stream", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("RGB-D Live Stream", w, h)
        cv2.moveWindow("RGB-D Live Stream", 20, 20)

    # Load DodecaPen calibration and params
    ddc_text_data = dodecapen.txt_data()
    ddc_params    = dodecapen.parameters()
    tip_loc_cent  = np.array([0.15100563, 137.52252061, -82.07403558, 1]).reshape(4, 1)
    post = 2

    frames, dets = 0, 0
    t0 = time.time()

    try:
        while True:
            ok, rgb = cap.read()
            if not ok or rgb is None:
                time.sleep(0.01)
                continue

            # Run object tracking
            obj = tracker.object_tracking(rgb, ddc_params, ddc_text_data, post)

            if obj is not None:
                arr = np.asarray(obj, dtype=float).reshape(-1)
                if arr.size == 6:
                    # [rvec(3), tvec(3)] → [t, R]
                    rvec = arr[:3].astype(np.float64).reshape(3, 1)
                    t    = arr[3:].astype(np.float64).reshape(3,)
                    R, _ = cv2.Rodrigues(rvec)
                elif arr.size == 12:
                    t = arr[:3].astype(np.float64).reshape(3,)
                    R = arr[3:].astype(np.float64).reshape(3, 3)
                elif arr.size == 16:
                    T = arr.reshape(4, 4)
                    R = T[:3, :3].astype(np.float64)
                    t = T[:3, 3].astype(np.float64)
                else:
                    R = None; t = None

                if R is not None and t is not None:
                    row_publish = np.empty((1, 12), dtype=np.float64)
                    row_publish[0, :3] = t
                    row_publish[0, 3:] = R.reshape(9)
                    _publish_pose(row_publish)
                    dets += 1

                    if not headless and (frames % 5) != 0:
                        # Draw coordinate axes
                        rvec, _ = cv2.Rodrigues(R)
                        cv2.drawFrameAxes(
                            rgb, ddc_params.mtx, ddc_params.dist,
                            rvec.reshape(3, 1), t.reshape(3, 1), 20
                        )

            # Display / performance logging
            frames += 1
            if time.time() - t0 > 1.0:
                print(f"[CV] fps={frames}, detections={dets}")
                frames, dets, t0 = 0, 0, time.time()

            if not headless:
                cv2.imshow("RGB-D Live Stream", rgb)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            else:
                time.sleep(0.001)

    finally:
        cap.release()
        if not headless:
            cv2.destroyAllWindows()


def main(headless: bool = False, cam_index: int = 0) -> None:
    start(headless=headless, cam_index=cam_index)

if __name__ == "__main__":
    main(headless=False)
