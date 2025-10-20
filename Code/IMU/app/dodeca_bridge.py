# dodeca_bridge.py  — unified, guarded bridge for Dodecaball → IMU EKF

from pathlib import Path
import sys
import time
import numpy as np

# --- Geometry configuration (kept local for simplicity) ---
# Vector from Dodecaball CENTER → PEN TIP in the body frame (mm→m)
TIP_OFFSET_BODY = np.array([0.0, 137.52252061, -82.07403558]) * 1e-3
# Vector from IMU → PEN TIP in the same body frame (meters). Update if calibrated.
IMU_TO_TIP_BODY = np.array([0.0, 0.0, 0.0])

# --- Make Computer_vision importable (once) ---
repo_root = Path(__file__).resolve().parents[2]      # .../Code
cv_dir = repo_root / "Computer_vision"
if str(cv_dir) not in sys.path:
    sys.path.append(str(cv_dir))

# Guarded import: this should NOT auto-run the CV window if run.py has an __main__ guard
try:
    import run as dcv_run   # Code/Computer_vision/run.py
except Exception:
    dcv_run = None

# --- Quaternion helper with sign continuity ---
_prev_q = None
def _rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    import transforms3d as t3d
    q = t3d.quaternions.mat2quat(R)  # [w, x, y, z]
    global _prev_q
    if _prev_q is not None and float(np.dot(q, _prev_q)) < 0.0:
        q = -q
    _prev_q = q
    return q

# --- Vision access ---
def get_vision_reading():
    """
    Returns (t_cam, R_cam, ts) from Dodecaball vision, or None if not available.
    t_cam: (3,) in meters; R_cam: (3,3)
    Requires that Computer_vision/run.py defines a module-level `object_pose`
    updated by its loop, but does NOT auto-run on import.
    """
    if dcv_run is None or not hasattr(dcv_run, "object_pose"):
        return None
    obj = dcv_run.object_pose
    if obj is None:
        return None
    t = obj[0, :3].astype(float)
    R = obj[0, 3:].reshape(3, 3).astype(float)
    return t, R, time.time()

# --- EKF measurement packaging ---
def make_ekf_measurements(tip_offset_body: np.ndarray = TIP_OFFSET_BODY,
                          imu_to_tip_body: np.ndarray = IMU_TO_TIP_BODY):
    """
    Map camera pose (center) → EKF-friendly measurements.
    Returns dict with:
      - imu_pos_cam: (3,)
      - tip_pos_cam: (3,)
      - R_cam: (3,3)
      - q_cam: (4,) [w,x,y,z]
      - timestamp: float
      - quality: float
    or None if no vision reading available.
    """
    out = get_vision_reading()
    if out is None:
        return None 
    t_cam, R_cam, ts = out
    t_cam_m = t_cam * 0.001

    # Offsets are defined in the body frame; transform to camera frame.
    tip_pos_cam = t_cam + R_cam @ tip_offset_body
    # IMU = tip - (IMU→tip)
    imu_pos_cam = t_cam_m + R_cam @ (tip_offset_body - imu_to_tip_body)

    q_cam = _rotmat_to_quat(R_cam)
    return {
        "imu_pos_cam": imu_pos_cam,
        "tip_pos_cam": tip_pos_cam,
        "R_cam": R_cam,
        "q_cam": q_cam,
        "timestamp": ts,
        "quality": 1.0,
    }

# Expose geometry constants if callers import them
__all__ = [
    "TIP_OFFSET_BODY",
    "IMU_TO_TIP_BODY",
    "get_vision_reading",
    "make_ekf_measurements",
]
