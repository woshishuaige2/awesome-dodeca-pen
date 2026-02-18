
import json
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from pyquaternion import Quaternion

# Add project directories to sys.path
repo_root = Path(__file__).resolve().parents[1]
sys.path.append(str(repo_root / "IMU"))

from app.filter import DpointFilter, initial_state, Q, imu_noise
from app.monitor_ble import StylusReading
from app.dodeca_bridge import CENTER_TO_TIP_BODY
import app.filter_core as fc

def run_workflow(input_file, mode="decoupled"):
    """
    Modes: 
    - 'cv_only': Only use CV updates, no IMU.
    - 'standard': Use 7D CV updates (Pos + Quat).
    - 'decoupled': Use 3D CV updates (Pos only).
    """
    with open(input_file, "r") as f:
        data = json.load(f)
    
    imu_readings = data.get("imu_readings", [])
    cv_readings = data.get("cv_readings", [])
    
    # Backward compatibility: rename old field names to new ones
    for cv_reading in cv_readings:
        if "imu_pos_cam" in cv_reading and "center_pos_cam" not in cv_reading:
            cv_reading["center_pos_cam"] = cv_reading["imu_pos_cam"]
    
    all_events = []
    
    if mode != "cv_only":
        for r in imu_readings:
            all_events.append(("IMU", r["local_timestamp"], r))
    
    for r in cv_readings:
        all_events.append(("CV", r["local_timestamp"], r))
    
    all_events.sort(key=lambda x: x[1])

    # Configuration for different modes
    original_camera_measurement = fc.camera_measurement
    original_fuse_camera = fc.fuse_camera

    # Standard mode logic helper
    def standard_fuse_camera(fs, imu_pos, orientation_quat):
        h = np.concatenate((fs.state[fc.i_pos], fs.state[fc.i_quat]))
        H = np.zeros((7, fc.STATE_SIZE))
        H[0:3, fc.i_pos] = np.eye(3)
        H[3:7, fc.i_quat] = np.eye(4)
        z = np.concatenate((imu_pos.flatten(), orientation_quat))
        # Use small noise to follow CV
        R = np.diag([1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 1e-4, 1e-4])
        state, statecov = fc.ekf_correct(fs.state, fs.statecov, h, H, z, R)
        state[fc.i_quat] = fc.repair_quaternion(state[fc.i_quat])
        return fc.FilterState(state, statecov)

    # Initialize Filter
    dt = 0.01
    if len(imu_readings) > 1:
        dt = imu_readings[1]["local_timestamp"] - imu_readings[0]["local_timestamp"]
    
    filter = DpointFilter(dt=dt, smoothing_length=15, camera_delay=5)
    trajectory = []

    first_cv = True
    for i, (type, ts, reading) in enumerate(all_events):
        # Skip negative timestamps as they cause EKF to diverge with zero-init
        if ts < 0:
            continue
            
        if type == "IMU":
            sr = StylusReading.from_json(reading)
            # Correct mapping for Dodeca-pen to Camera Frame:
            # Pen Y+ -> Camera X+
            # Pen Z+ -> Camera Y-
            # Pen X+ -> Camera Z+
            accel2 = np.array([sr.accel[1], -sr.accel[2], sr.accel[0]])
            gyro2 = np.array([sr.gyro[1], -sr.gyro[2], sr.gyro[0]])
            
            # Use Standard EKF update if in standard mode
            if mode == "standard":
                predicted = fc.ekf_predict(filter.fs, filter.dt, Q)
                h, H = fc.imu_measurement(predicted.state)
                z = np.concatenate((accel2, gyro2))
                state, statecov = fc.ekf_correct(predicted.state, predicted.statecov, h, H, z, imu_noise)
                state[fc.i_quat] = fc.repair_quaternion(state[fc.i_quat])
                filter.fs = fc.FilterState(state, statecov)
            else:
                filter.update_imu(sr.accel, sr.gyro)
            
            # If we haven't seen CV yet, we can't initialize position
            if first_cv:
                continue
                
            # Add tip position even for IMU updates to have high-frequency trajectory
            if mode != "cv_only":
                center = filter.fs.state[fc.i_pos]
                q = Quaternion(filter.fs.state[fc.i_quat])
                tip = center + q.rotation_matrix @ CENTER_TO_TIP_BODY
                trajectory.append({"t": ts, "x": tip[0], "y": tip[1], "z": tip[2]})
        elif type == "CV":
            # CV reading contains dodecahedron center position
            center_pos = np.array(reading["center_pos_cam"])
            r_cam = np.array(reading["R_cam"])
            q_cam = Quaternion(matrix=r_cam).elements
            
            if first_cv:
                # Initialize filter with first CV reading
                filter.fs = fc.FilterState(initial_state(center_pos, q_cam).state, filter.fs.statecov)
                first_cv = False
                # Append first point
                tip = center_pos + r_cam @ CENTER_TO_TIP_BODY
                trajectory.append({"t": ts, "x": tip[0], "y": tip[1], "z": tip[2]})
                continue
            
            if mode == "cv_only":
                # CV-only: just use the center position directly
                filter.fs = fc.FilterState(initial_state(center_pos, q_cam).state, filter.fs.statecov)
                # Calculate tip from center and rotation
                tip = center_pos + r_cam @ CENTER_TO_TIP_BODY
                trajectory.append({"t": ts, "x": tip[0], "y": tip[1], "z": tip[2]})
            elif mode == "standard":
                # Standard mode uses the custom 7D fuse
                filter.fs = standard_fuse_camera(filter.fs, center_pos, q_cam)
            else:
                # Decoupled mode uses update_camera
                filter.update_camera(center_pos, r_cam)

    # Restore original functions if modified
    fc.camera_measurement = original_camera_measurement
    
    return pd.DataFrame(trajectory)

def visualize(results_dict, output_path):
    fig = plt.figure(figsize=(18, 12))
    
    # 3D Plot
    ax = fig.add_subplot(221, projection='3d')
    # XY Plane (Top view)
    ax2 = fig.add_subplot(222)
    # Z-axis over time (Absolute to see lifting)
    ax3 = fig.add_subplot(223)
    # Z-axis jitter (Normalized)
    ax4 = fig.add_subplot(224)

    # Define plotting styles for each workflow
    plot_styles = {
        "CV Only (Raw)": {"alpha": 0.6, "linewidth": 1.0, "color": 'gray', "linestyle": '--', "zorder": 1},
        "Standard EKF (Coupled)": {"alpha": 0.7, "linewidth": 1.5, "color": 'orange', "linestyle": '-', "zorder": 2},
        "Decoupled EKF (Proposed)": {"alpha": 0.9, "linewidth": 2.0, "color": 'green', "linestyle": '-', "zorder": 3}
    }

    # Plot order: CV Only (Raw) first, then Standard EKF, then Decoupled on top
    # This ensures the EKF results are clearly visible over the noisy raw data
    plot_order = ["CV Only (Raw)", "Standard EKF (Coupled)", "Decoupled EKF (Proposed)"]
    sorted_results = {k: results_dict[k] for k in plot_order if k in results_dict}

    for name, df in sorted_results.items():
        if not df.empty:
            style = plot_styles.get(name, {})
            
            # 3D Plot
            ax.plot(df["x"], df["y"], df["z"], label=name, **style)
            
            # XY Plane (Top view)
            ax2.plot(df['x'], df['y'], label=name, **style)
            
            # Z-axis over time (Absolute to see lifting)
            ax3.plot(df['t'] - df['t'].iloc[0], df['z'], label=name, alpha=style.get('alpha', 0.8))
            
            # Z-axis jitter (Normalized)
            z_norm = df['z'] - df['z'].rolling(window=10, center=True).mean()
            ax4.plot(df['t'] - df['t'].iloc[0], z_norm, label=name, alpha=style.get('alpha', 0.8))

    ax.set_title("3D Pen-Tip Trajectory")
    ax.view_init(elev=20, azim=-60)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()

    ax2.set_title("XY Plane (Top View)")
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.axis('equal') # Maintain aspect ratio for the rectangle
    ax2.grid(True)
    ax2.legend()

    ax3.set_title("Z-Axis (Absolute Height)")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Z (m)")
    ax3.grid(True)
    ax3.legend()

    ax4.set_title("Z-Axis Jitter (High-pass filtered)")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Z Noise (m)")
    ax4.grid(True)
    ax4.legend()

    plt.tight_layout()
    plt.savefig(output_path)
    print(f"Comparison plot saved to {output_path}")

if __name__ == "__main__":
    data_file = "outputs/my_data.json"  # Input data file
    
    print("Running CV Only workflow...")
    df_cv = run_workflow(data_file, mode="cv_only")
    
    print("Running Standard EKF workflow...")
    df_std = run_workflow(data_file, mode="standard")
    
    print("Running Decoupled EKF workflow...")
    df_dec = run_workflow(data_file, mode="decoupled")
    
    results = {
        "CV Only (Raw)": df_cv,
        "Standard EKF (Coupled)": df_std,
        "Decoupled EKF (Proposed)": df_dec
    }
    
    for name, df in results.items():
        print(f"Workflow {name}: {len(df)} points")
        if not df.empty:
            print(f"  {name} - X mean: {df['x'].mean():.4f}, Y mean: {df['y'].mean():.4f}, Z mean: {df['z'].mean():.4f}")
    
    # Plot order: CV Only (Raw) first, then Standard EKF, then Decoupled on top
    # This ensures the EKF results are clearly visible over the noisy raw data
    plot_order = ["CV Only (Raw)", "Standard EKF (Coupled)", "Decoupled EKF (Proposed)"]
    sorted_results = {}
    for k in plot_order:
        if k in results:
            sorted_results[k] = results[k]
    
    visualize(sorted_results, "./outputs/workflow_comparison.png")
