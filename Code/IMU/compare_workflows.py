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

from app.filter import DpointFilter, initial_state
from app.monitor_ble import StylusReading
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

    # Mode-specific logic for EKF updates
    def custom_fuse_camera(fs, imu_pos, orientation_quat, meas_noise):
        if mode == "standard":
            # Standard 7D Update (Position + Orientation)
            # Use the original 7D measurement model
            h = np.concatenate((fs.state[fc.i_pos], fs.state[fc.i_quat]))
            H = np.zeros((7, fc.STATE_SIZE))
            H[0:3, fc.i_pos] = np.eye(3)
            H[3:7, fc.i_quat] = np.eye(4)
            z = np.concatenate((imu_pos.flatten(), orientation_quat))
            # Use a noise matrix that trusts orientation more to see the jitter
            R = np.eye(7)
            R[0:3, 0:3] *= 1e-5
            R[3:7, 3:7] *= 1e-5 
        else:
            # Decoupled 3D Update (Position Only)
            # Use the new 3D measurement model
            h = fs.state[fc.i_pos]
            H = np.zeros((3, fc.STATE_SIZE))
            H[0:3, fc.i_pos] = np.eye(3)
            z = imu_pos.flatten()
            R = np.eye(3) * 1e-5
            
        # We must call ekf_correct directly to bypass the monkey-patched fuse_camera
        state, statecov = fc.ekf_correct(fs.state, fs.statecov, h, H, z, R)
        state[fc.i_quat] = fc.repair_quaternion(state[fc.i_quat])
        return fc.FilterState(state, statecov)

    # Monkey patch the fuse_camera function
    fc.fuse_camera = custom_fuse_camera

    # Initialize Filter
    dt = 0.01
    if len(imu_readings) > 1:
        dt = imu_readings[1]["local_timestamp"] - imu_readings[0]["local_timestamp"]
    
    filter = DpointFilter(dt=dt, smoothing_length=15, camera_delay=5)
    trajectory = []

    for type, ts, reading in all_events:
        if type == "IMU":
            sr = StylusReading.from_json(reading)
            filter.update_imu(sr.accel, sr.gyro)
        elif type == "CV":
            imu_pos = np.array(reading["imu_pos_cam"])
            r_cam = np.array(reading["R_cam"])
            q_cam = Quaternion(matrix=r_cam).elements
            
            if mode == "cv_only":
                filter.fs = fc.FilterState(initial_state(imu_pos, q_cam).state, filter.fs.statecov)
                tips = [imu_pos]
            elif mode == "standard":
                # For standard mode, we force the update to use camera orientation
                # By bypassing the smoothed orientation in filter.py
                filter.fs = custom_fuse_camera(filter.fs, imu_pos, q_cam, np.eye(7)*1e-5)
                tips = [filter.fs.state[fc.i_pos]]
            else:
                tips = filter.update_camera(imu_pos, r_cam)
            
            if tips:
                trajectory.append({"t": ts, "x": tips[-1][0], "y": tips[-1][1], "z": tips[-1][2]})

    # Restore original functions if modified
    fc.camera_measurement = original_camera_measurement
    fc.fuse_camera = original_fuse_camera
    
    return pd.DataFrame(trajectory)

def visualize(results_dict, output_path):
    fig = plt.figure(figsize=(18, 12))
    
    # 3D Plot
    ax = fig.add_subplot(221, projection='3d')
    # Use different zorder and alpha to ensure visibility
    # We want CV Only to be visible but not overwhelming
    for i, (name, df) in enumerate(results_dict.items()):
        if not df.empty:
            if "CV Only" in name:
                alpha = 0.5
                linewidth = 1.0
                color = 'gray' # Use a distinct color for raw benchmark
                zorder = 1
            elif "Standard" in name:
                alpha = 0.7
                linewidth = 1.5
                color = 'orange'
                zorder = 2
            else: # Decoupled
                alpha = 0.9
                linewidth = 2.0
                color = 'green'
                zorder = 3
            
            ax.plot(df['x'], df['y'], df['z'], label=name, alpha=alpha, linewidth=linewidth, color=color, zorder=zorder)
            
    ax.set_title("3D Pen-Tip Trajectory")
    # Set a better initial perspective (elevation, azimuth)
    ax.view_init(elev=30, azim=45)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()

    # XY Plane (Top view)
    ax2 = fig.add_subplot(222)
    for name, df in results_dict.items():
        if not df.empty:
            if "CV Only" in name:
                ax2.plot(df['x'], df['y'], label=name, alpha=0.5, color='gray', linewidth=1.0, zorder=1)
            elif "Standard" in name:
                ax2.plot(df['x'], df['y'], label=name, alpha=0.7, color='orange', linewidth=1.5, zorder=2)
            else:
                ax2.plot(df['x'], df['y'], label=name, alpha=0.9, color='green', linewidth=2.0, zorder=3)
    ax2.set_title("XY Plane (Top View)")
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.axis('equal') # Maintain aspect ratio for the rectangle
    ax2.grid(True)
    ax2.legend()

    # Z-axis over time (Absolute to see lifting)
    ax3 = fig.add_subplot(223)
    for name, df in results_dict.items():
        if not df.empty:
            ax3.plot(df['t'] - df['t'].iloc[0], df['z'], label=name, alpha=0.8)
    ax3.set_title("Z-Axis (Absolute Height)")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Z (m)")
    ax3.grid(True)
    ax3.legend()

    # Z-axis jitter (Normalized)
    ax4 = fig.add_subplot(224)
    for name, df in results_dict.items():
        if not df.empty:
            # Use rolling mean to see jitter relative to local trend
            z_norm = df['z'] - df['z'].rolling(window=10, center=True).mean()
            ax4.plot(df['t'] - df['t'].iloc[0], z_norm, label=name, alpha=0.8)
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
