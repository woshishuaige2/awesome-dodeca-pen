import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from pathlib import Path

def load_and_analyze_data(json_file):
    """Load and analyze the recorded data."""
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    imu_readings = data.get("imu_readings", [])
    cv_readings = data.get("cv_readings", [])
    
    print(f"IMU readings: {len(imu_readings)}")
    print(f"CV readings: {len(cv_readings)}")
    
    return data, imu_readings, cv_readings

def filter_stationary_points(positions, timestamps, movement_threshold=0.002):
    """Remove stationary points to reduce noise."""
    filtered_positions = [positions[0]]
    filtered_timestamps = [timestamps[0]]
    filtered_indices = [0]
    
    for i in range(1, len(positions)):
        movement = np.linalg.norm(positions[i] - filtered_positions[-1])
        if movement > movement_threshold:
            filtered_positions.append(positions[i])
            filtered_timestamps.append(timestamps[i])
            filtered_indices.append(i)
    
    return np.array(filtered_positions), np.array(filtered_timestamps), filtered_indices

def smooth_trajectory(positions, window_length=11, polyorder=3):
    """Apply Savitzky-Golay filter to smooth the trajectory."""
    if len(positions) < window_length:
        window_length = len(positions) if len(positions) % 2 == 1 else len(positions) - 1
        if window_length < polyorder + 2:
            return positions
    
    smoothed = np.zeros_like(positions)
    for i in range(3):
        smoothed[:, i] = savgol_filter(positions[:, i], window_length, polyorder)
    return smoothed

def visualize_clean_trajectory(positions, timestamps, output_path):
    """Create comprehensive visualization of the cleaned trajectory."""
    fig = plt.figure(figsize=(16, 12))
    
    # Normalize timestamps
    t = timestamps - timestamps[0]
    
    # 3D trajectory
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.plot(positions[:,0], positions[:,1], positions[:,2], 'b-', linewidth=2, alpha=0.8)
    ax1.scatter(positions[0,0], positions[0,1], positions[0,2], c='green', s=100, marker='o', label='Start')
    ax1.scatter(positions[-1,0], positions[-1,1], positions[-1,2], c='red', s=100, marker='x', label='End')
    ax1.set_xlabel('X (m)', fontsize=12)
    ax1.set_ylabel('Y (m)', fontsize=12)
    ax1.set_zlabel('Z (m)', fontsize=12)
    ax1.set_title('3D Pen-Tip Trajectory (Cleaned)', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # XY plane (top view)
    ax2 = fig.add_subplot(222)
    ax2.plot(positions[:,0], positions[:,1], 'b-', linewidth=2, alpha=0.8)
    ax2.scatter(positions[:,0], positions[:,1], c=t, cmap='viridis', s=20, alpha=0.6)
    ax2.scatter(positions[0,0], positions[0,1], c='green', s=150, marker='o', edgecolors='black', linewidths=2, label='Start', zorder=10)
    ax2.scatter(positions[-1,0], positions[-1,1], c='red', s=150, marker='x', linewidths=3, label='End', zorder=10)
    ax2.set_xlabel('X (m)', fontsize=12)
    ax2.set_ylabel('Y (m)', fontsize=12)
    ax2.set_title('XY Plane (Top View)', fontsize=14, fontweight='bold')
    ax2.axis('equal')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Time series
    ax3 = fig.add_subplot(223)
    ax3.plot(t, positions[:,0], label='X', linewidth=2)
    ax3.plot(t, positions[:,1], label='Y', linewidth=2)
    ax3.plot(t, positions[:,2], label='Z', linewidth=2)
    ax3.set_xlabel('Time (s)', fontsize=12)
    ax3.set_ylabel('Position (m)', fontsize=12)
    ax3.set_title('Position vs Time', fontsize=14, fontweight='bold')
    ax3.legend(fontsize=11)
    ax3.grid(True, alpha=0.3)
    
    # Z-axis detail (to see lifting motion)
    ax4 = fig.add_subplot(224)
    ax4.plot(t, positions[:,2], 'b-', linewidth=2)
    ax4.set_xlabel('Time (s)', fontsize=12)
    ax4.set_ylabel('Z Position (m)', fontsize=12)
    ax4.set_title('Z-Axis (Vertical Motion)', fontsize=14, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=200)
    print(f"Saved cleaned trajectory visualization to {output_path}")

def main():
    # Load data
    json_file = "outputs/my_data.json"
    data, imu_readings, cv_readings = load_and_analyze_data(json_file)
    
    # Extract CV positions
    positions = np.array([r['imu_pos_cam'] for r in cv_readings])
    timestamps = np.array([r['local_timestamp'] for r in cv_readings])
    
    print(f"\nOriginal data statistics:")
    print(f"  Total points: {len(positions)}")
    print(f"  X range: {positions[:,0].max() - positions[:,0].min():.4f}m")
    print(f"  Y range: {positions[:,1].max() - positions[:,1].min():.4f}m")
    print(f"  Z range: {positions[:,2].max() - positions[:,2].min():.4f}m")
    
    # Filter stationary points
    filtered_pos, filtered_ts, indices = filter_stationary_points(positions, timestamps, movement_threshold=0.002)
    print(f"\nAfter filtering stationary points:")
    print(f"  Remaining points: {len(filtered_pos)}")
    
    # Smooth trajectory
    smoothed_pos = smooth_trajectory(filtered_pos, window_length=11, polyorder=3)
    print(f"\nAfter smoothing:")
    print(f"  Final points: {len(smoothed_pos)}")
    
    # Visualize
    visualize_clean_trajectory(smoothed_pos, filtered_ts, "outputs/clean_trajectory.png")
    
    # Save cleaned data to CSV
    df = pd.DataFrame({
        't': filtered_ts - filtered_ts[0],
        'x': smoothed_pos[:,0],
        'y': smoothed_pos[:,1],
        'z': smoothed_pos[:,2]
    })
    df.to_csv("outputs/clean_trajectory.csv", index=False)
    print(f"Saved cleaned trajectory data to outputs/clean_trajectory.csv")

if __name__ == "__main__":
    main()
