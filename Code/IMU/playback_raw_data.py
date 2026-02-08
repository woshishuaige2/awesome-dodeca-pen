import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd

# Add project directories to sys.path
repo_root = Path(__file__).resolve().parents[1]
sys.path.append(str(repo_root / "IMU"))

# Import project modules
try:
    from app.filter import DpointFilter
    from app.monitor_ble import StylusReading
except ImportError as e:
    print(f"Import error: {e}")
    sys.exit(1)

def playback(input_file, output_csv="playback_trajectory.csv"):
    if not os.path.exists(input_file):
        print(f"Error: Input file {input_file} not found.")
        return

    with open(input_file, "r") as f:
        data = json.load(f)

    # Reconstruct metadata and parameters
    # Note: In a real scenario, you might want to use the recorded offsets
    # metadata = data.get("metadata", {})
    
    imu_readings = data.get("imu_readings", [])
    cv_readings = data.get("cv_readings", [])

    # Combine and sort all readings by local_timestamp to simulate real-time arrival
    all_events = []
    for r in imu_readings:
        all_events.append(("IMU", r["local_timestamp"], r))
    for r in cv_readings:
        all_events.append(("CV", r["local_timestamp"], r))
    
    all_events.sort(key=lambda x: x[1])

    print(f"Starting playback of {len(all_events)} events...")

    # Initialize filter (using common defaults from app.py)
    dt = 1/100.0 # Assuming 100Hz IMU
    # Calculate actual dt if possible from first two IMU readings
    if len(imu_readings) > 1:
        dt = imu_readings[1]["local_timestamp"] - imu_readings[0]["local_timestamp"]

    # filter = DpointFilter(dt=dt, smoothing_length=10, camera_delay=5)
    # Using larger smoothing for offline analysis
    filter = DpointFilter(dt=dt, smoothing_length=20, camera_delay=10)

    trajectory = []
    
    for i, (type, ts, reading) in enumerate(all_events):
        if type == "IMU":
            # Convert dict back to StylusReading
            sr = StylusReading.from_json(reading)
            filter.update_imu(sr.accel, sr.gyro)
        elif type == "CV":
            # Update camera
            imu_pos = np.array(reading["imu_pos_cam"])
            r_cam = np.array(reading["R_cam"])
            
            # This returns a list of smoothed/predicted tip positions
            smoothed_tips = filter.update_camera(imu_pos, r_cam)
            
            if smoothed_tips:
                tip = smoothed_tips[-1]
                trajectory.append({
                    "t": ts,
                    "x": tip[0],
                    "y": tip[1],
                    "z": tip[2],
                    "event_type": "CV_UPDATE"
                })
        
        if i % 100 == 0:
            print(f"Processed {i}/{len(all_events)} events...")

    # Save results
    if trajectory:
        df = pd.DataFrame(trajectory)
        df.to_csv(output_csv, index=False)
        print(f"Playback complete. Trajectory saved to {output_csv}")
    else:
        print("No trajectory generated.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Playback recorded raw sensor data through the EKF.")
    parser.add_argument("input", type=str, help="Input JSON file with raw sensor data")
    parser.add_argument("--output", type=str, default="outputs/playback_trajectory.csv", help="Output CSV file path")
    args = parser.parse_args()

    playback(args.input, args.output)
