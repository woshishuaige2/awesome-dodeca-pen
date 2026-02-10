"""
Sync and Merge Script
Synchronizes IMU-only data with CV-from-video data and merges them into a single JSON file.
"""

import json
import sys
from pathlib import Path
import numpy as np

def load_json(filepath):
    """Load JSON data from file."""
    with open(filepath, 'r') as f:
        return json.load(f)

def sync_and_merge(imu_json, cv_json, output_json, time_offset=0.0, method="manual"):
    """
    Synchronize and merge IMU and CV data streams.
    
    Args:
        imu_json: Path to IMU-only JSON file
        cv_json: Path to CV-from-video JSON file
        output_json: Path to output merged JSON file
        time_offset: Time offset to add to IMU timestamps (seconds)
        method: Synchronization method ("manual" or "auto")
    """
    print("Loading data files...")
    imu_data = load_json(imu_json)
    cv_data = load_json(cv_json)
    
    imu_readings = imu_data["imu_readings"]
    cv_readings = cv_data["cv_readings"]
    
    print(f"IMU readings: {len(imu_readings)}")
    print(f"CV readings: {len(cv_readings)}")
    
    if method == "auto":
        # Auto-sync: align based on first significant movement
        print("Auto-sync: Detecting first significant movement...")
        
        # Find first significant IMU movement
        imu_start_idx = 0
        for i in range(1, len(imu_readings)):
            accel = np.array(imu_readings[i]["accel"])
            accel_prev = np.array(imu_readings[i-1]["accel"])
            accel_diff = np.linalg.norm(accel - accel_prev)
            if accel_diff > 1.0:  # Threshold for significant movement
                imu_start_idx = i
                print(f"First significant IMU movement at index {i}")
                break
        
        # Find first CV detection (already filtered, so first frame is good)
        cv_start_idx = 0
        
        # Calculate time offset
        imu_start_time = imu_readings[imu_start_idx]["local_timestamp"]
        cv_start_time = cv_readings[cv_start_idx]["timestamp"]
        time_offset = cv_start_time - (imu_start_time - imu_data["metadata"]["start_time"])
        
        print(f"Calculated time offset: {time_offset:.3f} seconds")
    
    # Apply time offset to IMU readings
    print(f"Applying time offset: {time_offset:.3f} seconds")
    imu_start_time = imu_data["metadata"]["start_time"]
    for reading in imu_readings:
        reading["timestamp"] = reading["local_timestamp"] - imu_start_time + time_offset
    
    # Adjust CV timestamps to match
    cv_start_time = cv_data["metadata"].get("start_time", 0)
    for reading in cv_readings:
        # CV timestamps are already relative to video start
        pass
    
    # Create merged data structure
    merged_data = {
        "metadata": {
            "start_time": min(imu_start_time, cv_start_time),
            "tip_offset_body": cv_data["metadata"]["tip_offset_body"],
            "imu_to_tip_body": cv_data["metadata"]["imu_to_tip_body"],
            "time_offset": time_offset,
            "sync_method": method,
            "imu_source": str(imu_json),
            "cv_source": str(cv_json)
        },
        "imu_readings": imu_readings,
        "cv_readings": cv_readings
    }
    
    # Save merged data
    with open(output_json, "w") as f:
        json.dump(merged_data, f, indent=2)
    
    print(f"\nMerged data saved to {output_json}")
    print(f"Total IMU readings: {len(imu_readings)}")
    print(f"Total CV readings: {len(cv_readings)}")
    
    # Calculate time ranges
    if imu_readings:
        imu_time_range = (imu_readings[0]["timestamp"], imu_readings[-1]["timestamp"])
        print(f"IMU time range: {imu_time_range[0]:.3f} - {imu_time_range[1]:.3f} s")
    
    if cv_readings:
        cv_time_range = (cv_readings[0]["timestamp"], cv_readings[-1]["timestamp"])
        print(f"CV time range: {cv_time_range[0]:.3f} - {cv_time_range[1]:.3f} s")

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Synchronize and merge IMU and CV data")
    parser.add_argument("imu_json", help="Path to IMU-only JSON file")
    parser.add_argument("cv_json", help="Path to CV-from-video JSON file")
    parser.add_argument("--output", default="outputs/merged_data.json", help="Output merged JSON file")
    parser.add_argument("--offset", type=float, default=0.0, help="Manual time offset (seconds)")
    parser.add_argument("--method", choices=["manual", "auto"], default="manual", 
                        help="Synchronization method")
    args = parser.parse_args()

    # Ensure output directory exists
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)

    sync_and_merge(args.imu_json, args.cv_json, args.output, args.offset, args.method)

if __name__ == "__main__":
    main()
