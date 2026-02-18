"""
Merge IMU and CV Data with Synchronization
Combines separately recorded IMU and CV data into a single my_data.json format.
Handles timestamp alignment and synchronization.
"""

import json
import sys
from pathlib import Path
import numpy as np
import argparse


def load_json(file_path):
    """Load JSON file"""
    with open(file_path, 'r') as f:
        return json.load(f)


def save_json(data, file_path):
    """Save JSON file"""
    output_path = Path(file_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)


def find_sync_point(imu_data, cv_data, method="first_detection"):
    """
    Find synchronization point between IMU and CV data.
    
    Methods:
    - "first_detection": Align at first CV detection (default)
    - "manual": Use manually specified offset
    - "cross_correlation": Use motion correlation (future enhancement)
    
    Returns:
        (imu_offset, cv_offset) - timestamps to use as t=0 for each stream
    """
    imu_readings = imu_data.get("imu_readings", [])
    cv_readings = cv_data.get("cv_readings", [])
    
    if len(imu_readings) == 0:
        print("[Sync] Warning: No IMU readings found")
        imu_start = 0
    else:
        imu_start = imu_readings[0]["local_timestamp"]
    
    if len(cv_readings) == 0:
        print("[Sync] Warning: No CV readings found")
        cv_start = 0
    else:
        cv_start = cv_readings[0]["local_timestamp"]
    
    if method == "first_detection":
        # Align both streams to start at the first CV detection
        # CRITICAL: Use the SAME sync point for both IMU and CV to ensure exact alignment
        print(f"[Sync] Using first_detection method")
        print(f"  IMU start: {imu_start:.3f}")
        print(f"  CV start: {cv_start:.3f}")
        
        # We assume the first CV detection happened at the same real-world time 
        # as the IMU reading with the closest timestamp.
        # However, for simplicity and since they are recorded on the same machine,
        # we can just use the absolute difference if they share the same clock.
        
        # ALWAYS use CV start as the reference point (t=0)
        # This ensures that the first CV detection is at exactly t=0
        # and IMU data is aligned relative to that point
        sync_point = cv_start
        time_diff = cv_start - imu_start
        
        print(f"  [Sync] Using CV start as t=0 reference: {sync_point:.3f}")
        print(f"  [Sync] Time difference: {time_diff:.3f}s")
        
        # Check if IMU and CV were recorded simultaneously
        if abs(time_diff) > 10.0:
            print(f"\n  [WARNING] Large time gap detected ({abs(time_diff):.1f}s)!")
            print(f"  This suggests the recordings might not be perfectly synchronized.")
            print(f"  We will use the first CV detection as the absolute t=0 and shift IMU accordingly.")
            
            # If the gap is huge (likely different sessions), we force the IMU to start at t=0 too
            if abs(time_diff) > 3600: # 1 hour
                print(f"  [CRITICAL] Gap > 1 hour. Forcing IMU to start at t=0 as well.")
                return imu_start, cv_start
        
        # Return the same sync point for both streams to maintain relative timing
        return sync_point, sync_point
    
    else:
        raise ValueError(f"Unknown sync method: {method}")


def align_timestamps(readings, sync_offset, allow_negative=False):
    """
    Align timestamps to start from sync point.
    CRITICAL: Both 'timestamp' and 'local_timestamp' must be aligned for EKF to work correctly.
    
    Args:
        readings: List of readings with "local_timestamp" field
        sync_offset: Timestamp to use as t=0
        allow_negative: If True, include readings before sync point (with negative timestamps)
    
    Returns:
        List of readings with aligned timestamps
    """
    aligned = []
    for reading in readings:
        # Include readings at or after sync point, or before if allow_negative=True
        if allow_negative or reading["local_timestamp"] >= sync_offset:
            aligned_reading = reading.copy()
            # CRITICAL: Update both timestamp fields to maintain consistency
            aligned_reading["timestamp"] = reading["local_timestamp"] - sync_offset
            aligned_reading["local_timestamp"] = reading["local_timestamp"] - sync_offset
            aligned.append(aligned_reading)
    
    return aligned


def merge_data(imu_file, cv_file, output_file, sync_method="first_detection", manual_offset=0.0):
    """
    Merge IMU and CV data into unified format.
    
    Args:
        imu_file: Path to IMU data JSON
        cv_file: Path to CV data JSON
        output_file: Path to output merged JSON
        sync_method: Synchronization method
        manual_offset: Manual time offset (CV - IMU) in seconds
    """
    print("=" * 60)
    print("MERGING IMU AND CV DATA")
    print("=" * 60)
    
    # Load data
    print(f"[Merge] Loading IMU data from: {imu_file}")
    imu_data = load_json(imu_file)
    
    print(f"[Merge] Loading CV data from: {cv_file}")
    cv_data = load_json(cv_file)
    
    # Find synchronization point
    if sync_method == "manual":
        print(f"[Sync] Using manual offset: {manual_offset:.3f} seconds")
        imu_readings = imu_data.get("imu_readings", [])
        cv_readings = cv_data.get("cv_readings", [])
        
        if len(imu_readings) > 0 and len(cv_readings) > 0:
            imu_offset = imu_readings[0]["local_timestamp"]
            cv_offset = cv_readings[0]["local_timestamp"] + manual_offset
        else:
            imu_offset = 0
            cv_offset = manual_offset
    else:
        imu_offset, cv_offset = find_sync_point(imu_data, cv_data, sync_method)
    
    # Align timestamps
    # CRITICAL: Allow IMU readings before CV start (negative timestamps) for proper EKF initialization
    print(f"[Merge] Aligning IMU timestamps (offset: {imu_offset:.3f}, allow_negative=True)")
    aligned_imu = align_timestamps(imu_data.get("imu_readings", []), imu_offset, allow_negative=True)
    
    print(f"[Merge] Aligning CV timestamps (offset: {cv_offset:.3f})")
    aligned_cv = align_timestamps(cv_data.get("cv_readings", []), cv_offset, allow_negative=False)
    
    # Create merged data structure (matching my_data.json format)
    merged_data = {
        "metadata": {
            "start_time": min(imu_offset, cv_offset),
            "end_time": max(
                aligned_imu[-1]["local_timestamp"] if aligned_imu else imu_offset,
                aligned_cv[-1]["local_timestamp"] if aligned_cv else cv_offset
            ),
            "tip_offset_body": cv_data["metadata"].get("tip_offset_body", [0, 0, 0]),
            "imu_to_tip_body": cv_data["metadata"].get("imu_to_tip_body", [0, 0, 0]),
            "filtered": cv_data["metadata"].get("filtered", False),
            "filter_type": cv_data["metadata"].get("filter_type", "None"),
            "imu_count": len(aligned_imu),
            "cv_count": len(aligned_cv),
            "sync_method": sync_method,
            "imu_sync_offset": imu_offset,
            "cv_sync_offset": cv_offset,
        },
        "imu_readings": aligned_imu,
        "cv_readings": aligned_cv
    }
    
    # Save merged data
    print(f"[Merge] Saving merged data to: {output_file}")
    save_json(merged_data, output_file)
    
    # Print summary
    print("\n" + "=" * 60)
    print("MERGE COMPLETE")
    print("=" * 60)
    print(f"IMU readings: {len(aligned_imu)}")
    print(f"CV readings: {len(aligned_cv)}")
    
    if aligned_imu and aligned_cv:
        imu_duration = aligned_imu[-1]["timestamp"] - aligned_imu[0]["timestamp"]
        cv_duration = aligned_cv[-1]["timestamp"] - aligned_cv[0]["timestamp"]
        print(f"IMU duration: {imu_duration:.2f} seconds")
        print(f"CV duration: {cv_duration:.2f} seconds")
        print(f"IMU sample rate: {len(aligned_imu)/imu_duration:.1f} Hz")
        print(f"CV sample rate: {len(aligned_cv)/cv_duration:.1f} Hz")
    
    print(f"\nOutput saved to: {output_file}")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="Merge IMU and CV data with synchronization")
    parser.add_argument("imu_file", help="Path to IMU data JSON file")
    parser.add_argument("cv_file", help="Path to CV data JSON file")
    parser.add_argument("--output", default="outputs/merged_data.json", help="Output merged JSON file (use my_data.json for compare_workflows.py)")
    parser.add_argument("--sync", default="first_detection", 
                       choices=["first_detection", "manual"],
                       help="Synchronization method")
    parser.add_argument("--offset", type=float, default=0.0,
                       help="Manual time offset in seconds (CV - IMU), only used with --sync manual")
    
    args = parser.parse_args()
    
    merge_data(
        args.imu_file,
        args.cv_file,
        args.output,
        sync_method=args.sync,
        manual_offset=args.offset
    )


if __name__ == "__main__":
    main()
