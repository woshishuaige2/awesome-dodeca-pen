"""
Enhanced Raw Data Recorder with One-Euro Filtering
Records both CV and IMU data with One-Euro filter applied to CV readings.
This produces clean, smooth CV trajectories similar to offline video processing.
"""

import json
import multiprocessing as mp
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

# Add project directories to sys.path
repo_root = Path(__file__).resolve().parents[1]
sys.path.append(str(repo_root / "IMU"))
sys.path.append(str(repo_root / "Computer_vision" / "src"))

# Try to import project modules
try:
    from app.dodeca_bridge import make_ekf_measurements, TIP_OFFSET_BODY, IMU_TO_TIP_BODY
    from app.monitor_ble import monitor_ble, StopCommand
    from filter import OneEuroFilter
    import run as cv_run
except ImportError as e:
    print(f"Import error: {e}")
    print("Ensure you are running this from the Code/IMU directory or paths are correct.")
    sys.exit(1)

class FilteredDataRecorder:
    def __init__(self, output_file="raw_sensor_data_filtered.json"):
        self.output_file = output_file
        self.data = {
            "metadata": {
                "start_time": time.time(),
                "tip_offset_body": TIP_OFFSET_BODY.tolist(),
                "imu_to_tip_body": IMU_TO_TIP_BODY.tolist(),
                "filtered": True,
                "filter_type": "OneEuro"
            },
            "imu_readings": [],
            "cv_readings": []
        }
        self.should_stop = False
        
        # One-Euro filters (initialized on first CV reading)
        self.filters_initialized = False
        self.filter_x = None
        self.filter_y = None
        self.filter_z = None
        self.filter_qw = None
        self.filter_qx = None
        self.filter_qy = None
        self.filter_qz = None

    def record_imu(self, ble_queue):
        print("[Recorder] IMU recording started.")
        imu_count = 0
        while not self.should_stop:
            try:
                reading = ble_queue.get(timeout=0.1)
                if hasattr(reading, "to_json"):
                    reading_dict = reading.to_json()
                else:
                    reading_dict = {
                        "accel": reading.accel.tolist(),
                        "gyro": reading.gyro.tolist(),
                        "t": reading.t,
                        "pressure": reading.pressure
                    }
                reading_dict["local_timestamp"] = time.time()
                self.data["imu_readings"].append(reading_dict)
                imu_count += 1
                if imu_count % 100 == 0:
                    print(f"[Recorder] IMU: received {imu_count} samples")
            except mp.queues.Empty:
                continue
            except Exception as e:
                print(f"[Recorder] IMU Error: {e}")

    def record_cv(self):
        print("[Recorder] CV recording started with One-Euro filtering.")
        last_ts = -1
        cv_count = 0
        while not self.should_stop:
            try:
                vis = make_ekf_measurements(TIP_OFFSET_BODY, IMU_TO_TIP_BODY)
                if vis is not None and vis["timestamp"] != last_ts:
                    last_ts = vis["timestamp"]
                    current_time = time.time()
                    
                    # Extract raw data
                    imu_pos = vis["imu_pos_cam"]
                    R_cam = vis["R_cam"]
                    
                    # Convert rotation matrix to quaternion
                    quat = R.from_matrix(R_cam).as_quat()  # [x, y, z, w]
                    quat = np.array([quat[3], quat[0], quat[1], quat[2]])  # [w, x, y, z]
                    
                    # Initialize filters on first reading
                    if not self.filters_initialized:
                        self.filter_x = OneEuroFilter(current_time, imu_pos[0])
                        self.filter_y = OneEuroFilter(current_time, imu_pos[1])
                        self.filter_z = OneEuroFilter(current_time, imu_pos[2])
                        self.filter_qw = OneEuroFilter(current_time, quat[0])
                        self.filter_qx = OneEuroFilter(current_time, quat[1])
                        self.filter_qy = OneEuroFilter(current_time, quat[2])
                        self.filter_qz = OneEuroFilter(current_time, quat[3])
                        self.filters_initialized = True
                        
                        # Store first reading as-is
                        cv_entry = {
                            "timestamp": vis["timestamp"],
                            "local_timestamp": current_time,
                            "imu_pos_cam": imu_pos.tolist(),
                            "tip_pos_cam": imu_pos.tolist(),
                            "R_cam": R_cam.tolist(),
                        }
                    else:
                        # Apply One-Euro filter
                        filtered_x = self.filter_x.filter_signal(current_time, imu_pos[0])
                        filtered_y = self.filter_y.filter_signal(current_time, imu_pos[1])
                        filtered_z = self.filter_z.filter_signal(current_time, imu_pos[2])
                        
                        filtered_qw = self.filter_qw.filter_signal(current_time, quat[0])
                        filtered_qx = self.filter_qx.filter_signal(current_time, quat[1])
                        filtered_qy = self.filter_qy.filter_signal(current_time, quat[2])
                        filtered_qz = self.filter_qz.filter_signal(current_time, quat[3])
                        
                        # Normalize quaternion
                        filtered_quat = np.array([filtered_qw, filtered_qx, filtered_qy, filtered_qz])
                        quat_norm = np.linalg.norm(filtered_quat)
                        if quat_norm > 1e-6:
                            filtered_quat = filtered_quat / quat_norm
                        else:
                            filtered_quat = quat  # Fallback to unfiltered
                        
                        # Reconstruct rotation matrix
                        filtered_R = R.from_quat([filtered_quat[1], filtered_quat[2], 
                                                   filtered_quat[3], filtered_quat[0]]).as_matrix()
                        
                        cv_entry = {
                            "timestamp": vis["timestamp"],
                            "local_timestamp": current_time,
                            "imu_pos_cam": [filtered_x, filtered_y, filtered_z],
                            "tip_pos_cam": [filtered_x, filtered_y, filtered_z],
                            "R_cam": filtered_R.tolist(),
                        }
                    
                    self.data["cv_readings"].append(cv_entry)
                    cv_count += 1
                    if cv_count % 50 == 0:
                        print(f"[Recorder] CV: received {cv_count} filtered samples")
                        
            except Exception as e:
                print(f"[Recorder] CV Error: {e}")
                import traceback
                traceback.print_exc()
            time.sleep(0.01)

    def save(self):
        self.data["metadata"]["end_time"] = time.time()
        self.data["metadata"]["imu_count"] = len(self.data["imu_readings"])
        self.data["metadata"]["cv_count"] = len(self.data["cv_readings"])
        
        with open(self.output_file, "w") as f:
            json.dump(self.data, f, indent=2)
        print(f"\n[Recorder] Data saved to {self.output_file}")
        print(f"[Recorder] Recorded {len(self.data['imu_readings'])} IMU and {len(self.data['cv_readings'])} filtered CV samples.")

def preview_status(should_stop_event=None):
    """Monitor and display marker detection status."""
    print("[Preview] Starting CV preview...")
    last_status = None
    while should_stop_event is None or not should_stop_event.is_set():
        try:
            vis = make_ekf_measurements(TIP_OFFSET_BODY, IMU_TO_TIP_BODY)
            if vis is not None:
                current_status = "DETECTED"
                if current_status != last_status:
                    timestamp = time.strftime("%H:%M:%S")
                    print(f"[{timestamp}] Marker: {current_status} | Pos: ({vis['tip_pos_cam'][0]:.3f}, {vis['tip_pos_cam'][1]:.3f}, {vis['tip_pos_cam'][2]:.3f})")
                    last_status = current_status
            else:
                current_status = "NOT DETECTED"
                if current_status != last_status:
                    timestamp = time.strftime("%H:%M:%S")
                    print(f"[{timestamp}] Marker: {current_status}")
                    last_status = current_status
        except Exception as e:
            print(f"[Preview] Error: {e}")
        time.sleep(0.5)

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Record filtered CV and IMU data")
    parser.add_argument("--output", default="outputs/my_data.json", help="Output JSON file")
    parser.add_argument("--video", help="Optional: Path to offline video file for CV processing")
    args = parser.parse_args()

    # Ensure output directory exists
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)

    recorder = FilteredDataRecorder(args.output)
    
    # Start CV thread (Crucial for initializing camera and processing frames)
    cv_process_thread = threading.Thread(
        target=cv_run.main, 
        kwargs={"headless": True, "cam_index": 0, "video_file": args.video}, 
        daemon=True
    )
    cv_process_thread.start()
    
    print("[Main] Initializing CV system (waiting 2 seconds)...")
    time.sleep(2)

    # Start BLE monitoring
    ble_queue = mp.Queue()
    ble_command_queue = mp.Queue()  # Command queue for stopping BLE
    # Using a Thread for BLE instead of a Process can be more stable on some systems
    ble_thread = threading.Thread(
        target=monitor_ble, 
        args=(ble_queue, ble_command_queue),
        daemon=True
    )
    ble_thread.start()

    # Start preview in separate thread
    stop_preview = threading.Event()
    preview_thread = threading.Thread(target=preview_status, args=(stop_preview,))
    preview_thread.daemon = True
    preview_thread.start()

    # Start recording threads
    imu_rec_thread = threading.Thread(target=recorder.record_imu, args=(ble_queue,))
    cv_rec_thread = threading.Thread(target=recorder.record_cv)
    
    imu_rec_thread.start()
    cv_rec_thread.start()

    print("\n=== Recording Started ===")
    print("Press Ctrl+C to stop recording\n")

    try:
        # Keep main thread alive until user interrupts
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[Main] Stopping...")
        recorder.should_stop = True
        stop_preview.set()
    finally:
        # Send stop command to BLE thread
        ble_command_queue.put(StopCommand())
        
        imu_rec_thread.join(timeout=1.0)
        cv_rec_thread.join(timeout=1.0)
        recorder.save()

if __name__ == "__main__":
    main()
