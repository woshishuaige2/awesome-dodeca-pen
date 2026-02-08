import argparse
import json
import multiprocessing as mp
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

# Add project directories to sys.path
repo_root = Path(__file__).resolve().parents[1]
sys.path.append(str(repo_root / "IMU"))
sys.path.append(str(repo_root / "Computer_vision"))

# Try to import project modules
try:
    from app.dodeca_bridge import make_ekf_measurements, TIP_OFFSET_BODY, IMU_TO_TIP_BODY
    from app.monitor_ble import monitor_ble, StopCommand
    import run as cv_run
except ImportError as e:
    print(f"Import error: {e}")
    print("Ensure you are running this from the Code/IMU directory or paths are correct.")
    sys.exit(1)

class RawDataRecorder:
    def __init__(self, output_file="raw_sensor_data.json"):
        self.output_file = output_file
        self.data = {
            "metadata": {
                "start_time": time.time(),
                "tip_offset_body": TIP_OFFSET_BODY.tolist(),
                "imu_to_tip_body": IMU_TO_TIP_BODY.tolist(),
            },
            "imu_readings": [],
            "cv_readings": []
        }
        self.should_stop = False

    def record_imu(self, ble_queue):
        print("[Recorder] IMU recording started.")
        while not self.should_stop:
            try:
                # Use a timeout so we can check should_stop
                reading = ble_queue.get(timeout=0.1)
                if hasattr(reading, "to_json"):
                    reading_dict = reading.to_json()
                else:
                    # Fallback if to_json isn't available
                    reading_dict = {
                        "accel": reading.accel.tolist(),
                        "gyro": reading.gyro.tolist(),
                        "t": reading.t,
                        "pressure": reading.pressure
                    }
                reading_dict["local_timestamp"] = time.time()
                self.data["imu_readings"].append(reading_dict)
            except mp.queues.Empty:
                continue
            except Exception as e:
                print(f"[Recorder] IMU Error: {e}")

    def record_cv(self):
        print("[Recorder] CV recording started.")
        last_ts = -1
        while not self.should_stop:
            try:
                vis = make_ekf_measurements(TIP_OFFSET_BODY, IMU_TO_TIP_BODY)
                if vis is not None and vis["timestamp"] != last_ts:
                    last_ts = vis["timestamp"]
                    cv_entry = {
                        "timestamp": vis["timestamp"],
                        "local_timestamp": time.time(),
                        "imu_pos_cam": vis["imu_pos_cam"].tolist(),
                        "tip_pos_cam": vis["tip_pos_cam"].tolist(),
                        "R_cam": vis["R_cam"].tolist(),
                    }
                    self.data["cv_readings"].append(cv_entry)
            except Exception as e:
                print(f"[Recorder] CV Error: {e}")
            time.sleep(0.01) # Poll at ~100Hz for CV updates

    def save(self):
        self.data["metadata"]["end_time"] = time.time()
        self.data["metadata"]["imu_count"] = len(self.data["imu_readings"])
        self.data["metadata"]["cv_count"] = len(self.data["cv_readings"])
        
        with open(self.output_file, "w") as f:
            json.dump(self.data, f, indent=2)
        print(f"\n[Recorder] Data saved to {self.output_file}")
        print(f"[Recorder] Recorded {len(self.data['imu_readings'])} IMU and {len(self.data['cv_readings'])} CV samples.")

def main():
    parser = argparse.ArgumentParser(description="Record raw IMU and CV data for Dodeca Pen project.")
    parser.add_argument("--output", type=str, default="raw_sensor_data.json", help="Output JSON file path")
    parser.add_argument("--video", type=str, default=None, help="Video file to use instead of live camera")
    parser.add_argument("--duration", type=int, default=None, help="Duration to record in seconds")
    args = parser.parse_args()

    ble_queue = mp.Queue()
    ble_command_queue = mp.Queue()
    
    recorder = RawDataRecorder(args.output)
    
    # Start CV thread
    cv_thread = threading.Thread(
        target=cv_run.main, 
        kwargs={"headless": True, "cam_index": 0, "video_file": args.video}, 
        daemon=True
    )
    cv_thread.start()
    
    # Start BLE thread
    ble_thread = threading.Thread(
        target=monitor_ble, 
        args=(ble_queue, ble_command_queue), 
        daemon=True
    )
    ble_thread.start()
    
    # Start recording threads
    imu_rec_thread = threading.Thread(target=recorder.record_imu, args=(ble_queue,))
    cv_rec_thread = threading.Thread(target=recorder.record_cv)
    
    imu_rec_thread.start()
    cv_rec_thread.start()
    
    print(f"\nRecording started. Output: {args.output}")
    print("Press Ctrl+C to stop recording manually.")
    
    start_time = time.time()
    try:
        while True:
            if args.duration and (time.time() - start_time) > args.duration:
                print(f"Duration of {args.duration}s reached.")
                break
            if not cv_thread.is_alive() and args.video:
                print("Video processing finished.")
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopping recording...")
    finally:
        recorder.should_stop = True
        ble_command_queue.put(StopCommand())
        
        imu_rec_thread.join(timeout=1.0)
        cv_rec_thread.join(timeout=1.0)
        recorder.save()
        
        print("Exiting.")

if __name__ == "__main__":
    main()
