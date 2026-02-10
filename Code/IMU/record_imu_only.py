"""
IMU-Only Recording Script
Records only IMU sensor data with high-precision timestamps.
Use this in parallel with video recording for hybrid CV+IMU workflow.
"""

import json
import multiprocessing as mp
import sys
import time
from pathlib import Path

# Add project directories to sys.path
repo_root = Path(__file__).resolve().parents[1]
sys.path.append(str(repo_root / "IMU"))

try:
    from app.monitor_ble import monitor_ble, StopCommand
except ImportError as e:
    print(f"Import error: {e}")
    print("Ensure you are running this from the Code/IMU directory.")
    sys.exit(1)

class IMURecorder:
    def __init__(self, output_file="imu_only_data.json"):
        self.output_file = output_file
        self.data = {
            "metadata": {
                "start_time": time.time(),
                "recording_type": "imu_only"
            },
            "imu_readings": []
        }
        self.should_stop = False

    def record_imu(self, ble_queue):
        print("[IMU Recorder] Recording started.")
        print("[IMU Recorder] Press Ctrl+C to stop recording.")
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
                    print(f"[IMU Recorder] Received {imu_count} samples")
            except mp.queues.Empty:
                continue
            except Exception as e:
                print(f"[IMU Recorder] Error: {e}")

    def save(self):
        self.data["metadata"]["end_time"] = time.time()
        self.data["metadata"]["imu_count"] = len(self.data["imu_readings"])
        self.data["metadata"]["duration"] = self.data["metadata"]["end_time"] - self.data["metadata"]["start_time"]
        
        with open(self.output_file, "w") as f:
            json.dump(self.data, f, indent=2)
        print(f"\n[IMU Recorder] Data saved to {self.output_file}")
        print(f"[IMU Recorder] Recorded {len(self.data['imu_readings'])} IMU samples over {self.data['metadata']['duration']:.2f} seconds.")

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Record IMU data only")
    parser.add_argument("--output", default="outputs/imu_only_data.json", help="Output JSON file")
    args = parser.parse_args()

    # Ensure output directory exists
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)

    recorder = IMURecorder(args.output)
    
    # Start BLE monitoring
    ble_queue = mp.Queue()
    stop_cmd = StopCommand()
    ble_process = mp.Process(target=monitor_ble, args=(ble_queue, stop_cmd))
    ble_process.start()

    try:
        recorder.record_imu(ble_queue)
    except KeyboardInterrupt:
        print("\n[IMU Recorder] Stopping...")
        recorder.should_stop = True
    finally:
        stop_cmd.stop()
        ble_process.join(timeout=2)
        if ble_process.is_alive():
            ble_process.terminate()
        recorder.save()

if __name__ == "__main__":
    main()
