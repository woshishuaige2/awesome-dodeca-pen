"""
IMU and Video Recorder
Records IMU data and raw video separately without real-time CV processing.
This avoids computational overhead and allows offline CV processing later.
"""

import json
import multiprocessing as mp
import os
import sys
import threading
import time
from pathlib import Path
import cv2
import numpy as np

# Add project directories to sys.path
repo_root = Path(__file__).resolve().parents[1]
sys.path.append(str(repo_root / "IMU"))

# Import only IMU-related modules
try:
    from app.monitor_ble import monitor_ble, StopCommand
    from app.dodeca_bridge import CENTER_TO_TIP_BODY, IMU_OFFSET_BODY
except ImportError as e:
    print(f"Import error: {e}")
    print("Ensure you are running this from the Code/IMU directory or paths are correct.")
    sys.exit(1)


class IMUAndVideoRecorder:
    def __init__(self, output_dir="outputs/recording"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.imu_file = self.output_dir / "imu_data.json"
        self.video_file = self.output_dir / "video.mp4"
        self.metadata_file = self.output_dir / "metadata.json"
        
        self.imu_data = {
            "metadata": {
                "start_time": time.time(),
                "tip_offset_body": CENTER_TO_TIP_BODY.tolist(),
                "imu_to_tip_body": IMU_OFFSET_BODY.tolist(),
            },
            "imu_readings": []
        }
        
        self.video_metadata = {
            "start_time": None,
            "end_time": None,
            "frame_count": 0,
            "fps": 0,
            "resolution": [0, 0]
        }
        
        self.should_stop = False
        self.video_writer = None
        self.video_start_time = None

    def record_imu(self, ble_queue):
        """Record IMU data from BLE queue"""
        print("[IMU Recorder] Started")
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
                self.imu_data["imu_readings"].append(reading_dict)
                imu_count += 1
                if imu_count % 100 == 0:
                    print(f"[IMU Recorder] Received {imu_count} samples")
            except mp.queues.Empty:
                continue
            except Exception as e:
                print(f"[IMU Recorder] Error: {e}")

    def record_video(self, cam_index=0):
        """Record raw video from camera"""
        print("[Video Recorder] Starting camera...")
        cap = cv2.VideoCapture(cam_index)
        
        if not cap.isOpened():
            print("[Video Recorder] ERROR: Could not open camera")
            return
        
        # Set camera properties for optimal quality
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Get actual camera properties
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        
        print(f"[Video Recorder] Camera opened: {width}x{height} @ {fps} FPS")
        
        # Initialize video writer
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(
            str(self.video_file),
            fourcc,
            fps,
            (width, height)
        )
        
        if not self.video_writer.isOpened():
            print("[Video Recorder] ERROR: Could not create video writer")
            cap.release()
            return
        
        self.video_metadata["resolution"] = [width, height]
        self.video_metadata["fps"] = fps
        self.video_start_time = time.time()
        self.video_metadata["start_time"] = self.video_start_time
        
        frame_count = 0
        print("[Video Recorder] Recording started")
        
        while not self.should_stop:
            ret, frame = cap.read()
            if not ret:
                print("[Video Recorder] Failed to read frame")
                break
            
            self.video_writer.write(frame)
            frame_count += 1
            
            if frame_count % 100 == 0:
                print(f"[Video Recorder] Recorded {frame_count} frames")
        
        self.video_metadata["end_time"] = time.time()
        self.video_metadata["frame_count"] = frame_count
        
        print(f"[Video Recorder] Stopped. Total frames: {frame_count}")
        
        # Cleanup
        self.video_writer.release()
        cap.release()

    def save(self):
        """Save all recorded data"""
        # Save IMU data
        self.imu_data["metadata"]["end_time"] = time.time()
        self.imu_data["metadata"]["imu_count"] = len(self.imu_data["imu_readings"])
        
        with open(self.imu_file, "w") as f:
            json.dump(self.imu_data, f, indent=2)
        print(f"[Recorder] IMU data saved to {self.imu_file}")
        
        # Save video metadata
        with open(self.metadata_file, "w") as f:
            json.dump({
                "imu": self.imu_data["metadata"],
                "video": self.video_metadata
            }, f, indent=2)
        print(f"[Recorder] Metadata saved to {self.metadata_file}")
        
        print(f"\n[Recorder] Recording complete:")
        print(f"  IMU samples: {len(self.imu_data['imu_readings'])}")
        print(f"  Video frames: {self.video_metadata['frame_count']}")
        print(f"  Video file: {self.video_file}")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Record IMU data and raw video separately")
    parser.add_argument("--output", default="outputs/recording", help="Output directory")
    parser.add_argument("--camera", type=int, default=0, help="Camera index")
    args = parser.parse_args()

    recorder = IMUAndVideoRecorder(args.output)
    
    # Start BLE monitoring
    print("[Main] Starting BLE monitoring...")
    ble_queue = mp.Queue()
    ble_command_queue = mp.Queue()
    ble_thread = threading.Thread(
        target=monitor_ble, 
        args=(ble_queue, ble_command_queue),
        daemon=True
    )
    ble_thread.start()
    
    # Give BLE time to connect
    print("[Main] Waiting for BLE connection (3 seconds)...")
    time.sleep(3)

    # Start recording threads
    print("[Main] Starting recording threads...")
    imu_thread = threading.Thread(target=recorder.record_imu, args=(ble_queue,))
    video_thread = threading.Thread(target=recorder.record_video, args=(args.camera,))
    
    imu_thread.start()
    video_thread.start()

    print("\n" + "=" * 60)
    print("RECORDING STARTED")
    print("=" * 60)
    print("Recording IMU data and video simultaneously")
    print("Press Ctrl+C to stop recording")
    print("=" * 60 + "\n")

    try:
        # Keep main thread alive until user interrupts
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[Main] Stopping recording...")
        recorder.should_stop = True
    finally:
        # Send stop command to BLE thread
        ble_command_queue.put(StopCommand())
        
        # Wait for threads to finish
        imu_thread.join(timeout=2.0)
        video_thread.join(timeout=2.0)
        
        # Save all data
        recorder.save()
        
        print("\n[Main] Recording stopped successfully")


if __name__ == "__main__":
    main()
