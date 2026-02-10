"""
Video-to-CV Processing Script
Processes a pre-recorded video and extracts filtered CV pose estimates.
Outputs a JSON file compatible with the playback workflow.
"""

import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# Add project directories to sys.path
repo_root = Path(__file__).resolve().parents[1]
sys.path.append(str(repo_root / "IMU"))
sys.path.append(str(repo_root / "Computer_vision"))

try:
    import DoDecahedronUtils as dodecapen
    from app.dodeca_bridge import TIP_OFFSET_BODY, IMU_TO_TIP_BODY
    from filter import OneEuroFilter
except ImportError as e:
    print(f"Import error: {e}")
    print("Ensure DoDecahedronUtils and filter modules are available.")
    sys.exit(1)

def process_video_to_cv_data(video_path, output_json, tip_offset=None):
    """
    Process a video file and extract filtered CV pose estimates.
    
    Args:
        video_path: Path to the input video file
        output_json: Path to the output JSON file
        tip_offset: Tip offset in dodecahedron frame (4x1 array)
    """
    if tip_offset is None:
        # Use calibrated tip offset
        tip_offset = np.array([-1.4259097, 138.2282741, -83.86195491, 1]).reshape(4, 1)
    
    # Initialize dodecahedron parameters
    data = dodecapen.txt_data()
    params = dodecapen.parameters()
    
    # Open video
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f"Error: Could not open video file {video_path}")
        sys.exit(1)
    
    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"Video: {frame_count} frames at {fps} FPS")
    
    # Initialize data structure
    cv_data = {
        "metadata": {
            "start_time": time.time(),
            "video_path": str(video_path),
            "tip_offset_body": TIP_OFFSET_BODY.tolist(),
            "imu_to_tip_body": IMU_TO_TIP_BODY.tolist(),
            "fps": fps
        },
        "cv_readings": []
    }
    
    # Initialize One-Euro filters
    first_valid_pose = False
    filter_x, filter_y, filter_z = None, None, None
    filter_qw, filter_qx, filter_qy, filter_qz = None, None, None, None
    
    frame_idx = 0
    valid_frames = 0
    
    print("Processing video frames...")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Estimate pose using dodecahedron detection
        try:
            _, _, _, pose_DPR, visib_flag = dodecapen.find_pose(frame, params, data)
        except Exception as e:
            print(f"Frame {frame_idx}: Pose estimation failed - {e}")
            frame_idx += 1
            continue
        
        if visib_flag == 1:
            # Convert pose to transformation matrix
            tf_cam_to_cent = dodecapen.RodriguesToTransf(pose_DPR)
            
            # Calculate IMU position in camera frame
            imu_pos_cam = tf_cam_to_cent[:3, 3]
            
            # Calculate tip position in camera frame
            tip_loc_cam = tf_cam_to_cent.dot(tip_offset)
            tip_pos_cam = tip_loc_cam[:3, 0]
            
            # Extract rotation matrix
            R_cam = tf_cam_to_cent[:3, :3]
            
            # Convert rotation matrix to quaternion
            from scipy.spatial.transform import Rotation as R
            quat = R.from_matrix(R_cam).as_quat()  # [x, y, z, w]
            quat = np.array([quat[3], quat[0], quat[1], quat[2]])  # [w, x, y, z]
            
            # Initialize filters with first valid pose
            if not first_valid_pose:
                t_start = frame_idx / fps
                filter_x = OneEuroFilter(t_start, imu_pos_cam[0])
                filter_y = OneEuroFilter(t_start, imu_pos_cam[1])
                filter_z = OneEuroFilter(t_start, imu_pos_cam[2])
                filter_qw = OneEuroFilter(t_start, quat[0])
                filter_qx = OneEuroFilter(t_start, quat[1])
                filter_qy = OneEuroFilter(t_start, quat[2])
                filter_qz = OneEuroFilter(t_start, quat[3])
                first_valid_pose = True
            
            # Apply One-Euro filter to position
            current_time = frame_idx / fps
            filtered_x = filter_x.filter_signal(current_time, imu_pos_cam[0])
            filtered_y = filter_y.filter_signal(current_time, imu_pos_cam[1])
            filtered_z = filter_z.filter_signal(current_time, imu_pos_cam[2])
            
            # Apply One-Euro filter to quaternion
            filtered_qw = filter_qw.filter_signal(current_time, quat[0])
            filtered_qx = filter_qx.filter_signal(current_time, quat[1])
            filtered_qy = filter_qy.filter_signal(current_time, quat[2])
            filtered_qz = filter_qz.filter_signal(current_time, quat[3])
            
            # Normalize quaternion
            filtered_quat = np.array([filtered_qw, filtered_qx, filtered_qy, filtered_qz])
            filtered_quat = filtered_quat / np.linalg.norm(filtered_quat)
            
            # Reconstruct rotation matrix from filtered quaternion
            filtered_R = R.from_quat([filtered_quat[1], filtered_quat[2], filtered_quat[3], filtered_quat[0]]).as_matrix()
            
            # Store filtered CV reading
            cv_entry = {
                "timestamp": current_time,
                "local_timestamp": current_time,
                "frame_idx": frame_idx,
                "imu_pos_cam": [filtered_x, filtered_y, filtered_z],
                "tip_pos_cam": [filtered_x, filtered_y, filtered_z],  # Simplified, should recalculate with filtered R
                "R_cam": filtered_R.tolist()
            }
            cv_data["cv_readings"].append(cv_entry)
            valid_frames += 1
            
            if valid_frames % 50 == 0:
                print(f"Processed {valid_frames} valid frames out of {frame_idx + 1} total frames")
        
        frame_idx += 1
    
    cap.release()
    
    # Save to JSON
    cv_data["metadata"]["end_time"] = time.time()
    cv_data["metadata"]["cv_count"] = len(cv_data["cv_readings"])
    cv_data["metadata"]["total_frames"] = frame_idx
    
    with open(output_json, "w") as f:
        json.dump(cv_data, f, indent=2)
    
    print(f"\nProcessed {valid_frames} valid frames out of {frame_idx} total frames")
    print(f"CV data saved to {output_json}")

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Process video to extract filtered CV pose estimates")
    parser.add_argument("video", help="Path to input video file")
    parser.add_argument("--output", default="outputs/cv_from_video.json", help="Output JSON file")
    args = parser.parse_args()

    # Ensure output directory exists
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)

    process_video_to_cv_data(args.video, args.output)

if __name__ == "__main__":
    main()
