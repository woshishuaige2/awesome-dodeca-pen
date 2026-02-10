# Hybrid Recording Workflow Guide

## Overview

This guide explains how to use the **hybrid recording workflow** to achieve clean pen-tip trajectories by combining:
- **Pre-recorded video** processed with One-Euro filtering for smooth CV pose estimates
- **Separately recorded IMU data** captured during the actual sketching session
- **Synchronization** to merge the two data streams into a single JSON file

This approach solves the CV jitter problem by applying the same filtering used in offline video analysis while maintaining synchronized IMU measurements.

---

## Why Use Hybrid Recording?

### The Problem
When recording CV and IMU data simultaneously in live mode (`record_raw_data.py`), the CV pose estimates are captured **without filtering**, resulting in high-frequency jitter. This jitter propagates through the EKF and produces noisy trajectories.

### The Solution
By processing a pre-recorded video separately, we can apply **One-Euro filtering** to the CV data (just like the offline video workflow), resulting in smooth pose estimates. We then synchronize this filtered CV data with separately recorded IMU data.

---

## Step-by-Step Workflow

### Step 1: Record Video and IMU Simultaneously

#### 1a. Start Video Recording
Use your camera to record a video of your sketching session. Save it as `my_sketch.mp4` (or any format).

**Tips:**
- Ensure good lighting and clear visibility of the dodecahedron markers
- Keep the camera stable
- Record at 30 FPS or higher

#### 1b. Start IMU Recording (in parallel)
In a separate terminal, run:

```bash
cd Code/IMU
python3 record_imu_only.py --output outputs/my_sketch_imu.json
```

**Important:** Start both recordings at approximately the same time. You can use:
- An **audio beep** (clap your hands) at the start
- A **visual cue** (wave the pen in front of the camera)
- Or simply note the time difference manually

Press `Ctrl+C` to stop the IMU recording when you finish sketching.

---

### Step 2: Process Video to Extract Filtered CV Data

Run the video processing script to extract CV pose estimates with One-Euro filtering:

```bash
cd Code/IMU
python3 process_video_to_cv.py ../Computer_vision/my_sketch.mp4 --output outputs/my_sketch_cv.json
```

This will:
- Detect the dodecahedron in each frame
- Apply One-Euro filtering to smooth the position and orientation
- Save the filtered CV data to JSON

**Expected output:**
```
Video: 450 frames at 30.0 FPS
Processing video frames...
Processed 50 valid frames out of 100 total frames
...
Processed 420 valid frames out of 450 total frames
CV data saved to outputs/my_sketch_cv.json
```

---

### Step 3: Synchronize and Merge the Data Streams

Now merge the IMU and CV data with proper time alignment:

#### Option A: Manual Synchronization
If you know the time offset between the two recordings (e.g., you started IMU recording 2 seconds before the video):

```bash
cd Code/IMU
python3 sync_and_merge.py outputs/my_sketch_imu.json outputs/my_sketch_cv.json \
    --output outputs/my_sketch_merged.json \
    --offset 2.0 \
    --method manual
```

#### Option B: Automatic Synchronization
Let the script automatically detect the first significant movement and align the streams:

```bash
cd Code/IMU
python3 sync_and_merge.py outputs/my_sketch_imu.json outputs/my_sketch_cv.json \
    --output outputs/my_sketch_merged.json \
    --method auto
```

**Expected output:**
```
Loading data files...
IMU readings: 509
CV readings: 420
Auto-sync: Detecting first significant movement...
First significant IMU movement at index 45
Calculated time offset: 1.523 seconds
Merged data saved to outputs/my_sketch_merged.json
```

---

### Step 4: Test with EKF Workflows

Now you can use the merged data with your existing comparison workflow:

```bash
cd Code/IMU
python3 compare_workflows.py
```

**Modify `compare_workflows.py`** to load your merged file:
```python
data_file = "outputs/my_sketch_merged.json"  # Change this line
```

You should now see **clean, rectangle-like trajectories** in the output plots, as the CV data is filtered and the IMU data is properly synchronized.

---

## File Structure Summary

| File | Purpose |
|------|---------|
| `record_imu_only.py` | Records only IMU data with timestamps |
| `process_video_to_cv.py` | Processes video to extract filtered CV pose estimates |
| `sync_and_merge.py` | Synchronizes and merges IMU + CV data streams |
| `compare_workflows.py` | Tests different EKF strategies on the merged data |
| `playback_raw_data.py` | Replays merged data through the EKF |

---

## Troubleshooting

### Problem: Auto-sync produces incorrect time offset
**Solution:** Use manual sync with `--offset` and estimate the time difference between when you started the video and IMU recordings.

### Problem: Video processing finds very few valid frames
**Solution:** 
- Check that the dodecahedron markers are clearly visible
- Ensure proper lighting
- Verify that the video path is correct

### Problem: Merged data shows misaligned IMU and CV
**Solution:**
- Visualize the time ranges printed by `sync_and_merge.py`
- Adjust the `--offset` parameter manually
- Use a clear synchronization cue (e.g., a sharp pen movement) at the start of both recordings

---

## Advantages of This Approach

1. **Clean CV Data:** One-Euro filtering removes high-frequency jitter from vision estimates
2. **High-Fidelity IMU:** IMU data is captured at full rate without any processing delays
3. **Repeatable Testing:** Once merged, you can test different EKF parameters without re-recording
4. **Flexible Sync:** Manual or automatic synchronization based on your needs

---

## Next Steps

Once you have a clean merged dataset:
1. Test the **Decoupled EKF** strategy to see if it further improves stability
2. Tune the `camera_noise_pos` parameter in `filter.py` for optimal smoothness
3. Compare the results with the **Standard EKF** to quantify the improvement

For live deployment, you can integrate the One-Euro filter directly into `record_raw_data.py` to apply filtering in real-time.
