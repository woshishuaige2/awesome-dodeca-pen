#!/bin/bash
# Complete Workflow Script
# This script guides you through the complete recording and processing workflow

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default paths
OUTPUT_DIR="outputs/recording"
VIDEO_FILE="$OUTPUT_DIR/video.mp4"
IMU_FILE="$OUTPUT_DIR/imu_data.json"
CV_FILE="$OUTPUT_DIR/cv_data.json"
MERGED_FILE="outputs/my_data.json"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Complete Recording & Processing Workflow${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if we should skip recording
SKIP_RECORDING=false
if [ -f "$VIDEO_FILE" ] && [ -f "$IMU_FILE" ]; then
    echo -e "${YELLOW}Existing recording found:${NC}"
    echo "  Video: $VIDEO_FILE"
    echo "  IMU: $IMU_FILE"
    echo ""
    read -p "Do you want to use existing recording? (y/n): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        SKIP_RECORDING=true
        echo -e "${GREEN}Using existing recording${NC}"
    else
        echo -e "${YELLOW}Will create new recording${NC}"
        rm -rf "$OUTPUT_DIR"
    fi
fi

# Step 1: Record
if [ "$SKIP_RECORDING" = false ]; then
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  Step 1: Recording IMU + Video${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo "This will record IMU data and video simultaneously."
    echo "Make sure:"
    echo "  - BLE device is powered on and in range"
    echo "  - Camera is connected and working"
    echo "  - Markers are visible to camera"
    echo ""
    echo -e "${YELLOW}Press Ctrl+C when you want to stop recording${NC}"
    echo ""
    read -p "Press Enter to start recording..." -r
    echo ""
    
    python record_imu_and_video.py --output "$OUTPUT_DIR"
    
    if [ $? -ne 0 ]; then
        echo -e "${RED}Recording failed!${NC}"
        exit 1
    fi
    
    echo ""
    echo -e "${GREEN}✓ Recording complete${NC}"
fi

# Check if video file exists
if [ ! -f "$VIDEO_FILE" ]; then
    echo -e "${RED}Error: Video file not found: $VIDEO_FILE${NC}"
    exit 1
fi

if [ ! -f "$IMU_FILE" ]; then
    echo -e "${RED}Error: IMU file not found: $IMU_FILE${NC}"
    exit 1
fi

# Step 2: Process video
echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Step 2: Processing Video${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo "This will process the recorded video to extract CV data."
echo "This may take a few minutes depending on video length."
echo ""

if [ -f "$CV_FILE" ]; then
    read -p "CV data already exists. Reprocess? (y/n): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${GREEN}Using existing CV data${NC}"
    else
        python process_video_to_cv_data.py "$VIDEO_FILE" --output "$CV_FILE"
        
        if [ $? -ne 0 ]; then
            echo -e "${RED}Video processing failed!${NC}"
            exit 1
        fi
        
        echo ""
        echo -e "${GREEN}✓ Video processing complete${NC}"
    fi
else
    python process_video_to_cv_data.py "$VIDEO_FILE" --output "$CV_FILE"
    
    if [ $? -ne 0 ]; then
        echo -e "${RED}Video processing failed!${NC}"
        exit 1
    fi
    
    echo ""
    echo -e "${GREEN}✓ Video processing complete${NC}"
fi

# Step 3: Merge data
echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Step 3: Merging IMU and CV Data${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

python merge_imu_cv_data.py "$IMU_FILE" "$CV_FILE" --output "$MERGED_FILE"

if [ $? -ne 0 ]; then
    echo -e "${RED}Merging failed!${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}✓ Data merge complete${NC}"

# Step 4: Analyze
echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Step 4: Analyzing Results${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

if [ -f "compare_workflows.py" ]; then
    python compare_workflows.py "$MERGED_FILE"
    
    if [ $? -ne 0 ]; then
        echo -e "${RED}Analysis failed!${NC}"
        exit 1
    fi
    
    echo ""
    echo -e "${GREEN}✓ Analysis complete${NC}"
else
    echo -e "${YELLOW}compare_workflows.py not found, skipping analysis${NC}"
fi

# Summary
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Workflow Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Generated files:"
echo "  - $MERGED_FILE"
echo "  - outputs/comparison.png (if analysis ran)"
echo ""
echo "You can now:"
echo "  1. View the comparison plot: outputs/comparison.png"
echo "  2. Use my_data.json with other analysis tools"
echo "  3. Re-run analysis: python compare_workflows.py $MERGED_FILE"
echo ""
