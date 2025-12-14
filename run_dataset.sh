#!/bin/bash

# Configuration
# Path from your terminal snippet
BAG_DIR="/media/thippe/project two world's HD/BUGGY_DATA"
BAG_NAME="big_ros_files"

# Locate script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source Environment
source "$SCRIPT_DIR/setup_env.sh"

# Trap Ctrl+C to kill all background processes
cleanup() {
    echo ""
    echo "Stopping all processes..."
    kill 0
}
trap cleanup SIGINT EXIT

echo "---------------------------------------------------"
echo "Automated Dataset Runner (Depth Anything V3)"
echo "Bag Location: $BAG_DIR/$BAG_NAME"
echo "---------------------------------------------------"

# 1. Start Republisher (Python Script)
echo "[1/3] Starting Image Republisher..."
python3 "$SCRIPT_DIR/republish_images.py" &
PID_REP=$!
sleep 2

# 2. Start Main System (Node + RViz)
echo "[2/3] Starting AI Node & RViz..."
"$SCRIPT_DIR/run_full.sh" \
    input_image_topic:=/camera/color/image_raw/compressed \
    input_camera_info_topic:=/camera/color/camera_info &
PID_MAIN=$!
sleep 5

# 3. Play Bag (Foreground)
echo "[3/3] Playing Rosbag..."
if [ -d "$BAG_DIR" ]; then
    cd "$BAG_DIR"
    # Play specific topics only
    ros2 bag play "$BAG_NAME" --topics /camera/color/image_raw /camera/color/camera_info
    
    echo "Bag playback finished. Press Ctrl+C to exit."
    wait # Wait for background processes
else
    echo "Error: Bag directory not found: $BAG_DIR"
    echo "Please check the BAG_DIR path in this script: $BAG_DIR"
    echo "You may need to update it if the drive name is different."
fi
