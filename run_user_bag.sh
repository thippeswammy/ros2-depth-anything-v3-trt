#!/bin/bash

# Configuration
BAG_PATH="/media/thippe/project two world's HD/dataset/BUGGY/buggy_ros2/Buggy1_sb_circles/"
PLAY_ARGS="--clock -r 0.2"

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source the setup environment
source "$SCRIPT_DIR/setup_env.sh"

# Trap Ctrl+C to kill all background processes
cleanup() {
    echo ""
    echo "Stopping all processes..."
    kill 0
}
trap cleanup SIGINT EXIT

echo "---------------------------------------------------"
echo "Running User Bag with Depth Anything V3"
echo "Bag: $BAG_PATH"
echo "---------------------------------------------------"

# 1. Start the Depth Anything Node
# We remap the input topics to match what is in your bag file:
#   input_image_topic       -> /camera/color/image_raw
#   input_camera_info_topic -> /camera/color/camera_info
echo "[1/2] Starting Depth Anything Node..."
ros2 launch depth_anything_v3 depth_anything_v3.launch.py \
    input_image_topic:=/camera/color/image_raw \
    input_camera_info_topic:=/camera/color/camera_info \
    output_depth_topic:=/depth_anything_v3/depth \
    output_point_cloud_topic:=/depth_anything_v3/points &

PID_NODE=$!
echo "Node started with PID $PID_NODE. Waiting 5 seconds for initialization..."
sleep 5

# 2. Play the Bag
echo "[2/2] Playing Rosbag..."
if [ -d "$BAG_PATH" ] || [ -f "$BAG_PATH" ]; then
    ros2 bag play "$BAG_PATH" $PLAY_ARGS
    
    echo "Bag playback finished. Press Ctrl+C to exit."
    wait
else
    echo "Error: Bag path not found: $BAG_PATH"
fi
