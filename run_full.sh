#!/bin/bash

# Get the project root directory (where this script lives)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$( cd "$SCRIPT_DIR/../.." && pwd )"

# Ensure we are executing from the workspace root
cd "$WORKSPACE_DIR"

# 1. Setup Environment
echo "[Step 1/3] Setting up environment..."
source "$SCRIPT_DIR/setup_env.sh"

# Trap Ctrl+C (SIGINT) to ensure we kill the background node when exiting
trap "kill 0" SIGINT

echo "---------------------------------------------------"
echo "Starting ROS2 Depth Anything V3 (Full Run)"
echo "---------------------------------------------------"
echo "Usage: ./run_full.sh [ROS_ARGS...]"
echo "Example: ./run_full.sh input_image_topic:=/my/camera/image"
echo "---------------------------------------------------"

# 2. Launch the Node in the background
echo "[Step 2/3] Launching Depth Anything Node..."
# We pass "$@" to allow valid ros arguments like remappings to be passed through
ros2 launch depth_anything_v3 depth_anything_v3.launch.py "$@" &
NODE_PID=$!

echo "Node started with PID $NODE_PID. Waiting 5 seconds for initialization..."
sleep 5

# 3. Launch RViz
echo "[Step 3/3] Launching RViz..."
if [ -f "$SCRIPT_DIR/ros2-depth-anything-v3-trt.rviz" ]; then
    rviz2 -d "$SCRIPT_DIR/ros2-depth-anything-v3-trt.rviz"
else
    rviz2 -d "$SCRIPT_DIR/depth_anything_v3/config/rviz.rviz"
fi

# Cleanup
echo "RViz closed. Stopping node..."
kill $NODE_PID
wait
echo "Exited."
