#!/bin/bash
# Source this file to setup the environment for ros2-depth-anything-v3-trt

# 0. Define Workspace Directory
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

# 1. Activate Virtual Environment
if [ -f "$WORKSPACE_DIR/.venv/bin/activate" ]; then
    source "$WORKSPACE_DIR/.venv/bin/activate"
else
    echo "Warning: .venv/bin/activate not found. Are you in the workspace root?"
fi

# 2. Add TensorRT libraries to LD_LIBRARY_PATH
# Get the absolute path to the workspace root (assuming script is in root)
 TRT_LIB_PATH="$WORKSPACE_DIR/.venv/lib/python3.10/site-packages/tensorrt_libs"

if [ -d "$TRT_LIB_PATH" ]; then
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$TRT_LIB_PATH
    echo "Added TensorRT libs to LD_LIBRARY_PATH: $TRT_LIB_PATH"
else
    echo "Error: TensorRT libs not found at $TRT_LIB_PATH"
fi

# 3. Source ROS2 Workspace
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
    echo "Sourced install/setup.bash"
else
    echo "Warning: install/setup.bash not found. Did you build the workspace?"
fi

echo "Environment setup complete. You can now run:"
echo "ros2 launch depth_anything_v3 depth_anything_v3.launch.py"
