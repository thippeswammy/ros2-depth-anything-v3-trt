# Installation Instructions for ROS2 Depth Anything V3

This guide details how to set up the environment and build the package on your local system (Ubuntu 22.04 + ROS2 Humble).

## Prerequisites
- **OS**: Ubuntu 22.04 LTS
- **ROS2 Distribution**: Humble Hawksbill
- **CUDA**: 12.x (Tested with 12.6)
- **TensorRT**: 8.6 or newer (Ensure you have the tarball or installed version available)

## 1. Clone the Repository
```bash
cd ~/ros2_ws/src
git clone <repository_url> ros2-depth-anything-v3-trt
cd ros2-depth-anything-v3-trt
```

## 2. Environment Setup (Virtual Environment)
We use a local Python virtual environment to manage dependencies without interfering with the system Python.

### Create and Activate venv
```bash
# Create venv if it doesn't exist
python3 -m venv .venv

# Activate it
source .venv/bin/activate
```

### Install Python Dependencies
```bash
pip install --upgrade pip
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118  # Adjust CUDA version as needed
pip install opencv-python onnx onnxruntime-gpu
# Install TensorRT python bindings matching your TRT version if not using the tarball method extensively
# pip install tensorrt
```
*Note: If you are using a TensorRT tarball, ensure you extract it and set `LD_LIBRARY_PATH` correctly (handled by `setup_env.sh` below).*

## 3. Build the ROS2 Package
```bash
# Sourcing ROS2
source /opt/ros/humble/setup.bash

# Install rosdep dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build with colcon
colcon build --symlink-install --packages-select depth_anything_v3
```

## 4. Setup Script
A helper script `setup_env.sh` is provided in `src/ros2-depth-anything-v3-trt/` to automatically setup the environment for running.

```bash
# This script activates the venv and sets LD_LIBRARY_PATH for TensorRT
source src/ros2-depth-anything-v3-trt/setup_env.sh
```

## 5. Model Preparation
Download the ONNX models and place them in `src/ros2-depth-anything-v3-trt/depth_anything_v3/models`. Then generate the TensorRT engines:

```bash
./src/ros2-depth-anything-v3-trt/generate_engines.sh
```
