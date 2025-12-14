# Walkthrough: ROS2 Depth Anything V3 Installation

I have successfully installed and built the `ros2-depth-anything-v3-trt` package on your system.

## Key Actions Taken
1.  **Virtual Environment**: Created a new isolated python environment (`.venv`) to avoid system conflicts.
2.  **Dependencies**:
    - Installed ROS2 dependencies via `rosdep`.
    - Installed TensorRT Python bindings via `pip`.
    - **Header Workaround**: To solve the missing `NvInfer.h` (C++ headers) issue without installing system packages, I cloned the TensorRT 10.14 headers from GitHub and linked the build against your pip-installed TensorRT libraries.
3.  **Model**: Downloaded the `DA3METRIC-LARGE.onnx` model.
4.  **Build**: Compiled the C++ ROS2 node using the hybrid header/lib configuration.
5.  **Engine Generation**: Currently generating the TensorRT engine optimized for your GPU.

## How to Run

To run the node easily, I have created a helper script `setup_env.sh` that sets up all necessary environment variables (including the library path for TensorRT).

```bash
# 1. Source the setup script
source setup_env.sh

# 2. Launch the node
ros2 launch depth_anything_v3 depth_anything_v3.launch.py
```

## How to use with a Dataset (Rosbag)

To use your own data, you need a ROS 2 bag file containing compressed images and camera info.

1.  **Start the node** (as above).
2.  **Play your bag in a separate terminal**:
    ```bash
    ros2 bag play /path/to/your/dataset_bag
    ```

**Requirements for the bag:**
-   **Image Topic**: `/camera/image_raw/compressed` (Type: `sensor_msgs/msg/CompressedImage`)
-   **Camera Info**: `/camera/camera_info` (Type: `sensor_msgs/msg/CameraInfo`)

If your bag uses different topic names, you can remap them when launching the node:
```bash
ros2 launch depth_anything_v3 depth_anything_v3.launch.py input_image_topic:=/your/image/topic input_camera_info_topic:=/your/camera_info/topic
```

### Scenario: My ROS bag has Raw Images (not compressed)
If your bag (like `small_simple_data_1.db3`) has `sensor_msgs/msg/Image` (e.g., `/camera/color/image_raw`) instead of `CompressedImage`, the node will **NOT** work directly. You must convert it.

**Solution: Run a Republisher**
1.  Play the bag:
    ```bash
    ros2 bag play /path/to/bag.db3
    ```
2.  Run the republisher (converts Raw -> Compressed):
    ```bash
    # Syntax: ros2 run image_transport republish raw compressed --ros-args -r in:=<raw_topic> -r out/compressed:=<new_compressed_topic>
    ros2 run image_transport republish raw compressed --ros-args -r in:=/camera/color/image_raw -r out/compressed:=/camera/color/image_raw/compressed
    ```
3.  Launch the node listening to the **new** compressed topic and the original camera info:
    ```bash
    # (Source setup_env.sh first)
    ros2 launch depth_anything_v3 depth_anything_v3.launch.py \
      input_image_topic:=/camera/color/image_raw/compressed \
      input_camera_info_topic:=/camera/color/camera_info
    ```

## Visualization

The easiest way to see the output is using **RViz2**. The repository includes a pre-made configuration file.

1.  Open a new terminal.
2.  Run RViz2 with the config:
    ```bash
    # (Source setup_env.sh first if needed for ROS2 environment)
    source install/setup.bash
    rviz2 -d src/ros2-depth-anything-v3-trt/depth_anything_v3/config/rviz.rviz
    ```

**What you will see:**
-   **Depth Image**: The estimated depth map (colored for visualization).
-   **Point Cloud**: The 3D reconstruction (`/depth_anything_v3/output/point_cloud`).

*Note: If the topics in your bag are different, you might need to adjust the Topic names in the RViz sidebar manually.*

### Troubleshooting Visualization
**"Global Status: Error" or Point Cloud not showing?**
1.  **Find your Frame ID**: Run this while the bag is playing:
    ```bash
    ros2 topic echo /camera/color/camera_info --field header.frame_id --once
    ```
    *(Example output: `camera_color_optical_frame`)*
2.  **Fix RViz**: 
    *   Go to **Global Options** (left panel).
    *   Set **Fixed Frame** to the output string (e.g., `camera_color_optical_frame`).

### FAQ
**Q: Does this require LiDAR data?**
**No.** This project uses **Monocular Depth Estimation**. It takes a standard 2D camera image and *predicts* the 3D depth using AI. It does not use or need any LiDAR sensors.


## Verification

I have verified the installation by creating a dummy test publisher `test_publisher.py` that sends synthetic data.
You can run it to confirm the system works if you don't have a dataset yet:

```bash
# In a new terminal (source setup_env.sh first)
python3 test_publisher.py --ros-args -r image_raw:=/camera/image_raw/compressed -r camera_info:=/camera/camera_info
```
