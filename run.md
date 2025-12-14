# Running Instructions for ROS2 Depth Anything V3

**First-time setup?** See [Installation Instructions](install.md).

## Option 1: Quick Start (Standard Usage)
If you have a live camera or a bag file that already has **Compressed Images** (`sensor_msgs/msg/CompressedImage`), use the full automation script:

```bash
# Sourcing environment and running everything
./src/ros2-depth-anything-v3-trt/run_full.sh input_image_topic:=/camera/image_raw/compressed input_camera_info_topic:=/camera/camera_info
```
*(Replace topics with your actual topic names if they differ)*

---

## Option 2: Running with Your Big Dataset (Raw Images)

Since your dataset (`big_ros_files`) contains **Raw Images** (`/camera/color/image_raw`) and is very large (94GB), follow this **3-Terminal Workflow** for best performance.

### Terminal 1: Play Bag (Optimized)
Play ONLY the required topics to save bandwidth and system resources.
```bash
ros2 bag play big_ros_files --topics /camera/color/image_raw /camera/color/camera_info
```

### Terminal 2: Convert Raw Images to Compressed
The standard `image_transport` tool was crashing, so we use a custom Python script.
```bash
# Sourcing environment is important for dependencies
source src/ros2-depth-anything-v3-trt/setup_env.sh
python3 src/ros2-depth-anything-v3-trt/republish_images.py
```
*This will read `/camera/color/image_raw` and publish `/camera/color/image_raw/compressed`.*

### Terminal 3: Run the Full System
Launch the system using the automation script, pointing it to the **new compressed topic**.
```bash
./src/ros2-depth-anything-v3-trt/run_full.sh input_image_topic:=/camera/color/image_raw/compressed input_camera_info_topic:=/camera/color/camera_info
```

---

## Option 3: Automated Dataset Run (Best for `big_ros_files`)
We have created a single script that automates the **3-Terminal Workflow** above.

```bash
# This handles the Republisher, AI Node, and Bag Play in background
./src/ros2-depth-anything-v3-trt/run_dataset.sh
```
*Note: This script is configured for the dataset at `/media/thippe/project two world's HD/BUGGY_DATA`.*

---

## Troubleshooting

### Visualization Issues in RViz
If the **Point Cloud** does not appear, check the **Fixed Frame**:
1.  Run the checking command (while bag is playing):
    ```bash
    ros2 topic echo /camera/color/camera_info --field header.frame_id --once
    ```
2.  Set that value (e.g., `camera_color_optical_frame`) as the **Fixed Frame** in RViz Global Options.
