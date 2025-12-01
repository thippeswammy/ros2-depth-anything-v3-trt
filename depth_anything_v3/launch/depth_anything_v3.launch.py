# Copyright 2025 Institute for Automotive Engineering (ika), RWTH Aachen University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    depth_anything_v3_dir = get_package_share_directory('depth_anything_v3')
    config_path = os.path.join(depth_anything_v3_dir, "config", "depth_anything_v3.param.yaml")

    zenoh_router_node = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "rmw_zenoh_cpp",
            "rmw_zenohd"
        ],
        output="screen",
    )

    rosbag_play_node = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            "/bags/2025-08-13_JBU_ika-hof-pedestrians/rosbag2_2025_08_13-17_19_57_fixed",
            "-l",
            "-r",
            "1",
        ],
        output="screen",
    )

    depth_anything_v3_node = Node(
        package='depth_anything_v3',
        executable='depth_anything_v3_main',
        name='depth_anything_v3',
        output='screen',
        remappings=[
            ('~/input/image', "/drivers/zed_camera/front_center/left/image_rect_color/compressed"),
            ('~/input/camera_info', "/drivers/zed_camera/front_center/left/camera_info"),
            ('~/output/depth_image', "/depth_anything_v3/output/depth_image"),
            ('~/output/point_cloud', "/depth_anything_v3/output/point_cloud")
        ],
        parameters=[config_path]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d' + os.path.join(get_package_share_directory('depth_anything_v3'), 'config', 'rviz.rviz')]
    )

    ld = LaunchDescription()
    ld.add_action(rosbag_play_node)
    ld.add_action(rviz_node)
    ld.add_action(depth_anything_v3_node)
    ld.add_action(zenoh_router_node)
    return ld
