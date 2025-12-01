#!/bin/bash

# Convert MP4 video to GIF using ffmpeg
# ffmpeg -i dav3_long2.mp4 -ss 4 -frames 15 -vf "fps=2,scale=720:-1:flags=lanczos" -c:v gif dav3_short2.gif

ffmpeg -i ros2_depth_anything_v3_tensorrt.mp4 -ss 1 -vf "fps=24,scale=1000:-2" -pix_fmt yuv420p -c:v libx264 -crf 25 ros2_depth_anything_v3_tensorrt_short.mp4
