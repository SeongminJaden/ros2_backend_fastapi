#!/bin/bash
MAP_ID=$1
if [ -z "$MAP_ID" ]; then
    echo "No mapid provided!"
    exit 1
fi

MAP_FILE="$HOME/map/${MAP_ID}.yaml"
echo "Starting Navigation with map file: $MAP_FILE"

# ROS 환경 설정 (본인 환경에 맞게 수정)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 네비게이션 실행
ros2 launch kepco_slam navigation2.launch.py map:=$MAP_FILE

