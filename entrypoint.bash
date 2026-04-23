#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/dtc/ws/install/setup.bash

if [ "$RMW_IMPLEMENTATION" = "rmw_zenoh_cpp" ]; then
    echo "[GLIDER] Starting Zenoh router..."
    ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenoh_router.log 2>&1 &
    sleep 2
fi

if [ "$RUN" = "true" ]; then
    echo "[GLIDER] Starting foxglove_bridge..."
    nohup ros2 run foxglove_bridge foxglove_bridge --ros-args -p address:='0.0.0.0' -p port:=8765 > /dev/null 2>&1 &
    sleep 3
    echo "[GLIDER] Launching glider..."
    ros2 launch glider glider-node.launch.py
else
    echo "[GLIDER] RUN=false, keeping container alive..."
fi

exec "$@"