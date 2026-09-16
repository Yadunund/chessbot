#!/usr/bin/env bash
# Stop a running chessbot stack started from this workspace (sim + nodes).
# Leaves the router running unless --all is given.
pkill -INT -f "ros2 launch chessbot_bringup" 2>/dev/null
sleep 5
pkill -KILL -f "component_container.*robot_io" 2>/dev/null
pkill -KILL -f "chessbot/.pixi/envs/default/.*(robot_state_publisher|spawner|create|brain_node|motion_node|calibration_node)" 2>/dev/null
if [[ "${1:-}" == "--all" ]]; then
  pkill -INT -f "zenohd -c config/zenoh/router.json5" 2>/dev/null
fi
exit 0
