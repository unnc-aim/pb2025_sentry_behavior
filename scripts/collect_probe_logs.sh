#!/usr/bin/env bash
# collect_probe_logs.sh — Collect multi-package runtime logs after a probe session.
#
# Usage:
#   ./collect_probe_logs.sh [output_dir]
#
# If output_dir is not given, it uses the latest ~/sentry_probe_logs/<timestamp> directory.

set -euo pipefail

if [ -n "${1:-}" ]; then
    OUT_DIR="$1"
else
    OUT_DIR="$(ls -td ~/sentry_probe_logs/*/ 2>/dev/null | head -1)"
    if [ -z "$OUT_DIR" ]; then
        echo "No probe log directory found. Run the probe launch first."
        exit 1
    fi
fi

echo "=== Collecting logs into: $OUT_DIR ==="

# 1. Copy latest .ros/log files (all nodes from this boot session)
ROS_LOG_LATEST="$HOME/.ros/log"
if [ -d "$ROS_LOG_LATEST" ]; then
    mkdir -p "$OUT_DIR/ros_log"
    # Copy the most recent per-node log files
    find "$ROS_LOG_LATEST" -maxdepth 1 -name "*.log" -newer "$OUT_DIR" -exec cp {} "$OUT_DIR/ros_log/" \; 2>/dev/null || true
    # Copy latest launch log dirs
    for d in $(ls -td "$ROS_LOG_LATEST"/2*/ 2>/dev/null | head -3); do
        cp -r "$d" "$OUT_DIR/ros_log/" 2>/dev/null || true
    done
fi

# 2. Snapshot active ROS2 topics, nodes, actions, services
mkdir -p "$OUT_DIR/ros2_snapshot"
(source /opt/ros/humble/setup.bash && \
 ros2 node list  > "$OUT_DIR/ros2_snapshot/node_list.txt"   2>&1 || true; \
 ros2 topic list > "$OUT_DIR/ros2_snapshot/topic_list.txt"  2>&1 || true; \
 ros2 action list -t > "$OUT_DIR/ros2_snapshot/action_list.txt" 2>&1 || true; \
 ros2 lifecycle get /bt_navigator > "$OUT_DIR/ros2_snapshot/bt_navigator_lifecycle.txt" 2>&1 || true; \
 ros2 action info /navigate_to_pose > "$OUT_DIR/ros2_snapshot/nav_to_pose_info.txt" 2>&1 || true; \
 ros2 action info /navigate_through_poses > "$OUT_DIR/ros2_snapshot/nav_through_poses_info.txt" 2>&1 || true)

# 3. Snapshot listening ports
ss -tlnp > "$OUT_DIR/ros2_snapshot/listening_ports.txt" 2>&1 || true

# 4. Snapshot running ROS-related processes
ps aux | grep -E 'ros2|nav2|pb2025|sentry|bt_navigator|controller_server|planner_server' | grep -v grep \
    > "$OUT_DIR/ros2_snapshot/processes.txt" 2>&1 || true

echo "=== Done. Review logs at: $OUT_DIR ==="
ls -lh "$OUT_DIR"
