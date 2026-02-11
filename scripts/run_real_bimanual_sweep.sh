#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   scripts/run_real_bimanual_sweep.sh [cycles] [duration_sec] [scale] [namespace]
# Example:
#   scripts/run_real_bimanual_sweep.sh 6 2 1.0 real

CYCLES="${1:-6}"
DURATION_SEC="${2:-2}"
SCALE="${3:-1.0}"
NAMESPACE="${4:-real}"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "$HOME/ros2_ws/install/setup.bash"
  set -u
fi

PREFIX=""
if [[ -n "$NAMESPACE" ]]; then
  PREFIX="/$NAMESPACE"
fi

RIGHT_ACTION="${PREFIX}/right_joint_trajectory_controller/follow_joint_trajectory"
LEFT_ACTION="${PREFIX}/left_joint_trajectory_controller/follow_joint_trajectory"

if ! ros2 action list | grep -q "^${RIGHT_ACTION}$"; then
  echo "Missing action: ${RIGHT_ACTION}" >&2
  echo "Check launch/controller status first." >&2
  exit 1
fi
if ! ros2 action list | grep -q "^${LEFT_ACTION}$"; then
  echo "Missing action: ${LEFT_ACTION}" >&2
  echo "Check launch/controller status first." >&2
  exit 1
fi

scale_positions() {
  local csv="$1"
  awk -v s="$SCALE" -F',' '{
    for (i = 1; i <= NF; i++) {
      v = $i * s
      printf(i == 1 ? "%.4f" : ", %.4f", v)
    }
  }' <<< "$csv"
}

send_goal() {
  local side="$1"         # right|left
  local positions_csv="$2" # already formatted for YAML list
  local action="${PREFIX}/${side}_joint_trajectory_controller/follow_joint_trajectory"
  local joints="openarm_${side}_joint1, openarm_${side}_joint2, openarm_${side}_joint3, openarm_${side}_joint4, openarm_${side}_joint5, openarm_${side}_joint6, openarm_${side}_joint7"

  ros2 action send_goal "$action" control_msgs/action/FollowJointTrajectory \
    "{trajectory: {joint_names: [$joints], points: [{positions: [$positions_csv], time_from_start: {sec: ${DURATION_SEC}, nanosec: 0}}]}}"
}

# Pose A/B (right), left side mirrors sign.
RIGHT_A_RAW="0.45,0.35,0.25,0.30,0.20,0.15,0.10"
RIGHT_B_RAW="-0.45,-0.25,-0.15,-0.20,-0.10,-0.08,-0.05"
LEFT_A_RAW="-0.45,-0.35,-0.25,-0.30,-0.20,-0.15,-0.10"
LEFT_B_RAW="0.45,0.25,0.15,0.20,0.10,0.08,0.05"

RIGHT_A="$(scale_positions "$RIGHT_A_RAW")"
RIGHT_B="$(scale_positions "$RIGHT_B_RAW")"
LEFT_A="$(scale_positions "$LEFT_A_RAW")"
LEFT_B="$(scale_positions "$LEFT_B_RAW")"

echo "Running bimanual sweep: cycles=${CYCLES}, duration=${DURATION_SEC}s, scale=${SCALE}, ns='${NAMESPACE}'"

for ((i = 1; i <= CYCLES; i++)); do
  echo "Cycle ${i}/${CYCLES}: pose A"
  send_goal right "$RIGHT_A" &
  pid_r=$!
  send_goal left "$LEFT_A" &
  pid_l=$!
  wait "$pid_r" "$pid_l"

  echo "Cycle ${i}/${CYCLES}: pose B"
  send_goal right "$RIGHT_B" &
  pid_r=$!
  send_goal left "$LEFT_B" &
  pid_l=$!
  wait "$pid_r" "$pid_l"
done

echo "Done."
