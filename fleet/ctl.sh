#!/bin/bash
# Fleet control helpers. All commands exec into the sim container, which sees
# the whole graph.
#
#   ./ctl.sh status                    list robots (mode, battery, pose topics)
#   ./ctl.sh pause robot1              halt one robot's movement (nav goal survives)
#   ./ctl.sh resume robot1             continue to the current goal
#   ./ctl.sh goal robot1 5.0 2.0      send a NavigateToPose goal
#   ./ctl.sh mode robot1 cleaning      switch the fake operating mode
#   ./ctl.sh topics                    all topics in the domain
set -euo pipefail

IN_SIM=(docker exec fleet-sim /entrypoint.sh)
cmd=${1:?usage: ctl.sh status|pause|resume|goal|mode|topics ...}

case "$cmd" in
  pause|resume)
    ns=${2:?robot namespace required}
    val=$([ "$cmd" = pause ] && echo true || echo false)
    "${IN_SIM[@]}" ros2 service call "/$ns/pause" std_srvs/srv/SetBool "{data: $val}"
    ;;
  goal)
    ns=${2:?robot namespace required}; x=${3:?x}; y=${4:?y}
    "${IN_SIM[@]}" ros2 action send_goal "/$ns/navigate_to_pose" nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: $ns/map}, pose: {position: {x: $x, y: $y}, orientation: {w: 1.0}}}}"
    ;;
  mode)
    ns=${2:?robot namespace required}; mode=${3:?mode name}
    "${IN_SIM[@]}" ros2 topic pub --once "/$ns/mode_cmd" std_msgs/msg/String "{data: $mode}"
    ;;
  status)
    robots=$(sed -n 's/.*namespace: *//p' "$(dirname "$0")/sim/world.yaml")
    for ns in $robots; do
      echo "== $ns"
      "${IN_SIM[@]}" bash -c "timeout 3 ros2 topic echo --once /$ns/mode --field data 2>/dev/null | head -1 | sed 's/^/  mode: /' || echo '  mode: (no answer)'"
      "${IN_SIM[@]}" bash -c "timeout 3 ros2 topic echo --once /$ns/battery_state --field percentage 2>/dev/null | head -1 | sed 's/^/  battery: /' || echo '  battery: (no answer)'"
    done
    ;;
  topics)
    "${IN_SIM[@]}" ros2 topic list
    ;;
  *)
    echo "unknown command: $cmd" >&2; exit 1
    ;;
esac
