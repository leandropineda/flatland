#!/bin/bash
# End-to-end smoke test for the fleet. Run after `docker compose up -d`.
# Every check prints PASS/FAIL; exits non-zero if anything failed.
# Checks are made from inside the sim container (same DDS domain).
set -uo pipefail

E=(docker exec fleet-sim /entrypoint.sh)
FAIL=0
ROBOTS=$(sed -n 's/.*namespace: *//p' "$(dirname "$0")/sim/world.yaml")

check() { # check <label> <cmd...>
  local label=$1; shift
  if "$@" >/dev/null 2>&1; then echo "PASS  $label"; else echo "FAIL  $label"; FAIL=1; fi
}

hz_at_least() { # hz_at_least <topic> <min_hz> <window_s>
  local topic=$1 min=$2 win=${3:-6}
  "${E[@]}" bash -c "timeout $win ros2 topic hz $topic 2>/dev/null | grep -m1 'average rate' \
    | awk '{exit (\$3 >= $min * 0.5) ? 0 : 1}'" # at least half the nominal rate
}

echo "== containers"
for c in $(docker compose -f "$(dirname "$0")/docker-compose.yml" ps --format '{{.Name}}'); do
  echo "  up: $c"
done

echo "== clock"
check "/clock publishing" hz_at_least /clock 50 3

for ns in $ROBOTS; do
  echo "== $ns"
  check "$ns bt_navigator active" bash -c \
    "${E[*]} bash -c 'timeout 10 ros2 lifecycle get /$ns/bt_navigator' | grep -q active"
  check "$ns scan ~5 Hz" hz_at_least "/$ns/scan" 5 4
  check "$ns odom ~20 Hz" hz_at_least "/$ns/odom" 20 3
  check "$ns TF map->base_link" bash -c \
    "${E[*]} bash -c 'timeout 15 ros2 run tf2_ros tf2_echo $ns/map $ns/base_link 2>&1 | grep -m1 -q Translation'"
  check "$ns camera streams on demand" bash -c \
    "${E[*]} bash -c 'timeout 10 ros2 topic echo --once /$ns/image_raw --field width' | grep -q 320"
  check "$ns battery_state" bash -c \
    "${E[*]} bash -c 'timeout 5 ros2 topic echo --once /$ns/battery_state --field percentage' | grep -q '[0-9]'"
  check "$ns mode latched" bash -c \
    "${E[*]} bash -c 'timeout 5 ros2 topic echo --once /$ns/mode --field data' | grep -q ."
done

first=$(echo "$ROBOTS" | head -1)
echo "== navigation + pause/resume ($first)"

# short goal ~1m ahead of spawn; world.yaml spawns robot1 at (2,1) facing +x
check "$first accepts and reaches a goal" bash -c \
  "${E[*]} bash -c 'timeout 120 ros2 action send_goal /$first/navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: $first/map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}\"' | grep -q 'SUCCEEDED'"

# pause mid-goal: send a longer goal in background, pause, confirm zero motion,
# resume, confirm the same goal still completes.
"${E[@]}" bash -c "nohup timeout 180 ros2 action send_goal /$first/navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: $first/map}, pose: {position: {x: 3.5, y: 0.5}, orientation: {w: 1.0}}}}' > /tmp/goal.log 2>&1 &"
sleep 4
check "$first pause acknowledged" bash -c \
  "${E[*]} ros2 service call /$first/pause std_srvs/srv/SetBool '{data: true}' | grep -q 'success=True'"
sleep 2
check "$first stands still while paused" bash -c \
  "${E[*]} bash -c 'timeout 5 ros2 topic echo --once /$first/ground_truth --field twist.twist.linear.x' | grep -qE '^-?0\.0'"
check "$first resume acknowledged" bash -c \
  "${E[*]} ros2 service call /$first/pause std_srvs/srv/SetBool '{data: false}' | grep -q 'success=True'"
check "$first completes the paused goal" bash -c \
  "${E[*]} bash -c 'for i in \$(seq 90); do grep -q SUCCEEDED /tmp/goal.log && exit 0; sleep 2; done; cat /tmp/goal.log; exit 1'"

echo
[ "$FAIL" = 0 ] && echo "SMOKE: ALL PASS" || echo "SMOKE: FAILURES ABOVE"
exit $FAIL
