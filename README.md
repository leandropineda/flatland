# Flatland (ROS 2 fork)

![build](https://github.com/leandropineda/flatland/actions/workflows/build.yml/badge.svg?branch=ros2-devel)

Fork of [avidbots/flatland](https://github.com/avidbots/flatland), a
performance-centric 2D robot simulator. This `ros2-devel` branch continues
upstream's `ros2-jazzy` line and builds on every supported (non-EOL) ROS 2
distro: **humble, jazzy, kilted, lyrical** (CI matrix).

What this branch adds over upstream `ros2-jazzy`:

* **Multi-robot worlds that work**: every plugin topic goes through the
  model namespace (`/robot1/scan`, `/robot1/cmd_vel`, ...) and TF frames are
  prefixed ROS2-style (`robot1/odom`, not `robot1_odom`). One world file, N
  robots, zero collisions.
* **Camera plugin**: raycasted 2.5D camera streaming `sensor_msgs/Image`
  plus JPEG `CompressedImage` and `CameraInfo`; renders only while
  subscribed. **Battery plugin**: motion-based drain, charging zones,
  `BatteryState`, dock command. **Modes plugin**: fake operating modes
  (idle/cleaning/...) as latched state. (Camera/battery imported from
  [OpenRobOps/sim-flatland](https://github.com/OpenRobOps/sim-flatland).)
* **Per-robot movement pause** (std_srvs/SetBool, DiffDrive `pause_service`
  param, default `pause_motion`): pins the robot while the nav stack keeps
  its goal — made for traffic-manager testing. World-level `/pause`,
  `/resume`, `/toggle_pause` still exist.
* **DiffDrive/TricycleDrive `stamped_cmd_vel` param**: `Twist` (default,
  nav2 ≤ jazzy) or `TwistStamped` (nav2 kilted+) — humble compatibility.
* `docker/Dockerfile` builds any distro:
  `docker build -f docker/Dockerfile --build-arg ROS_DISTRO=humble .`

## Simulated fleet

[`fleet/`](fleet/) runs a complete multi-robot setup with docker compose:
one sim container + one nav2 container per robot (AMCL on the world's own
map by default, SLAM optional), selectable worlds, one shared DDS domain,
pause/resume and camera streaming out of the box. See
[fleet/README.md](fleet/README.md).

## Native build

```bash
# in a colcon workspace src/
rosdep install --from-paths src --ignore-src -y
colcon build   # flatland_viz stays COLCON_IGNOREd (WIP upstream, issue #108)
```

## Upstream docs

* How to use: http://flatland-simulator.readthedocs.io
* Doxygen: http://flatland-simulator-api.readthedocs.io

## License

BSD 3-clause (see LICENSE for details), same as upstream.

Flatland includes open source libraries in its source tree:
- [ThreadPool](https://github.com/progschj/ThreadPool) Copyright (c) 2012 Jakob Progsch, Václav Zeman (zlib license)
- [Tweeny](https://github.com/mobius3/tweeny) Copyright (c) 2016 Leonardo Guilherme de Freitas (MIT license)
- [Box2d](https://github.com/erincatto/Box2D) Copyright (c) 2006-2017 Erin Catto [http://www.box2d.org](http://www.box2d.org) (zlib license)
