# Simulated fleet

One flatland world, N robots, one nav2+SLAM container per robot.

```
fleet-sim ──────────── flatland_server: physics, /clock, and per robot:
                       /robotN/{scan,odom,cmd_vel,image_raw,battery_state,mode,pause}
fleet-robot1 ───────── slam_toolbox + nav2 for robot1 (namespace /robot1)
fleet-robot2 ───────── same for robot2
fleet-robot3 ───────── same for robot3
```

All containers share one DDS domain (CycloneDDS over the compose network —
no host networking, no shared memory, no discovery config). Robots are
separated by ROS namespace and TF frame prefix (`robot1/odom`,
`robot1/base_link`, ...), the ROS2 multi-robot convention. Each robot runs
SLAM, so each gets its own map frame `robotN/map` and `/robotN/map` topic.
`/clock` and `/tf` are shared, published once by the sim.

## Run

```bash
cd fleet
docker compose up --build -d     # first build ~10 min
./smoke.sh                       # end-to-end checks incl. nav + pause/resume
```

CI also publishes ready-made images per distro (no clone needed):
`ghcr.io/leandropineda/flatland-sim`, `.../flatland-fleet-nav2` and
`.../flatland-fleet-rviz`, tagged `humble|jazzy|kilted|lyrical` (plus
`<distro>-<sha>` pins). They contain the engine and the nav2 launch/params;
worlds and models are mounted at runtime, so a downstream compose file plus
a `sim/` config dir is a complete deployment.

## Drive it

```bash
./ctl.sh goal robot1 3.0 0.5     # NavigateToPose (in robot1/map frame)
                                 # robots run SLAM: pick goals in/near space
                                 # the robot has already seen, or drive it
                                 # around a bit first
./ctl.sh pause robot1            # halt movement; the goal stays active
./ctl.sh resume robot1           # continue to the same goal
./ctl.sh mode robot2 cleaning    # fake operating mode (latched /robot2/mode)
./ctl.sh status                  # mode + battery per robot
```

Pause/resume is a service on the robot itself (`/robotN/pause`,
std_srvs/SetBool), implemented in the DiffDrive plugin: the body is pinned
and cmd_vel ignored while nav2 keeps the mission — built for exercising an
external traffic manager. The whole world can also be paused with the
flatland `/pause` + `/resume` services.

## Watch it (RViz2)

```bash
xhost +local:docker                    # allow the container to use your X display
docker compose --profile viz up -d --build rviz
```

One RViz window shows the whole world: walls and every robot body as
flatland ground-truth markers, per-robot laser scans and nav plans in
distinct colors, robot1's SLAM map overlay, its camera stream, battery text
and charging zones. Fixed frame is `map` — the rviz launch publishes static
`map -> robotN/map` transforms so all the per-robot SLAM trees line up in
one world view (flatland odometry is world-anchored, so they agree).

The markers come from flatland's `show_viz` (on by default here, 10 Hz;
`SHOW_VIZ=false docker compose up -d sim` turns them off). flatland_viz,
the old Qt app, remains disabled upstream — RViz2 covers it.

The camera renders only while something subscribes to
`/robotN/image_raw` (or `/compressed`), so idle cost is zero:

```bash
docker exec fleet-sim /entrypoint.sh ros2 topic hz /robot1/image_raw
```

## Add a robot

1. `sim/world.yaml`: copy a `models:` entry — unique `name`/`namespace`, free pose.
2. `docker-compose.yml`: copy a `robotN:` service, bump the name and namespace.

## Change distro

```bash
ROS_DISTRO=humble docker compose up --build -d
```

Default is jazzy (LTS). On kilted/lyrical nav2 defaults to TwistStamped
cmd_vel; the params already pin `enable_stamped_cmd_vel: false`, and the
DiffDrive plugin can take `stamped_cmd_vel: true` instead if you want
stamped end to end.

## Kubernetes later

The layout maps 1:1 — sim Deployment + one nav2 Deployment per robot (or a
templated Helm chart). The only change is DDS discovery: pod networks do
not route multicast, so add a Fast DDS discovery server or a
`CYCLONEDDS_URI` unicast peer list pointing at the sim service; namespaces,
frames and params all stay the same. A fleet-management agent per robot is
one more container on the same domain — deployable as a compose overlay
that joins this stack's network, without touching this repo.
