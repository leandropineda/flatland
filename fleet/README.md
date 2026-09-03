# Simulated fleet

One flatland world, N robots, one nav2 container per robot.

```
fleet-sim          physics, /clock, per robot: /robotN/{scan,odom,cmd_vel,
                   image_raw,battery_state,mode,pause}
fleet-robot1..3    map_server+AMCL (default) or slam_toolbox, + nav2
fleet-rviz         optional world view (profile "viz")
```

One DDS domain (CycloneDDS over the compose network); robots are separated
by ROS namespace and TF frame prefix (`robot1/odom`, ...). `/clock` and
`/tf` are shared.

Two env vars select the scenario — use the same values for `up`, `ctl.sh`
and `smoke.sh`:

- `WORLD=office|warehouse` (default office): picks `sim/worlds/<WORLD>/`,
  which holds the map (`map.png|yaml`) and the robots (`world.yaml`).
- `LOCALIZATION=amcl|slam` (default amcl): amcl serves the world's own map
  per robot and localizes on it — shared global `map` frame, initial pose
  from the world file, converges immediately. slam maps from scratch — each
  robot owns its `robotN/map` frame; goals then use that frame, and rviz
  stitches the frames with static transforms.

## Run

```bash
cd fleet
docker compose up --build -d          # WORLD=warehouse ... for the other world
./smoke.sh                            # end-to-end checks incl. nav + pause/resume
./ctl.sh goal robot1 3.0 0.5          # NavigateToPose | pause | resume | mode | status
```

Pause/resume is the robot's own `/robotN/pause` service (DiffDrive plugin):
the body halts, the nav goal survives, resume continues — made for
traffic-manager testing. The whole world pauses via flatland's `/pause`.

The camera renders only while subscribed (zero idle cost):
`docker exec fleet-sim /entrypoint.sh ros2 topic hz /robot1/image_raw`

## Watch it (RViz2)

```bash
xhost +local:docker
docker compose --profile viz up -d rviz
```

Walls and robot bodies (flatland ground truth), the map, colored per-robot
scans and plans, robot1's camera and battery. Fixed frame `map`.

## Add a world / robot

- World: new `sim/worlds/<name>/` with `map.png`, `map.yaml`, `world.yaml`
  (copy an existing one), then `WORLD=<name>`.
- Robot: a `models:` entry in the world file (unique name/namespace, free
  pose) + one `robotN:` compose service.

CI publishes these as images per distro (`flatland-sim`,
`flatland-fleet-nav2`, `flatland-fleet-rviz`; tags `humble|jazzy|kilted|
lyrical`, `<distro>-<sha>` pins) — a downstream compose file + a `sim/`
dir runs the fleet without cloning. Change distro: `ROS_DISTRO=humble
docker compose up --build -d`.

## Kubernetes later

Maps 1:1 — sim Deployment + one nav2 Deployment per robot. Only DDS
discovery changes (pod networks drop multicast): Fast DDS discovery server
or a `CYCLONEDDS_URI` unicast peer list. A fleet-management agent per robot
is one more container, deployable as a compose overlay on this stack's
network without touching this repo.
