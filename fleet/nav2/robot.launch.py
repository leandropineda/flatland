"""Navigation stack for ONE robot, namespaced, composed into one process.

    ros2 launch robot.launch.py namespace:=robot1 \
        [localization:=amcl|slam] [world_dir:=/fleet/sim/worlds/office]

Localization (default amcl): the same map image the sim builds its walls
from is served by a per-robot map_server and AMCL localizes on it — every
robot shares the one global `map` frame, and the robot's initial pose is
read from the world file, so it converges immediately. `slam` instead runs
slam_toolbox mapping from scratch (own `robotN/map` frame per robot).

Everything — localization, planner, controller, behaviors, bt_navigator,
lifecycle manager — loads as components into a single
an isolated component container using the EventsExecutor (one process per
robot; see fleet/container/ — waitset executors burned idle CPU waking on
the 50 Hz sim clock).

Deliberately not nav2_bringup: that remaps /tf into the namespace, while
flatland broadcasts every robot's TF on the global /tf. Frames are
disambiguated by name prefix (robotN/odom, ...) instead.
"""


import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

NAV2_COMPONENTS = [
    ('nav2_controller', 'nav2_controller::ControllerServer', 'controller_server'),
    ('nav2_planner', 'nav2_planner::PlannerServer', 'planner_server'),
    ('nav2_behaviors', 'behavior_server::BehaviorServer', 'behavior_server'),
    ('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator'),
]


def spawn_pose(world_dir, ns):
    """The robot's spawn pose from the world file (world coords == map frame)."""
    with open(os.path.join(world_dir, 'world.yaml')) as f:
        world = yaml.safe_load(f)
    for model in world.get('models', []):
        if model.get('namespace') == ns:
            x, y, yaw = model['pose']
            return float(x), float(y), float(yaw)
    raise RuntimeError(f'namespace {ns} not found in {world_dir}/world.yaml')


def setup(context, *args, **kwargs):
    ns = LaunchConfiguration('namespace').perform(context)
    localization = LaunchConfiguration('localization').perform(context)
    world_dir = LaunchConfiguration('world_dir').perform(context)
    if localization not in ('amcl', 'slam'):
        raise RuntimeError(f'localization must be amcl or slam, got {localization}')

    # With a known map all robots share the global `map` frame; with SLAM each
    # robot owns its own map frame.
    global_frame = 'map' if localization == 'amcl' else f'{ns}/map'

    template = os.path.join(os.path.dirname(__file__), 'nav2_params.yaml')
    with open(template) as f:
        text = f.read().replace('${NS}', ns).replace('${GLOBAL_FRAME}', global_frame)
    params = yaml.safe_load(text)
    if localization == 'amcl':
        x, y, yaw = spawn_pose(world_dir, ns)
        params['map_server']['ros__parameters']['yaml_filename'] = \
            os.path.join(world_dir, 'map.yaml')
        params['amcl']['ros__parameters']['initial_pose'] = \
            {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw}
    params_file = os.path.join(tempfile.gettempdir(), f'{ns}_nav2_params.yaml')
    with open(params_file, 'w') as f:
        # nest under the namespace so the sections match /<ns>/<node> nodes
        yaml.safe_dump({ns: params}, f)

    container_kind = LaunchConfiguration('container').perform(context)
    pkg, exe = {
        'events': ('fleet_container', 'events_container'),
        'isolated': ('rclcpp_components', 'component_container_isolated'),
    }[container_kind]
    container = Node(
        package=pkg,
        executable=exe,
        name='nav2_container',
        namespace=ns,
        output='screen',
        # no respawn: a respawned container would come back EMPTY (the
        # one-shot LoadComposableNodes never re-runs). Let the launch die so
        # compose restarts the whole thing, components included.
        respawn=False,
        # the file must ALSO be process-wide: the costmap sub-nodes that
        # controller/planner create internally read the process globals, not
        # the per-component LoadNode parameters (same trick as nav2_bringup)
        parameters=[params_file, {'use_sim_time': True}],
    )

    if localization == 'amcl':
        loc_components = [
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                namespace=ns,
                parameters=[params_file],
            ),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                namespace=ns,
                parameters=[params_file],
            ),
        ]
        loc_names = ['map_server', 'amcl']
    else:
        loc_components = [
            ComposableNode(
                package='slam_toolbox',
                plugin='slam_toolbox::AsynchronousSlamToolbox',
                name='slam_toolbox',
                namespace=ns,
                parameters=[params_file],
                # slam_toolbox hardcodes absolute /map; pull it into the
                # namespace so N robots don't fight over one topic
                remappings=[('/map', 'map'), ('/map_metadata', 'map_metadata')],
            )
        ]
        loc_names = ['slam_toolbox']

    components = list(loc_components)
    for package, plugin, name in NAV2_COMPONENTS:
        components.append(ComposableNode(
            package=package,
            plugin=plugin,
            name=name,
            namespace=ns,
            parameters=[params_file],
        ))
    components.append(ComposableNode(
        package='nav2_lifecycle_manager',
        plugin='nav2_lifecycle_manager::LifecycleManager',
        name='lifecycle_manager_navigation',
        namespace=ns,
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            # no bond heartbeats: fewer idle wakeups per robot; compose
            # restarts the container on crashes instead
            'bond_timeout': 0.0,
            # localization first, so the map->odom TF exists before the
            # costmaps come up
            'node_names': loc_names + [n for _, _, n in NAV2_COMPONENTS],
        }],
    ))

    load = LoadComposableNodes(
        target_container=f'/{ns}/nav2_container',
        composable_node_descriptions=components,
    )
    return [container, load]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description='robot namespace, e.g. robot1'),
        DeclareLaunchArgument('localization', default_value='amcl',
                              description='amcl (known map, shared frame) or slam'),
        DeclareLaunchArgument('world_dir', default_value='/fleet/sim/worlds/office',
                              description='world dir with world.yaml + map.yaml'),
        DeclareLaunchArgument('container', default_value='events',
                              description='events (EventsExecutor, lower idle '
                                          'CPU) or isolated (stock waitset)'),
        OpaqueFunction(function=setup),
    ])
