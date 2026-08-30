"""Navigation stack for ONE robot, namespaced, composed into one process.

    ros2 launch robot.launch.py namespace:=robot1

slam_toolbox (the robot maps as it goes; own map frame robotN/map) plus a
minimal nav2 — planner, controller, behaviors, bt_navigator — and the
lifecycle manager, all loaded as components into a single
component_container_isolated (each keeps its own executor thread). One
process instead of six: the duplicated DDS receive/clock machinery per
process was a third of each robot's idle CPU.

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


def setup(context, *args, **kwargs):
    ns = LaunchConfiguration('namespace').perform(context)

    template = os.path.join(os.path.dirname(__file__), 'nav2_params.yaml')
    with open(template) as f:
        params = yaml.safe_load(f.read().replace('${NS}', ns))
    params_file = os.path.join(tempfile.gettempdir(), f'{ns}_nav2_params.yaml')
    with open(params_file, 'w') as f:
        # nest under the namespace so the sections match /<ns>/<node> nodes
        yaml.safe_dump({ns: params}, f)

    container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        namespace=ns,
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        # the file must ALSO be process-wide: the costmap sub-nodes that
        # controller/planner create internally read the process globals, not
        # the per-component LoadNode parameters (same trick as nav2_bringup)
        parameters=[params_file, {'use_sim_time': True}],
    )

    components = [
        ComposableNode(
            package='slam_toolbox',
            plugin='slam_toolbox::AsynchronousSlamToolbox',
            name='slam_toolbox',
            namespace=ns,
            parameters=[params_file],
            # slam_toolbox hardcodes absolute /map; pull it into the namespace
            # so N robots don't fight over one topic
            remappings=[('/map', 'map'), ('/map_metadata', 'map_metadata')],
        )
    ]
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
            # no bond heartbeats: fewer idle wakeups per robot; the container
            # respawn covers crashes instead
            'bond_timeout': 0.0,
            # slam_toolbox is a lifecycle node too (jazzy+); activate it
            # first so map->odom TF exists before the costmaps come up
            'node_names': ['slam_toolbox'] + [n for _, _, n in NAV2_COMPONENTS],
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
        OpaqueFunction(function=setup),
    ])
