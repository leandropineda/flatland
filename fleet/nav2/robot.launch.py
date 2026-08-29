"""Navigation stack for ONE robot, namespaced.

    ros2 launch robot.launch.py namespace:=robot1

Runs slam_toolbox (the robot maps as it goes; own map frame robotN/map)
plus a minimal nav2: planner, controller, behaviors, bt_navigator and a
lifecycle manager. Deliberately not nav2_bringup: that remaps /tf into the
namespace, while flatland broadcasts every robot's TF on the global /tf.
Frames are disambiguated by name prefix (robotN/odom, ...) instead.
"""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV2_NODES = [
    ('nav2_controller', 'controller_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
]


def setup(context, *args, **kwargs):
    ns = LaunchConfiguration('namespace').perform(context)

    template = os.path.join(os.path.dirname(__file__), 'nav2_params.yaml')
    with open(template) as f:
        params = f.read().replace('${NS}', ns)
    params_file = os.path.join(tempfile.gettempdir(), f'{ns}_nav2_params.yaml')
    with open(params_file, 'w') as f:
        f.write(params)

    nodes = [
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=ns,
            output='screen',
            parameters=[params_file],
        )
    ]
    for package, executable in NAV2_NODES:
        nodes.append(Node(
            package=package,
            executable=executable,
            name=executable,
            namespace=ns,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[params_file],
        ))
    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            # no bond heartbeats: fewer idle wakeups per robot; respawn=True
            # on the nodes covers crashes instead
            'bond_timeout': 0.0,
            'node_names': [e for _, e in NAV2_NODES],
        }],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description='robot namespace, e.g. robot1'),
        OpaqueFunction(function=setup),
    ])
