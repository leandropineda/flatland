"""RViz2 + the static TFs that stitch the fleet into one world view.

Flatland's debug markers (walls, robot bodies) and the battery/charging-zone
markers live in the frame "map". Each robot's SLAM tree hangs off its own
robotN/map. Flatland odometry is world-anchored and slam_toolbox starts with
map == odom, so identity transforms map -> robotN/map line every tree up with
the world (SLAM keeps them aligned within its correction of odom noise).

Robot namespaces are read from the mounted world file, so adding a robot to
sim/world.yaml automatically shows up here.
"""

import os

import yaml
from launch import LaunchDescription
from launch_ros.actions import Node

WORLD_YAML = os.environ.get('FLEET_WORLD', '/fleet/sim/world.yaml')


def generate_launch_description():
    with open(WORLD_YAML) as f:
        world = yaml.safe_load(f)
    namespaces = [m['namespace'] for m in world.get('models', []) if m.get('namespace')]

    nodes = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'world_to_{ns}_map',
            arguments=['--frame-id', 'map', '--child-frame-id', f'{ns}/map'],
            parameters=[{'use_sim_time': True}],
        )
        for ns in namespaces
    ]
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(os.path.dirname(__file__), 'fleet.rviz')],
        parameters=[{'use_sim_time': True}],
    ))
    return LaunchDescription(nodes)
