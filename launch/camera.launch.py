from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    package_path = FindPackageShare('unitree_camera_ros2')

    config_path = PathJoinSubstitution(
        [package_path, 'config', 'camera.yaml']
    )

    front_camera_node = Node(
        package="unitree_camera_ros2",
        executable="unitree_camera_node",
        name="front_camera_node",
        output="both",
        parameters=[config_path],
    )

    rear_camera_node = Node(
        package="unitree_camera_ros2",
        executable="unitree_camera_node",
        name="rear_camera_node",
        output="both",
        parameters=[config_path],
    )

    ld = LaunchDescription()

    ld.add_action(front_camera_node)
    ld.add_action(rear_camera_node)
    return ld
