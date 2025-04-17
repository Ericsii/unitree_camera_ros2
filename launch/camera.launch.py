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

    container = Node(
        package="rclcpp_components",
        executable="component_container_mt",
        name="unitree_camera_container",
        output="both",
        parameters=[config_path],
    )

    load_composable_nodes = LoadComposableNodes(
        target_container="unitree_camera_container",
        composable_node_descriptions=[
            ComposableNode(
                package="unitree_camera_ros2",
                plugin="unitree_camera_ros2::UnitreeCameraNode",
                name="front_camera_node",
                parameters=[config_path],
            ),
            ComposableNode(
                package="unitree_camera_ros2",
                plugin="unitree_camera_ros2::UnitreeCameraNode",
                name="rear_camera_node",
                parameters=[config_path],
            )
        ],
    )

    ld = LaunchDescription()

    ld.add_action(container)
    ld.add_action(load_composable_nodes)
    return ld
