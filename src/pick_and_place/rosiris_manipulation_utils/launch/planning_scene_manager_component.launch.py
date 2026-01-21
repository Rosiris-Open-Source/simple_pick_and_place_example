from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name="scene_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                ComposableNode(
                    package="rosiris_manipulation_utils",
                    plugin="rosiris_manipulation_utils::PlanningSceneManager",
                    name="planning_scene_manager"
                )
            ],
            output="screen",
        )
    ])