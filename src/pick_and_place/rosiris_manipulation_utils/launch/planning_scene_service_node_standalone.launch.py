from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rosiris_manipulation_utils", 
            executable="planning_scene_node",    
            name="planning_scene_service_node",
            output="screen",
            parameters=[],                        
            remappings=[],                        
        )
    ])
