from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rosiris_manipulation_utils", 
            executable="scenario_manager.py",    
            name="scenario_manager",
            output="screen",
            parameters=[],                        
            remappings=[],                        
        )
    ])
