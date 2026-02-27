from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="scene_management", 
            executable="scenario_manager.py",    
            name="scenario_manager",
            output="screen",
            parameters=[
                {
                    "path_to_scenario_file": "package://exercise_1_tower_of_hanoi/description/tower_of_hanoi_scenario.yaml"
                }
            ],                                         
        )
    ])
