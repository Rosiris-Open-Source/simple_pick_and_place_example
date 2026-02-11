from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rosiris_manipulation_utils", 
            executable="scenario_manager.py",    
            name="scenario_manager",
            output="screen",
            parameters=[
                {
                    "wait_for_services_timeout": 10,
                    "path_to_scenario": "package://exercise_1_tower_of_hanoi/description/scenario.yaml"
                }
            ],                                         
        )
    ])
