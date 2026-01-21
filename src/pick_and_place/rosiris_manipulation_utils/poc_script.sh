#!/usr/bin/env bash
set -e

echo "Adding box"
ros2 service call /planning_scene_manager/add_box moveit_msgs/srv/ApplyPlanningScene "{}"
sleep 2

echo "Moving box"
ros2 service call /planning_scene_manager/move_box moveit_msgs/srv/ApplyPlanningScene "{}"
sleep 2

echo "Allowing collision with ee_link"
ros2 service call /planning_scene_manager/allow_collision moveit_msgs/srv/ApplyPlanningScene "{}"
sleep 2

echo "Removing box"
ros2 service call /planning_scene_manager/remove_box moveit_msgs/srv/ApplyPlanningScene "{}"

echo "Done"
