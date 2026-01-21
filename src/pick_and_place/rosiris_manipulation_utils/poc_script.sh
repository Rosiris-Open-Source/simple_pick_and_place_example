#!/usr/bin/env bash
set -e

echo "Adding box"
ros2 service call /scene/add_box moveit_msgs/srv/ApplyPlanningScene "{}"
sleep 2

echo "Moving box"
ros2 service call /scene/move_box moveit_msgs/srv/ApplyPlanningScene "{}"
sleep 2

echo "Allowing collision with ee_link"
ros2 service call /scene/allow_collision moveit_msgs/srv/ApplyPlanningScene "{}"
sleep 2

echo "Removing box"
ros2 service call /scene/remove_box moveit_msgs/srv/ApplyPlanningScene "{}"

echo "Done"
