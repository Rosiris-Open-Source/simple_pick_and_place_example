architecture: 
https://moveit.picknik.ai/main/index.html
https://moveit.picknik.ai/main/doc/concepts/planning_scene_monitor.html

Important takeaway from https://moveit.picknik.ai/main/doc/concepts/planning_scene_monitor.html

Particularly, the move_group node as well as the rviz planning scene plugin maintain their own Planning Scene Monitor (PSM). The move_group node’s PSM listens to the topic /planning_scene and publishes its planning scene state to the topic monitored_planning_scene. The latter is listened to by the rviz planning scene plugin.

