#!/usr/bin/env bash
set -e

BOX_ID="box"
FRAME="world"
EEF_LINK="ee_link"

echo "Adding box..."

ros2 service call \
  /planning_scene_manager/add_collision_objects \
  rosiris_manipulation_interfaces/srv/AddCollisionObjects \
  "collision_objects:
  - collision_object:
      id: '${BOX_ID}'
      header:
        frame_id: '${FRAME}'
      primitives:
        - type: 1
          dimensions: [0.2, 0.2, 0.2]
      primitive_poses:
        - position:
            x: 0.5
            y: 0.0
            z: 0.1
          orientation:
            x: 0.0
            y: 0.0
            z: 0.0
            w: 1.0
    collision_entries: []"

sleep 2

echo "Moving box..."

ros2 service call /planning_scene_manager/move_collision_objects rosiris_manipulation_interfaces/srv/MoveCollisionObjects \
"object_ids:
- '${BOX_ID}'
poses:
- header:
    frame_id: '${FRAME}'
  pose:
    position:
      x: 1.5
      y: 1.5
      z: 0.8
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0"

sleep 2

echo "Allowing collisions for box..."

ros2 service call \
  /planning_scene_manager/update_allowed_collisions \
  rosiris_manipulation_interfaces/srv/UpdateAllowedCollisions \
"object_ids: 
- '${BOX_ID}'
collision_entries:
- object_id: 'desk_1_link'
collision_allowed: true"

sleep 2

echo "Attaching box to end effector..."

ros2 service call \
  /planning_scene_manager/attach_collision_object \
  rosiris_manipulation_interfaces/srv/AttachCollisionObject \
"collision_object_id: '${BOX_ID}'
attach_to_link: '${EEF_LINK}'
collision_entries:
- object_id: '${BOX_ID}'
  collision_allowed: true"

sleep 2

echo "Detaching box from end effector..."

ros2 service call \
  /planning_scene_manager/detach_collision_object \
  rosiris_manipulation_interfaces/srv/DetachCollisionObject \
"collision_object_id: '${BOX_ID}'
detach_to_link: '${EEF_LINK}'
collision_entries: []"

sleep 2

echo "Removing box..."

ros2 service call \
  /planning_scene_manager/remove_collision_objects \
  rosiris_manipulation_interfaces/srv/RemoveCollisionObjects \
  "object_ids: ['${BOX_ID}']"

echo "Done."
