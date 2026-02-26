#!/usr/bin/env bash
set -e

BOX_ID="box_38473238"
FRAME="world"
EEF_LINK="end_effector_link"

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
          x: -1.5
          y: -1.5
          z: 0.1
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
    subframe_names:
      - 'grasp_point'
    subframe_poses:
      - position:
          x: -0.5
          y: -0.5
          z: -0.5
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
  self_collision_allowed: true
  allowed_touch_links: []"

sleep 2

echo "Moving ${BOX_ID}..."

ros2 service call /planning_scene_manager/move_collision_objects \
rosiris_manipulation_interfaces/srv/MoveCollisionObjects \
"collision_object_pose_updates:
- object_id: '${BOX_ID}'
  pose:
    header:
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
        w: 1.0
  collision_matrix_update: {}"


sleep 2

echo "Allowing collisions for box..."

ros2 service call \
  /planning_scene_manager/update_allowed_collisions \
  rosiris_manipulation_interfaces/srv/UpdateAllowedCollisions "
collision_matrix_updates:
- target_link: '${BOX_ID}'
  mode: 1   # MERGE
  collision_entries:
  - touch_link: desk_1_link
    collision_allowed: true"

sleep 2

echo "Attaching box to end effector..."

ros2 service call \
  /planning_scene_manager/attach_collision_object \
  rosiris_manipulation_interfaces/srv/AttachCollisionObject \
"collision_object_id: '${BOX_ID}'
attach_to_link: '${EEF_LINK}'
allowed_touch_links: []"

sleep 2

echo "Detaching box from end effector..."

ros2 service call \
  /planning_scene_manager/detach_collision_object \
  rosiris_manipulation_interfaces/srv/DetachCollisionObject \
"detach_from_link: '${EEF_LINK}'
collision_object_id: '${BOX_ID}'
disallowed_touch_links: []"

sleep 2

echo "Removing box..."

ros2 service call \
  /planning_scene_manager/remove_collision_objects \
  rosiris_manipulation_interfaces/srv/RemoveCollisionObjects \
  "object_ids: ['${BOX_ID}']"

sleep 2

ros2 service call \
  /planning_scene_manager/add_collision_objects \
  rosiris_manipulation_interfaces/srv/AddCollisionObjects \
"collision_objects:
- collision_object:
    id: '${BOX_ID}_2'
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
  self_collision_allowed: true
  allowed_touch_links: ['desk_1_link']"

sleep 2
echo "Moving ${BOX_ID}_2..."

ros2 service call /planning_scene_manager/move_collision_objects \
rosiris_manipulation_interfaces/srv/MoveCollisionObjects \
"collision_object_pose_updates:
- object_id: '${BOX_ID}_2'
  pose:
    header:
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
        w: 1.0
  collision_matrix_update: {}"


sleep 2

echo "Removing ${BOX_ID}_2..."

ros2 service call \
  /planning_scene_manager/remove_collision_objects \
  rosiris_manipulation_interfaces/srv/RemoveCollisionObjects \
  "object_ids: ['${BOX_ID}_2']"


echo "Done."

