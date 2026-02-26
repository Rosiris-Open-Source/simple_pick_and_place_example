# Copyright 2026 Manuel Muth
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env python3
import math
from urllib.parse import urlparse

import rclpy

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import Mesh, SolidPrimitive
from tf_transformations import quaternion_from_euler
from rosiris_manipulation_utils.mesh_loader import MeshLoader

from rosiris_manipulation_interfaces.srv import (
    AddCollisionObjects,
    MoveCollisionObjects,
    AttachCollisionObject,
    DetachCollisionObject,
    RemoveCollisionObjects,
    UpdateAllowedCollisions,
)
from rosiris_manipulation_interfaces.msg import (
    CollisionObject as RosirisCollisionObject,
    CollisionObjectPoseUpdate,
    CollisionMatrixUpdate,
    CollisionEntry,
)


BOX_ID = "box_458398093"
FRAME = "world"
EEF_LINK = "end_effector_link"


class PlanningSceneDemo(Node):

    def __init__(self):
        super().__init__("planning_scene_demo")

        self.add_cli = self.create_client(
            AddCollisionObjects, "/planning_scene_manager/add_collision_objects")
        self.move_cli = self.create_client(
            MoveCollisionObjects, "/planning_scene_manager/move_collision_objects")
        self.attach_cli = self.create_client(
            AttachCollisionObject, "/planning_scene_manager/attach_collision_object")
        self.detach_cli = self.create_client(
            DetachCollisionObject, "/planning_scene_manager/detach_collision_object")
        self.remove_cli = self.create_client(
            RemoveCollisionObjects, "/planning_scene_manager/remove_collision_objects")
        self.acm_cli = self.create_client(
            UpdateAllowedCollisions, "/planning_scene_manager/update_allowed_collisions")

        for cli in [
            self.add_cli, self.move_cli, self.attach_cli,
            self.detach_cli, self.remove_cli, self.acm_cli
        ]:
            cli.wait_for_service()

    def run(self):
        print("start demo:")
        self.sleep(2.0)

        print("Add box.")
        self.add_box()
        self.sleep(2.0)

        print("Move box in desk.")
        pose = PoseStamped()
        pose.header.frame_id = FRAME
        pose.pose.position.x = 1.5
        pose.pose.position.y = 1.5
        pose.pose.position.z = 0.8
        pose.pose.orientation.w = 1.0
        self.move_box(pose=pose)
        self.sleep(2.0)

        print("Allow collisions box and desk.")
        self.allow_box_desk_collision()
        self.sleep(2.0)

        print("Attach box to eef.")
        self.attach_box()
        self.sleep(2.0)

        print("Detach box from eef.")
        self.detach_box()
        self.sleep(2.0)

        print("Move box to world.")
        pose.header.frame_id = FRAME

        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0

        roll  = math.radians(45.0)
        pitch = math.radians(45.0)
        yaw   = math.radians(45.0)

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.move_box(pose=pose)
        self.sleep(2.0)

        print("remove box")
        self.remove_box()

        print("Demo finished")
        exit()


    def call_and_wait(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    # ---------- ADD ----------
    def add_box(self):
        box = CollisionObject()
        box.id = BOX_ID
        box.header.frame_id = FRAME

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.2, 0.2, 0.2]

        box.primitives = [primitive]

        mesh_path = "package://rosiris_environment_description_common/meshes/desk/desk_visual.stl"
        try:
            mesh : Mesh = MeshLoader.mesh_from_file(path=mesh_path)
            box.meshes = [mesh]
        except (FileNotFoundError, RuntimeError) as e:
            self.get_logger().warn(f"Could not create mesh from file: {e}. Fallback to primitive only.")

        pose = PoseStamped()
        pose.header.frame_id = FRAME
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.1
        pose.pose.orientation.w = 1.0

        box.primitive_poses = [pose.pose]

        req = AddCollisionObjects.Request()
        collision_obj = RosirisCollisionObject()
        collision_obj.collision_object = box
        req.collision_objects.append(collision_obj)

        self.call_and_wait(self.add_cli, req)

    # ---------- MOVE ----------
    def move_box(self, pose: PoseStamped):
        update = CollisionObjectPoseUpdate()
        update.object_id = BOX_ID

        update.pose = pose
        update.collision_matrix_update = CollisionMatrixUpdate()

        req = MoveCollisionObjects.Request()
        req.collision_object_pose_updates = [update]

        self.call_and_wait(self.move_cli, req)

    # ---------- ACM ----------
    def allow_box_desk_collision(self):
        entry = CollisionEntry()
        entry.touch_link = "desk_1_link"
        entry.collision_allowed = True

        cmu = CollisionMatrixUpdate()
        cmu.target_link = BOX_ID
        cmu.mode = CollisionMatrixUpdate.MERGE
        cmu.collision_entries = [entry]

        req = UpdateAllowedCollisions.Request()
        req.collision_matrix_updates = [cmu]

        self.call_and_wait(self.acm_cli, req)

    # ---------- ATTACH ----------
    def attach_box(self):
        req = AttachCollisionObject.Request()
        req.collision_object_id = BOX_ID
        req.attach_to_link = EEF_LINK
        req.allowed_touch_links = []

        self.call_and_wait(self.attach_cli, req)

    # ---------- DETACH ----------
    def detach_box(self):
        req = DetachCollisionObject.Request()
        req.detach_from_link = EEF_LINK
        req.collision_object_id = BOX_ID
        req.disallowed_touch_links = []

        self.call_and_wait(self.detach_cli, req)

    # ---------- REMOVE ----------
    def remove_box(self):
        req = RemoveCollisionObjects.Request()
        req.object_ids = [BOX_ID]

        self.call_and_wait(self.remove_cli, req)

    def sleep(self, seconds: float):
        self.get_clock().sleep_for(
            rclpy.duration.Duration(seconds=seconds)
        )


def main():
    rclpy.init()
    node = PlanningSceneDemo()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    node.run()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
