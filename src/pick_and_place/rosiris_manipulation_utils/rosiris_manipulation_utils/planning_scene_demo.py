#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from rosiris_manipulation_interfaces.srv import (
    AddCollisionObjects,
    MoveCollisionObjects,
    AttachCollisionObject,
    DetachCollisionObject,
    RemoveCollisionObjects,
)
from rosiris_manipulation_interfaces.msg import CollisionEntry, CollisionObject as COWithCollisions


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

        for cli in [
            self.add_cli, self.move_cli, self.attach_cli,
            self.detach_cli, self.remove_cli
        ]:
            cli.wait_for_service()

        self.run_demo()

    def run_demo(self):
        self.add_objects()
        self.sleep(2.0)

        self.move_objects()
        self.sleep(2.0)

        self.attach_box()
        self.sleep(2.0)

        self.detach_box()
        self.sleep(2.0)

        self.remove_objects()
        self.get_logger().info("Demo finished successfully")

    def add_objects(self):
        self.get_logger().info("Adding box and cylinder")

        box = CollisionObject()
        box.id = "box"
        box.header.frame_id = "world"

        box_primitive = SolidPrimitive()
        box_primitive.type = SolidPrimitive.BOX
        box_primitive.dimensions = [0.2, 0.2, 0.2]

        box_pose = PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = 0.5
        box_pose.pose.position.z = 0.1
        box_pose.pose.orientation.w = 1.0

        box.primitives = [box_primitive]
        box.primitive_poses = [box_pose.pose]

        cylinder = CollisionObject()
        cylinder.id = "cylinder"
        cylinder.header.frame_id = "world"

        cyl_primitive = SolidPrimitive()
        cyl_primitive.type = SolidPrimitive.CYLINDER
        cyl_primitive.dimensions = [0.3, 0.05]  # height, radius

        cyl_pose = PoseStamped()
        cyl_pose.header.frame_id = "world"
        cyl_pose.pose.position.x = 0.7
        cyl_pose.pose.position.z = 0.15
        cyl_pose.pose.orientation.w = 1.0

        cylinder.primitives = [cyl_primitive]
        cylinder.primitive_poses = [cyl_pose.pose]

        req = AddCollisionObjects.Request()
        req.collision_objects = [
            COWithCollisions(collision_object=box, collisions=[]),
            COWithCollisions(collision_object=cylinder, collisions=[]),
        ]

        self.add_cli.call(req)

    def move_objects(self):
        self.get_logger().info("Moving box and cylinder")

        box_pose = PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = 0.4
        box_pose.pose.position.z = 0.2
        box_pose.pose.orientation.w = 1.0

        cyl_pose = PoseStamped()
        cyl_pose.header.frame_id = "world"
        cyl_pose.pose.position.x = 0.6
        cyl_pose.pose.position.z = 0.25
        cyl_pose.pose.orientation.w = 1.0

        req = MoveCollisionObjects.Request()
        req.object_id = ["box", "cylinder"]
        req.pose = [box_pose, cyl_pose]

        self.move_cli.call(req)

    def attach_box(self):
        self.get_logger().info("Attaching box to end effector")

        collision = CollisionEntry()
        collision.object_id = "box"
        collision.collision_allowed = True

        req = AttachCollisionObject.Request()
        req.collision_object_id = "box"
        req.attach_to_link = "ee_link"
        req.collisions = [collision]

        self.attach_cli.call(req)

    def detach_box(self):
        self.get_logger().info("Detaching box from end effector")

        req = DetachCollisionObject.Request()
        req.collision_object_id = "box"
        req.detach_to_link = "ee_link"
        req.collisions = []

        self.detach_cli.call(req)

    def remove_objects(self):
        self.get_logger().info("Removing box and cylinder")

        req = RemoveCollisionObjects.Request()
        req.object_id = ["box", "cylinder"]

        self.remove_cli.call(req)

    def sleep(self, seconds: float):
        self.get_clock().sleep_for(
            rclpy.duration.Duration(seconds=seconds)
        )


def main():
    rclpy.init()
    node = PlanningSceneDemo()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
