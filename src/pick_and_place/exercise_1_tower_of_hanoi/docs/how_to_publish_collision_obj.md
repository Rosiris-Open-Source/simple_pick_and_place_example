```python
    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    import time

    rclpy.init()
    node = Node('test_subframes')

    # Create publisher
    pub = node.create_publisher(CollisionObject, '/collision_object', 10)
    time.sleep(1)  # Wait for publisher to connect

    # Create collision object
    collision_object = CollisionObject()
    collision_object.header.frame_id = 'world'
    collision_object.id = 'test_box'

    # Add box primitive
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [0.3, 0.3, 0.3]

    box_pose = Pose()
    box_pose.position.x = 0.6
    box_pose.position.y = 0.0
    box_pose.position.z = 0.15
    box_pose.orientation.w = 1.0

    collision_object.primitives = [primitive]
    collision_object.primitive_poses = [box_pose]

    # Add subframes
    collision_object.subframe_names = ['top_grasp', 'side_grasp']

    top_grasp = Pose()
    top_grasp.position.z = 0.15
    top_grasp.orientation.w = 1.0

    side_grasp = Pose()
    side_grasp.position.x = 5.15
    side_grasp.orientation.y = 0.707
    side_grasp.orientation.w = 0.707

    collision_object.subframe_poses = [top_grasp, side_grasp]
    collision_object.operation = CollisionObject.ADD

    # Publish
    pub.publish(collision_object)
    print("Published collision object with subframes")

    time.sleep(2)
    rclpy.shutdown()
```