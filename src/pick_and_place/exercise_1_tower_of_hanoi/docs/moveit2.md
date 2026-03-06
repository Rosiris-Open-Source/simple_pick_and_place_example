architecture: 
https://moveit.picknik.ai/main/index.html
https://moveit.picknik.ai/main/doc/concepts/planning_scene_monitor.html

Important takeaway from https://moveit.picknik.ai/main/doc/concepts/planning_scene_monitor.html

Particularly, the move_group node as well as the rviz planning scene plugin maintain their own Planning Scene Monitor (PSM). The move_group node’s PSM listens to the topic /planning_scene and publishes its planning scene state to the topic monitored_planning_scene. The latter is listened to by the rviz planning scene plugin.

# Octomap
https://moveit.picknik.ai/humble/doc/examples/perception_pipeline/perception_pipeline_tutorial.html
https://moveit.picknik.ai/main/doc/examples/perception_pipeline/perception_pipeline_tutorial.html
https://www.youtube.com/watch?v=-EMLBPCnnkI
# Recognize objects service

# Realtime control
Great question — they're solving the **same problem** but at different levels of the stack.

---

## MoveIt Servo vs PicknikTwistController
Twist controller:
https://github.com/PickNikRobotics/picknik_controllers?tab=readme-ov-file#pickniktwistcontroller

| | MoveIt Servo | PicknikTwistController |
|---|---|---|
| **Runs in** | ROS2 / MoveIt layer | `ros2_control` layer |
| **Does IK** | Yes — in software | No — delegates to robot driver |
| **Output** | Joint commands → `JointTrajectoryController` | Twist → hardware interface directly |
| **Collision avoidance** | Yes (optional) | No |
| **Singularity handling** | Yes (built-in) | Depends on robot driver |
| **Robot requirements** | Any robot with joint control | Robot driver must support Cartesian twist natively |

---

## The Core Architectural Difference

**MoveIt Servo** does all the hard work itself:
```
Twist input → MoveIt Servo (solves IK, checks collisions) → joint commands → ros2_control → hardware
```

**PicknikTwistController** just passes the buck to the robot:
```
Twist input → ros2_control → hardware driver (robot solves it internally)
```

---

## When to Use Which

**Use MoveIt Servo when:**
- Your robot doesn't have a built-in Cartesian controller (most industrial arms, basically all of them without a fancy driver)
- You need collision avoidance during servoing
- You need singularity avoidance handled for you
- You're on a generic robot (UR, Franka, etc.)

**Use PicknikTwistController when:**
- Your robot driver **natively supports** Cartesian twist (e.g. some PickNik-specific hardware or drivers that expose a Cartesian hardware interface)
- You want **lower latency** — no IK computation in the middle
- You trust the robot's own internal Cartesian control to be better than software IK

---

## TL;DR

MoveIt Servo is the **general-purpose solution** that works on almost any robot. PicknikTwistController is a **thin passthrough** for robots that already speak Cartesian natively — fewer moving parts, lower latency, but only works if the hardware supports it.

In practice, **most people use MoveIt Servo** unless they're working with specific hardware that has its own Cartesian streaming interface.