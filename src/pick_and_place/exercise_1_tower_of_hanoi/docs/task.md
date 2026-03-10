Problems with this approach

1. Manual grasp frames -> e.g. grasp from front will fail, demonstrate with maybe an obstacle
2. Unsafe Cartesian assumptions:
    * Approach and retreat vectors overwrite position instead of offsetting: `approach_grasp_pose.pose.position.x = approach_vector.x();`
3. No grasp planning:
    * No fallback grasp strategies.
4. No feedback:
    * No perception integration

Architectural:
1. Violates separation of concerns, all logic in once place:
    * Planning
    * execution
    * gripper control
    * collision management
    * and task logic all in one class.
2. No state machine or behavior tree
3. Hard-coded configuration:
    * Planner IDs, frame names, link names, and topics hard-coded.
    * Not configurable via parameters.

    MoveitProblems:

    1. No kinematics Plugin -> cannot move in rviz2
    2. Multiple eef links -> robot green and does not "stick to base" when moved in rviz2
    3. Missing controllers -> cannot move