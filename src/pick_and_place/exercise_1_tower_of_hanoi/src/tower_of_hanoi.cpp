#include <control_msgs/action/parallel_gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using ParallelGripperCommand = control_msgs::action::ParallelGripperCommand;
using GoalHandleGripper =
    rclcpp_action::ClientGoalHandle<ParallelGripperCommand>;

class TowerOfHanoiGrasp : public rclcpp::Node {
public:
  TowerOfHanoiGrasp() : Node("hanoi_grasp_node") {
    // Action client for the gripper
    gripper_client_ = rclcpp_action::create_client<ParallelGripperCommand>(
        this, "/robotiq_gripper_controller/gripper_cmd");
  }

  void open_gripper() { sendGripperCommand(0.0, 0.1); }

  void close_gripper() { sendGripperCommand(0.8, 0.5); }

  void configure_planner_ompl(
      moveit::planning_interface::MoveGroupInterface &move_group) {
    // -- Set the Planner --
    // For OMPL, "RRTConnectkConfigDefault" is a standard robust choice
    move_group.setPlanningPipelineId("ompl");
    move_group.setPlannerId("RRTConnectkConfigDefault");

    // -- Set Planning Parameters --
    move_group.setPlanningTime(5.0); // Give it time to find a solution
    move_group.setNumPlanningAttempts(10);
    move_group.setGoalTolerance(0.01);

    // -- Set Tolerances --
    // Position tolerance in meters (e.g., 0.001 = 1mm)
    move_group.setGoalPositionTolerance(0.001);
    // Orientation tolerance in radians (e.g., 0.01 = ~0.5 degrees)
    move_group.setGoalOrientationTolerance(0.01);

    // -- Set Scaling Factors (Speed) --
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
  }

  void configure_planner_pilz_lin(
      moveit::planning_interface::MoveGroupInterface &move_group) {

    // 1. Switch to the Pilz Industrial Motion Planner pipeline
    move_group.setPlanningPipelineId("pilz");

    // 2. Set the solver to LIN (Linear)
    // Options are usually "LIN", "CIRC", or "PTP"
    move_group.setPlannerId("LIN");

    // 3. Pilz is very strict. It requires well-defined limits.
    move_group.setPlanningTime(
        2.0); // LIN plans are fast (math-based, not search-based)

    // 4. Set scaling factors
    // Note: Pilz often ignores global MoveIt velocity scaling unless
    // configured specifically in the joint_limits.yaml
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);
  }

  void executeGrasp() {
    auto move_group = moveit::planning_interface::MoveGroupInterface(
        shared_from_this(), "manipulator_robotiq_85");
    configure_planner_ompl(move_group);
    // 1. Pre-grasp Pose (5cm +y from grasp_point)
    // Assuming 'grasp_point' is a frame in your TF tree or planning scene
    geometry_msgs::msg::PoseStamped grasp_pose;
    // In a real scenario, lookupTransform "base_link" ->
    // "move_group/cube_1/grasp_point"
    grasp_pose.header.frame_id = "move_group/cube_3/grasp_point";
    grasp_pose.pose.position.y = 0.05; // 5cm offset
    grasp_pose.pose.orientation.w = 1.0;

    move_group.setPoseTarget(grasp_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    RCLCPP_INFO(this->get_logger(), "Planning to pregrasp pose...");
    auto success = move_group.plan(my_plan);

    if (success == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
      move_group.execute(my_plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed! Error code: %d",
                   success.val);
      return;
    }
    // 2. Open Gripper
    open_gripper();

    // configure to move lin
    configure_planner_pilz_lin(move_group);

    // 3. Move to Grasp Point
    grasp_pose.pose.position.y = 0.0;
    move_group.setPoseTarget(grasp_pose);
    RCLCPP_INFO(this->get_logger(), "Planning to grasp pose...");
    success = move_group.plan(my_plan);

    if (success == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
      move_group.execute(my_plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed! Error code: %d",
                   success.val);
      return;
    }

    // 4. Close Gripper
    close_gripper();

    // 5. Move Linear (Post-grasp 5cm +y)
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose post_grasp = move_group.getCurrentPose().pose;
    post_grasp.position.y += 0.05;
    waypoints.push_back(post_grasp);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction =
        move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    if (fraction > 0.9) {
      move_group.execute(trajectory);
    }
  }

  void home() {
    auto move_group = moveit::planning_interface::MoveGroupInterface(
        shared_from_this(), "manipulator");
    configure_planner_ompl(move_group);

    // Use the "Home" named target from your SRDF
    move_group.setNamedTarget("Home");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    RCLCPP_INFO(this->get_logger(), "Planning to Home state...");

    auto success = move_group.plan(my_plan);

    if (success == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Plan found! Executing...");
      move_group.execute(my_plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed! Code: %d",
                   success.val);
    }
  }

  void home_with_goal_pose() {
    auto move_group = moveit::planning_interface::MoveGroupInterface(
        shared_from_this(), "manipulator");
    configure_planner_ompl(move_group);

    // Define the target pose
    geometry_msgs::msg::Pose target_pose;

    target_pose.position.x = 1.4758;
    target_pose.position.y = 1.776;
    target_pose.position.z = 1.2698;

    target_pose.orientation.x = 0.49945;
    target_pose.orientation.y = 0.49519;
    target_pose.orientation.z = 0.50571;
    target_pose.orientation.w = 0.49961;

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    RCLCPP_INFO(this->get_logger(),
                "Planning to Home Pose (Cartesian Goal)...");

    auto success = move_group.plan(my_plan);

    if (success == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Pose plan found! Executing...");
      move_group.execute(my_plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Pose planning failed! Error Code: %d",
                   success.val);

      // Peer tip: If this fails with 99999, double check that "base_link"
      // is the reference frame for these coordinates.
    }
  }

private:
  void sendGripperCommand(double pos, double effort) {
    auto goal_msg = ParallelGripperCommand::Goal();
    goal_msg.command.position.push_back(pos);
    goal_msg.command.effort.push_back(effort);
    gripper_client_->wait_for_action_server();
    gripper_client_->async_send_goal(goal_msg);
    // Add sleep or wait for result logic here
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  rclcpp_action::Client<ParallelGripperCommand>::SharedPtr gripper_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec(
      rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  auto node = std::make_shared<TowerOfHanoiGrasp>();
  exec.add_node(node);
  auto spin_thread = std::make_unique<std::thread>([&exec, &node]() {
    exec.spin();
    exec.remove_node(node);
  });

  node->executeGrasp();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
