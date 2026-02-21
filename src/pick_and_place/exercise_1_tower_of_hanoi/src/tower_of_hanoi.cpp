#include "exercise_1_tower_of_hanoi/tower_of_hanoi.hpp"

#include <rcl_interfaces/msg/integer_range.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosiris_manipulation_interfaces/msg/service_result.hpp>
namespace tower_of_hanoi {

using ParallelGripperCommand = control_msgs::action::ParallelGripperCommand;
using GoalHandleGripper =
    rclcpp_action::ClientGoalHandle<ParallelGripperCommand>;

TowerOfHanoi::TowerOfHanoi(std::string node_name,
                           const rclcpp::NodeOptions &options)
    : Node(node_name, options) {
  getParameters();
  setupServiceClients();
  setupActionClients();
}

void TowerOfHanoi::buildTowerOfHanoi() {
  auto move_group = moveit::planning_interface::MoveGroupInterface(
      shared_from_this(), "manipulator_robotiq_85");
  configurePlannerOmpl(move_group);
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
  openGripper();

  // configure to move lin
  configurePlannerPilzLin(move_group);

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
  closeGripper();

  if (!attachObjectToGripper("cube_3")) {
    return;
  }

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

void TowerOfHanoi::getParameters() {
  param_listener_ = std::make_shared<tower_of_hanoi::ParamListener>(
      get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // validate parameters
  if (params_.gripper_names.empty()) {
    throw std::runtime_error("Parameter 'gripper_names' cannot be empty.");
  }
  for (const auto &gripper_name : params_.gripper_names) {
    auto gripper_it = params_.grippers.gripper_names_map.find(gripper_name);
    if (gripper_it == params_.grippers.gripper_names_map.end()) {
      throw std::runtime_error("Gripper '" + gripper_name +
                               "' listed in 'gripper_names' is not defined in "
                               "'grippers' parameter.");
    }
    if (gripper_it->second.eef_link.empty()) {
      throw std::runtime_error("Gripper '" + gripper_name +
                               "' must have a non-empty 'eef_link' defined.");
    }
  }
  if (std::find(params_.gripper_names.begin(), params_.gripper_names.end(),
                params_.used_gripper) == params_.gripper_names.end()) {
    throw std::runtime_error("Parameter 'used_gripper' must be one of the "
                             "entries in 'gripper_names'.");
  }
}

void TowerOfHanoi::setupServiceClients() {
  robot_attach_cli_ = create_client<
      rosiris_manipulation_interfaces::srv::AttachCollisionObject>(
      "planning_scene_manager/attach_collision_object");
  while (!robot_attach_cli_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      std::stringstream interrup_err_msg;
      interrup_err_msg << "Interrupted while waiting for <"
                       << robot_attach_cli_->get_service_name()
                       << "> service to become available.";
      throw std::runtime_error(interrup_err_msg.str());
    }
    RCLCPP_INFO_STREAM(get_logger(),
                       "waiting for service "
                           << robot_attach_cli_->get_service_name() << " ...");
  }

  robot_detach_cli_ = create_client<
      rosiris_manipulation_interfaces::srv::DetachCollisionObject>(
      "planning_scene_manager/detach_collision_object");
  while (!robot_detach_cli_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      std::stringstream interrup_err_msg;
      interrup_err_msg << "Interrupted while waiting for <"
                       << robot_detach_cli_->get_service_name()
                       << "> service to become available.";
      throw std::runtime_error(interrup_err_msg.str());
    }
    RCLCPP_INFO_STREAM(get_logger(),
                       "waiting for service "
                           << robot_detach_cli_->get_service_name() << " ...");
  }
}

void TowerOfHanoi::setupActionClients() {
  gripper_client_ = rclcpp_action::create_client<ParallelGripperCommand>(
      this, "/robotiq_gripper_controller/gripper_cmd");
}

void TowerOfHanoi::configurePlannerOmpl(
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

void TowerOfHanoi::configurePlannerPilzLin(
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

bool TowerOfHanoi::attachObjectToGripper(const std::string &obj_to_attach) {
  auto attach_req = std::make_shared<
      rosiris_manipulation_interfaces::srv::AttachCollisionObject::Request>();
  attach_req->attach_to_link =
      params_.grippers.gripper_names_map.at(params_.used_gripper).eef_link;
  attach_req->collision_object_id = obj_to_attach;
  attach_req->allowed_touch_links =
      params_.grippers.gripper_names_map.at(params_.used_gripper)
          .allowed_touch_links;

  auto result_future = robot_attach_cli_->async_send_request(attach_req);
  auto status = result_future.wait_for(
      std::chrono::milliseconds(params_.srv_call_timeout));
  if (status != std::future_status::ready) {
    // either status == std::future_status::timeout
    // or future deferred
    robot_attach_cli_->remove_pending_request(result_future);
    RCLCPP_ERROR_STREAM(get_logger(),
                        "service call to "
                            << robot_attach_cli_->get_service_name()
                            << " failed.");
    return false;
  }
  auto result = result_future.get()->result;

  using ret_type = rosiris_manipulation_interfaces::msg::ServiceResult;
  if (result.return_type != ret_type::SUCCESS) {
    return false;
  }

  return true;
}

bool TowerOfHanoi::detachObjectFromGripper(const std::string &obj_to_detach) {
  auto detach_req = std::make_shared<
      rosiris_manipulation_interfaces::srv::DetachCollisionObject::Request>();
  detach_req->detach_from_link =
      params_.grippers.gripper_names_map.at(params_.used_gripper).eef_link;
  ;
  detach_req->collision_object_id = obj_to_detach;
  detach_req->disallowed_touch_links =
      params_.grippers.gripper_names_map.at(params_.used_gripper)
          .allowed_touch_links;

  auto result_future = robot_detach_cli_->async_send_request(detach_req);
  auto status = result_future.wait_for(
      std::chrono::milliseconds(params_.srv_call_timeout));
  if (status != std::future_status::ready) {
    // either status == std::future_status::timeout
    // or future deferred
    robot_detach_cli_->remove_pending_request(result_future);
    RCLCPP_ERROR_STREAM(get_logger(),
                        "service call to "
                            << robot_detach_cli_->get_service_name()
                            << " failed.");
    return false;
  }
  auto result = result_future.get()->result;

  using ret_type = rosiris_manipulation_interfaces::msg::ServiceResult;
  if (result.return_type != ret_type::SUCCESS) {
    return false;
  }

  return true;
}

void TowerOfHanoi::homeRobot() {
  auto move_group = moveit::planning_interface::MoveGroupInterface(
      shared_from_this(), "manipulator");
  configurePlannerOmpl(move_group);

  // Use the "Home" named target from your SRDF
  move_group.setNamedTarget("Home");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  RCLCPP_INFO(this->get_logger(), "Planning to Home state...");

  auto success = move_group.plan(my_plan);

  if (success == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Plan found! Executing...");
    move_group.execute(my_plan);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planning failed! Code: %d", success.val);
  }
}

void TowerOfHanoi::sendGripperCommand(double pos, double effort) {
  auto goal_msg = ParallelGripperCommand::Goal();
  goal_msg.command.position.push_back(pos);
  goal_msg.command.effort.push_back(effort);
  gripper_client_->wait_for_action_server();
  gripper_client_->async_send_goal(goal_msg);
  // Add sleep or wait for result logic here
  rclcpp::sleep_for(std::chrono::seconds(1));
}

} // namespace tower_of_hanoi

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec(
      rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  auto node = std::make_shared<tower_of_hanoi::TowerOfHanoi>();
  exec.add_node(node);
  auto spin_thread = std::make_unique<std::thread>([&exec, &node]() {
    exec.spin();
    exec.remove_node(node);
  });

  node->buildTowerOfHanoi();

  // we are finished so we shutdown and wait for the spin_thread to shutdown
  rclcpp::shutdown();
  spin_thread->join();
  return 0;
}
