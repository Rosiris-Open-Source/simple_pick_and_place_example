#include "exercise_1_tower_of_hanoi/tower_of_hanoi.hpp"

#include <rcl_interfaces/msg/integer_range.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosiris_manipulation_interfaces/msg/collision_entry.hpp>
#include <rosiris_manipulation_interfaces/msg/service_result.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace tower_of_hanoi {

using ParallelGripperCommand = control_msgs::action::ParallelGripperCommand;
using GoalHandleGripper =
    rclcpp_action::ClientGoalHandle<ParallelGripperCommand>;

TowerOfHanoi::TowerOfHanoi(
    std::shared_ptr<transform_manager::TransformManager> transform_manager,
    std::string reference_frame, std::string node_name,
    const rclcpp::NodeOptions &options)
    : Node(node_name, options), transform_manager_(transform_manager),
      reference_frame_(reference_frame) {
  getParameters();
  setupServiceClients();
  setupActionClients();
}

void TowerOfHanoi::buildTowerOfHanoi() {
  auto cube3 = cubes_.at("cube_3");
  auto cube2 = cubes_.at("cube_2");
  auto cube1 = cubes_.at("cube_1");

  using Step = std::function<bool()>;

  std::vector<Step> plan = {
      [&] { return homeRobot(); },
      [&] { return pickCube(cube3); },
      [&] { return placeCubeAtLocation(cube3, Location::MIDDLE); },
      [&] { return pickCube(cube2); },
      [&] { return placeCubeOnCube(cube2, cube3); },
      [&] { return pickCube(cube1); },
      [&] { return placeCubeAtLocation(cube1, Location::LEFT); },
      [&] { return pickCube(cube2); },
      [&] { return placeCubeOnCube(cube2, cube1); },
      [&] { return pickCube(cube3); },
      [&] { return placeCubeOnCube(cube3, cube2); },
      [&] { return homeRobot(); }};

  for (auto &step : plan) {
    if (!step()) {
      return;
    }
  }
}

void TowerOfHanoi::getParameters() {
  param_listener_ = std::make_unique<tower_of_hanoi::ParamListener>(
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

  scene_upd_collision_srv_ = create_client<
      rosiris_manipulation_interfaces::srv::UpdateAllowedCollisions>(
      "planning_scene_manager/update_allowed_collisions");
  while (!scene_upd_collision_srv_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      std::stringstream interrup_err_msg;
      interrup_err_msg << "Interrupted while waiting for <"
                       << scene_upd_collision_srv_->get_service_name()
                       << "> service to become available.";
      throw std::runtime_error(interrup_err_msg.str());
    }
    RCLCPP_INFO_STREAM(get_logger(),
                       "waiting for service "
                           << scene_upd_collision_srv_->get_service_name()
                           << " ...");
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
  move_group.setGoalTolerance(0.1);

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

  // -- Set Planning Parameters --
  move_group.setPlanningTime(5.0); // Give it time to find a solution
  move_group.setNumPlanningAttempts(10);
  move_group.setGoalTolerance(0.1);

  // -- Set Tolerances --
  // Position tolerance in meters (e.g., 0.001 = 1mm)
  move_group.setGoalPositionTolerance(0.001);
  // Orientation tolerance in radians (e.g., 0.01 = ~0.5 degrees)
  move_group.setGoalOrientationTolerance(0.01);

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

bool TowerOfHanoi::updateAllowedCollision(
    const Cube &cube, const std::vector<std::string> &allow_collisions,
    const std::vector<std::string> &disallow_collisions,
    rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate::_mode_type
        mode) {
  auto update_col_req = std::make_shared<
      rosiris_manipulation_interfaces::srv::UpdateAllowedCollisions::Request>();

  // create collision matrix update msg
  rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate
      collision_matrix_update{};
  collision_matrix_update.target_link = cube.id();
  collision_matrix_update.mode = mode;
  // update collisions
  for (const auto &allow_link : allow_collisions) {
    collision_matrix_update.collision_entries.emplace_back()
        .set__touch_link(allow_link)
        .set__collision_allowed(true);
  }
  for (const auto &disallow_link : disallow_collisions) {
    collision_matrix_update.collision_entries.emplace_back()
        .set__touch_link(disallow_link)
        .set__collision_allowed(false);
  }
  update_col_req->collision_matrix_updates.push_back(collision_matrix_update);

  auto result_future =
      scene_upd_collision_srv_->async_send_request(update_col_req);
  auto status = result_future.wait_for(
      std::chrono::milliseconds(params_.srv_call_timeout));
  if (status != std::future_status::ready) {
    // either status == std::future_status::timeout
    // or future deferred
    scene_upd_collision_srv_->remove_pending_request(result_future);
    RCLCPP_ERROR_STREAM(get_logger(),
                        "service call to "
                            << scene_upd_collision_srv_->get_service_name()
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

bool TowerOfHanoi::homeRobot() {
  RCLCPP_INFO(get_logger(), "%s():", __func__);
  auto move_group = moveit::planning_interface::MoveGroupInterface(
      shared_from_this(), "manipulator");
  configurePlannerOmpl(move_group);

  // Use the "Home" named target from SRDF
  const std::string target = "Home";
  move_group.setNamedTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  RCLCPP_INFO(get_logger(), "%s(): Planning to target: '%s'.", __func__,
              target.c_str());

  auto plan_result = move_group.plan(my_plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "%s():Planning failed for target '%s' Code: %d",
                 __func__, target.c_str(), plan_result.val);
    return false;
  }

  RCLCPP_INFO(get_logger(), "%s():Plan found for target '%s', executing...",
              __func__, target.c_str());

  moveit::core::MoveItErrorCode move_result;
  move_result = move_group.execute(my_plan);
  if (move_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "%s():Execution failed for target '%s' Code: %d",
                 __func__, target.c_str(), move_result.val);
    return false;
  }
  RCLCPP_INFO(get_logger(), "%s():Successfully moved to target '%s'", __func__,
              target.c_str());

  return true;
}

bool TowerOfHanoi::pickCube(const Cube &cube) {
  const auto cube_name = cube.id().c_str();
  RCLCPP_INFO(get_logger(), "%s(%s)", __func__, cube_name);
  removeVisualizedPoses();
  auto grasp_frame = cube.grasp_point();
  geometry_msgs::msg::PoseStamped grasp_pose{};
  grasp_pose.header.frame_id = grasp_frame;
  grasp_pose.pose.orientation.w = 1.0;
  visualizePose(grasp_pose, grasp_frame + "_grasp");

  // create approach pose
  auto approach_vector = pre_grasp_direction_ * pre_grasp_distance_;
  geometry_msgs::msg::PoseStamped approach_grasp_pose = grasp_pose;
  approach_grasp_pose.pose.position.x = approach_vector.x();
  approach_grasp_pose.pose.position.y = approach_vector.y();
  approach_grasp_pose.pose.position.z = approach_vector.z();

  visualizePose(approach_grasp_pose, grasp_frame + "_approach_grasp");
  if (!moveOmpl(approach_grasp_pose)) {
    RCLCPP_ERROR(get_logger(),
                 "%s(%s): Failed to move to approach grasp pose '%s'.",
                 __func__, cube_name, grasp_frame.c_str());
    return false;
  }

  if (!openGripper()) {
    RCLCPP_ERROR(get_logger(), "%s(%s): Failed to open gripper.", __func__,
                 cube_name);
    return false;
  }

  if (!moveLin(grasp_pose)) {
    RCLCPP_ERROR(get_logger(), "%s(%s): Failed to move to grasp point '%s'.",
                 __func__, cube_name, grasp_frame.c_str());
    return false;
  }

  if (!closeGripper()) {
    RCLCPP_ERROR(get_logger(), "%s(%s): Failed to close gripper.", __func__,
                 cube_name);
    return false;
  }

  if (!attachObjectToGripper(cube.id())) {
    RCLCPP_ERROR(get_logger(), "%s(%s): Failed to attach object '%s'.",
                 __func__, cube_name, cube_name);
    return false;
  }

  auto retreat_vector = post_grasp_direction_ * post_grasp_distance_;
  geometry_msgs::msg::PoseStamped retreat_grasp_pose = grasp_pose;
  retreat_grasp_pose.pose.position.x = retreat_vector.x();
  retreat_grasp_pose.pose.position.y = retreat_vector.y();
  retreat_grasp_pose.pose.position.z = retreat_vector.z();

  visualizePose(retreat_grasp_pose, grasp_frame + "_retreat_grasp");
  if (!moveLin(retreat_grasp_pose)) {
    RCLCPP_ERROR(get_logger(),
                 "%s(%s): Failed to move to retreat grasp pose '%s'.", __func__,
                 cube_name, grasp_frame.c_str());
    return false;
  }

  // update collisions to exclusive only allow collisions with gripper since we
  // pick the cube it was before allowed to touch the objects it was placed on
  // which we don't want anymore
  if (!updateAllowedCollision(
          cube,
          params_.grippers.gripper_names_map.at(params_.used_gripper)
              .allowed_touch_links,
          {},
          rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate::
              REPLACE)) {
    RCLCPP_ERROR(get_logger(),
                 "%s(%s): Failed to updated allowed collisions for obj '%s'.",
                 __func__, cube_name, cube.id().c_str());
    return false;
  }

  return true;
}

bool TowerOfHanoi::placeCubeAtLocation(const Cube &cube_to_place,
                                       const Location &location) {
  const auto cube_name = cube_to_place.id().c_str();
  std::string location_name;
  geometry_msgs::msg::PoseStamped place_pose{};
  try {
    location_name = place_locations_.at(location);
    place_pose.header.frame_id = location_name;
  } catch (const std::out_of_range &e) {
    RCLCPP_ERROR(get_logger(), "%s(%s, %i): Invalid location specified.",
                 __func__, cube_name, static_cast<int>(location));
    return false;
  }
  RCLCPP_INFO(get_logger(), "%s(%s, %s)", __func__, cube_name,
              location_name.c_str());

  place_pose.pose.position.z =
      cube_to_place.dimension() / 2.0 + cube_to_place.grasp_pose_offset();
  place_pose.pose.orientation = tf2::toMsg(rot_place_to_cube_);

  // create approach pose
  auto approach_vector = pre_place_distance_ * pre_place_location_direction_;
  geometry_msgs::msg::PoseStamped approach_place_pose = place_pose;
  approach_place_pose.pose.position.x = approach_vector.x();
  approach_place_pose.pose.position.y = approach_vector.y();
  approach_place_pose.pose.position.z = approach_vector.z();

  // create retreat pose
  auto retreat_vector = post_place_distance_ * post_place_location_direction_;
  geometry_msgs::msg::PoseStamped retreat_place_pose = place_pose;
  retreat_place_pose.pose.position.x = retreat_vector.x();
  retreat_place_pose.pose.position.y = retreat_vector.y();
  retreat_place_pose.pose.position.z = retreat_vector.z();
  return placeCube(cube_to_place, place_pose, approach_place_pose,
                   retreat_place_pose, {"desk_1_link"});
}

bool TowerOfHanoi::placeCubeOnCube(const Cube &cube_to_place,
                                   const Cube &target_cube) {
  RCLCPP_INFO(get_logger(), "%s(%s, %s)", __func__, cube_to_place.id().c_str(),
              target_cube.id().c_str());
  geometry_msgs::msg::PoseStamped place_pose{};
  place_pose.header.frame_id = target_cube.ref_frame();
  // Compute placement height so the bottom face of cube_to_place
  // rests exactly on top of target_cube.
  //
  // The reference frame is centered in target_cube.
  // Therefore:
  // - Add half the height of target_cube (top surface relative to its center)
  // - Add half the height of cube_to_place (its center relative to its bottom)
  // - Add grasp_pose_offset(), which accounts for the vertical offset
  //   between the cube center and the grasp frame used during manipulation
  //
  // This ensures the cubes are stacked without penetration or gap.
  place_pose.pose.position.z = cube_to_place.dimension() / 2.0 +
                               target_cube.dimension() / 2.0 +
                               cube_to_place.grasp_pose_offset();
  place_pose.pose.orientation = tf2::toMsg(rot_place_to_cube_);

  // create approach pose
  auto approach_vector = pre_place_distance_ * pre_place_cube_direction_;
  geometry_msgs::msg::PoseStamped approach_place_pose = place_pose;
  approach_place_pose.pose.position.x = approach_vector.x();
  approach_place_pose.pose.position.y = approach_vector.y();
  approach_place_pose.pose.position.z = approach_vector.z();

  // create retreat pose
  auto retreat_vector = post_place_distance_ * post_place_cube_direction_;
  geometry_msgs::msg::PoseStamped retreat_place_pose = place_pose;
  retreat_place_pose.pose.position.x = retreat_vector.x();
  retreat_place_pose.pose.position.y = retreat_vector.y();
  retreat_place_pose.pose.position.z = retreat_vector.z();

  return placeCube(cube_to_place, place_pose, approach_place_pose,
                   retreat_place_pose, {target_cube.id()});
}

bool TowerOfHanoi::placeCube(
    const Cube &cube_to_place,
    const geometry_msgs::msg::PoseStamped &place_pose,
    const geometry_msgs::msg::PoseStamped &approach_place_pose,
    const geometry_msgs::msg::PoseStamped &retreat_place_pose,
    const std::vector<std::string> &allow_collisions) {
  RCLCPP_INFO(get_logger(), "%s()", __func__);
  auto place_frame = place_pose.header.frame_id;
  removeVisualizedPoses();
  visualizePose(place_pose, place_frame + "_place");
  visualizePose(approach_place_pose, place_frame + "_approach_place");

  if (!moveOmpl(approach_place_pose)) {
    RCLCPP_ERROR(get_logger(),
                 "%s(): Failed to move to approach place pose '%s'.", __func__,
                 place_frame.c_str());
    return false;
  }

  // update collisions
  // extend with the object with place on given in allowd_collisions
  if (!updateAllowedCollision(
          cube_to_place, allow_collisions, {},
          rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate::MERGE)) {
    RCLCPP_ERROR(get_logger(),
                 "%s(): Failed to updated allowed collisions for obj '%s'.",
                 __func__, cube_to_place.id().c_str());
    return false;
  }

  if (!moveLin(place_pose)) {
    RCLCPP_ERROR(get_logger(), "%s(): Failed to move to place point '%s'.",
                 __func__, place_frame.c_str());
    return false;
  }

  if (!openGripper()) {
    RCLCPP_ERROR(get_logger(), "%s(): Failed to open gripper.", __func__);
    return false;
  }

  if (!detachObjectFromGripper(cube_to_place.id())) {
    RCLCPP_ERROR(get_logger(), "%s(): Failed to detach object '%s'.", __func__,
                 cube_to_place.id().c_str());
    return false;
  }

  visualizePose(retreat_place_pose, place_frame + "_retreat_place");
  if (!moveLin(retreat_place_pose)) {
    RCLCPP_ERROR(get_logger(),
                 "%s(): Failed to move to retreat place pose '%s'.", __func__,
                 place_frame.c_str());
    return false;
  }

  return true;
}

bool TowerOfHanoi::moveLocationOmpl(const Location &location, double offset) {
  RCLCPP_INFO(get_logger(), "%s(%i, %f)", __func__, static_cast<int>(location),
              offset);
  geometry_msgs::msg::PoseStamped goal_pose{};
  try {
    goal_pose.header.frame_id = place_locations_.at(location);
  } catch (const std::out_of_range &e) {
    RCLCPP_ERROR(get_logger(), "%s(%i, %f):Invalid location specified.",
                 __func__, static_cast<int>(location), offset);
    return false;
  }
  // rotation between place location and orientation of cubes
  goal_pose.pose.position.z = offset;
  goal_pose.pose.orientation = tf2::toMsg(rot_place_to_cube_);

  return moveOmpl(goal_pose);
}

bool TowerOfHanoi::moveLocationLin(const Location &location, double offset) {
  RCLCPP_INFO(get_logger(), "%s(%i, %f)", __func__, static_cast<int>(location),
              offset);
  geometry_msgs::msg::PoseStamped goal_pose{};
  try {
    goal_pose.header.frame_id = place_locations_.at(location);
  } catch (const std::out_of_range &e) {
    RCLCPP_ERROR(get_logger(), "%s(%i, %f): Invalid location specified.",
                 __func__, static_cast<int>(location), offset);
    return false;
  }
  // rotation between place location and orientation of cubes
  goal_pose.pose.position.z = offset;
  goal_pose.pose.orientation = tf2::toMsg(rot_place_to_cube_);

  return moveLin(goal_pose);
}

bool TowerOfHanoi::moveOmpl(
    const geometry_msgs::msg::PoseStamped &target_pose) {
  const auto target_pose_frame = target_pose.header.frame_id.c_str();
  RCLCPP_INFO(get_logger(), "%s(%s)", __func__, target_pose_frame);
  if (!move_group_) {
    RCLCPP_ERROR(get_logger(), "%s(%s): Move group interface not initialized.",
                 __func__, target_pose_frame);
    return false;
  }
  configurePlannerOmpl(*move_group_);
  return planAndMove(target_pose);
}

bool TowerOfHanoi::moveLin(const geometry_msgs::msg::PoseStamped &target_pose) {
  const auto target_pose_frame = target_pose.header.frame_id.c_str();
  RCLCPP_INFO(get_logger(), "%s(%s)", __func__, target_pose_frame);
  if (!move_group_) {
    RCLCPP_ERROR(get_logger(), "%s(%s): Move group interface not initialized.",
                 __func__, target_pose_frame);
    return false;
  }
  configurePlannerPilzLin(*move_group_);
  return planAndMove(target_pose);
}

bool TowerOfHanoi::planAndMove(
    const geometry_msgs::msg::PoseStamped &target_pose) {
  const auto target_pose_frame = target_pose.header.frame_id.c_str();
  RCLCPP_INFO(get_logger(), "%s(%s)", __func__, target_pose_frame);
  if (!move_group_) {
    RCLCPP_ERROR(get_logger(),
                 "%s(%s): Move group interface not initialized. "
                 "Call setMoveGroup() first.",
                 __func__, target_pose_frame);
    return false;
  }
  move_group_->clearPoseTargets();
  move_group_->setPoseTarget(target_pose);
  visualizePose(target_pose, std::string(target_pose_frame) + "_pose");
  // plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  RCLCPP_INFO(get_logger(), "%s(%s): Planning to target: '%s'.", __func__,
              target_pose_frame, target_pose_frame);
  auto plan_result = move_group_->plan(my_plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(),
                 "%s(%s): Planning failed for target '%s' Code: %d", __func__,
                 target_pose_frame, target_pose_frame, plan_result.val);
    return false;
  }

  RCLCPP_INFO(get_logger(), "%s(%s): Plan found for target '%s', executing...",
              __func__, target_pose_frame, target_pose_frame);

  moveit::core::MoveItErrorCode move_result;
  move_result = move_group_->execute(my_plan);
  if (move_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(),
                 "%s(%s): Execution failed for target '%s' Code: %d", __func__,
                 target_pose_frame, target_pose_frame, move_result.val);
    return false;
  }
  RCLCPP_INFO(get_logger(), "%s(%s): Successfully moved to target '%s'",
              __func__, target_pose_frame, target_pose_frame);

  return true;
}

bool TowerOfHanoi::sendGripperCommand(double pos) {
  if (!gripper_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_logger(), "%s(%f): Gripper action server not available.",
                 __func__, pos);
    return false;
  }

  ParallelGripperCommand::Goal goal;
  goal.command.position = {pos};

  rclcpp_action::Client<ParallelGripperCommand>::SendGoalOptions options;

  // Feedback
  options.feedback_callback = [this](auto /*goal_handle*/,
                                     const auto feedback) {
    const auto &state = feedback->state;
    for (size_t i = 0; i < state.name.size(); ++i) {
      const double pos = (i < state.position.size()) ? state.position[i] : NAN;
      const double vel = (i < state.velocity.size()) ? state.velocity[i] : NAN;
      const double eff = (i < state.effort.size()) ? state.effort[i] : NAN;

      RCLCPP_INFO(get_logger(),
                  "%s(%f): Joint '%s': { pos=%f, vel=%f, eff=%f }", __func__,
                  pos, state.name[i].c_str(), pos, vel, eff);
    }
  };

  // Send goal
  auto goal_handle_future = gripper_client_->async_send_goal(goal, options);

  auto goal_accepted = goal_handle_future.wait_for(std::chrono::seconds(5));
  if (goal_accepted != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "%s(%f): Sending of gripper goal timed out.",
                 __func__, pos);
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "%s(%f): Gripper goal was rejected.", __func__,
                 pos);
    return false; // goal was rejected
  }

  // Get result and wait until gripper finishes
  auto result_future = gripper_client_->async_get_result(goal_handle);
  auto result_accepted = result_future.wait_for(std::chrono::seconds(10));
  if (result_accepted != std::future_status::ready) {
    gripper_client_->async_cancel_goal(goal_handle);
    RCLCPP_ERROR(get_logger(),
                 "%s(%f): Failed to get gripper result, cancel goal.", __func__,
                 pos);
    return false;
  }

  // check teh result code and handle accordingly
  auto wrapped_result = result_future.get();
  switch (wrapped_result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(get_logger(), "G%s(%f): ripper action succeeded.", __func__,
                pos);
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(get_logger(), "%s(%f): Gripper action aborted.", __func__,
                 pos);
    return false;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(get_logger(), "%s(%f): Gripper action canceled.", __func__,
                 pos);
    return false;
  default:
    RCLCPP_ERROR(get_logger(), "%s(%f): Unknown result code.", __func__, pos);
    return false;
  }

  // get the actual result message
  auto result = wrapped_result.result.get();
  // we should also handle stalling or if it reached
  return result->reached_goal || result->stalled;
}

} // namespace tower_of_hanoi

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec(
      rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));

  rclcpp::NodeOptions tf_mng_options = rclcpp::NodeOptions{};
  tf_mng_options.arguments(
      {"--ros-args", "-r", "__node:=tower_of_hanoi_tf_manager_private"});
  auto transform_manager_node =
      std::make_shared<transform_manager::TransformManager>(tf_mng_options);
  exec.add_node(transform_manager_node);
  auto tower_of_hanoi_node = std::make_shared<tower_of_hanoi::TowerOfHanoi>(
      transform_manager_node, "world");
  exec.add_node(tower_of_hanoi_node);
  auto spin_thread = std::make_unique<std::thread>([&exec]() { exec.spin(); });

  tower_of_hanoi_node->configureMoveit("manipulator_robotiq_85");
  tower_of_hanoi_node->buildTowerOfHanoi();
  // we are finished so we shutdown and wait for the spin_thread to shutdown
  rclcpp::shutdown();
  spin_thread->join();
  return 0;
}
