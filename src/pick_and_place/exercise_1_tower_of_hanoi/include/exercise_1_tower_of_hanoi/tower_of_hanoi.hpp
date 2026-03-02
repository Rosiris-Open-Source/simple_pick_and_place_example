// Copyright 2026 Manuel Muth
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EXERCISE_1_TOWER_OF_HANOI__TOWER_OF_HANOI_HPP_
#define EXERCISE_1_TOWER_OF_HANOI__TOWER_OF_HANOI_HPP_

#include <cmath>
#include <iostream>
#include <string>

#include <control_msgs/action/parallel_gripper_command.hpp>
#include <exercise_1_tower_of_hanoi/tower_of_hanoi_parameters.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rosiris_manipulation_interfaces/msg/collision_matrix_update.hpp>
#include <rosiris_manipulation_interfaces/srv/attach_collision_object.hpp>
#include <rosiris_manipulation_interfaces/srv/detach_collision_object.hpp>
#include <rosiris_manipulation_interfaces/srv/update_allowed_collisions.hpp>

// TransforManager
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
namespace tower_of_hanoi {
class TransformManager : public rclcpp::Node {
public:
  TransformManager(std::string node_name = "transform_manager",
                   const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node(node_name, options), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  }

  geometry_msgs::msg::PoseStamped
  getPoseInFrame(const geometry_msgs::msg::PoseStamped &pose,
                 const std::string &target_frame) {
    return tf_buffer_.transform(pose, target_frame, tf2::durationFromSec(0.0));
  }

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

struct Cube {
  Cube(const std::string &move_group, const std::string &id,
       const std::string &grasp_point, double dimension,
       double grasp_pose_offset)
      : move_group_(move_group), id_(id), grasp_point_(grasp_point),
        dimension_(dimension), grasp_pose_offset_(grasp_pose_offset) {}

  std::string id() const { return id_; }
  std::string grasp_point() const {
    return move_group_ + "/" + id_ + "/" + grasp_point_;
  }
  std::string ref_frame() const { return move_group_ + "/" + id_; }
  double dimension() const { return dimension_; }
  double grasp_pose_offset() const { return grasp_pose_offset_; }

private:
  std::string move_group_;
  std::string id_;
  std::string grasp_point_;
  double dimension_;
  double grasp_pose_offset_;
};

enum class Location { LEFT, MIDDLE, RIGHT };

class TowerOfHanoi : public rclcpp::Node {
public:
  TowerOfHanoi(std::shared_ptr<TransformManager> transform_manager,
               std::string reference_frame = "world",
               std::string node_name = "hanoi_grasp_node",
               const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~TowerOfHanoi() = default;

  void configureMoveit(std::string move_group_name) {
    move_group_ =
        std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(), move_group_name);
    visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(
        shared_from_this(),
        reference_frame_, // reference frame
        rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_->getRobotModel());
    visual_tools_->setLifetime(0.0);
  }

  void buildTowerOfHanoi();

protected:
  void getParameters();

  void setupServiceClients();
  void setupActionClients();

  void configurePlannerOmpl(
      moveit::planning_interface::MoveGroupInterface &move_group);
  void configurePlannerPilzLin(
      moveit::planning_interface::MoveGroupInterface &move_group);

  bool homeRobot();
  bool pickCube(const Cube &cube);
  bool placeCubeAtLocation(const Cube &cube_to_place, const Location &location);
  bool placeCubeOnCube(const Cube &cube_to_place, const Cube &target_cube);
  bool placeCube(const Cube &cube_to_place,
                 const geometry_msgs::msg::PoseStamped &place_pose,
                 const geometry_msgs::msg::PoseStamped &approach_place_pose,
                 const geometry_msgs::msg::PoseStamped &retreat_place_pose,
                 const std::vector<std::string> &allow_collisions);

  bool moveLocationOmpl(const Location &location, double distance = 0.0);
  bool moveLocationLin(const Location &location, double distance = 0.0);
  bool moveOmpl(const geometry_msgs::msg::PoseStamped &target_pose);
  bool moveLin(const geometry_msgs::msg::PoseStamped &target_pose);
  bool planAndMove(const geometry_msgs::msg::PoseStamped &target_pose);

  bool openGripper() { return sendGripperCommand(0.01); }
  bool closeGripper() { return sendGripperCommand(0.75); }
  bool sendGripperCommand(double pos);

  bool attachObjectToGripper(const std::string &obj_to_attach);
  bool detachObjectFromGripper(const std::string &obj_to_detach);
  bool updateAllowedCollision(
      const Cube &cube, const std::vector<std::string> &allow_collisions,
      const std::vector<std::string> &disallow_collisions,
      rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate::_mode_type
          mode);

  void visualizePose(const geometry_msgs::msg::PoseStamped &pose,
                     const std::string &text) {
    if (!visual_tools_) {
      RCLCPP_ERROR(this->get_logger(), "Visual tools not initialized.");
      return;
    }
    if (!transform_manager_) {
      RCLCPP_ERROR(this->get_logger(), "Transform manager not initialized.");
      return;
    }
    auto pose_in_ref =
        transform_manager_->getPoseInFrame(pose, reference_frame_);
    visual_tools_->publishAxisLabeled(pose_in_ref.pose, text);
    visual_tools_->trigger();
  }

  void removeVisualizedPoses() {
    if (!visual_tools_) {
      RCLCPP_ERROR(this->get_logger(), "Visual tools not initialized.");
      return;
    }
    visual_tools_->deleteAllMarkers();
  }

  inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

  rclcpp_action::Client<control_msgs::action::ParallelGripperCommand>::SharedPtr
      gripper_client_;
  rclcpp::Client<rosiris_manipulation_interfaces::srv::AttachCollisionObject>::
      SharedPtr robot_attach_cli_;
  rclcpp::Client<rosiris_manipulation_interfaces::srv::DetachCollisionObject>::
      SharedPtr robot_detach_cli_;
  rclcpp::Client<
      rosiris_manipulation_interfaces::srv::UpdateAllowedCollisions>::SharedPtr
      scene_upd_collision_srv_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ =
      nullptr;
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_ =
      nullptr;
  std::shared_ptr<TransformManager> transform_manager_;
  // parameters
  std::unique_ptr<tower_of_hanoi::ParamListener> param_listener_;
  tower_of_hanoi::Params params_;

private:
  std::string reference_frame_;
  const std::unordered_map<std::string, Cube> cubes_{
      {"cube_1", {"move_group", "cube_1", "grasp_point", 0.06, 0.015}},
      {"cube_2", {"move_group", "cube_2", "grasp_point", 0.05, 0.015}},
      {"cube_3", {"move_group", "cube_3", "grasp_point", 0.04, 0.015}}};

  const std::unordered_map<Location, std::string> place_locations_{
      {Location::LEFT, "desk_1_place_location_left"},
      {Location::MIDDLE, "desk_1_place_location_middle"},
      {Location::RIGHT, "desk_1_place_location_right"}};
  const double pre_grasp_distance_ = 0.1;
  const double post_grasp_distance_ = 0.1;
  const double pre_place_distance_ = 0.1;
  const double post_place_distance_ = 0.1;
  const tf2::Vector3 pre_grasp_direction_ =
      tf2::Vector3(0.0, -1.0, -1.0).normalize();
  const tf2::Vector3 post_grasp_direction_ =
      tf2::Vector3(0.0, 0.0, -1.0).normalize();
  const tf2::Vector3 pre_place_cube_direction_ =
      tf2::Vector3(0.0, 0.0, 1.0).normalize();
  const tf2::Vector3 post_place_cube_direction_ =
      tf2::Vector3(0.0, 0.0, 1.0).normalize();
  const tf2::Vector3 pre_place_location_direction_ =
      tf2::Vector3(0.0, 0.0, 1.0).normalize();
  const tf2::Vector3 post_place_location_direction_ =
      tf2::Vector3(0.0, 0.0, 1.0).normalize();
  // r: 180, y: 90 like in tower_of_hanoi_scenario.yaml
  const tf2::Quaternion rot_place_to_cube_{0.7071068, 0.7071068, 0, 0};
};

} // namespace tower_of_hanoi

#endif // EXERCISE_1_TOWER_OF_HANOI__TOWER_OF_HANOI_HPP_