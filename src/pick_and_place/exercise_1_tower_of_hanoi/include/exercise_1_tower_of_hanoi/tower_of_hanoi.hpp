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

#include <iostream>
#include <string>

#include <control_msgs/action/parallel_gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rosiris_manipulation_interfaces/srv/attach_collision_object.hpp>
#include <rosiris_manipulation_interfaces/srv/detach_collision_object.hpp>

#include "exercise_1_tower_of_hanoi/tower_of_hanoi_parameters.hpp"

namespace tower_of_hanoi {

class TowerOfHanoi : public rclcpp::Node {
public:
  TowerOfHanoi(std::string node_name = "hanoi_grasp_node",
                    const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~TowerOfHanoi() = default;

  void buildTowerOfHanoi();

protected:
  void declareAndGetParameters();

  void setupServiceClients();

  void setupActionClients();

  void configurePlannerOmpl(
      moveit::planning_interface::MoveGroupInterface &move_group);

  void configurePlannerPilzLin(
      moveit::planning_interface::MoveGroupInterface &move_group);

  bool attachObjectToGripper(const std::string &obj_to_attach,
                             std::vector<std::string> allowed_touch_links);

  bool detachObjectFromGripper(const std::string &obj_to_detach,
                               std::vector<std::string> disallowed_touch_links);

  void homeRobot();

  void openGripper() { sendGripperCommand(0.0, 0.1); }

  void closeGripper() { sendGripperCommand(0.8, 0.5); }

  void sendGripperCommand(double pos, double effort);

  rclcpp_action::Client<control_msgs::action::ParallelGripperCommand>::SharedPtr
      gripper_client_;
  rclcpp::Client<rosiris_manipulation_interfaces::srv::AttachCollisionObject>::
      SharedPtr robot_attach_cli_;
  rclcpp::Client<rosiris_manipulation_interfaces::srv::DetachCollisionObject>::
      SharedPtr robot_detach_cli_;

  // parameters
  std::shared_ptr<tower_of_hanoi::ParamListener> param_listener_;
  tower_of_hanoi::Params params_;
  std::string eef_link_;
  int64_t srv_call_timeout_;
};

} // namespace tower_of_hanoi

#endif // EXERCISE_1_TOWER_OF_HANOI__TOWER_OF_HANOI_HPP_