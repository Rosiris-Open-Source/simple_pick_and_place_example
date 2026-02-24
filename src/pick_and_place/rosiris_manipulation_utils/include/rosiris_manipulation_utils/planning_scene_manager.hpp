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

#ifndef CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
#define CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

#include <rosiris_manipulation_interfaces/msg/collision_object.hpp>
// services
#include <rosiris_manipulation_interfaces/srv/add_collision_objects.hpp>
#include <rosiris_manipulation_interfaces/srv/attach_collision_object.hpp>
#include <rosiris_manipulation_interfaces/srv/detach_collision_object.hpp>
#include <rosiris_manipulation_interfaces/srv/get_attached_collision_object_ids.hpp>
#include <rosiris_manipulation_interfaces/srv/get_collision_object_ids.hpp>
#include <rosiris_manipulation_interfaces/srv/move_collision_objects.hpp>
#include <rosiris_manipulation_interfaces/srv/remove_collision_objects.hpp>
#include <rosiris_manipulation_interfaces/srv/update_allowed_collisions.hpp>
namespace rosiris_manipulation_utils
{
class PlanningSceneManager : public rclcpp::Node
{
public:
  PlanningSceneManager(
    std::string node_name = "planning_scene_manager",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~PlanningSceneManager() = default;

  /**
   * @brief start the planning scene manager
   */
  bool initialize();

private:
  /**
   * @brief start the service servers
   */
  void initializeServices();

  /**
   * @brief Apply collision matrix updates to ACM
   */
  std::tuple<std::vector<std::string>, std::vector<std::string>> applyCollisionMatrixUpdates(
    const std::vector<rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate> & updates);

  /**
   * @brief Apply single collision matrix update
   */
  bool applyCollisionMatrixUpdate(
    collision_detection::AllowedCollisionMatrix & acm,
    const rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate & update);
  /**
   * @brief Initialize ACM entries for new objects
   */
  void initializeObjectsInACM(
    const std::vector<rosiris_manipulation_interfaces::msg::CollisionObject> & objs);

  /**
   * @brief Remove objects from ACM
   */
  std::vector<std::string> removeObjectsFromACM(
    const std::vector<std::string> & object_ids, moveit_msgs::msg::PlanningScene & update);

  /**
   * @brief Validate objects exist in planning scene
   */
  void validateObjectsExist(
    const std::vector<std::string> & object_ids, std::vector<std::string> & existing,
    std::vector<std::string> & missing);

  rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate
  convert_touch_links_to_collision_matrix_update(
    const std::string & target_link, const std::vector<std::string> & touch_links, uint8_t mode,
    bool allow_touch);

  void addCollisionObjects(
    const std::shared_ptr<rosiris_manipulation_interfaces::srv::AddCollisionObjects::Request>
      request,
    std::shared_ptr<rosiris_manipulation_interfaces::srv::AddCollisionObjects::Response> response);

  void removeCollisionObjects(
    const std::shared_ptr<rosiris_manipulation_interfaces::srv::RemoveCollisionObjects::Request>
      request,
    std::shared_ptr<rosiris_manipulation_interfaces::srv::RemoveCollisionObjects::Response>
      response);

  void attachCollisionObject(
    const std::shared_ptr<rosiris_manipulation_interfaces::srv::AttachCollisionObject::Request>
      request,
    std::shared_ptr<rosiris_manipulation_interfaces::srv::AttachCollisionObject::Response>
      response);

  void detachCollisionObject(
    const std::shared_ptr<rosiris_manipulation_interfaces::srv::DetachCollisionObject::Request>
      request,
    std::shared_ptr<rosiris_manipulation_interfaces::srv::DetachCollisionObject::Response>
      response);

  void moveCollisionObjects(
    const std::shared_ptr<rosiris_manipulation_interfaces::srv::MoveCollisionObjects::Request>
      request,
    std::shared_ptr<rosiris_manipulation_interfaces::srv::MoveCollisionObjects::Response> response);

  void updateAllowedCollisions(
    const std::shared_ptr<rosiris_manipulation_interfaces::srv::UpdateAllowedCollisions::Request>
      request,
    std::shared_ptr<rosiris_manipulation_interfaces::srv::UpdateAllowedCollisions::Response>
      response);

  void getAttachedCollisionObjectIds(
    const std::shared_ptr<
      rosiris_manipulation_interfaces::srv::GetAttachedCollisionObjectIds::Request>
      _,
    std::shared_ptr<rosiris_manipulation_interfaces::srv::GetAttachedCollisionObjectIds::Response>
      response);

  void getCollisionObjectIds(
    const std::shared_ptr<rosiris_manipulation_interfaces::srv::GetCollisionObjectIds::Request> _,
    std::shared_ptr<rosiris_manipulation_interfaces::srv::GetCollisionObjectIds::Response>
      response);

  rosiris_manipulation_interfaces::msg::ServiceResult triggerMoveGroupSceneUpdate(bool is_diff);
  rosiris_manipulation_interfaces::msg::ServiceResult triggerMoveGroupSceneUpdate(
    moveit_msgs::msg::PlanningScene update);

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  rclcpp::Service<rosiris_manipulation_interfaces::srv::AddCollisionObjects>::SharedPtr
    add_collision_objects_srv_;
  rclcpp::Service<rosiris_manipulation_interfaces::srv::RemoveCollisionObjects>::SharedPtr
    remove_collision_objects_srv_;
  rclcpp::Service<rosiris_manipulation_interfaces::srv::MoveCollisionObjects>::SharedPtr
    move_collision_objects_srv_;
  rclcpp::Service<rosiris_manipulation_interfaces::srv::AttachCollisionObject>::SharedPtr
    attach_collision_object_srv_;
  rclcpp::Service<rosiris_manipulation_interfaces::srv::DetachCollisionObject>::SharedPtr
    detach_collision_object_srv_;
  rclcpp::Service<rosiris_manipulation_interfaces::srv::UpdateAllowedCollisions>::SharedPtr
    update_allowed_collisions_srv_;
  rclcpp::Service<rosiris_manipulation_interfaces::srv::GetAttachedCollisionObjectIds>::SharedPtr
    get_attached_collision_obj_ids_srv_;
  rclcpp::Service<rosiris_manipulation_interfaces::srv::GetCollisionObjectIds>::SharedPtr
    get_active_collision_obj_ids_srv_;
};
}  // namespace rosiris_manipulation_utils

#endif  // CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_