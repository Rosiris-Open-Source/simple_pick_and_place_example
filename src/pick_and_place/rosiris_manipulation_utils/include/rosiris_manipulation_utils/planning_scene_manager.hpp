#ifndef CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
#define CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_

#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

// services
#include <rosiris_manipulation_interfaces/srv/add_collision_objects.hpp>
#include <rosiris_manipulation_interfaces/srv/attach_collision_object.hpp>
#include <rosiris_manipulation_interfaces/srv/detach_collision_object.hpp>
#include <rosiris_manipulation_interfaces/srv/move_collision_objects.hpp>
#include <rosiris_manipulation_interfaces/srv/remove_collision_objects.hpp>
#include <rosiris_manipulation_interfaces/srv/update_allowed_collisions.hpp>

namespace rosiris_manipulation_utils
{
class PlanningSceneManager : public rclcpp::Node
{
public:
  PlanningSceneManager(
    std::string node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // needed for rclcpp_components
  explicit PlanningSceneManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~PlanningSceneManager();

protected:
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_update_client_;

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

  void initializeServices();

  void addCollisionObjects(
    const std::shared_ptr<rosiris_manipulation_interfaces::srv::AddCollisionObjects::Request> req,
    std::shared_ptr<rosiris_manipulation_interfaces::srv::AddCollisionObjects::Response> res);

  void removeCollisionObjects(
    const std::shared_ptr<rosiris_manipulation_interfaces::srv::RemoveCollisionObjects::Request>
      req,
    std::shared_ptr<rosiris_manipulation_interfaces::srv::RemoveCollisionObjects::Response> res);

  void moveCollisionObjects(
    const std::shared_ptr<rosiris_manipulation_interfaces::srv::MoveCollisionObjects::Request> req,
    std::shared_ptr<rosiris_manipulation_interfaces::srv::MoveCollisionObjects::Response> res);

  void attachCollisionObject(
    const std::shared_ptr<rosiris_manipulation_interfaces::srv::AttachCollisionObject::Request> req,
    std::shared_ptr<rosiris_manipulation_interfaces::srv::AttachCollisionObject::Response> res);

  void detachCollisionObject(
    const std::shared_ptr<rosiris_manipulation_interfaces::srv::DetachCollisionObject::Request> req,
    std::shared_ptr<rosiris_manipulation_interfaces::srv::DetachCollisionObject::Response> res);

  void updateAllowedCollisions(
    const std::shared_ptr<rosiris_manipulation_interfaces::srv::UpdateAllowedCollisions::Request>
      req,
    std::shared_ptr<rosiris_manipulation_interfaces::srv::UpdateAllowedCollisions::Response> res);

  bool apply_planning_scene(const moveit_msgs::msg::PlanningScene & planning_scene);
};
}  // namespace rosiris_manipulation_utils

#endif  // CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_