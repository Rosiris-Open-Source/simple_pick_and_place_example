#ifndef CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
#define CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

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
    const std::vector<std::string> & object_ids, bool allow_self_collision = false);

  /**
   * @brief Remove objects from ACM
   */
  std::vector<std::string> removeObjectsFromACM(const std::vector<std::string> & object_ids);

  /**
   * @brief Validate objects exist in planning scene
   */
  void validateObjectsExist(
    const std::vector<std::string> & object_ids, std::vector<std::string> & existing,
    std::vector<std::string> & missing);

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

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

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
};
}  // namespace rosiris_manipulation_utils

#endif  // CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_