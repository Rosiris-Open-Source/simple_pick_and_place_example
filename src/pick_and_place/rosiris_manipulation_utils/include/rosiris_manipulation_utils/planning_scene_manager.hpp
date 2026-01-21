#ifndef CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
#define CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_

#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

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
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_publisher_;

  rclcpp::Service<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr add_srv_;
  rclcpp::Service<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr move_srv_;
  rclcpp::Service<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr remove_srv_;
  rclcpp::Service<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr allow_collision_srv_;

  void addBox(
    const std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Request> req,
    std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Response> res);
  void removeBox(
    const std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Request> req,
    std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Response> res);
  bool publishCollisionObject(const moveit_msgs::msg::CollisionObject & collision_object);
};
}  // namespace rosiris_manipulation_utils

#endif  // CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_