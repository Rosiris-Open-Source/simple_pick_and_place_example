#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

using moveit_msgs::srv::ApplyPlanningScene;
namespace planning_scene_service_node
{
class PlanningSceneServiceNode : public rclcpp::Node
{
public:
  explicit PlanningSceneServiceNode(const rclcpp::NodeOptions& options)
  : Node("planning_scene_service_node", options)
  {

    get_scene_client_ = this->create_client<moveit_msgs::srv::GetPlanningScene>(
        "/get_planning_scene");

    scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
      "/planning_scene", 10);

    add_srv_ = create_service<ApplyPlanningScene>(
      "scene/add_box",
      std::bind(&PlanningSceneServiceNode::addBox, this, std::placeholders::_1, std::placeholders::_2));

    remove_srv_ = create_service<ApplyPlanningScene>(
      "scene/remove_box",
      std::bind(&PlanningSceneServiceNode::removeBox, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "PlanningScene service component started");
  }

private:
rclcpp::TimerBase::SharedPtr init_timer_;

  planning_scene_monitor::PlanningSceneMonitorPtr psm_;

  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_publisher_;

  rclcpp::Service<ApplyPlanningScene>::SharedPtr add_srv_;
  rclcpp::Service<ApplyPlanningScene>::SharedPtr move_srv_;
  rclcpp::Service<ApplyPlanningScene>::SharedPtr remove_srv_;
  rclcpp::Service<ApplyPlanningScene>::SharedPtr allow_collision_srv_;

  void addBox(
    const std::shared_ptr<ApplyPlanningScene::Request> req,
    std::shared_ptr<ApplyPlanningScene::Response> res)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = "box1";
    obj.header.frame_id = "world";
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.4, 0.4, 0.4};

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.6;
    pose.position.z = 0.2;

    obj.primitives = {primitive};
    obj.primitive_poses = {pose};
    res->success = publishCollisionObject(obj);
  }

  void removeBox(
    const std::shared_ptr<ApplyPlanningScene::Request> req,
    std::shared_ptr<ApplyPlanningScene::Response> res)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = "box1";
    obj.header.frame_id = "world";
    obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    res->success = publishCollisionObject(obj);
  }

  bool publishCollisionObject(const moveit_msgs::msg::CollisionObject & collision_object)
  {
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.world.collision_objects.push_back(collision_object);

    scene_publisher_->publish(planning_scene);
    
    RCLCPP_INFO(this->get_logger(), "Published planning scene diff: %s", 
                collision_object.id.c_str());

    return true;
  }
};
} // namespace planning_scene_service_node

// main for standalone execution, makes it possible to debug the node directly
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  auto node = std::make_shared<planning_scene_service_node::PlanningSceneServiceNode>(rclcpp::NodeOptions());
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);
  rclcpp::shutdown();
  return 0;
}

// register the component with class_loader 
RCLCPP_COMPONENTS_REGISTER_NODE(planning_scene_service_node::PlanningSceneServiceNode)