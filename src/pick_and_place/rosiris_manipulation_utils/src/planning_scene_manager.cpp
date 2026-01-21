#include "rosiris_manipulation_utils/planning_scene_manager.hpp"

namespace rosiris_manipulation_utils
{

PlanningSceneManager::PlanningSceneManager(
  std::string node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  get_scene_client_ =
    this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

  scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

  add_srv_ = create_service<moveit_msgs::srv::ApplyPlanningScene>(
    "planning_scene_manager/add_box",
    std::bind(&PlanningSceneManager::addBox, this, std::placeholders::_1, std::placeholders::_2));

  remove_srv_ = create_service<moveit_msgs::srv::ApplyPlanningScene>(
    "planning_scene_manager/remove_box",
    std::bind(
      &PlanningSceneManager::removeBox, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "PlanningScene service component started");
}

PlanningSceneManager::PlanningSceneManager(const rclcpp::NodeOptions & options)
: Node("planning_scene_manager", options)
{
  get_scene_client_ =
    this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

  scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

  add_srv_ = create_service<moveit_msgs::srv::ApplyPlanningScene>(
    "planning_scene_manager/add_box",
    std::bind(&PlanningSceneManager::addBox, this, std::placeholders::_1, std::placeholders::_2));

  remove_srv_ = create_service<moveit_msgs::srv::ApplyPlanningScene>(
    "planning_scene_manager/remove_box",
    std::bind(
      &PlanningSceneManager::removeBox, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "PlanningScene service component started");
}

PlanningSceneManager::~PlanningSceneManager() {}

void PlanningSceneManager::addBox(
  const std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Request> req,
  std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Response> res)
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

void PlanningSceneManager::removeBox(
  const std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Request> req,
  std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Response> res)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = "box1";
  obj.header.frame_id = "world";
  obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  res->success = publishCollisionObject(obj);
}

bool PlanningSceneManager::publishCollisionObject(
  const moveit_msgs::msg::CollisionObject & collision_object)
{
  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  planning_scene.world.collision_objects.push_back(collision_object);

  scene_publisher_->publish(planning_scene);

  RCLCPP_INFO(this->get_logger(), "Published planning scene diff: %s", collision_object.id.c_str());

  return true;
}
}  // namespace rosiris_manipulation_utils

// register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(rosiris_manipulation_utils::PlanningSceneManager)