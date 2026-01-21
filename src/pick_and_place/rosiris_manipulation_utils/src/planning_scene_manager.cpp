#include "rosiris_manipulation_utils/planning_scene_manager.hpp"

// messages
#include <rosiris_manipulation_interfaces/msg/collision_entry.hpp>
#include <rosiris_manipulation_interfaces/msg/collision_object.hpp>
#include <rosiris_manipulation_interfaces/msg/service_result.hpp>
#include <rosiris_manipulation_interfaces/msg/service_return_state.hpp>
namespace rosiris_manipulation_utils
{

PlanningSceneManager::PlanningSceneManager(
  std::string node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  initializeServices();

  RCLCPP_INFO(this->get_logger(), "initialization complete. Ready to receive requests.");
}

PlanningSceneManager::PlanningSceneManager(const rclcpp::NodeOptions & options)
: Node("planning_scene_manager", options)
{
  initializeServices();
  RCLCPP_INFO(this->get_logger(), "initialization complete. Ready to receive requests.");
}

PlanningSceneManager::~PlanningSceneManager() {}

void PlanningSceneManager::initializeServices()
{
  get_scene_client_ =
    this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

  // Wait for planning scene service to be available
  while (!get_scene_client_->wait_for_service())
  {
    RCLCPP_INFO(this->get_logger(), "/get_planning_scene service not available, waiting again...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  apply_scene_update_client_ =
    this->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");

  // Wait for planning scene service to be available
  while (!apply_scene_update_client_->wait_for_service())
  {
    RCLCPP_INFO(
      this->get_logger(), "/apply_planning_scene service not available, waiting again...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  using std::placeholders::_1;
  using std::placeholders::_2;

  add_collision_objects_srv_ =
    create_service<rosiris_manipulation_interfaces::srv::AddCollisionObjects>(
      "planning_scene_manager/add_collision_objects",
      std::bind(&PlanningSceneManager::addCollisionObjects, this, _1, _2));

  remove_collision_objects_srv_ =
    create_service<rosiris_manipulation_interfaces::srv::RemoveCollisionObjects>(
      "planning_scene_manager/remove_collision_objects",
      std::bind(&PlanningSceneManager::removeCollisionObjects, this, _1, _2));

  move_collision_objects_srv_ =
    create_service<rosiris_manipulation_interfaces::srv::MoveCollisionObjects>(
      "planning_scene_manager/move_collision_objects",
      std::bind(&PlanningSceneManager::moveCollisionObjects, this, _1, _2));

  attach_collision_object_srv_ =
    create_service<rosiris_manipulation_interfaces::srv::AttachCollisionObject>(
      "planning_scene_manager/attach_collision_object",
      std::bind(&PlanningSceneManager::attachCollisionObject, this, _1, _2));

  detach_collision_object_srv_ =
    create_service<rosiris_manipulation_interfaces::srv::DetachCollisionObject>(
      "planning_scene_manager/detach_collision_object",
      std::bind(&PlanningSceneManager::detachCollisionObject, this, _1, _2));

  update_allowed_collisions_srv_ =
    create_service<rosiris_manipulation_interfaces::srv::UpdateAllowedCollisions>(
      "planning_scene_manager/update_allowed_collisions",
      std::bind(&PlanningSceneManager::updateAllowedCollisions, this, _1, _2));
}

void PlanningSceneManager::addCollisionObjects(
  const std::shared_ptr<rosiris_manipulation_interfaces::srv::AddCollisionObjects::Request> req,
  std::shared_ptr<rosiris_manipulation_interfaces::srv::AddCollisionObjects::Response> res)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;

  for (const auto & obj : req->collision_objects)
  {
    auto co = obj.collision_object;
    co.operation = moveit_msgs::msg::CollisionObject::ADD;
    scene.world.collision_objects.push_back(co);

    res->added_object_ids.push_back(co.id);
  }

  if (!apply_planning_scene(scene))
  {
    res->result.state = rosiris_manipulation_interfaces::msg::ServiceReturnState::ERROR;
    res->result.msg = "Failed to apply planning scene";
    res->not_added_object_ids = res->added_object_ids;
    res->added_object_ids.clear();
    return;
  }

  res->result.state = rosiris_manipulation_interfaces::msg::ServiceReturnState::SUCCESS;
}

void PlanningSceneManager::removeCollisionObjects(
  const std::shared_ptr<rosiris_manipulation_interfaces::srv::RemoveCollisionObjects::Request> req,
  std::shared_ptr<rosiris_manipulation_interfaces::srv::RemoveCollisionObjects::Response> res)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;

  for (const auto & id : req->object_ids)
  {
    moveit_msgs::msg::CollisionObject co;
    co.id = id;
    co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene.world.collision_objects.push_back(co);
    res->removed_object_ids.push_back(id);
  }

  if (!apply_planning_scene(scene))
  {
    res->result.state = rosiris_manipulation_interfaces::msg::ServiceReturnState::ERROR;
    res->result.msg = "Failed removing collision objects";
    res->not_removed_object_ids = res->removed_object_ids;
    res->removed_object_ids.clear();
    return;
  }

  res->result.state = rosiris_manipulation_interfaces::msg::ServiceReturnState::SUCCESS;
}

void PlanningSceneManager::moveCollisionObjects(
  const std::shared_ptr<rosiris_manipulation_interfaces::srv::MoveCollisionObjects::Request> req,
  std::shared_ptr<rosiris_manipulation_interfaces::srv::MoveCollisionObjects::Response> res)
{
  if (req->object_ids.size() != req->poses.size())
  {
    res->result.state = rosiris_manipulation_interfaces::msg::ServiceReturnState::ERROR;
    res->result.msg = "object_id and pose array sizes differ";
    return;
  }

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;

  for (size_t i = 0; i < req->object_ids.size(); ++i)
  {
    moveit_msgs::msg::CollisionObject co;
    co.id = req->object_ids[i];
    co.header = req->poses[i].header;
    co.pose = req->poses[i].pose;
    co.operation = moveit_msgs::msg::CollisionObject::MOVE;

    scene.world.collision_objects.push_back(co);
    res->moved_object_ids.push_back(co.id);
  }

  if (!apply_planning_scene(scene))
  {
    res->result.state = rosiris_manipulation_interfaces::msg::ServiceReturnState::ERROR;
    res->result.msg = "Failed moving collision objects";
    res->not_moved_object_ids = res->moved_object_ids;
    res->moved_object_ids.clear();
    return;
  }

  res->result.state = rosiris_manipulation_interfaces::msg::ServiceReturnState::SUCCESS;
}

void PlanningSceneManager::attachCollisionObject(
  const std::shared_ptr<rosiris_manipulation_interfaces::srv::AttachCollisionObject::Request> req,
  std::shared_ptr<rosiris_manipulation_interfaces::srv::AttachCollisionObject::Response> res)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;

  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.object.id = req->collision_object_id;
  aco.link_name = req->attach_to_link;
  aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  scene.robot_state.attached_collision_objects.push_back(aco);

  for (const auto & c : req->collision_entries)
  {
    scene.allowed_collision_matrix.entry_names.push_back(c.object_id);
    moveit_msgs::msg::AllowedCollisionEntry entry;
    entry.enabled.push_back(c.collision_allowed);
    scene.allowed_collision_matrix.entry_values.push_back(entry);
  }

  if (!apply_planning_scene(scene))
  {
    res->result.state = rosiris_manipulation_interfaces::msg::ServiceReturnState::ERROR;
    res->result.msg = "Failed attaching collision object";
    return;
  }

  res->result.state = rosiris_manipulation_interfaces::msg::ServiceReturnState::SUCCESS;
}

void PlanningSceneManager::detachCollisionObject(
  const std::shared_ptr<rosiris_manipulation_interfaces::srv::DetachCollisionObject::Request> req,
  std::shared_ptr<rosiris_manipulation_interfaces::srv::DetachCollisionObject::Response> res)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;

  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.object.id = req->collision_object_id;
  aco.link_name = req->detach_to_link;
  aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  scene.robot_state.attached_collision_objects.push_back(aco);

  if (!apply_planning_scene(scene))
  {
    res->result.state = rosiris_manipulation_interfaces::msg::ServiceReturnState::ERROR;
    res->result.msg = "Failed detaching collision object";
    return;
  }

  res->result.state = rosiris_manipulation_interfaces::msg::ServiceReturnState::SUCCESS;
}
void PlanningSceneManager::updateAllowedCollisions(
  const std::shared_ptr<rosiris_manipulation_interfaces::srv::UpdateAllowedCollisions::Request> req,
  std::shared_ptr<rosiris_manipulation_interfaces::srv::UpdateAllowedCollisions::Response> res)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;

  for (size_t i = 0; i < req->object_ids.size(); ++i)
  {
    scene.allowed_collision_matrix.entry_names.push_back(req->object_ids[i]);
    moveit_msgs::msg::AllowedCollisionEntry entry;
    entry.enabled.push_back(req->collision_entries[i].collision_allowed);
    scene.allowed_collision_matrix.entry_values.push_back(entry);

    res->updated_object_ids.push_back(req->object_ids[i]);
  }

  if (!apply_planning_scene(scene))
  {
    res->result.state = rosiris_manipulation_interfaces::msg::ServiceReturnState::ERROR;
    res->result.msg = "Failed updating ACM";
    res->not_updated_object_ids = res->updated_object_ids;
    res->updated_object_ids.clear();
    return;
  }

  res->result.state = rosiris_manipulation_interfaces::msg::ServiceReturnState::SUCCESS;
}

bool PlanningSceneManager::apply_planning_scene(
  const moveit_msgs::msg::PlanningScene & planning_scene)
{
  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene = planning_scene;
  auto future_result = apply_scene_update_client_->async_send_request(request);
  auto future_status = future_result.wait_for(std::chrono::seconds(5));
  if (future_status != std::future_status::ready)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to apply planning scene");
    return false;
  }
  auto response = future_result.get();
  return response->success;
}
}  // namespace rosiris_manipulation_utils

// register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(rosiris_manipulation_utils::PlanningSceneManager)