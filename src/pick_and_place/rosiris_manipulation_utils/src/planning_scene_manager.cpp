#include "rosiris_manipulation_utils/planning_scene_manager.hpp"

// messages
#include <rosiris_manipulation_interfaces/msg/collision_entry.hpp>
#include <rosiris_manipulation_interfaces/msg/service_error_code.hpp>
#include <rosiris_manipulation_interfaces/msg/service_result.hpp>
namespace rosiris_manipulation_utils
{

namespace rosiris_manip_srv = rosiris_manipulation_interfaces::srv;
using rosiris_srv_error_codes = rosiris_manipulation_interfaces::msg::ServiceErrorCode;
using rosiris_srv_result = rosiris_manipulation_interfaces::msg::ServiceResult;

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
  // Create service servers
  add_collision_objects_srv_ = create_service<rosiris_manip_srv::AddCollisionObjects>(
    "planning_scene_manager/add_collision_objects",
    std::bind(
      &PlanningSceneManager::addCollisionObjects, this, std::placeholders::_1,
      std::placeholders::_2));

  remove_collision_objects_srv_ = create_service<rosiris_manip_srv::RemoveCollisionObjects>(
    "planning_scene_manager/remove_collision_objects",
    std::bind(
      &PlanningSceneManager::removeCollisionObjects, this, std::placeholders::_1,
      std::placeholders::_2));

  move_collision_objects_srv_ = create_service<rosiris_manip_srv::MoveCollisionObjects>(
    "planning_scene_manager/move_collision_objects",
    std::bind(
      &PlanningSceneManager::moveCollisionObjects, this, std::placeholders::_1,
      std::placeholders::_2));

  attach_collision_object_srv_ = create_service<rosiris_manip_srv::AttachCollisionObject>(
    "planning_scene_manager/attach_collision_object",
    std::bind(
      &PlanningSceneManager::attachCollisionObject, this, std::placeholders::_1,
      std::placeholders::_2));

  detach_collision_object_srv_ = create_service<rosiris_manip_srv::DetachCollisionObject>(
    "planning_scene_manager/detach_collision_object",
    std::bind(
      &PlanningSceneManager::detachCollisionObject, this, std::placeholders::_1,
      std::placeholders::_2));

  update_allowed_collisions_srv_ = create_service<rosiris_manip_srv::UpdateAllowedCollisions>(
    "planning_scene_manager/update_allowed_collisions",
    std::bind(
      &PlanningSceneManager::updateAllowedCollisions, this, std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Planning Scene Manager started.");
}

bool PlanningSceneManager::initialize()
{
  // Initialize PlanningSceneMonitor
  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
    shared_from_this(), "robot_description");

  if (!planning_scene_monitor_->getPlanningScene())
  {
    RCLCPP_ERROR(get_logger(), "Failed to initialize planning scene monitor");
    return false;
  }

  // Start monitoring
  planning_scene_monitor_->startSceneMonitor();
  planning_scene_monitor_->startWorldGeometryMonitor();
  planning_scene_monitor_->startStateMonitor();

  // Explicitly synchronize once to get current state
  planning_scene_monitor_->requestPlanningSceneState();

  planning_scene_monitor_->startPublishingPlanningScene(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  // Optional: Continuously publish planning scene updates
  // Uncomment if you want continuous publishing:
  // planning_scene_monitor_->startPublishingPlanningScene(
  //   planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  return true;
}

/**
 * @brief Apply collision matrix updates to ACM
 */
std::vector<std::string> PlanningSceneManager::applyCollisionMatrixUpdates(
  const std::vector<rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate> & updates)
{
  std::vector<std::string> updated_links;

  if (updates.empty())
  {
    return updated_links;
  }

  planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
  if (!scene)
  {
    RCLCPP_ERROR(get_logger(), "Failed to lock planning scene");
    return updated_links;
  }

  collision_detection::AllowedCollisionMatrix & acm = scene->getAllowedCollisionMatrixNonConst();

  for (const auto & update : updates)
  {
    if (applyCollisionMatrixUpdate(acm, update))
    {
      updated_links.push_back(update.target_link);
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Failed to apply ACM update for: %s", update.target_link.c_str());
    }
  }

  return updated_links;
}

/**
 * @brief Apply single collision matrix update
 */
bool PlanningSceneManager::applyCollisionMatrixUpdate(
  collision_detection::AllowedCollisionMatrix & acm,
  const rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate & update)
{
  using CollisionMatrixUpdate = rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate;

  // Validate target_link exists in the scene
  planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
  if (!scene || !scene->knowsFrameTransform(update.target_link))
  {
    RCLCPP_WARN(get_logger(), "Target link not found in scene: %s", update.target_link.c_str());
    return false;
  }

  switch (update.mode)
  {
    case CollisionMatrixUpdate::REPLACE:
    {
      // Get all existing entries and clear them for target_link
      std::vector<std::string> all_names;
      acm.getAllEntryNames(all_names);

      for (const auto & name : all_names)
      {
        if (name != update.target_link)
        {
          acm.removeEntry(update.target_link, name);
        }
      }

      // Set new entries
      for (const auto & entry : update.collision_entries)
      {
        acm.setEntry(update.target_link, entry.touch_link, entry.collision_allowed);

        RCLCPP_DEBUG(
          get_logger(), "ACM REPLACE: %s <-> %s = %s", update.target_link.c_str(),
          entry.touch_link.c_str(), entry.collision_allowed ? "ALLOWED" : "DISALLOWED");
      }
      break;
    }

    case CollisionMatrixUpdate::MERGE:
    {
      // Update or add specified entries, keep existing ones
      for (const auto & entry : update.collision_entries)
      {
        acm.setEntry(update.target_link, entry.touch_link, entry.collision_allowed);

        RCLCPP_DEBUG(
          get_logger(), "ACM MERGE: %s <-> %s = %s", update.target_link.c_str(),
          entry.touch_link.c_str(), entry.collision_allowed ? "ALLOWED" : "DISALLOWED");
      }
      break;
    }

    case CollisionMatrixUpdate::REMOVE:
    {
      // Remove specified entries
      for (const auto & entry : update.collision_entries)
      {
        if (acm.hasEntry(update.target_link, entry.touch_link))
        {
          acm.removeEntry(update.target_link, entry.touch_link);

          RCLCPP_DEBUG(
            get_logger(), "ACM REMOVE: %s <-> %s", update.target_link.c_str(),
            entry.touch_link.c_str());
        }
      }
      break;
    }

    default:
    {
      RCLCPP_ERROR(get_logger(), "Unknown collision matrix update mode: %d", update.mode);
      return false;
    }
  }

  return true;
}

/**
 * @brief Initialize ACM entries for new objects
 */
void PlanningSceneManager::initializeObjectsInACM(
  const std::vector<std::string> & object_ids, bool allow_self_collision)
{
  if (object_ids.empty())
  {
    return;
  }

  planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
  if (!scene)
  {
    RCLCPP_ERROR(get_logger(), "Failed to lock planning scene");
    return;
  }

  collision_detection::AllowedCollisionMatrix & acm = scene->getAllowedCollisionMatrixNonConst();

  // Add each object to ACM with default: disallow collision with everything
  for (const auto & object_id : object_ids)
  {
    if (!acm.hasEntry(object_id))
    {
      acm.setDefaultEntry(object_id, false);
      RCLCPP_DEBUG(get_logger(), "Initialized ACM entry for: %s", object_id.c_str());
    }
  }

  // Optionally allow collisions between the new objects themselves
  if (allow_self_collision && object_ids.size() > 1)
  {
    for (size_t i = 0; i < object_ids.size(); ++i)
    {
      for (size_t j = i + 1; j < object_ids.size(); ++j)
      {
        acm.setEntry(object_ids[i], object_ids[j], true);
        RCLCPP_DEBUG(
          get_logger(), "Allowed self-collision: %s <-> %s", object_ids[i].c_str(),
          object_ids[j].c_str());
      }
    }
  }
}

/**
 * @brief Remove objects from ACM
 */
std::vector<std::string> PlanningSceneManager::removeObjectsFromACM(
  const std::vector<std::string> & object_ids)
{
  std::vector<std::string> removed;

  planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
  if (!scene)
  {
    RCLCPP_ERROR(get_logger(), "Failed to lock planning scene");
    return removed;
  }

  collision_detection::AllowedCollisionMatrix & acm = scene->getAllowedCollisionMatrixNonConst();

  for (const auto & object_id : object_ids)
  {
    if (acm.hasEntry(object_id))
    {
      acm.removeEntry(object_id);
      removed.push_back(object_id);
      RCLCPP_DEBUG(get_logger(), "Removed from ACM: %s", object_id.c_str());
    }
  }

  return removed;
}

/**
 * @brief Validate objects exist in planning scene
 */
void PlanningSceneManager::validateObjectsExist(
  const std::vector<std::string> & object_ids, std::vector<std::string> & existing,
  std::vector<std::string> & missing)
{
  existing.clear();
  missing.clear();

  planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
  if (!scene)
  {
    missing = object_ids;
    return;
  }

  for (const auto & object_id : object_ids)
  {
    if (scene->knowsFrameTransform(object_id))
    {
      existing.push_back(object_id);
    }
    else
    {
      missing.push_back(object_id);
    }
  }
}

void PlanningSceneManager::addCollisionObjects(
  const std::shared_ptr<rosiris_manip_srv::AddCollisionObjects::Request> request,
  std::shared_ptr<rosiris_manip_srv::AddCollisionObjects::Response> response)
{
  std::vector<std::string> added_ids;
  std::vector<std::string> not_added_ids;

  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
    if (!scene)
    {
      response->result.return_type = rosiris_srv_result::ERROR;
      response->result.error_code.error_code =
        rosiris_srv_error_codes::ERROR_LOCKING_PLANNING_SCENE;
      response->result.message = "Failed to lock planning scene";
      return;
    }

    // Add objects to planning scene
    for (const auto & obj : request->collision_objects)
    {
      if (scene->processCollisionObjectMsg(obj))
      {
        added_ids.push_back(obj.id);
        RCLCPP_INFO(get_logger(), "Added collision object: %s", obj.id.c_str());
      }
      else
      {
        not_added_ids.push_back(obj.id);
        RCLCPP_WARN(get_logger(), "Failed to add collision object: %s", obj.id.c_str());
      }
    }
  }

  // Initialize ACM entries for successfully added objects
  if (!added_ids.empty())
  {
    initializeObjectsInACM(added_ids, false);
  }

  // Apply custom collision matrix updates if provided
  if (!request->collision_matrix_update.empty())
  {
    auto updated_links = applyCollisionMatrixUpdates(request->collision_matrix_update);

    if (updated_links.size() != request->collision_matrix_update.size())
    {
      RCLCPP_WARN(get_logger(), "Some ACM updates failed");
    }
  }

  // Trigger planning scene update
  planning_scene_monitor_->triggerSceneUpdateEvent(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  response->added_object_ids = added_ids;
  response->not_added_object_ids = not_added_ids;
  response->result.return_type =
    not_added_ids.empty() ? rosiris_srv_result::SUCCESS : rosiris_srv_result::PARTIAL_SUCCESS;
  response->result.error_code.error_code =
    not_added_ids.empty() ? rosiris_srv_error_codes::NO_ERROR
                          : rosiris_srv_error_codes::ERROR_ADDING_COLLISION_OBJECT;
  response->result.message =
    not_added_ids.empty() ? "All objects added successfully"
                          : "Failed to add " + std::to_string(not_added_ids.size()) + " object(s)";
}

void PlanningSceneManager::removeCollisionObjects(
  const std::shared_ptr<rosiris_manip_srv::RemoveCollisionObjects::Request> request,
  std::shared_ptr<rosiris_manip_srv::RemoveCollisionObjects::Response> response)
{
  std::vector<std::string> removed_ids;
  std::vector<std::string> not_removed_ids;

  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
    if (!scene)
    {
      response->result.return_type = rosiris_srv_result::ERROR;
      response->result.error_code.error_code =
        rosiris_srv_error_codes::ERROR_LOCKING_PLANNING_SCENE;
      response->result.message = "Failed to lock planning scene";
      return;
    }

    // Remove from planning scene
    for (const auto & object_id : request->object_ids)
    {
      moveit_msgs::msg::CollisionObject obj;
      obj.id = object_id;
      obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;

      if (scene->processCollisionObjectMsg(obj))
      {
        removed_ids.push_back(object_id);
        RCLCPP_INFO(get_logger(), "Removed collision object: %s", object_id.c_str());
      }
      else
      {
        not_removed_ids.push_back(object_id);
        RCLCPP_WARN(get_logger(), "Failed to remove collision object: %s", object_id.c_str());
      }
    }
  }

  // Remove from ACM
  if (!removed_ids.empty())
  {
    auto acm_removed = removeObjectsFromACM(removed_ids);
    RCLCPP_DEBUG(get_logger(), "Removed %zu object(s) from ACM", acm_removed.size());
  }

  // Trigger planning scene update
  planning_scene_monitor_->triggerSceneUpdateEvent(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  response->removed_object_ids = removed_ids;
  response->not_removed_object_ids = not_removed_ids;
  response->result.return_type =
    not_removed_ids.empty() ? rosiris_srv_result::SUCCESS : rosiris_srv_result::PARTIAL_SUCCESS;
  response->result.error_code.error_code =
    not_removed_ids.empty() ? rosiris_srv_error_codes::NO_ERROR
                            : rosiris_srv_error_codes::ERROR_REMOVING_COLLISION_OBJECT;
  response->result.message =
    not_removed_ids.empty()
      ? "All objects removed successfully"
      : "Failed to remove " + std::to_string(not_removed_ids.size()) + " object(s)";
}

void PlanningSceneManager::attachCollisionObject(
  const std::shared_ptr<rosiris_manip_srv::AttachCollisionObject::Request> request,
  std::shared_ptr<rosiris_manip_srv::AttachCollisionObject::Response> response)
{
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
    if (!scene)
    {
      response->result.return_type = rosiris_srv_result::ERROR;
      response->result.error_code.error_code =
        rosiris_srv_error_codes::ERROR_LOCKING_PLANNING_SCENE;
      response->result.message = "Failed to lock planning scene";
      return;
    }

    // Validate object and link exist
    if (!scene->knowsFrameTransform(request->collision_object_id))
    {
      response->result.return_type = rosiris_srv_result::ERROR;
      response->result.error_code.error_code =
        rosiris_srv_error_codes::ERROR_COLLISION_OBJECT_NOT_FOUND;
      response->result.message = "Object not found: " + request->collision_object_id;
      return;
    }

    if (!scene->knowsFrameTransform(request->attach_to_link))
    {
      response->result.return_type = rosiris_srv_result::ERROR;
      response->result.error_code.error_code = rosiris_srv_error_codes::ERROR_LINK_NOT_FOUND;
      response->result.message = "Link not found: " + request->attach_to_link;
      return;
    }

    // Perform attachment
    moveit_msgs::msg::AttachedCollisionObject attached_obj;
    attached_obj.object.id = request->collision_object_id;
    attached_obj.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    attached_obj.link_name = request->attach_to_link;

    if (!scene->processAttachedCollisionObjectMsg(attached_obj))
    {
      response->result.return_type = rosiris_srv_result::ERROR;
      response->result.error_code.error_code =
        rosiris_srv_error_codes::ERROR_ATTACH_COLLISION_OBJECT;
      response->result.message = "Failed to attach object";
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Attached object %s to link %s", request->collision_object_id.c_str(),
      request->attach_to_link.c_str());
  }

  // Apply collision matrix update
  if (!request->collision_matrix_update.collision_entries.empty())
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
    if (scene)
    {
      collision_detection::AllowedCollisionMatrix & acm =
        scene->getAllowedCollisionMatrixNonConst();

      if (!applyCollisionMatrixUpdate(acm, request->collision_matrix_update))
      {
        RCLCPP_WARN(get_logger(), "ACM update failed during attachment");
      }
    }
  }
  else
  {
    // Default behavior: Allow collision between object and attach_to_link
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
    if (scene)
    {
      collision_detection::AllowedCollisionMatrix & acm =
        scene->getAllowedCollisionMatrixNonConst();
      acm.setEntry(request->collision_object_id, request->attach_to_link, true);

      RCLCPP_DEBUG(
        get_logger(), "Default ACM: Allowed collision %s <-> %s",
        request->collision_object_id.c_str(), request->attach_to_link.c_str());
    }
  }

  // Trigger planning scene update
  planning_scene_monitor_->triggerSceneUpdateEvent(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  response->result.return_type = rosiris_srv_result::SUCCESS;
  response->result.error_code.error_code = rosiris_srv_error_codes::NO_ERROR;
  response->result.message = "Object attached successfully";
}

void PlanningSceneManager::detachCollisionObject(
  const std::shared_ptr<rosiris_manip_srv::DetachCollisionObject::Request> request,
  std::shared_ptr<rosiris_manip_srv::DetachCollisionObject::Response> response)
{
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
    if (!scene)
    {
      response->result.return_type = rosiris_srv_result::ERROR;
      response->result.error_code.error_code =
        rosiris_srv_error_codes::ERROR_LOCKING_PLANNING_SCENE;
      response->result.message = "Failed to lock planning scene";
      return;
    }

    // Validate object exists
    if (!scene->knowsFrameTransform(request->collision_object_id))
    {
      response->result.return_type = rosiris_srv_result::ERROR;
      response->result.error_code.error_code =
        rosiris_srv_error_codes::ERROR_COLLISION_OBJECT_NOT_FOUND;
      response->result.message = "Object not found: " + request->collision_object_id;
      return;
    }

    // Perform detachment
    moveit_msgs::msg::AttachedCollisionObject attached_obj;
    attached_obj.object.id = request->collision_object_id;
    attached_obj.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    attached_obj.link_name = request->detach_to_link;

    if (!scene->processAttachedCollisionObjectMsg(attached_obj))
    {
      response->result.return_type = rosiris_srv_result::ERROR;
      response->result.error_code.error_code =
        rosiris_srv_error_codes::ERROR_DETACH_COLLISION_OBJECT;
      response->result.message = "Failed to detach object";
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Detached object %s from link %s", request->collision_object_id.c_str(),
      request->detach_to_link.c_str());
  }

  // Apply collision matrix update
  if (!request->collision_matrix_update.collision_entries.empty())
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
    if (scene)
    {
      collision_detection::AllowedCollisionMatrix & acm =
        scene->getAllowedCollisionMatrixNonConst();

      if (!applyCollisionMatrixUpdate(acm, request->collision_matrix_update))
      {
        RCLCPP_WARN(get_logger(), "ACM update failed during detachment");
      }
    }
  }
  else
  {
    // Default behavior: Disallow collision with all robot links after detachment
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
    if (scene)
    {
      collision_detection::AllowedCollisionMatrix & acm =
        scene->getAllowedCollisionMatrixNonConst();

      const moveit::core::RobotModelConstPtr & robot_model = scene->getRobotModel();
      const std::vector<std::string> & link_names = robot_model->getLinkModelNames();

      for (const auto & link_name : link_names)
      {
        acm.setEntry(request->collision_object_id, link_name, false);
      }

      RCLCPP_DEBUG(get_logger(), "Default ACM: Disallowed collision with all robot links");
    }
  }

  // Trigger planning scene update
  planning_scene_monitor_->triggerSceneUpdateEvent(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  response->result.return_type = rosiris_srv_result::SUCCESS;
  response->result.error_code.error_code = rosiris_srv_error_codes::NO_ERROR;
  response->result.message = "Object detached successfully";
}

void PlanningSceneManager::moveCollisionObjects(
  const std::shared_ptr<rosiris_manip_srv::MoveCollisionObjects::Request> request,
  std::shared_ptr<rosiris_manip_srv::MoveCollisionObjects::Response> response)
{
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
    if (!scene)
    {
      response->result.return_type = rosiris_srv_result::ERROR;
      response->result.error_code.error_code =
        rosiris_srv_error_codes::ERROR_LOCKING_PLANNING_SCENE;
      response->result.message = "Failed to lock planning scene";
      return;
    }

    // Process each move
    for (const auto & update : request->collision_object_pose_updates)
    {
      // Validate object exists
      if (!scene->knowsFrameTransform(update.object_id))
      {
        RCLCPP_WARN(get_logger(), "Object not found: %s", update.object_id.c_str());
        response->not_moved_object_ids.push_back(update.object_id);
        continue;
      }

      // Move the object
      moveit_msgs::msg::CollisionObject obj;
      obj.id = update.object_id;
      obj.operation = moveit_msgs::msg::CollisionObject::MOVE;
      obj.pose = update.pose.pose;
      obj.header = update.pose.header;

      if (scene->processCollisionObjectMsg(obj))
      {
        response->moved_object_ids.push_back(update.object_id);
        RCLCPP_INFO(get_logger(), "Moved object: %s", update.object_id.c_str());
      }
      else
      {
        response->not_moved_object_ids.push_back(update.object_id);
        RCLCPP_WARN(get_logger(), "Failed to move object: %s", update.object_id.c_str());
      }
    }
  }

  // Apply ACM updates if provided
  for (const auto & update : request->collision_object_pose_updates)
  {
    if (!update.collision_matrix_update.collision_entries.empty())
    {
      planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
      if (scene)
      {
        collision_detection::AllowedCollisionMatrix & acm =
          scene->getAllowedCollisionMatrixNonConst();

        if (!applyCollisionMatrixUpdate(acm, update.collision_matrix_update))
        {
          RCLCPP_WARN(
            get_logger(), "ACM update failed for moved object: %s", update.object_id.c_str());
        }
      }
    }
  }

  // Trigger planning scene update
  planning_scene_monitor_->triggerSceneUpdateEvent(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  response->result.return_type = response->not_moved_object_ids.empty()
                                   ? rosiris_srv_result::SUCCESS
                                   : rosiris_srv_result::PARTIAL_SUCCESS;
  response->result.error_code.error_code =
    response->not_moved_object_ids.empty() ? rosiris_srv_error_codes::NO_ERROR
                                           : rosiris_srv_error_codes::ERROR_MOVING_COLLISION_OBJECT;
  response->result.message =
    response->not_moved_object_ids.empty()
      ? "All objects moved successfully"
      : "Failed to move " + std::to_string(response->not_moved_object_ids.size()) + " object(s)";
}

void PlanningSceneManager::updateAllowedCollisions(
  const std::shared_ptr<rosiris_manip_srv::UpdateAllowedCollisions::Request> request,
  std::shared_ptr<rosiris_manip_srv::UpdateAllowedCollisions::Response> response)
{
  auto updated_links = applyCollisionMatrixUpdates(request->collision_matrix_updates);

  // Trigger planning scene update
  planning_scene_monitor_->triggerSceneUpdateEvent(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  response->updated_object_ids = updated_links;

  // Determine which updates failed
  for (const auto & update : request->collision_matrix_updates)
  {
    if (
      std::find(updated_links.begin(), updated_links.end(), update.target_link) ==
      updated_links.end())
    {
      response->not_updated_object_ids.push_back(update.target_link);
    }
  }

  response->result.return_type = response->not_updated_object_ids.empty()
                                   ? rosiris_srv_result::SUCCESS
                                   : rosiris_srv_result::PARTIAL_SUCCESS;
  response->result.error_code.error_code = response->not_updated_object_ids.empty()
                                             ? rosiris_srv_error_codes::NO_ERROR
                                             : rosiris_srv_error_codes::ERROR_UPDATING_ACM;
  response->result.message =
    response->not_updated_object_ids.empty()
      ? "ACM updated successfully"
      : "Failed to update " + std::to_string(response->not_updated_object_ids.size()) + " link(s)";
}

}  // namespace rosiris_manipulation_utils

// register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(rosiris_manipulation_utils::PlanningSceneManager)