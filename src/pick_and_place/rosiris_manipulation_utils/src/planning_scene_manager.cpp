#include "rosiris_manipulation_utils/planning_scene_manager.hpp"

// messages
#include <rosiris_manipulation_interfaces/msg/collision_entry.hpp>
#include <rosiris_manipulation_interfaces/msg/service_error_code.hpp>
#include <rosiris_manipulation_interfaces/msg/service_result.hpp>
namespace rosiris_manipulation_utils
{

namespace rosiris_manip_srv = rosiris_manipulation_interfaces::srv;
namespace rosiris_manip_msg = rosiris_manipulation_interfaces::msg;
using rosiris_srv_error_codes = rosiris_manipulation_interfaces::msg::ServiceErrorCode;
using rosiris_srv_result = rosiris_manipulation_interfaces::msg::ServiceResult;

PlanningSceneManager::PlanningSceneManager(
  std::string node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  initializeServices();
  RCLCPP_INFO(this->get_logger(), "initialization complete. Ready to receive requests.");
}

PlanningSceneManager::~PlanningSceneManager() {}

void PlanningSceneManager::initializeServices()
{
  // Create service servers
  add_collision_objects_srv_ = create_service<rosiris_manip_srv::AddCollisionObjects>(
    std::string(get_name()) + "/add_collision_objects",
    std::bind(
      &PlanningSceneManager::addCollisionObjects, this, std::placeholders::_1,
      std::placeholders::_2));

  remove_collision_objects_srv_ = create_service<rosiris_manip_srv::RemoveCollisionObjects>(
    std::string(get_name()) + "/remove_collision_objects",
    std::bind(
      &PlanningSceneManager::removeCollisionObjects, this, std::placeholders::_1,
      std::placeholders::_2));

  move_collision_objects_srv_ = create_service<rosiris_manip_srv::MoveCollisionObjects>(
    std::string(get_name()) + "/move_collision_objects",
    std::bind(
      &PlanningSceneManager::moveCollisionObjects, this, std::placeholders::_1,
      std::placeholders::_2));

  attach_collision_object_srv_ = create_service<rosiris_manip_srv::AttachCollisionObject>(
    std::string(get_name()) + "/attach_collision_object",
    std::bind(
      &PlanningSceneManager::attachCollisionObject, this, std::placeholders::_1,
      std::placeholders::_2));

  detach_collision_object_srv_ = create_service<rosiris_manip_srv::DetachCollisionObject>(
    std::string(get_name()) + "/detach_collision_object",
    std::bind(
      &PlanningSceneManager::detachCollisionObject, this, std::placeholders::_1,
      std::placeholders::_2));

  update_allowed_collisions_srv_ = create_service<rosiris_manip_srv::UpdateAllowedCollisions>(
    std::string(get_name()) + "/update_allowed_collisions",
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

  // Start publishing the planning scene
  planning_scene_monitor_->startPublishingPlanningScene(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  return true;
}

/**
 * @brief Apply collision matrix updates to ACM
 */
std::tuple<std::vector<std::string>, std::vector<std::string>>
PlanningSceneManager::applyCollisionMatrixUpdates(
  const std::vector<rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate> & updates)
{
  std::vector<std::string> updated_links;
  std::vector<std::string> failed_links;

  planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
  if (!scene)
  {
    RCLCPP_ERROR(get_logger(), "Failed to lock planning scene");
    for (const auto & update : updates)
    {
      failed_links.push_back(update.target_link);
    }
    return std::make_tuple(updated_links, failed_links);
  }

  collision_detection::AllowedCollisionMatrix & acm = scene->getAllowedCollisionMatrixNonConst();

  for (const auto & update : updates)
  {
    // Validate target_link exists in the scene
    if (!scene->knowsFrameTransform(update.target_link))
    {
      RCLCPP_WARN(get_logger(), "Target link not found in scene: %s", update.target_link.c_str());
      continue;
    }

    if (applyCollisionMatrixUpdate(acm, update))
    {
      updated_links.push_back(update.target_link);
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Failed to apply ACM update for: %s", update.target_link.c_str());
      failed_links.push_back(update.target_link);
    }
  }

  return std::make_tuple(updated_links, failed_links);
}

/**
 * @brief Apply single collision matrix update
 */
bool PlanningSceneManager::applyCollisionMatrixUpdate(
  collision_detection::AllowedCollisionMatrix & acm,
  const rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate & update)
{
  using CollisionMatrixUpdate = rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate;

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
  const std::vector<rosiris_manip_msg::CollisionObject> & objs)
{
  if (objs.empty())
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
  for (const auto & obj : objs)
  {
    if (!acm.hasEntry(obj.collision_object.id))
    {
      acm.setDefaultEntry(obj.collision_object.id, false);
      acm.setEntry(obj.collision_object.id, obj.collision_object.id, obj.self_collision_allowed);
      RCLCPP_DEBUG(get_logger(), "Initialized ACM entry for: %s", obj.collision_object.id.c_str());
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
  std::vector<rosiris_manip_msg::CollisionObject> added_objs;
  added_objs.reserve(request->collision_objects.size());
  std::vector<rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate> cmus;
  cmus.reserve(request->collision_objects.size());

  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
    if (!scene)
    {
      response->result.return_type = rosiris_srv_result::ERROR;
      response->result.error_code.error_code =
        rosiris_srv_error_codes::ERROR_ACQUIRE_PLANNING_SCENE;
      response->result.message = "Planning scene not available";
      return;
    }

    // Add objects to planning scene
    for (auto & obj : request->collision_objects)
    {
      if (scene->processCollisionObjectMsg(obj.collision_object))
      {
        added_ids.push_back(obj.collision_object.id);
        cmus.push_back(convert_touch_links_to_collision_matrix_update(
          obj.collision_object.id, obj.allowed_touch_links,
          rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate::MERGE, true));
        added_objs.push_back(std::move(obj));
      }
      else
      {
        not_added_ids.push_back(obj.collision_object.id);
        RCLCPP_WARN(
          get_logger(), "Failed to add collision object: %s", obj.collision_object.id.c_str());
      }
    }
  }

  if (added_ids.empty())
  {
    response->result.return_type = rosiris_srv_result::ERROR;
    response->result.error_code.error_code = rosiris_srv_error_codes::ERROR_ADDING_COLLISION_OBJECT;
    response->result.message = "No objects were added successfully";
    return;
  }

  // Initialize ACM entries for successfully added objects
  initializeObjectsInACM(added_objs);

  auto [acm_updated_links, acm_updated_failed_links] = applyCollisionMatrixUpdates(cmus);

  planning_scene_monitor_->triggerSceneUpdateEvent(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  response->added_object_ids = added_ids;
  response->not_added_object_ids = not_added_ids;
  if (!not_added_ids.empty())
  {
    response->result.return_type = rosiris_srv_result::PARTIAL_SUCCESS;
    response->result.error_code.error_code = rosiris_srv_error_codes::ERROR_ADDING_COLLISION_OBJECT;
    response->result.message =
      "Failed to add " + std::to_string(not_added_ids.size()) + " object(s)";
  }

  if (!acm_updated_failed_links.empty())
  {
    response->result.return_type = rosiris_srv_result::PARTIAL_SUCCESS;
    response->result.error_code.error_code = rosiris_srv_error_codes::ERROR_UPDATING_ACM;
    response->result.message = "Could not apply ACM updates for " +
                               std::to_string(acm_updated_failed_links.size()) + " links";
    return;
  }

  response->result.return_type = rosiris_srv_result::SUCCESS;
  response->result.error_code.error_code = rosiris_srv_error_codes::NO_ERROR;
  response->result.message = "All objects added successfully";
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
        rosiris_srv_error_codes::ERROR_ACQUIRE_PLANNING_SCENE;
      response->result.message = "Planning scene not available";
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
        rosiris_srv_error_codes::ERROR_ACQUIRE_PLANNING_SCENE;
      response->result.message = "Planning scene not available";
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
    attached_obj.touch_links = request->allowed_touch_links;

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
        rosiris_srv_error_codes::ERROR_ACQUIRE_PLANNING_SCENE;
      response->result.message = "Planning scene not available";
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
    attached_obj.link_name = request->detach_from_link;

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
      request->detach_from_link.c_str());
  }

  // Apply collision matrix update
  std::vector<rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate> cmus;
  cmus.push_back(convert_touch_links_to_collision_matrix_update(
    request->collision_object_id, request->disallowed_touch_links,
    rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate::MERGE, false));
  auto [acm_updated_links, acm_updated_failed_links] = applyCollisionMatrixUpdates(cmus);

  planning_scene_monitor_->triggerSceneUpdateEvent(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  if (!acm_updated_failed_links.empty())
  {
    response->result.return_type = rosiris_srv_result::PARTIAL_SUCCESS;
    response->result.error_code.error_code = rosiris_srv_error_codes::ERROR_UPDATING_ACM;
    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < acm_updated_failed_links.size(); ++i)
    {
      ss << acm_updated_failed_links[i];
      if (i + 1 < acm_updated_failed_links.size())
      {
        ss << ", ";
      }
    }
    ss << "]";
    response->result.message = "Could not apply ACM updates for links: " + ss.str();
    return;
  }

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
        rosiris_srv_error_codes::ERROR_ACQUIRE_PLANNING_SCENE;
      response->result.message = "Planning scene not available";
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

  if (response->moved_object_ids.empty())
  {
    response->result.return_type = rosiris_srv_result::ERROR;
    response->result.error_code.error_code = rosiris_srv_error_codes::ERROR_MOVING_COLLISION_OBJECT;
    response->result.message = "No objects were moved successfully";
    return;
  }

  // Apply ACM updates only for successfully moved objects if provided
  std::vector<rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate> moved_obj_acm_updates;
  moved_obj_acm_updates.reserve(response->moved_object_ids.size());
  for (const auto & update : request->collision_object_pose_updates)
  {
    if (
      std::find(
        response->moved_object_ids.begin(), response->moved_object_ids.end(), update.object_id) !=
      response->moved_object_ids.end())
    {
      moved_obj_acm_updates.push_back(update.collision_matrix_update);
    }
  }
  auto [acm_updated_links, acm_update_failed_links] =
    applyCollisionMatrixUpdates(moved_obj_acm_updates);

  planning_scene_monitor_->triggerSceneUpdateEvent(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  if (!response->not_moved_object_ids.empty())
  {
    response->result.return_type = rosiris_srv_result::PARTIAL_SUCCESS;
    response->result.error_code.error_code = rosiris_srv_error_codes::ERROR_MOVING_COLLISION_OBJECT;
    response->result.message =
      "Failed to move " + std::to_string(response->not_moved_object_ids.size()) + " object(s)";
    return;
  }

  if (!acm_update_failed_links.empty())
  {
    response->result.return_type = rosiris_srv_result::PARTIAL_SUCCESS;
    response->result.error_code.error_code = rosiris_srv_error_codes::ERROR_UPDATING_ACM;
    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < acm_update_failed_links.size(); ++i)
    {
      ss << acm_update_failed_links[i];
      if (i + 1 < acm_update_failed_links.size())
      {
        ss << ", ";
      }
    }
    ss << "]";
    response->result.message = "Could not apply ACM updates for links: " + ss.str();
    return;
  }
  response->result.return_type = rosiris_srv_result::SUCCESS;
  response->result.error_code.error_code = rosiris_srv_error_codes::NO_ERROR;
  response->result.message = "All objects moved successfully";
}

void PlanningSceneManager::updateAllowedCollisions(
  const std::shared_ptr<rosiris_manip_srv::UpdateAllowedCollisions::Request> request,
  std::shared_ptr<rosiris_manip_srv::UpdateAllowedCollisions::Response> response)
{
  auto [acm_updated_links, acm_update_failed_links] =
    applyCollisionMatrixUpdates(request->collision_matrix_updates);

  // Trigger planning scene update
  planning_scene_monitor_->triggerSceneUpdateEvent(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  response->updated_object_ids = acm_updated_links;
  response->not_updated_object_ids = acm_update_failed_links;

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

rosiris_manipulation_interfaces::msg::CollisionMatrixUpdate
PlanningSceneManager::convert_touch_links_to_collision_matrix_update(
  const std::string & target_link, const std::vector<std::string> & touch_links, uint8_t mode,
  bool allow_touch)
{
  rosiris_manip_msg::CollisionMatrixUpdate cmu;
  cmu.target_link = target_link;
  cmu.mode = mode;
  for (const std::string & touch_link : touch_links)
  {
    rosiris_manip_msg::CollisionEntry entry;
    entry.touch_link = touch_link;
    entry.collision_allowed = allow_touch;
    cmu.collision_entries.push_back(entry);
  }
  return cmu;
}

}  // namespace rosiris_manipulation_utils