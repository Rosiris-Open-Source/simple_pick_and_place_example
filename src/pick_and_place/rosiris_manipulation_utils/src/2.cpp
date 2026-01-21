#ifndef PLANNING_SCENE_SERVICE_NODE_HPP
#define PLANNING_SCENE_SERVICE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <memory>
#include <string>
#include <vector>
#include <map>

namespace planning_scene_services
{

class PlanningSceneServiceNode : public rclcpp::Node
{
public:
  explicit PlanningSceneServiceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("planning_scene_service_node", options)
  {
    // Declare parameters
    this->declare_parameter("reference_frame", "world");
    this->declare_parameter("robot_description", "robot_description");
    this->declare_parameter("use_service_queries", false);  // NEW: option to use service instead
    
    reference_frame_ = this->get_parameter("reference_frame").as_string();
    use_service_queries_ = this->get_parameter("use_service_queries").as_bool();

    if (use_service_queries_) {
      // OPTION 1: Use service calls to query move_group's scene (lighter weight)
      RCLCPP_INFO(this->get_logger(), "Using /get_planning_scene service for queries");
      
      get_scene_client_ = this->create_client<moveit_msgs::srv::GetPlanningScene>(
        "/get_planning_scene"
      );
      
    } else {
      // OPTION 2: Create our own monitor that syncs with move_group's scene
      RCLCPP_INFO(this->get_logger(), "Creating local planning scene monitor");
      
      planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        shared_from_this(),
        this->get_parameter("robot_description").as_string()
      );

      if (!planning_scene_monitor_->getPlanningScene()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize planning scene monitor");
        return;
      }

      // Subscribe to the SAME topics as move_group
      // This keeps our local copy synchronized with move_group's copy
      planning_scene_monitor_->startSceneMonitor("/planning_scene");
      planning_scene_monitor_->startStateMonitor("/joint_states");
      planning_scene_monitor_->startPublishingPlanningScene(
        planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
        "/our_monitored_planning_scene"  // Different topic to avoid confusion
      );
      
      // Wait for initial scene
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    // Publisher for planning scene updates (our modifications)
    // This is how we WRITE to the scene that move_group will also see
    scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
      "/planning_scene", 10);

    RCLCPP_INFO(this->get_logger(), "Planning Scene Service Node initialized");
    RCLCPP_INFO(this->get_logger(), "Reference frame: %s", reference_frame_.c_str());
  }

  /**
   * @brief Check if an object exists in the scene
   */
  bool objectExists(const std::string & name)
  {
    if (use_service_queries_) {
      return objectExistsViaService(name);
    } else {
      planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
      return scene->getWorld()->hasObject(name);
    }
  }

private:
  /**
   * @brief Query move_group's scene via service call
   */
  bool objectExistsViaService(const std::string & name)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = 
      moveit_msgs::srv::GetPlanningScene::Request::PlanningSceneComponents::WORLD_OBJECT_NAMES;

    auto result = get_scene_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = result.get();
      for (const auto & obj : response->scene.world.collision_objects) {
        if (obj.id == name) {
          return true;
        }
      }
    }
    return false;
  }

public:

  /**
   * @brief Get all object IDs currently in the scene
   */
  std::vector<std::string> getObjectIds()
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    return scene->getWorld()->getObjectIds();
  }

  /**
   * @brief Get the pose of an object
   */
  bool getObjectPose(const std::string & name, Eigen::Isometry3d & pose)
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    
    auto obj = scene->getWorld()->getObject(name);
    if (!obj) {
      RCLCPP_WARN(this->get_logger(), "Object %s not found", name.c_str());
      return false;
    }
    
    pose = obj->pose_;
    return true;
  }

  /**
   * @brief Add a box collision object (checks if exists first)
   */
  void addBox(const std::string & name, 
              const geometry_msgs::msg::Pose & pose,
              const std::vector<double> & dimensions,
              const std::string & frame_id = "",
              bool update_if_exists = true)
  {
    if (dimensions.size() != 3) {
      RCLCPP_ERROR(this->get_logger(), "Box dimensions must have 3 values [x, y, z]");
      return;
    }

    // Check if object already exists
    if (objectExists(name)) {
      if (update_if_exists) {
        RCLCPP_INFO(this->get_logger(), "Object %s already exists, updating...", name.c_str());
        updateObjectPose(name, pose);
        return;
      } else {
        RCLCPP_WARN(this->get_logger(), "Object %s already exists, skipping add", name.c_str());
        return;
      }
    }

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id.empty() ? reference_frame_ : frame_id;
    collision_object.header.stamp = this->now();
    collision_object.id = name;
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Define box primitive
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = dimensions;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);

    publishCollisionObject(collision_object);
  }

  /**
   * @brief Add a sphere collision object (checks if exists first)
   */
  void addSphere(const std::string & name,
                 const geometry_msgs::msg::Pose & pose,
                 double radius,
                 const std::string & frame_id = "",
                 bool update_if_exists = true)
  {
    if (objectExists(name)) {
      if (update_if_exists) {
        RCLCPP_INFO(this->get_logger(), "Object %s already exists, updating...", name.c_str());
        updateObjectPose(name, pose);
        return;
      } else {
        RCLCPP_WARN(this->get_logger(), "Object %s already exists, skipping add", name.c_str());
        return;
      }
    }

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id.empty() ? reference_frame_ : frame_id;
    collision_object.header.stamp = this->now();
    collision_object.id = name;
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    primitive.dimensions = {radius};

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);

    publishCollisionObject(collision_object);
  }

  /**
   * @brief Add a cylinder collision object (checks if exists first)
   */
  void addCylinder(const std::string & name,
                   const geometry_msgs::msg::Pose & pose,
                   double height,
                   double radius,
                   const std::string & frame_id = "",
                   bool update_if_exists = true)
  {
    if (objectExists(name)) {
      if (update_if_exists) {
        RCLCPP_INFO(this->get_logger(), "Object %s already exists, updating...", name.c_str());
        updateObjectPose(name, pose);
        return;
      } else {
        RCLCPP_WARN(this->get_logger(), "Object %s already exists, skipping add", name.c_str());
        return;
      }
    }

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id.empty() ? reference_frame_ : frame_id;
    collision_object.header.stamp = this->now();
    collision_object.id = name;
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions = {height, radius};

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);

    publishCollisionObject(collision_object);
  }

  /**
   * @brief Remove a collision object by name (checks if exists first)
   */
  void removeObject(const std::string & name, const std::string & frame_id = "")
  {
    if (!objectExists(name)) {
      RCLCPP_WARN(this->get_logger(), "Cannot remove object %s - does not exist", name.c_str());
      return;
    }

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id.empty() ? reference_frame_ : frame_id;
    collision_object.header.stamp = this->now();
    collision_object.id = name;
    collision_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    publishCollisionObject(collision_object);
    
    RCLCPP_INFO(this->get_logger(), "Removed collision object: %s", name.c_str());
  }

  /**
   * @brief Update an existing collision object's pose
   */
  void updateObjectPose(const std::string & name, const geometry_msgs::msg::Pose & new_pose)
  {
    if (!objectExists(name)) {
      RCLCPP_WARN(this->get_logger(), "Cannot update object %s - does not exist", name.c_str());
      return;
    }

    // Get the current object to preserve its geometry
    moveit_msgs::msg::CollisionObject collision_object;
    {
      planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
      auto obj = scene->getWorld()->getObject(name);
      
      collision_object.header.frame_id = reference_frame_;
      collision_object.header.stamp = this->now();
      collision_object.id = name;
      collision_object.operation = moveit_msgs::msg::CollisionObject::MOVE;
      
      // Copy shapes from current object
      for (const auto& shape : obj->shapes_) {
        shape_msgs::msg::SolidPrimitive primitive;
        // Convert shape to primitive (simplified - you'd need full conversion)
        collision_object.primitives.push_back(primitive);
      }
      
      collision_object.primitive_poses.push_back(new_pose);
    }

    publishCollisionObject(collision_object);
    RCLCPP_INFO(this->get_logger(), "Updated pose for object: %s", name.c_str());
  }

  /**
   * @brief Check if two objects/links would collide
   */
  bool wouldCollide(const std::string & name1, const std::string & name2)
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    
    // Get the allowed collision matrix
    const collision_detection::AllowedCollisionMatrix& acm = 
      scene->getAllowedCollisionMatrix();
    
    collision_detection::AllowedCollision::Type type;
    return !acm.getEntry(name1, name2, type) || 
           type == collision_detection::AllowedCollision::NEVER;
  }

  /**
   * @brief Attach an object to a robot link
   */
  void attachObject(const std::string & object_name,
                    const std::string & link_name,
                    const std::vector<std::string> & touch_links = {})
  {
    if (!objectExists(object_name)) {
      RCLCPP_ERROR(this->get_logger(), "Cannot attach object %s - does not exist", 
                   object_name.c_str());
      return;
    }

    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = link_name;
    attached_object.object.id = object_name;
    attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Set touch links
    if (touch_links.empty()) {
      attached_object.touch_links = {link_name};
    } else {
      attached_object.touch_links = touch_links;
    }

    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);

    scene_publisher_->publish(planning_scene);
    
    RCLCPP_INFO(this->get_logger(), "Attached object %s to link %s", 
                object_name.c_str(), link_name.c_str());
  }

  /**
   * @brief Detach an object from a robot link
   */
  void detachObject(const std::string & object_name, const std::string & link_name)
  {
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = link_name;
    attached_object.object.id = object_name;
    attached_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);

    scene_publisher_->publish(planning_scene);
    
    RCLCPP_INFO(this->get_logger(), "Detached object %s from link %s", 
                object_name.c_str(), link_name.c_str());
  }

  /**
   * @brief Allow collisions between two objects/links
   */
  void allowCollisions(const std::string & name1, const std::string & name2)
  {
    updateAllowedCollision(name1, name2, true);
  }

  /**
   * @brief Disallow collisions between two objects/links
   */
  void disallowCollisions(const std::string & name1, const std::string & name2)
  {
    updateAllowedCollision(name1, name2, false);
  }

  /**
   * @brief Clear all collision objects from the scene
   */
  void clearAllObjects()
  {
    auto object_ids = getObjectIds();
    
    for (const auto & name : object_ids) {
      removeObject(name);
    }
    
    RCLCPP_INFO(this->get_logger(), "Cleared %zu collision objects", object_ids.size());
  }

  /**
   * @brief Get the number of objects currently in the scene
   */
  size_t getObjectCount() const
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    return scene->getWorld()->getObjectIds().size();
  }

  /**
   * @brief Print all objects in the scene
   */
  void printSceneObjects()
  {
    auto object_ids = getObjectIds();
    
    RCLCPP_INFO(this->get_logger(), "=== Planning Scene Objects (%zu) ===", object_ids.size());
    for (const auto & id : object_ids) {
      Eigen::Isometry3d pose;
      if (getObjectPose(id, pose)) {
        RCLCPP_INFO(this->get_logger(), "  - %s at [%.2f, %.2f, %.2f]", 
                    id.c_str(), pose.translation().x(), 
                    pose.translation().y(), pose.translation().z());
      }
    }
  }

private:
  void publishCollisionObject(const moveit_msgs::msg::CollisionObject & collision_object)
  {
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.world.collision_objects.push_back(collision_object);

    scene_publisher_->publish(planning_scene);
    
    RCLCPP_INFO(this->get_logger(), "Published collision object: %s", 
                collision_object.id.c_str());
  }

  void updateAllowedCollision(const std::string & name1, 
                             const std::string & name2, 
                             bool allowed)
  {
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;

    auto & acm = planning_scene.allowed_collision_matrix;
    acm.entry_names = {name1, name2};
    
    moveit_msgs::msg::AllowedCollisionEntry entry1, entry2;
    entry1.enabled = {false, allowed};
    entry2.enabled = {allowed, false};
    
    acm.entry_values = {entry1, entry2};

    scene_publisher_->publish(planning_scene);
    
    RCLCPP_INFO(this->get_logger(), "%s collisions between %s and %s",
                allowed ? "Allowed" : "Disallowed", name1.c_str(), name2.c_str());
  }

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_publisher_;
  std::string reference_frame_;
};

}  // namespace planning_scene_services

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planning_scene_services::PlanningSceneServiceNode)

#endif  // PLANNING_SCENE_SERVICE_NODE_HPP