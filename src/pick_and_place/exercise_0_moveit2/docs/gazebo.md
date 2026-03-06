## Versions
https://gazebosim.org/about


## Simtime
https://design.ros2.org/articles/clock_and_time.html
Set parameter action to set on every node
https://robotics.stackexchange.com/questions/97957/ros2-set-parameters-from-file-and-directly
https://docs.ros.org/en/kilted/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html



## Controller Manager

## Mesh loading

Gazebo must be able to find mesh/model files.

There are **three different mechanisms**, used at different layers.

---

# 1️⃣ `package://` in URDF/Xacro

**Layer:** ROS (ament index)
**Used for:** Meshes inside URDF

Example:

```xml
<mesh filename="package://my_pkg/meshes/object.stl"/>
```

**Works in:**

* RViz
* robot_state_publisher
* Gazebo (via ROS integration)

**Requirement:**
Package must be installed or built and the environment sourced.

**Use this for:**
Reusable description packages (xacro + meshes).

---

# 2️⃣ `<export><gazebo_ros gazebo_model_path=...>` in `package.xml`

**Layer:** Gazebo Sim resource path
**Sets:** `GZ_SIM_RESOURCE_PATH`

Example:

```xml
<export>
  <gazebo_ros gazebo_model_path="${prefix}/../"/>
</export>
```

**Effect:**
Makes Gazebo aware of your package’s install location (e.g. `/opt/ros/.../share`).

**Works when:**

* Package is installed (workspace or apt)
* Environment is sourced

**Use this for:**
Packages that provide simulation assets used inside Gazebo (meshes or SDF models).

---

# 3️⃣ Manually setting `GZ_SIM_RESOURCE_PATH` in launch

**Layer:** Gazebo runtime only

Example:

```python
AppendEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH',
    get_package_share_directory('my_pkg')
)
```

**Effect:**
Adds path only for that launch session.

**Use this when:**

* You cannot modify the exporting package
* You need simulation-specific overrides

---

# Clean Rule

If it is a **URDF mesh inside a ROS package** → use `package://`.

If Gazebo cannot find it → expose the package path via
`<gazebo_ros gazebo_model_path=...>`.

If you need a temporary fix → set `GZ_SIM_RESOURCE_PATH` in launch.

---

# Practical Recommendation

For reusable description packages:

* Use `package://`
* Add `<gazebo_ros gazebo_model_path="${prefix}/../"/>`

This works for:

* local workspaces
* `/opt/ros` installs
* future reuse without cloning source
