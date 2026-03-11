ros2 service call /gz_server/spawn_entity simulation_interfaces/srv/SpawnEntity "
name: 'cube'
allow_renaming: true
uri: ''
resource_string: '
<sdf version=\"1.9\">
  <model name=\"cube\">
    <static>false</static>
    <link name=\"link\">
      <inertial>
        <mass>0.2</mass>
        <inertia>
          <ixx>0.0000833</ixx>
          <iyy>0.0000833</iyy>
          <izz>0.0000833</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>

      <collision name=\"collision\">
        <geometry>
          <box>
            <size>0.1 0.1 0.1</size>
          </box>
        </geometry>
      </collision>

      <visual name=\"visual\">
        <geometry>
          <box>
            <size>0.1 0.1 0.1</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
'
entity_namespace: ''
initial_pose:
  header:
    frame_id: 'world'
  pose:
    position:
      x: 1.5
      y: 1.8
      z: 1.5
    orientation:
      w: 1.0
"

with pose publishing:


ros2 service call /gz_server/spawn_entity simulation_interfaces/srv/SpawnEntity "{
  name: 'cube_5cm',
  allow_renaming: true,
  uri: '',
  resource_string: '<sdf version=\"1.9\">
<model name=\"cube_5cm\">
  <static>false</static>
  <link name=\"link\">
    <inertial>
      <mass>0.2</mass>
      <inertia>
        <ixx>0.0000833</ixx>
        <iyy>0.0000833</iyy>
        <izz>0.0000833</izz>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyz>0</iyz>
      </inertia>
    </inertial>

    <collision name=\"collision\">
      <geometry>
        <box>
          <size>0.05 0.05 0.05</size>
        </box>
      </geometry>
    </collision>

    <visual name=\"visual\">
      <geometry>
        <box>
          <size>0.05 0.05 0.05</size>
        </box>
      </geometry>
    </visual>
  </link>

  <plugin filename=\"gz-sim-pose-publisher-system\" name=\"gz::sim::systems::PosePublisher\">
    <publish_link_pose>true</publish_link_pose>
    <publish_collision_pose>true</publish_collision_pose>
    <publish_visual_pose>false</publish_visual_pose>
    <publish_nested_model_pose>false</publish_nested_model_pose>
  </plugin>
</model>
</sdf>',
  entity_namespace: '',
  initial_pose: {
    header: { frame_id: 'world' },
    pose: {
      position: { x: 1.5, y: 1.8, z: 1.5 },
      orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    }
  }
}"