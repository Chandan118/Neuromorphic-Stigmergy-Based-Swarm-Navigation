import os
import subprocess
import time

sdf_template = """
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{name}">
    <pose>{x} {y} 0.1 0 0 0</pose>
    <link name="chassis">
      <pose>0 0 0 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.2 0.15 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.2 0.15 0.05</size>
          </box>
        </geometry>
      </collision>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <iyy>0.01</iyy>
          <izz>0.01</izz>
        </inertia>
      </inertial>
    </link>
    <link name="left_wheel">
      <pose>0.05 0.09 0 -1.5707 0 0</pose>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
      </collision>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
    </link>
    <link name="right_wheel">
      <pose>0.05 -0.09 0 -1.5707 0 0</pose>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
      </collision>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
    </link>
    <joint name="left_wheel_joint" type="fixed">
      <parent>chassis</parent>
      <child>left_wheel</child>
    </joint>
    <joint name="right_wheel_joint" type="fixed">
      <parent>chassis</parent>
      <child>right_wheel</child>
    </joint>
    <plugin
      filename="ignition-gazebo-velocity-control-system"
      name="ignition::gazebo::systems::VelocityControl">
      <link_name>chassis</link_name>
      <initial_linear>0 0 0</initial_linear>
      <initial_angular>0 0 0</initial_angular>
    </plugin>
  </model>
</sdf>
"""

num_robots = 50
print("Spawning 50 Wheelbase Robot models...")

# Ensure output dir exists for temp sdfs
os.makedirs("temp_sdfs", exist_ok=True)

for i in range(num_robots):
    name = f"robot_{i}"
    x = (i % 10) * 1.0 - 5.0
    y = (i // 10) * 1.0 - 2.5
    sdf = sdf_template.format(name=name, x=x, y=y)
    
    filename = f"temp_sdfs/{name}.sdf"
    with open(filename, "w") as f:
        f.write(sdf)
        
    cmd = [
        "gz", "service", "-s", "/world/swarm_world/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "1000",
        "--req", f'sdf_filename: "{os.path.abspath(filename)}", name: "{name}"'
    ]
    subprocess.run(cmd)
    print(f"Spawned {name} at {x}, {y}")
    
    # Wait a bit to let the server process it safely
    time.sleep(0.5)

print("Finished spawning 50 robots.")
