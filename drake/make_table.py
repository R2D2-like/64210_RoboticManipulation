import os

# Define the SDF for the table with shorter legs and lower height
table_sdf = """
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="table">
    <static>true</static>
    <link name="link">
      <collision name="surface">
        <pose>0 0.75 0.4 0 0 0</pose> <!-- テーブル高さを0.4に変更 -->
        <geometry>
          <box>
            <size>1.5 0.8 0.03</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.6</mu>
              <mu2>0.6</mu2>
            </ode>
          </friction>
        </surface>
        <drake:proximity_properties>
        <drake:compliant_hydroelastic/>
        <drake:hydroelastic_modulus>1.0e6</drake:hydroelastic_modulus>
      </drake:proximity_properties>
      </collision>
      <visual name="visual1">
        <pose>0 0.75 0.4 0 0 0</pose> <!-- テーブル高さを0.4に変更 -->
        <geometry>
          <box>
            <size>1.5 0.8 0.03</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Wood</name>
          </script>
        </material>
      </visual>
      <!-- Shortened legs to 0.4 meters -->
      <collision name="front_left_leg">
        <pose>0.68 1.13 0.2 0 0 0</pose> <!-- 脚の中心位置を0.2に調整 -->
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.4</length>  <!-- 脚の長さを0.4に変更 -->
          </cylinder>
        </geometry>
      </collision>
      <visual name="front_left_leg">
        <pose>0.68 1.13 0.2 0 0 0</pose> <!-- 脚の中心位置を0.2に調整 -->
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.4</length>  <!-- 脚の長さを0.4に変更 -->
          </cylinder>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script>
        </material>
      </visual>
      <collision name="front_right_leg">
        <pose>0.68 0.37 0.2 0 0 0</pose> <!-- 脚の中心位置を0.2に調整 -->
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.4</length>  <!-- 脚の長さを0.4に変更 -->
          </cylinder>
        </geometry>
      </collision>
      <visual name="front_right_leg">
        <pose>0.68 0.37 0.2 0 0 0</pose> <!-- 脚の中心位置を0.2に調整 -->
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.4</length>  <!-- 脚の長さを0.4に変更 -->
          </cylinder>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script>
        </material>
      </visual>
      <collision name="back_right_leg">
        <pose>-0.68 0.37 0.2 0 0 0</pose> <!-- 脚の中心位置を0.2に調整 -->
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.4</length>  <!-- 脚の長さを0.4に変更 -->
          </cylinder>
        </geometry>
      </collision>
      <visual name="back_right_leg">
        <pose>-0.68 0.37 0.2 0 0 0</pose> <!-- 脚の中心位置を0.2に調整 -->
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.4</length>  <!-- 脚の長さを0.4に変更 -->
          </cylinder>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script>
        </material>
      </visual>
      <collision name="back_left_leg">
        <pose>-0.68 1.13 0.2 0 0 0</pose> <!-- 脚の中心位置を0.2に調整 -->
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.4</length>  <!-- 脚の長さを0.4に変更 -->
          </cylinder>
        </geometry>
      </collision>
      <visual name="back_left_leg">
        <pose>-0.68 1.13 0.2 0 0 0</pose> <!-- 脚の中心位置を0.2に調整 -->
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.4</length>  <!-- 脚の長さを0.4に変更 -->
          </cylinder>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

# Save the table SDF to a file for reference
table_sdf_path = os.path.join(os.getcwd(), "table.sdf")
with open(table_sdf_path, "w") as f:
    f.write(table_sdf)
