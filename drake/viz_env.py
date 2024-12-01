import os
import numpy as np
from pydrake.all import (
    AddDefaultVisualization,
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    LoadModelDirectives,
    LoadModelDirectivesFromString,
    Parser,
    ProcessModelDirectives,
    RigidTransform,
    RollPitchYaw,
    Simulator,
    StartMeshcat,
)
from pydrake.visualization import ModelVisualizer
from manipulation.utils import ConfigureParser
from manipulation.station import LoadScenario, MakeHardwareStation
import time

# Start Meshcat for visualization
meshcat = StartMeshcat()
print(meshcat.web_url())


###########THIS IS WORKED##########
arrow_sdf_path = os.path.join(os.getcwd(), "arrow/mesh/arrow.sdf")
print(arrow_sdf_path)
table_sdf_path = os.path.join(os.getcwd(), "table.sdf")
# Create a simple package.xml file to define our dummy package
pkg_xml = """<?xml version="1.0"?>
<package format="2">
  <name>dummy_project</name>
  <version>0.0.0</version>
  <description>dummy project</description>
  <maintainer email="manipulation-student@mit.edu">IIWA</maintainer>
  <author>IIWA</author>
  <license>N/A</license>
</package>
"""

# Save the package.xml file
package_xml_path = os.path.join(os.getcwd(), "package.xml")
with open(package_xml_path, "w") as f:
    f.write(pkg_xml)

model_directive = f"""
directives:
# - add_directives:
#     file: package://manipulation/iiwa_and_wsg.dmd.yaml
- add_model:
    name: iiwa
    file: package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf
    default_joint_positions:
        iiwa_joint_1: [-1.57]
        iiwa_joint_2: [0.1]
        iiwa_joint_3: [0]
        iiwa_joint_4: [-1.2]
        iiwa_joint_5: [0]
        iiwa_joint_6: [ 1.6]
        iiwa_joint_7: [0]
- add_weld:
    parent: world
    child: iiwa::iiwa_link_0
    X_PC:
        translation: [0, 0, 0]  # 必要に応じて調整
        rotation: !Rpy {{ deg: [0, 0, 180] }}
- add_model:
    name: wsg
    file: package://manipulation/hydro/schunk_wsg_50_with_tip.sdf
- add_weld:
    parent: iiwa::iiwa_link_7
    child: wsg::body
    X_PC:
        translation: [0, 0, 0.09]
        rotation: !Rpy {{ deg: [90, 0, 90] }}
- add_model:
    name: table
    file: file://{table_sdf_path}  # テーブルモデルを使用
    default_free_body_pose:
      link:
        translation: [0, 0, 0]  # テーブルを配置
        rotation: !Rpy {{ deg: [0, 0, 0] }}
- add_model:
    name: arrow
    file: file://{arrow_sdf_path} 
    default_free_body_pose:
      arrow_body_link:
        translation: [0.0, 0.75, 1]  # テーブルの上に配置
        rotation: !Rpy {{ deg: [0, 90, 90] }}      
"""



# Define a function to create the scene using the model directives
def create_scene_directives(model_directive, sim_time_step=0.001):
    meshcat.Delete()
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    parser = Parser(plant)
    ConfigureParser(parser)
    
    # Add package mappings to find the models
    parser.package_map().Add("dummy_project", os.path.abspath(""))


    
    # Load model directives from string and process them
    directives = LoadModelDirectivesFromString(model_directive)
    ProcessModelDirectives(directives, plant, parser)
    
    
    # Finalize the plant after adding all models
    plant.Finalize()
    
    # Add default visualization
    AddDefaultVisualization(builder, meshcat)
    
    # Build the diagram and return it
    diagram = builder.Build()
    return diagram

# Function to run the simulation
def run_simulation(sim_time_step):
    diagram = create_scene_directives(model_directive, sim_time_step)
    simulator = Simulator(diagram)
    meshcat.StartRecording()
    simulator.AdvanceTo(2.0)
    meshcat.PublishRecording()

# Run the simulation with a timestep of 1ms
run_simulation(0.001)
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Interrupted")