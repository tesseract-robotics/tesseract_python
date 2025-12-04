"""Mesh viewer example demonstrating GLB and DAE mesh loading with materials."""

import sys
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics_viewer import TesseractViewer
import os

shapes_urdf = """
<robot name="mesh_viewer">

  <link name="world"/>
  <link name="mesh_gltf2_link">
    <visual>
      <geometry>
        <mesh filename="package://tesseract_support/meshes/tesseract_material_mesh.glb"/>
      </geometry>
    </visual>
  </link>

  <joint name="mesh_gltf2_joint" type="revolute">
    <parent link="world"/>
    <child link="mesh_gltf2_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0 -1 0.25"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

  <link name="mesh_dae_link">
    <visual>
      <geometry>
        <mesh filename="package://tesseract_support/meshes/tesseract_material_mesh.dae"/>
      </geometry>
    </visual>
  </link>

  <joint name="mesh_dae_joint" type="revolute">
    <parent link="world"/>
    <child link="mesh_dae_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0 1 0.25"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

</robot>
"""


def main():
    HEADLESS = os.environ.get("TESSERACT_HEADLESS", "0") == "1" or "pytest" in sys.modules

    t_env = Environment()

    # GeneralResourceLocator uses TESSERACT_RESOURCE_PATH env var
    locator = GeneralResourceLocator()
    t_env.init(shapes_urdf, locator)

    if not HEADLESS:
        viewer = TesseractViewer()
        viewer.update_environment(t_env, [0, 0, 0])
        viewer.start_serve_background()
        input("Press Enter to exit...")
    else:
        print("tesseract_material_mesh_viewer.py: PASSED")


if __name__ == "__main__":
    main()
