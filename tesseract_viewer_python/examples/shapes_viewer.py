"""Simple shapes viewer example using tesseract_robotics with nanobind bindings."""

from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import ResourceLocator, SimpleLocatedResource
import os
import re
import traceback
from tesseract_robotics_viewer import TesseractViewer

shapes_urdf = """
<robot name="multipleshapes">

  <link name="world"/>
  <link name="cylinder_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="clyinder_joint" type="revolute">
    <parent link="world"/>
    <child link="cylinder_link"/>
    <axis xyz="0 1 0"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

  <link name="box_link">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="box_joint" type="revolute">
    <parent link="world"/>
    <child link="box_link"/>
    <origin xyz="1 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

  <link name="sphere_link">
    <visual>
      <geometry>
        <sphere radius="0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>

  <joint name="sphere_joint" type="revolute">
    <parent link="world"/>
    <child link="sphere_link"/>
    <origin xyz="2 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

</robot>
"""


class TesseractSupportResourceLocator(ResourceLocator):
    """Resource locator for tesseract_support package resources."""

    def __init__(self):
        super().__init__()

    def locateResource(self, url):
        try:
            if os.path.exists(url):
                return SimpleLocatedResource(url, url, self)

            url_match = re.match(r"^package:\/\/tesseract_support\/(.*)$", url)
            if url_match is None:
                print(f"url_match failed for: {url}")
                return None

            tesseract_support = os.environ.get("TESSERACT_SUPPORT_DIR")
            if tesseract_support is None:
                print("TESSERACT_SUPPORT_DIR not set")
                return None

            filename = os.path.join(tesseract_support, os.path.normpath(url_match.group(1)))
            return SimpleLocatedResource(url, filename, self)
        except Exception:
            traceback.print_exc()
            return None


def main():
    HEADLESS = os.environ.get("TESSERACT_HEADLESS", "0") == "1"

    t_env = Environment()

    # locator must be kept alive by maintaining a reference
    locator = TesseractSupportResourceLocator()
    t_env.init(shapes_urdf, locator)

    viewer = TesseractViewer()
    viewer.update_environment(t_env, [0, 0, 0])

    if not HEADLESS:
        viewer.start_serve_background()
        input("Press Enter to exit...")
    else:
        print("shapes_viewer.py: PASSED")


if __name__ == "__main__":
    main()
