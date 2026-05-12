# SSL may be necessary for some systems that use WebXR. By default WebXR is not allowed to run on insecure origins.
# The user will need to accept the security warning for a self signed certificate.
# Also allow all incoming addresses to connect to the server by binding to 0.0.0.0 instead of localhost.
# Firewalls may also need to be configured to allow incoming connections.
# Point your VR headset to the https address of the computer running the server. For example
# https://192.168.1.10:8000 if the server is running on port 8000 on the computer with IP address
# 192.168.1.10. 
#
# To generate a self-signed certificate, run the following command (openssl must be installed):
#
# openssl req -newkey rsa:2048 -nodes -keyout key.pem -x509 -days 365 -out cert.pem
#
# When viewed on a VR headset web browser, click the "Enter VR" button to enter VR mode.

from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import GeneralResourceLocator
import os
import re
import traceback
from tesseract_robotics_viewer import TesseractViewer
import numpy as np
import time
import sys
import ssl

shapes_urdf="""
<robot name="multipleshapes" tesseract:make_convex="true">
  
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


t_env = Environment()

# locator must be kept alive by maintaining a reference
locator=GeneralResourceLocator()
t_env.init(shapes_urdf, locator)

ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
ssl_context.load_cert_chain(certfile='cert.pem', keyfile='key.pem')
viewer = TesseractViewer(("0.0.0.0", 8000), ssl_context)

viewer.update_environment(t_env, [0,0,0])

viewer.start_serve_background()

input("press enter")

