# Based on https://github.com/robotraconteur/robotraconteur/blob/master/RobotRaconteurPython/setup.py.in

from setuptools import setup, Distribution
import os
from xml.etree import ElementTree as ET

tree = ET.parse(os.path.join(os.path.dirname(__file__), 'package.xml'))
root = tree.getroot()
version = root.find('version').text


setup(name='tesseract-robotics-viewer',
      version=version,
      description='Tesseract Viewer Python Library',
      author='John Wason',
      author_email='wason@wasontech.com',
      url='http://robotraconteur.com/',
      packages=['tesseract_robotics_viewer','tesseract_robotics_viewer.resources'],
      package_data={'tesseract_robotics_viewer.resources':['static/index.html','static/app.js','geometries.json']},
	  license='Apache-2.0',
	  install_requires=['numpy','tesseract_robotics'],
	  long_description='Tesseract Robotics viewer package for Python'
     )