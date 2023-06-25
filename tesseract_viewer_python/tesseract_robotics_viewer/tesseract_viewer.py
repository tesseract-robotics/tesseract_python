# Copyright 2019 Wason Technology, LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: John Wason (wason@wasontech.com)
# Date: 12/10/2019

from __future__ import absolute_import

import threading
import time
from tesseract_robotics_viewer.tesseract_env_to_gltf import tesseract_env_to_gltf, tesseract_env_to_glb
import pkg_resources
import traceback
import os
import numpy as np
from tesseract_robotics import tesseract_environment
from .util import tesseract_trajectory_to_list, joint_positions_to_trajectory_json, trajectory_list_to_json
from . import tesseract_viewer_aio
import asyncio

class TesseractViewer():
    """
    A class for viewing Tesseract Environments and Trajectories in a web browser
    
    TesseractViewer uses ThreeJS to render the environment and trajectories in a web browser. It uses a websocket
    to communicate with the browser. The scene is converted to glTF format and sent to the browser. Trajectories
    are converted to JSON format and sent to the browser. The browser then renders the scene and trajectories.

    By default the server will listen on port 8000 on the local machine. Use http://localhost:8000 to view the
    scene in a web browser. 
    
    The server can be started in the background using the start_serve_background() method.

    This class provides a synchronous interface. The TessractViewerAIO class provides an asynchronous interface, and
    can be used with asyncio.

    :param server_address: The address to bind the websocket server to. Defaults to ('127.0.0.1',8000)
    :type server_address: Tuple[str,int], optional
    """
    def __init__(self, server_address = ('127.0.0.1',8000)):
        
        self.server_address = server_address
        self.aio_viewer = tesseract_viewer_aio.TesseractViewerAIO(self.server_address)

        self.scene_json = None
        self.scene_glb = None
        self.trajectory_json = None
        self._lock = threading.Lock()

        self._loop_ready_evt = threading.Event()

        self.loop_thread = threading.Thread(target=self._run)
        self.loop_thread.daemon = True
        self.loop_thread.start()

        self._loop_ready_evt.wait(timeout=5)

    async def _a_close(self):
        if self.aio_viewer is not None:
            await self.aio_viewer.close()
    
    def start_serve_background(self):
        """
        Start the web server in the background. This method returns immediately.
        """
        asyncio.run_coroutine_threadsafe(self._a_start(), self.loop).result()

    def close(self):
        """
        Close the web server and stop the background thread.
        """
        try:
            res = asyncio.run_coroutine_threadsafe(self._a_close(), self.loop)
            res.result()
        except:
            traceback.print_exc()

        if self.loop is not None:
            self.loop.stop()
            self.loop.close()

    async def _a_start(self):
        await self.aio_viewer.start()

    def _run(self):
        self.loop = asyncio.new_event_loop()
        self._loop_ready_evt.set()
        self.loop.run_forever()


    async def _a_update_environment(self, scene_gltf, scene_glb, t_env):
        if self.aio_viewer is not None:
            await self.aio_viewer._update_environment_gltf(scene_gltf, scene_glb, t_env)

    def update_environment(self, tesseract_env, origin_offset = [0,0,0], trajectory = None):
        """
        Update the environment from a tesseract_environment.Environment object. This must be called to load the
        environment, and after the environment changes.

        :param tesseract_env: The environment to load
        :type tesseract_env: tesseract_environment.Environment
        :param origin_offset: The offset of the origin in the world frame, defaults to [0,0,0]
        :type origin_offset: List[float], optional
        :param trajectory: The trajectory to display, defaults to None
        :type trajectory: tesseract_command_language.CompositeInstruction, optional
        """

        assert isinstance(tesseract_env, tesseract_environment.Environment)
        with self._lock:
            self.scene_json = tesseract_env_to_gltf(tesseract_env, origin_offset, trajectory=trajectory)
            self.scene_glb = tesseract_env_to_glb(tesseract_env, origin_offset)
            asyncio.run_coroutine_threadsafe(self._a_update_environment(self.scene_json, self.scene_glb, tesseract_env), self.loop).result()

    async def _a_set_trajectory(self, trajectory_json):
        if self.aio_viewer is not None:
            await self.aio_viewer._update_trajectory_json(trajectory_json)

    def update_joint_positions(self, joint_names, joint_positions):
        """
        Set the stationary joint positions of the robots. Will stop any animations.

        :param joint_names: The joint names
        :type joint_names: List[str]
        :param joint_positions: The joint positions
        :type joint_positions: Union[List[float], np.ndarray]
        """
        self.trajectory_json = joint_positions_to_trajectory_json(joint_names, joint_positions)
        asyncio.run_coroutine_threadsafe(self._a_set_trajectory(self.trajectory_json), self.loop).result()
        

    def update_trajectory(self, tesseract_trajectory):
        """
        Update the trajectory to display. The trajectory will be animated based on the timestamps in the trajectory.

        :param tesseract_trajectory: The trajectory to display
        :type tesseract_trajectory: tesseract_robotics.tesseract_command_language.CompositeInstruction
        """
        
        joint_names, traj = tesseract_trajectory_to_list(tesseract_trajectory)

        self.trajectory_json = trajectory_list_to_json(joint_names, traj)
        asyncio.run_coroutine_threadsafe(self._a_set_trajectory(self.trajectory_json), self.loop).result()

    def update_trajectory_list(self, joint_names, trajectory):
        """
        Update the trajectory from a list. Each entry in the trajectory should be the joint values and the timestamp
        as the last element. For instance, a robot with 6 joints would have a 7 element array for each entry in the
        list, the first 6 elements being the joint values, and the last element being the timestamp.

        :param joint_names: The joint names
        :type joint_names: List[str]
        :param trajectory: The trajectory to display
        :type trajectory: Union[List[List[float]], List[np.ndarray]]
        """

        self.trajectory_json = trajectory_list_to_json(joint_names, trajectory)
        asyncio.run_coroutine_threadsafe(self._a_set_trajectory(self.trajectory_json), self.loop).result()
        
    def serve_forever(self):
        """
        Serve the web page forever. This method blocks until the server is closed.
        """
        self.start_server_background()
        # wait for keyboard interrupt
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def add_axes_marker(self, position, quaternion, size=0.15, parent_link = "world", name = None, tags=None, update_now=True):
        """
        Add an axes marker to the scene. This is useful for visualizing coordinate frames and waypoints.

        :param position: The position of the marker
        :type position: Union[List[float], np.ndarray]
        :param quaternion: The orientation of the marker in (w,x,y,z) format
        :type quaternion: Union[List[float], np.ndarray]
        :param size: The size of the marker, defaults to 0.15
        :type size: float, optional
        :param parent_link: The name of the parent link, defaults to "world"
        :type parent_link: str, optional
        :param name: The name of the marker, defaults to a generated name
        :type name: str, optional
        :param tags: The tags for the marker, defaults to None
        :type tags: List[str], optional
        :param update_now: Whether to update the scene immediately, defaults to True. Set to False if adding multiple markers at once.
        :type update_now: bool, optional
        :return: The name of the marker
        :rtype: str
        """
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.add_axes_marker(position, quaternion, size, parent_link,name, tags, update_now), self.loop).result()
        else:
            return None
    
    def add_arrow_marker(self, direction, origin, length, color = None, parent_link = "world", name = None, tags=None, update_now=True, position=None, quaternion=None):
        """
        Add an arrow marker to the scene. 

        :param direction: The direction of the arrow as a unit vector
        :type direction: Union[List[float], np.ndarray]
        :param origin: The origin of the arrow
        :type origin: Union[List[float], np.ndarray]
        :param length: The length of the arrow
        :type length: float
        :param color: The color of the arrow, defaults to white
        :type color: Union[List[float], np.ndarray], optional
        :param parent_link: The name of the parent link, defaults to "world"
        :type parent_link: str, optional
        :param name: The name of the marker, defaults to a generated name
        :type name: str, optional
        :param tags: The tags for the marker, defaults to None
        :type tags: List[str], optional
        :param update_now: Whether to update the scene immediately, defaults to True. Set to False if adding multiple markers at once.
        :type update_now: bool, optional
        :param position: The position of the marker, defaults to origin
        :type position: Union[List[float], np.ndarray], optional
        :param quaternion: The orientation of the marker in (w,x,y,z) format, defaults to no rotation
        :type quaternion: Union[List[float], np.ndarray], optional
        :return: The name of the marker
        :rtype: str
        """
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.add_arrow_marker(direction, origin, length, color, parent_link, name, tags, update_now, position, quaternion), self.loop).result()
        else:
            return None
        
    def add_box_marker(self, position, quaternion, size, color = None, parent_link = "world", name = None, tags=None, update_now=True):
        """
        Add a box marker to the scene.

        :param position: The position of the marker
        :type position: Union[List[float], np.ndarray]
        :param quaternion: The orientation of the marker in (w,x,y,z) format
        :type quaternion: Union[List[float], np.ndarray]
        :param size: The size of the marker in (x,y,z) format
        :type size: Union[List[float], np.ndarray]
        :param color: The color of the marker, defaults to white
        :type color: Union[List[float], np.ndarray], optional
        :param parent_link: The name of the parent link, defaults to "world"
        :type parent_link: str, optional
        :param name: The name of the marker, defaults to a generated name
        :type name: str, optional
        :param tags: The tags for the marker, defaults to None
        :type tags: List[str], optional
        :param update_now: Whether to update the scene immediately, defaults to True. Set to False if adding multiple markers at once.
        :type update_now: bool, optional
        :return: The name of the marker
        :rtype: str
        """
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.add_box_marker(position, quaternion, size, color, parent_link, name, tags, update_now), self.loop).result()
        else:
            return None
        
    def add_sphere_marker(self, position, radius, color = None, parent_link = "world", name = None, tags=None, update_now=True):
        """
        Add a sphere marker to the scene.

        :param position: The position of the marker
        :type position: Union[List[float], np.ndarray]
        :param radius: The radius of the sphere
        :type radius: float
        :param color: The color of the marker, defaults to white
        :type color: Union[List[float], np.ndarray], optional
        :param parent_link: The name of the parent link, defaults to "world"
        :type parent_link: str, optional
        :param name: The name of the marker, defaults to a generated name
        :type name: str, optional
        :param tags: The tags for the marker, defaults to None
        :type tags: List[str], optional
        :param update_now: Whether to update the scene immediately, defaults to True. Set to False if adding multiple markers at once.
        :type update_now: bool, optional
        :return: The name of the marker
        :rtype: str
        """
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.add_sphere_marker(position, radius, color, parent_link, name, tags, update_now), self.loop).result()
        else:
            return None
        
    def add_cylinder_marker(self, position, quaternion, length, radius, color = None, parent_link = "world", name = None, tags=None, update_now=True):
        """
        Add a cylinder marker to the scene.

        :param position: The position of the marker
        :type position: Union[List[float], np.ndarray]
        :param quaternion: The orientation of the marker in (w,x,y,z) format
        :type quaternion: Union[List[float], np.ndarray]
        :param length: The length of the cylinder
        :type length: float
        :param radius: The radius of the cylinder
        :type radius: float
        :param color: The color of the marker, defaults to white
        :type color: Union[List[float], np.ndarray], optional
        :param parent_link: The name of the parent link, defaults to "world"
        :type parent_link: str, optional
        :param name: The name of the marker, defaults to a generated name
        :type name: str, optional
        :param tags: The tags for the marker, defaults to None
        :type tags: List[str], optional
        :param update_now: Whether to update the scene immediately, defaults to True. Set to False if adding multiple markers at once.
        :type update_now: bool, optional
        :return: The name of the marker
        :rtype: str
        """
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.add_cylinder_marker(position, quaternion, length, radius, color, parent_link, name, tags, update_now), self.loop).result()
        else:
            return None
    
    # capsule marker
    def add_capsule_marker(self, position, quaternion, length, radius, color, parent_link = "world", name = None, tags=None, update_now=True):
        """
        Add a capsule marker to the scene.

        :param position: The position of the marker
        :type position: Union[List[float], np.ndarray]
        :param quaternion: The orientation of the marker in (w,x,y,z) format
        :type quaternion: Union[List[float], np.ndarray]
        :param length: The length of the capsule
        :type length: float
        :param radius: The radius of the capsule
        :type radius: float
        :param color: The color of the marker, defaults to white
        :type color: Union[List[float], np.ndarray], optional
        :param parent_link: The name of the parent link, defaults to "world"
        :type parent_link: str, optional
        :param name: The name of the marker, defaults to a generated name
        :type name: str, optional
        :param tags: The tags for the marker, defaults to None
        :type tags: List[str], optional
        :param update_now: Whether to update the scene immediately, defaults to True. Set to False if adding multiple markers at once.
        :type update_now: bool, optional
        :return: The name of the marker
        :rtype: str
        """
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.add_capsule_marker(position, quaternion, length, radius, color, parent_link, name, tags, update_now), self.loop).result()
        else:
            return None
        
    # lines marker
    def add_lines_marker(self, vertices, color = None, linewidth = 1.0, parent_link = "world", name = None, tags=None, update_now=True):
        """
        Add line segments marker to the scene.

        :param vertices: The vertices of the line segments
        :type vertices: Union[List[List[float]], List[np.ndarray]]
        :param color: The color of the marker, defaults to white
        :type color: Union[List[float], np.ndarray], optional
        :param linewidth: The width of the lines, defaults to 1.0
        :type linewidth: float, optional
        :param parent_link: The name of the parent link, defaults to "world"
        :type parent_link: str, optional
        :param name: The name of the marker, defaults to a generated name
        :type name: str, optional
        :param tags: The tags for the marker, defaults to None
        :type tags: List[str], optional
        :param update_now: Whether to update the scene immediately, defaults to True. Set to False if adding multiple markers at once.
        :type update_now: bool, optional
        :return: The name of the marker
        :rtype: str
        """
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.add_lines_marker(vertices, color, linewidth, parent_link, name, tags, update_now), self.loop).result()
        else:
            return None
        
    def clear_all_markers(self):
        """
        Clear all markers from the scene.
        """
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.clear_all_markers(), self.loop).result()
        else:
            return None
        
    def clear_markers_by_tags(self, tags, update_now=True):
        """
        Clear all markers with the given tags from the scene.

        :param tags: The tags to clear
        :type tags: List[str]
        :param update_now: Whether to update the scene immediately, defaults to True. Set to False if clearing multiple markers at once.
        :type update_now: bool, optional
        """
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.clear_markers_by_tags(tags, update_now), self.loop).result()
        else:
            return None
        
    def clear_markers_by_name(self, name, update_now=True):
        """
        Clear all markers with the given name from the scene.

        :param name: The name to clear
        :type name: str
        :param update_now: Whether to update the scene immediately, defaults to True. Set to False if clearing multiple markers at once.
        :type update_now: bool, optional
        """
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.clear_markers_by_name(name, update_now), self.loop).result()
        else:
            return None
        
    def send_update_markers(self):
        """
        Send an update to the scene to update all markers.
        """
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.send_update_markers(), self.loop).result()
        else:
            return None
        
    def plot_trajectory(self, tesseract_trajectory, manipulator_info, color = None, linewidth = 0.001, axes = True, axes_length = 0.1, tags = None, update_now=True):
        """
        Plot a trajectory stored in a tesseract_robotics.tesseract_command_language.CompositeInstruction to the scene. 
        This will draw the trajectory
        as a series of line segments and display axes at each trajectory waypoint.

        :param tesseract_trajectory: The trajectory to plot
        :type tesseract_trajectory: tesseract_robotics.tesseract_command_language.CompositeInstruction
        :param manipulator_info: The manipulator info for the manipulator that generated the trajectory
        :type manipulator_info: tesseract_robotics.tesseract_kinematics.ManipulatorInfo
        :param color: The color of the trajectory, defaults to white
        :type color: Union[List[float], np.ndarray], optional
        :param linewidth: The width of the trajectory line segments, defaults to 0.001
        :type linewidth: float, optional
        :param axes: Whether to display axes at each trajectory waypoint, defaults to True
        :type axes: bool, optional
        :param axes_length: The length of the axes, defaults to 0.1
        :type axes_length: float, optional
        :param tags: The tags for the trajectory, defaults to None
        :type tags: List[str], optional
        :param update_now: Whether to update the scene immediately, defaults to True. Set to False if plotting multiple trajectories at once.
        :type update_now: bool, optional
        :return: The name of the trajectory
        :rtype: str
        """
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.plot_trajectory(tesseract_trajectory, manipulator_info, color, linewidth, axes, axes_length, tags, update_now), self.loop).result()
        else:
            return None
        
    def plot_trajectory_list(self, joint_names, trajectory, manipulator_info, color = None, linewidth = 0.001, axes = True, axes_length = 0.1, tags = None, update_now = True):
        """
        Plot a trajectory stored as a list of joint positions to the scene. This will draw the trajectory
        as a series of line segments and display axes at each trajectory waypoint.

        :param joint_names: The joint names for the trajectory
        :type joint_names: List[str]
        :param trajectory: The trajectory to plot
        :type trajectory: Union[List[List[float]], List[np.ndarray]]
        :param manipulator_info: The manipulator info for the manipulator that generated the trajectory
        :type manipulator_info: tesseract_robotics.tesseract_kinematics.ManipulatorInfo
        :param color: The color of the trajectory, defaults to white
        :type color: Union[List[float], np.ndarray], optional
        :param linewidth: The width of the trajectory line segments, defaults to 0.001
        :type linewidth: float, optional
        :param axes: Whether to display axes at each trajectory waypoint, defaults to True
        :type axes: bool, optional
        :param axes_length: The length of the axes, defaults to 0.1
        :type axes_length: float, optional
        :param tags: The tags for the trajectory, defaults to None
        :type tags: List[str], optional
        :param update_now: Whether to update the scene immediately, defaults to True. Set to False if plotting multiple trajectories at once.
        :type update_now: bool, optional
        :return: The name of the trajectory
        :rtype: str
        """
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.plot_trajectory_list(joint_names, trajectory, manipulator_info, color, linewidth, axes, axes_length, tags, update_now), self.loop).result()
        else:
            return None


    def save(self, directory, overwrite=False):
        """
        Save the current scene to the given directory. This will create a directory with the given name and
        save the scene as a set of files in that directory. The scene can be loaded using a web browser offline.

        :param directory: The directory to save the scene to
        :type directory: str
        :param overwrite: Whether to overwrite existing files, defaults to False
        :type overwrite: bool, optional
        """
        assert os.path.isdir(directory), "Invalid target directory %s" % directory
        assert self.scene_json is not None, "Tesseract environment not set"

        index_html = pkg_resources.resource_string('tesseract_robotics_viewer.resources', "static/index.html")
        tesseract_viewer_js = pkg_resources.resource_string('tesseract_robotics_viewer.resources', "static/tesseract_viewer.js")

        files = {"index.html": index_html, "tesseract_viewer.js": tesseract_viewer_js,
            "tesseract_scene.babylon": self.scene_json.encode()}
        if self.trajectory_json is not None:
            files["tesseract_trajectory.json"] = self.trajectory_json.encode()
        if not overwrite:
            for k in files:
                assert not os.path.exists(k), "File already exists"

        for k,v in files.items():
            with open(os.path.join(directory,k), "wb") as f:
                f.write(v)

    def save_scene_gltf(self, fname, overwrite=False):
        """
        Save the scene to a glTF 2 file.

        :param fname: The file name to save to
        :type fname: str
        :param overwrite: Whether to overwrite existing files, defaults to False
        :type overwrite: bool, optional
        """
        assert self.scene_json is not None, "Tesseract environment not set"
        s = self.scene_json
        if isinstance(s,str):
            s = s.encode()
        
        fmode = 'wb' if overwrite else 'xb'
        with open(fname,fmode) as f:
            f.write(s)

    def save_scene_glb(self, fname, overwrite=False):
        """
        Save the scene to a glTF 2 binary file.

        :param fname: The file name to save to
        :type fname: str
        :param overwrite: Whether to overwrite existing files, defaults to False
        :type overwrite: bool, optional
        """
        assert self.scene_json is not None, "Tesseract environment not set"
        s = self.scene_glb
                
        fmode = 'wb' if overwrite else 'xb'
        with open(fname,fmode) as f:
            f.write(s)