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
        asyncio.run_coroutine_threadsafe(self._a_start(), self.loop).result()

    def close(self):
        # call _a_close() thread safe
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


    async def _a_update_environment(self, scene_gltf, scene_glb):
        if self.aio_viewer is not None:
            await self.aio_viewer._update_environment_gltf(scene_gltf, scene_glb)

    def update_environment(self, tesseract_env, origin_offset = [0,0,0], trajectory = None):

        assert isinstance(tesseract_env, tesseract_environment.Environment)
        with self._lock:
            self.scene_json = tesseract_env_to_gltf(tesseract_env, origin_offset, trajectory=trajectory)
            self.scene_glb = tesseract_env_to_glb(tesseract_env, origin_offset)
            asyncio.run_coroutine_threadsafe(self._a_update_environment(self.scene_json, self.scene_glb), self.loop).result()

    async def _a_set_trajectory(self, trajectory_json):
        if self.aio_viewer is not None:
            await self.aio_viewer._update_trajectory_json(trajectory_json)

    def update_joint_positions(self, joint_names, joint_positions):
        self.trajectory_json = joint_positions_to_trajectory_json(joint_names, joint_positions)
        asyncio.run_coroutine_threadsafe(self._a_set_trajectory(self.trajectory_json), self.loop).result()
        

    def update_trajectory(self, tesseract_trajectory):
        
        traj, joint_names = tesseract_trajectory_to_list(tesseract_trajectory)

        self.trajectory_json = trajectory_list_to_json(traj, joint_names)
        asyncio.run_coroutine_threadsafe(self._a_set_trajectory(self.trajectory_json), self.loop).result()
        
    def serve_forever(self):
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
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.add_axes_marker(position, quaternion, size, parent_link,name, tags, update_now), self.loop).result()
        else:
            return None
    
    def add_arrow_marker(self, direction, origin, length, color = None, parent_link = "world", name = None, tags=None, update_now=True, position=None, quaternion=None):
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.add_arrow_marker(direction, origin, length, color, parent_link, name, tags, update_now, position, quaternion), self.loop).result()
        else:
            return None
        
    def add_box_marker(self, position, quaternion, size, color = None, parent_link = "world", name = None, tags=None, update_now=True):
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.add_box_marker(position, quaternion, size, color, parent_link, name, tags, update_now), self.loop).result()
        else:
            return None
        
    def add_sphere_marker(self, position, radius, color = None, parent_link = "world", name = None, tags=None, update_now=True):
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.add_sphere_marker(position, radius, color, parent_link, name, tags, update_now), self.loop).result()
        else:
            return None
        
    def add_cylinder_marker(self, position, quaternion, length, radius, color = None, parent_link = "world", name = None, tags=None, update_now=True):
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.add_cylinder_marker(position, quaternion, length, radius, color, parent_link, name, tags, update_now), self.loop).result()
        else:
            return None
    
    # capsule marker
    def add_capsule_marker(self, position, quaternion, length, radius, color, parent_link = "world", name = None, tags=None, update_now=True):
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.add_capsule_marker(position, quaternion, length, radius, color, parent_link, name, tags, update_now), self.loop).result()
        else:
            return None
        
    # lines marker
    def add_lines_marker(self, vertices, color = None, linewidth = 1.0, parent_link = "world", name = None, tags=None, update_now=True):
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.add_lines_marker(vertices, color, linewidth, parent_link, name, tags, update_now), self.loop).result()
        else:
            return None
        
    def clear_all_markers(self):
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.clear_all_markers(), self.loop).result()
        else:
            return None
        
    def clear_markers_by_tags(self, tags, update_now=True):
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.clear_markers_by_tags(tags, update_now), self.loop).result()
        else:
            return None
        
    def clear_markers_by_name(self, name, update_now=True):
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.clear_markers_by_name(name, update_now), self.loop).result()
        else:
            return None
        
    def send_update_markers(self):
        if self.aio_viewer is not None:
            return asyncio.run_coroutine_threadsafe(self.aio_viewer.send_update_markers(), self.loop).result()
        else:
            return None


    def save(self, directory, overwrite=False):
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
        assert self.scene_json is not None, "Tesseract environment not set"
        s = self.scene_json
        if isinstance(s,str):
            s = s.encode()
        
        fmode = 'wb' if overwrite else 'xb'
        with open(fname,fmode) as f:
            f.write(s)

    def save_scene_glb(self, fname, overwrite=False):
        assert self.scene_json is not None, "Tesseract environment not set"
        s = self.scene_glb
                
        fmode = 'wb' if overwrite else 'xb'
        with open(fname,fmode) as f:
            f.write(s)