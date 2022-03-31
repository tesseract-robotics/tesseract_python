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
from tesseract_robotics_viewer.tesseract_env_to_gltf import tesseract_env_to_gltf, tesseract_env_to_glb
import pkg_resources
import mimetypes
import posixpath
import traceback
import os
import numpy as np
import json
from tesseract_robotics import tesseract_environment
import hashlib
import base64
import sys
from tesseract_robotics.tesseract_command_language import isStateWaypoint, isMoveInstruction

if sys.version_info[0] < 3:
    from BaseHTTPServer import BaseHTTPRequestHandler
    from BaseHTTPServer import HTTPServer

else:
    from http.server import BaseHTTPRequestHandler    
    from http.server import ThreadingHTTPServer as HTTPServer

if not mimetypes.inited:
    mimetypes.init()

class _TesseractViewerRequestHandler(BaseHTTPRequestHandler):
    def __init__(self, request, client_address, server ):        
        self.scene_json = server.viewer.scene_json
        self.scene_glb = server.viewer.scene_glb
        self.trajectory_json = server.viewer.trajectory_json
        if sys.version_info[0] < 3:
            BaseHTTPRequestHandler.__init__(self,request,client_address,server)
        else:
            super(_TesseractViewerRequestHandler,self).__init__(request,client_address,server)

    def do_file(self, send_data):
        path = self.path.split("?")[0]
        if (self.path == "/"):
            path = "/index.html"
        path=path[1:]

        if path == 'tesseract_scene.gltf':
            if self.scene_json is None:
                file_data = "{}".encode()
            else:
                file_data = self.scene_json.encode()
        elif path == 'tesseract_scene.glb':
            if self.scene_glb is None:
                file_data = b""
            else:
                file_data = self.scene_glb
        elif path == 'tesseract_trajectory.json':
            if self.trajectory_json is None:
                self.send_response(404)
                return
            else:
                file_data = self.trajectory_json.encode()
        else:
            if '/' in path:
                self.send_response(404)
                return    
            try:
                file_data = pkg_resources.resource_string('tesseract_robotics_viewer.resources', "static/" + path)
            except:
                traceback.print_exc()
                self.send_response(404)
                return

        self.send_response(200)
        
        _, ext = posixpath.splitext(path)
        if ext == '.js':
            ctype = 'text/javascript'
        elif ext in mimetypes.types_map:
            ctype = mimetypes.types_map[ext]
        else:
            ctype = 'application/octet-stream'

        self.send_response(200)
        self.send_header("Content-type", ctype)
        self.send_header("Content-Length", len(file_data))
        if sys.version_info[0] < 3:
            self.send_header("ETag", "\"" + str(base64.b64encode(hashlib.sha1(file_data).digest())) + "\"")
        else:
            self.send_header("ETag", "\"" + str(base64.b64encode(hashlib.sha1(file_data).digest()), "utf-8") + "\"")
        
        self.end_headers()
        if send_data:
            self.wfile.write(file_data)
        


    def do_HEAD(self):
        self.do_file(False)

    def do_GET(self):            
        self.do_file(True)

    def log_message(self, format, *args):
        return


class TesseractViewer():
    def __init__(self, server_address = ('',8000)):
        self.scene_json = None
        self.scene_glb = None
        self.trajectory_json = None
        self._lock = threading.Lock()
        self._serve_thread = None
        self._httpd = HTTPServer(server_address,_TesseractViewerRequestHandler)
        setattr(self._httpd,"viewer",self)   

    def update_environment(self, tesseract_env, origin_offset = [0,0,0], trajectory = None):

        assert isinstance(tesseract_env, tesseract_environment.Environment)
        with self._lock:
            self.scene_json = tesseract_env_to_gltf(tesseract_env, origin_offset, trajectory=trajectory)
            self.scene_glb = tesseract_env_to_glb(tesseract_env, origin_offset)

    def update_joint_positions(self, joint_names, joint_positions):
        # Create "infinite" animation with constant joint angles

        trajectory_json = dict()
        trajectory_json["use_time"] = True
        trajectory_json["loop_time"] = 10000

        assert joint_names and all(isinstance(s,str) for s in joint_names), "Joint names must all be strings"
        trajectory_json["joint_names"] = joint_names
        assert isinstance(joint_positions,np.ndarray), "Expected numpy array for joint_positions"
        assert joint_positions.dtype == np.float64, "Expected double float array for joint_positions"
        joint_positions = list(joint_positions.flatten())
        assert len(joint_positions) == len(joint_names)
        trajectory_json["trajectory"] = [joint_positions + [0.0], joint_positions + [1e6]]

        self.trajectory_json=json.dumps(trajectory_json)

    def update_trajectory(self, tesseract_trajectory):
        
        start_instruction_o = tesseract_trajectory[0]
        assert isMoveInstruction(start_instruction_o)
        start_waypoint_o = start_instruction_o.as_MoveInstruction().getWaypoint()
        assert isStateWaypoint(start_waypoint_o)
        start_waypoint = start_waypoint_o.as_StateWaypoint()

        trajectory_json = dict()
        trajectory_json["use_time"] = True
        trajectory_json["loop_time"] = 20
        trajectory_json["joint_names"] = list(start_waypoint.joint_names)
        
        trajectory2 = []
        for i in range(len(tesseract_trajectory)):
            instr = tesseract_trajectory[i]
            assert isMoveInstruction(instr)
            wp = instr.as_MoveInstruction().getWaypoint()
            assert isStateWaypoint(wp)
            state_wp = wp.as_StateWaypoint()
            trajectory2.append(state_wp.position.flatten().tolist() + [state_wp.time])
        trajectory_json["trajectory"] = trajectory2
        self.trajectory_json=json.dumps(trajectory_json)
        
    def serve_forever(self):
        self._httpd.serve_forever()

    def start_serve_background(self):
        with self._lock:
            assert self._serve_thread is None, "Already serving"

            self._serve_thread = threading.Thread(target= lambda: self.serve_forever())
            self._serve_thread.daemon = True
            self._serve_thread.start()

    def shutdown_serve_background(self):
        with self._lock:
            if self._serve_thread is not None:
                self._httpd.shutdown()

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