
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

from .tesseract_env_to_gltf import tesseract_env_to_gltf, tesseract_env_to_glb
from .util import tesseract_trajectory_to_list, trajectory_list_to_json, trajectory_list_to_frames
import importlib_resources
import asyncio
import hashlib
import weakref

import aiohttp
from aiohttp import web as aiohttp_web

if not mimetypes.inited:
    mimetypes.init()

class _TesseractViewerAIOServer:
    def __init__(self):
        self._static_pkg = importlib_resources.files('tesseract_robotics_viewer.resources.static')
        self.scene_gltf = None
        self.scene_glb = None
        self.trajectory_json = None
        self.scene_gltf_etag = self.hash_bytes(self.scene_gltf)
        self.scene_glb_etag = self.hash_bytes(self.scene_glb)
        self.trajectory_json_etag = self.hash_bytes(self.trajectory_json)
        self.markers_json = None
        self.markers_json_etag = self.hash_bytes(self.markers_json)
        self._ws = []
        self._ws_send_lock = asyncio.Lock()

    def hash_bytes(self, data):
        if data is None:
            return "None"
        if isinstance(data, str):
            data = data.encode("utf8")
        return hashlib.sha256(data).hexdigest()

    async def start(self, host="127.0.0.1", port=8000):
        try:
            self._app = aiohttp_web.Application()
            self._app.add_routes([aiohttp_web.get("/", self.index), aiohttp_web.get("/index.html", self.index)])
            self._app.add_routes([aiohttp_web.get("/app.js", self.app_js)])
            self._app.add_routes([aiohttp_web.get("/tesseract_scene.gltf", self.tesseract_scene_gltf)])
            self._app.add_routes([aiohttp_web.get("/tesseract_scene.glb", self.tesseract_scene_glb)])
            self._app.add_routes([aiohttp_web.get("/tesseract_trajectory.json", self.tesseract_trajectory_json)])
            self._app.add_routes([aiohttp_web.get("/tesseract_markers.json", self.tesseract_markers_json)])
            self._app.add_routes([aiohttp_web.head("/tesseract_scene.gltf", self.tesseract_scene_gltf_head)])
            self._app.add_routes([aiohttp_web.head("/tesseract_scene.glb", self.tesseract_scene_glb_head)])
            self._app.add_routes([aiohttp_web.head("/tesseract_trajectory.json", self.tesseract_trajectory_json_head)])
            self._app.add_routes([aiohttp_web.head("/tesseract_markers.json", self.tesseract_markers_json_head)])
            self._app.add_routes([aiohttp_web.get('/websocket', self.websocket_handler)])

            self._runner = aiohttp_web.AppRunner(self._app)
            await self._runner.setup()
            self._site = aiohttp_web.TCPSite(self._runner, host, port)
            await self._site.start()

        except:
            traceback.print_exc()
            raise
                
    async def index(self, request):
        return await self.get_static_file("index.html")

    async def app_js(self, request):
        return await self.get_static_file("app.js")
    
    async def tesseract_scene_gltf(self, request):
        if self.scene_gltf is not None:
            r = aiohttp_web.Response(body=self.scene_gltf, content_type="model/gltf+json")
        else:            
            r = aiohttp_web.Response(body=b'{}', content_type="model/gltf+json")
        r.etag = self.scene_gltf_etag
        return r
        
    async def tesseract_scene_gltf_head(self, request):
        
        r = aiohttp_web.Response(content_type="model/gltf+json")
        r.etag = self.scene_gltf_etag
        return r
    
    async def tesseract_scene_glb(self, request):
        if self.scene_glb is not None:
            r = aiohttp_web.Response(body=self.scene_glb, content_type="application/octet-stream")
        else:
            r = aiohttp_web.Response(body=b'', content_type="application/octet-stream")
        r.etag = self.scene_glb_etag
        return r
        
    async def tesseract_scene_glb_head(self, request):
        
        r = aiohttp_web.Response(content_type="application/octet-stream")
        r.etag = self.scene_glb_etag
        return r
    
    async def tesseract_trajectory_json(self, request):
        if self.trajectory_json is not None:
            r = aiohttp_web.Response(body=self.trajectory_json, content_type="application/json")
            r.etag = self.trajectory_json_etag
            return r
        return aiohttp_web.Response(status=404)
    
    async def tesseract_trajectory_json_head(self, request):
        if self.trajectory_json is not None:
            r = aiohttp_web.Response(content_type="application/json")
            r.etag = self.trajectory_json_etag
            return
        return aiohttp_web.Response(status=404)
    
    async def tesseract_markers_json(self, request):
        if self.markers_json is not None:
            r = aiohttp_web.Response(body=self.markers_json, content_type="application/json")
            r.etag = self.markers_json_etag
            return r
        return aiohttp_web.Response(status=404)
    
    async def tesseract_markers_json_head(self, request):
        if self.markers_json is not None:
            r = aiohttp_web.Response(content_type="application/json")
            r.etag = self.markers_json_etag
            return r
        return aiohttp_web.Response(status=404)
    
    async def websocket_handler(self, request):
        ws = aiohttp_web.WebSocketResponse()
        await ws.prepare(request)

        # Create an instance of WebSocketHandler for the connection
        handler = _TesseractViewerAIOWebsocketConnection(ws)
        await handler.start()

        self._ws.append(ws)

        await handler.message_listener()
    
    async def send_ws_message(self, msg):
        async with self._ws_send_lock:
            remove_list = []
            for ws in self._ws:
                try:
                    await ws.send_str(msg)
                except:
                    traceback.print_exc()
                    remove_list.append(ws)
            for ws in remove_list:
                self._ws.remove(ws)

    async def close_ws(self):
        for ws in self._ws:
            await ws.close()
        self._ws = []

    async def get_static_file(self, filename):
        with importlib_resources.as_file(self._static_pkg / filename) as f_path:
            with open(f_path, "rb") as f:
                contents = f.read()
        return aiohttp_web.Response(body=contents, content_type=mimetypes.guess_type(filename)[0])
    
    async def set_environment(self, scene_gltf, scene_glb):
        self.scene_gltf = scene_gltf
        self.scene_glb = scene_glb
        self.scene_gltf_etag = self.hash_bytes(self.scene_gltf)
        self.scene_glb_etag = self.hash_bytes(self.scene_glb)

        await self.send_ws_message(json.dumps({
            "command": "refresh_scene",
        }))

    async def set_trajectory(self, trajectory_json):
        self.trajectory_json = trajectory_json
        self.trajectory_json_etag = self.hash_bytes(self.trajectory_json)
        await self.send_ws_message(json.dumps({
            "command": "joint_trajectory",
            "params": json.loads(trajectory_json)
        }))

    async def set_markers_json(self, markers_json, update_now = True):
        self.markers_json = markers_json
        self.markers_json_etag = self.hash_bytes(self.markers_json)
        if update_now:
            await self.send_ws_message(json.dumps({
                "command": "markers",
                "params": json.loads(markers_json)
            }))

def _fix_marker_vector(v):
    if isinstance(v, list):
        return v
    if isinstance(v, np.ndarray):
        return v.tolist()
    return v

def _fix_marker_color(c):
    ret = []
    if isinstance(c, list):
        ret = c.copy()
    elif isinstance(c, np.ndarray):
        ret = c.flatten().tolist()
    else:
        return c
    if len(ret) == 3:
        ret.append(1.0)
    return ret

def _fix_marker_quaternion(q):
    if isinstance(q, list):
        return q
    if isinstance(q, np.ndarray):
        return q.flatten().tolist()
    return q
    


class _TesseractViewerAIOWebsocketConnection:

    def __init__(self, ws):
        self.ws = ws
        self.mutex = asyncio.Lock()
        self.loop = asyncio.get_event_loop()
        
    async def start(self):
        # Start listening for incoming messages
        # self.loop.create_task(self.message_listener())
        print("Websocket connected!")

    async def message_listener(self):
        try:
            async for msg in self.ws:
                await self.handle_message(msg)
        except aiohttp.ClientConnectionError:
            await self.handle_close()

    async def handle_message(self, msg: aiohttp.WSMessage):
        if msg.type == aiohttp.WSMsgType.TEXT:
            await self.process_text_message(msg.data)
        elif msg.type == aiohttp.WSMsgType.BINARY:
            await self.process_binary_message(msg.data)
        elif msg.type == aiohttp.WSMsgType.CLOSE:
            await self.handle_close()

    async def process_text_message(self, data: str):
        # Process incoming text message
        # ...

        # Send a response
        # await self.send('Response message')
        print("WebSocket Connection received message: " + data)

    async def process_binary_message(self, data: bytes):
        # Process incoming binary message
        # ...

        # Send a response
        # await self.send(b'Response message')
        print("WebSocket Connection received binary message")

    async def handle_close(self):
        # Handle WebSocket close event
        # ...
        print("WebSocket Connection closed")

    async def send(self, message: aiohttp.WSMessage):
        async with self.mutex:
            await self.ws.send_str(message)

    async def close(self):
        async with self.mutex:
            await self.ws.close()

class TesseractViewerAIO:
    """
    A class for viewing Tesseract environments in a web browser using a Python asyncio server.

    TesseractViewer uses ThreeJS to render the environment and trajectories in a web browser. It uses a websocket
    to communicate with the browser. The scene is converted to glTF format and sent to the browser. Trajectories
    are converted to JSON format and sent to the browser. The browser then renders the scene and trajectories.

    By default the server will listen on port 8000 on the local machine. Use http://localhost:8000 to view the
    scene in a web browser. 
    
    The server can be started and stopped using the start() and close() methods. A task is created to run the server
    in the background. The server will continue to run until close() is called.

    :param server_address: The address to listen on. Default is localhost:8000.
    :type server_address: tuple
    """
    def __init__(self, server_address = ("localhost", 8080)):
        self.server_address = server_address
        self.server = _TesseractViewerAIOServer()
        self._lock = asyncio.Lock()
        self.server_task = None
        self.marker_count = 1
        self.markers = []
        self.t_env = None

    async def update_environment(self, tesseract_env, origin_offset = [0,0,0], trajectory = None):
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
        async with self._lock:
            self.scene_json = tesseract_env_to_gltf(tesseract_env, origin_offset, trajectory=trajectory)
            self.scene_glb = tesseract_env_to_glb(tesseract_env, origin_offset)
            await self.server.set_environment(self.scene_json, self.scene_glb)
    
    async def _update_environment_gltf(self, scene_glft, scene_glb, t_env):
        async with self._lock:
            self.scene_json = scene_glft
            self.scene_glb = scene_glb
            self.t_env = weakref.ref(t_env)
            await self.server.set_environment(self.scene_json, self.scene_glb)

    async def update_trajectory(self, trajectory):
        """
        Update the trajectory to display. The trajectory will be animated based on the timestamps in the trajectory.

        :param tesseract_trajectory: The trajectory to display
        :type tesseract_trajectory: tesseract_robotics.tesseract_command_language.CompositeInstruction
        """
        async with self._lock:
            joint_names, traj = tesseract_trajectory_to_list(trajectory)
            self.trajectory_json = trajectory_list_to_json(joint_names, traj)
            await self.server.set_trajectory(self.trajectory_json)
    
    async def update_trajectory_list(self, joint_names, trajectory):
        """
        Update the trajectory from a list. Each entry in the trajectory should be the joint values and the timestamp
        as the last element. For instance, a robot with 6 joints would have a 7 element array for each entry in the
        list, the first 6 elements being the joint values, and the last element being the timestamp.

        :param joint_names: The joint names
        :type joint_names: List[str]
        :param trajectory: The trajectory to display
        :type trajectory: Union[List[List[float]], List[np.ndarray]]
        """
        async with self._lock:            
            self.trajectory_json = trajectory_list_to_json(joint_names, trajectory)
            await self.server.set_trajectory(self.trajectory_json)

    async def _update_trajectory_json(self, trajectory_json):
        async with self._lock:
            self.trajectory_json = trajectory_json
            await self.server.set_trajectory(self.trajectory_json)

    async def start(self):
        """
        Start the server. This will start the server in the background. The server will continue to run until
        close() is called.
        """
        async with self._lock:
            self.server_task = asyncio.create_task(self.server.start(self.server_address[0], self.server_address[1]))

    async def close(self):
        """
        Close the server. This will stop the server and wait for it to finish.
        """
        if self.server_task is not None:
            async with self._lock:
                await self.server.close()
                await self.server_task

    def _new_marker_dict(self, marker_type, parent_link, position, quaternion, name, color, tags):
        if name is None:
            name = f"axes_marker_{self.marker_count}"
        self.marker_count += 1
        if tags is None:
            tags = []
        if color is None:
            color = [1,1,1,1]
        return {
            "marker_type": marker_type,
            "parent_link_name": parent_link,
            "position": _fix_marker_vector(position),
            "quaternion": _fix_marker_quaternion(quaternion),
            "name": name,
            "color": _fix_marker_color(color),
            "tags": tags
        }
    
    async def _do_append_marker(self, marker, update_now):
        self.markers.append(marker)
        await self._update_markers(self.markers, update_now)
        return marker["name"]

    async def add_axes_marker(self, position, quaternion, size=0.15, parent_link = "world", name = None, tags = None, update_now = True):
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
        async with self._lock:            
            marker = self._new_marker_dict("axes", parent_link, position, quaternion, name, None, tags)
            marker["size"] = size
            return await self._do_append_marker(marker, update_now)
        
    async def add_arrow_marker(self, direction, origin, length, color = None, parent_link = "world",name=None, tags=None, update_now = True, position = None, quaternion = None):
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
        if position is None:
            position = [0,0,0]
        if quaternion is None:
            quaternion = [1,0,0,0]

        async with self._lock:
            marker = self._new_marker_dict("arrow", parent_link, position, quaternion, name, color, tags)
            marker["direction"] = _fix_marker_vector(direction)
            marker["origin"] = _fix_marker_vector(origin)
            marker["length"] = length
            return await self._do_append_marker(marker, update_now)
        
    async def add_box_marker(self, position, quaternion, size, color = None, parent_link = "world",  name=None, tags=None, update_now = True):
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
        async with self._lock:
            marker = self._new_marker_dict("box", parent_link, position, quaternion, name, color, tags)
            marker["size"] = _fix_marker_vector(size)
            return await self._do_append_marker(marker, update_now)
        
    async def add_sphere_marker(self, position, radius, color = None, parent_link = "world",  name=None, tags=None, update_now = True):
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
        async with self._lock:
            quaternion = [1,0,0,0]
            marker = self._new_marker_dict("sphere", parent_link, position, quaternion, name, color, tags)
            marker["radius"] = radius
            return await self._do_append_marker(marker, update_now)
        
    async def add_cylinder_marker(self, position, quaternion, radius, length, color = None, parent_link = "world", name=None, tags=None, update_now = True):
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
        async with self._lock:
            marker = self._new_marker_dict("cylinder", parent_link, position, quaternion, name, color, tags)
            marker["radius"] = radius
            marker["length"] = length
            return await self._do_append_marker(marker, update_now)
        
    async def add_capsule_marker(self, position, quaternion, radius, length, color = None, parent_link = "world", name=None, tags=None, update_now = True):
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
        async with self._lock:
            marker = self._new_marker_dict("capsule", parent_link, position, quaternion, name, color, tags)
            marker["radius"] = radius
            marker["length"] = length
            return await self._do_append_marker(marker, update_now)
        
    async def add_lines_marker(self, vertices, color = None, linewidth = 1.0, parent_link = "world", name=None, tags=None, update_now = True, position = None, quaternion = None):
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
        if position is None:
            position = [0,0,0]
        if quaternion is None:
            quaternion = [1,0,0,0]

        async with self._lock:
            marker = self._new_marker_dict("lines", parent_link, position, quaternion, name, color, tags)
            marker["vertices"] = [_fix_marker_vector(p) for p in vertices]
            marker["linewidth"] = linewidth
            return await self._do_append_marker(marker, update_now)
    
    async def _update_markers(self, markers, update_now):
        self.markers_json = json.dumps({"markers": markers})
        await self.server.set_markers_json(self.markers_json, update_now)

    async def _set_markers_json(self, markers_json):
        async with self._lock:
            self._update_markers(markers_json)

    async def clear_all_markers(self):
        """
        Clear all markers from the scene.
        """
        async with self._lock:
            self.markers = []
            await self._update_markers(self.markers, True)

    async def clear_markers_by_tags(self, tags, update_now = True):
        """
        Clear all markers with the given tags from the scene.

        :param tags: The tags to clear
        :type tags: List[str]
        :param update_now: Whether to update the scene immediately, defaults to True. Set to False if clearing multiple markers at once.
        :type update_now: bool, optional
        """
        async with self._lock:
            self.markers = [m for m in self.markers if m["tags"] != tags]
            await self._update_markers(self.markers, update_now)

    async def clear_markers_by_name(self, name, update_now = True):
        """
        Clear all markers with the given name from the scene.

        :param name: The name to clear
        :type name: str
        :param update_now: Whether to update the scene immediately, defaults to True. Set to False if clearing multiple markers at once.
        :type update_now: bool, optional
        """
        async with self._lock:
            self.markers = [m for m in self.markers if m["name"] != name]
            await self._update_markers(self.markers, update_now)

    async def plot_trajectory(self, trajectory, manipulator_info, color = None, linewidth = 0.001, axes = True, axes_length = 0.1, tags = None, update_now = True):
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
        joint_names, trajectory = tesseract_trajectory_to_list(trajectory)
        return await self.plot_trajectory_list(joint_names, trajectory, manipulator_info, color, linewidth, axes, axes_length, tags, update_now)            

    async def plot_trajectory_list(self, joint_names, trajectory, manipulator_info, color = None, linewidth = 0.001, axes = True, axes_length = 0.1, tags = None, update_now = True):
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
        async with self._lock:
            if self.t_env is None:
                return None
            t_env = self.t_env()
            if t_env is None:
                return None
            if tags is None:
                tags = []
            tags.append("trajectory")
            if color is None:
                color = [0,1,0,1]
            trajectory_frames = trajectory_list_to_frames(t_env, manipulator_info, joint_names, trajectory)
            root_link = t_env.getRootLinkName()
            points = []
            for i in range(len(trajectory_frames)):
                points.append(trajectory_frames[i][0])
            line_marker = self._new_marker_dict("lines", root_link, [0,0,0], [1,0,0,0], None, color, tags)
            line_marker["vertices"] = [_fix_marker_vector(p) for p in points]
            line_marker["linewidth"] = linewidth
            await self._do_append_marker(line_marker, update_now)
            if axes:
                for i in range(len(trajectory_frames)):
                    axes_marker = self._new_marker_dict("axes", root_link, trajectory_frames[i][0], trajectory_frames[i][1], None, None, tags)
                    axes_marker["size"] = axes_length
                    await self._do_append_marker(axes_marker, False)
            await self._update_markers(self.markers, update_now)

async def amain():
    t = _TesseractViewerAIOServer()
    await t.start()

    # wait for exit
    for i in range(1000):
        await asyncio.sleep(1)
        try:
            axes_cmd = {
                "command": "add_axes_marker",
                "params": {
                    "name": f"axes{i}",
                    "position": [0, 0, (i%20)*.1],
                    "quaternion": [0, 0, 0, 1],
                    "size": 0.15,
                    "parent_link": "world"
                }
            }
            await t.send_ws_message(json.dumps(axes_cmd))
            pass
        except:
            traceback.print_exc()

    await t.close_ws()

    while True:
        await asyncio.sleep(1)


if __name__ == "__main__":
    asyncio.run(amain())