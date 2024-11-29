import * as THREE from 'https://unpkg.com/three@0.153.0/build/three.module.js';

import { OrbitControls } from 'https://unpkg.com/three@0.153.0/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from 'https://unpkg.com/three@0.153.0/examples/jsm/loaders/GLTFLoader.js';
import { VRButton } from 'https://unpkg.com/three@0.153.0/examples/jsm/webxr/VRButton.js';
import { LineMaterial } from 'https://unpkg.com/three@0.153.0/examples/jsm/lines/LineMaterial.js'
import { Line2 } from 'https://unpkg.com/three@0.153.0/examples/jsm/lines/Line2.js'
import { LineGeometry } from 'https://unpkg.com/three@0.153.0/examples/jsm/lines/LineGeometry.js'
import { XRControllerModelFactory } from 'https://unpkg.com/three@0.153.0/examples/jsm/webxr/XRControllerModelFactory.js';
import 'https://cdn.jsdelivr.net/npm/robust-websocket@1.0.0/robust-websocket.min.js';

class TesseractViewer {

    constructor()
    {
        this._scene = null;
        this._clock = null;
        this._camera = null;
        this._renderer = null;
        this._light = null;
        this._scene_etag = null;
        this._trajectory_etag = null;
        this._disable_update_trajectory = false;
        this._disable_update_markers = false;
        this._markers_etag = null;
        this._animation_mixer = null;
        this._animation = null;
        this._animation_action = null;
        this._root_z_up = null;
        this._root_env = null;
        this._websocket = null;
        this._update_trajectory_timer = null;
        this._update_markers_timer = null;
        this._update_scene_timer = null;
        this._xr_dolly = null;
        this._xr_controller1_grip = null;
        this._xr_controller2_grip = null;
        this._xr_controller1_model = null;
        this._xr_controller2_model = null;
        this._xr_gamepad1 = null;
        this._xr_gamepad2 = null;
        this._controls = null;
        this._xr_drag_controller_start = null;
        this._xr_drag_controller_orientation = null;
        this._xr_drag_dolly_start = null;

    }

    async createScene() {
        this._scene = new THREE.Scene();
        this._clock = new THREE.Clock();

        const camera = new THREE.PerspectiveCamera( 45, window.innerWidth/window.innerHeight, 0.1, 1000 );
        camera.position.set(3, 3, -1.5)
        this._camera = camera;

        const renderer = new THREE.WebGLRenderer( { antialias: true } );
        renderer.setPixelRatio( window.devicePixelRatio );
        renderer.setSize( window.innerWidth, window.innerHeight );
        renderer.outputEncoding = THREE.sRGBEncoding;
        renderer.xr.enabled = true;

        renderer.setClearColor("#000000");

        this._renderer = renderer;


        window.addEventListener( 'resize', onWindowResize, false );

        function onWindowResize(){

            camera.aspect = window.innerWidth / window.innerHeight;
            camera.updateProjectionMatrix();

            renderer.setSize( window.innerWidth, window.innerHeight );
        }

        const light = new THREE.HemisphereLight( 0xffffbb, 0x202018, 1 );
        this._scene.add( light );
        this._light = light;

        document.body.appendChild( renderer.domElement );

        this._controls = new OrbitControls( camera, renderer.domElement );

        let _this = this;
        
        // Only add VR button if it is supported
        if ( 'xr' in navigator )
        {
            if (await navigator.xr.isSessionSupported( 'immersive-vr' ))
            {
                document.body.appendChild( VRButton.createButton( renderer ) );

                renderer.xr.addEventListener('sessionstart', function() {
                    _this.enterXR();
                });
            }
        }

        renderer.setAnimationLoop( this.render.bind(this) );

        const gridHelper = new THREE.GridHelper( 10, 10 );
        this._scene.add( gridHelper );

        const root_z_up = new THREE.Object3D();
        root_z_up.rotateX(-Math.PI / 2.0);
        this._scene.add(root_z_up);

        const root_env = new THREE.Object3D();
        root_z_up.add(root_env);

        this._root_z_up = root_z_up;
        this._root_env = root_env;

        this._animation_mixer = new THREE.AnimationMixer( this._root_env );

        await this.updateScene();

        const queryString = window.location.search;
        const urlParams = new URLSearchParams(queryString);
        let do_update = true;
        if (urlParams.has("noupdate")) {
            if (urlParams.get("noupdate") === "true") {
                do_update = false;
            }
        }
        if (do_update) {
            self._update_trajectory_timer = setTimeout(() => _this.updateTrajectory(), 1000);
            self._update_markers_timer = setTimeout(() => _this.updateMarkers(), 1000);
        }

    }

    createWebSocket() {
        // Create a new WebSocket instance
        
        // Get host and port from current URL
        const host = window.location.hostname;
        const port = window.location.port;
        let ws_protocol = "ws";

        if (window.location.protocol === "https:") {
            ws_protocol = "wss";
        }

        const ws_url = ws_protocol + "://" + host + ":" + port + "/websocket";

        const socket = new RobustWebSocket(ws_url, null, {
            shouldReconnect: (event,ws) => { return 1000; }
        });

        // Event handlers
        
        let this_ = this;
        socket.onopen = function(event) {
        console.log('WebSocket connection established');
        if (this_._update_scene_timer) {
            clearTimeout(this_._update_scene_timer);
        }
        this_._update_scene_timer = setTimeout(() => this_.updateScene(), 500);
        if (this_._update_trajectory_timer) {
            clearTimeout(this_._update_trajectory_timer);
        }
        this_._update_trajectory_timer = setTimeout(() => this_.updateTrajectory(), 500);
        if (this_._update_markers_timer) {
            clearTimeout(this_._update_markers_timer);
        }
        this_._update_markers_timer = setTimeout(() => this_.updateMarkers(), 500);
        };
        
        socket.onmessage = function(event) {
        this_.processMessage(event.data)
        };

        socket.onclose = function(event) {
        console.log('WebSocket connection closed');
        };
    }

    render() {
        // Render the scene
        this._renderer.render(this._scene, this._camera);

        var delta = this._clock.getDelta();
        if ( this._animation_mixer ) this._animation_mixer.update( delta );

        this.xrLocomotion();
    }

    async fetchIfModified(url, etag) {
        let fetch_res;
        try {
            fetch_res = await fetch(url, { method: "HEAD" });
        }
        catch (_a) {
            return [null,null];
        }
        let new_etag = fetch_res.headers.get('etag');
        if (new_etag !== etag) {
            let fetch_res = await fetch(url);
            if (fetch_res.ok) {
                let res_json = await fetch_res.json();
                return [res_json, new_etag];
            }
        }
        return [null,null];
        
    }

    async updateScene() {
        if (this._update_scene_timer !== null) {
            clearTimeout(this._update_scene_timer);
            this._update_scene_timer = null;
        }
        let fetch_res;
        try {
            fetch_res = await fetch("tesseract_scene.gltf", { method: "HEAD" });
        }
        catch (_a) {
            let _this = this;
            self._update_scene_timer = setTimeout(() => _this.updateScene(), 5000);
            return;
        }
        let etag = fetch_res.headers.get('etag');
        if (etag !== null) {
            if (this._scene_etag !== null) {
                if (this._scene_etag != etag) {
                    this._scene_etag = null;
                    let _this = this;
                    self._update_scene_timer = setTimeout(() => _this.updateScene(), 0);
                    return;
                }
                else {
                    let _this = this;
                    self._update_scene_timer = setTimeout(() => _this.updateScene(), 5000);
                    return;
                }
            }
        }
        const loader = new GLTFLoader();

        let gltf = await new Promise((resolve, reject) => {
            loader.load('tesseract_scene.gltf', data=> resolve(data), null, reject);
        });

        if (this._root_env)
        {
            for( var i = this._root_env.children.length - 1; i >= 0; i--) { 
                let obj = this._root_env.children[i];
                this._root_env.remove(obj); 
            }
        }

        gltf.scene.traverse( function( node ) {
            if( node.material ) {
                node.material.side = THREE.DoubleSide;
            }
        });

        this._root_env.add(gltf.scene);        

        if (gltf.animations.length > 0)
        {
            
            this._animation_mixer.stopAllAction();
            this._animation_mixer.uncacheRoot(this._root_env);
             
            let animation_action = this._animation_mixer.clipAction(gltf.animations[0]);
            animation_action.play();

            this._animation = gltf.animations[0];
            this._animation_action = animation_action;
        }

        this._trajectory_etag = null;
        this._markers_etag = null;
        if (this._update_trajectory_timer !== null) {
            clearTimeout(this._update_trajectory_timer);
            this._update_trajectory_timer = null;
        }
        this._update_trajectory_timer = setTimeout(() => this.updateTrajectory(), 250);
        if (this._update_markers_timer !== null) {
            clearTimeout(this._update_markers_timer);
            this._update_markers_timer = null;
        }
        this._update_markers_timer = setTimeout(() => this.updateMarkers(), 250);

        if (etag !== null) {
            this._scene_etag = etag;
            let _this = this;
            this._update_scene_timer = setTimeout(() => _this.updateScene(), 1000);
        }
    }

    async updateTrajectory() {
        if (this._update_trajectory_timer !== null) {
            clearTimeout(this._update_trajectory_timer);
            this._update_trajectory_timer = null;
        }
        
        if (this._disable_update_trajectory) {
            return;
        }
        let etag = null;
        let trajectory_json = null;
        try {
            [trajectory_json, etag] = await this.fetchIfModified("./tesseract_trajectory.json", this._trajectory_etag);            
            if (trajectory_json === null) {
                console.log("No updated trajectory");
            }
            else{
                // console.log(trajectory_json)
                this.setTrajectory(trajectory_json.joint_names, trajectory_json.trajectory);
            }
        }
        catch (e) {
            console.log("Trajectory not available");
            console.log(e);
        }
        if (etag !== null) {
            this._trajectory_etag = etag;
        }        
        let _this = this;
        this._update_trajectory_timer = setTimeout(() => _this.updateTrajectory(), 10000);
    }

    async updateMarkers() {

        if (this._update_markers_timer !== null) {
            clearTimeout(this._update_markers_timer);
            this._update_markers_timer = null;
        }

        if (this._disable_update_markers) {
            return;
        }
        let etag = null;
        let markers_json = null;
        try {
            [markers_json, etag] = await this.fetchIfModified("./tesseract_markers.json", this._markers_etag);
            if (markers_json === null) {
                console.log("No updated markers");
            }
            else{
                console.log(markers_json)
                this.setMarkers(markers_json.markers);
            }
        }
        catch (e) {
            console.log("Markers not available");
            console.log(e);
        }
        if (etag !== null) {
            this._markers_etag = etag;
        }
        let _this = this;
        this._update_markers_timer = setTimeout(() => _this.updateMarkers(), 10000);
    }

    disableUpdateTrajectory() {
        this._disable_update_trajectory = true;
    }
    enableUpdateTrajectory() {
        this._disable_update_trajectory = false;
    }
    disableUpdateMarkers() {
        this._disable_update_markers = true;
    }
    enableUpdateMarkers() {
        this._disable_update_markers = false;
    }
    setJointPositions(joint_names, joint_positions) {
        let trajectory = [[...joint_positions, 0], [...joint_positions, 100000]];
        this.setTrajectory(joint_names, trajectory);
    }

    setTrajectory(joint_names, trajectory) {
        
        //this._animation_mixer.stopAllAction();
        //this._animation_mixer.uncacheRoot(this._root_env);
        
        let anim = this.trajectoryToAnimation(joint_names, trajectory);
        let animation_action = this._animation_mixer.clipAction(anim);
        animation_action.play();
        if (this._animation_action !== null)
        {
            this._animation_action.stop();
        }

        this._animation = anim;
        this._animation_action = animation_action;
    }

    trajectoryToAnimation(joint_names, trajectory) {
        let joints = this.findJoints(joint_names);
        let tracks = []
        joint_names.forEach((joint_name, joint_index) => {
            let joint = joints[joint_name];
            switch (joint.type) {
                case 1:
                case 2:
                    {
                        let values = [];
                        let times = []
                        trajectory.forEach(ee => {
                            let axis_vec = new THREE.Vector3().fromArray(joint.axes);
                            let quat = new THREE.Quaternion().setFromAxisAngle(axis_vec, ee[joint_index]);
                            let quat_array = quat.toArray();
                            values.push(...quat_array);
                            times.push(ee[ee.length - 1])
                        });
                        let track = new THREE.QuaternionKeyframeTrack(joint.joint.name + ".quaternion", times, values);                    
                        tracks.push(track);
                    }
                    break;
                case 3:
                    {
                        let values = [];
                        let times = []
                        trajectory.forEach(ee => {
                            let axis_vec = new THREE.Vector3().fromArray(joint.axes);
                            let vec = axis_vec.multiplyScalar(ee[joint_index]);
                            let vec_array = vec.toArray();
                            values.push(...vec_array);
                            times.push(ee[ee.length - 1])
                        });
                        let track = new THREE.VectorKeyframeTrack(joint.joint.name + ".position", times, values);                    
                        tracks.push(track);
                    }
                    break;
                default:
                    throw new Error("Unknown joint type");
            }
        });

        let animation_clip = new THREE.AnimationClip("tesseract_trajectory", -1, tracks);

        return animation_clip;
    }

    findJoints(joint_names)
    {
        let ret = {}
        this._root_env.traverse(tf => {
            if (tf.userData && tf.userData["tesseract_joint"])
            {
                let t_j = tf.userData["tesseract_joint"];

                if (joint_names && joint_names.indexOf(t_j["name"]) == -1) {
                    return;
                }
                let t = {};
                t.joint_name = t_j["name"];
                t.node_name = tf.name;
                t.joint = tf;
                t.axes = t_j.axis;
                t.type = t_j.type;
                ret[t.joint_name] = t;
            }
        });
        return ret;
    }

    processMessage(message) {
        let data = JSON.parse(message);
        if (data.command === "refresh_scene") {
            this.updateScene();
        }
        if (data.command === "refresh_trajectory") {
            this.updateTrajectory();
        }
        if (data.command === "refresh_markers") {
            this.updateMarkers();
        }
        if (data.command === "joint_positions") {            
            this.setJointPositions(data.params.joint_names, data.params.joint_positions);
        }
        if (data.command === "joint_trajectory") {            
            this.setTrajectory(data.params.joint_names, data.params.trajectory);
        }

        if (data.command === "markers") {            
            this.setMarkers(data.params.markers);
        }
    }

    findTesseractLink(name) {
        let ret = null;
        this._root_env.traverse(tf => {
            if (tf.userData && tf.userData["tesseract_link"] && tf.userData["tesseract_link"]["name"] === name) {
                ret = tf;            
            }
        });
        return ret;
    }

    setMarkers(markers) {
        this.clearMarkers();
        markers.forEach(marker => {
            if (marker.marker_type === "axes")
            {
                this.addAxesMarker(marker);
            }
            if (marker.marker_type === "arrow")
            {
                this.addArrowMarker(marker);
            }
            // box
            if (marker.marker_type === "box")
            {
                this.addBoxMarker(marker);
            }
            if (marker.marker_type === "sphere")
            {
                this.addSphereMarker(marker);
            }
            if (marker.marker_type === "cylinder")
            {
                this.addCylinderMarker(marker);
            }
            // cone
            if (marker.marker_type === "cone")
            {
                this.addConeMarker(marker);
            }
            // capsule
            if (marker.marker_type === "capsule")
            {
                this.addCapsuleMarker(marker);
            }
            // lines
            if (marker.marker_type === "lines")
            {
                this.addLinesMarker(marker);
            }


        });
    }

    addMarkerNode(marker_node,marker)
    {
        // find parent link by "tesseract_link" user data field
        let parent_link = this.findTesseractLink(marker.parent_link_name);
        if (!parent_link) {
            console.log("Parent link not found: " + marker.parent_link_name);
            return;
        }
        marker_node.position.fromArray(marker.position);
        marker_node.quaternion.w = marker.quaternion[0];
        marker_node.quaternion.x = marker.quaternion[1];
        marker_node.quaternion.y = marker.quaternion[2];
        marker_node.quaternion.z = marker.quaternion[3];
        marker_node.userData["tesseract_marker"] = marker.name;
        marker_node.userData["tesseract_tags"] = marker.tags;
        marker_node.userData["tesseract_label"] = marker.label;
        // Display label in scene
        this.addLabelToMarker(marker_node, marker.label);
        parent_link.add(marker_node);
    }

    addLabelToMarker(marker_node, label) {
        if (!label) {
            return;
        }
        let label_node = new THREE.Object3D();
        let label_text = new THREE.TextSprite({
            alignment: 'center',
            color: '#000000',
            fontFamily: '"Times New Roman", Times, serif',
            fontSize: 0.5,
            fontStyle: 'normal',
            text: label,
        });
        label_node.add(label_text);
        label_node.position.set(0,0,0.5);
        marker_node.add(label_node);
    }

    getMarkerMaterial(marker) {
        if (marker.color.length == 3) {
            marker.color.push(1.0);
        }
        let material = new THREE.MeshPhongMaterial({
            color: new THREE.Color(...marker.color),
            opacity: marker.color[3],
            transparent: marker.color[3] < 1.0,
            side: THREE.DoubleSide
        });
        return material;
    }

    addAxesMarker(marker) {
        
        let axes = new THREE.AxesHelper(marker.size);
        this.addMarkerNode(axes, marker);
    }

    addArrowMarker(marker)
    {
        let parent_link = this.findTesseractLink(marker.parent_link_name);
        if (!parent_link) {
            console.log("Parent link not found: " + marker.parent_link_name);
            return;
        }
        let arrow = new THREE.ArrowHelper(new THREE.Vector3(...marker.direction), new THREE.Vector3(...marker.origin), marker.length, new THREE.Color(...marker.color).getHex());
        arrow.userData["tesseract_marker"] = marker.name;
        arrow.userData["tesseract_tags"] = marker.tags || [];
        arrow.userData["tesseract_label"] = marker.label;
        // Display label in scene
        this.addLabelToMarker(arrow, marker.label);
        parent_link.add(arrow);
    }

    addBoxMarker(marker) {
        
        let box = new THREE.Mesh(new THREE.BoxGeometry(marker.size[0], marker.size[1], marker.size[2]), this.getMarkerMaterial(marker));
        this.addMarkerNode(box, marker);
    }

    addSphereMarker(marker) {

        let sphere = new THREE.Mesh(new THREE.SphereGeometry(marker.radius), this.getMarkerMaterial(marker));
        this.addMarkerNode(sphere, marker);
    }

    addCylinderMarker(marker) {

        let cylinder = new THREE.Mesh(new THREE.CylinderGeometry(marker.radius, marker.radius, marker.length), this.getMarkerMaterial(marker));
        this.addMarkerNode(cylinder, marker);
    }

    addConesMarker(marker) {

        let cone = new THREE.Mesh(new THREE.ConeGeometry(marker.radius, marker.length), this.getMarkerMaterial(marker));
        this.addMarkerNode(cone, marker);
    }

    addCapsuleMarker(marker) {
        console.log("Warning - capsule marker not implemented yet")
        return
        // let capsule = new THREE.Mesh(new THREE.CapsuleGeometry(marker.radius, marker.length), this.getMarkerMaterial(marker));
        // this.addMarkerNode(capsule, marker);
    }

    addMeshMarker(marker) {

        let geometry = new THREE.Geometry();
        geometry.fromBufferGeometry(marker.mesh);
        let material = this.getMarkerMaterial(marker);
        let mesh = new THREE.Mesh(geometry, material);
        this.addMarkerNode(mesh, marker);
    }

    addLinesMarker(marker) {
        // flatten marker.vertices array
        let points_array = [];
        marker.vertices.forEach(v => points_array.push(...v));
        
        const geometry = new LineGeometry();
        geometry.setPositions(points_array);

        const material = new LineMaterial({
            color: new THREE.Color(...marker.color),
            linewidth: marker.linewidth || 0.001, // in pixels
            vertexColors: false,
            dashed: false
        });

        const line = new Line2(geometry, material);
        line.computeLineDistances();
        line.scale.set(1, 1, 1);
        this.addMarkerNode(line, marker);
    }
    
    clearMarkers() {
        let remove_list = [];
        this._root_env.traverse(tf => {
            try
                {
                if (tf.userData && tf.userData["tesseract_marker"]) {
                    remove_list.push(tf);
                }
            }
            catch (e)
            {
                console.log(e); 
            }
        });
        remove_list.forEach(tf => {
            tf.parent.remove(tf);
        });
    }

    clearMarkerWithName(name) {
        let remove_list = [];
        this._root_env.traverse(tf => {
            try
                {
                if (tf.userData && tf.userData["tesseract_marker"] && tf.userData["tesseract_marker"] == name) {
                    remove_list.push(tf);
                }
            }
            catch (e)
            {
                console.log(e); 
            }
        });
        remove_list.forEach(tf => {
            tf.parent.remove(tf);
        });
    }

    clearMarkersWithTags(tags) {
        let remove_list = [];
        this._root_env.traverse(tf => {
            try
                {
                if (tf.userData && tf.userData["tesseract_tags"]) {
                    let tf_tags = tf.userData["tesseract_tags"];
                    if (tags.every(tag => tf_tags.includes(tag))) {
                        remove_list.push(tf);
                    }
                }
            }
            catch (e)
            {
                console.log(e); 
            }
        });
        remove_list.forEach(tf => {
            tf.parent.remove(tf);
        });
    }

    initXRControllers(dolly) {
        const controller1 = this._renderer.xr.getController(0);
        const controller2 = this._renderer.xr.getController(1);

        dolly.add(controller1);
        dolly.add(controller2);

        const controllerModelFactory = new XRControllerModelFactory();

        const controllerGrip1 = this._renderer.xr.getControllerGrip(0);
        this._xr_controller1_grip = controllerGrip1;
        this._xr_controller1_model = controllerModelFactory.createControllerModel(controllerGrip1);
        controllerGrip1.add(this._xr_controller1_model);
        dolly.add(controllerGrip1);

        const controllerGrip2 = this._renderer.xr.getControllerGrip(1);
        this._xr_controller2_grip = controllerGrip2;
        this._xr_controller2_model = controllerModelFactory.createControllerModel(controllerGrip2);
        controllerGrip2.add(this._xr_controller2_model);
        dolly.add(controllerGrip2);

        let _this = this;

        controllerGrip1.addEventListener("connected", (e)=> {
            _this._xr_gamepad1 = e.data.gamepad;
        })

        controllerGrip2.addEventListener("connected", (e)=> {
            _this._xr_gamepad2 = e.data.gamepad;
        })
    }

    enterXR() {
        this._controls.saveState();
        this._xr_dolly = new THREE.Object3D();
        this.initXRControllers(this._xr_dolly)
        this._xr_dolly.add(this._camera);
        this._camera.position.set(0, 0, 0);
        this._camera.rotation.set(0, 0, 0);
        this._scene.add(this._xr_dolly);
        this._xr_dolly.position.set(2.5,0,0);
        this._xr_dolly.rotateY(Math.PI / 2.0);

        this._renderer.xr.getSession().addEventListener('end', () => {
            this._scene.add(this._camera);
            this._scene.remove(this._xr_dolly);
            this._xr_dolly.remove(this._camera);
            this._xr_dolly = null;
            this._xr_controller1_grip.remove(this._xr_controller1_model);
            this._xr_controller2_grip.remove(this._xr_controller2_model);
            this._xr_controller1_grip = null;
            this._xr_controller2_grip = null;
            this._xr_controller1_model = null;
            this._xr_controller2_model = null;
            this._controls.reset();
            this._controls.update();
        });


    }
    
    xrLocomotion()
    {
        if (this._xr_dolly && this._xr_gamepad2) {
            try
            {
            
            if (this._xr_gamepad2.buttons[5].pressed)
            {
                if (!this._xr_drag_controller_start)
                {
                    this._xr_drag_controller_start = this._xr_controller2_grip.position.clone();
                    let world_quat = new THREE.Quaternion();
                    this._xr_drag_controller_orientation = this._camera.getWorldQuaternion(world_quat);
                    
                    this._xr_drag_dolly_start = this._xr_dolly.position.clone();
                }

                let controller_diff = this._xr_controller2_grip.position.clone().sub(this._xr_drag_controller_start);
                controller_diff.applyQuaternion(this._xr_drag_controller_orientation.clone());
                let y_diff = controller_diff.y;
                y_diff = Math.floor(y_diff / 0.1);
                if (y_diff < 0) {
                    y_diff = y_diff + 1;
                }
                controller_diff.y = y_diff * 0.1;
                let dolly_pos = this._xr_drag_dolly_start.clone().sub(controller_diff);
                this._xr_dolly.position.copy(dolly_pos);
            }
            else
            {
                if (this._xr_drag_controller_start)
                {
                    this._xr_drag_controller_start = null;
                    this._xr_drag_dolly_start = null;
                }
            }

            }
            catch (e)
            {
                console.log(e);
            }

            if (this._xr_gamepad2.axes.length == 4) {
                let axis_2 = this._xr_gamepad2.axes[2];
                if (axis_2 > 0.2)
                {
                    let scale = -(axis_2 - 0.2)/0.8;
                    this._xr_dolly.rotateY(0.01 * scale);
                }
                if (axis_2 < -0.2)
                {
                    let scale = -(axis_2 + 0.2)/0.8;
                    this._xr_dolly.rotateY(0.01 * scale);
                }
                
                let axis_3 = this._xr_gamepad2.axes[3];
                let dolly_world_quat = new THREE.Quaternion();
                this._xr_dolly.getWorldQuaternion(dolly_world_quat);
                let dolly_forward = new THREE.Vector3(0,0,-1);
                dolly_forward.applyQuaternion(dolly_world_quat);

                if (axis_3 > 0.2)
                {
                    let scale = -(axis_3 - 0.2)/0.8;
                    this._xr_dolly.position.add(dolly_forward.clone().multiplyScalar(0.01 * scale));
                }
                if (axis_3 < -0.2)
                {
                    let scale = -(axis_3 + 0.2)/0.8;
                    this._xr_dolly.position.add(dolly_forward.clone().multiplyScalar(0.01 * scale));
                }
            }
        }
    };
}



window.addEventListener("DOMContentLoaded", async function () {
    let viewer = new TesseractViewer();
    window.tesseract_viewer = viewer;
    try
    {
        await viewer.createScene();
    }
    catch (e)
    {
        console.log(e);
    }
    await viewer.createWebSocket();
    window.addEventListener("message", function (event) {
        let data = event.data;
        viewer.processMessage(data);
    });
    viewer.render();
})