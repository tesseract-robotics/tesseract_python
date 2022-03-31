import * as THREE from 'https://unpkg.com/three@0.127.0/build/three.module.js';

import { OrbitControls } from 'https://unpkg.com/three@0.127.0/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from 'https://unpkg.com/three@0.127.0/examples/jsm/loaders/GLTFLoader.js'
import { VRButton } from 'https://unpkg.com/three@0.127.0/examples/jsm/webxr/VRButton.js'

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
        this._animation_mixer = null;
        this._animation = null;
        this._animation_action = null;
        this._root_z_up = null;
        this._root_env = null;

    }

    async createScene() {
        this._scene = new THREE.Scene();
        this._clock = new THREE.Clock();

        const camera = new THREE.PerspectiveCamera( 45, window.innerWidth/window.innerHeight, 0.1, 1000 );
        camera.position.x = 3;
        camera.position.y = 3;
        camera.position.z = -1.5;
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

        const controls = new OrbitControls( camera, renderer.domElement );

        document.body.appendChild( VRButton.createButton( renderer ) );

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

        let _this = this;
        const queryString = window.location.search;
        const urlParams = new URLSearchParams(queryString);
        let do_update = true;
        if (urlParams.has("noupdate")) {
            if (urlParams.get("noupdate") === "true") {
                do_update = false;
            }
        }
        if (do_update) {
            setTimeout(() => _this.updateTrajectory(), 2000);
        }

    }

    render() {
        // Render the scene
        this._renderer.render(this._scene, this._camera);

        var delta = this._clock.getDelta();
        if ( this._animation_mixer ) this._animation_mixer.update( delta );
    };

    async updateScene() {
        let fetch_res;
        try {
            fetch_res = await fetch("tesseract_scene.gltf", { method: "HEAD" });
        }
        catch (_a) {
            let _this = this;
            setTimeout(() => _this.updateScene(), 1000);
            return;
        }
        let etag = fetch_res.headers.get('etag');
        if (etag !== null) {
            if (this._scene_etag !== null) {
                if (this._scene_etag != etag) {
                    this._scene_etag = null;
                    let _this = this;
                    setTimeout(() => _this.updateScene(), 0);
                    return;
                }
                else {
                    let _this = this;
                    setTimeout(() => _this.updateScene(), 1000);
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

        if (etag !== null) {
            this._scene_etag = etag;
            let _this = this;
            setTimeout(() => _this.updateScene(), 1000);
        }
    }

    async updateTrajectory() {
        
        if (this._disable_update_trajectory) {
            return;
        }
        let fetch_res;
        let _this = this;
        try {
            fetch_res = await fetch("tesseract_trajectory.json", { method: "HEAD" });
        }
        catch (_a) {
            setTimeout(() => _this.updateTrajectory(), 1000);
            return;
        }
        if (!fetch_res.ok) {
            setTimeout(() => _this.updateTrajectory(), 1000);
            return;
        }
        let etag = fetch_res.headers.get('etag');
        if (etag == null || this._trajectory_etag == etag) {
            console.log("No updated trajectory");
            setTimeout(() => _this.updateTrajectory(), 1000);
            return;
        }
        try {
            let trajectory_response = await fetch("./tesseract_trajectory.json");
            let trajectory_json = await trajectory_response.json();
            console.log(trajectory_json)
            this.setTrajectory(trajectory_json.joint_names, trajectory_json.trajectory);
        }
        catch (e) {
            console.log("Trajectory not available");
            console.log(e);
        }
        if (etag !== null) {
            this._trajectory_etag = etag;
            setTimeout(() => _this.updateTrajectory(), 1000);
        }
        
    }
    disableUpdateTrajectory() {
        this._disable_update_trajectory = true;
    }
    enableUpdateTrajectory() {
        this._disable_update_trajectory = false;
    }
    setJointPositions(joint_names, joint_positions) {
        let trajectory = [[...joint_positions, 0], [...joint_positions, 100000]];
        this.setTrajectory(joint_names, trajectory);
    }

    setTrajectory(joint_names, trajectory) {
        
        this._animation_mixer.stopAllAction();
        this._animation_mixer.uncacheRoot(this._root_env);

        let anim = this.trajectoryToAnimation(joint_names, trajectory);
        let animation_action = this._animation_mixer.clipAction(anim);
        animation_action.play();

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
                case 2:
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
}

window.addEventListener("DOMContentLoaded", async function () {
    let viewer = new TesseractViewer();
    window.tesseract_viewer = viewer;
    await viewer.createScene();
    window.addEventListener("message", function (event) {
        let data = event.data;
        if (data.command === "joint_positions") {
            viewer.disableUpdateTrajectory();
            viewer.setJointPositions(data.joint_names, data.joint_positions);
        }
        if (data.command === "joint_trajectory") {
            viewer.disableUpdateTrajectory();
            viewer.setTrajectory(data.joint_names, data.joint_trajectory);
        }
    });
    viewer.render();
})