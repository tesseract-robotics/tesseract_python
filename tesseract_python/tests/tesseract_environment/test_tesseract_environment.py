from tesseract_robotics import tesseract_scene_graph
from tesseract_robotics import tesseract_collision
from tesseract_robotics import tesseract_environment
from tesseract_robotics.tesseract_common import Isometry3d, Translation3d, AngleAxisd
from tesseract_robotics import tesseract_common
from tesseract_robotics import tesseract_collision
from tesseract_robotics import tesseract_urdf
from tesseract_robotics import tesseract_srdf
from ..tesseract_support_resource_locator import TesseractSupportResourceLocator
import traceback
import os
import re
import numpy as np

def get_scene_graph():
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    path =  os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.urdf")
    locator = TesseractSupportResourceLocator()
    return tesseract_urdf.parseURDFFile(path, locator).release()

def get_srdf_model(scene_graph):
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    path =  os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.srdf")
    srdf = tesseract_srdf.SRDFModel()
    locator = TesseractSupportResourceLocator()
    srdf.initFile(scene_graph, path, locator)
    return srdf

def get_environment():
    scene_graph = get_scene_graph()
    assert scene_graph is not None

    srdf = get_srdf_model(scene_graph)
    assert srdf is not None

    env = tesseract_environment.Environment()
    assert env is not None

    assert env.getRevision() == 0

    success = env.init(scene_graph,srdf)
    assert success
    assert env.getRevision() == 3
    
    joint_names = [f"joint_a{i+1}" for i in range(7)]
    joint_values = np.array([1,2,1,2,1,2,1],dtype=np.float64)

    scene_state_changed = [False]
    command_applied = [False]

    def event_cb_py(evt):
        try:
            if evt.type == tesseract_environment.Events_SCENE_STATE_CHANGED:
                evt2 = tesseract_environment.cast_SceneStateChangedEvent(evt)
                if len(evt2.state.joints) != 7:
                    print("joint state length error")
                    return
                for i in range(len(joint_names)):
                    if evt2.state.joints[joint_names[i]] != joint_values[i]:
                        print("joint value mismatch")
                        return
                scene_state_changed[0] = True
            if evt.type == tesseract_environment.Events_COMMAND_APPLIED:
                evt2 = tesseract_environment.cast_CommandAppliedEvent(evt)
                print(evt2.revision)
                if evt2.revision == 4:
                    command_applied[0] = True
        except:
            traceback.print_exc()
    event_cb = tesseract_environment.EventCallbackFn(event_cb_py)

    env.addEventCallback(12345, event_cb)

    env.setState(joint_names, joint_values)
    assert scene_state_changed[0]

    cmd = tesseract_environment.RemoveJointCommand("joint_a7-tool0")
    assert env.applyCommand(cmd)
    assert command_applied[0]

    # env.init() now populates contact managers?

    """discrete_create_fn = tesseract_collision.DiscreteContactManagerFactoryCreateMethod(tesseract_collision_bullet.BulletDiscreteBVHManager.create)
    assert env.registerDiscreteContactManager(tesseract_collision_bullet.BulletDiscreteBVHManager.name(),
        discrete_create_fn)

    cont_create_fn = tesseract_collision.ContinuousContactManagerFactoryCreateMethod(tesseract_collision_bullet.BulletCastBVHManager.create)
    assert env.registerContinuousContactManager(tesseract_collision_bullet.BulletCastBVHManager.name(),
        cont_create_fn)

    env.setActiveDiscreteContactManager(tesseract_collision_bullet.BulletDiscreteBVHManager.name())
    env.setActiveContinuousContactManager(tesseract_collision_bullet.BulletCastBVHManager.name())"""

    return env

def test_env():
    get_environment()