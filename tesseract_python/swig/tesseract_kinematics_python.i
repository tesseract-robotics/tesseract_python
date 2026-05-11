/**
 * @file tesseract_kinematics_python.i
 * @brief The tesseract_kinematics_python SWIG master file.
 *
 * @author John Wason
 * @date December 8, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Wason Technology, LLC
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

%module(directors="1", package="tesseract_robotics.tesseract_kinematics") tesseract_kinematics_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

%import "tesseract_common_python.i"
//%import "tesseract_geometry_python.i"
%import "tesseract_scene_graph_python.i"
%import "tesseract_srdf_python.i"
%import "tesseract_state_solver_python.i"

%{

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/directed_graph.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/breadth_first_search.hpp>


#include <tesseract/geometry/geometries.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/srdf/kinematics_information.h>

// tesseract_state_solver
#include <tesseract/state_solver/mutable_state_solver.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/state_solver/kdl/kdl_state_solver.h>
#include <tesseract/state_solver/ofkt/ofkt_state_solver.h>

// tesseract_kinematics
#include <tesseract/kinematics/forward_kinematics.h>
#include <tesseract/kinematics/inverse_kinematics.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/kinematic_group.h>
#include <tesseract/kinematics/kinematics_plugin_factory.h>
#include <tesseract/kinematics/rep_factory.h>
#include <tesseract/kinematics/rop_factory.h>
#include <tesseract/kinematics/utils.h>

#include <tesseract/kinematics/kdl/kdl_factories.h>
#include <tesseract/kinematics/opw/opw_factory.h>
#include <tesseract/kinematics/ur/ur_factory.h>

#include <boost_plugin_loader/utils.h>

%}

%template(pair_bool_matrix) std::pair<bool,Eigen::MatrixXd>;

//TODO: passing unique_ptr as parameter with move command
%ignore tesseract::kinematics::KinematicGroup::KinematicGroup;

// tesseract_kinematics
#define TESSERACT_KINEMATICS_CORE_PUBLIC
#define TESSERACT_KINEMATICS_IKFAST_PUBLIC
#define TESSERACT_KINEMATICS_KDL_PUBLIC
#define TESSERACT_KINEMATICS_OPW_PUBLIC
%include "rework_include/tesseract_kinematics/core/types.i"

%shared_ptr(tesseract::kinematics::ForwardKinematics)
%wrap_unique_ptr(ForwardKinematicsUPtr,tesseract::kinematics::ForwardKinematics)
%include "tesseract/kinematics/forward_kinematics.h"

%shared_ptr(tesseract::kinematics::InverseKinematics)
%wrap_unique_ptr(InverseKinematicsUPtr,tesseract::kinematics::InverseKinematics)
%include "tesseract/kinematics/inverse_kinematics.h"

%shared_ptr(tesseract::kinematics::JointGroup)
%wrap_unique_ptr(JointGroupUPtr,tesseract::kinematics::JointGroup)
%include "tesseract/kinematics/joint_group.h"

%shared_ptr(tesseract::kinematics::KinematicGroup)
%wrap_unique_ptr(KinematicGroupUPtr,tesseract::kinematics::KinematicGroup)

namespace tesseract::kinematics
{
struct KinGroupIKInput;
%tesseract_aligned_vector_using(KinGroupIKInputs,tesseract::kinematics::KinGroupIKInput)
}

%include "tesseract/kinematics/kinematic_group.h"
%tesseract_aligned_vector(KinGroupIKInputs,tesseract::kinematics::KinGroupIKInput)

%shared_ptr(tesseract::kinematics::KinematicsPluginFactory)
%shared_ptr(tesseract::kinematics::FwdKinFactory)
%shared_ptr(tesseract::kinematics::InvKinFactory)
%include "tesseract/kinematics/kinematics_plugin_factory.h"



// TODO
//%shared_ptr(tesseract::kinematics::RobotWithExternalPositionerInvKin)
//%shared_ptr(tesseract::kinematics::RobotOnPositionerInvKin)
//%shared_ptr(tesseract::kinematics::IKFastInvKin)
//%shared_ptr(tesseract::kinematics::KDLFwdKinChain)
//%shared_ptr(tesseract::kinematics::KDLInvKinChainLMA)
//%shared_ptr(tesseract::kinematics::KDLInvKinChainNR)
//%shared_ptr(tesseract::kinematics::OPWInvKin)

%eigen_typemaps(%arg(tesseract::kinematics::VectorX<double>));
%include "tesseract/kinematics/utils.h"
%template(getRedundantSolutions) tesseract::kinematics::getRedundantSolutions<double>;




%init %{
// TODO: fix anchors
boost_plugin_loader::addSymbolLibraryToSearchLibrariesEnv(tesseract::kinematics::REPInvKinFactoriesAnchor(), "TESSERACT_KINEMATICS_PLUGINS");
boost_plugin_loader::addSymbolLibraryToSearchLibrariesEnv(tesseract::kinematics::ROPInvKinFactoriesAnchor(), "TESSERACT_KINEMATICS_PLUGINS");
boost_plugin_loader::addSymbolLibraryToSearchLibrariesEnv(tesseract::kinematics::KDLFactoriesAnchor(), "TESSERACT_KINEMATICS_PLUGINS");
boost_plugin_loader::addSymbolLibraryToSearchLibrariesEnv(tesseract::kinematics::OPWFactoriesAnchor(), "TESSERACT_KINEMATICS_PLUGINS");
boost_plugin_loader::addSymbolLibraryToSearchLibrariesEnv(tesseract::kinematics::URFactoriesAnchor(), "TESSERACT_KINEMATICS_PLUGINS");


%}
