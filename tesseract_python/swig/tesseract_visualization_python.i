/**
 * @file tesseract_visualization_python.i
 * @brief The tesseract_visualization_python SWIG master file.
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

%module(directors="1", package="tesseract_robotics.tesseract_visualization") tesseract_visualization_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

%import "tesseract_environment_python.i"

%{

// tesseract_geometry

#include <tesseract_geometry/geometries.h>

// tesseract_visualization
#include <tesseract_visualization/markers/marker.h>
#include <tesseract_visualization/markers/arrow_marker.h>
#include <tesseract_visualization/markers/axis_marker.h>
#include <tesseract_visualization/markers/contact_results_marker.h>
#include <tesseract_visualization/markers/geometry_marker.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_visualization/visualization_loader.h>
#include <tesseract_visualization/trajectory_interpolator.h>
#include <tesseract_visualization/trajectory_player.h>


#include <tesseract_common/resource_locator.h>

// tesseract_state_solver
#include <tesseract_state_solver/mutable_state_solver.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_state_solver/kdl/kdl_state_solver.h>
#include <tesseract_state_solver/ofkt/ofkt_state_solver.h>

// tesseract_kinematics
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>

// tesseract_environment
#include <tesseract_environment/commands.h>
#include <tesseract_environment/events.h>
#include <tesseract_environment/environment.h>

#include "tesseract_environment_python_std_functions.h"

%}

// tesseract_visualization
#define TESSERACT_VISUALIZATION_PUBLIC

%shared_ptr(tesseract_visualization::Marker)
%include "tesseract_visualization/markers/marker.h"

%shared_ptr(tesseract_visualization::ArrowMarker)
%include "tesseract_visualization/markers/arrow_marker.h"

%shared_ptr(tesseract_visualization::AxisMarker)
%include "tesseract_visualization/markers/axis_marker.h"

%shared_ptr(tesseract_visualization::ContactResultsMarker)
%include "tesseract_visualization/markers/contact_results_marker.h"

%shared_ptr(tesseract_visualization::GeometryMarker)
%include "tesseract_visualization/markers/geometry_marker.h"

%shared_ptr(tesseract_visualization::ToolpathMarker)
%include "tesseract_visualization/markers/toolpath_marker.h"
%include "tesseract_visualization/trajectory_interpolator.h"
%include "tesseract_visualization/trajectory_player.h"

%shared_ptr(tesseract_visualization::Visualization)
%include "tesseract_visualization/visualization.h"
%include "tesseract_visualization/visualization_loader.h"


