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

#include <tesseract/geometry/geometries.h>

// tesseract_visualization
#include <tesseract/visualization/markers/marker.h>
#include <tesseract/visualization/markers/arrow_marker.h>
#include <tesseract/visualization/markers/axis_marker.h>
#include <tesseract/visualization/markers/contact_results_marker.h>
#include <tesseract/visualization/markers/geometry_marker.h>
#include <tesseract/visualization/markers/toolpath_marker.h>
#include <tesseract/visualization/visualization.h>
#include <tesseract/visualization/visualization_loader.h>
#include <tesseract/visualization/trajectory_interpolator.h>
#include <tesseract/visualization/trajectory_player.h>


#include <tesseract/common/resource_locator.h>

// tesseract_state_solver
#include <tesseract/state_solver/mutable_state_solver.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/state_solver/kdl/kdl_state_solver.h>
#include <tesseract/state_solver/ofkt/ofkt_state_solver.h>

// tesseract_kinematics
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/kinematic_group.h>

// tesseract_environment
#include <tesseract/environment/commands.h>
#include <tesseract/environment/events.h>
#include <tesseract/environment/environment.h>

#include "tesseract_environment_python_std_functions.h"

%}

// tesseract_visualization
#define TESSERACT_VISUALIZATION_PUBLIC

%shared_ptr(tesseract::visualization::Marker)
%include "tesseract/visualization/markers/marker.h"

%shared_ptr(tesseract::visualization::ArrowMarker)
%include "tesseract/visualization/markers/arrow_marker.h"

%shared_ptr(tesseract::visualization::AxisMarker)
%include "tesseract/visualization/markers/axis_marker.h"

%shared_ptr(tesseract::visualization::ContactResultsMarker)
%include "tesseract/visualization/markers/contact_results_marker.h"

%shared_ptr(tesseract::visualization::GeometryMarker)
%include "tesseract/visualization/markers/geometry_marker.h"

%shared_ptr(tesseract::visualization::ToolpathMarker)
%include "tesseract/visualization/markers/toolpath_marker.h"
%include "tesseract/visualization/trajectory_interpolator.h"
%include "tesseract/visualization/trajectory_player.h"

%shared_ptr(tesseract::visualization::Visualization)
%include "tesseract/visualization/visualization.h"
%include "tesseract/visualization/visualization_loader.h"


