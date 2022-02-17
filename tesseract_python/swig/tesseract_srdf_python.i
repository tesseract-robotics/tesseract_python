/**
 * @file tesseract_srdf_python.i
 * @brief The tesseract_scene_graph_python SWIG master file.
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

%module(directors="1", package="tesseract_robotics.tesseract_srdf") tesseract_srdf_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

%import "tesseract_scene_graph_python.i"

%{

#include <tesseract_common/status_code.h>
#include <tesseract_common/resource_locator.h>

#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/collision_margin_data.h>
#include <tesseract_geometry/geometries.h>

// tesseract_srdf
#include <tesseract_srdf/kinematics_information.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_srdf/utils.h>

%}

// tesseract_srdf
#define TESSERACT_SRDF_PUBLIC
%include "tesseract_srdf/kinematics_information.h"
%include "tesseract_srdf/srdf_model.h"
%include "tesseract_srdf/utils.h"
