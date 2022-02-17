/**
 * @file tesseract_scene_graph_python.i
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

%module(directors="1", package="tesseract_robotics.tesseract_scene_graph") tesseract_scene_graph_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

//%import "tesseract_common_python.i"
%import "tesseract_geometry_python.i"

%{

#include <tesseract_common/status_code.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/allowed_collision_matrix.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/directed_graph.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include <tesseract_geometry/geometries.h>

// tesseract_scene_graph
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>


%}

// tesseract_scene_graph
#define TESSERACT_SCENE_GRAPH_PUBLIC
%include "tesseract_scene_graph/joint.h"
%include "tesseract_scene_graph/link.h"
%include "tesseract_scene_graph/graph.h"
%include "tesseract_scene_graph/scene_state.h"


