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
%shared_ptr(tesseract_scene_graph::JointDynamics)
%shared_ptr(tesseract_scene_graph::JointLimits)
%shared_ptr(tesseract_scene_graph::JointSafety)
%shared_ptr(tesseract_scene_graph::JointMimic)
%shared_ptr(tesseract_scene_graph::JointCalibration)
%shared_ptr(tesseract_scene_graph::Joint)
%shared_ptr(tesseract_scene_graph::Link)
%nocopyctor tesseract_scene_graph::Joint;
%ignore clone;
%include "tesseract_scene_graph/joint.h"
%rename("%s") clone;
%extend tesseract_scene_graph::Joint
{
  tesseract_scene_graph::Joint::Ptr clone() const
  {
    return std::make_shared<tesseract_scene_graph::Joint>(std::move($self->clone()));
  }

  tesseract_scene_graph::Joint::Ptr clone(const std::string& name) const
  {
    return std::make_shared<tesseract_scene_graph::Joint>(std::move($self->clone(name)));
  }
}


%shared_ptr(tesseract_scene_graph::Material)
%shared_ptr(tesseract_scene_graph::Inertial)
%shared_ptr(tesseract_scene_graph::Visual)
%shared_ptr(tesseract_scene_graph::Collision)
%shared_ptr(tesseract_scene_graph::Link)
%template(tesseract_scene_graph_VisualVector) std::vector<std::shared_ptr<tesseract_scene_graph::Visual> >;
%template(tesseract_scene_graph_CollisionVector) std::vector<std::shared_ptr<tesseract_scene_graph::Collision> >;
%nocopyctor tesseract_scene_graph::Link;
%ignore clone;
%include "tesseract_scene_graph/link.h"
%rename("%s") clone;
%extend tesseract_scene_graph::Link
{
  tesseract_scene_graph::Link::Ptr clone() const
  {
    return std::make_shared<tesseract_scene_graph::Link>(std::move($self->clone()));
  }

  tesseract_scene_graph::Link::Ptr clone(const std::string& name) const
  {
    return std::make_shared<tesseract_scene_graph::Link>(std::move($self->clone(name)));
  }
}

%shared_ptr(tesseract_scene_graph::SceneGraph)
%wrap_unique_ptr(SceneGraphUPtr,tesseract_scene_graph::SceneGraph)
%template(LinkVector) std::vector<std::shared_ptr<tesseract_scene_graph::Link> >;
%template(JointVector) std::vector<std::shared_ptr<tesseract_scene_graph::Joint> >;
%template(LinkConstVector) std::vector<std::shared_ptr<tesseract_scene_graph::Link const> >;
%template(JointConstVector) std::vector<std::shared_ptr<tesseract_scene_graph::Joint const> >;
%include "tesseract_scene_graph/graph.h"

%shared_ptr(tesseract_scene_graph::SceneState)
%include "tesseract_scene_graph/scene_state.h"


