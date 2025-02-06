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


#include <tesseract_common/resource_locator.h>

#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/collision_margin_data.h>
#include <tesseract_geometry/geometries.h>

// tesseract_srdf
#include <tesseract_srdf/kinematics_information.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_srdf/utils.h>

#include <tesseract_scene_graph/graph.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/yaml_utils.h>
#include <tesseract_common/yaml_extenstions.h>

%}

// tesseract_srdf
#define TESSERACT_SRDF_PUBLIC

%shared_ptr(tesseract_srdf::SRDFModel)
%include "tesseract_srdf/kinematics_information.h"
%include "tesseract_srdf/srdf_model.h"
%include "tesseract_srdf/utils.h"

%inline {
tesseract_common::CalibrationInfo parseCalibrationConfigString(const tesseract_scene_graph::SceneGraph& scene_graph,
                                                         const std::string& yaml_str)
{
  YAML::Node config;
  try
  {
    config = YAML::Load(yaml_str);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("calibration_config: YAML failed to parse calibration config string"));
  }

  const YAML::Node& cal_info = config[tesseract_common::CalibrationInfo::CONFIG_KEY];
  auto info = cal_info.as<tesseract_common::CalibrationInfo>();

  // Check to make sure calibration joints exist
  for (const auto& cal_joint : info.joints)
  {
    if (scene_graph.getJoint(cal_joint.first) == nullptr)
      std::throw_with_nested(std::runtime_error("calibration_config: joint '" + cal_joint.first + "' does not exist!"));
  }

  return info;
}

tesseract_common::KinematicsPluginInfo parseKinematicsPluginConfigString(const std::string& yaml_str)
{
  
  YAML::Node config;
  try
  {
    config = YAML::Load(yaml_str);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("kinematics_plugin_config: YAML failed to parse kinematics plugins string" ));
  }

  const YAML::Node& kin_plugin_info = config[tesseract_common::KinematicsPluginInfo::CONFIG_KEY];

  return kin_plugin_info.as<tesseract_common::KinematicsPluginInfo>();
}

tesseract_common::ContactManagersPluginInfo
parseContactManagersPluginConfigString(const std::string& yaml_str)
{
  YAML::Node config;
  try
  {
    config = YAML::Load(yaml_str);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("contact_managers_plugin_config: YAML failed to parse contact "
                                              "managers plugins string"));
  }

  const YAML::Node& cm_plugin_info = config[tesseract_common::ContactManagersPluginInfo::CONFIG_KEY];
  return cm_plugin_info.as<tesseract_common::ContactManagersPluginInfo>();
}
}
