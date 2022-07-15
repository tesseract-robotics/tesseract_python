/**
 * @file tesseract_collision_python.i
 * @brief The tesseract_collision_python SWIG master file.
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

%module(directors="1", package="tesseract_robotics.tesseract_collision") tesseract_collision_python

#pragma SWIG nowarn=473

%pythondynamic tesseract_collision::ContactResult;

%include "tesseract_swig_include.i"
%include "tesseract_std_function.i"

//%import "tesseract_common_python.i"
%import "tesseract_geometry_python.i"

%{
#include <tesseract_common/status_code.h>
// tesseract_collision
#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/contact_managers_plugin_factory.h>
#include <tesseract_collision/bullet/bullet_factories.h>
#include <tesseract_collision/fcl/fcl_factories.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract_collision/fcl/fcl_discrete_managers.h>

#include "tesseract_collisions_python_std_functions.h"
%}

%ignore getIsContactAllowedFn;

%tesseract_std_function(IsContactAllowedFn,tesseract_collision,bool,const std::string&,a,const std::string&,b);
%tesseract_std_function(IsContactValidFn,tesseract_collision,bool,const tesseract_collision::ContactResult&,a);

// tesseract_collision
#define TESSERACT_COLLISION_CORE_PUBLIC
%include "tesseract_collision/core/types.h"
%include "tesseract_collision/core/discrete_contact_manager.h"
%include "tesseract_collision/core/continuous_contact_manager.h"
%include "tesseract_collision/core/contact_managers_plugin_factory.h"

%init %{
tesseract_common::PluginLoader::addSymbolLibraryToSearchLibrariesEnv(tesseract_collision::tesseract_collision_bullet::BulletFactoriesAnchor(), "TESSERACT_CONTACT_MANAGERS_PLUGINS");
tesseract_common::PluginLoader::addSymbolLibraryToSearchLibrariesEnv(tesseract_collision::tesseract_collision_fcl::FCLFactoriesAnchor(), "TESSERACT_CONTACT_MANAGERS_PLUGINS");
%}
