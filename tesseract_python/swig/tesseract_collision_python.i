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

namespace tesseract_collision
{
    class ContactResult;
}

%pythondynamic tesseract_collision::ContactResult;

%include "tesseract_swig_include.i"
%include "tesseract_std_function.i"

//%import "tesseract_common_python.i"
%import "tesseract_geometry_python.i"

%{
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

// tesseract_common
#include <tesseract_common/resource_locator.h>
#include <tesseract_geometry/geometries.h>

#include "tesseract_collisions_python_std_functions.h"
%}

%ignore getIsContactAllowedFn;
%ignore tesseract_collision::ContactTestData;

%tesseract_std_function(IsContactAllowedFn,tesseract_collision,bool,const std::string&,a,const std::string&,b);
%tesseract_std_function(IsContactValidFn,tesseract_collision,bool,const tesseract_collision::ContactResult&,a);

%shared_ptr(tesseract_collision::tesseract_collision_bullet::BulletCastBVHManager)
%shared_ptr(tesseract_collision::tesseract_collision_bullet::BulletCastSimpleManager)
%shared_ptr(tesseract_collision::tesseract_collision_bullet::BulletDiscreteBVHManager)
%shared_ptr(tesseract_collision::tesseract_collision_bullet::BulletDiscreteSimpleManager)
%shared_ptr(tesseract_collision::ContactManagersPluginFactory)
%shared_ptr(tesseract_collision::DiscreteContactManagerFactory)
%shared_ptr(tesseract_collision::ContinuousContactManagerFactory)
%shared_ptr(tesseract_collision::ContinuousContactManager)
%wrap_unique_ptr(ContinuousContactManagerUPtr,tesseract_collision::ContinuousContactManager)
%shared_ptr(tesseract_collision::DiscreteContactManager)
%wrap_unique_ptr(DiscreteContactManagerUPtr,tesseract_collision::DiscreteContactManager)
%shared_ptr(tesseract_collision::tesseract_collision_fcl::FCLDiscreteBVHManager)
%tesseract_aligned_vector(ContactResultVector, tesseract_collision::ContactResult);
// %tesseract_aligned_map_of_aligned_vector(ContactResultMap, %arg(std::pair<std::string,std::string>), tesseract_collision::ContactResult);

namespace tesseract_collision { 
class ContactResult;
%tesseract_aligned_vector_using(ContactResultVector, tesseract_collision::ContactResult);
// %tesseract_aligned_map_of_aligned_vector_using(ContactResultMap, %arg(std::pair<std::string,std::string>), tesseract_collision::ContactResult);
}
%ignore ContactResultVector;
%ignore trajectoryCollisionResultsTable;
// %ignore ContactResultMap;
// tesseract_collision
#define TESSERACT_COLLISION_CORE_PUBLIC
%ignore tesseract_collision::ContactTrajectoryResults::trajectoryCollisionResultsTable;
%ignore tesseract_collision::ContactTrajectoryResults::collisionFrequencyPerLink;
%include "tesseract_collision/core/types.h"
%include "tesseract_collision/core/discrete_contact_manager.h"
%include "tesseract_collision/core/continuous_contact_manager.h"
%include "tesseract_collision/core/contact_managers_plugin_factory.h"

%init %{
// TODO: fix anchors
tesseract_common::PluginLoader::addSymbolLibraryToSearchLibrariesEnv(tesseract_collision::tesseract_collision_bullet::BulletFactoriesAnchor(), "TESSERACT_CONTACT_MANAGERS_PLUGINS");
tesseract_common::PluginLoader::addSymbolLibraryToSearchLibrariesEnv(tesseract_collision::tesseract_collision_fcl::FCLFactoriesAnchor(), "TESSERACT_CONTACT_MANAGERS_PLUGINS");
%}
