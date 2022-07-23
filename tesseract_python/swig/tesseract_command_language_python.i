/**
 * @file tesseract_command_language_python.i
 * @brief The tesseract_command_language_python SWIG master file.
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

%module(directors="1", package="tesseract_robotics.tesseract_command_language") tesseract_command_language_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"
%include "tesseract_std_function.i"

%import "tesseract_common_python.i"

%{

// tesseract_command_language
#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/command_language.h>

#include <tesseract_command_language/profile_dictionary.h>

#include <tesseract_command_language/utils/utils.h>
#include <tesseract_command_language/utils/get_instruction_utils.h>
#include <tesseract_command_language/utils/filter_functions.h>
#include <tesseract_command_language/utils/flatten_utils.h>

#include <tesseract_common/status_code.h>
#include <tesseract_common/resource_locator.h>

#include "tesseract_command_language_python_std_functions.h"
#include "tesseract_command_language_python_profile_dictionary_functions.h"

#include <tesseract_common/type_erasure.h>
%}

%include "tesseract_vector_reference_wrapper_instruction_typemaps.i"

// tesseract_command_language
#define TESSERACT_COMMAND_LANGUAGE_PUBLIC

%define %tesseract_erasure_ctor(class_type,inner_type)
%extend tesseract_planning::class_type {
  class_type (tesseract_planning::inner_type && inner_waypoint)
  {
     return new tesseract_planning::class_type (inner_waypoint);
  }
}
%enddef

%define %tesseract_command_language_add_waypoint_type(TYPE)
%extend tesseract_planning::Waypoint {
tesseract_planning::TYPE as_ ## TYPE() {return $self->as<tesseract_planning::TYPE>();}
const tesseract_planning::TYPE as_const_ ## TYPE() {return $self->as<const tesseract_planning::TYPE>();}
}
%tesseract_erasure_ctor(Waypoint,TYPE);
%enddef

%define %tesseract_command_language_add_instruction_type(TYPE)
%extend tesseract_planning::Instruction {
tesseract_planning::TYPE  as_ ## TYPE() {return $self->as<tesseract_planning::TYPE>();}
const tesseract_planning::TYPE as_const_ ## TYPE() {return $self->as<const tesseract_planning::TYPE>();}
}
%tesseract_erasure_ctor(Instruction,TYPE);
%enddef

%tesseract_std_function(flattenFilterFn,tesseract_planning,bool,const tesseract_planning::Instruction&,a,const tesseract_planning::CompositeInstruction&,b,bool,c);
%tesseract_std_function(locateFilterFn,tesseract_planning,bool,const tesseract_planning::Instruction&,a,const tesseract_planning::CompositeInstruction&,b,bool,c);

%ignore tesseract_common::TypeErasureInterface::clone;
%include "tesseract_common/type_erasure.h"
%include "tesseract_command_language/types.h"

%shared_ptr(tesseract_planning::ProfileDictionary)
%include "tesseract_command_language/profile_dictionary.h"
%include "tesseract_command_language_python_profile_dictionary_functions.h"

//%template(Waypoints) std::vector<tesseract_planning::Waypoint>;
%ignore tesseract_planning::Waypoint::getType;
%include "tesseract_command_language/core/waypoint.h"

%ignore std::vector<tesseract_planning::Instruction>::vector(size_type);
%ignore std::vector<tesseract_planning::Instruction>::resize(size_type);
%ignore tesseract_planning::Instruction::getType;
%pythondynamic tesseract_planning::Instruction;
%include "tesseract_command_language/core/instruction.h"
%template(Instructions) std::vector<tesseract_planning::Instruction>;

%include "tesseract_command_language/command_language.h"

%include "tesseract_command_language/instruction_type.h"
%include "tesseract_command_language/null_instruction.h"
%tesseract_command_language_add_instruction_type(NullInstruction)

%include "tesseract_command_language/null_waypoint.h"
%tesseract_command_language_add_waypoint_type(NullWaypoint)

%include "tesseract_command_language/cartesian_waypoint.h"
%tesseract_command_language_add_waypoint_type(CartesianWaypoint)

%include "tesseract_command_language/composite_instruction.h"
%tesseract_command_language_add_instruction_type(CompositeInstruction)

%include "tesseract_command_language/joint_waypoint.h"
%extend tesseract_planning::JointWaypoint {
  JointWaypoint(std::vector<std::string> joint_names, const Eigen::VectorXd& other)
  {
    return new tesseract_planning::JointWaypoint(joint_names, other);
  }
}
%tesseract_command_language_add_waypoint_type(JointWaypoint)


%include "tesseract_command_language/move_instruction.h"
%tesseract_command_language_add_instruction_type(MoveInstruction)

%include "tesseract_command_language/state_waypoint.h"
%tesseract_command_language_add_waypoint_type(StateWaypoint)

%include "tesseract_command_language/waypoint_type.h"
%include "tesseract_command_language/timer_instruction.h"
%tesseract_command_language_add_instruction_type(TimerInstruction)

%include "tesseract_command_language/wait_instruction.h"
%tesseract_command_language_add_instruction_type(WaitInstruction)

%include "tesseract_command_language/set_tool_instruction.h"
%tesseract_command_language_add_instruction_type(SetToolInstruction)

%include "tesseract_command_language/set_analog_instruction.h"
%tesseract_command_language_add_instruction_type(SetAnalogInstruction)

%include "tesseract_command_language/utils/filter_functions.h"
%include "tesseract_command_language/utils/utils.h"
%include "tesseract_command_language/utils/get_instruction_utils.h"
%include "tesseract_command_language/utils/flatten_utils.h"

%define %tesseract_command_language_add_profile_type( TYPE )
%template(ProfileDictionary_hasProfileEntry_##TYPE) tesseract_planning::ProfileDictionary_hasProfileEntry<tesseract_planning::TYPE>;
%template(ProfileDictionary_removeProfileEntry_##TYPE) tesseract_planning::ProfileDictionary_removeProfileEntry<tesseract_planning::TYPE>;
%template(ProfileDictionary_getProfileEntry_##TYPE) tesseract_planning::ProfileDictionary_getProfileEntry<tesseract_planning::TYPE>;
%template(ProfileDictionary_addProfile_##TYPE) tesseract_planning::ProfileDictionary_addProfile<tesseract_planning::TYPE>;
%template(ProfileDictionary_getProfile_##TYPE) tesseract_planning::ProfileDictionary_getProfile<tesseract_planning::TYPE>;
%template(ProfileDictionary_hasProfile_##TYPE) tesseract_planning::ProfileDictionary_hasProfile<tesseract_planning::TYPE>;
%template(ProfileDictionary_removeProfile_##TYPE) tesseract_planning::ProfileDictionary_removeProfile<tesseract_planning::TYPE>;
%enddef

%define %tesseract_command_language_add_profile_type2(NAME, TYPE )
%template(ProfileDictionary_hasProfileEntry_##NAME) tesseract_planning::ProfileDictionary_hasProfileEntry<tesseract_planning::TYPE>;
%template(ProfileDictionary_removeProfileEntry_##NAME) tesseract_planning::ProfileDictionary_removeProfileEntry<tesseract_planning::TYPE>;
%template(ProfileDictionary_getProfileEntry_##NAME) tesseract_planning::ProfileDictionary_getProfileEntry<tesseract_planning::TYPE>;
%template(ProfileDictionary_addProfile_##NAME) tesseract_planning::ProfileDictionary_addProfile<tesseract_planning::TYPE>;
%template(ProfileDictionary_getProfile_##NAME) tesseract_planning::ProfileDictionary_getProfile<tesseract_planning::TYPE>;
%template(ProfileDictionary_hasProfile_##NAME) tesseract_planning::ProfileDictionary_hasProfile<tesseract_planning::TYPE>;
%template(ProfileDictionary_removeProfile_##NAME) tesseract_planning::ProfileDictionary_removeProfile<tesseract_planning::TYPE>;
%enddef
