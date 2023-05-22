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
%include "std_unique_ptr.i"

%import "tesseract_common_python.i"

%{

// tesseract_command_language
#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>

#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/constants.h>
#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/set_analog_instruction.h>
#include <tesseract_command_language/set_tool_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/timer_instruction.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_command_language/types.h>
#include <tesseract_command_language/wait_instruction.h>



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
  class_type (tesseract_planning::inner_type  inner_waypoint)
  {
     return new tesseract_planning::class_type (inner_waypoint);
  }
}
%enddef

%define %tesseract_erasure_as(source_class_type,dest_class_type)
%inline {
  tesseract_planning::dest_class_type source_class_type ## _as_ ## dest_class_type (tesseract_planning::source_class_type& self)
  {
    return self.as<tesseract_planning::dest_class_type>();
  }
}
%enddef

%define %tesseract_command_language_add_waypoint_type(TYPE)
/*%extend tesseract_planning::WaypointPoly {
tesseract_planning::TYPE as_ ## TYPE() {return $self->as<tesseract_planning::TYPE>();}
const tesseract_planning::TYPE as_const_ ## TYPE() {return $self->as<const tesseract_planning::TYPE>();}
}*/
%tesseract_erasure_ctor(WaypointPoly,TYPE);
%tesseract_erasure_as(WaypointPoly,TYPE);
%enddef

%define %tesseract_command_language_add_instruction_type(TYPE)
/*%extend tesseract_planning::InstructionPoly {
tesseract_planning::TYPE  as_ ## TYPE() {return $self->as<tesseract_planning::TYPE>();}
const tesseract_planning::TYPE as_const_ ## TYPE() {return $self->as<const tesseract_planning::TYPE>();}
}*/
%tesseract_erasure_ctor(InstructionPoly,TYPE);
%tesseract_erasure_as(InstructionPoly,TYPE);
%enddef

%define %tesseract_command_language_add_waypoint_poly_type(TYPE)
/*%extend tesseract_planning::WaypointPoly {
tesseract_planning::TYPE as_ ## TYPE() {return $self->as<tesseract_planning::TYPE>();}
const tesseract_planning::TYPE as_const_ ## TYPE() {return $self->as<const tesseract_planning::TYPE>();}
}*/
%tesseract_erasure_as(WaypointPoly,TYPE);
%tesseract_erasure_as(TYPE,WaypointPoly);
%enddef

%define %tesseract_command_language_add_instruction_poly_type(TYPE)
/*%extend tesseract_planning::InstructionPoly {
tesseract_planning::TYPE  as_ ## TYPE() {return $self->as<tesseract_planning::TYPE>();}
const tesseract_planning::TYPE as_const_ ## TYPE() {return $self->as<const tesseract_planning::TYPE>();}
}*/
%tesseract_erasure_as(InstructionPoly,TYPE);
%tesseract_erasure_as(TYPE,InstructionPoly);
%enddef

%tesseract_std_function(flattenFilterFn,tesseract_planning,bool,const tesseract_planning::InstructionPoly&,a,const tesseract_planning::CompositeInstruction&,b,bool,c);
%tesseract_std_function(locateFilterFn,tesseract_planning,bool,const tesseract_planning::InstructionPoly&,a,const tesseract_planning::CompositeInstruction&,b,bool,c);

%ignore tesseract_common::TypeErasureInterface::clone;
%include "tesseract_common/type_erasure.h"
%include "tesseract_command_language/types.h"

%shared_ptr(tesseract_planning::ProfileDictionary)
%include "tesseract_command_language/profile_dictionary.h"
%include "tesseract_command_language_python_profile_dictionary_functions.h"

%include "rework_include/tesseract_command_language/poly/waypoint_poly.i"
%include "rework_include/tesseract_command_language/poly/cartesian_waypoint_poly.i"
%include "rework_include/tesseract_command_language/poly/joint_waypoint_poly.i"
%include "rework_include/tesseract_command_language/poly/state_waypoint_poly.i"
%include "rework_include/tesseract_command_language/poly/instruction_poly.i"
%include "rework_include/tesseract_command_language/poly/move_instruction_poly.i"

%tesseract_command_language_add_waypoint_poly_type(CartesianWaypointPoly)
%tesseract_command_language_add_waypoint_poly_type(JointWaypointPoly)
%tesseract_command_language_add_waypoint_poly_type(StateWaypointPoly)

%tesseract_command_language_add_instruction_poly_type(MoveInstructionPoly)

%template(Waypoints) std::vector<tesseract_planning::WaypointPoly>;
%template(Instructions) std::vector<tesseract_planning::InstructionPoly>;

%template(MoveInstructionPolyVector) std::vector<tesseract_planning::MoveInstructionPoly>;

%include "tesseract_command_language/instruction_type.h"

%include "tesseract_command_language/cartesian_waypoint.h"
%tesseract_command_language_add_waypoint_type(CartesianWaypoint)

#define TESSERACT_JOINT_WAYPOINT_EXPORT_KEY(a,b)
%include "tesseract_command_language/joint_waypoint.h"
%tesseract_command_language_add_waypoint_type(JointWaypoint)

%pythondynamic tesseract_planning::StateWaypoint;
%include "tesseract_command_language/state_waypoint.h"
%tesseract_command_language_add_waypoint_type(StateWaypoint)

%include "rework_include/tesseract_command_language/composite_instruction.i"
%tesseract_command_language_add_instruction_type(CompositeInstruction)

#define TESSERACT_MOVE_INSTRUCTION_EXPORT_KEY(a,b)
%include "tesseract_command_language/move_instruction.h"
%tesseract_command_language_add_instruction_type(MoveInstruction)

%include "tesseract_command_language/timer_instruction.h"
%tesseract_command_language_add_instruction_type(TimerInstruction)

%include "tesseract_command_language/wait_instruction.h"
%tesseract_command_language_add_instruction_type(WaitInstruction)

%include "tesseract_command_language/set_tool_instruction.h"
%tesseract_command_language_add_instruction_type(SetToolInstruction)

%include "tesseract_command_language/set_analog_instruction.h"
%tesseract_command_language_add_instruction_type(SetAnalogInstruction)

// TODO: implement validateSeedStructure
%ignore validateSeedStructure;
%include "tesseract_command_language/utils.h"

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
