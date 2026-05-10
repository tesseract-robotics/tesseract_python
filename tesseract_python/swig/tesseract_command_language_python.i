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

#include <tinyxml2.h>

// tesseract_common
#include <tesseract/common/manipulator_info.h>

// tesseract_command_language
#include <tesseract/command_language/poly/waypoint_poly.h>
#include <tesseract/command_language/poly/instruction_poly.h>
#include <tesseract/command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract/command_language/poly/joint_waypoint_poly.h>
#include <tesseract/command_language/poly/move_instruction_poly.h>
#include <tesseract/command_language/poly/state_waypoint_poly.h>

#include <tesseract/command_language/instruction_type.h>
#include <tesseract/command_language/cartesian_waypoint.h>
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/command_language/constants.h>
#include <tesseract/command_language/instruction_type.h>
#include <tesseract/command_language/joint_waypoint.h>
#include <tesseract/command_language/move_instruction.h>
#include <tesseract/command_language/set_analog_instruction.h>
#include <tesseract/command_language/set_tool_instruction.h>
#include <tesseract/command_language/state_waypoint.h>
#include <tesseract/command_language/timer_instruction.h>
#include <tesseract/command_language/utils.h>
#include <tesseract/command_language/fwd.h>
#include <tesseract/command_language/wait_instruction.h>



#include <tesseract/common/resource_locator.h>

#include "tesseract_command_language_python_std_functions.h"

%}

%include "tesseract_vector_reference_wrapper_instruction_typemaps.i"

// tesseract_command_language
#define TESSERACT_COMMAND_LANGUAGE_PUBLIC

%include "tesseract_type_erasure_macros.i"

#define TESSERACT_CARTESIAN_WAYPOINT_EXPORT_KEY(a,b)
#define TESSERACT_STATE_WAYPOINT_EXPORT_KEY(a,b)
#define TESSERACT_INSTRUCTION_EXPORT_KEY(a,b)

%define %tesseract_erasure_ctor_command_language(source_class_type,dest_class_type)
%tesseract_erasure_ctor(source_class_type,tesseract::command_language,dest_class_type,tesseract::command_language);
%enddef

%define %tesseract_erasure_as_command_language(source_class_type,dest_class_type)
%tesseract_erasure_as(source_class_type,tesseract::command_language,dest_class_type,tesseract::command_language);
%enddef

%define %tesseract_any_poly_type_command_language(TYPE)
%tesseract_any_poly_type(TYPE,tesseract::command_language);
%enddef

%define %tesseract_command_language_add_waypoint_type(TYPE)
/*%extend tesseract::command_language::WaypointPoly {
tesseract::command_language::TYPE as_ ## TYPE() {return $self->as<tesseract::command_language::TYPE>();}
const tesseract::command_language::TYPE as_const_ ## TYPE() {return $self->as<const tesseract::command_language::TYPE>();}
}*/
%tesseract_erasure_ctor_command_language(WaypointPoly,TYPE);
%tesseract_erasure_as_command_language(WaypointPoly,TYPE);
%tesseract_any_poly_type_command_language(TYPE);
%enddef

%define %tesseract_command_language_add_instruction_type(TYPE)
/*%extend tesseract::command_language::InstructionPoly {
tesseract::command_language::TYPE  as_ ## TYPE() {return $self->as<tesseract::command_language::TYPE>();}
const tesseract::command_language::TYPE as_const_ ## TYPE() {return $self->as<const tesseract::command_language::TYPE>();}
}*/
%tesseract_erasure_ctor_command_language(InstructionPoly,TYPE);
%tesseract_erasure_as_command_language(InstructionPoly,TYPE);
%tesseract_any_poly_type_command_language(TYPE);
%enddef

%define %tesseract_command_language_add_waypoint_poly_type(TYPE)
/*%extend tesseract::command_language::WaypointPoly {
tesseract::command_language::TYPE as_ ## TYPE() {return $self->as<tesseract::command_language::TYPE>();}
const tesseract::command_language::TYPE as_const_ ## TYPE() {return $self->as<const tesseract::command_language::TYPE>();}
}*/
%tesseract_erasure_as_command_language(WaypointPoly,TYPE);
%tesseract_erasure_as_command_language(TYPE,WaypointPoly);
%tesseract_any_poly_type_command_language(TYPE);
%enddef

%define %tesseract_command_language_add_instruction_poly_type(TYPE)
/*%extend tesseract::command_language::InstructionPoly {
tesseract::command_language::TYPE  as_ ## TYPE() {return $self->as<tesseract::command_language::TYPE>();}
const tesseract::command_language::TYPE as_const_ ## TYPE() {return $self->as<const tesseract::command_language::TYPE>();}
}*/
%tesseract_erasure_as_command_language(InstructionPoly,TYPE);
%tesseract_erasure_as_command_language(TYPE,InstructionPoly);
%tesseract_any_poly_type_command_language(TYPE);
%enddef

%tesseract_std_function(flattenFilterFn,tesseract::command_language,bool,const tesseract::command_language::InstructionPoly&,a,const tesseract::command_language::CompositeInstruction&,b,bool,c);
%tesseract_std_function(locateFilterFn,tesseract::command_language,bool,const tesseract::command_language::InstructionPoly&,a,const tesseract::command_language::CompositeInstruction&,b,bool,c);

// %include "tesseract/command_language/fwd.h"

%include "tesseract/command_language/types.h"


// Temporarily override return of non-const eigen reference
%typemap(out, fragment="Eigen_Fragments") Eigen::VectorXd &
{
  if (!ConvertFromEigenToNumPyMatrix<Eigen::VectorXd>(&$result, $1))
    SWIG_fail;
}

%pythondynamic tesseract::command_language::WaypointPoly;


%pythondynamic tesseract::command_language::InstructionPoly;
%pythondynamic tesseract::command_language::WaypointPoly;

%include "tesseract/command_language/poly/waypoint_poly.h"
%include "tesseract/command_language/poly/cartesian_waypoint_poly.h"
%include "tesseract/command_language/poly/joint_waypoint_poly.h"
%include "tesseract/command_language/poly/state_waypoint_poly.h"
%include "tesseract/command_language/poly/instruction_poly.h"
%include "tesseract/command_language/poly/move_instruction_poly.h"

%tesseract_command_language_add_waypoint_poly_type(CartesianWaypointPoly)
%tesseract_erasure_ctor_command_language(CartesianWaypointPoly,CartesianWaypoint);
%tesseract_command_language_add_waypoint_poly_type(JointWaypointPoly)
%tesseract_erasure_ctor_command_language(JointWaypointPoly,JointWaypoint);
%tesseract_command_language_add_waypoint_poly_type(StateWaypointPoly)
%tesseract_erasure_ctor_command_language(StateWaypointPoly,StateWaypoint);

%tesseract_command_language_add_instruction_poly_type(MoveInstructionPoly)
%tesseract_erasure_ctor_command_language(MoveInstructionPoly,MoveInstruction);

%template(Waypoints) std::vector<tesseract::command_language::WaypointPoly>;
%template(Instructions) std::vector<tesseract::command_language::InstructionPoly>;

%template(MoveInstructionPolyVector) std::vector<tesseract::command_language::MoveInstructionPoly>;

%wrap_unique_ptr(WaypointPolyUPtr,tesseract::command_language::WaypointPoly)
%wrap_unique_ptr(InstructionPolyUPtr,tesseract::command_language::InstructionPoly)
%wrap_unique_ptr(MoveInstructionPolyUPtr,tesseract::command_language::MoveInstructionPoly)
%wrap_unique_ptr(CartesianWaypointPolyUPtr,tesseract::command_language::CartesianWaypointPoly)
%wrap_unique_ptr(JointWaypointPolyUPtr,tesseract::command_language::JointWaypointPoly)
%wrap_unique_ptr(StateWaypointPolyUPtr,tesseract::command_language::StateWaypointPoly)   
%wrap_unique_ptr(WaypointInterfaceUPtr,tesseract::command_language::WaypointInterface)
%wrap_unique_ptr(InstructionInterfaceUPtr,tesseract::command_language::InstructionInterface)
%wrap_unique_ptr(MoveInstructionInterfaceUPtr,tesseract::command_language::MoveInstructionInterface)
%wrap_unique_ptr(CartesianWaypointInterfaceUPtr,tesseract::command_language::CartesianWaypointInterface)
%wrap_unique_ptr(JointWaypointInterfaceUPtr,tesseract::command_language::JointWaypointInterface)
%wrap_unique_ptr(StateWaypointInterfaceUPtr,tesseract::command_language::StateWaypointInterface)

%include "tesseract/command_language/instruction_type.h"

%include "tesseract/command_language/cartesian_waypoint.h"
%tesseract_command_language_add_waypoint_type(CartesianWaypoint)

#define TESSERACT_JOINT_WAYPOINT_EXPORT_KEY(a,b)
%include "tesseract/command_language/joint_waypoint.h"
%tesseract_command_language_add_waypoint_type(JointWaypoint)

%pythondynamic tesseract::command_language::StateWaypoint;
%include "tesseract/command_language/state_waypoint.h"
%tesseract_command_language_add_waypoint_type(StateWaypoint)


#define TESSERACT_MOVE_INSTRUCTION_EXPORT_KEY(a,b)
%include "tesseract/command_language/move_instruction.h"
%tesseract_command_language_add_instruction_type(MoveInstruction)

%include "tesseract/command_language/timer_instruction.h"
%tesseract_command_language_add_instruction_type(TimerInstruction)

%include "tesseract/command_language/wait_instruction.h"
%tesseract_command_language_add_instruction_type(WaitInstruction)

%include "tesseract/command_language/set_tool_instruction.h"
%tesseract_command_language_add_instruction_type(SetToolInstruction)

%include "tesseract/command_language/set_analog_instruction.h"
%tesseract_command_language_add_instruction_type(SetAnalogInstruction)

%include "rework_include/tesseract_command_language/composite_instruction.i"
%tesseract_command_language_add_instruction_type(CompositeInstruction)

// TODO: implement validateSeedStructure
%ignore validateSeedStructure;
%include "tesseract/command_language/utils.h"

%define %tesseract_command_language_add_profile_type( TYPE )
// %template(ProfileDictionary_hasProfileEntry_##TYPE) tesseract::command_language::ProfileDictionary_hasProfileEntry<tesseract::command_language::TYPE>;
// %template(ProfileDictionary_removeProfileEntry_##TYPE) tesseract::command_language::ProfileDictionary_removeProfileEntry<tesseract::command_language::TYPE>;
// %template(ProfileDictionary_getProfileEntry_##TYPE) tesseract::command_language::ProfileDictionary_getProfileEntry<tesseract::command_language::TYPE>;
// %template(ProfileDictionary_addProfile_##TYPE) tesseract::command_language::ProfileDictionary_addProfile<tesseract::command_language::TYPE>;
// %template(ProfileDictionary_getProfile_##TYPE) tesseract::command_language::ProfileDictionary_getProfile<tesseract::command_language::TYPE>;
// %template(ProfileDictionary_hasProfile_##TYPE) tesseract::command_language::ProfileDictionary_hasProfile<tesseract::command_language::TYPE>;
// %template(ProfileDictionary_removeProfile_##TYPE) tesseract::command_language::ProfileDictionary_removeProfile<tesseract::command_language::TYPE>;
%enddef

%define %tesseract_command_language_add_profile_type2(NAME, TYPE )
// %template(ProfileDictionary_hasProfileEntry_##NAME) tesseract::command_language::ProfileDictionary_hasProfileEntry<tesseract::command_language::TYPE>;
// %template(ProfileDictionary_removeProfileEntry_##NAME) tesseract::command_language::ProfileDictionary_removeProfileEntry<tesseract::command_language::TYPE>;
// %template(ProfileDictionary_getProfileEntry_##NAME) tesseract::command_language::ProfileDictionary_getProfileEntry<tesseract::command_language::TYPE>;
// %template(ProfileDictionary_addProfile_##NAME) tesseract::command_language::ProfileDictionary_addProfile<tesseract::command_language::TYPE>;
// %template(ProfileDictionary_getProfile_##NAME) tesseract::command_language::ProfileDictionary_getProfile<tesseract::command_language::TYPE>;
// %template(ProfileDictionary_hasProfile_##NAME) tesseract::command_language::ProfileDictionary_hasProfile<tesseract::command_language::TYPE>;
// %template(ProfileDictionary_removeProfile_##NAME) tesseract::command_language::ProfileDictionary_removeProfile<tesseract::command_language::TYPE>;
%enddef

%include "tesseract/command_language/constants.h"
