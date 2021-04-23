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

%module(directors="1", package="tesseract.tesseract_command_language") tesseract_command_language_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"
%include "tesseract_std_function.i"

%import "tesseract_common_python.i"

%{

// tesseract_command_language
#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/core/serialization.h>
#include <tesseract_command_language/command_language.h>

#include <tesseract_command_language/profile_dictionary.h>

#include <tesseract_command_language/utils/utils.h>
#include <tesseract_command_language/utils/get_instruction_utils.h>
#include <tesseract_command_language/utils/filter_functions.h>
#include <tesseract_command_language/utils/flatten_utils.h>

#include <tesseract_common/status_code.h>
#include <tesseract_common/resource.h>

#include "tesseract_command_language_python_std_functions.h"
#include "tesseract_command_language_python_profile_dictionary_functions.h"
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
%template(as_##TYPE) tesseract_planning::Waypoint::as<tesseract_planning::TYPE>;
%tesseract_erasure_ctor(Waypoint,TYPE);
%enddef

%define %tesseract_command_language_add_instruction_type(TYPE)
%template(as_##TYPE) tesseract_planning::Instruction::as<tesseract_planning::TYPE>;
%tesseract_erasure_ctor(Instruction,TYPE);
%enddef

%tesseract_std_function(flattenFilterFn,tesseract_planning,bool,const tesseract_planning::Instruction&,a,const tesseract_planning::CompositeInstruction&,b,bool,c);
%tesseract_std_function(locateFilterFn,tesseract_planning,bool,const tesseract_planning::Instruction&,a,const tesseract_planning::CompositeInstruction&,b,bool,c);

%include "tesseract_command_language/types.h"
%include "tesseract_command_language/profile_dictionary.h"
%include "tesseract_command_language_python_profile_dictionary_functions.h"
%include "tesseract_command_language/core/waypoint.h"
%include "tesseract_command_language/core/instruction.h"
%include "tesseract_command_language/core/serialization.h"
%include "tesseract_command_language/command_language.h"
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
