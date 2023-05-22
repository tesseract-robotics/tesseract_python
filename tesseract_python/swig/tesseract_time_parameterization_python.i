/**
 * @file tesseract_time_parameterization_python.i
 * @brief The tesseract_time_parameterization_python SWIG master file.
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

%module(directors="1", package="tesseract_robotics.tesseract_time_parameterization") tesseract_time_parameterization_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

%import "tesseract_command_language_python.i"

%{


#include <tesseract_common/resource_locator.h>

// tesseract_time_parameterization
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization.h>
#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation.h>
#include <tesseract_time_parameterization/ruckig/ruckig_trajectory_smoothing.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>
%}

%ignore tesseract_planning::totg;

// tesseract_time_parameterization
#define TESSERACT_TIME_PARAMETERIZATION_PUBLIC
%shared_ptr(tesseract_planning::TrajectoryContainer)
%include "tesseract_time_parameterization/core/trajectory_container.h"

%shared_ptr(tesseract_planning::InstructionsTrajectory)
%include "tesseract_time_parameterization/core/instructions_trajectory.h"

// tesseract_time_parameterization_isp
%include "tesseract_time_parameterization/isp/iterative_spline_parameterization.h"

// tesseract_time_parameterization_totg
%shared_ptr(tesseract_planning::TimeOptimalTrajectoryGeneration)
%include "tesseract_time_parameterization/totg/time_optimal_trajectory_generation.h"

// tesseract_time_parameterization_ruckig
%include "tesseract_time_parameterization/ruckig/ruckig_trajectory_smoothing.h"

