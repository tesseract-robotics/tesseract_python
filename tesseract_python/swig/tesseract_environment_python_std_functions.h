/**
 * @file tesseract_environment_python_std_functions.h
 * @brief Callback directors for tesseract_python module
 *
 * @author John Wason
 * @date December 18, 2020
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

#include <tesseract_environment/environment.h>

#pragma once

class FindTCPOffsetCallbackFnBase
{
public:
  virtual Eigen::Isometry3d call(const tesseract_common::ManipulatorInfo& a) = 0;
  virtual ~FindTCPOffsetCallbackFnBase() {}
};

class EventCallbackFnBase
{
public:
  virtual void call(const tesseract_environment::Event& a) = 0;
  virtual ~EventCallbackFnBase() {}
};
