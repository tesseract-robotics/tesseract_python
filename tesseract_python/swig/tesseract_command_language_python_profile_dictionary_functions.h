/**
 * @file tesseract_command_language_python_profile_dictionary_functions.h
 * @brief Utility functions for template types in tesseract_planning::ProfileDictionary
 *
 * @author John Wason
 * @date March 21, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Wason Technology, LLC
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

namespace tesseract_planning
{
  template<typename T>
  bool ProfileDictionary_hasProfileEntry(std::shared_ptr<tesseract_planning::ProfileDictionary> profile_dictionary)
  {
    return profile_dictionary->hasProfileEntry<T>();
  }

  template<typename T>
  void ProfileDictionary_removeProfileEntry(std::shared_ptr<tesseract_planning::ProfileDictionary> profile_dictionary)
  {
    profile_dictionary->removeProfileEntry<T>();
  }

  template<typename T>
  std::unordered_map<std::string, std::shared_ptr<const T>> ProfileDictionary_getProfileEntry(std::shared_ptr<tesseract_planning::ProfileDictionary> profile_dictionary)
  {
    return profile_dictionary->getProfileEntry<T>();
  }

  template<typename T>
  void ProfileDictionary_addProfile(std::shared_ptr<tesseract_planning::ProfileDictionary> profile_dictionary, const std::string& profile_name, std::shared_ptr<const T> profile)
  {
    profile_dictionary->addProfile(profile_name, profile);
  }

  template<typename T>
  bool ProfileDictionary_hasProfile(std::shared_ptr<tesseract_planning::ProfileDictionary> profile_dictionary, const std::string& profile_name)
  {
    return profile_dictionary->hasProfile<T>(profile_name);
  }

  template<typename T>
  std::shared_ptr<const T> ProfileDictionary_getProfile(std::shared_ptr<tesseract_planning::ProfileDictionary> profile_dictionary, const std::string& profile_name)
  {
    return profile_dictionary->getProfile<T>(profile_name);
  }

  template<typename T>
  void ProfileDictionary_removeProfile(std::shared_ptr<tesseract_planning::ProfileDictionary> profile_dictionary, const std::string& profile_name)
  {
    profile_dictionary->removeProfile<T>(profile_name);
  }

}