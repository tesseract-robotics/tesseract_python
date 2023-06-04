/**
 * @file boost_uuid.i
 * @brief Implement very simple wrapper for boost::uuids::uuid
 *
 * @author John Wason
 * @date June 2, 2023
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Wason Technology, LLC
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

 %{
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/random_generator.hpp>
 %}

 namespace boost {
namespace uuids {
    %rename(Uuid) uuid;
    /**
    * @brief Wrapper for boost::uuids::uuid
    *
    * @details
    * This class is a simple wrapper for boost::uuids::uuid.  It is used to
    * provide a simple interface for the uuid class in python.
    */
    class uuid
    {
    public:
    %extend {
        bool __eq__(const uuid& other) const
        {
            return *$self == other;
        }

        bool __neq__(const uuid& other) const
        {
            return *$self != other;
        }

        /**
        * @brief Convert uuid to string
        *
        */
        std::string __str__() const
        {
            return boost::uuids::to_string(*$self);
        }
    }
    };   


}
}

%inline {
    /**
    * @brief Generate a new random uuid
    *
    * @details
    * This function generates a new random uuid
    *
    * @return A new random uuid
    */

    boost::uuids::uuid newRandomUuid()
    {
        return boost::uuids::random_generator()();
    }
}
