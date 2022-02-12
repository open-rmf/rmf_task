/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "utils.hpp"

#include <rmf_task/Header.hpp>

namespace rmf_task_sequence {
namespace events {
namespace utils {

//==============================================================================
std::string waypoint_name(
  const std::size_t index,
  const rmf_task::Parameters& parameters)
{
  return rmf_task::standard_waypoint_name(
    parameters.planner()->get_configuration().graph(),
    index);
}

//==============================================================================
void fail(const std::string& header, const std::string& msg)
{
  throw std::runtime_error(
          header + " " + msg);
}

} // namespace utils
} // namespace events
} // namespace rmf_task_sequence
