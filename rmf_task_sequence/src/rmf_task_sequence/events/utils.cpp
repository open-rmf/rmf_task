/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

namespace rmf_task_sequence {
namespace events {
namespace utils {

std::string waypoint_name(
  const std::size_t index,
  const rmf_task::Parameters& parameters)
{
  const auto& graph = parameters.planner()->get_configuration().graph();
  if (index < graph.num_waypoints())
  {
    if (const auto* name = graph.get_waypoint(index).name())
      return *name;
  }

  return "#" + std::to_string(index);
}

} // namespace utils
} // namespace events
} // namespace rmf_task_sequence
