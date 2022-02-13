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

#include <rmf_task/Header.hpp>

namespace rmf_task {

//==============================================================================
class Header::Implementation
{
public:

  std::string category;
  std::string detail;
  rmf_traffic::Duration duration;

};

//==============================================================================
Header::Header(
  std::string category_,
  std::string detail_,
  rmf_traffic::Duration estimate_)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(category_), std::move(detail_), estimate_
      }))
{
  // Do nothing
}

//==============================================================================
const std::string& Header::category() const
{
  return _pimpl->category;
}

//==============================================================================
const std::string& Header::detail() const
{
  return _pimpl->detail;
}

//==============================================================================
rmf_traffic::Duration Header::original_duration_estimate() const
{
  return _pimpl->duration;
}

//==============================================================================
std::string standard_waypoint_name(
  const rmf_traffic::agv::Graph& graph,
  std::size_t waypoint)
{
  if (waypoint >= graph.num_waypoints())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[rmf_task::standard_waypoint_name] Waypoint index ["
      + std::to_string(waypoint) + "] is too high for the number of waypoints ["
      + std::to_string(graph.num_waypoints()) + "] in the graph");
    // *INDENT-ON*
  }

  return graph.get_waypoint(waypoint).name_or_index(
    "[place:%s]", "[graph-wp:%d]");
}

} // namespace rmf_task
