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

#ifndef RMF_TASK__HEADER_HPP
#define RMF_TASK__HEADER_HPP

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <string>

namespace rmf_task {

//==============================================================================
class Header
{
public:

  /// Constructor
  ///
  /// \param[in] category_
  ///   Category of the subject
  ///
  /// \param[in] detail_
  ///   Details about the subject
  ///
  /// \param[in] estimate_
  ///   The original (ideal) estimate of how long the subject will last
  Header(
    std::string category_,
    std::string detail_,
    rmf_traffic::Duration estimate_);

  /// Category of the subject
  const std::string& category() const;

  /// Details about the subject
  const std::string& detail() const;

  /// The original (ideal) estimate of how long the subject will last
  rmf_traffic::Duration original_duration_estimate() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
std::string standard_waypoint_name(
  const rmf_traffic::agv::Graph& graph,
  std::size_t waypoint);

} // namespace rmf_task

#endif // RMF_TASK__HEADER_HPP
