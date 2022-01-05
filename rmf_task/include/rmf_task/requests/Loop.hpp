/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef RMF_TASK__REQUESTS__LOOP_HPP
#define RMF_TASK__REQUESTS__LOOP_HPP

#include <chrono>
#include <optional>
#include <string>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_battery/MotionPowerSink.hpp>
#include <rmf_battery/DevicePowerSink.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <rmf_task/State.hpp>
#include <rmf_task/Request.hpp>
#include <rmf_task/Estimate.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
/// A class that generates a Request which requires an AGV to repeatedly travel
/// between two locations
class Loop
{
public:

  // Forward declare the Model for this request
  class Model;

  class Description : public Task::Description
  {
  public:

    /// Generate the description for this request
    static Task::ConstDescriptionPtr make(
      std::size_t start_waypoint,
      std::size_t finish_waypoint,
      std::size_t num_loops);

    // Documentation inherited
    Task::ConstModelPtr make_model(
      rmf_traffic::Time earliest_start_time,
      const Parameters& parameters) const final;

    // Documentation inherited
    Info generate_info(
      const State& initial_state,
      const Parameters& parameters) const final;

    /// Get the start waypoint of the loop in this request
    std::size_t start_waypoint() const;

    /// Get the finish waypoint of the loop in this request
    std::size_t finish_waypoint() const;

    /// Get the numbert of loops in this request
    std::size_t num_loops() const;

    class Implementation;
  private:
    Description();

    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Generate a loop request
  ///
  /// \param[in] start_waypoint
  ///   The graph index for the starting waypoint of the loop
  ///
  /// \param[in] finish_waypoint
  ///   The graph index for the finishing waypoint of the loop
  ///
  /// \param[in] num_loops
  ///   The number of times the AGV should loop between the start_waypoint and
  ///   finish_waypoint
  ///
  /// \param[in] id
  ///   A unique id for this request
  ///
  /// \param[in] earliest_start_time
  ///   The desired start time for this request
  ///
  /// \param[in] priority
  ///   The priority for this request
  ///
  /// \param[in] automatic
  ///   True if this request is auto-generated
  static ConstRequestPtr make(
    std::size_t start_waypoint,
    std::size_t finish_waypoint,
    std::size_t num_loops,
    const std::string& id,
    rmf_traffic::Time earliest_start_time,
    ConstPriorityPtr priority = nullptr,
    bool automatic = false);
};

} // namespace tasks
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__LOOP_HPP
