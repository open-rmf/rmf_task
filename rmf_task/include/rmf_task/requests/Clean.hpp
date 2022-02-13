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

#ifndef RMF_TASK__REQUESTS__CLEAN_HPP
#define RMF_TASK__REQUESTS__CLEAN_HPP

#include <chrono>
#include <string>
#include <optional>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_battery/MotionPowerSink.hpp>
#include <rmf_battery/DevicePowerSink.hpp>

#include <rmf_task/State.hpp>
#include <rmf_task/Request.hpp>
#include <rmf_task/Estimate.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
/// A class that generates a Request which requires an AGV to perform a cleaning
/// operation at a location
class Clean
{
public:

  // Forward declare the model for this request
  class Model;

  class Description : public Task::Description
  {
  public:

    /// Generate the description for this request
    static std::shared_ptr<Description> make(
      std::size_t start_waypoint,
      std::size_t end_waypoint,
      const rmf_traffic::Trajectory& cleaning_path);

    // Documentation inherited
    Task::ConstModelPtr make_model(
      rmf_traffic::Time earliest_start_time,
      const Parameters& parameters) const final;

    // Documentation inherited
    Info generate_info(
      const State& initial_state,
      const Parameters& parameters) const final;

    /// Get the start waypoint in this request
    std::size_t start_waypoint() const;

    /// Get the end waypoint in this request
    std::size_t end_waypoint() const;

    class Implementation;
  private:
    Description();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Generate a clean request
  ///
  /// \param[in] start_waypoint
  ///   The graph index for the location where the AGV should begin its cleaning
  ///   operation
  ///
  /// \param[in] end_waypoint
  ///   The graph index for the location where the AGV ends up after its cleaning
  ///   operation
  ///
  /// \param[in] cleaning_path
  ///   A trajectory that describes the motion of the AGV during the cleaning
  ///   operation. This is used to determine the process duration and expected
  ///   battery drain
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
    std::size_t end_waypoint,
    const rmf_traffic::Trajectory& cleaning_path,
    const std::string& id,
    rmf_traffic::Time earliest_start_time,
    ConstPriorityPtr priority = nullptr,
    bool automatic = false);
};

} // namespace requests
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__CLEAN_HPP
