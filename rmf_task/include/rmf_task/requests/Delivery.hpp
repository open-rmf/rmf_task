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

#ifndef RMF_TASK__REQUESTS__DELIVERY_HPP
#define RMF_TASK__REQUESTS__DELIVERY_HPP

#include <chrono>
#include <string>
#include <optional>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_battery/MotionPowerSink.hpp>
#include <rmf_battery/DevicePowerSink.hpp>

#include <rmf_task/State.hpp>
#include <rmf_task/Request.hpp>
#include <rmf_task/Estimate.hpp>
#include <rmf_task/Payload.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
/// A class that generates a Request which requires an AGV to pickup items from
/// one location and deliver them to another
class Delivery
{
public:

  // Forward declare the Model for this request
  class Model;

  class Description : public Task::Description
  {
  public:

    using Start = rmf_traffic::agv::Planner::Start;

    /// Generate the description for this request
    static Task::ConstDescriptionPtr make(
      std::size_t pickup_waypoint,
      rmf_traffic::Duration pickup_duration,
      std::size_t dropoff_waypoint,
      rmf_traffic::Duration dropoff_duration,
      Payload payload,
      std::string pickup_from_dispenser = "",
      std::string dropoff_to_ingestor = "");

    // Documentation inherited
    Task::ConstModelPtr make_model(
      rmf_traffic::Time earliest_start_time,
      const Parameters& parameters) const final;

    // Documentation inherited
    Info generate_info(
      const State& initial_state,
      const Parameters& parameters) const final;

    /// Get the pickup waypoint in this request
    std::size_t pickup_waypoint() const;

    /// Get the name of the dispenser that we're picking up from
    std::string pickup_from_dispenser() const;

    /// Get the duration over which delivery items are loaded
    rmf_traffic::Duration pickup_wait() const;

    /// Get the dropoff waypoint in this request
    std::size_t dropoff_waypoint() const;

    /// Get the name of the ingestor that we're dropping off to
    std::string dropoff_to_ingestor() const;

    /// Get the duration over which delivery items are unloaded
    rmf_traffic::Duration dropoff_wait() const;

    /// Get the payload that is being delivered
    const Payload& payload() const;

    class Implementation;
  private:
    Description();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Generate a delivery request
  ///
  /// \param[in] pickup_waypoint
  ///   The graph index for the pickup location
  ///
  /// \param[in] pickup_wait
  ///   The expected duration the AGV has to wait at the pickup location for
  ///   the items to be loaded
  ///
  /// \param[in] dropoff_waypoint
  ///   The graph index for the dropoff location
  ///
  /// \param[in] dropoff_wait
  ///   The expected duration the AGV has to wait at the dropoff location for
  ///   the items to be unloaded
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
    std::size_t pickup_waypoint,
    rmf_traffic::Duration pickup_wait,
    std::size_t dropoff_waypoint,
    rmf_traffic::Duration dropoff_wait,
    Payload payload,
    const std::string& id,
    rmf_traffic::Time earliest_start_time,
    ConstPriorityPtr priority = nullptr,
    bool automatic = false,
    std::string pickup_from_dispenser = "",
    std::string dropoff_to_ingestor = "");
};

} // namespace requests
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__DELIVERY_HPP
