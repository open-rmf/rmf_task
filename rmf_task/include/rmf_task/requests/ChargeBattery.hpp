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

#ifndef RMF_TASK__REQUESTS__CHARGEBATTERY_HPP
#define RMF_TASK__REQUESTS__CHARGEBATTERY_HPP

#include <string>
#include <optional>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/MotionPowerSink.hpp>
#include <rmf_battery/DevicePowerSink.hpp>

#include <rmf_task/State.hpp>
#include <rmf_task/Request.hpp>
#include <rmf_task/Estimate.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
/// A class that generates a Request which requires an AGV to return to its
/// desginated charging_waypoint as specified in its agv::State and wait till
/// its battery charges up to the recharge_soc confugred in agv::Constraints
class ChargeBattery
{
public:

  // Forward declare the model for this request
  class Model;

  class Description : public Task::Description
  {
  public:

    /// Generate the description for this request
    static Task::ConstDescriptionPtr make();

    // Documentation inherited
    Task::ConstModelPtr make_model(
      rmf_traffic::Time earliest_start_time,
      const Parameters& parameters) const final;

    // Documentation inherited
    Info generate_info(
      const State& initial_state,
      const Parameters& parameters) const final;

    class Implementation;
  private:
    Description();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Generate a chargebattery request
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
    rmf_traffic::Time earliest_start_time,
    ConstPriorityPtr priority = nullptr,
    bool automatic = true);
};

} // namespace requests
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__CHARGEBATTERY_HPP
