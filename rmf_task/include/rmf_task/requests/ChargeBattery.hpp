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

    /// Make a charging task that will last indefinitely.
    static std::shared_ptr<Description> make_indefinite();

    // Documentation inherited
    Task::ConstModelPtr make_model(
      rmf_traffic::Time earliest_start_time,
      const Parameters& parameters) const final;

    // Documentation inherited
    Info generate_info(
      const State& initial_state,
      const Parameters& parameters) const final;

    /// Set the charging task to run indefinitely. This means it will never
    /// declare itself as finished and must instead be canceled. This can be
    /// used for idle tasks that are canceled automatically when a task request
    /// comes in. If indefinite is false, the robot will charge up to its
    /// designated recharge level.
    void set_indefinite(bool value);

    /// Should this recharge task run indefinitely?
    bool indefinite() const;

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

  /// Generate a chargebattery request.
  ///
  /// \param[in] earliest_start_time
  ///   The desired start time for this request.
  ///
  /// \param[in] requester
  ///   The entity that issued this request.
  ///
  /// \param[in] request_time
  ///   The time this request was generated or submitted.
  ///
  /// \param[in] priority
  ///   The priority for this request.
  ///
  /// \param[in] automatic
  ///   True if this request is auto-generated, default value as true.
  static ConstRequestPtr make(
    rmf_traffic::Time earliest_start_time,
    const std::string& requester,
    rmf_traffic::Time request_time,
    ConstPriorityPtr priority = nullptr,
    bool automatic = true);
};

} // namespace requests
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__CHARGEBATTERY_HPP
