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

#include <rmf_task/agv/State.hpp>
#include <rmf_task/Request.hpp>
#include <rmf_task/Estimate.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
class ChargeBatteryDescription : public rmf_task::Request::Description
{
public:

  static DescriptionPtr make(
    rmf_battery::agv::BatterySystem battery_system,
    rmf_battery::ConstMotionPowerSinkPtr motion_sink,
    rmf_battery::ConstDevicePowerSinkPtr device_sink,
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    rmf_traffic::Time start_time,
    double max_charge_soc = 1.0,
    bool drain_battery = true);

  std::optional<rmf_task::Estimate> estimate_finish(
    const agv::State& initial_state,
    const agv::Constraints& task_planning_constraints,
    const std::shared_ptr<EstimateCache> estimate_cache) const final;

  rmf_traffic::Duration invariant_duration() const final;

  /// Get the battery system in this request
  const rmf_battery::agv::BatterySystem& battery_system() const;

  /// Retrieve the state of charge to which the battery will be recharged
  double max_charge_soc() const;

  class Implementation;
private:
  ChargeBatteryDescription();

  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class ChargeBattery
{
public:

  static ConstRequestPtr make(
    rmf_battery::agv::BatterySystem battery_system,
    rmf_battery::ConstMotionPowerSinkPtr motion_sink,
    rmf_battery::ConstDevicePowerSinkPtr device_sink,
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    rmf_traffic::Time start_time,
    double max_charge_soc = 1.0,
    bool drain_battery = true,
    ConstPriorityPtr priority = nullptr);
};

using ChargeBatteryDescriptionPtr = std::shared_ptr<ChargeBatteryDescription>;

} // namespace requests
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__CHARGEBATTERY_HPP
