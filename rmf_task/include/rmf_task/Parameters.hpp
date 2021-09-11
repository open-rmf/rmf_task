/*
 * Copyright (C) 2021 Open Source agentics Foundation
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

#ifndef RMF_TASK__AGV__PARAMETERS_HPP
#define RMF_TASK__AGV__PARAMETERS_HPP

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/MotionPowerSink.hpp>
#include <rmf_battery/DevicePowerSink.hpp>

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_task {

//==============================================================================
/// A class that containts parameters that are common among the agents/AGVs
/// available for performing requests
class Parameters
{
public:
  /// Constructor
  ///
  /// \param[in] battery_system
  ///   The battery system of the agent
  ///
  /// \param[in] motion_sink
  ///   The motion sink of the agent. This describes how power gets drained
  ///   while the agent is moving.
  ///
  /// \param[in] ambient_sink
  ///   The ambient device sink of the agent. This describes how power gets
  ///   drained at all times from passive use of the agent's electronics.
  ///
  /// \param[in] planner
  ///   The planner for a agent in this fleet
  ///
  /// \param[in] tool_sink
  ///   An additional device sink of the agent. This describes how power gets
  ///   drained when a tool/device is active.
  Parameters(
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    rmf_battery::agv::BatterySystem battery_system,
    rmf_battery::ConstMotionPowerSinkPtr motion_sink,
    rmf_battery::ConstDevicePowerSinkPtr ambient_sink,
    rmf_battery::ConstDevicePowerSinkPtr tool_sink = nullptr);

  /// Get the planner
  const std::shared_ptr<const rmf_traffic::agv::Planner>& planner() const;

  /// Set the planner
  Parameters& planner(
    std::shared_ptr<const rmf_traffic::agv::Planner> planner);

  /// Get the battery system
  const rmf_battery::agv::BatterySystem& battery_system() const;

  /// Set the battery_system
  Parameters& battery_system(
    rmf_battery::agv::BatterySystem battery_system);

  /// Get the motion sink
  const rmf_battery::ConstMotionPowerSinkPtr& motion_sink() const;

  /// Set the motion_sink
  Parameters& motion_sink(
    rmf_battery::ConstMotionPowerSinkPtr motion_sink);

  /// Get the ambient device sink
  const rmf_battery::ConstDevicePowerSinkPtr& ambient_sink() const;

  /// Set the ambient device sink
  Parameters& ambient_sink(
    rmf_battery::ConstDevicePowerSinkPtr ambient_sink);

  /// Get the tool device sink
  const rmf_battery::ConstDevicePowerSinkPtr& tool_sink() const;

  /// Set the tool device sink
  Parameters& tool_sink(
    rmf_battery::ConstDevicePowerSinkPtr tool_sink);

  class Implementation;

private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
using ConstParametersPtr = std::shared_ptr<const Parameters>;

} // namespace rmf_task

#endif // RMF_TASK__AGV__PARAMETERS_HPP
