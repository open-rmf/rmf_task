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

#include <rmf_task/Parameters.hpp>

namespace rmf_task {

//==============================================================================
class Parameters::Implementation
{
public:
  std::shared_ptr<const rmf_traffic::agv::Planner> planner;
  rmf_battery::agv::BatterySystem battery_system;
  rmf_battery::ConstMotionPowerSinkPtr motion_sink;
  rmf_battery::ConstDevicePowerSinkPtr ambient_sink;
  rmf_battery::ConstDevicePowerSinkPtr tool_sink;
};

//==============================================================================
Parameters::Parameters(
  std::shared_ptr<const rmf_traffic::agv::Planner> planner,
  rmf_battery::agv::BatterySystem battery_system,
  rmf_battery::ConstMotionPowerSinkPtr motion_sink,
  rmf_battery::ConstDevicePowerSinkPtr ambient_sink,
  rmf_battery::ConstDevicePowerSinkPtr tool_sink)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(planner),
        battery_system,
        std::move(motion_sink),
        std::move(ambient_sink),
        std::move(tool_sink)
      }))
{
  // Do nothing
}

//==============================================================================
const rmf_battery::agv::BatterySystem&
Parameters::battery_system() const
{
  return _pimpl->battery_system;
}

//==============================================================================
auto Parameters::battery_system(
  rmf_battery::agv::BatterySystem battery_system) -> Parameters&
{
  _pimpl->battery_system = battery_system;
  return *this;
}

//==============================================================================
const std::shared_ptr<const rmf_battery::MotionPowerSink>&
Parameters::motion_sink() const
{
  return _pimpl->motion_sink;
}

//==============================================================================
auto Parameters::motion_sink(
  rmf_battery::ConstMotionPowerSinkPtr motion_sink) -> Parameters&
{
  _pimpl->motion_sink = std::move(motion_sink);
  return *this;
}

//==============================================================================
const rmf_battery::ConstDevicePowerSinkPtr&
Parameters::ambient_sink() const
{
  return _pimpl->ambient_sink;
}

//==============================================================================
auto Parameters::ambient_sink(
  rmf_battery::ConstDevicePowerSinkPtr ambient_sink) -> Parameters&
{
  _pimpl->ambient_sink = std::move(ambient_sink);
  return *this;
}

//==============================================================================
const std::shared_ptr<const rmf_traffic::agv::Planner>&
Parameters::planner() const
{
  return _pimpl->planner;
}

//==============================================================================
auto Parameters::planner(
  std::shared_ptr<const rmf_traffic::agv::Planner> planner) -> Parameters&
{
  _pimpl->planner = std::move(planner);
  return *this;
}

//==============================================================================
const rmf_battery::ConstDevicePowerSinkPtr&
Parameters::tool_sink() const
{
  return _pimpl->tool_sink;
}

//==============================================================================
auto Parameters::tool_sink(
  rmf_battery::ConstDevicePowerSinkPtr tool_sink) -> Parameters&
{
  _pimpl->tool_sink = std::move(tool_sink);
  return *this;
}

} // namespace task
