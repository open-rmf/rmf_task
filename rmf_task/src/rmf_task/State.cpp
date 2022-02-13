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

#include <stdexcept>

#include <rmf_task/State.hpp>

namespace rmf_task {

//==============================================================================
std::optional<std::size_t> State::waypoint() const
{
  if (const auto* wp = get<CurrentWaypoint>())
    return wp->value;

  return std::nullopt;
}

//==============================================================================
State& State::waypoint(std::size_t new_waypoint)
{
  with<CurrentWaypoint>(new_waypoint);
  return *this;
}

//==============================================================================
std::optional<double> State::orientation() const
{
  if (const auto* ori = get<CurrentOrientation>())
    return ori->value;

  return std::nullopt;
}

//==============================================================================
State& State::orientation(double new_orientation)
{
  with<CurrentOrientation>(new_orientation);
  return *this;
}

//==============================================================================
std::optional<rmf_traffic::Time> State::time() const
{
  if (const auto* time = get<CurrentTime>())
    return time->value;

  return std::nullopt;
}

//==============================================================================
State& State::time(rmf_traffic::Time new_time)
{
  with<CurrentTime>(new_time);
  return *this;
}

//==============================================================================
std::optional<std::size_t> State::dedicated_charging_waypoint() const
{
  if (const auto* p = get<DedicatedChargingPoint>())
    return p->value;

  return std::nullopt;
}

//==============================================================================
State& State::dedicated_charging_waypoint(std::size_t new_charging_waypoint)
{
  with<DedicatedChargingPoint>(new_charging_waypoint);
  return *this;
}

//==============================================================================
std::optional<double> State::battery_soc() const
{
  if (const auto* b = get<CurrentBatterySoC>())
    return b->value;

  return std::nullopt;
}

//==============================================================================
State& State::battery_soc(double new_battery_soc)
{
  if (new_battery_soc < 0.0 || new_battery_soc > 1.0)
  {
    // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
    throw std::invalid_argument(
      "Battery State of Charge needs to be between 0.0 and 1.0.");
    // *INDENT-ON*
  }

  with<CurrentBatterySoC>(new_battery_soc);
  return *this;
}

//==============================================================================
State& State::load_basic(
  const rmf_traffic::agv::Plan::Start& input_location,
  std::size_t input_charging_point,
  double input_battery_soc)
{
  load(input_location);
  dedicated_charging_waypoint(input_charging_point);
  battery_soc(input_battery_soc);
  return *this;
}

//==============================================================================
State& State::load(const rmf_traffic::agv::Plan::Start& location)
{
  with<CurrentWaypoint>(location.waypoint());
  with<CurrentOrientation>(location.orientation());
  with<CurrentTime>(location.time());
  return *this;
}

//==============================================================================
std::optional<rmf_traffic::agv::Plan::Start> State::project_plan_start(
  double default_orientation,
  rmf_traffic::Time default_time) const
{
  const auto* wp = get<CurrentWaypoint>();
  if (!wp)
    return std::nullopt;

  rmf_traffic::agv::Plan::Start start(
    default_time, wp->value, default_orientation);

  if (const auto* ori = get<CurrentOrientation>())
    start.orientation(ori->value);

  if (const auto* t = get<CurrentTime>())
    start.time(t->value);

  return start;
}

//==============================================================================
std::optional<rmf_traffic::agv::Plan::Start> State::extract_plan_start() const
{
  const auto* wp = get<CurrentWaypoint>();
  if (!wp)
    return std::nullopt;

  const auto* ori = get<CurrentOrientation>();
  if (!ori)
    return std::nullopt;

  const auto* t = get<CurrentTime>();
  if (!t)
    return std::nullopt;

  return rmf_traffic::agv::Plan::Start(t->value, wp->value, ori->value);
}

} // namespace rmf_task
