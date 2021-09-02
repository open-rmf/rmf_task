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

#include <rmf_task/requests/ParkRobotFactory.hpp>
#include <rmf_task/requests/Loop.hpp>

#include <random>

namespace rmf_task {
namespace requests {

//==============================================================================
namespace {
std::string generate_uuid(const std::size_t length = 3)
{
  std::stringstream ss;
  for (std::size_t i = 0; i < length; ++i)
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    const auto random_char = dis(gen);
    std::stringstream hexstream;
    hexstream << std::hex << random_char;
    auto hex = hexstream.str();
    ss << (hex.length() < 2 ? '0' + hex : hex);
  }
  return ss.str();
}

} // anonymous namespace

//==============================================================================
class ParkRobotFactory::Implementation
{
public:

  std::optional<std::size_t> parking_waypoint;
};

//==============================================================================
ParkRobotFactory::ParkRobotFactory(
  std::optional<std::size_t> parking_waypoint)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        parking_waypoint
      }))
{
  // Do nothing
}

//==============================================================================
ConstRequestPtr ParkRobotFactory::make_request(
  const State& state) const
{
  std::string id = "ParkRobot" + generate_uuid();
  const auto start_waypoint = state.waypoint().value();
  const auto finish_waypoint = _pimpl->parking_waypoint.has_value() ?
    _pimpl->parking_waypoint.value() :
    state.dedicated_charging_waypoint().value();

  const auto request = Loop::make(
    start_waypoint,
    finish_waypoint,
    1,
    id,
    state.time().value(),
    nullptr,
    true);

  return request;
}

} // namespace requests
} // namespace rmf_task
