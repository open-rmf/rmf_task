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

#include <rmf_task/requests/ChargeBatteryFactory.hpp>
#include <rmf_task/requests/ChargeBattery.hpp>
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
class ChargeBatteryFactory::Implementation
{
public:
  std::optional<std::string> requester;
  std::function<rmf_traffic::Time()> time_now_cb;
  bool indefinite = false;
};

//==============================================================================
ChargeBatteryFactory::ChargeBatteryFactory()
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{std::nullopt, nullptr}))
{
  // Do nothing
}

//==============================================================================
ChargeBatteryFactory::ChargeBatteryFactory(
  const std::string& requester,
  std::function<rmf_traffic::Time()> time_now_cb)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{requester, std::move(time_now_cb)}))
{
  // Do nothing
}

//==============================================================================
void ChargeBatteryFactory::set_indefinite(bool value)
{
  _pimpl->indefinite = value;
}

//==============================================================================
bool ChargeBatteryFactory::indefinite() const
{
  return _pimpl->indefinite;
}

//==============================================================================
ConstRequestPtr ChargeBatteryFactory::make_request(const State& state) const
{
  const std::string id = "Charge" + generate_uuid();
  Task::ConstBookingPtr booking;
  if (_pimpl->requester.has_value() && _pimpl->time_now_cb)
  {
    booking =
      std::make_shared<const rmf_task::Task::Booking>(
      std::move(id),
      state.time().value(),
      nullptr,
      _pimpl->requester.value(),
      _pimpl->time_now_cb(),
      true);
  }
  else
  {
    booking =
      std::make_shared<const rmf_task::Task::Booking>(
      std::move(id),
      state.time().value(),
      nullptr,
      true);
  }

  const auto description = ChargeBattery::Description::make_indefinite();
  description->set_indefinite(_pimpl->indefinite);

  return std::make_shared<Request>(std::move(booking), std::move(description));
}

} // namespace requests
} // namespace rmf_task
