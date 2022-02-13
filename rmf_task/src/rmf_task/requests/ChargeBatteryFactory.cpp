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

namespace rmf_task {
namespace requests {

//==============================================================================
class ChargeBatteryFactory::Implementation
{

};

//==============================================================================
ChargeBatteryFactory::ChargeBatteryFactory()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
  // Do nothing
}

//==============================================================================
ConstRequestPtr ChargeBatteryFactory::make_request(
  const State& state) const
{
  return ChargeBattery::make(state.time().value());
}

} // namespace requests
} // namespace rmf_task
