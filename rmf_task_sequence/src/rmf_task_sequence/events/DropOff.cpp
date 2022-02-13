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

#include "internal_PayloadTransfer.hpp"

#include <rmf_task_sequence/events/DropOff.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
class DropOff::Description::Implementation
{
public:
  PayloadTransfer transfer;
};

//==============================================================================
auto DropOff::Description::make(
  Location drop_off_location,
  std::string into_ingestor,
  Payload payload,
  rmf_traffic::Duration unloading_duration_estimate) -> DescriptionPtr
{
  auto output = std::shared_ptr<Description>(new Description);
  output->_pimpl = rmf_utils::make_unique_impl<Implementation>(
    Implementation{
      PayloadTransfer(
        std::move(drop_off_location),
        std::move(into_ingestor),
        std::move(payload),
        unloading_duration_estimate)
    });

  return output;
}

//==============================================================================
auto DropOff::Description::drop_off_location() const -> const Location&
{
  return _pimpl->transfer.go_to_place->destination();
}

//==============================================================================
auto DropOff::Description::drop_off_location(Location new_location)
-> Description&
{
  _pimpl->transfer.go_to_place->destination(std::move(new_location));
  return *this;
}

//==============================================================================
const std::string& DropOff::Description::into_ingestor() const
{
  return _pimpl->transfer.target;
}

//==============================================================================
auto DropOff::Description::into_ingestor(std::string new_ingestor)
-> Description&
{
  _pimpl->transfer.target = std::move(new_ingestor);
  return *this;
}

//==============================================================================
const Payload& DropOff::Description::payload() const
{
  return _pimpl->transfer.payload;
}

//==============================================================================
auto DropOff::Description::payload(Payload new_payload) -> Description&
{
  _pimpl->transfer.payload = std::move(new_payload);
  return *this;
}

//==============================================================================
rmf_traffic::Duration DropOff::Description::unloading_duration_estimate() const
{
  return _pimpl->transfer.wait_for->duration();
}

//==============================================================================
auto DropOff::Description::unloading_duration_estimate(
  const rmf_traffic::Duration new_duration) -> Description&
{
  _pimpl->transfer.wait_for->duration(new_duration);
  return *this;
}

//==============================================================================
Activity::ConstModelPtr DropOff::Description::make_model(
  State invariant_initial_state,
  const Parameters& parameters) const
{
  return _pimpl->transfer.make_model(
    std::move(invariant_initial_state), parameters);
}

//==============================================================================
Header DropOff::Description::generate_header(
  const State& initial_state,
  const Parameters& parameters) const
{
  return _pimpl->transfer
    .generate_header("Drop off", initial_state, parameters);
}

//==============================================================================
DropOff::Description::Description()
{
  // Do nothing
}

} // namespace events
} // namespace rmf_task_sequence
