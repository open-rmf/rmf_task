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

namespace rmf_task_sequence {
namespace events {

//==============================================================================
PayloadTransfer::PayloadTransfer(
  Location location_,
  std::string target_,
  Payload payload_,
  rmf_traffic::Duration loading_duration_estimate)
: target(std::move(target_)),
  payload(std::move(payload_)),
  go_to_place(events::GoToPlace::Description::make(std::move(location_))),
  wait_for(events::WaitFor::Description::make(loading_duration_estimate))
{
  descriptions = {go_to_place, wait_for};
}

//==============================================================================
Activity::ConstModelPtr PayloadTransfer::make_model(
  State invariant_initial_state,
  const Parameters& parameters) const
{
  return Activity::SequenceModel::make(
    descriptions,
    std::move(invariant_initial_state),
    parameters);
}

//==============================================================================
Header PayloadTransfer::generate_header(
  const std::string& type,
  const State& initial_state,
  const Parameters& parameters) const
{
  const auto model = make_model(initial_state, parameters);

  return Header(
    type,
    type + " " + payload.brief("into") + " at "
    + go_to_place->destination_name(parameters),
    model->invariant_duration());
}

} // namespace events
} // namespace rmf_task_sequence
