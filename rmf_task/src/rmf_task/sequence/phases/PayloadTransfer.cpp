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

namespace rmf_task {
namespace sequence {
namespace phases {

//==============================================================================
Phase::ConstModelPtr PayloadTransfer::Model::make(
  State invariant_initial_state,
  const Parameters& parameters,
  const Phase::ConstDescriptionPtr& go_to_place,
  rmf_traffic::Duration transfer_duration)
{
  auto go_to_place_model = go_to_place->make_model(
    std::move(invariant_initial_state),
    parameters);

  if (!go_to_place_model)
    return nullptr;

  const auto transfer_battery_drain =
    parameters.ambient_sink()->compute_change_in_charge(
      rmf_traffic::time::to_seconds(transfer_duration));

  return std::shared_ptr<Model>(
    new Model(
      std::move(go_to_place_model),
      transfer_duration,
      transfer_battery_drain));
}

//==============================================================================
std::optional<State> PayloadTransfer::Model::estimate_finish(
  State initial_state,
  const Constraints& constraints,
  const TravelEstimator& travel_estimator) const
{
  // TODO(MXG): Consider adding the changed Payload to the state
  auto finish_state =
    _go_to_place->estimate_finish(initial_state, constraints, travel_estimator);

  if (!finish_state.has_value())
    return std::nullopt;

  finish_state->time(finish_state->time().value() + _transfer_duration);

  if (constraints.drain_battery())
  {
    finish_state->battery_soc(
      finish_state->battery_soc().value() - _transfer_battery_drain);
  }

  const auto battery_threshold = constraints.threshold_soc();
  if (finish_state->battery_soc().value() <= battery_threshold)
    return std::nullopt;

  return finish_state;
}

//==============================================================================
rmf_traffic::Duration PayloadTransfer::Model::invariant_duration() const
{
  return _transfer_duration;
}

//==============================================================================
State PayloadTransfer::Model::invariant_finish_state() const
{
  return _go_to_place->invariant_finish_state();
}

//==============================================================================
PayloadTransfer::Model::Model(
  Phase::ConstModelPtr go_to_place,
  rmf_traffic::Duration transfer_duration,
  double transfer_battery_drain)
: _go_to_place(std::move(go_to_place)),
  _transfer_duration(transfer_duration),
  _transfer_battery_drain(transfer_battery_drain)
{
  // Do nothing
}

} // namespace phases
} // namespace sequence
} // namespace rmf_task
