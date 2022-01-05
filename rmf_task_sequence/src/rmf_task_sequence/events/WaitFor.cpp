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

#include <rmf_task_sequence/events/WaitFor.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
class WaitFor::Model : public Activity::Model
{
public:

  Model(
    rmf_task::State invariant_initial_state,
    rmf_traffic::Duration duration,
    const rmf_task::Parameters& parameters);

  std::optional<Estimate> estimate_finish(
    rmf_task::State initial_state,
    rmf_traffic::Time earliest_arrival_time,
    const Constraints& constraints,
    const TravelEstimator& travel_estimator) const final;

  rmf_traffic::Duration invariant_duration() const final;

  State invariant_finish_state() const final;

private:
  rmf_task::State _invariant_finish_state;
  double _invariant_battery_drain;
  rmf_traffic::Duration _duration;
};

//==============================================================================
class WaitFor::Description::Implementation
{
public:

  rmf_traffic::Duration duration;

};

//==============================================================================
auto WaitFor::Description::make(rmf_traffic::Duration wait_duration)
-> DescriptionPtr
{
  return std::make_shared<Description>(wait_duration);
}

//==============================================================================
WaitFor::Description::Description(rmf_traffic::Duration duration_)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{duration_}))
{
  // Do nothing
}

//==============================================================================
rmf_traffic::Duration WaitFor::Description::duration() const
{
  return _pimpl->duration;
}

//==============================================================================
auto WaitFor::Description::duration(rmf_traffic::Duration new_duration)
-> Description&
{
  _pimpl->duration = new_duration;
  return *this;
}

//==============================================================================
Activity::ConstModelPtr WaitFor::Description::make_model(
  State invariant_initial_state,
  const Parameters& parameters) const
{
  return std::make_shared<Model>(
    invariant_initial_state, _pimpl->duration, parameters);
}

//==============================================================================
Header WaitFor::Description::generate_header(
  const State&, const Parameters&) const
{
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(
    _pimpl->duration);

  return Header(
    "Waiting",
    "Waiting for [" + std::to_string(seconds.count()) + "] seconds to elapse",
    _pimpl->duration);
}

//==============================================================================
WaitFor::Model::Model(
  State invariant_initial_state,
  rmf_traffic::Duration duration,
  const Parameters& parameters)
: _invariant_finish_state(std::move(invariant_initial_state)),
  _duration(duration)
{
  if (parameters.ambient_sink())
  {
    _invariant_battery_drain =
      parameters.ambient_sink()->compute_change_in_charge(
      rmf_traffic::time::to_seconds(_duration));
  }
  else
  {
    _invariant_battery_drain = 0.0;
  }
}

//==============================================================================
std::optional<Estimate> WaitFor::Model::estimate_finish(
  State state,
  rmf_traffic::Time earliest_arrival_time,
  const Constraints& constraints,
  const TravelEstimator&) const
{
  state.time(state.time().value() + _duration);

  if (constraints.drain_battery())
    state.battery_soc(state.battery_soc().value() - _invariant_battery_drain);

  if (state.battery_soc().value() <= _invariant_battery_drain)
    return std::nullopt;

  return Estimate(state, earliest_arrival_time);
}

//==============================================================================
rmf_traffic::Duration WaitFor::Model::invariant_duration() const
{
  return _duration;
}

//==============================================================================
State WaitFor::Model::invariant_finish_state() const
{
  return _invariant_finish_state;
}

} // namespace phases
} // namespace rmf_task_sequence
