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

#include <map>

#include <rmf_task/requests/Delivery.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
class Delivery::Model : public Task::Model
{
public:

  std::optional<Estimate> estimate_finish(
    const State& initial_state,
    const Constraints& task_planning_constraints,
    const TravelEstimator& travel_estimator) const final;

  rmf_traffic::Duration invariant_duration() const final;

  Model(
    const rmf_traffic::Time earliest_start_time,
    const Parameters& parameters,
    std::size_t pickup_waypoint,
    rmf_traffic::Duration pickup_wait,
    std::size_t dropoff_waypoint,
    rmf_traffic::Duration dropoff_wait);

private:
  rmf_traffic::Time _earliest_start_time;
  Parameters _parameters;
  std::size_t _pickup_waypoint;
  std::size_t _dropoff_waypoint;

  rmf_traffic::Duration _invariant_duration;
  double _invariant_battery_drain;
};

//==============================================================================
Delivery::Model::Model(
  const rmf_traffic::Time earliest_start_time,
  const Parameters& parameters,
  std::size_t pickup_waypoint,
  rmf_traffic::Duration pickup_wait,
  std::size_t dropoff_waypoint,
  rmf_traffic::Duration dropoff_wait)
: _earliest_start_time(earliest_start_time),
  _parameters(parameters),
  _pickup_waypoint(pickup_waypoint),
  _dropoff_waypoint(dropoff_waypoint)
{
  // Calculate duration of invariant component of task
  _invariant_duration = pickup_wait + dropoff_wait;
  _invariant_battery_drain =
    _parameters.ambient_sink()->compute_change_in_charge(
    rmf_traffic::time::to_seconds(pickup_wait + dropoff_wait));

  if (_pickup_waypoint != _dropoff_waypoint)
  {
    rmf_traffic::agv::Planner::Start start{
      _earliest_start_time,
      _pickup_waypoint,
      0.0};

    rmf_traffic::agv::Planner::Goal goal{_dropoff_waypoint};
    const auto result_to_dropoff = _parameters.planner()->plan(start, goal);

    auto itinerary_start_time = _earliest_start_time;
    for (const auto& itinerary : result_to_dropoff->get_itinerary())
    {
      const auto& trajectory = itinerary.trajectory();
      const auto& finish_time = *trajectory.finish_time();
      const auto itinerary_duration = finish_time - itinerary_start_time;

      // Compute the invariant battery drain
      const double dSOC_motion =
        _parameters.motion_sink()->compute_change_in_charge(trajectory);
      const double dSOC_device =
        _parameters.ambient_sink()->compute_change_in_charge(
        rmf_traffic::time::to_seconds(itinerary_duration));
      _invariant_battery_drain += dSOC_motion + dSOC_device;

      _invariant_duration += itinerary_duration;
      itinerary_start_time = finish_time;
    }
  }

}

//==============================================================================
std::optional<rmf_task::Estimate> Delivery::Model::estimate_finish(
  const State& initial_state,
  const Constraints& task_planning_constraints,
  const TravelEstimator& travel_estimator) const
{
  rmf_traffic::agv::Plan::Start final_plan_start{
    initial_state.time().value(),
    _dropoff_waypoint,
    initial_state.orientation().value()};

  auto state = State().load_basic(
    std::move(final_plan_start),
    initial_state.dedicated_charging_waypoint().value(),
    initial_state.battery_soc().value());

  rmf_traffic::Duration variant_duration(0);

  double battery_soc = initial_state.battery_soc().value();
  const bool drain_battery = task_planning_constraints.drain_battery();
  const auto& ambient_sink = *_parameters.ambient_sink();
  const double battery_threshold = task_planning_constraints.threshold_soc();

  // Factor in battery drain while moving to start waypoint of task
  if (initial_state.waypoint() != _pickup_waypoint)
  {
    const auto travel = travel_estimator.estimate(
      initial_state.extract_plan_start().value(),
      _pickup_waypoint);

    if (!travel)
      return std::nullopt;

    variant_duration = travel->duration();
    if (drain_battery)
      battery_soc = battery_soc - travel->change_in_charge();

    if (battery_soc <= battery_threshold)
      return std::nullopt;
  }

  const rmf_traffic::Time ideal_start = _earliest_start_time - variant_duration;
  const rmf_traffic::Time wait_until =
    initial_state.time().value() > ideal_start ?
    initial_state.time().value() : ideal_start;

  // Factor in battery drain while waiting to move to start waypoint. If a robot
  // is initially at a charging waypoint, it is assumed to be continually charging
  if (drain_battery && wait_until > initial_state.time().value() &&
    initial_state.waypoint() != initial_state.dedicated_charging_waypoint())
  {
    const rmf_traffic::Duration wait_duration(
      wait_until - initial_state.time().value());

    const double dSOC_device = ambient_sink.compute_change_in_charge(
      rmf_traffic::time::to_seconds(wait_duration));

    battery_soc = battery_soc - dSOC_device;

    if (battery_soc <= battery_threshold)
      return std::nullopt;
  }

  // Factor in invariants
  state.time(wait_until + variant_duration + _invariant_duration);

  if (drain_battery)
  {
    // Calculate how much battery is drained while waiting for the pickup and
    // waiting for the dropoff
    battery_soc -= _invariant_battery_drain;
    if (battery_soc <= battery_threshold)
      return std::nullopt;

    // Check if the robot has enough charge to head back to nearest charger
    if (_dropoff_waypoint != state.dedicated_charging_waypoint().value())
    {
      const auto travel = travel_estimator.estimate(
        state.extract_plan_start().value(),
        state.dedicated_charging_waypoint().value());

      if (!travel.has_value())
        return std::nullopt;

      if (battery_soc - travel->change_in_charge() <= battery_threshold)
        return std::nullopt;
    }

    state.battery_soc(battery_soc);
  }

  return Estimate(state, wait_until);
}

//==============================================================================
rmf_traffic::Duration Delivery::Model::invariant_duration() const
{
  return _invariant_duration;
}

//==============================================================================
class Delivery::Description::Implementation
{
public:

  std::size_t pickup_waypoint;
  rmf_traffic::Duration pickup_wait;
  std::size_t dropoff_waypoint;
  rmf_traffic::Duration dropoff_wait;
  rmf_task::Payload payload;
  std::string pickup_from_dispenser;
  std::string dropoff_to_ingestor;
};

//==============================================================================
Task::ConstDescriptionPtr Delivery::Description::make(
  std::size_t pickup_waypoint,
  rmf_traffic::Duration pickup_wait,
  std::size_t dropoff_waypoint,
  rmf_traffic::Duration dropoff_wait,
  Payload payload,
  std::string pickup_from_dispenser,
  std::string dropoff_to_ingestor)
{
  std::shared_ptr<Description> delivery(new Description());
  delivery->_pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{
      pickup_waypoint,
      pickup_wait,
      dropoff_waypoint,
      dropoff_wait,
      std::move(payload),
      std::move(pickup_from_dispenser),
      std::move(dropoff_to_ingestor)
    });

  return delivery;
}

//==============================================================================
Delivery::Description::Description()
{
  // Do nothing
}

//==============================================================================
Task::ConstModelPtr Delivery::Description::make_model(
  rmf_traffic::Time earliest_start_time,
  const Parameters& parameters) const
{
  return std::make_shared<Delivery::Model>(
    earliest_start_time,
    parameters,
    _pimpl->pickup_waypoint,
    _pimpl->pickup_wait,
    _pimpl->dropoff_waypoint,
    _pimpl->dropoff_wait);
}

//==============================================================================
auto Delivery::Description::generate_info(
  const State&,
  const Parameters& parameters) const -> Info
{
  const auto& graph = parameters.planner()->get_configuration().graph();
  return Info{
    "Delivery from " + standard_waypoint_name(graph, _pimpl->pickup_waypoint)
    + " to " + standard_waypoint_name(graph, _pimpl->dropoff_waypoint),
    "" // TODO(MXG): Add details about the payload
  };
}

//==============================================================================
std::size_t Delivery::Description::pickup_waypoint() const
{
  return _pimpl->pickup_waypoint;
}

//==============================================================================
std::string Delivery::Description::pickup_from_dispenser() const
{
  return _pimpl->pickup_from_dispenser;
}

//==============================================================================
std::string Delivery::Description::dropoff_to_ingestor() const
{
  return _pimpl->dropoff_to_ingestor;
}

//==============================================================================
rmf_traffic::Duration Delivery::Description::pickup_wait() const
{
  return _pimpl->pickup_wait;
}

//==============================================================================
rmf_traffic::Duration Delivery::Description::dropoff_wait() const
{
  return _pimpl->dropoff_wait;
}

//==============================================================================
std::size_t Delivery::Description::dropoff_waypoint() const
{
  return _pimpl->dropoff_waypoint;
}

//==============================================================================
const Payload& Delivery::Description::payload() const
{
  return _pimpl->payload;
}

//==============================================================================
ConstRequestPtr Delivery::make(
  std::size_t pickup_waypoint,
  rmf_traffic::Duration pickup_wait,
  std::size_t dropoff_waypoint,
  rmf_traffic::Duration dropoff_wait,
  Payload payload,
  const std::string& id,
  rmf_traffic::Time earliest_start_time,
  ConstPriorityPtr priority,
  bool automatic,
  std::string pickup_from_dispenser,
  std::string dropoff_to_ingestor)
{
  const auto description = Description::make(
    pickup_waypoint,
    pickup_wait,
    dropoff_waypoint,
    dropoff_wait,
    std::move(payload),
    std::move(pickup_from_dispenser),
    std::move(dropoff_to_ingestor));

  return std::make_shared<Request>(
    id, earliest_start_time, std::move(priority), description, automatic);
}

} // namespace requests
} // namespace rmf_task
