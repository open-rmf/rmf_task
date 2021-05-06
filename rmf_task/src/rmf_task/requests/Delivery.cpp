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
class Delivery::Model : public Request::Model
{
public:

  std::optional<Estimate> estimate_finish(
    const agv::State& initial_state,
    const agv::Constraints& task_planning_constraints,
    EstimateCache& estimate_cache) const final;

  rmf_traffic::Duration invariant_duration() const final;

  Model(
    const rmf_traffic::Time earliest_start_time,
    const agv::Parameters& parameters,
    std::size_t pickup_waypoint,
    std::size_t dropoff_waypoint);

private:
  rmf_traffic::Time _earliest_start_time;
  agv::Parameters _parameters;
  std::size_t _pickup_waypoint;
  std::size_t _dropoff_waypoint;

  rmf_traffic::Duration _invariant_duration;
  double _invariant_battery_drain;
};

//==============================================================================
Delivery::Model::Model(
  const rmf_traffic::Time earliest_start_time,
  const agv::Parameters& parameters,
  std::size_t pickup_waypoint,
  std::size_t dropoff_waypoint)
: _earliest_start_time(earliest_start_time),
  _parameters(parameters),
  _pickup_waypoint(pickup_waypoint),
  _dropoff_waypoint(dropoff_waypoint)
{
  // Calculate duration of invariant component of task
  _invariant_duration = rmf_traffic::Duration{0};
  _invariant_battery_drain = 0.0;

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
  const agv::State& initial_state,
  const agv::Constraints& task_planning_constraints,
  EstimateCache& estimate_cache) const
{
  rmf_traffic::agv::Plan::Start final_plan_start{
    initial_state.finish_time(),
    _dropoff_waypoint,
    initial_state.location().orientation()};
  agv::State state{
    std::move(final_plan_start),
    initial_state.charging_waypoint(),
    initial_state.battery_soc()};

  rmf_traffic::Duration variant_duration(0);

  const rmf_traffic::Time start_time = initial_state.finish_time();
  double battery_soc = initial_state.battery_soc();
  double dSOC_motion = 0.0;
  double dSOC_device = 0.0;
  const bool drain_battery = task_planning_constraints.drain_battery();
  const auto& planner = *_parameters.planner();
  const auto& motion_sink = *_parameters.motion_sink();
  const auto& ambient_sink = *_parameters.ambient_sink();

  // Factor in battery drain while moving to start waypoint of task
  if (initial_state.waypoint() != _pickup_waypoint)
  {
    const auto endpoints = std::make_pair(initial_state.waypoint(),
        _pickup_waypoint);
    const auto& cache_result = estimate_cache.get(endpoints);
    // Use previously memoized values if possible
    if (cache_result)
    {
      variant_duration = cache_result->duration;
      if (drain_battery)
        battery_soc = battery_soc - cache_result->dsoc;
    }
    else
    {
      // Compute plan to pickup waypoint along with battery drain
      rmf_traffic::agv::Planner::Goal goal{endpoints.second};
      const auto result_to_pickup = planner.plan(
        initial_state.location(), goal);
      // We assume we can always compute a plan
      auto itinerary_start_time = start_time;
      double variant_battery_drain = 0.0;
      for (const auto& itinerary : result_to_pickup->get_itinerary())
      {
        const auto& trajectory = itinerary.trajectory();
        const auto& finish_time = *trajectory.finish_time();
        const rmf_traffic::Duration itinerary_duration =
          finish_time - itinerary_start_time;

        if (drain_battery)
        {
          // Compute battery drain
          dSOC_motion = motion_sink.compute_change_in_charge(
            trajectory);
          dSOC_device =
            ambient_sink.compute_change_in_charge(
            rmf_traffic::time::to_seconds(itinerary_duration));
          battery_soc = battery_soc - dSOC_motion - dSOC_device;
          variant_battery_drain += dSOC_device + dSOC_motion;
        }
        itinerary_start_time = finish_time;
        variant_duration += itinerary_duration;
      }
      estimate_cache.set(endpoints, variant_duration,
        variant_battery_drain);
    }

    if (battery_soc <= task_planning_constraints.threshold_soc())
      return std::nullopt;
  }

  const rmf_traffic::Time ideal_start = _earliest_start_time - variant_duration;
  const rmf_traffic::Time wait_until =
    initial_state.finish_time() > ideal_start ?
    initial_state.finish_time() : ideal_start;

  // Factor in battery drain while waiting to move to start waypoint. If a robot
  // is initially at a charging waypoint, it is assumed to be continually charging
  if (drain_battery && wait_until > initial_state.finish_time() &&
    initial_state.waypoint() != initial_state.charging_waypoint())
  {
    rmf_traffic::Duration wait_duration(
      wait_until - initial_state.finish_time());
    dSOC_device = ambient_sink.compute_change_in_charge(
      rmf_traffic::time::to_seconds(wait_duration));
    battery_soc = battery_soc - dSOC_device;

    if (battery_soc <= task_planning_constraints.threshold_soc())
    {
      return std::nullopt;
    }
  }

  // Factor in invariants
  state.finish_time(
    wait_until + variant_duration + _invariant_duration);

  if (drain_battery)
  {
    battery_soc -= _invariant_battery_drain;
    if (battery_soc <= task_planning_constraints.threshold_soc())
      return std::nullopt;

    // Check if the robot has enough charge to head back to nearest charger
    double retreat_battery_drain = 0.0;
    if (_dropoff_waypoint != state.charging_waypoint())
    {
      const auto endpoints = std::make_pair(_dropoff_waypoint,
          state.charging_waypoint());
      const auto& cache_result = estimate_cache.get(endpoints);
      if (cache_result)
      {
        retreat_battery_drain = cache_result->dsoc;
      }
      else
      {
        rmf_traffic::agv::Planner::Start start{
          state.finish_time(),
          endpoints.first,
          0.0};

        rmf_traffic::agv::Planner::Goal goal{endpoints.second};

        const auto result_to_charger = planner.plan(start, goal);
        // We assume we can always compute a plan
        auto itinerary_start_time = state.finish_time();
        rmf_traffic::Duration retreat_duration(0);
        for (const auto& itinerary : result_to_charger->get_itinerary())
        {
          const auto& trajectory = itinerary.trajectory();
          const auto& finish_time = *trajectory.finish_time();
          const rmf_traffic::Duration itinerary_duration =
            finish_time - itinerary_start_time;

          dSOC_motion =
            motion_sink.compute_change_in_charge(trajectory);
          dSOC_device = ambient_sink.compute_change_in_charge(
            rmf_traffic::time::to_seconds(itinerary_duration));
          retreat_battery_drain += dSOC_motion + dSOC_device;

          itinerary_start_time = finish_time;
          retreat_duration += itinerary_duration;
        }
        estimate_cache.set(endpoints, retreat_duration,
          retreat_battery_drain);
      }
    }

    if (battery_soc - retreat_battery_drain <=
      task_planning_constraints.threshold_soc())
      return std::nullopt;

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

  Implementation()
  {}

  std::size_t pickup_waypoint;
  std::string pickup_dispenser;
  std::size_t dropoff_waypoint;
  std::string dropoff_ingestor;
  std::vector<DispenserRequestItem> items;
};

//==============================================================================
rmf_task::DescriptionPtr Delivery::Description::make(
  std::size_t pickup_waypoint,
  std::string pickup_dispenser,
  std::size_t dropoff_waypoint,
  std::string dropoff_ingestor,
  std::vector<DispenserRequestItem> items)
{
  std::shared_ptr<Description> delivery(new Description());
  delivery->_pimpl->pickup_waypoint = pickup_waypoint;
  delivery->_pimpl->pickup_dispenser = std::move(pickup_dispenser);
  delivery->_pimpl->dropoff_waypoint = dropoff_waypoint;
  delivery->_pimpl->dropoff_ingestor = std::move(dropoff_ingestor);
  delivery->_pimpl->items = std::move(items);

  return delivery;
}

//==============================================================================
Delivery::Description::Description()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<Request::Model> Delivery::Description::make_model(
  rmf_traffic::Time earliest_start_time,
  const agv::Parameters& parameters) const
{
  return std::make_shared<Delivery::Model>(
    earliest_start_time,
    parameters,
    _pimpl->pickup_waypoint,
    _pimpl->dropoff_waypoint);
}

//==============================================================================
std::size_t Delivery::Description::pickup_waypoint() const
{
  return _pimpl->pickup_waypoint;
}

//==============================================================================
const std::string& Delivery::Description::pickup_dispenser() const
{
  return _pimpl->pickup_dispenser;
}

//==============================================================================
const std::string& Delivery::Description::dropoff_ingestor() const
{
  return _pimpl->dropoff_ingestor;
}

//==============================================================================
std::size_t Delivery::Description::dropoff_waypoint() const
{
  return _pimpl->dropoff_waypoint;
}

//==============================================================================
const std::vector<Delivery::Description::DispenserRequestItem>&
Delivery::Description::items() const
{
  return _pimpl->items;
}

//==============================================================================
ConstRequestPtr Delivery::make(
  std::size_t pickup_waypoint,
  std::string pickup_dispenser,
  std::size_t dropoff_waypoint,
  std::string dropoff_ingestor,
  std::vector<DispenserRequestItem> items,
  const std::string& id,
  rmf_traffic::Time earliest_start_time,
  ConstPriorityPtr priority)
{
  const auto description = Description::make(
    pickup_waypoint,
    pickup_dispenser,
    dropoff_waypoint,
    dropoff_ingestor,
    items);

  return std::make_shared<Request>(
    id, earliest_start_time, std::move(priority), description);
}

} // namespace requests
} // namespace rmf_task
