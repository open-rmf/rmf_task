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

#include <rmf_task/requests/Loop.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
class Loop::Model : public Request::Model
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
    std::size_t start_waypoint,
    std::size_t finish_waypoint,
    std::size_t num_loops);

private:
  rmf_traffic::Time _earliest_start_time;
  agv::Parameters _parameters;
  std::size_t _start_waypoint;
  std::size_t _finish_waypoint;

  rmf_traffic::Duration _invariant_duration;
  double _invariant_battery_drain;
};

//==============================================================================
Loop::Model::Model(
  const rmf_traffic::Time earliest_start_time,
  const agv::Parameters& parameters,
  std::size_t start_waypoint,
  std::size_t finish_waypoint,
  std::size_t num_loops)
: _earliest_start_time(earliest_start_time),
  _parameters(parameters),
  _start_waypoint(start_waypoint),
  _finish_waypoint(finish_waypoint)
{
  // Calculate the invariant duration and battery drain for this task
  _invariant_duration = rmf_traffic::Duration{0};
  _invariant_battery_drain = 0.0;
  if (_start_waypoint != _finish_waypoint)
  {
    rmf_traffic::agv::Planner::Start loop_start{
      _earliest_start_time,
      _start_waypoint,
      0.0};
    rmf_traffic::agv::Planner::Goal loop_end_goal{_finish_waypoint};

    const auto forward_loop_plan = _parameters.planner()->plan(
      loop_start, loop_end_goal);

    auto itinerary_start_time = _earliest_start_time;
    double forward_battery_drain = 0.0;
    rmf_traffic::Duration forward_duration(0);
    for (const auto& itinerary : forward_loop_plan->get_itinerary())
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
      forward_battery_drain += dSOC_motion + dSOC_device;

      forward_duration += itinerary_duration;
      itinerary_start_time = finish_time;
    }
    _invariant_duration =
      (2 * num_loops - 1) * forward_duration;
    _invariant_battery_drain =
      (2 * num_loops - 1) * forward_battery_drain;
  }

}

//==============================================================================
std::optional<rmf_task::Estimate> Loop::Model::estimate_finish(
  const agv::State& initial_state,
  const agv::Constraints& task_planning_constraints,
  EstimateCache& estimate_cache) const
{
  rmf_traffic::Duration variant_duration(0);

  const rmf_traffic::Time start_time = initial_state.finish_time();
  double battery_soc = initial_state.battery_soc();
  double dSOC_motion = 0.0;
  double dSOC_device = 0.0;
  const bool drain_battery = task_planning_constraints.drain_battery();
  const auto& planner = *_parameters.planner();
  const auto& motion_sink = *_parameters.motion_sink();
  const auto& ambient_sink = *_parameters.ambient_sink();

  // Check if a plan has to be generated from finish location to start_waypoint
  if (initial_state.waypoint() != _start_waypoint)
  {
    auto endpoints = std::make_pair(initial_state.waypoint(),
        _start_waypoint);
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
      // Compute plan to start_waypoint along with battery drain
      rmf_traffic::agv::Planner::Goal loop_start_goal{endpoints.second};
      const auto plan_to_start = planner.plan(
        initial_state.location(), loop_start_goal);
      // We assume we can always compute a plan
      auto itinerary_start_time = start_time;
      double variant_battery_drain = 0.0;
      for (const auto& itinerary : plan_to_start->get_itinerary())
      {
        const auto& trajectory = itinerary.trajectory();
        const auto& finish_time = *trajectory.finish_time();
        const rmf_traffic::Duration itinerary_duration =
          finish_time - itinerary_start_time;

        if (drain_battery)
        {
          // Compute battery drain
          dSOC_motion =
            motion_sink.compute_change_in_charge(trajectory);
          dSOC_device =
            ambient_sink.compute_change_in_charge(
            rmf_traffic::time::to_seconds(itinerary_duration));
          battery_soc = battery_soc - dSOC_motion - dSOC_device;
          variant_battery_drain += dSOC_motion + dSOC_device;
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

  // Compute wait_until
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

  // Compute finish time
  const rmf_traffic::Time state_finish_time =
    wait_until + variant_duration  + _invariant_duration;

  // Subtract invariant battery drain and check if robot can return to its charger
  double retreat_battery_drain = 0.0;
  if (drain_battery)
  {
    battery_soc -= _invariant_battery_drain;
    if (battery_soc <= task_planning_constraints.threshold_soc())
      return std::nullopt;

    if (_finish_waypoint != initial_state.charging_waypoint())
    {
      const auto endpoints = std::make_pair(_finish_waypoint,
          initial_state.charging_waypoint());
      const auto& cache_result = estimate_cache.get(endpoints);
      if (cache_result)
      {
        retreat_battery_drain = cache_result->dsoc;
      }
      else
      {
        rmf_traffic::agv::Planner::Start retreat_start{
          state_finish_time,
          endpoints.first,
          0.0};

        rmf_traffic::agv::Planner::Goal charger_goal{
          endpoints.second};

        const auto result_to_charger = planner.plan(
          retreat_start, charger_goal);
        // We assume we can always compute a plan
        auto itinerary_start_time = state_finish_time;
        rmf_traffic::Duration retreat_duration(0);
        for (const auto& itinerary : result_to_charger->get_itinerary())
        {
          const auto& trajectory = itinerary.trajectory();
          const auto& finish_time = *trajectory.finish_time();
          const rmf_traffic::Duration itinerary_duration =
            finish_time - itinerary_start_time;

          dSOC_motion = motion_sink.compute_change_in_charge(
            trajectory);
          dSOC_device = ambient_sink.compute_change_in_charge(
            rmf_traffic::time::to_seconds(itinerary_duration));
          retreat_battery_drain += dSOC_motion + dSOC_device;

          itinerary_start_time = finish_time;
          retreat_duration += itinerary_duration;
        }
        estimate_cache.set(endpoints, retreat_duration,
          retreat_battery_drain);
      }

      if (battery_soc - retreat_battery_drain <=
        task_planning_constraints.threshold_soc())
        return std::nullopt;
    }
  }

  // Return Estimate
  rmf_traffic::agv::Planner::Start location{
    state_finish_time,
    _finish_waypoint,
    initial_state.location().orientation()};
  agv::State state{
    std::move(location),
    initial_state.charging_waypoint(),
    battery_soc};

  return Estimate(state, wait_until);
}

//==============================================================================
rmf_traffic::Duration Loop::Model::invariant_duration() const
{
  return _invariant_duration;
}

//==============================================================================
class Loop::Description::Implementation
{
public:

  Implementation()
  {}

  std::size_t start_waypoint;
  std::size_t finish_waypoint;
  std::size_t num_loops;
};

//==============================================================================
DescriptionPtr Loop::Description::make(
  std::size_t start_waypoint,
  std::size_t finish_waypoint,
  std::size_t num_loops)
{
  std::shared_ptr<Description> loop(new Description());
  loop->_pimpl->start_waypoint = start_waypoint;
  loop->_pimpl->finish_waypoint = finish_waypoint;
  loop->_pimpl->num_loops = num_loops;

  return loop;
}

//==============================================================================
Loop::Description::Description()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<Request::Model> Loop::Description::make_model(
  rmf_traffic::Time earliest_start_time,
  const agv::Parameters& parameters) const
{
  return std::make_shared<Loop::Model>(
    earliest_start_time,
    parameters,
    _pimpl->start_waypoint,
    _pimpl->finish_waypoint,
    _pimpl->num_loops);
}

//==============================================================================
std::size_t Loop::Description::start_waypoint() const
{
  return _pimpl->start_waypoint;
}

//==============================================================================
std::size_t Loop::Description::finish_waypoint() const
{
  return _pimpl->finish_waypoint;
}

//==============================================================================
std::size_t Loop::Description::num_loops() const
{
  return _pimpl->num_loops;
}

//==============================================================================
ConstRequestPtr Loop::make(
  std::size_t start_waypoint,
  std::size_t finish_waypoint,
  std::size_t num_loops,
  const std::string& id,
  rmf_traffic::Time earliest_start_time,
  ConstPriorityPtr priority,
  bool automatic)
{
  const auto description = Description::make(
    start_waypoint,
    finish_waypoint,
    num_loops);

  return std::make_shared<Request>(
    id, earliest_start_time, std::move(priority), description, automatic);

}

} // namespace requests
} // namespace rmf_task
