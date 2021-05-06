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

#include <rmf_task/requests/Clean.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
class Clean::Model : public Request::Model
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
    const rmf_traffic::Trajectory& cleaning_path,
    std::size_t start_waypoint,
    std::size_t end_waypoint);

private:
  rmf_traffic::Time _earliest_start_time;
  agv::Parameters _parameters;
  std::size_t _start_waypoint;
  std::size_t _end_waypoint;

  rmf_traffic::Duration _invariant_duration;
  double _invariant_battery_drain;
};

//==============================================================================
Clean::Model::Model(
  const rmf_traffic::Time earliest_start_time,
  const agv::Parameters& parameters,
  const rmf_traffic::Trajectory& cleaning_path,
  std::size_t start_waypoint,
  std::size_t end_waypoint)
: _earliest_start_time(earliest_start_time),
  _parameters(parameters),
  _start_waypoint(start_waypoint),
  _end_waypoint(end_waypoint)
{
  // Calculate duration of invariant component of task
  const auto& cleaning_start_time = cleaning_path.begin()->time();
  const auto& cleaning_finish_time = *cleaning_path.finish_time();

  _invariant_duration =
    cleaning_finish_time - cleaning_start_time;

  // Compute battery drain over invariant path
  const double dSOC_motion =
    _parameters.motion_sink()->compute_change_in_charge(cleaning_path);
  const double dSOC_ambient =
    _parameters.ambient_sink()->compute_change_in_charge(
    rmf_traffic::time::to_seconds(_invariant_duration));
  const double dSOC_cleaning =
    _parameters.tool_sink()->compute_change_in_charge(
    rmf_traffic::time::to_seconds(_invariant_duration));
  _invariant_battery_drain = dSOC_motion + dSOC_ambient +
    dSOC_cleaning;
}

//==============================================================================
std::optional<rmf_task::Estimate> Clean::Model::estimate_finish(
  const agv::State& initial_state,
  const agv::Constraints& task_planning_constraints,
  EstimateCache& estimate_cache) const
{
  rmf_traffic::agv::Plan::Start final_plan_start{
    initial_state.finish_time(),
    _end_waypoint,
    initial_state.location().orientation()};
  agv::State state{
    std::move(final_plan_start),
    initial_state.charging_waypoint(),
    initial_state.battery_soc()};

  rmf_traffic::Duration variant_duration(0);
  rmf_traffic::Duration end_duration(0);

  const rmf_traffic::Time start_time = initial_state.finish_time();
  double battery_soc = initial_state.battery_soc();
  double dSOC_motion = 0.0;
  double dSOC_ambient = 0.0;
  const bool drain_battery = task_planning_constraints.drain_battery();
  const auto& planner = *_parameters.planner();
  const auto& motion_sink = *_parameters.motion_sink();
  const auto& ambient_sink = *_parameters.ambient_sink();

  if (initial_state.waypoint() != _start_waypoint)
  {
    const auto endpoints = std::make_pair(initial_state.waypoint(),
        _start_waypoint);
    const auto& cache_result = estimate_cache.get(endpoints);
    if (cache_result)
    {
      variant_duration = cache_result->duration;
      if (drain_battery)
        battery_soc = battery_soc - cache_result->dsoc;
    }
    else
    {
      rmf_traffic::agv::Planner::Goal goal{endpoints.second};

      const auto result_to_start = planner.plan(
        initial_state.location(), goal);
      // We assume we can always compute a plan
      auto itinerary_start_time = start_time;
      double variant_battery_drain = 0.0;
      for (const auto& itinerary : result_to_start->get_itinerary())
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
          dSOC_ambient =
            ambient_sink.compute_change_in_charge(
            rmf_traffic::time::to_seconds(itinerary_duration));
          battery_soc = battery_soc - dSOC_motion - dSOC_ambient;
          variant_battery_drain += dSOC_motion + dSOC_ambient;
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

  if (_start_waypoint != _end_waypoint)
  {
    // TODO(YV) Account for battery drain and duration when robot moves from
    // end of cleaning trajectory to its end_waypoint. We currently define the
    // end_waypoint near the start_waypoint in the nav graph for minimum error
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
    dSOC_ambient = ambient_sink.compute_change_in_charge(
      rmf_traffic::time::to_seconds(wait_duration));
    battery_soc = battery_soc - dSOC_ambient;

    if (battery_soc <= task_planning_constraints.threshold_soc())
    {
      return std::nullopt;
    }
  }

  // Factor in invariants
  state.finish_time(
    wait_until + variant_duration + _invariant_duration + end_duration);

  if (drain_battery)
  {
    battery_soc -= _invariant_battery_drain;
    if (battery_soc <= task_planning_constraints.threshold_soc())
      return std::nullopt;

    // Check if the robot has enough charge to head back to nearest charger
    double retreat_battery_drain = 0.0;
    if (_end_waypoint != state.charging_waypoint())
    {
      const auto endpoints = std::make_pair(_end_waypoint,
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

          dSOC_motion = motion_sink.compute_change_in_charge(
            trajectory);
          dSOC_ambient = ambient_sink.compute_change_in_charge(
            rmf_traffic::time::to_seconds(itinerary_duration));
          retreat_battery_drain += dSOC_motion + dSOC_ambient;

          itinerary_start_time = finish_time;
          retreat_duration += itinerary_duration;
        }
        estimate_cache.set(endpoints, retreat_duration, retreat_battery_drain);
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
rmf_traffic::Duration Clean::Model::invariant_duration() const
{
  return _invariant_duration;
}

//==============================================================================
class Clean::Description::Implementation
{
public:

  Implementation()
  {}

  std::size_t start_waypoint;
  std::size_t end_waypoint;
  rmf_traffic::Trajectory cleaning_path;
};

//==============================================================================
rmf_task::DescriptionPtr Clean::Description::make(
  std::size_t start_waypoint,
  std::size_t end_waypoint,
  const rmf_traffic::Trajectory& cleaning_path)
{
  std::shared_ptr<Clean::Description> clean(new Clean::Description());
  clean->_pimpl->start_waypoint = start_waypoint;
  clean->_pimpl->end_waypoint = end_waypoint;
  clean->_pimpl->cleaning_path = cleaning_path;

  return clean;
}

//==============================================================================
Clean::Description::Description()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<Request::Model> Clean::Description::make_model(
  rmf_traffic::Time earliest_start_time,
  const agv::Parameters& parameters) const
{
  if (parameters.tool_sink() == nullptr)
  {
    // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
    throw std::invalid_argument(
      "Required parameter tool_sink is undefined in the supplied parameters");
    // *INDENT-ON*
  }

  return std::make_shared<Clean::Model>(
    earliest_start_time,
    parameters,
    _pimpl->cleaning_path,
    _pimpl->start_waypoint,
    _pimpl->end_waypoint);
}

//==============================================================================
std::size_t Clean::Description::start_waypoint() const
{
  return _pimpl->start_waypoint;
}

//==============================================================================
std::size_t Clean::Description::end_waypoint() const
{
  return _pimpl->end_waypoint;
}

//==============================================================================
ConstRequestPtr Clean::make(
  std::size_t start_waypoint,
  std::size_t end_waypoint,
  const rmf_traffic::Trajectory& cleaning_path,
  const std::string& id,
  rmf_traffic::Time earliest_start_time,
  ConstPriorityPtr priority)
{
  const auto description = Clean::Description::make(
    start_waypoint,
    end_waypoint,
    cleaning_path);

  return std::make_shared<Request>(
    id, earliest_start_time, priority, description);
}

} // namespace requests
} // namespace rmf_task
