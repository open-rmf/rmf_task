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

#include <rmf_task_sequence/events/GoToPlace.hpp>

#include "utils.hpp"

namespace rmf_task_sequence {
namespace events {

namespace {
//==============================================================================
std::optional<rmf_traffic::Duration> estimate_duration(
  const std::shared_ptr<const rmf_traffic::agv::Planner>& planner,
  const State& initial_state,
  const GoToPlace::Goal& goal)
{
  const auto result =
    planner->setup(initial_state.project_plan_start().value(), goal);

  // TODO(MXG): Perhaps print errors/warnings about these failure conditions
  if (result.disconnected())
    return std::nullopt;

  if (!result.ideal_cost().has_value())
    return std::nullopt;

  return rmf_traffic::time::from_seconds(*result.ideal_cost());
}
} // anonymous namespace

//==============================================================================
class GoToPlace::Model : public Activity::Model
{
public:

  static Activity::ConstModelPtr make(
    State invariant_initial_state,
    const Parameters& parameters,
    Goal goal);

  // Documentation inherited
  std::optional<Estimate> estimate_finish(
    State initial_state,
    rmf_traffic::Time earliest_arrival_time,
    const Constraints& constraints,
    const TravelEstimator& estimator) const final;

  // Documentation inherited
  rmf_traffic::Duration invariant_duration() const final;

  // Documentation inherited
  State invariant_finish_state() const final;

private:

  Model(
    State invariant_finish_state,
    rmf_traffic::Duration invariant_duration,
    Goal goal);

  State _invariant_finish_state;
  rmf_traffic::Duration _invariant_duration;
  Goal _goal;
};

//==============================================================================
Activity::ConstModelPtr GoToPlace::Model::make(
  State invariant_initial_state,
  const Parameters& parameters,
  Goal goal)
{
  auto invariant_finish_state = invariant_initial_state;
  invariant_finish_state.waypoint(goal.waypoint());

  if (goal.orientation())
    invariant_finish_state.orientation(*goal.orientation());
  else
    invariant_finish_state.erase<State::CurrentOrientation>();

  auto invariant_duration = rmf_traffic::Duration(0);
  if (invariant_initial_state.waypoint().has_value())
  {
    const auto invariant_duration_opt = estimate_duration(
      parameters.planner(),
      invariant_initial_state,
      goal);

    if (!invariant_duration_opt.has_value())
      return nullptr;

    invariant_duration = *invariant_duration_opt;
  }

  return std::shared_ptr<Model>(
    new Model(
      std::move(invariant_finish_state),
      invariant_duration,
      std::move(goal)));
}

//==============================================================================
std::optional<Estimate> GoToPlace::Model::estimate_finish(
  State initial_state,
  rmf_traffic::Time earliest_arrival_time,
  const Constraints& constraints,
  const TravelEstimator& travel_estimator) const
{
  auto finish = initial_state;
  finish.waypoint(_goal.waypoint());

  const auto travel = travel_estimator.estimate(
    initial_state.extract_plan_start().value(),
    _goal);

  if (!travel.has_value())
    return std::nullopt;

  const auto arrival_time =
    std::max(
    initial_state.time().value() + travel->duration(),
    earliest_arrival_time);

  const auto wait_until_time = arrival_time - travel->duration();
  finish.time(wait_until_time + travel->duration());
  auto battery_soc = finish.battery_soc().value();

  if (constraints.drain_battery())
    battery_soc = battery_soc - travel->change_in_charge();

  finish.battery_soc(battery_soc);

  const auto battery_threshold = constraints.threshold_soc();
  if (battery_soc <= battery_threshold)
    return std::nullopt;

  return Estimate(finish, wait_until_time);
}

//==============================================================================
rmf_traffic::Duration GoToPlace::Model::invariant_duration() const
{
  return _invariant_duration;
}

//==============================================================================
State GoToPlace::Model::invariant_finish_state() const
{
  return _invariant_finish_state;
}

//==============================================================================
GoToPlace::Model::Model(
  State invariant_finish_state,
  rmf_traffic::Duration invariant_duration,
  Goal goal)
: _invariant_finish_state(std::move(invariant_finish_state)),
  _invariant_duration(invariant_duration),
  _goal(std::move(goal))
{
  // Do nothing
}

//==============================================================================
class GoToPlace::Description::Implementation
{
public:

  rmf_traffic::agv::Plan::Goal destination;
  std::vector<rmf_traffic::agv::Plan::Goal> expected_next_destinations;
};

//==============================================================================
auto GoToPlace::Description::make(Goal goal) -> DescriptionPtr
{
  auto desc = std::shared_ptr<Description>(new Description);
  desc->_pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{std::move(goal), {}});

  return desc;
}

//==============================================================================
Activity::ConstModelPtr GoToPlace::Description::make_model(
  State invariant_initial_state,
  const Parameters& parameters) const
{
  return Model::make(
    std::move(invariant_initial_state),
    parameters,
    _pimpl->destination);
}

//==============================================================================
Header GoToPlace::Description::generate_header(
  const State& initial_state,
  const Parameters& parameters) const
{
  const std::string& fail_header = "[GoToPlace::Description::generate_header]";

  const auto start_wp_opt = initial_state.waypoint();
  if (!start_wp_opt)
    utils::fail(fail_header, "Initial state is missing a waypoint");

  const auto start_wp = *start_wp_opt;

  const auto& graph = parameters.planner()->get_configuration().graph();
  if (graph.num_waypoints() <= start_wp)
  {
    utils::fail(fail_header, "Initial waypoint [" + std::to_string(start_wp)
      + "] is outside the graph [" + std::to_string(graph.num_waypoints())
      + "]");
  }

  const auto start_name = rmf_task::standard_waypoint_name(graph, start_wp);

  if (graph.num_waypoints() <= _pimpl->destination.waypoint())
  {
    utils::fail(fail_header, "Destination waypoint ["
      + std::to_string(_pimpl->destination.waypoint())
      + "] is outside the graph [" + std::to_string(graph.num_waypoints())
      + "]");
  }

  const auto goal_name_ = destination_name(parameters);

  const auto estimate = estimate_duration(
    parameters.planner(), initial_state, _pimpl->destination);

  if (!estimate.has_value())
  {
    utils::fail(fail_header, "Cannot find a path from ["
      + start_name + "] to [" + goal_name_ + "]");
  }

  return Header(
    "Go to " + goal_name_,
    "Moving the robot from " + start_name + " to " + goal_name_,
    *estimate);
}

//==============================================================================
auto GoToPlace::Description::destination() const -> const Goal&
{
  return _pimpl->destination;
}

//==============================================================================
auto GoToPlace::Description::destination(Goal new_goal) -> Description&
{
  _pimpl->destination = std::move(new_goal);
  return *this;
}

//==============================================================================
std::string GoToPlace::Description::destination_name(
  const Parameters& parameters) const
{
  return rmf_task::standard_waypoint_name(
    parameters.planner()->get_configuration().graph(),
    _pimpl->destination.waypoint());
}

//==============================================================================
auto GoToPlace::Description::expected_next_destinations() const
-> const std::vector<Goal>&
{
  return _pimpl->expected_next_destinations;
}

//==============================================================================
auto GoToPlace::Description::expected_next_destinations(std::vector<Goal> value)
-> Description&
{
  _pimpl->expected_next_destinations = std::move(value);
  return *this;
}

//==============================================================================
GoToPlace::Description::Description()
{
  // Do nothing
}

} // namespace phases
} // namespace rmf_task_sequence
