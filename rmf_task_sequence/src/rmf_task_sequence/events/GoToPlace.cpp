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
    const std::vector<Goal>& goal);

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
  const std::vector<Goal>& goals)
{
  if (goals.empty())
  {
    return nullptr;
  }

  auto invariant_finish_state = invariant_initial_state;

  auto selected_goal = goals[0];
  std::optional<rmf_traffic::Duration> shortest_travel_time = std::nullopt;
  if (invariant_initial_state.waypoint().has_value())
  {
    for (const auto& goal: goals)
    {
      const auto invariant_duration_opt = estimate_duration(
        parameters.planner(),
        invariant_initial_state,
        goal);

      if (!shortest_travel_time.has_value())
      {
        shortest_travel_time = invariant_duration_opt;
        selected_goal = goal;
      }
      else if (shortest_travel_time.value() > invariant_duration_opt.value())
      {
        shortest_travel_time = invariant_duration_opt;
        selected_goal = goal;
      }
    }

    if (!shortest_travel_time.has_value())
      return nullptr;
  }

  invariant_finish_state.waypoint(selected_goal.waypoint());

  if (selected_goal.orientation())
    invariant_finish_state.orientation(*selected_goal.orientation());
  else
    invariant_finish_state.erase<State::CurrentOrientation>();

  return std::shared_ptr<Model>(
    new Model(
      std::move(invariant_finish_state),
      shortest_travel_time.value_or(rmf_traffic::Duration(0)),
      std::move(selected_goal)));
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

  if (constraints.drain_battery())
  {
    const auto new_battery_soc =
      finish.battery_soc().value() - travel->change_in_charge();
    if (new_battery_soc < 0.0)
    {
      return std::nullopt;
    }
    finish.battery_soc(new_battery_soc);
  }

  if (finish.battery_soc().value() <= constraints.threshold_soc())
  {
    return std::nullopt;
  }

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
  std::vector<rmf_traffic::agv::Plan::Goal> one_of;
  std::vector<rmf_traffic::agv::Plan::Goal> expected_next_destinations;
  bool prefer_same_map = false;
};

//==============================================================================
auto GoToPlace::Description::make(Goal goal) -> DescriptionPtr
{
  auto desc = std::shared_ptr<Description>(new Description);
  desc->_pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{{std::move(goal)}, {}});

  return desc;
}

//==============================================================================
auto GoToPlace::Description::make_for_one_of(std::vector<Goal> goal)
-> DescriptionPtr
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
  if (_pimpl->prefer_same_map && invariant_initial_state.waypoint().has_value())
  {
    const std::size_t wp = invariant_initial_state.waypoint().value();
    const auto& graph = parameters.planner()->get_configuration().graph();
    const auto& map = graph.get_waypoint(wp).get_map_name();
    std::vector<Goal> goals;
    for (const auto& g : _pimpl->one_of)
    {
      const auto& goal_map = graph.get_waypoint(g.waypoint()).get_map_name();
      if (goal_map == map)
        goals.push_back(g);
    }

    const auto model = Model::make(invariant_initial_state, parameters, goals);
    if (model)
      return model;
  }

  return Model::make(
    std::move(invariant_initial_state),
    parameters,
    _pimpl->one_of);
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

  if (_pimpl->one_of.size() == 0)
  {
    utils::fail(fail_header, "No destination was specified");
  }

  const auto start_name = rmf_task::standard_waypoint_name(graph, start_wp);

  std::optional<rmf_traffic::Duration> estimate = std::nullopt;
  std::size_t selected_index = 0;
  for (std::size_t i = 0; i < _pimpl->one_of.size(); i ++)
  {
    auto dest = _pimpl->one_of[i];

    if (graph.num_waypoints() <= dest.waypoint())
    {
      utils::fail(fail_header, "Destination waypoint ["
        + std::to_string(dest.waypoint())
        + "] is outside the graph [" + std::to_string(graph.num_waypoints())
        + "]");
    }

    if (estimate.has_value())
    {
      auto curr_est = estimate_duration(
        parameters.planner(), initial_state, dest);
      if (curr_est.has_value() && curr_est.value() < estimate)
      {
        estimate = curr_est;
        selected_index = i;
      }
    }
    else
    {
      estimate = estimate_duration(
        parameters.planner(), initial_state, dest);
      selected_index = i;
    }
  }

  if (!estimate.has_value())
  {
    Header(
      "Go to one of [" + destination_name(parameters) + "]",
      "Waiting for path to open up",
      rmf_traffic::Duration(0));
  }

  auto goal_name =
    [&](const rmf_traffic::agv::Plan::Goal& goal)
    {
      return rmf_task::standard_waypoint_name(
        parameters.planner()->get_configuration().graph(),
        goal.waypoint());
    };

  const auto goal_name_ = goal_name(_pimpl->one_of[selected_index]);

  return Header(
    "Go to " + goal_name_,
    "Moving the robot from " + start_name + " to " + goal_name_,
    *estimate);
}

//==============================================================================
auto GoToPlace::Description::destination() const -> const Goal&
{
  return _pimpl->one_of.front();
}

//==============================================================================
auto GoToPlace::Description::one_of() const -> const std::vector<Goal>&
{
  return _pimpl->one_of;
}


//==============================================================================
auto GoToPlace::Description::destination(Goal new_goal) -> Description&
{
  _pimpl->one_of.resize(1, new_goal);
  return *this;
}

//==============================================================================
std::string GoToPlace::Description::destination_name(
  const Parameters& parameters) const
{
  if (_pimpl->one_of.empty())
    return "<none>";

  auto goal_name =
    [&](const rmf_traffic::agv::Plan::Goal& goal)
    {
      return rmf_task::standard_waypoint_name(
        parameters.planner()->get_configuration().graph(),
        goal.waypoint());
    };

  return std::accumulate(
    std::next(_pimpl->one_of.begin()),
    _pimpl->one_of.end(),
    goal_name(_pimpl->one_of.front()),
    [&](std::string a, const rmf_traffic::agv::Plan::Goal& goal)
    {
      a += " | ";
      a += goal_name(goal);
      return a;
    });
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
bool GoToPlace::Description::prefer_same_map() const
{
  return _pimpl->prefer_same_map;
}

//==============================================================================
auto GoToPlace::Description::prefer_same_map(bool choice) -> Description&
{
  _pimpl->prefer_same_map = choice;
  return *this;
}

//==============================================================================
GoToPlace::Description::Description()
{
  // Do nothing
}

} // namespace phases
} // namespace rmf_task_sequence
