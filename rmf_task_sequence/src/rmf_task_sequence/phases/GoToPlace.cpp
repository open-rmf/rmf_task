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

#include <rmf_task/sequence/phases/GoToPlace.hpp>

namespace rmf_task {
namespace sequence {
namespace phases {

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
class GoToPlace::Model : public Phase::Model
{
public:

  static Phase::ConstModelPtr make(
    State invariant_initial_state,
    const Parameters& parameters,
    Goal goal);

  // Documentation inherited
  std::optional<State> estimate_finish(
    State initial_state,
    const Constraints& constraints,
    const TravelEstimator& ) const final;

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
Phase::ConstModelPtr GoToPlace::Model::make(
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
std::optional<State> GoToPlace::Model::estimate_finish(
  State initial_state,
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

  finish.time(finish.time().value() + travel->duration());
  auto battery_soc = finish.battery_soc().value();

  if (constraints.drain_battery())
    battery_soc = battery_soc - travel->change_in_charge();

  finish.battery_soc(battery_soc);

  const auto battery_threshold = constraints.threshold_soc();
  if (battery_soc <= battery_threshold)
    return std::nullopt;

  return finish;
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

  rmf_traffic::agv::Plan::Goal goal;

  static std::string waypoint_name(
    const std::size_t index,
    const Parameters& parameters)
  {
    const auto& graph = parameters.planner()->get_configuration().graph();
    if (index < graph.num_waypoints())
    {
      if (const auto* name = graph.get_waypoint(index).name())
        return *name;
    }

    return "#" + std::to_string(index);
  }
};

//==============================================================================
auto GoToPlace::Description::make(Goal goal) -> DescriptionPtr
{
  auto desc = std::shared_ptr<Description>(new Description);
  desc->_pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{
      std::move(goal)
    });

  return desc;
}

//==============================================================================
Phase::ConstModelPtr GoToPlace::Description::make_model(
  State invariant_initial_state,
  const Parameters& parameters) const
{
  return Model::make(
    std::move(invariant_initial_state),
    parameters,
    _pimpl->goal);
}

//==============================================================================
execute::Phase::ConstTagPtr GoToPlace::Description::make_tag(
  execute::Phase::Tag::Id id,
  const State& initial_state,
  const Parameters& parameters) const
{
  const auto start_wp_opt = initial_state.waypoint();
  if (!start_wp_opt)
    return nullptr;

  const auto start_wp = *start_wp_opt;

  const auto& graph = parameters.planner()->get_configuration().graph();
  if (graph.num_waypoints() <= start_wp)
    return nullptr;

  const auto start_name = Implementation::waypoint_name(start_wp, parameters);

  if (graph.num_waypoints() <= _pimpl->goal.waypoint())
    return nullptr;

  const auto goal_name_ = goal_name(parameters);

  const auto estimate = estimate_duration(
    parameters.planner(), initial_state, _pimpl->goal);

  if (!estimate.has_value())
    return nullptr;

  return std::make_shared<execute::Phase::Tag>(
    id,
    "Go to [" + goal_name_ + "]",
    "Moving the robot from [" + start_name + "] to [" + goal_name_ + "]",
    *estimate);
}

//==============================================================================
auto GoToPlace::Description::goal() const -> const Goal&
{
  return _pimpl->goal;
}

//==============================================================================
auto GoToPlace::Description::goal(Goal new_goal) -> Description&
{
  _pimpl->goal = std::move(new_goal);
  return *this;
}

//==============================================================================
std::string GoToPlace::Description::goal_name(
  const Parameters& parameters) const
{
  return Implementation::waypoint_name(_pimpl->goal.waypoint(), parameters);
}

//==============================================================================
GoToPlace::Description::Description()
{
  // Do nothing
}

} // namespace phases
} // namespace sequence
} // namespace rmf_task
