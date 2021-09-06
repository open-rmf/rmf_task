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
    const Constraints& constraints) const final;

  // Documentation inherited
  rmf_traffic::Duration invariant_duration() const final;

  // Documentation inherited
  State invariant_finish_state() const final;

private:

  Model(
    State invariant_finish_state,
    rmf_traffic::Duration invariant_duration,
    const Parameters& parameters,
    Goal goal);

  State _invariant_finish_state;
  rmf_traffic::Duration _invariant_duration;
  Parameters _parameters;
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
      parameters,
      std::move(goal)));
}

//==============================================================================
std::optional<State> GoToPlace::Model::estimate_finish(
  State initial_state,
  const Constraints& constraints) const
{
  auto finish = initial_state;
  finish.waypoint(_goal.waypoint());

}

//==============================================================================
class GoToPlace::Description::Implementation
{
public:

  rmf_traffic::agv::Plan::Goal goal;

};

//==============================================================================
auto GoToPlace::Description::make(Goal goal)
-> std::shared_ptr<Description>
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

  auto get_wp_name = [&graph](std::size_t index) -> std::string
    {
      if (const auto* name = graph.get_waypoint(index).name())
        return *name;

      return "#" + std::to_string(index);
    };

  const auto start_name = get_wp_name(start_wp);

  if (graph.num_waypoints() <= _pimpl->goal.waypoint())
    return nullptr;

  const auto goal_name = get_wp_name(_pimpl->goal.waypoint());

  const auto estimate = estimate_duration(
    parameters.planner(), initial_state, _pimpl->goal);

  if (!estimate.has_value())
    return nullptr;

  return std::make_shared<execute::Phase::Tag>(
    "Go to [" + goal_name + "]",
    "Moving the robot from [" + start_name + "] to [" + goal_name + "]",
    *estimate);
}

//==============================================================================
GoToPlace::Description::Description()
{
  // Do nothing
}

} // namespace phases
} // namespace sequence
} // namespace rmf_task
