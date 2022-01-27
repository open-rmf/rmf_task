/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <rmf_task_sequence/events/PerformAction.hpp>

#include "utils.hpp"

namespace rmf_task_sequence {
namespace events {

//==============================================================================
class PerformAction::Model : public Activity::Model
{
public:

  Model(
    rmf_task::State invariant_finish_state,
    rmf_traffic::Duration invariant_duration,
    bool use_tool_sink,
    const Parameters& parameters);

  std::optional<rmf_task::Estimate> estimate_finish(
    rmf_task::State initial_state,
    rmf_traffic::Time earliest_arrival_time,
    const Constraints& constraints,
    const TravelEstimator& travel_estimator) const final;

  rmf_traffic::Duration invariant_duration() const final;

  State invariant_finish_state() const final;

private:
  rmf_task::State _invariant_finish_state;
  double _invariant_battery_drain;
  rmf_traffic::Duration _invariant_duration;
  bool _use_tool_sink;
};

//==============================================================================
PerformAction::Model::Model(
  rmf_task::State invariant_finish_state,
  rmf_traffic::Duration invariant_duration,
  bool use_tool_sink,
  const Parameters& parameters)
: _invariant_finish_state(invariant_finish_state),
  _invariant_duration(invariant_duration),
  _use_tool_sink(use_tool_sink)
{
  if (parameters.ambient_sink() != nullptr)
  {
    _invariant_battery_drain =
      parameters.ambient_sink()->compute_change_in_charge(
      rmf_traffic::time::to_seconds(_invariant_duration));
  }

  if (_use_tool_sink && parameters.tool_sink() != nullptr)
  {
    _invariant_battery_drain +=
      parameters.tool_sink()->compute_change_in_charge(
      rmf_traffic::time::to_seconds(_invariant_duration));
  }
}

//==============================================================================
std::optional<rmf_task::Estimate> PerformAction::Model::estimate_finish(
  rmf_task::State initial_state,
  rmf_traffic::Time earliest_arrival_time,
  const Constraints& constraints,
  const TravelEstimator& travel_estimator) const
{
  initial_state.time(initial_state.time().value() + _invariant_duration);
  if (_invariant_finish_state.waypoint().has_value())
    initial_state.waypoint(_invariant_finish_state.waypoint().value());
  if (_invariant_finish_state.orientation().has_value())
    initial_state.orientation(_invariant_finish_state.orientation().value());

  if (constraints.drain_battery())
    initial_state.battery_soc(
      initial_state.battery_soc().value() - _invariant_battery_drain);

  if (initial_state.battery_soc().value() <= constraints.threshold_soc())
    return std::nullopt;

  return Estimate(initial_state, earliest_arrival_time);
}

//==============================================================================
rmf_traffic::Duration PerformAction::Model::invariant_duration() const
{
  return _invariant_duration;
}

//==============================================================================
State PerformAction::Model::invariant_finish_state() const
{
  return _invariant_finish_state;
}

//==============================================================================
class PerformAction::Description::Implementation
{
public:
  std::string category;
  nlohmann::json description;
  rmf_traffic::Duration action_duration_estimate;
  bool use_tool_sink;
  std::optional<Location> expected_finish_location;
};

//==============================================================================
auto PerformAction::Description::make(
  const std::string& category,
  nlohmann::json action,
  rmf_traffic::Duration duration,
  bool use_tool_sink,
  std::optional<PerformAction::Location> location) -> DescriptionPtr
{
  auto description = std::shared_ptr<Description>(new Description);
  description->_pimpl = rmf_utils::make_impl<Implementation>(
    Implementation
    {
      std::move(category),
      std::move(action),
      std::move(duration),
      std::move(use_tool_sink),
      std::move(location)
    });

  return description;
}

//==============================================================================
PerformAction::Description::Description()
{
  // Do nothing
}

//==============================================================================
const std::string&
PerformAction::Description::category() const
{
  return _pimpl->category;
}

//==============================================================================
auto PerformAction::Description::category(
  const std::string& new_category) -> Description&
{
  _pimpl->category = new_category;
  return *this;
}

//==============================================================================
const nlohmann::json&
PerformAction::Description::description() const
{
  return _pimpl->description;
}

//==============================================================================
auto PerformAction::Description::description(
  const nlohmann::json& new_description) -> Description&
{
  _pimpl->description = new_description;
  return *this;
}

//==============================================================================
const rmf_traffic::Duration&
PerformAction::Description::action_duration_estimate() const
{
  return _pimpl->action_duration_estimate;
}

//==============================================================================
auto PerformAction::Description::action_duration_estimate(
  rmf_traffic::Duration new_duration) -> Description&
{
  _pimpl->action_duration_estimate = std::move(new_duration);
  return *this;
}

//==============================================================================
bool PerformAction::Description::use_tool_sink() const
{
  return _pimpl->use_tool_sink;
}

//==============================================================================
auto PerformAction::Description::use_tool_sink(
  bool use_tool) -> Description&
{
  _pimpl->use_tool_sink = use_tool;
  return *this;
}

//==============================================================================
auto PerformAction::Description::expected_finish_location() const
-> std::optional<Location>
{
  return _pimpl->expected_finish_location;
}

//==============================================================================
auto PerformAction::Description::expected_finish_location(
  std::optional<Location> new_location) -> Description&
{
  _pimpl->expected_finish_location = std::move(new_location);
  return *this;
}

//==============================================================================
Activity::ConstModelPtr PerformAction::Description::make_model(
  State invariant_initial_state,
  const Parameters& parameters) const
{
  auto invariant_finish_state = invariant_initial_state;
  if (_pimpl->expected_finish_location.has_value())
  {
    const auto& goal = _pimpl->expected_finish_location.value();
    invariant_finish_state.waypoint(goal.waypoint());
    if (goal.orientation() != nullptr)
    {
      invariant_finish_state.orientation(*goal.orientation());
    }
  }

  return std::make_shared<Model>(
    invariant_finish_state,
    _pimpl->action_duration_estimate,
    _pimpl->use_tool_sink,
    parameters);
}

//==============================================================================
Header PerformAction::Description::generate_header(
  const rmf_task::State& initial_state,
  const Parameters& parameters) const
{

  const std::string& error_header =
    "[PerformAction::Description::generate_header]";

  const auto start_wp_opt = initial_state.waypoint();
  if (!start_wp_opt)
    utils::fail(
      error_header,
      "Initial state is missing a waypoint");

  const auto start_wp = *start_wp_opt;
  const auto start_name = utils::waypoint_name(start_wp, parameters);

  return Header(
    "Perform action",
    "Performing action " +  _pimpl->category +
    " at waypoint [" + start_name + "]",
    _pimpl->action_duration_estimate);

}

} // namespace events
} // namespace rmf_task_sequence
