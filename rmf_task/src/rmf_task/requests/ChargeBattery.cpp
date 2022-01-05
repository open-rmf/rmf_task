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

#include <string>
#include <sstream>
#include <random>

#include <rmf_task/requests/ChargeBattery.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
namespace {
std::string generate_uuid(const std::size_t length = 3)
{
  std::stringstream ss;
  for (std::size_t i = 0; i < length; ++i)
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    const auto random_char = dis(gen);
    std::stringstream hexstream;
    hexstream << std::hex << random_char;
    auto hex = hexstream.str();
    ss << (hex.length() < 2 ? '0' + hex : hex);
  }
  return ss.str();
}

} // anonymous namespace

//==============================================================================
// Definition for forward declared class
class ChargeBattery::Model : public Task::Model
{
public:

  std::optional<Estimate> estimate_finish(
    const State& initial_state,
    const Constraints& task_planning_constraints,
    const TravelEstimator& travel_estimator) const final;

  rmf_traffic::Duration invariant_duration() const final;

  Model(
    const rmf_traffic::Time earliest_start_time,
    Parameters parameters);

private:
  rmf_traffic::Time _earliest_start_time;
  Parameters _parameters;
  rmf_traffic::Duration _invariant_duration;
};

//==============================================================================
ChargeBattery::Model::Model(
  const rmf_traffic::Time earliest_start_time,
  Parameters parameters)
: _earliest_start_time(earliest_start_time),
  _parameters(parameters)
{
  _invariant_duration = rmf_traffic::time::from_seconds(0.0);
}

//==============================================================================
std::optional<rmf_task::Estimate>
ChargeBattery::Model::estimate_finish(const State& initial_state,
  const Constraints& task_planning_constraints,
  const TravelEstimator& travel_estimator) const
{
  // Important to return nullopt if a charging task is not needed. In the task
  // planner, if a charging task is added, the node's latest time may be set to
  // the finishing time of the charging task, and consequently fall below the
  // segmentation threshold, causing `solve` to return. This may cause an
  // infinite loop as a new identical charging task is added in each call to
  // `solve` before returning.
  const auto recharge_soc = task_planning_constraints.recharge_soc();
  if (initial_state.battery_soc() >= recharge_soc - 1e-3
    && initial_state.waypoint().value()
    == initial_state.dedicated_charging_waypoint().value())
  {
    return std::nullopt;
  }

  // Compute time taken to reach charging waypoint from current location
  rmf_traffic::agv::Plan::Start final_plan_start{
    initial_state.time().value(),
    initial_state.dedicated_charging_waypoint().value(),
    initial_state.orientation().value()};

  auto state = State().load_basic(
    std::move(final_plan_start),
    initial_state.dedicated_charging_waypoint().value(),
    initial_state.battery_soc().value());

  double battery_soc = initial_state.battery_soc().value();
  rmf_traffic::Duration variant_duration(0);

  if (initial_state.waypoint() != initial_state.dedicated_charging_waypoint())
  {
    const auto travel = travel_estimator.estimate(
      initial_state.extract_plan_start().value(),
      rmf_traffic::agv::Plan::Goal(
        initial_state.dedicated_charging_waypoint().value()));

    if (!travel.has_value())
      return std::nullopt;

    variant_duration = travel->duration();
    if (task_planning_constraints.drain_battery())
      battery_soc = battery_soc - travel->change_in_charge();

    // If a robot cannot reach its charging dock given its initial battery soc
    if (battery_soc <= task_planning_constraints.threshold_soc())
      return std::nullopt;
  }

  double delta_soc = recharge_soc - battery_soc;
  if (delta_soc <= 1e-3)
  {
    // The robot's battery level when it reaches the charger is greater than
    // the specified battery level the robot should be charged up to. Hence, we
    // do not need a ChargeBattery task.
    return std::nullopt;
  }
  double time_to_charge =
    (3600 * delta_soc * _parameters.battery_system().capacity()) /
    _parameters.battery_system().charging_current();

  const rmf_traffic::Time wait_until = initial_state.time().value();
  state.time(
    wait_until + variant_duration +
    rmf_traffic::time::from_seconds(time_to_charge));

  state.battery_soc(recharge_soc);

  return Estimate(state, wait_until);
}

//==============================================================================
rmf_traffic::Duration ChargeBattery::Model::invariant_duration() const
{
  return _invariant_duration;
}

//==============================================================================
class ChargeBattery::Description::Implementation
{

};

//==============================================================================
Task::ConstDescriptionPtr ChargeBattery::Description::make()
{
  std::shared_ptr<Description> description(
    new Description());
  return description;
}

//==============================================================================
ChargeBattery::Description::Description()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
  // Do nothing
}

//==============================================================================
Task::ConstModelPtr ChargeBattery::Description::make_model(
  rmf_traffic::Time earliest_start_time,
  const Parameters& parameters) const
{
  return std::make_shared<ChargeBattery::Model>(
    earliest_start_time,
    parameters);
}

//==============================================================================
auto ChargeBattery::Description::generate_info(
  const State&,
  const Parameters&) const -> Info
{
  return Info{
    "Charge battery",
    ""
  };
}

//==============================================================================
ConstRequestPtr ChargeBattery::make(
  rmf_traffic::Time earliest_start_time,
  ConstPriorityPtr priority,
  bool automatic)
{

  std::string id = "Charge" + generate_uuid();
  const auto description = Description::make();

  return std::make_shared<Request>(
    id, earliest_start_time, priority, description, automatic);

}

} // namespace requests
} // namespace rmf_task
