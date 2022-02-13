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

#ifndef TEST__UNIT__UTILS_HPP
#define TEST__UNIT__UTILS_HPP

// #include <rmf_task_sequence/detail/ContactCard.hpp>
#include <rmf_task_sequence/Activity.hpp>

#include <rmf_task/Parameters.hpp>

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/Profile.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/BatterySystem.hpp>

#include <memory>

namespace {
using namespace std::chrono_literals;
//==============================================================================
// void CHECK_CONTACT(
//   const rmf_task_sequence::detail::ContactCard& contact,
//   const std::string& name,
//   const std::string& address,
//   const std::string& email,
//   const rmf_task_sequence::detail::ContactCard::PhoneNumber& number)
// {
//   CHECK(contact.name() == name);
//   CHECK(contact.address() == address);
//   CHECK(contact.email() == email);
//   CHECK(contact.number().country_code == number.country_code);
//   CHECK(contact.number().number == number.number);
// }

//==============================================================================
std::shared_ptr<rmf_task::Constraints> make_test_constraints(
  bool drain_battery = true)
{
  return std::make_shared<rmf_task::Constraints>(
    0.1,
    1.0,
    drain_battery);
}

//==============================================================================
std::shared_ptr<rmf_task::Parameters> make_test_parameters(
  bool drain_battery = true)
{
  using BatterySystem = rmf_battery::agv::BatterySystem;
  using MechanicalSystem = rmf_battery::agv::MechanicalSystem;
  using PowerSystem = rmf_battery::agv::PowerSystem;
  using SimpleMotionPowerSink = rmf_battery::agv::SimpleMotionPowerSink;
  using SimpleDevicePowerSink = rmf_battery::agv::SimpleDevicePowerSink;

  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {-5, -5}).set_passthrough_point(true); // 0
  graph.add_waypoint(test_map_name, { 0, -5}).set_passthrough_point(true); // 1
  graph.add_waypoint(test_map_name, { 5, -5}).set_passthrough_point(true); // 2
  graph.add_waypoint(test_map_name, {10, -5}).set_passthrough_point(true); // 3
  graph.add_waypoint(test_map_name, {-5, 0}); // 4
  graph.add_waypoint(test_map_name, { 0, 0}); // 5
  graph.add_waypoint(test_map_name, { 5, 0}); // 6
  graph.add_waypoint(test_map_name, {10, 0}).set_passthrough_point(true); // 7
  graph.add_waypoint(test_map_name, {10, 4}).set_passthrough_point(true); // 8
  graph.add_waypoint(test_map_name, { 0, 8}).set_passthrough_point(true); // 9
  graph.add_waypoint(test_map_name, { 5, 8}).set_passthrough_point(true); // 10
  graph.add_waypoint(test_map_name, {10, 12}).set_passthrough_point(true); // 11
  graph.add_waypoint(test_map_name, {12, 12}).set_passthrough_point(true); // 12
  REQUIRE(graph.num_waypoints() == 13);

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };

  add_bidir_lane(0, 1);
  add_bidir_lane(1, 2);
  add_bidir_lane(2, 3);
  add_bidir_lane(1, 5);
  add_bidir_lane(3, 7);
  add_bidir_lane(4, 5);
  add_bidir_lane(6, 10);
  add_bidir_lane(7, 8);
  add_bidir_lane(9, 10);
  add_bidir_lane(10, 11);

  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);
  const rmf_traffic::Profile profile{shape, shape};
  const rmf_traffic::agv::VehicleTraits traits(
    {1.0, 0.7}, {0.6, 0.5}, profile);
  rmf_traffic::schedule::Database database;
  const auto default_planner_options = rmf_traffic::agv::Planner::Options{
    nullptr};

  auto planner = std::make_shared<rmf_traffic::agv::Planner>(
    rmf_traffic::agv::Planner::Configuration{graph, traits},
    default_planner_options);

  auto battery_system_optional = BatterySystem::make(24.0, 40.0, 8.8);
  REQUIRE(battery_system_optional);
  BatterySystem& battery_system = *battery_system_optional;
  auto mechanical_system_optional = MechanicalSystem::make(70.0, 40.0, 0.22);
  REQUIRE(mechanical_system_optional);
  MechanicalSystem& mechanical_system = *mechanical_system_optional;
  auto power_system_optional = PowerSystem::make(20.0);
  REQUIRE(power_system_optional);
  PowerSystem& power_system_processor = *power_system_optional;

  std::shared_ptr<SimpleMotionPowerSink> motion_sink =
    std::make_shared<SimpleMotionPowerSink>(battery_system, mechanical_system);
  std::shared_ptr<SimpleDevicePowerSink> device_sink =
    std::make_shared<SimpleDevicePowerSink>(battery_system,
      power_system_processor);

  return std::make_shared<rmf_task::Parameters>(
    planner,
    battery_system,
    motion_sink,
    device_sink);

}

//==============================================================================
void CHECK_STATE(
  const rmf_task::State& estimated_state,
  const rmf_task::State& expected_state)
{
  if (expected_state.waypoint().has_value())
  {
    REQUIRE(estimated_state.waypoint().has_value());
    CHECK(
      expected_state.waypoint().value() == estimated_state.waypoint().value());
  }

  if (expected_state.orientation().has_value())
  {
    REQUIRE(estimated_state.orientation().has_value());
    CHECK(std::abs(expected_state.orientation().value() -
      estimated_state.orientation().value()) < 1e-3);
  }

  if (expected_state.time().has_value())
  {
    REQUIRE(estimated_state.time().has_value());
    CHECK(expected_state.time().value() -
      estimated_state.time().value() < 100ms);
  }

  if (expected_state.dedicated_charging_waypoint().has_value())
  {
    REQUIRE(estimated_state.dedicated_charging_waypoint().has_value());
    CHECK(expected_state.dedicated_charging_waypoint().value() ==
      estimated_state.dedicated_charging_waypoint().value());
  }

  if (expected_state.battery_soc().has_value())
  {
    REQUIRE(estimated_state.battery_soc().has_value());
    CHECK(estimated_state.battery_soc().value() <=
      expected_state.battery_soc().value());
  }
}

//==============================================================================
// TODO(YV): Also check for invariant_finish_state
void CHECK_MODEL(
  const rmf_task_sequence::Activity::Model& model,
  const rmf_task::State& initial_state,
  const rmf_traffic::Time earliest_arrival_time,
  const rmf_task::Constraints& constraints,
  const rmf_task::TravelEstimator& travel_estimator,
  const rmf_task::State& expected_finish_state)
{

  const auto estimated_finish = model.estimate_finish(
    initial_state,
    earliest_arrival_time,
    constraints,
    travel_estimator);

  REQUIRE(estimated_finish.has_value());

  CHECK_STATE(estimated_finish.value().finish_state(), expected_finish_state);
}

//==============================================================================
std::optional<rmf_traffic::Duration> estimate_travel_duration(
  const std::shared_ptr<const rmf_traffic::agv::Planner>& planner,
  const rmf_task::State& initial_state,
  const rmf_traffic::agv::Planner::Goal& goal)
{
  const auto result =
    planner->setup(initial_state.project_plan_start().value(), goal);

  if (result.disconnected())
    return std::nullopt;

  if (!result.ideal_cost().has_value())
    return std::nullopt;

  return rmf_traffic::time::from_seconds(*result.ideal_cost());
}

} // anonymous namespace

#endif // TEST__UNIT__UTILS_HPP
