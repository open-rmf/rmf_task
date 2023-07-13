/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/MechanicalSystem.hpp>
#include <rmf_battery/agv/PowerSystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>

#include <rmf_task/BinaryPriorityScheme.hpp>
#include <rmf_task/Task.hpp>
#include <rmf_task/TaskPlanner.hpp>

#include <rmf_task_sequence/Event.hpp>
#include <rmf_task_sequence/events/GoToPlace.hpp>
#include <rmf_task_sequence/events/PerformAction.hpp>
#include <rmf_task_sequence/events/WaitFor.hpp>

#include <rmf_task_sequence/Task.hpp>
#include <rmf_task_sequence/phases/SimplePhase.hpp>

#include <rmf_traffic/geometry/Circle.hpp>

#include <rmf_utils/catch.hpp>

using TaskPlanner = rmf_task::TaskPlanner;

using PerformAction = rmf_task_sequence::events::PerformAction;
using GoToPlace = rmf_task_sequence::events::GoToPlace;
using WaitFor = rmf_task_sequence::events::WaitFor;

using BatterySystem = rmf_battery::agv::BatterySystem;
using MechanicalSystem = rmf_battery::agv::MechanicalSystem;
using PowerSystem = rmf_battery::agv::PowerSystem;
using SimpleMotionPowerSink = rmf_battery::agv::SimpleMotionPowerSink;
using SimpleDevicePowerSink = rmf_battery::agv::SimpleDevicePowerSink;

//==============================================================================
SCENARIO("Test GoToPlace and PerformAction Compose task")
{
  // Simple graph with just two waypoints
  rmf_traffic::agv::Graph graph;
  const std::string map_name = "test_map";
  graph.add_waypoint(map_name, {0.0, 0.0}).set_charger(true);
  graph.add_waypoint(map_name, {0.0, 10.0});

  graph.add_lane(0, 1);
  graph.add_lane(1, 0);

  // Vehicle traits
  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);
  const rmf_traffic::Profile profile{shape, shape};
  const rmf_traffic::agv::VehicleTraits traits(
    {1.0, 0.7}, {0.6, 0.5}, profile);
  const auto default_planner_options = rmf_traffic::agv::Planner::Options{
    nullptr};

  auto planner = std::make_shared<rmf_traffic::agv::Planner>(
    rmf_traffic::agv::Planner::Configuration{graph, traits},
    default_planner_options);

  // Battery and task planning parameters
  const bool drain_battery = true;
  auto battery_system_optional = BatterySystem::make(24.0, 40.0, 8.8);
  REQUIRE(battery_system_optional);
  BatterySystem& battery_system = *battery_system_optional;
  auto mechanical_system_optional = MechanicalSystem::make(70.0, 40.0, 0.22);
  REQUIRE(mechanical_system_optional);
  MechanicalSystem& mechanical_system = *mechanical_system_optional;
  auto power_system_optional = PowerSystem::make(1.0);
  REQUIRE(power_system_optional);
  PowerSystem& power_system_processor = *power_system_optional;
  // Will consume 50% battery for the requested 1h task
  auto tool_system_optional = PowerSystem::make(480.0);
  REQUIRE(tool_system_optional);
  PowerSystem& tool_system_processor = *tool_system_optional;

  std::shared_ptr<SimpleMotionPowerSink> motion_sink =
    std::make_shared<SimpleMotionPowerSink>(battery_system, mechanical_system);
  std::shared_ptr<SimpleDevicePowerSink> device_sink =
    std::make_shared<SimpleDevicePowerSink>(battery_system,
      power_system_processor);
  std::shared_ptr<SimpleDevicePowerSink> tool_sink =
    std::make_shared<SimpleDevicePowerSink>(battery_system,
      tool_system_processor);

  const auto cost_calculator =
    rmf_task::BinaryPriorityScheme::make_cost_calculator();

  const rmf_task::Constraints constraints{0.2, 1.0, drain_battery};
  const rmf_task::Parameters parameters{
    planner,
    battery_system,
    motion_sink,
    device_sink,
    tool_sink};

  const TaskPlanner::Configuration task_config{
    parameters,
    constraints,
    cost_calculator};

  // But default we use the optimal solver
  const auto default_options = TaskPlanner::Options{
    false,
    nullptr,
    nullptr};

  auto gotoplace_description =
    GoToPlace::Description::make(GoToPlace::Goal(1));
  auto action_description = PerformAction::Description::make(
    "clean",
    {},
    rmf_traffic::time::from_seconds(3600),
    true);

  auto gotoplace_phase =
    rmf_task_sequence::phases::SimplePhase::Description::make(
    gotoplace_description);
  auto action_phase = rmf_task_sequence::phases::SimplePhase::Description::make(
    action_description);

  // Build the task
  rmf_task_sequence::Task::Builder builder;
  builder.add_phase(gotoplace_phase, {});

  auto gotoplace_task = builder.build("mock_category", "mock_tag");

  builder.add_phase(action_phase, {});
  auto compose_task = builder.build("mock_category", "mock_tag");

  builder = rmf_task_sequence::Task::Builder();
  builder.add_phase(action_phase, {});
  auto action_task = builder.build("mock_category", "mock_tag");

  rmf_task::Task::ConstBookingPtr compose_booking =
    std::make_shared<const rmf_task::Task::Booking>(
    "mock_id",
    std::chrono::steady_clock::now(),
    nullptr);
  auto compose_request = std::make_shared<rmf_task::Request>(
    std::move(compose_booking),
    std::move(compose_task));

  rmf_task::Task::ConstBookingPtr gotoplace_booking =
    std::make_shared<const rmf_task::Task::Booking>(
    "mock_id2",
    std::chrono::steady_clock::now(),
    nullptr);
  auto gotoplace_request = std::make_shared<rmf_task::Request>(
    std::move(gotoplace_booking),
    std::move(gotoplace_task));

  rmf_task::Task::ConstBookingPtr action_booking =
    std::make_shared<const rmf_task::Task::Booking>(
    "mock_id3",
    std::chrono::steady_clock::now(),
    nullptr);
  auto action_request = std::make_shared<rmf_task::Request>(
    std::move(action_booking),
    std::move(action_task));

  WHEN("Planning for a task with a gotoplace and perform_action phase")
  {
    // Result for both scenarios when battery is low should be the same
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;
    const double initial_soc = 0.3;

    rmf_traffic::agv::Plan::Start first_location{now, 0, default_orientation};
    rmf_task::State initial_state = rmf_task::State().load_basic(first_location,
        0,
        initial_soc);

    WHEN("It is a compose task with two phases")
    {
      TaskPlanner task_planner(task_config, default_options);

      const auto optimal_result = task_planner.plan(
        now, {initial_state}, {compose_request});
      const auto optimal_assignments = std::get_if<
        TaskPlanner::Assignments>(&optimal_result);
      CHECK(optimal_assignments);
    }

    WHEN("It is two separate tasks with one phase")
    {
      TaskPlanner task_planner(task_config, default_options);

      const auto optimal_result = task_planner.plan(
        now, {initial_state}, {gotoplace_request, action_request});
      const auto optimal_assignments = std::get_if<
        TaskPlanner::Assignments>(&optimal_result);
      CHECK(optimal_assignments);
    }
  }
}

//==============================================================================
SCENARIO("Test battery drain for events")
{
  // Simple graph with just two waypoints
  // 0 ---- 1
  // |
  // .
  // |
  // 2
  rmf_traffic::agv::Graph graph;
  const std::string map_name = "test_map";
  graph.add_waypoint(map_name, {0.0, 0.0}).set_charger(true);
  graph.add_waypoint(map_name, {0.0, 10.0});
  graph.add_waypoint(map_name, {1e6, 0.0});

  graph.add_lane(0, 1);
  graph.add_lane(1, 0);
  graph.add_lane(0, 2);
  graph.add_lane(2, 0);

  // Vehicle traits
  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);
  const rmf_traffic::Profile profile{shape, shape};
  const rmf_traffic::agv::VehicleTraits traits(
    {1.0, 0.7}, {0.6, 0.5}, profile);
  const auto default_planner_options = rmf_traffic::agv::Planner::Options{
    nullptr};

  auto planner = std::make_shared<rmf_traffic::agv::Planner>(
    rmf_traffic::agv::Planner::Configuration{graph, traits},
    default_planner_options);

  // Battery and task planning parameters
  const bool drain_battery = true;
  auto battery_system_optional = BatterySystem::make(24.0, 40.0, 8.8);
  REQUIRE(battery_system_optional);
  BatterySystem& battery_system = *battery_system_optional;
  auto mechanical_system_optional = MechanicalSystem::make(70.0, 40.0, 0.22);
  REQUIRE(mechanical_system_optional);
  MechanicalSystem& mechanical_system = *mechanical_system_optional;
  auto power_system_optional = PowerSystem::make(1.0);
  REQUIRE(power_system_optional);
  PowerSystem& power_system_processor = *power_system_optional;
  // Will consume 50% battery for the requested 1h task
  auto tool_system_optional = PowerSystem::make(480.0);
  REQUIRE(tool_system_optional);
  PowerSystem& tool_system_processor = *tool_system_optional;

  std::shared_ptr<SimpleMotionPowerSink> motion_sink =
    std::make_shared<SimpleMotionPowerSink>(battery_system, mechanical_system);
  std::shared_ptr<SimpleDevicePowerSink> device_sink =
    std::make_shared<SimpleDevicePowerSink>(battery_system,
      power_system_processor);
  std::shared_ptr<SimpleDevicePowerSink> tool_sink =
    std::make_shared<SimpleDevicePowerSink>(battery_system,
      tool_system_processor);

  const auto cost_calculator =
    rmf_task::BinaryPriorityScheme::make_cost_calculator();

  const rmf_task::Constraints constraints{0.2, 1.0, drain_battery};
  const rmf_task::Parameters parameters{
    planner,
    battery_system,
    motion_sink,
    device_sink,
    tool_sink};

  const TaskPlanner::Configuration task_config{
    parameters,
    constraints,
    cost_calculator};

  // But default we use the optimal solver
  const auto default_options = TaskPlanner::Options{
    false,
    nullptr,
    nullptr};

  // Common robot states and description.
  // Result for both scenarios when battery is low should be the same
  const auto now = std::chrono::steady_clock::now();
  const double default_orientation = 0.0;
  const double initial_soc = 0.3;

  rmf_traffic::agv::Plan::Start first_location{now, 0, default_orientation};
  rmf_task::State initial_state = rmf_task::State().load_basic(first_location,
      0,
      initial_soc);

  const auto gotoplace_description0 =
    GoToPlace::Description::make(GoToPlace::Goal(0));
  auto gotoplace0_phase =
    rmf_task_sequence::phases::SimplePhase::Description::make(
    gotoplace_description0);
  const auto gotoplace_description1 =
    GoToPlace::Description::make(GoToPlace::Goal(1));
  auto gotoplace1_phase =
    rmf_task_sequence::phases::SimplePhase::Description::make(
    gotoplace_description1);
  const auto gotoplace_description2 =
    GoToPlace::Description::make(GoToPlace::Goal(2));
  auto gotoplace2_phase =
    rmf_task_sequence::phases::SimplePhase::Description::make(
    gotoplace_description2);

  WHEN("Request contains a single feasible GoToPlace event")
  {
    TaskPlanner task_planner(task_config, default_options);

    // Generate a composable request.
    rmf_task_sequence::Task::Builder builder;
    builder.add_phase(gotoplace0_phase, {});
    auto compose_task = builder.build("mock_category", "mock_tag");
    rmf_task::Task::ConstBookingPtr compose_booking =
      std::make_shared<const rmf_task::Task::Booking>(
      "mock_id",
      std::chrono::steady_clock::now(),
      nullptr);
    auto compose_request = std::make_shared<rmf_task::Request>(
      std::move(compose_booking),
      std::move(compose_task));

    // Generate the plan.
    const auto optimal_result = task_planner.plan(
      now, {initial_state}, {compose_request});
    const auto optimal_assignments = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    CHECK(optimal_assignments);
  }
  WHEN("Request is infeasible due to two or more GoToPlace events")
  {
    TaskPlanner task_planner(task_config, default_options);

    // Generate a composable request.
    rmf_task_sequence::Task::Builder builder;
    for (std::size_t i = 0; i < 1e3; ++i)
    {
      builder.add_phase(gotoplace1_phase, {});
      builder.add_phase(gotoplace0_phase, {});
    }
    auto compose_task = builder.build("mock_category", "mock_tag");
    rmf_task::Task::ConstBookingPtr compose_booking =
      std::make_shared<const rmf_task::Task::Booking>(
      "mock_id",
      std::chrono::steady_clock::now(),
      nullptr);
    auto compose_request = std::make_shared<rmf_task::Request>(
      std::move(compose_booking),
      std::move(compose_task));

    // Generate the plan.
    const auto optimal_result = task_planner.plan(
      now, {initial_state}, {compose_request});
    const auto error = std::get_if<
      TaskPlanner::TaskPlannerError>(&optimal_result);
    REQUIRE(error);
    CHECK(*error == TaskPlanner::TaskPlannerError::limited_capacity);
  }
  WHEN("Request contains a single infeasible GoToPlace event")
  {
    TaskPlanner task_planner(task_config, default_options);

    // Generate a composable request.
    rmf_task_sequence::Task::Builder builder;
    builder.add_phase(gotoplace2_phase, {});
    auto compose_task = builder.build("mock_category", "mock_tag");
    rmf_task::Task::ConstBookingPtr compose_booking =
      std::make_shared<const rmf_task::Task::Booking>(
      "mock_id",
      std::chrono::steady_clock::now(),
      nullptr);
    auto compose_request = std::make_shared<rmf_task::Request>(
      std::move(compose_booking),
      std::move(compose_task));

    // Generate the plan.
    const auto optimal_result = task_planner.plan(
      now, {initial_state}, {compose_request});
    const auto error = std::get_if<
      TaskPlanner::TaskPlannerError>(&optimal_result);
    REQUIRE(error);
    CHECK(*error == TaskPlanner::TaskPlannerError::limited_capacity);
  }
  WHEN("Request contains a single feasible WaitFor event")
  {
    TaskPlanner task_planner(task_config, default_options);

    // Generate a composable request.
    const auto waitfor_description =
      WaitFor::Description::make(std::chrono::steady_clock::duration(10));
    auto waitfor_phase =
      rmf_task_sequence::phases::SimplePhase::Description::make(
      waitfor_description);

    rmf_task_sequence::Task::Builder builder;
    builder.add_phase(waitfor_phase, {});
    auto compose_task = builder.build("mock_category", "mock_tag");
    rmf_task::Task::ConstBookingPtr compose_booking =
      std::make_shared<const rmf_task::Task::Booking>(
      "mock_id",
      std::chrono::steady_clock::now(),
      nullptr);
    auto compose_request = std::make_shared<rmf_task::Request>(
      std::move(compose_booking),
      std::move(compose_task));

    // Generate the plan.
    const auto optimal_result = task_planner.plan(
      now, {initial_state}, {compose_request});
    const auto assignments = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    CHECK(assignments);
  }
  WHEN("Request contains a single infeasible WaitFor event")
  {
    TaskPlanner task_planner(task_config, default_options);

    // Generate a composable request.
    const auto waitfor_description =
      WaitFor::Description::make(std::chrono::steady_clock::duration::max());
    auto waitfor_phase =
      rmf_task_sequence::phases::SimplePhase::Description::make(
      waitfor_description);

    rmf_task_sequence::Task::Builder builder;
    builder.add_phase(waitfor_phase, {});
    auto compose_task = builder.build("mock_category", "mock_tag");
    rmf_task::Task::ConstBookingPtr compose_booking =
      std::make_shared<const rmf_task::Task::Booking>(
      "mock_id",
      std::chrono::steady_clock::now(),
      nullptr);
    auto compose_request = std::make_shared<rmf_task::Request>(
      std::move(compose_booking),
      std::move(compose_task));

    // Generate the plan.
    const auto optimal_result = task_planner.plan(
      now, {initial_state}, {compose_request});
    const auto error = std::get_if<
      TaskPlanner::TaskPlannerError>(&optimal_result);
    REQUIRE(error);
    CHECK(*error == TaskPlanner::TaskPlannerError::limited_capacity);
  }
  WHEN("Request contains a WaitFor event with invalid duration")
  {
    TaskPlanner task_planner(task_config, default_options);

    // Generate a composable request.
    const auto waitfor_description =
      WaitFor::Description::make(std::chrono::steady_clock::duration::min());
    auto waitfor_phase =
      rmf_task_sequence::phases::SimplePhase::Description::make(
      waitfor_description);

    rmf_task_sequence::Task::Builder builder;
    builder.add_phase(waitfor_phase, {});
    auto compose_task = builder.build("mock_category", "mock_tag");
    rmf_task::Task::ConstBookingPtr compose_booking =
      std::make_shared<const rmf_task::Task::Booking>(
      "mock_id",
      std::chrono::steady_clock::now(),
      nullptr);
    auto compose_request = std::make_shared<rmf_task::Request>(
      std::move(compose_booking),
      std::move(compose_task));

    // Generate the plan.
    const auto optimal_result = task_planner.plan(
      now, {initial_state}, {compose_request});
    const auto assignments = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    CHECK(assignments);
  }
}
