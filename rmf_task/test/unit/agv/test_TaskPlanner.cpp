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

#include <rmf_task/agv/TaskPlanner.hpp>
#include <rmf_task/agv/State.hpp>
#include <rmf_task/agv/Constraints.hpp>
#include <rmf_task/agv/Parameters.hpp>
#include <rmf_task/requests/Delivery.hpp>
#include <rmf_task/requests/ChargeBattery.hpp>
#include <rmf_task/requests/Loop.hpp>

#include <rmf_task/BinaryPriorityScheme.hpp>

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

#include <rmf_utils/catch.hpp>

#include <iostream>

using TaskPlanner = rmf_task::agv::TaskPlanner;

//==============================================================================
inline void CHECK_TIMES(const TaskPlanner::Assignments& assignments,
  const rmf_traffic::Time now)
{
  for (const auto& agent : assignments)
  {
    for (std::size_t i = 0; i < agent.size(); ++i)
    {
      CHECK(agent[i].deployment_time() >= now);
      CHECK(agent[i].state().finish_time() >= agent[i].deployment_time());
      if (i == 0)
        continue;
      CHECK(agent[i].deployment_time() >= agent[i-1].state().finish_time());
    }
  }
}

//==============================================================================
inline bool check_implicit_charging_task_start(
  const TaskPlanner::Assignments& assignments,
  const double initial_soc)
{
  bool implicit_charging_task_added = false;
  for (const auto& agent : assignments)
  {
    if (!agent.size())
    {
      continue;
    }

    const auto& s = agent[0].state();
    auto is_charge_request =
      std::dynamic_pointer_cast<
      const rmf_task::requests::ChargeBattery::Description>(
      agent[0].request()->description());

    // No task should consume more charge than (1.0 - initial_soc)
    // in the current test, so we are guaranted to find any occurrence
    // of an implicit charging task.
    if (!is_charge_request && s.battery_soc() > initial_soc)
    {
      implicit_charging_task_added = true;
      break;
    }
  }

  return implicit_charging_task_added;
}

//==============================================================================
inline void display_solution(
  std::string parent,
  const TaskPlanner::Assignments& assignments,
  const double cost)
{
  std::cout << parent << " cost: " << cost << std::endl;
  std::cout << parent << " assignments:" << std::endl;
  for (std::size_t i = 0; i < assignments.size(); ++i)
  {
    std::cout << "--Agent: " << i << std::endl;
    for (const auto& a : assignments[i])
    {
      const auto& s = a.state();
      const double request_seconds =
        a.request()->earliest_start_time().time_since_epoch().count();
      const double start_seconds =
        a.deployment_time().time_since_epoch().count();
      const rmf_traffic::Time finish_time = s.finish_time();
      const double finish_seconds = finish_time.time_since_epoch().count();
      std::cout << std::fixed
                << "    <" << a.request()->id() << ": " << request_seconds
                << ", " << start_seconds
                << ", "<< finish_seconds << ", " << 100* s.battery_soc()
                << "%>" << std::endl;
    }
  }
  std::cout << " ----------------------" << std::endl;
}

//==============================================================================
SCENARIO("Grid World")
{
  const bool display_solutions = false;
  const int grid_size = 4;
  const double edge_length = 1000;
  const bool drain_battery = true;

  using BatterySystem = rmf_battery::agv::BatterySystem;
  using MechanicalSystem = rmf_battery::agv::MechanicalSystem;
  using PowerSystem = rmf_battery::agv::PowerSystem;
  using SimpleMotionPowerSink = rmf_battery::agv::SimpleMotionPowerSink;
  using SimpleDevicePowerSink = rmf_battery::agv::SimpleDevicePowerSink;

  rmf_traffic::agv::Graph graph;
  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };

  const std::string map_name = "test_map";

  for (int i = 0; i < grid_size; ++i)
  {
    for (int j = 0; j < grid_size; ++j)
    {
      // const auto random = (double) rand() / RAND_MAX;
      const double random = 1.0;
      graph.add_waypoint(map_name,
        {j* edge_length* random, -i* edge_length* random});
    }
  }

  for (int i = 0; i < grid_size*grid_size; ++i)
  {
    if ((i+1) % grid_size != 0)
      add_bidir_lane(i, i+1);
    if (i + grid_size < grid_size*grid_size)
      add_bidir_lane(i, i+4);
  }

  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);
  const rmf_traffic::Profile profile{shape, shape};
  const rmf_traffic::agv::VehicleTraits traits(
    {1.0, 0.7}, {0.6, 0.5}, profile);
  rmf_traffic::schedule::Database database;
  const auto default_options = rmf_traffic::agv::Planner::Options{
    nullptr};

  auto planner = std::make_shared<rmf_traffic::agv::Planner>(
    rmf_traffic::agv::Planner::Configuration{graph, traits},
    default_options);

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

  const auto cost_calculator =
    rmf_task::BinaryPriorityScheme::make_cost_calculator();

  const rmf_task::agv::Constraints constraints{0.2, 1.0, drain_battery};
  const rmf_task::agv::Parameters parameters{
    planner,
    battery_system,
    motion_sink,
    device_sink};

  const TaskPlanner::Configuration task_config{
    parameters,
    constraints,
    cost_calculator};

  WHEN("Planning for 3 requests and 2 agents")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};
    rmf_traffic::agv::Plan::Start second_location{now, 2, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, 1.0},
      rmf_task::agv::State{second_location, 2, 1.0}
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Delivery::make(
        0,
        "dispenser",
        3,
        "ingestor",
        {},
        "1",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        15,
        "dispenser",
        2,
        "ingestor",
        {},
        "2",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        7,
        "dispenser",
        9,
        "ingestor",
        {},
        "3",
        now + rmf_traffic::time::from_seconds(0))
    };


    TaskPlanner task_planner(task_config);

    auto start_time = std::chrono::steady_clock::now();
    const auto greedy_result = task_planner.greedy_plan(
      now, initial_states, requests);
    const auto greedy_assignments = std::get_if<
      TaskPlanner::Assignments>(&greedy_result);
    REQUIRE(greedy_assignments);
    const double greedy_cost = task_planner.compute_cost(*greedy_assignments);
    auto finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(*greedy_assignments, now);

    if (display_solutions)
    {
      std::cout << "Greedy solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Greedy", *greedy_assignments, greedy_cost);
    }

    // Create new TaskPlanner to reset cache so that measured run times
    // remain independent of one another
    task_planner = TaskPlanner(task_config);
    start_time = std::chrono::steady_clock::now();
    const auto optimal_result = task_planner.optimal_plan(
      now, initial_states, requests, nullptr);
    const auto optimal_assignments = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    const double optimal_cost = task_planner.compute_cost(*optimal_assignments);
    finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(*optimal_assignments, now);

    if (display_solutions)
    {
      std::cout << "Optimal solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Optimal", *optimal_assignments, optimal_cost);
    }

    REQUIRE(optimal_cost <= greedy_cost);
  }

  WHEN("Planning for 11 requests and 2 agents")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};
    rmf_traffic::agv::Plan::Start second_location{now, 2, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, 1.0},
      rmf_task::agv::State{second_location, 2, 1.0}
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Delivery::make(
        0,
        "dispenser",
        3,
        "ingestor",
        {},
        "1",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        15,
        "dispenser",
        2,
        "ingestor",
        {},
        "2",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        7,
        "dispenser",
        9,
        "ingestor",
        {},
        "3",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        8,
        "dispenser",
        11,
        "ingestor",
        {},
        "4",
        now + rmf_traffic::time::from_seconds(50000)),

      rmf_task::requests::Delivery::make(
        10,
        "dispenser",
        0,
        "ingestor",
        {},
        "5",
        now + rmf_traffic::time::from_seconds(50000)),

      rmf_task::requests::Delivery::make(
        4,
        "dispenser",
        8,
        "ingestor",
        {},
        "6",
        now + rmf_traffic::time::from_seconds(60000)),

      rmf_task::requests::Delivery::make(
        8,
        "dispenser",
        14,
        "ingestor",
        {},
        "7",
        now + rmf_traffic::time::from_seconds(60000)),

      rmf_task::requests::Delivery::make(
        5,
        "dispenser",
        11,
        "ingestor",
        {},
        "8",
        now + rmf_traffic::time::from_seconds(60000)),

      rmf_task::requests::Delivery::make(
        9,
        "dispenser",
        0,
        "ingestor",
        {},
        "9",
        now + rmf_traffic::time::from_seconds(60000)),

      rmf_task::requests::Delivery::make(
        1,
        "dispenser",
        3,
        "ingestor",
        {},
        "10",
        now + rmf_traffic::time::from_seconds(60000)),

      rmf_task::requests::Delivery::make(
        0,
        "dispenser",
        12,
        "ingestor",
        {},
        "11",
        now + rmf_traffic::time::from_seconds(60000))
    };

    TaskPlanner task_planner(task_config);

    auto start_time = std::chrono::steady_clock::now();
    const auto greedy_result = task_planner.greedy_plan(
      now, initial_states, requests);
    const auto greedy_assignments = std::get_if<
      TaskPlanner::Assignments>(&greedy_result);
    REQUIRE(greedy_assignments);
    const double greedy_cost = task_planner.compute_cost(*greedy_assignments);
    auto finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(*greedy_assignments, now);

    if (display_solutions)
    {
      std::cout << "Greedy solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Greedy", *greedy_assignments, greedy_cost);
    }

    // Create new TaskPlanner to reset cache so that measured run times
    // remain independent of one another
    task_planner = TaskPlanner(task_config);
    start_time = std::chrono::steady_clock::now();
    const auto optimal_result = task_planner.optimal_plan(
      now, initial_states, requests, nullptr);
    const auto optimal_assignments = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    const double optimal_cost = task_planner.compute_cost(*optimal_assignments);
    finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(*optimal_assignments, now);

    if (display_solutions)
    {
      std::cout << "Optimal solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Optimal", *optimal_assignments, optimal_cost);
    }

    REQUIRE(optimal_cost <= greedy_cost);
  }

  WHEN("Initial charge is low")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;
    const double initial_soc = 0.3;

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};
    rmf_traffic::agv::Plan::Start second_location{now, 2, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, initial_soc},
      rmf_task::agv::State{second_location, 2, initial_soc}
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Delivery::make(
        0,
        "dispenser",
        3,
        "ingestor",
        {},
        "1",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        15,
        "dispenser",
        2,
        "ingestor",
        {},
        "2",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        9,
        "dispenser",
        4,
        "ingestor",
        {},
        "3",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        8,
        "dispenser",
        11,
        "ingestor",
        {},
        "4",
        now + rmf_traffic::time::from_seconds(50000))
    };

    TaskPlanner task_planner(task_config);

    auto start_time = std::chrono::steady_clock::now();
    const auto greedy_result = task_planner.greedy_plan(
      now, initial_states, requests);
    const auto greedy_assignments = std::get_if<
      TaskPlanner::Assignments>(&greedy_result);
    REQUIRE(greedy_assignments);
    const double greedy_cost = task_planner.compute_cost(*greedy_assignments);
    auto finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(*greedy_assignments, now);

    if (display_solutions)
    {
      std::cout << "Greedy solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Greedy", *greedy_assignments, greedy_cost);
    }

    // Create new TaskPlanner to reset cache so that measured run times
    // remain independent of one another
    task_planner = TaskPlanner(task_config);
    start_time = std::chrono::steady_clock::now();
    const auto optimal_result = task_planner.optimal_plan(
      now, initial_states, requests, nullptr);
    const auto optimal_assignments = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    const double optimal_cost = task_planner.compute_cost(*optimal_assignments);
    finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(*optimal_assignments, now);

    if (display_solutions)
    {
      std::cout << "Optimal solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Optimal", *optimal_assignments, optimal_cost);
    }

    REQUIRE(optimal_cost <= greedy_cost);

    // Checks if Assignments take into account a charging task in the beginning
    // without explicitly including the task in Assignments.
    bool implicit_charging_task_added = check_implicit_charging_task_start(
      *greedy_assignments, initial_soc);
    REQUIRE(!implicit_charging_task_added);

    implicit_charging_task_added = check_implicit_charging_task_start(
      *optimal_assignments, initial_soc);
    REQUIRE(!implicit_charging_task_added);
  }

  WHEN("Planning for 11 requests and 2 agents no.2")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};
    rmf_traffic::agv::Plan::Start second_location{now, 2, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 9, 1.0},
      rmf_task::agv::State{second_location, 2, 1.0}
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Delivery::make(
        6,
        "dispenser",
        3,
        "ingestor",
        {},
        "1",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        10,
        "dispenser",
        7,
        "ingestor",
        {},
        "2",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        2,
        "dispenser",
        12,
        "ingestor",
        {},
        "3",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        8,
        "dispenser",
        11,
        "ingestor",
        {},
        "4",
        now + rmf_traffic::time::from_seconds(50000)),

      rmf_task::requests::Delivery::make(
        10,
        "dispenser",
        6,
        "ingestor",
        {},
        "5",
        now + rmf_traffic::time::from_seconds(50000)),

      rmf_task::requests::Delivery::make(
        2,
        "dispenser",
        9,
        "ingestor",
        {},
        "6",
        now + rmf_traffic::time::from_seconds(70000)),

      rmf_task::requests::Delivery::make(
        3,
        "dispenser",
        4,
        "ingestor",
        {},
        "7",
        now + rmf_traffic::time::from_seconds(70000)),

      rmf_task::requests::Delivery::make(
        5,
        "dispenser",
        11,
        "ingestor",
        {},
        "8",
        now + rmf_traffic::time::from_seconds(70000)),

      rmf_task::requests::Delivery::make(
        9,
        "dispenser",
        1,
        "ingestor",
        {},
        "9",
        now + rmf_traffic::time::from_seconds(70000)),

      rmf_task::requests::Delivery::make(
        1,
        "dispenser",
        5,
        "ingestor",
        {},
        "10",
        now + rmf_traffic::time::from_seconds(70000)),

      rmf_task::requests::Delivery::make(
        13,
        "dispenser",
        10,
        "ingestor",
        {},
        "11",
        now + rmf_traffic::time::from_seconds(70000))
    };

    TaskPlanner task_planner(task_config);

    auto start_time = std::chrono::steady_clock::now();
    const auto greedy_result = task_planner.greedy_plan(
      now, initial_states, requests);
    const auto greedy_assignments = std::get_if<
      TaskPlanner::Assignments>(&greedy_result);
    REQUIRE(greedy_assignments);
    const double greedy_cost = task_planner.compute_cost(*greedy_assignments);
    auto finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(*greedy_assignments, now);

    if (display_solutions)
    {
      std::cout << "Greedy solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Greedy", *greedy_assignments, greedy_cost);
    }

    // Create new TaskPlanner to reset cache so that measured run times
    // remain independent of one another
    task_planner = TaskPlanner(task_config);
    start_time = std::chrono::steady_clock::now();
    const auto optimal_result = task_planner.optimal_plan(
      now, initial_states, requests, nullptr);
    const auto optimal_assignments = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    const double optimal_cost = task_planner.compute_cost(*optimal_assignments);
    finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(*optimal_assignments, now);

    if (display_solutions)
    {
      std::cout << "Optimal solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Optimal", *optimal_assignments, optimal_cost);
    }

    REQUIRE(optimal_cost <= greedy_cost);
  }

  WHEN("A loop request is impossible to fulfil due to battery capacity")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, 1.0},
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Loop::make(
        0,
        15,
        1000,
        "Loop1",
        now)
    };

    TaskPlanner task_planner(task_config);

    const auto greedy_result = task_planner.greedy_plan(
      now, initial_states, requests);
    const auto greedy_assignments = std::get_if<
      TaskPlanner::Assignments>(&greedy_result);
    REQUIRE_FALSE(greedy_assignments);
    auto error = std::get_if<
      TaskPlanner::TaskPlannerError>(&greedy_result);
    CHECK(*error == TaskPlanner::TaskPlannerError::limited_capacity);

    // Create new TaskPlanner to reset cache so that measured run times
    // remain independent of one another
    task_planner = TaskPlanner(task_config);
    const auto optimal_result = task_planner.optimal_plan(
      now, initial_states, requests, nullptr);
    const auto optimal_assignments = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    REQUIRE_FALSE(optimal_assignments);
    error = std::get_if<TaskPlanner::TaskPlannerError>(
      &optimal_result);
    CHECK(*error == TaskPlanner::TaskPlannerError::limited_capacity);
  }

  WHEN("A loop request is impossible to fulfil due to low initial battery")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;

    rmf_traffic::agv::Plan::Start first_location{now, 9, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, 0.0},
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Loop::make(
        0,
        15,
        1000,
        "Loop1",
        now)
    };

    TaskPlanner task_planner(task_config);

    const auto greedy_result = task_planner.greedy_plan(
      now, initial_states, requests);
    const auto greedy_assignments = std::get_if<
      TaskPlanner::Assignments>(&greedy_result);
    REQUIRE_FALSE(greedy_assignments);
    auto error = std::get_if<
      TaskPlanner::TaskPlannerError>(&greedy_result);
    CHECK(*error == TaskPlanner::TaskPlannerError::low_battery);

    // Create new TaskPlanner to reset cache so that measured run times
    // remain independent of one another
    task_planner = TaskPlanner(task_config);
    const auto optimal_result = task_planner.optimal_plan(
      now, initial_states, requests, nullptr);
    const auto optimal_assignments = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    REQUIRE_FALSE(optimal_assignments);
    error = std::get_if<TaskPlanner::TaskPlannerError>(
      &optimal_result);
    CHECK(*error == TaskPlanner::TaskPlannerError::low_battery);
  }

  WHEN("Planning for one robot, one high priority and two low priority tasks")
  {

    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, 1.0},
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Delivery::make(
        0,
        "dispenser",
        3,
        "ingestor",
        {},
        "1",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        15,
        "dispenser",
        2,
        "ingestor",
        {},
        "2",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        7,
        "dispenser",
        9,
        "ingestor",
        {},
        "3",
        now + rmf_traffic::time::from_seconds(0))
    };

    TaskPlanner task_planner(task_config);

    auto start_time = std::chrono::steady_clock::now();
    const auto optimal_result = task_planner.optimal_plan(
      now, initial_states, requests, nullptr);
    const auto optimal_assignments_ptr = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    REQUIRE(optimal_assignments_ptr);
    const auto& optimal_assignments = *optimal_assignments_ptr;
    const double optimal_cost = task_planner.compute_cost(optimal_assignments);
    auto finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(optimal_assignments, now);

    if (display_solutions)
    {
      std::cout << "Optimal solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Optimal", optimal_assignments, optimal_cost);
    }

    // We expect request with task_id:3 to be at the back of the assignment queue
    CHECK(optimal_assignments.front().back().request()->id() == "3");

    THEN("When replanning with high priority for request with task_id:3")
    {
      requests[2] = rmf_task::requests::Delivery::make(
        7,
        "dispenser",
        9,
        "ingestor",
        {},
        "3",
        now + rmf_traffic::time::from_seconds(0),
        rmf_task::BinaryPriorityScheme::make_high_priority());
    }

    // Reset the planner cache
    task_planner = TaskPlanner(task_config);
    start_time = std::chrono::steady_clock::now();
    const auto new_optimal_result = task_planner.optimal_plan(
      now, initial_states, requests, nullptr);
    const auto new_optimal_assignments_ptr = std::get_if<
      TaskPlanner::Assignments>(&new_optimal_result);
    REQUIRE(new_optimal_assignments_ptr);
    const auto& new_optimal_assignments = *new_optimal_assignments_ptr;
    const double new_optimal_cost = task_planner.compute_cost(
      new_optimal_assignments);
    finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(new_optimal_assignments, now);

    if (display_solutions)
    {
      std::cout << "Optimal solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Optimal", new_optimal_assignments, new_optimal_cost);
    }

    // We expect request with task_id:3 to be at the front of the assignment queue
    CHECK(new_optimal_assignments.front().front().request()->id() == "3");
  }

  WHEN("Planning for one robot, three high priority tasks")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, 1.0},
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Delivery::make(
        0,
        "dispenser",
        3,
        "ingestor",
        {},
        "1",
        now + rmf_traffic::time::from_seconds(0),
        rmf_task::BinaryPriorityScheme::make_high_priority()),

      rmf_task::requests::Delivery::make(
        15,
        "dispenser",
        2,
        "ingestor",
        {},
        "2",
        now + rmf_traffic::time::from_seconds(0),
        rmf_task::BinaryPriorityScheme::make_high_priority()),

      rmf_task::requests::Delivery::make(
        7,
        "dispenser",
        9,
        "ingestor",
        {},
        "3",
        now + rmf_traffic::time::from_seconds(0),
        rmf_task::BinaryPriorityScheme::make_high_priority())
    };

    TaskPlanner task_planner(task_config);

    auto start_time = std::chrono::steady_clock::now();
    const auto optimal_result = task_planner.optimal_plan(
      now, initial_states, requests, nullptr);
    const auto optimal_assignments_ptr = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    REQUIRE(optimal_assignments_ptr);
    const auto& optimal_assignments = *optimal_assignments_ptr;
    const double optimal_cost = task_planner.compute_cost(optimal_assignments);
    auto finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(optimal_assignments, now);

    if (display_solutions)
    {
      std::cout << "Optimal solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Optimal", optimal_assignments, optimal_cost);
    }

    // We expect request with task_id:3 to be at the back of the assignment queue
    CHECK(optimal_assignments.front().back().request()->id() == "3");
  }

  WHEN("Planning for 1 robot, two high priority and two low priority tasks")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, 1.0},
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Delivery::make(
        0,
        "dispenser",
        3,
        "ingestor",
        {},
        "1",
        now + rmf_traffic::time::from_seconds(0),
        rmf_task::BinaryPriorityScheme::make_high_priority()),

      rmf_task::requests::Delivery::make(
        15,
        "dispenser",
        2,
        "ingestor",
        {},
        "2",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        7,
        "dispenser",
        9,
        "ingestor",
        {},
        "3",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        4,
        "dispenser",
        7,
        "ingestor",
        {},
        "4",
        now + rmf_traffic::time::from_seconds(0),
        rmf_task::BinaryPriorityScheme::make_high_priority())
    };

    TaskPlanner task_planner(task_config);

    auto start_time = std::chrono::steady_clock::now();
    const auto optimal_result = task_planner.optimal_plan(
      now, initial_states, requests, nullptr);
    const auto optimal_assignments_ptr = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    REQUIRE(optimal_assignments_ptr);
    const auto& optimal_assignments = *optimal_assignments_ptr;
    const double optimal_cost = task_planner.compute_cost(optimal_assignments);
    auto finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(optimal_assignments, now);

    if (display_solutions)
    {
      std::cout << "Optimal solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Optimal", optimal_assignments, optimal_cost);
    }

    // Based on the assigned priority and start time of the tasks, we expect
    // tasks to be allocated in the following order: 1->4->3->2
    const auto& assignments = optimal_assignments.front();
    std::unordered_map<std::string, std::size_t> index_map = {};
    for (std::size_t i = 0; i < assignments.size(); ++i)
      index_map.insert({assignments[i].request()->id(), i});
    CHECK(index_map["1"] < index_map["2"]);
    CHECK(index_map["1"] < index_map["3"]);
    CHECK(index_map["4"] < index_map["2"]);
    CHECK(index_map["4"] < index_map["3"]);
  }

  WHEN(
    "Planning for 1 robot and 2 tasks per time segment with one priority task per segment")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, 1.0},
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Delivery::make(
        0,
        "dispenser",
        3,
        "ingestor",
        {},
        "1",
        now + rmf_traffic::time::from_seconds(0),
        rmf_task::BinaryPriorityScheme::make_high_priority()),

      rmf_task::requests::Delivery::make(
        15,
        "dispenser",
        2,
        "ingestor",
        {},
        "2",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        7,
        "dispenser",
        9,
        "ingestor",
        {},
        "3",
        now + rmf_traffic::time::from_seconds(100000)),

      rmf_task::requests::Delivery::make(
        7,
        "dispenser",
        6,
        "ingestor",
        {},
        "4",
        now + rmf_traffic::time::from_seconds(100000),
        rmf_task::BinaryPriorityScheme::make_high_priority())
    };

    TaskPlanner task_planner(task_config);

    auto start_time = std::chrono::steady_clock::now();
    const auto optimal_result = task_planner.optimal_plan(
      now, initial_states, requests, nullptr);
    const auto optimal_assignments_ptr = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    REQUIRE(optimal_assignments_ptr);
    const auto& optimal_assignments = *optimal_assignments_ptr;
    const double optimal_cost = task_planner.compute_cost(optimal_assignments);
    auto finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(optimal_assignments, now);

    if (display_solutions)
    {
      std::cout << "Optimal solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Optimal", optimal_assignments, optimal_cost);
    }

    // Based on the assigned priority and start time of the tasks, we expect
    // tasks to be allocated in the following order: 1->2->4->3
    const auto& assignments = optimal_assignments.front();
    std::unordered_map<std::string, std::size_t> index_map = {};
    for (std::size_t i = 0; i < assignments.size(); ++i)
      index_map.insert({assignments[i].request()->id(), i});
    CHECK(index_map["1"] < index_map["2"]);
    CHECK(index_map["1"] < index_map["3"]);
    CHECK(index_map["1"] < index_map["4"]);
    CHECK(index_map["4"] > index_map["2"]);
    CHECK(index_map["4"] < index_map["3"]);
  }

  WHEN("Planning for 2 robots and 4 tasks")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};
    rmf_traffic::agv::Plan::Start second_location{now, 1, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, 1.0},
      rmf_task::agv::State{second_location, 1, 1.0}
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Delivery::make(
        0,
        "dispenser",
        3,
        "ingestor",
        {},
        "1",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        15,
        "dispenser",
        2,
        "ingestor",
        {},
        "2",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        7,
        "dispenser",
        9,
        "ingestor",
        {},
        "3",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        7,
        "dispenser",
        6,
        "ingestor",
        {},
        "4",
        now + rmf_traffic::time::from_seconds(0))
    };

    TaskPlanner task_planner(task_config);

    auto start_time = std::chrono::steady_clock::now();
    const auto optimal_result = task_planner.optimal_plan(
      now, initial_states, requests, nullptr);
    const auto optimal_assignments_ptr = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    REQUIRE(optimal_assignments_ptr);
    const auto& optimal_assignments = *optimal_assignments_ptr;
    const double optimal_cost = task_planner.compute_cost(optimal_assignments);
    auto finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(optimal_assignments, now);

    if (display_solutions)
    {
      std::cout << "Optimal solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Optimal", optimal_assignments, optimal_cost);
    }

    // We expect tasks 2 & 1 to be the first assignments of each agent respectively
    REQUIRE(optimal_assignments.size() == 2);
    const auto& agent_0_assignments = optimal_assignments[0];
    const auto& agent_1_assignments = optimal_assignments[1];
    CHECK(agent_0_assignments.front().request()->id() == "2");
    CHECK(agent_1_assignments.front().request()->id() == "1");

    THEN("When task 3 & 4 are assigned high priority")
    {
      std::vector<rmf_task::ConstRequestPtr> requests =
      {
        rmf_task::requests::Delivery::make(
          0,
          "dispenser",
          3,
          "ingestor",
          {},
          "1",
          now + rmf_traffic::time::from_seconds(0)),

        rmf_task::requests::Delivery::make(
          15,
          "dispenser",
          2,
          "ingestor",
          {},
          "2",
          now + rmf_traffic::time::from_seconds(0)),

        rmf_task::requests::Delivery::make(
          7,
          "dispenser",
          9,
          "ingestor",
          {},
          "3",
          now + rmf_traffic::time::from_seconds(0),
          rmf_task::BinaryPriorityScheme::make_high_priority()),

        rmf_task::requests::Delivery::make(
          7,
          "dispenser",
          6,
          "ingestor",
          {},
          "4",
          now + rmf_traffic::time::from_seconds(0),
          rmf_task::BinaryPriorityScheme::make_high_priority())
      };

      auto start_time = std::chrono::steady_clock::now();
      const auto optimal_result = task_planner.optimal_plan(
        now, initial_states, requests, nullptr);
      const auto optimal_assignments_ptr = std::get_if<
        TaskPlanner::Assignments>(&optimal_result);
      REQUIRE(optimal_assignments_ptr);
      const auto& optimal_assignments = *optimal_assignments_ptr;
      const double optimal_cost =
        task_planner.compute_cost(optimal_assignments);
      auto finish_time = std::chrono::steady_clock::now();
      CHECK_TIMES(optimal_assignments, now);

      if (display_solutions)
      {
        std::cout << "Optimal solution found in: "
                  << (finish_time - start_time).count() / 1e9 << std::endl;
        display_solution("Optimal", optimal_assignments, optimal_cost);
      }

      // We expect tasks high priority tasks 4 & 3 to be the first assignments of each agent respectively
      REQUIRE(optimal_assignments.size() == 2);
      const auto& agent_0_assignments = optimal_assignments[0];
      const auto& agent_1_assignments = optimal_assignments[1];
      CHECK(agent_0_assignments.front().request()->id() == "4");
      CHECK(agent_1_assignments.front().request()->id() == "3");
    }
  }

  WHEN("Planning for 3 robots and 4 tasks")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};
    rmf_traffic::agv::Plan::Start second_location{now, 1, default_orientation};
    rmf_traffic::agv::Plan::Start third_location{now, 5, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, 1.0},
      rmf_task::agv::State{second_location, 1, 1.0},
      rmf_task::agv::State{third_location, 5, 1.0}
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Delivery::make(
        0,
        "dispenser",
        3,
        "ingestor",
        {},
        "1",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        15,
        "dispenser",
        2,
        "ingestor",
        {},
        "2",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        7,
        "dispenser",
        9,
        "ingestor",
        {},
        "3",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        7,
        "dispenser",
        6,
        "ingestor",
        {},
        "4",
        now + rmf_traffic::time::from_seconds(0))
    };

    TaskPlanner task_planner(task_config);

    auto start_time = std::chrono::steady_clock::now();
    const auto optimal_result = task_planner.optimal_plan(
      now, initial_states, requests, nullptr);
    const auto optimal_assignments_ptr = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    REQUIRE(optimal_assignments_ptr);
    const auto& optimal_assignments = *optimal_assignments_ptr;
    const double optimal_cost = task_planner.compute_cost(optimal_assignments);
    auto finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(optimal_assignments, now);

    if (display_solutions)
    {
      std::cout << "Optimal solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Optimal", optimal_assignments, optimal_cost);
    }

    // We do not expect tasks 1, 2 & 3 to be the first assignment of each agent
    std::vector<std::string> first_assignments;
    for (const auto& agent : optimal_assignments)
    {
      if (!agent.empty())
        first_assignments.push_back(agent.front().request()->id());
    }
    std::size_t id_count = 0;
    for (const auto& id : first_assignments)
    {
      if ((id == "1") || (id == "2") || (id == "3"))
        id_count++;
    }
    CHECK_FALSE(id_count == 3);

    THEN("When tasks 1, 2 & 3 are assigned high priority")
    {
      std::vector<rmf_task::ConstRequestPtr> requests =
      {
        rmf_task::requests::Delivery::make(
          0,
          "dispenser",
          3,
          "ingestor",
          {},
          "1",
          now + rmf_traffic::time::from_seconds(0),
          rmf_task::BinaryPriorityScheme::make_high_priority()),

        rmf_task::requests::Delivery::make(
          15,
          "dispenser",
          2,
          "ingestor",
          {},
          "2",
          now + rmf_traffic::time::from_seconds(0),
          rmf_task::BinaryPriorityScheme::make_high_priority()),

        rmf_task::requests::Delivery::make(
          7,
          "dispenser",
          9,
          "ingestor",
          {},
          "3",
          now + rmf_traffic::time::from_seconds(0),
          rmf_task::BinaryPriorityScheme::make_high_priority()),

        rmf_task::requests::Delivery::make(
          7,
          "dispenser",
          6,
          "ingestor",
          {},
          "4",
          now + rmf_traffic::time::from_seconds(0))
      };

      auto start_time = std::chrono::steady_clock::now();
      const auto optimal_result = task_planner.optimal_plan(
        now, initial_states, requests, nullptr);
      const auto optimal_assignments_ptr = std::get_if<
        TaskPlanner::Assignments>(&optimal_result);
      REQUIRE(optimal_assignments_ptr);
      const auto& optimal_assignments = *optimal_assignments_ptr;
      const double optimal_cost =
        task_planner.compute_cost(optimal_assignments);
      auto finish_time = std::chrono::steady_clock::now();
      CHECK_TIMES(optimal_assignments, now);

      if (display_solutions)
      {
        std::cout << "Optimal solution found in: "
                  << (finish_time - start_time).count() / 1e9 << std::endl;
        display_solution("Optimal", optimal_assignments, optimal_cost);
      }

      // We expect tasks 1, 2 & 3 to be the first assignment of each agent
      std::vector<std::string> first_assignments;
      for (const auto& agent : optimal_assignments)
      {
        if (!agent.empty())
          first_assignments.push_back(agent.front().request()->id());
      }
      std::size_t id_count = 0;
      for (const auto& id : first_assignments)
      {
        if ((id == "1") || (id == "2") || (id == "3"))
          id_count++;
      }
      CHECK(id_count == 3);
    }
  }

  WHEN("Initial charge is low and battery recharge soc is constrained")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;
    const double initial_soc = 0.3;
    const double recharge_soc = 0.9;
    rmf_task::agv::Constraints new_constraints{0.2, recharge_soc,
      drain_battery};
    rmf_task::agv::TaskPlanner::Configuration new_task_config{
      parameters,
      new_constraints,
      cost_calculator};

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};
    rmf_traffic::agv::Plan::Start second_location{now, 2, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, initial_soc},
      rmf_task::agv::State{second_location, 2, initial_soc}
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Delivery::make(
        0,
        "dispenser",
        3,
        "ingestor",
        {},
        "1",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        15,
        "dispenser",
        2,
        "ingestor",
        {},
        "2",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        9,
        "dispenser",
        4,
        "ingestor",
        {},
        "3",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        8,
        "dispenser",
        11,
        "ingestor",
        {},
        "4",
        now + rmf_traffic::time::from_seconds(50000))
    };

    TaskPlanner task_planner(new_task_config);

    auto start_time = std::chrono::steady_clock::now();
    const auto greedy_result = task_planner.greedy_plan(
      now, initial_states, requests);
    const auto greedy_assignments = std::get_if<
      TaskPlanner::Assignments>(&greedy_result);
    REQUIRE(greedy_assignments);
    const double greedy_cost = task_planner.compute_cost(*greedy_assignments);
    auto finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(*greedy_assignments, now);

    if (display_solutions)
    {
      std::cout << "Greedy solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Greedy", *greedy_assignments, greedy_cost);
    }

    // Create new TaskPlanner to reset cache so that measured run times
    // remain independent of one another
    task_planner = TaskPlanner(new_task_config);
    start_time = std::chrono::steady_clock::now();
    const auto optimal_result = task_planner.optimal_plan(
      now, initial_states, requests, nullptr);
    const auto optimal_assignments = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    const double optimal_cost = task_planner.compute_cost(*optimal_assignments);
    finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(*optimal_assignments, now);

    if (display_solutions)
    {
      std::cout << "Optimal solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Optimal", *optimal_assignments, optimal_cost);
    }

    REQUIRE(optimal_cost <= greedy_cost);
    // Here we check that the battery was charged only up to recharge_soc
    for (const auto& agent : *optimal_assignments)
    {
      for (const auto& assignment : agent)
      {
        if (std::dynamic_pointer_cast<
            const rmf_task::requests::ChargeBattery::Description>(
            assignment.request()->description()))
        {
          CHECK(assignment.state().battery_soc() == recharge_soc);
        }
      }
    }

  }

  WHEN("start_time for requests are earlier than time_now")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, 1.0},
    };

    const auto start_time =
      std::chrono::steady_clock::time_point::min();

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Loop::make(
        0,
        15,
        1,
        "Loop1",
        start_time),
      rmf_task::requests::Loop::make(
        0,
        1,
        1,
        "Loop2",
        start_time),
      rmf_task::requests::Loop::make(
        3,
        4,
        1,
        "Loop3",
        start_time),
    };

    TaskPlanner task_planner(task_config);

    const auto optimal_result = task_planner.optimal_plan(
      now, initial_states, requests, nullptr);
    const auto optimal_assignments_ptr = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    REQUIRE(optimal_assignments_ptr);
    const auto& optimal_assignments = *optimal_assignments_ptr;
    const double optimal_cost = task_planner.compute_cost(optimal_assignments);
    auto finish_time = std::chrono::steady_clock::now();
    CHECK_TIMES(optimal_assignments, now);

    if (display_solutions)
    {
      std::cout << "Optimal solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Optimal", optimal_assignments, optimal_cost);
    }

  }

  WHEN("start_time for requests are earlier than time_now and low battery")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;
    const double initial_soc = 0.3;
    const double recharge_soc = 1.0;
    rmf_task::agv::Constraints new_constraints{0.2, recharge_soc,
      drain_battery};
    rmf_task::agv::TaskPlanner::Configuration new_task_config{
      parameters,
      new_constraints,
      cost_calculator};

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, initial_soc},
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Delivery::make(
        0,
        "dispenser",
        3,
        "ingestor",
        {},
        "1",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        15,
        "dispenser",
        2,
        "ingestor",
        {},
        "2",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        9,
        "dispenser",
        4,
        "ingestor",
        {},
        "3",
        now + rmf_traffic::time::from_seconds(0)),

      rmf_task::requests::Delivery::make(
        8,
        "dispenser",
        11,
        "ingestor",
        {},
        "4",
        now + rmf_traffic::time::from_seconds(50000))
    };

    TaskPlanner task_planner(new_task_config);

    auto start_time = std::chrono::steady_clock::now();
    const auto optimal_result = task_planner.optimal_plan(
      now, initial_states, requests, nullptr);
    auto finish_time = std::chrono::steady_clock::now();
    const auto optimal_assignments_ptr = std::get_if<
      TaskPlanner::Assignments>(&optimal_result);
    REQUIRE(optimal_assignments_ptr);
    const auto& optimal_assignments = *optimal_assignments_ptr;
    const double optimal_cost = task_planner.compute_cost(optimal_assignments);
    REQUIRE((!optimal_assignments.empty() && !optimal_assignments[0].empty()));
    const auto& first_assignment = optimal_assignments[0][0];
    CHECK(std::dynamic_pointer_cast<
        const rmf_task::requests::ChargeBattery::Description>(
        first_assignment.request()->description()));
    CHECK_TIMES(optimal_assignments, now);

    if (display_solutions)
    {
      std::cout << "Optimal solution found in: "
                << (finish_time - start_time).count() / 1e9 << std::endl;
      display_solution("Optimal", optimal_assignments, optimal_cost);
    }

  }

}
