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

#include <rmf_task/Activator.hpp>

#include <rmf_task_sequence/Event.hpp>
#include <rmf_task_sequence/events/Bundle.hpp>
#include <rmf_task_sequence/Task.hpp>
#include <rmf_task_sequence/phases/SimplePhase.hpp>
#include <rmf_task_sequence/events/Bundle.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/MechanicalSystem.hpp>
#include <rmf_battery/agv/PowerSystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>

#include <rmf_utils/catch.hpp>

#include "../mock/MockActivity.hpp"

#include <iostream>

using namespace test_rmf_task_sequence;

//==============================================================================
rmf_task_sequence::Phase::ConstDescriptionPtr make_sequence_for_phase(
  const std::vector<std::shared_ptr<MockActivity::Controller>>& ctrls)
{
  std::vector<rmf_task_sequence::Event::ConstDescriptionPtr> descs;
  for (const auto& c : ctrls)
    descs.push_back(std::make_shared<MockActivity::Description>(c));

  auto sequence =
    std::make_shared<rmf_task_sequence::events::Bundle::Description>(
    descs, rmf_task_sequence::events::Bundle::Type::Sequence);

  return rmf_task_sequence::phases::SimplePhase::Description::make(sequence);
}

//==============================================================================
rmf_task_sequence::Phase::ConstDescriptionPtr make_phase(
  std::shared_ptr<MockActivity::Controller> ctrl)
{
  return rmf_task_sequence::phases::SimplePhase::Description::make(
    std::make_shared<MockActivity::Description>(std::move(ctrl)));
}

//==============================================================================
void check_active(
  const std::vector<std::shared_ptr<MockActivity::Controller>>& ctrls)
{
  for (const auto& c : ctrls)
    CHECK(c->active);
}

//==============================================================================
void check_status(
  const std::vector<std::shared_ptr<MockActivity::Controller>>& ctrls,
  rmf_task::Event::Status status)
{
  for (const auto& c : ctrls)
  {
    REQUIRE(c->active);
    CHECK(c->active->state()->status() == status);
  }
}

//==============================================================================
void check_statuses(
  const std::vector<rmf_task::Event::ConstStatePtr>& states,
  const std::vector<rmf_task::Event::Status>& statuses)
{
  REQUIRE(states.size() == statuses.size());
  for (std::size_t i = 0; i < states.size(); ++i)
    CHECK(states[i]->status() == statuses[i]);
}

//==============================================================================
void check_inactive(
  const std::vector<std::shared_ptr<MockActivity::Controller>>& ctrls)
{
  for (const auto& c : ctrls)
    CHECK_FALSE(c->active);
}

//==============================================================================
SCENARIO("Test Event Sequences")
{
  const auto event_initializer =
    std::make_shared<rmf_task_sequence::Event::Initializer>();

  // Add the Bundle event to the initializer
  rmf_task_sequence::events::Bundle::add(event_initializer);

  MockActivity::add(event_initializer);
  auto ctrl_1_0 = std::make_shared<MockActivity::Controller>();
  auto ctrl_1_1 = std::make_shared<MockActivity::Controller>();
  auto ctrl_1_2 = std::make_shared<MockActivity::Controller>();
  auto ctrl_1_3 = std::make_shared<MockActivity::Controller>();

  auto ctrl_2_0 = std::make_shared<MockActivity::Controller>();

  auto ctrl_3_0 = std::make_shared<MockActivity::Controller>();
  auto ctrl_3_1 = std::make_shared<MockActivity::Controller>();

  auto cancel_ctrl_0 = std::make_shared<MockActivity::Controller>();
  auto cancel_ctrl_1 = std::make_shared<MockActivity::Controller>();

  const auto phase_activator =
    std::make_shared<rmf_task_sequence::Phase::Activator>();
  rmf_task_sequence::phases::SimplePhase::add(
    *phase_activator, event_initializer);

  rmf_task::Activator task_activator;
  rmf_task_sequence::Task::add(
    task_activator,
    phase_activator,
    []() { return std::chrono::steady_clock::now(); });

  rmf_task_sequence::Task::Builder builder;
  builder.add_phase(
    make_sequence_for_phase({ctrl_1_0, ctrl_1_1, ctrl_1_2, ctrl_1_3}),
    {make_phase(cancel_ctrl_0), make_phase(cancel_ctrl_1)});

  builder.add_phase(make_sequence_for_phase({ctrl_2_0}), {});

  builder.add_phase(
    make_sequence_for_phase({ctrl_3_0, ctrl_3_1}),
    {make_phase(cancel_ctrl_0)});

  auto battery_system_optional =
    rmf_battery::agv::BatterySystem::make(24.0, 40.0, 8.8);
  auto mechanical_system_optional =
    rmf_battery::agv::MechanicalSystem::make(70.0, 40.0, 0.22);
  auto power_system_optional =
    rmf_battery::agv::PowerSystem::make(20.0);

  auto motion_sink =
    std::make_shared<rmf_battery::agv::SimpleMotionPowerSink>(
    *battery_system_optional, *mechanical_system_optional);
  auto device_sink =
    std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system_optional, *power_system_optional);

  const auto params = std::make_shared<rmf_task::Parameters>(
    nullptr,
    rmf_battery::agv::BatterySystem::make(1.0, 1.0, 1.0).value(),
    motion_sink,
    device_sink);

  rmf_task::Phase::ConstSnapshotPtr last_snapshot;
  std::optional<rmf_task::Task::Active::Backup> last_backup;
  rmf_task::Phase::ConstCompletedPtr last_finished_phase;
  std::size_t task_finished_counter = 0;

  auto task = task_activator.activate(
    []() { return rmf_task::State().time(std::chrono::steady_clock::now()); },
    params,
    rmf_task::Request(
      "mock_request_01",
      std::chrono::steady_clock::now(),
      nullptr,
      builder.build("Mock Task", "Mocking a task")),
    [&last_snapshot](rmf_task::Phase::ConstSnapshotPtr snapshot)
    {
      last_snapshot = std::move(snapshot);
    },
    [&last_backup](rmf_task::Task::Active::Backup backup)
    {
      last_backup = std::move(backup);
    },
    [&last_finished_phase](rmf_task::Phase::ConstCompletedPtr finished_phase)
    {
      last_finished_phase = std::move(finished_phase);
    },
    [&task_finished_counter]()
    {
      ++task_finished_counter;
    }
  );

  WHEN("Run through whole task")
  {
    check_active({ctrl_1_0});
    check_inactive(
      {ctrl_1_1, ctrl_1_2, ctrl_1_3, ctrl_2_0, ctrl_3_0, ctrl_3_1});
    REQUIRE(last_snapshot);
    CHECK(last_snapshot->tag()->id() == 1);
    CHECK(last_snapshot->final_event()->dependencies().size() == 4);
    check_statuses(
      {last_snapshot->final_event()},
      {rmf_task::Event::Status::Underway});
    check_statuses(
      last_snapshot->final_event()->dependencies(),
      {
        rmf_task::Event::Status::Underway,
        rmf_task::Event::Status::Standby,
        rmf_task::Event::Status::Standby,
        rmf_task::Event::Status::Standby
      });

    CHECK(task->completed_phases().size() == 0);
    CHECK(task->pending_phases().size() == 2);
    last_snapshot = nullptr;

    ctrl_1_0->active->complete();
    check_status({ctrl_1_0}, rmf_task::Event::Status::Completed);
    check_active({ctrl_1_1});
    check_inactive({ctrl_1_2, ctrl_1_3, ctrl_2_0, ctrl_3_0, ctrl_3_1});
    REQUIRE(last_snapshot);
    CHECK(last_snapshot->tag()->id() == 1);
    CHECK(last_snapshot->final_event()->dependencies().size() == 4);
    check_statuses(
      {last_snapshot->final_event()},
      {rmf_task::Event::Status::Underway});
    check_statuses(
      last_snapshot->final_event()->dependencies(),
      {
        rmf_task::Event::Status::Completed,
        rmf_task::Event::Status::Underway,
        rmf_task::Event::Status::Standby,
        rmf_task::Event::Status::Standby
      });

    CHECK(last_backup.has_value());

    CHECK(task->completed_phases().size() == 0);
    CHECK(task->pending_phases().size() == 2);
    last_snapshot = nullptr;
    last_backup = std::nullopt;

    ctrl_1_1->active->complete();
    check_status({ctrl_1_0, ctrl_1_1}, rmf_task::Event::Status::Completed);
    check_active({ctrl_1_2});
    check_inactive({ctrl_1_3, ctrl_2_0, ctrl_3_0, ctrl_3_1});
    REQUIRE(last_snapshot);
    CHECK(last_snapshot->tag()->id() == 1);
    check_statuses(
      {last_snapshot->final_event()},
      {rmf_task::Event::Status::Underway});
    check_statuses(
      last_snapshot->final_event()->dependencies(),
      {
        rmf_task::Event::Status::Completed,
        rmf_task::Event::Status::Completed,
        rmf_task::Event::Status::Underway,
        rmf_task::Event::Status::Standby
      });

    CHECK(last_backup.has_value());

    CHECK(task->completed_phases().size() == 0);
    CHECK(task->pending_phases().size() == 2);
    last_snapshot = nullptr;
    last_backup = std::nullopt;

    ctrl_1_2->active->complete();
    check_status(
      {ctrl_1_0, ctrl_1_1, ctrl_1_2},
      rmf_task::Event::Status::Completed);
    check_active({ctrl_1_3});
    check_inactive({ctrl_2_0, ctrl_3_0, ctrl_3_1});
    REQUIRE(last_snapshot);
    CHECK(last_snapshot->tag()->id() == 1);
    check_statuses(
      {last_snapshot->final_event()},
      {rmf_task::Event::Status::Underway});
    check_statuses(
      last_snapshot->final_event()->dependencies(),
      {
        rmf_task::Event::Status::Completed,
        rmf_task::Event::Status::Completed,
        rmf_task::Event::Status::Completed,
        rmf_task::Event::Status::Underway
      });

    CHECK(last_backup.has_value());

    CHECK(task->completed_phases().size() == 0);
    CHECK(task->pending_phases().size() == 2);
    last_snapshot = nullptr;
    last_backup = std::nullopt;

    ctrl_1_3->active->signals.checkpoint();
    CHECK(last_backup.has_value());

    CHECK(!last_finished_phase);

    ctrl_1_3->active->complete();
    check_status(
      {ctrl_1_0, ctrl_1_1, ctrl_1_2, ctrl_1_3},
      rmf_task::Event::Status::Completed);
    check_active({ctrl_2_0});
    check_inactive({ctrl_3_0, ctrl_3_1});
    REQUIRE(last_snapshot);
    CHECK(last_snapshot->tag()->id() == 2);
    check_statuses(
      {last_snapshot->final_event()},
      {rmf_task::Event::Status::Underway});
    check_statuses(
      last_snapshot->final_event()->dependencies(),
      {
        rmf_task::Event::Status::Underway
      });

    REQUIRE(last_finished_phase);
    CHECK(last_finished_phase->snapshot()->tag()->id() == 1);
    check_statuses(
      {last_finished_phase->snapshot()->final_event()},
      {rmf_task::Event::Status::Completed});
    check_statuses(
      last_finished_phase->snapshot()->final_event()->dependencies(),
      {
        rmf_task::Event::Status::Completed,
        rmf_task::Event::Status::Completed,
        rmf_task::Event::Status::Completed,
        rmf_task::Event::Status::Completed
      });
    check_statuses(
      {last_finished_phase->snapshot()->final_event()},
      {rmf_task::Event::Status::Completed});
    CHECK(last_backup.has_value());

    CHECK(task->completed_phases().size() == 1);
    CHECK(task->pending_phases().size() == 1);
    last_snapshot = nullptr;
    last_finished_phase = nullptr;
    last_backup = std::nullopt;

    ctrl_2_0->active->complete();
    check_status(
      {ctrl_1_0, ctrl_1_1, ctrl_1_2, ctrl_1_3, ctrl_2_0},
      rmf_task::Event::Status::Completed);
    check_active({ctrl_3_0});
    check_inactive({ctrl_3_1});
    check_statuses(
      {last_snapshot->final_event()},
      {rmf_task::Event::Status::Underway});
    check_statuses(
      last_snapshot->final_event()->dependencies(),
      {
        rmf_task::Event::Status::Underway,
        rmf_task::Event::Status::Standby
      });

    REQUIRE(last_finished_phase);
    CHECK(last_finished_phase->snapshot()->tag()->id() == 2);
    check_statuses(
      {last_finished_phase->snapshot()->final_event()},
      {rmf_task::Event::Status::Completed});
    check_statuses(
      last_finished_phase->snapshot()->final_event()->dependencies(),
      {
        rmf_task::Event::Status::Completed
      });

    CHECK(task->completed_phases().size() == 2);
    CHECK(task->pending_phases().size() == 0);
    last_snapshot = nullptr;
    last_finished_phase = nullptr;
    last_backup = std::nullopt;

    ctrl_3_0->active->complete();
    check_status(
      {ctrl_1_0, ctrl_1_1, ctrl_1_2, ctrl_1_3, ctrl_2_0, ctrl_3_0},
      rmf_task::Event::Status::Completed);
    check_active({ctrl_3_1});
    check_statuses(
      {last_snapshot->final_event()},
      {rmf_task::Event::Status::Underway});
    check_statuses(
      last_snapshot->final_event()->dependencies(),
      {
        rmf_task::Event::Status::Completed,
        rmf_task::Event::Status::Underway
      });

    CHECK(!last_finished_phase);
    CHECK(last_backup.has_value());

    CHECK(task->completed_phases().size() == 2);
    CHECK(task->pending_phases().size() == 0);
    last_snapshot = nullptr;
    last_finished_phase = nullptr;
    last_backup = std::nullopt;

    ctrl_3_1->active->complete();
    check_status(
      {ctrl_1_0, ctrl_1_1, ctrl_1_2, ctrl_1_3, ctrl_2_0, ctrl_3_0, ctrl_3_1},
      rmf_task::Event::Status::Completed);

    CHECK(!last_snapshot);
    REQUIRE(last_finished_phase);
    CHECK(last_finished_phase->snapshot()->tag()->id() == 3);
    check_statuses(
      {last_finished_phase->snapshot()->final_event()},
      {rmf_task::Event::Status::Completed});
    check_statuses(
      last_finished_phase->snapshot()->final_event()->dependencies(),
      {
        rmf_task::Event::Status::Completed,
        rmf_task::Event::Status::Completed
      });

    CHECK(task->completed_phases().size() == 3);
    CHECK(task->pending_phases().size() == 0);
  }
}
