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

#include <rmf_task/BackupFileManager.hpp>

#include "../mock/MockDelivery.hpp"

#include <rmf_utils/catch.hpp>

SCENARIO("Back up to file")
{
  rmf_task::Activator activator;
  activator.add_activator(test_rmf_task::MockDelivery::make_activator());

  const std::string backup_root_dir = "/tmp/rmf_task/test_backups";
  rmf_task::BackupFileManager backup(backup_root_dir);
  backup.clear_on_startup();

  auto robot_backup = backup.make_group("group")->make_robot("robot");
  CHECK_FALSE(robot_backup->read().has_value());

  using namespace std::chrono_literals;
  auto request = rmf_task::requests::Delivery::make(
    0, 1min, 1, 1min, "request_0", rmf_traffic::Time());

  rmf_task::Phase::ConstSnapshotPtr phase_snapshot;
  auto active_task = activator.activate(
    nullptr,
    nullptr,
    *request,
    [&phase_snapshot](auto s){ phase_snapshot = s; },
    [robot_backup](auto b){ robot_backup->write(b); },
    [](auto) {},
    []() {});

  REQUIRE(active_task);

  auto mock_active_task =
    std::dynamic_pointer_cast<test_rmf_task::MockDelivery::Active>(active_task);

  REQUIRE(mock_active_task);

  // ====== Advance the robot forward through some phases ======
  CHECK(phase_snapshot == nullptr);
  mock_active_task->start_next_phase(rmf_traffic::Time());
  CHECK(phase_snapshot == nullptr);

  mock_active_task->_active_phase->send_update();
  REQUIRE(phase_snapshot != nullptr);
  CHECK(phase_snapshot->tag()->id() == 0);

  mock_active_task->start_next_phase(rmf_traffic::Time());
  REQUIRE(phase_snapshot != nullptr);
  CHECK(phase_snapshot->tag()->id() == 0);
  mock_active_task->_active_phase->send_update();
  REQUIRE(phase_snapshot != nullptr);
  CHECK(phase_snapshot->tag()->id() == 1);

  // ====== Backup the task ======
  mock_active_task->issue_backup();

  // ====== Restore the task ======
  rmf_task::BackupFileManager restore(backup_root_dir);

  auto robot_restore = restore.make_group("group")->make_robot("robot");
  const auto restored_state = robot_restore->read();
  REQUIRE(restored_state.has_value());

  rmf_task::Phase::ConstSnapshotPtr restored_snapshot;
  auto restored_task = activator.restore(
    nullptr,
    nullptr,
    *request,
    restored_state.value(),
    [&restored_snapshot](auto s){ restored_snapshot = s; },
    [robot_restore](auto b){ robot_restore->write(b); },
    [](auto){},
    [](){});

  auto mock_restored_task =
    std::dynamic_pointer_cast<test_rmf_task::MockDelivery::Active>(
      restored_task);

  REQUIRE(mock_restored_task);
  REQUIRE(mock_restored_task->_restored_state.has_value());
  CHECK(mock_restored_task->_restored_state == restored_state.value());

  // When the task is restored, it should resume where mock_active_task left off
  // and it should issue a phase snapshot to reflect that
  REQUIRE(restored_snapshot);
  CHECK(restored_snapshot->tag()->id() == phase_snapshot->tag()->id());
  CHECK(restored_snapshot->tag()->name() == phase_snapshot->tag()->name());
  CHECK(restored_snapshot->tag()->detail() == phase_snapshot->tag()->detail());
}
