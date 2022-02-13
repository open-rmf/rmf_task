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

#include <rmf_utils/catch.hpp>

#include "../mock/MockDelivery.hpp"

SCENARIO("Activate fresh task")
{
  rmf_task::Activator activator;
  activator.add_activator(test_rmf_task::MockDelivery::make_activator());

  using namespace std::chrono_literals;
  auto request = rmf_task::requests::Delivery::make(
    0, 1min, 1, 1min, {{}}, "request_0", rmf_traffic::Time());

  rmf_task::Phase::ConstSnapshotPtr phase_snapshot;
  std::optional<rmf_task::Task::Active::Backup> backup;
  auto active = activator.activate(
    nullptr,
    nullptr,
    *request,
    [&phase_snapshot](auto s) { phase_snapshot = s; },
    [&backup](auto b) { backup = b; },
    [](auto) {},
    []() {});

  REQUIRE(active);

  auto mock_active =
    std::dynamic_pointer_cast<test_rmf_task::MockDelivery::Active>(active);

  REQUIRE(mock_active);

  CHECK(phase_snapshot == nullptr);

  mock_active->_active_phase->send_update();
  REQUIRE(phase_snapshot != nullptr);
  CHECK(phase_snapshot->tag()->id() == 0);

  mock_active->start_next_phase(rmf_traffic::Time());
  REQUIRE(phase_snapshot != nullptr);
  CHECK(phase_snapshot->tag()->id() == 0);
  mock_active->_active_phase->send_update();
  REQUIRE(phase_snapshot != nullptr);
  CHECK(phase_snapshot->tag()->id() == 1);

  // ====== Restoring a task ========
  CHECK_FALSE(backup.has_value());
  mock_active->issue_backup();
  REQUIRE(backup.has_value());
  CHECK(backup->sequence() == 0);
  CHECK(backup->state() == "1");

  auto restored = activator.restore(
    nullptr,
    nullptr,
    *request,
    backup->state(),
    [&phase_snapshot](auto s) { phase_snapshot = s; },
    [&backup](auto b) { backup = b; },
    [](auto) {},
    []() {});

  auto mock_restored =
    std::dynamic_pointer_cast<test_rmf_task::MockDelivery::Active>(restored);

  REQUIRE(mock_restored);
  REQUIRE(mock_restored->_restored_state.has_value());
  CHECK(mock_restored->_restored_state == backup->state());
}
