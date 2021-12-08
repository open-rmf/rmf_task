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
#include <rmf_task_sequence/SimplePhase.hpp>

#include <rmf_utils/catch.hpp>

#include "../mock/MockActivity.hpp"

using namespace test_rmf_task_sequence;

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

  rmf_task_sequence::Task::Builder builder;


  rmf_task::Activator task_activator;
  const auto phase_activator =
    std::make_shared<rmf_task_sequence::Phase::Activator>();
  rmf_task_sequence::SimplePhase::add(*phase_activator, event_initializer);

}
