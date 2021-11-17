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

#include <rmf_task_sequence/events/Repeat.hpp>
#include <rmf_task_sequence/events/WaitFor.hpp>

#include "../utils.hpp"

using namespace std::chrono_literals;

SCENARIO("Test Repeat")
{
  using WaitFor = rmf_task_sequence::events::WaitFor;
  using Repeat = rmf_task_sequence::events::Repeat;

  const auto duration = 60s;
  const auto event = WaitFor::Description::make(duration);
  const std::size_t repetitions = 5;

  auto description = Repeat::Description::make(event, repetitions);

  const auto parameters = make_test_parameters();
  const auto constraints = make_test_constraints();
  const auto now = std::chrono::steady_clock::now();
  rmf_task::State initial_state;
  initial_state.waypoint(0)
  .orientation(0.0)
  .time(now)
  .dedicated_charging_waypoint(0)
  .battery_soc(1.0);

  WHEN("Testing getters")
  {
    CHECK(description->event() == event);
    CHECK(description->repetitions() == repetitions);
  }

  WHEN("Testing setters")
  {
    const auto new_event = WaitFor::Description::make(30s);
    description->event(new_event);
    // TODO(YV): More meaningful equality checks
    CHECK(description->event() != event);

    description->repetitions(10);
    CHECK(description->repetitions() == 10);
  }

  WHEN("Testing model and header")
  {
    // TODO(YV): Test model for cases where state is missing some parameters
    const auto model = description->make_model(
      initial_state,
      *parameters);
    const auto travel_estimator = rmf_task::TravelEstimator(*parameters);

    rmf_task::State expected_finish_state = initial_state;
    REQUIRE(expected_finish_state.time().has_value());
    const auto total_duration = repetitions * duration;
    expected_finish_state
    .time(initial_state.time().value() + total_duration);

    CHECK_MODEL(
      *model,
      initial_state,
      *constraints,
      travel_estimator,
      expected_finish_state);

    const auto header = description->generate_header(
      initial_state,
      *parameters);

    CHECK(!header.category().empty());
    CHECK(!header.detail().empty());
    CHECK(header.original_duration_estimate() == total_duration);
  }
}
