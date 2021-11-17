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

#include <rmf_task_sequence/events/GoToPlace.hpp>

#include "../utils.hpp"

using namespace std::chrono_literals;

SCENARIO("Test GoToPlace")
{
  using GoToPlace = rmf_task_sequence::events::GoToPlace;
  using Goal = GoToPlace::Goal;

  const auto goal = Goal{1};
  auto description = GoToPlace::Description::make(goal);

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
    CHECK(description->destination().waypoint() == goal.waypoint());
    CHECK_FALSE(description->destination().orientation());
    CHECK(description->destination_name(*parameters) == "#1");
  }

  WHEN("Testing setters")
  {
    description->destination(Goal{2, 1.57});
    CHECK(description->destination().waypoint() == 2);
    REQUIRE(description->destination().orientation());
    CHECK(abs(
      *description->destination().orientation() - 1.57) < 1e-3);
    CHECK(description->destination_name(*parameters) == "#2");
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
    const auto duration_opt = estimate_travel_duration(
      parameters->planner(),
      initial_state,
      goal);
    REQUIRE(duration_opt.has_value());
    expected_finish_state.waypoint(goal.waypoint())
    .time(initial_state.time().value() + duration_opt.value());

    if (goal.orientation())
      expected_finish_state.orientation(*goal.orientation());
    else
      expected_finish_state.erase<rmf_task::State::CurrentOrientation>();

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
    CHECK(header.original_duration_estimate() == duration_opt.value());
  }
}
