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

#include <rmf_utils/catch.hpp>

#include <rmf_task_sequence/events/GoToPlace.hpp>

#include "../utils.hpp"

SCENARIO("Test GoToPlace")
{
  using GoToPlace = rmf_task_sequence::events::GoToPlace;

  const auto parameters = make_test_parameters();
  const auto constraints = make_test_constraints();
  const auto now = std::chrono::steady_clock::now();
  rmf_task::State initial_state;
  initial_state.waypoint(1)
  .orientation(0.0)
  .time(now)
  .dedicated_charging_waypoint(0)
  .battery_soc(1.0);

  const auto travel_estimator = rmf_task::TravelEstimator(*parameters);

  WHEN("Goal set is empty")
  {
    auto description = GoToPlace::Description::make_for_one_of({});
    THEN("make_model should return nullptr")
    {
      const auto model = description->make_model(initial_state, *parameters);
      CHECK(model == nullptr);
    }
  }

  WHEN("Not constrained to any map")
  {
    auto description = GoToPlace::Description::make_for_one_of({0, 8, 12});
    const auto model = description->make_model(initial_state, *parameters);
    const auto finish = model->estimate_finish(
      initial_state, now, *constraints, travel_estimator);

    REQUIRE(finish.has_value());
    CHECK(finish->finish_state().waypoint() == 0);
  }

  WHEN("Constrained to the same map")
  {
    auto description = GoToPlace::Description::make_for_one_of({0, 8, 12});
    description->prefer_same_map(true);
    const auto model = description->make_model(initial_state, *parameters);
    const auto finish = model->estimate_finish(
      initial_state, now, *constraints, travel_estimator);

    REQUIRE(finish.has_value());
    CHECK(finish->finish_state().waypoint() == 8);
  }
}
