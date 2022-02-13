/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <rmf_task_sequence/events/PerformAction.hpp>

#include "../utils.hpp"

using namespace std::chrono_literals;

SCENARIO("Test PerformAction")
{
  using PerformAction = rmf_task_sequence::events::PerformAction;
  const auto duration = 10s;
  const std::string category = "test_action";
  const nlohmann::json desc;
  const rmf_traffic::agv::Planner::Goal finish_location{1};

  auto description = PerformAction::Description::make(
    category,
    desc,
    duration,
    false,
    finish_location);

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
    CHECK(description->category() == category);
    CHECK(description->description() == desc);
    CHECK(description->action_duration_estimate() == duration);
    CHECK(description->use_tool_sink() == false);
    REQUIRE(description->expected_finish_location().has_value());
    CHECK(description->expected_finish_location().value().waypoint() == 1);
  }

  WHEN("Testing setters")
  {
    description->category("new_category");
    CHECK(description->category() == "new_category");
    nlohmann::json new_desc = {{"key", "value"}};
    description->description(new_desc);
    CHECK(description->description() == new_desc);
    description->action_duration_estimate(20s);
    CHECK(description->action_duration_estimate() == 20s);
    description->use_tool_sink(true);
    CHECK(description->use_tool_sink() == true);
    description->expected_finish_location(std::nullopt);
    CHECK_FALSE(description->expected_finish_location().has_value());
  }

  WHEN("Testing model")
  {
    // TODO(YV): Test model for cases where state is missing some parameters
    const auto model = description->make_model(
      initial_state,
      *parameters);
    const auto travel_estimator = rmf_task::TravelEstimator(*parameters);

    rmf_task::State expected_finish_state = initial_state;
    REQUIRE(initial_state.time().has_value());
    expected_finish_state.time(initial_state.time().value() + duration)
    .waypoint(1);

    CHECK_MODEL(
      *model,
      initial_state,
      now,
      *constraints,
      travel_estimator,
      expected_finish_state);
  }

  WHEN("Testing header")
  {
    const auto header = description->generate_header(
      initial_state,
      *parameters);

    CHECK(!header.category().empty());
    CHECK(!header.detail().empty());
    CHECK(header.original_duration_estimate() == duration);
  }
}
