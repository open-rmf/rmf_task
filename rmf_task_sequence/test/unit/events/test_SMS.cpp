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

#include <rmf_task_sequence/events/SMS.hpp>

#include "../utils.hpp"

using namespace std::chrono_literals;

SCENARIO("Test SMS")
{
  using ContactCard = rmf_task_sequence::detail::ContactCard;
  using PhoneNumber = ContactCard::PhoneNumber;
  using SMS = rmf_task_sequence::events::SMS;

  const std::string message = "hello world";
  const auto number = PhoneNumber{42, 11311};
  const auto contact = ContactCard{
    "foo",
    "bar",
    "baz",
    number
  };

  const auto duration = 1s;
  auto description = SMS::Description::make(
    message,
    contact,
    duration);

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
    CHECK(description->message() == message);
    CHECK_CONTACT(description->contact(), "foo", "bar", "baz", number);
    CHECK(description->sms_duration_estimate() == duration);
  }

  WHEN("Testing setters")
  {
    const std::string new_message = "Hello World!!";
    description->message(new_message);
    CHECK(description->message() == new_message);

    const auto new_number = PhoneNumber{11311, 42};
    description->contact(
      ContactCard{"FOO", "BAR", "BAZ", new_number});
    CHECK_CONTACT(description->contact(), "FOO", "BAR", "BAZ", new_number);

    description->sms_duration_estimate(20s);
    CHECK(description->sms_duration_estimate() == 20s);
  }

  WHEN("Testing model")
  {
    // TODO(YV): Test model for cases where state is missing some parameters
    const auto model = description->make_model(
      initial_state,
      *parameters);
    const auto travel_estimator = rmf_task::TravelEstimator(*parameters);

    rmf_task::State expected_finish_state = initial_state;
    REQUIRE(expected_finish_state.time().has_value());
    expected_finish_state.time(initial_state.time().value() + duration);

    CHECK_MODEL(
      *model,
      initial_state,
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
