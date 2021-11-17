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

#include <rmf_task_sequence/detail/ContactCard.hpp>

SCENARIO("Test ContactCard")
{
  using ContactCard = rmf_task_sequence::detail::ContactCard;
  using PhoneNumber = ContactCard::PhoneNumber;

  auto contact = ContactCard{
    "foo",
    "bar",
    "baz",
    PhoneNumber{42, 11311}
  };
  CHECK(contact.name() == "foo");
  CHECK(contact.address() == "bar");
  CHECK(contact.email() == "baz");
  CHECK(contact.number().country_code == 42);
  CHECK(contact.number().number == 11311);

  WHEN("Setting name")
  {
    contact.name("FOO");
    CHECK(contact.name() == "FOO");
  }

  WHEN("Setting address")
  {
    contact.address("BAR");
    CHECK(contact.address() == "BAR");
  }

  WHEN("Setting email")
  {
    contact.email("BAZ");
    CHECK(contact.email() == "BAZ");
  }

  WHEN("Setting number")
  {
    contact.number(PhoneNumber{11311, 42});
    CHECK(contact.number().country_code == 11311);
    CHECK(contact.number().number == 42);
  }

}
