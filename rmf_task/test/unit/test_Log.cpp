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

#include <rmf_task/Log.hpp>

SCENARIO("Writing and reading logs")
{
  rmf_task::Log log;
  rmf_task::Log::Reader reader;

  std::size_t expected_count = 0;
  std::size_t count = 0;
  for (const auto& entry : reader.read(log.view()))
  {
    CHECK(entry.seq() == count);
    ++count;
  }

  CHECK(count == expected_count);

  log.info("Test text");
  log.info("More test text");
  log.warn("Some warning text");

  expected_count = 3;
  count = 0;
  for (const auto& entry : reader.read(log.view()))
  {
    CHECK(entry.seq() == count);
    ++count;
  }

  CHECK(count == expected_count);

  expected_count = 0;
  count = 0;
  for (const auto& entry : reader.read(log.view()))
  {
    (void)(entry);
    ++count;
  }

  CHECK(count == expected_count);
}
