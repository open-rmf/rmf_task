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

#include "MockCondition.hpp"

namespace test_rmf_task {

//==============================================================================
MockCondition::MockCondition(
  std::string name_,
  std::string detail_,
  Status initial_status)
: _status(initial_status),
  _name(std::move(name_)),
  _detail(std::move(detail_))
{
  // Do nothing
}

//==============================================================================
auto MockCondition::status() const -> Status
{
  return _status;
}

//==============================================================================
std::string MockCondition::name() const
{
  return _name;
}

//==============================================================================
std::string MockCondition::detail() const
{
  return _detail;
}

//==============================================================================
rmf_task::Log::View MockCondition::log() const
{
  return _log.view();
}

//==============================================================================
std::vector<rmf_task::ConstConditionPtr> MockCondition::subconditions() const
{
  return std::vector<rmf_task::ConstConditionPtr>(
    _subconditions.begin(), _subconditions.end());
}

} // namespace test_rmf_task
