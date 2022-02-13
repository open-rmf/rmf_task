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

#include "MockEvent.hpp"

namespace test_rmf_task {

//==============================================================================
MockEvent::MockEvent(
  uint64_t id_,
  std::string name_,
  std::string detail_,
  Status initial_status)
: _id(id_),
  _status(initial_status),
  _name(std::move(name_)),
  _detail(std::move(detail_))
{
  // Do nothing
}

//==============================================================================
uint64_t MockEvent::id() const
{
  return _id;
}

//==============================================================================
auto MockEvent::status() const -> Status
{
  return _status;
}

//==============================================================================
rmf_task::VersionedString::View MockEvent::name() const
{
  return _name.view();
}

//==============================================================================
rmf_task::VersionedString::View MockEvent::detail() const
{
  return _detail.view();
}

//==============================================================================
rmf_task::Log::View MockEvent::log() const
{
  return _log.view();
}

//==============================================================================
std::vector<rmf_task::Event::ConstStatePtr> MockEvent::dependencies() const
{
  return std::vector<rmf_task::Event::ConstStatePtr>(
    _dependencies.begin(), _dependencies.end());
}

} // namespace test_rmf_task
