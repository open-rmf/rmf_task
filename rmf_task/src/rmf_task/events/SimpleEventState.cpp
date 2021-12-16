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

#include <rmf_task/events/SimpleEventState.hpp>

namespace rmf_task {
namespace events {

//==============================================================================
class SimpleEventState::Implementation
{
public:

  uint64_t id;
  Status status;
  VersionedString name;
  VersionedString detail;
  Log log;
  std::vector<Event::ConstStatePtr> dependencies;

};

//==============================================================================
std::shared_ptr<SimpleEventState> SimpleEventState::make(
  uint64_t id,
  std::string name,
  std::string detail,
  Event::Status initial_status,
  std::vector<Event::ConstStatePtr> dependencies,
  std::function<rmf_traffic::Time()> clock)
{
  SimpleEventState output;
  output._pimpl = rmf_utils::make_unique_impl<Implementation>(
    Implementation{
      id,
      initial_status,
      std::move(name),
      std::move(detail),
      Log(std::move(clock)),
      std::move(dependencies)
    });

  return std::make_shared<SimpleEventState>(std::move(output));
}

//==============================================================================
uint64_t SimpleEventState::id() const
{
  return _pimpl->id;
}

//==============================================================================
Event::Status SimpleEventState::status() const
{
  return _pimpl->status;
}

//==============================================================================
SimpleEventState& SimpleEventState::update_status(Event::Status new_status)
{
  _pimpl->status = new_status;
  return *this;
}

//==============================================================================
VersionedString::View SimpleEventState::name() const
{
  return _pimpl->name.view();
}

//==============================================================================
SimpleEventState& SimpleEventState::update_name(std::string new_name)
{
  _pimpl->name.update(std::move(new_name));
  return *this;
}

//==============================================================================
VersionedString::View SimpleEventState::detail() const
{
  return _pimpl->detail.view();
}

//==============================================================================
SimpleEventState& SimpleEventState::update_detail(std::string new_detail)
{
  _pimpl->detail.update(std::move(new_detail));
  return *this;
}

//==============================================================================
Log::View SimpleEventState::log() const
{
  return _pimpl->log.view();
}

//==============================================================================
Log& SimpleEventState::update_log()
{
  return _pimpl->log;
}

//==============================================================================
std::vector<Event::ConstStatePtr> SimpleEventState::dependencies() const
{
  return _pimpl->dependencies;
}

//==============================================================================
SimpleEventState& SimpleEventState::update_dependencies(
  std::vector<ConstStatePtr> new_dependencies)
{
  _pimpl->dependencies = new_dependencies;
  return *this;
}

//==============================================================================
SimpleEventState& SimpleEventState::add_dependency(ConstStatePtr new_dependency)
{
  _pimpl->dependencies.push_back(new_dependency);
  return *this;
}

//==============================================================================
SimpleEventState::SimpleEventState()
{
  // Do nothing
}

} // namespace events
} // namespace rmf_task
