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

#include <rmf_task/Event.hpp>

namespace rmf_task {

namespace {
//==============================================================================
std::vector<Event::ConstStatePtr> snapshot_dependencies(
  const std::vector<Event::ConstStatePtr>& queue)
{
  // NOTE(MXG): This implementation is using recursion. That should be fine
  // since I don't expect much depth in the trees of dependencies, but we may
  // want to revisit this and implement it as a queue instead if we ever find
  // a use-case with deep recursion.
  std::vector<Event::ConstStatePtr> output;
  output.reserve(queue.size());
  for (const auto& c : queue)
    output.push_back(Event::Snapshot::make(*c));

  return output;
}
} // anonymous namespace

//==============================================================================
Event::Status Event::sequence_status(Status earlier, Status later)
{
  // If either status "needs attention" then we elevate that status, in order
  // of criticality.
  for (const auto& s :
    {Status::Failed, Status::Error, Status::Blocked, Status::Uninitialized})
  {
    if (earlier == s || later == s)
      return s;
  }

  // If the earlier status is "finished" then we use the later status
  for (const auto& s :
    {Status::Completed, Status::Killed, Status::Canceled, Status::Skipped})
  {
    if (earlier == s)
      return later;
  }

  // If the earlier status is not finished, then we use the earlier status
  return earlier;
}

//==============================================================================
bool Event::State::finished() const
{
  switch (status())
  {
    case Status::Skipped:
    case Status::Canceled:
    case Status::Killed:
    case Status::Completed:
      return true;
    default:
      return false;
  }
}

//==============================================================================
class Event::Snapshot::Implementation
{
public:

  uint64_t id;
  Status status;
  VersionedString::View name;
  VersionedString::View detail;
  Log::View log;
  std::vector<ConstStatePtr> dependencies;

};

//==============================================================================
auto Event::Snapshot::make(const State& other) -> ConstSnapshotPtr
{
  Snapshot output;
  output._pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{
      other.id(),
      other.status(),
      other.name(),
      other.detail(),
      other.log(),
      snapshot_dependencies(other.dependencies())
    });

  return std::make_shared<Snapshot>(std::move(output));
}

//==============================================================================
uint64_t Event::Snapshot::id() const
{
  return _pimpl->id;
}

//==============================================================================
auto Event::Snapshot::status() const -> Status
{
  return _pimpl->status;
}

//==============================================================================
VersionedString::View Event::Snapshot::name() const
{
  return _pimpl->name;
}

//==============================================================================
VersionedString::View Event::Snapshot::detail() const
{
  return _pimpl->detail;
}

//==============================================================================
Log::View Event::Snapshot::log() const
{
  return _pimpl->log;
}

//==============================================================================
std::vector<Event::ConstStatePtr> Event::Snapshot::dependencies() const
{
  return _pimpl->dependencies;
}

//==============================================================================
Event::Snapshot::Snapshot()
{
  // Do nothing
}

//==============================================================================
class Event::AssignID::Implementation
{
public:
  mutable uint64_t next_id = 0;
};

//==============================================================================
Event::AssignIDPtr Event::AssignID::make()
{
  return std::make_shared<AssignID>();
}

//==============================================================================
Event::AssignID::AssignID()
: _pimpl(rmf_utils::make_unique_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
uint64_t Event::AssignID::assign() const
{
  return _pimpl->next_id++;
}

} // namespace rmf_task
