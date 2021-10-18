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
std::vector<Event::ConstActivePtr> snapshot_dependencies(
  const std::vector<Event::ConstActivePtr>& queue)
{
  // NOTE(MXG): This implementation is using recursion. That should be fine
  // since I don't expect much depth in the trees of dependencies, but we may
  // want to revisit this and implement it as a queue instead if we ever find
  // a use-case with deep recursion.
  std::vector<Event::ConstActivePtr> output;
  output.reserve(queue.size());
  for (const auto& c : queue)
    output.push_back(Event::Snapshot::make(*c));

  return output;
}
} // anonymous namespace

//==============================================================================
class Event::Snapshot::Implementation
{
public:

  Status status;
  VersionedString::View name;
  VersionedString::View detail;
  Log::View log;
  std::vector<ConstActivePtr> dependencies;

};

//==============================================================================
auto Event::Snapshot::make(const Active& other) -> ConstSnapshotPtr
{
  Snapshot output;
  output._pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{
      other.status(),
      other.name(),
      other.detail(),
      other.log(),
      snapshot_dependencies(other.dependencies())
    });

  return std::make_shared<Snapshot>(std::move(output));
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
std::vector<Event::ConstActivePtr> Event::Snapshot::dependencies() const
{
  return _pimpl->dependencies;
}

//==============================================================================
Event::Snapshot::Snapshot()
{
  // Do nothing
}

} // namespace rmf_task
