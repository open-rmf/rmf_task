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

#include <rmf_task/Condition.hpp>

namespace rmf_task {

namespace {
//==============================================================================
std::vector<ConstConditionPtr> snapshot_conditions(
  const std::vector<ConstConditionPtr>& queue)
{
  // NOTE(MXG): This implementation is using recursion. That should be fine
  // since I don't expect much depth in the trees of subconditions, but we may
  // want to revisit this and implement it as a queue instead if we ever find
  // a use-case with deep recursion.
  std::vector<ConstConditionPtr> output;
  output.reserve(queue.size());
  for (const auto& c : queue)
    output.push_back(Condition::Snapshot::make(*c));

  return output;
}
} // anonymous namespace

//==============================================================================
class Condition::Snapshot::Implementation
{
public:

  Status status;
  std::string name;
  std::string detail;
  Log::View log;
  std::vector<ConstConditionPtr> subconditions;

};

//==============================================================================
ConstConditionPtr Condition::Snapshot::make(const Condition& other)
{
  Snapshot output;
  output._pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{
      other.status(),
      other.name(),
      other.detail(),
      other.log(),
      snapshot_conditions(other.subconditions())
    });

  return std::make_shared<Snapshot>(std::move(output));
}

//==============================================================================
auto Condition::Snapshot::status() const -> Status
{
  return _pimpl->status;
}

//==============================================================================
std::string Condition::Snapshot::name() const
{
  return _pimpl->name;
}

//==============================================================================
std::string Condition::Snapshot::detail() const
{
  return _pimpl->detail;
}

//==============================================================================
Log::View Condition::Snapshot::log() const
{
  return _pimpl->log;
}

//==============================================================================
std::vector<ConstConditionPtr> Condition::Snapshot::subconditions() const
{
  return _pimpl->subconditions;
}

//==============================================================================
Condition::Snapshot::Snapshot()
{
  // Do nothing
}

} // namespace rmf_task
