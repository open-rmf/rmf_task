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

#include <rmf_task_sequence/Event.hpp>

namespace rmf_task_sequence {

//==============================================================================
class Event::Initializer::Implementation
{
public:

  std::unordered_map<std::type_index, Initialize<Description>> initializers;
  std::unordered_map<std::type_index, Restore<Description>> restorers;

};

//==============================================================================
Event::Initializer::Initializer()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
Event::StandbyPtr Event::Initializer::initialize(
  const AssignIDPtr& id,
  const std::function<rmf_task::State()>& get_state,
  const ConstParametersPtr& parameters,
  const Event::Description& description,
  std::function<void()> update) const
{
  const auto& type = typeid(description);
  const auto it = _pimpl->initializers.find(type);
  if (it == _pimpl->initializers.end())
    return nullptr;

  return it->second(
    id,
    get_state,
    parameters,
    description,
    std::move(update));
}

//==============================================================================
Event::ActivePtr Event::Initializer::restore(
  const AssignIDPtr& id,
  const std::function<rmf_task::State()>& get_state,
  const ConstParametersPtr& parameters,
  const Event::Description& description,
  const nlohmann::json& backup,
  std::function<void()> update,
  std::function<void()> checkpoint,
  std::function<void()> finished) const
{
  const auto& type = typeid(description);
  const auto it = _pimpl->restorers.find(type);
  if (it == _pimpl->restorers.end())
    return nullptr;

  return it->second(
    id,
    get_state,
    parameters,
    description,
    backup,
    std::move(update),
    std::move(checkpoint),
    std::move(finished));
}

//==============================================================================
void Event::Initializer::_add(
  std::type_index type,
  Initialize<Event::Description> initializer,
  Restore<Event::Description> restorer)
{
  _pimpl->initializers.insert_or_assign(type, std::move(initializer));
  _pimpl->restorers.insert_or_assign(type, std::move(restorer));
}

} // namespace rmf_task_sequence
