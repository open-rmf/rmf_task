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

#include <rmf_task_sequence/Phase.hpp>

namespace rmf_task_sequence {

//==============================================================================
class Phase::Activator::Implementation
{
public:

  std::unordered_map<std::type_index, Activate<Phase::Description>> activators;

};

//==============================================================================
Phase::Activator::Activator()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
Phase::ActivePtr Phase::Activator::activate(
  const std::function<State()>& get_state,
  const ConstParametersPtr& parameters,
  ConstTagPtr tag,
  const Description& description,
  std::optional<nlohmann::json> backup_state,
  std::function<void(rmf_task::Phase::ConstSnapshotPtr)> update,
  std::function<void(Active::Backup)> checkpoint,
  std::function<void()> finished) const
{
  const auto& type = typeid(description);
  const auto it = _pimpl->activators.find(type);
  if (it == _pimpl->activators.end())
    return nullptr;

  return it->second(
    get_state,
    parameters,
    std::move(tag),
    description,
    std::move(backup_state),
    std::move(update),
    std::move(checkpoint),
    std::move(finished));
}

//==============================================================================
void Phase::Activator::_add_activator(
  std::type_index type, Activate<Phase::Description> activator)
{
  _pimpl->activators.insert_or_assign(type, std::move(activator));
}

} // namespace rmf_task_sequence
