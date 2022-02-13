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

#include <rmf_task/Activator.hpp>

namespace rmf_task {

//==============================================================================
class Activator::Implementation
{
public:

  std::unordered_map<std::type_index, Activate<Task::Description>> activators;

};

//==============================================================================
Activator::Activator()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
Task::ActivePtr Activator::activate(
  const std::function<State()>& get_state,
  const ConstParametersPtr& parameters,
  const Request& request,
  std::function<void(Phase::ConstSnapshotPtr)> update,
  std::function<void(Task::Active::Backup)> checkpoint,
  std::function<void(Phase::ConstCompletedPtr)> phase_finished,
  std::function<void()> task_finished) const
{
  // TODO(MXG): Should we issue some kind of error/warning to distinguish
  // between a missing description versus a description that doesn't have a
  // corresponding activator? Same for the restore(~) function.
  if (!request.description())
    return nullptr;

  const auto& desc = *request.description();
  const auto& type = typeid(desc);
  const auto it = _pimpl->activators.find(type);
  if (it == _pimpl->activators.end())
    return nullptr;

  return it->second(
    get_state,
    parameters,
    request.booking(),
    *request.description(),
    std::nullopt,
    std::move(update),
    std::move(checkpoint),
    std::move(phase_finished),
    std::move(task_finished));
}

//==============================================================================
Task::ActivePtr Activator::restore(
  const std::function<State()>& get_state,
  const ConstParametersPtr& parameters,
  const Request& request,
  std::string backup_state,
  std::function<void(Phase::ConstSnapshotPtr)> update,
  std::function<void(Task::Active::Backup)> checkpoint,
  std::function<void(Phase::ConstCompletedPtr)> phase_finished,
  std::function<void()> task_finished) const
{
  if (!request.description())
    return nullptr;

  const auto& desc = *request.description();
  const auto& type = typeid(desc);
  const auto it = _pimpl->activators.find(type);
  if (it == _pimpl->activators.end())
    return nullptr;

  return it->second(
    get_state,
    parameters,
    request.booking(),
    *request.description(),
    std::move(backup_state),
    std::move(update),
    std::move(checkpoint),
    std::move(phase_finished),
    std::move(task_finished));
}

//==============================================================================
void Activator::_add_activator(
  std::type_index type,
  Activate<Task::Description> activator)
{
  _pimpl->activators.insert_or_assign(type, std::move(activator));
}

} // namespace rmf_task
