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

#ifndef RMF_TASK__DETAIL__IMPL_ACTIVATOR_HPP
#define RMF_TASK__DETAIL__IMPL_ACTIVATOR_HPP

#include <rmf_task/Activator.hpp>

namespace rmf_task {

//==============================================================================
template<typename Description>
void Activator::add_activator(Activate<Description> activator)
{
  _add_activator(
    typeid(Description),
    [activator](
      std::function<State()> get_state,
      const ConstParametersPtr& parameters,
      const Task::ConstBookingPtr& booking,
      const Task::Description& description,
      std::optional<std::string> backup_state,
      std::function<void(Phase::ConstSnapshotPtr)> update,
      std::function<void(Task::Active::Backup)> checkpoint,
      std::function<void(Phase::ConstCompletedPtr)> phase_finished,
      std::function<void()> task_finished) -> Task::ActivePtr
    {
      return activator(
        std::move(get_state),
        parameters,
        booking,
        static_cast<const Description&>(description),
        std::move(backup_state),
        std::move(update),
        std::move(checkpoint),
        std::move(phase_finished),
        std::move(task_finished));
    });
}

} // namespace rmf_task

#endif // RMF_TASK__DETAIL__IMPL_ACTIVATOR_HPP
