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

#ifndef RMF_TASK_SEQUENCE__DETAIL__IMPL_TASK_HPP
#define RMF_TASK_SEQUENCE__DETAIL__IMPL_TASK_HPP

#include <rmf_task_sequence/Task.hpp>

namespace rmf_task_sequence {

//==============================================================================
template<typename OtherDesc>
void Task::unfold(
  std::function<Description(const OtherDesc&)> unfold_description,
  rmf_task::Activator& task_activator,
  Phase::ConstActivatorPtr phase_activator,
  std::function<rmf_traffic::Time()> clock)
{
  auto sequence_activator =
    make_activator(std::move(phase_activator), std::move(clock));

  task_activator.add_activator<OtherDesc>(
    [
      unfold = std::move(unfold_description),
      sequence_activator = std::move(sequence_activator)
    ](
      const std::function<State()>& get_state,
      const ConstParametersPtr& parameters,
      const Task::ConstBookingPtr& booking,
      const OtherDesc& other_desc,
      std::optional<std::string> backup_state,
      std::function<void(Phase::ConstSnapshotPtr)> update,
      std::function<void(rmf_task::Task::Active::Backup)> checkpoint,
      std::function<void(rmf_task::Phase::ConstCompletedPtr)> phase_finished,
      std::function<void()> task_finished)
    {
      return sequence_activator(
        get_state,
        parameters,
        booking,
        unfold(other_desc),
        std::move(backup_state),
        std::move(update),
        std::move(checkpoint),
        std::move(phase_finished),
        std::move(task_finished));
    });
}

} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__DETAIL__IMPL_TASK_HPP
