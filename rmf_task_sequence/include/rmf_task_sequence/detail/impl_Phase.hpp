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

#ifndef RMF_TASK_SEQUENCE__DETAIL__IMPL_PHASE_HPP
#define RMF_TASK_SEQUENCE__DETAIL__IMPL_PHASE_HPP

#include <rmf_task_sequence/Phase.hpp>

namespace rmf_task_sequence {

//==============================================================================
template<typename Desc>
void Phase::Activator::add_activator(Activate<Desc> activator)
{
  _add_activator(
    typeid(Desc),
    [activator](
      const std::function<State()>& get_state,
      const ConstParametersPtr& parameters,
      ConstTagPtr tag,
      const Phase::Description& description,
      std::optional<std::string> backup_state,
      std::function<void(rmf_task::Phase::ConstSnapshotPtr)> phase_update,
      std::function<void(Active::Backup)> phase_checkpoint,
      std::function<void()> phase_finished)
    {
      return activator(
        get_state,
        parameters,
        std::move(tag),
        static_cast<const Desc&>(description),
        std::move(backup_state),
        std::move(phase_update),
        std::move(phase_checkpoint),
        std::move(phase_finished));
    });
}

} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__DETAIL__IMPL_PHASE_HPP
