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

#include "MockDelivery.hpp"
#include <stdexcept>

namespace test_rmf_task {

//==============================================================================
auto MockDelivery::make_activator() -> Activator
{
  return [](
    std::function<rmf_task::State()> get_state,
    const rmf_task::ConstParametersPtr& parameters,
    const Task::ConstBookingPtr& booking,
    const Description& description,
    std::optional<std::string> backup_state,
    std::function<void(Phase::ConstSnapshotPtr)> update,
    std::function<void(Task::Active::Backup)> checkpoint,
    std::function<void(Phase::ConstCompletedPtr)> phase_finished,
    std::function<void()> task_finished) -> Task::ActivePtr
    {
      return std::shared_ptr<Active>(
        new Active(
          description,
          std::move(backup_state),
          std::move(get_state),
          parameters,
          booking,
          std::move(update),
          std::move(checkpoint),
          std::move(phase_finished),
          std::move(task_finished)));
    };
}

auto MockDelivery::Active::backup() const -> Backup
{
  std::stringstream ss;
  ss << this->active_phase()->tag()->id();
  return Backup::make(_backup_seq++, ss.str());
}

} // namespace test_rmf_task
