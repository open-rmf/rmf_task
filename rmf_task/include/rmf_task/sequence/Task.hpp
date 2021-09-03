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

#ifndef RMF_TASK__SEQUENCE__TASK_HPP
#define RMF_TASK__SEQUENCE__TASK_HPP

#include <rmf_task/execute/Task.hpp>

namespace rmf_task {
namespace sequence {

//==============================================================================
class Task : public execute::Task
{
public:

  // Documentation inherited
  const std::vector<execute::CompletedPhase>& completed_phases() const final;

  // Documentation inherited
  execute::ConstActivePhasePtr active_phase() const final;

  // Documentation inherited
  std::vector<execute::PendingPhase> pending_phases() const final;

  // Documentation inherited
  std::string id() const final;

  // Documentation inherited
  std::string category() const final;

  // Documentation inherited
  std::string detail() const final;

  // Documentation inherited
  const Request& original_request() const final;

  // Documentation inherited
  rmf_traffic::Time estimate_finish_time() const final;

  // Documentation inherited
  Resume interrupt(std::function<void ()> task_is_interrupted) final;

  // Documentation inherited
  void cancel() final;

  // Documentation inherited
  void kill() final;

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace sequence
} // namespace rmf_task

#endif // RMF_TASK__SEQUENCE__TASK_HPP
