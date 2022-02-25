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

#ifndef TEST__MOCK__MOCKTASK_HPP
#define TEST__MOCK__MOCKTASK_HPP

#include <rmf_task/Task.hpp>

#include "MockPhase.hpp"

namespace test_rmf_task {

//==============================================================================
class MockTask : public rmf_task::Task
{
public:

  using Phase = rmf_task::Phase;

  class Active : public rmf_task::Task::Active
  {
  public:

    rmf_task::Event::Status status_overview() const override;

    bool finished() const override;

    const std::vector<Phase::ConstCompletedPtr>&
    completed_phases() const override;

    Phase::ConstActivePtr active_phase() const override;

    std::optional<rmf_traffic::Time> active_phase_start_time() const override;

    const std::vector<Phase::Pending>& pending_phases() const override;

    const ConstTagPtr& tag() const override;

    rmf_traffic::Duration estimate_remaining_time() const override;

    Backup backup() const override;

    Resume interrupt(std::function<void()> task_is_interrupted) override;

    void cancel() override;

    void kill() override;

    void skip(uint64_t phase_id, bool value = true) override;

    void rewind(uint64_t phase_id) override;

    Active(
      std::function<rmf_task::State()> get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const Task::ConstBookingPtr& booking,
      std::function<void(Phase::ConstSnapshotPtr)> update,
      std::function<void(Task::Active::Backup)> checkpoint,
      std::function<void(Phase::ConstCompletedPtr)> phase_finished,
      std::function<void()> task_finished);

    void add_pending_phase(
      std::string name,
      std::string detail,
      rmf_traffic::Duration estimate);

    void start_next_phase(rmf_traffic::Time current_time);

    void issue_backup();

    ConstTagPtr _tag;
    std::vector<Phase::ConstCompletedPtr> _completed_phases;
    std::shared_ptr<MockPhase::Active> _active_phase;
    std::vector<Phase::Pending> _pending_phases;
    std::size_t _next_phase_id = 0;

    std::function<void(Phase::ConstSnapshotPtr)> _update;
    std::function<void(Task::Active::Backup)> _checkpoint;
    std::function<void(Phase::ConstCompletedPtr)> _phase_finished;
    std::function<void()> _task_finished;
  };

};

} // namespace test_rmf_task

#endif // TEST__MOCK__MOCKTASK_HPP
