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

#include <list>

#include <rmf_task_sequence/Task.hpp>

namespace rmf_task_sequence {

namespace {
//==============================================================================
struct Stage
{
  Phase::Tag::Id id;
  Phase::ConstDescriptionPtr description;
  std::vector<Phase::ConstDescriptionPtr> cancellation_sequence;
};
using ConstStagePtr = std::shared_ptr<const Stage>;
} // anonymous namespace

//==============================================================================
class Task::Builder::Implementation
{
public:
  std::vector<ConstStagePtr> stages;
};

//==============================================================================
class Task::Description::Implementation
{
public:

  std::vector<ConstStagePtr> stages;

  static std::list<ConstStagePtr> get_stages(const Description& desc)
  {
    return std::list<ConstStagePtr>(
      desc._pimpl->stages.begin(),
      desc._pimpl->stages.end());
  }

};

//==============================================================================
class Task::Active
  : public rmf_task::Task::Active,
  public std::enable_shared_from_this<Active>
{
public:

  static Task::ActivePtr make(
    Phase::ConstActivatorPtr phase_activator,
    const ConstBookingPtr& booking,
    const Description& description,
    std::optional<std::string> backup_state,
    std::function<void(Phase::ConstSnapshotPtr)> update,
    std::function<void(Phase::ConstCompletedPtr)> phase_finished,
    std::function<void()> task_finished)
  {
    auto task = std::shared_ptr<Active>(
      new Active(
        std::move(phase_activator),
        booking,
        description,
        std::move(update),
        std::move(phase_finished),
        std::move(task_finished)));

    // TODO(MXG): Make use of backup_state to fast forward the task to the
    // relevant stage

    task->generate_pending_phases();
    task->begin_next_stage();

    return task;
  }

  // Documentation inherited
  const std::vector<Phase::ConstCompletedPtr>& completed_phases() const final;

  // Documentation inherited
  Phase::ConstActivePtr active_phase() const final;

  // Documentation inherited
  std::vector<Phase::Pending> pending_phases() const final;

  // Documentation inherited
  const ConstTagPtr& tag() const final;

  // Documentation inherited
  rmf_traffic::Time estimate_finish_time() const final;

  // Documentation inherited
  Backup backup() const final;

  // Documentation inherited
  Resume interrupt(std::function<void()> task_is_interrupted) final;

  // Documentation inherited
  void cancel() final;

  // Documentation inherited
  void kill() final;

  // Documentation inherited
  void skip(uint64_t phase_id, bool value = true) final;

  // Documentation inherited
  void rewind(uint64_t phase_id) final;

private:

  void generate_pending_phases();

  void begin_next_stage();

  Active(
    Phase::ConstActivatorPtr phase_activator,
    const ConstBookingPtr& booking,
    const Description& description,
    std::function<void(Phase::ConstSnapshotPtr)> update,
    std::function<void(Phase::ConstCompletedPtr)> phase_finished,
    std::function<void()> task_finished)
    : _phase_activator(std::move(phase_activator)),
      _booking(std::move(booking)),
      _update(std::move(update)),
      _phase_finished(std::move(phase_finished)),
      _task_finished(std::move(task_finished)),
      _pending_stages(Description::Implementation::get_stages(description))
  {
    // Do nothing
  }

  Phase::ConstActivatorPtr _phase_activator;
  ConstBookingPtr _booking;
  std::function<void(Phase::ConstSnapshotPtr)> _update;
  std::function<void(Phase::ConstCompletedPtr)> _phase_finished;
  std::function<void()> _task_finished;

  std::list<ConstStagePtr> _pending_stages;
  std::vector<Phase::ConstPendingPtr> _pending_phases;

  ConstStagePtr _active_stage;
  Phase::ActivePtr _active_phase;
  std::list<ConstStagePtr> _completed_stages;

  std::optional<Resume> _resume_interrupted_phase;
  bool _cancelled = false;
  bool _killed = false;
};

//==============================================================================
void Task::Active::generate_pending_phases()
{
  _pending_phases.reserve(_pending_stages.size());
  for (const auto& s : _pending_stages)
  {

  }
}

//==============================================================================
void Task::Active::begin_next_stage()
{

}

//==============================================================================
auto Task::make_activator(Phase::ConstActivatorPtr phase_activator)
-> rmf_task::Activator::Activate<Description>
{

}

} // namespace rmf_task_sequence
