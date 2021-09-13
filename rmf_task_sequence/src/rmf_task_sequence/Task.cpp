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
    std::function<rmf_traffic::Time()> clock,
    std::function<State()> get_state,
    const ConstParametersPtr& parameters,
    const ConstBookingPtr& booking,
    const Description& description,
    std::optional<std::string> backup_state,
    std::function<void(Phase::ConstSnapshotPtr)> update,
    std::function<void(Task::Active::Backup)> checkpoint,
    std::function<void(Phase::ConstCompletedPtr)> phase_finished,
    std::function<void()> task_finished)
  {
    auto task = std::shared_ptr<Active>(
      new Active(
        std::move(phase_activator),
        std::move(clock),
        std::move(get_state),
        parameters,
        booking,
        description,
        std::move(update),
        std::move(checkpoint),
        std::move(phase_finished),
        std::move(task_finished)));

    task->_load_backup(std::move(backup_state));
    task->_generate_pending_phases();
    task->_begin_next_stage();

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

  /// Get a weak reference to this object
  std::weak_ptr<Active> weak_from_this() const;

private:

  /// _load_backup should only be used in the make(~) function. It will
  /// fast-forward the progress of the task to catch up to a backed up state,
  /// since the task is being restored from a task that was already in progress.
  void _load_backup(std::optional<std::string> backup_state);
  void _generate_pending_phases();

  void _finish_phase();
  void _begin_next_stage();
  void _finish_task();

  void _issue_backup(Phase::Active::Backup backup);

  Active(
    Phase::ConstActivatorPtr phase_activator,
    std::function<rmf_traffic::Time()> clock,
    std::function<State()> get_state,
    const ConstParametersPtr& parameters,
    const ConstBookingPtr& booking,
    const Description& description,
    std::function<void(Phase::ConstSnapshotPtr)> update,
    std::function<void(Backup)> checkpoint,
    std::function<void(Phase::ConstCompletedPtr)> phase_finished,
    std::function<void()> task_finished)
    : _phase_activator(std::move(phase_activator)),
      _clock(std::move(clock)),
      _get_state(std::move(get_state)),
      _parameters(parameters),
      _booking(std::move(booking)),
      _update(std::move(update)),
      _checkpoint(std::move(checkpoint)),
      _phase_finished(std::move(phase_finished)),
      _task_finished(std::move(task_finished)),
      _pending_stages(Description::Implementation::get_stages(description))
  {
    // Do nothing
  }

  Phase::ConstActivatorPtr _phase_activator;
  std::function<rmf_traffic::Time()> _clock;
  std::function<State()> _get_state;
  ConstParametersPtr _parameters;
  ConstBookingPtr _booking;
  std::function<void(Phase::ConstSnapshotPtr)> _update;
  std::function<void(Backup)> _checkpoint;
  std::function<void(Phase::ConstCompletedPtr)> _phase_finished;
  std::function<void()> _task_finished;

  std::list<ConstStagePtr> _pending_stages;
  std::vector<Phase::ConstPendingPtr> _pending_phases;

  ConstStagePtr _active_stage;
  Phase::ActivePtr _active_phase;
  std::optional<rmf_traffic::Time> _current_phase_start_time;

  std::list<ConstStagePtr> _completed_stages;
  std::vector<Phase::ConstCompletedPtr> _completed_phases;

  std::optional<Resume> _resume_interrupted_phase;
  bool _cancelled = false;
  bool _killed = false;
};

//==============================================================================
void Task::Active::_load_backup(std::optional<std::string> backup_state_opt)
{

}

//==============================================================================
void Task::Active::_generate_pending_phases()
{
  auto state = _get_state();
  _pending_phases.reserve(_pending_stages.size());
  for (const auto& s : _pending_stages)
  {
    _pending_phases.push_back(
      std::make_shared<Phase::Pending>(
        s->description->make_tag(s->id, state, *_parameters)));
  }
}

//==============================================================================
void Task::Active::_finish_phase()
{
  _completed_stages.push_back(_active_stage);
  _active_stage = nullptr;

  const auto phase_finish_time = _clock();
  const auto completed_phase =
    std::make_shared<Phase::Completed>(
      _active_phase->tag(),
      _active_phase->finish_condition()->log(),
      _current_phase_start_time.value(),
      phase_finish_time);

  _completed_phases.push_back(completed_phase);
  _phase_finished(completed_phase);

  _begin_next_stage();
}

//==============================================================================
void Task::Active::_begin_next_stage()
{
  if (_pending_stages.empty())
    return _finish_task();

  assert(!_pending_phases.empty());
  _active_stage = _pending_stages.front();
  assert(_active_stage->id == _pending_phases.front()->tag()->id());

  _pending_stages.pop_front();
  auto tag = _pending_phases.front()->tag();
  _pending_phases.erase(_pending_phases.begin());

  _current_phase_start_time = _clock();
  _active_phase = _phase_activator->activate(
    _get_state,
    std::move(tag),
    *_active_stage->description,
    [me = weak_from_this()](Phase::ConstSnapshotPtr snapshot)
    {
      if (const auto self = me.lock())
        self->_update(snapshot);
    },
    [me = weak_from_this()](Phase::Active::Backup backup)
    {
      if (const auto self = me.lock())
        self->_issue_backup(std::move(backup));
    },
    [me = weak_from_this()]()
    {
      if (const auto self = me.lock())
        self->_finish_phase();
    });
}

//==============================================================================
void Task::Active::_finish_task()
{
  _task_finished();
}

//==============================================================================
void Task::Active::_issue_backup(Phase::Active::Backup backup)
{

}

//==============================================================================
auto Task::make_activator(
  Phase::ConstActivatorPtr phase_activator,
  std::function<rmf_traffic::Time()> clock)
-> rmf_task::Activator::Activate<Description>
{
  return [
      phase_activator = std::move(phase_activator),
      clock = std::move(clock)
    ](
    std::function<State()> get_state,
    const ConstParametersPtr& parameters,
    const ConstBookingPtr& booking,
    const Description& description,
    std::optional<std::string> backup_state,
    std::function<void(Phase::ConstSnapshotPtr)> update,
    std::function<void(Task::Active::Backup)> checkpoint,
    std::function<void(Phase::ConstCompletedPtr)> phase_finished,
    std::function<void()> task_finished) -> ActivePtr
    {
      return Active::make(
        phase_activator,
        clock,
        std::move(get_state),
        parameters,
        booking,
        description,
        std::move(backup_state),
        std::move(update),
        std::move(checkpoint),
        std::move(phase_finished),
        std::move(task_finished));
    };
}

} // namespace rmf_task_sequence
