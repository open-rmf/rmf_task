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

#include <rmf_task/phases/RestoreBackup.hpp>

#include <rmf_task_sequence/Task.hpp>
#include <rmf_task_sequence/schemas/ErrorHandler.hpp>
#include <rmf_task_sequence/schemas/backup_PhaseSequenceTask_v0_1.hpp>

#include <nlohmann/json-schema.hpp>

#include <rmf_utils/Modular.hpp>

#include <iostream>

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

  std::string category;
  std::string detail;
  std::vector<ConstStagePtr> stages;

  static std::list<ConstStagePtr> get_stages(const Description& desc)
  {
    return std::list<ConstStagePtr>(
      desc._pimpl->stages.begin(),
      desc._pimpl->stages.end());
  }

  static DescriptionPtr make(
    std::string category,
    std::string detail,
    std::vector<ConstStagePtr> stages)
  {
    Description desc;
    desc._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(category),
        std::move(detail),
        std::move(stages)
      });

    return std::make_shared<Description>(std::move(desc));
  }
};

//==============================================================================
class Task::Model : public rmf_task::Task::Model
{
public:

  Model(
    Activity::ConstModelPtr activity_model,
    rmf_traffic::Time earliest_start_time)
  : _activity_model(std::move(activity_model)),
    _earliest_start_time(earliest_start_time)
  {
    // Do nothing
  }

  std::optional<rmf_task::Estimate> estimate_finish(
    const State& initial_state,
    const Constraints& task_planning_constraints,
    const TravelEstimator& travel_estimator) const final
  {
    return _activity_model->estimate_finish(
      initial_state,
      _earliest_start_time,
      task_planning_constraints,
      travel_estimator);
  }

  rmf_traffic::Duration invariant_duration() const final
  {
    return _activity_model->invariant_duration();
  }

private:
  Activity::ConstModelPtr _activity_model;
  rmf_traffic::Time _earliest_start_time;
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

    if (backup_state.has_value())
    {
      task->_load_backup(std::move(*backup_state));
      return task;
    }

    task->_generate_pending_phases();
    task->_begin_next_stage();

    return task;
  }

  // Documentation inherited
  Event::Status status_overview() const final;

  // Documentation inherited
  bool finished() const final;

  // Documentation inherited
  const std::vector<Phase::ConstCompletedPtr>& completed_phases() const final;

  // Documentation inherited
  Phase::ConstActivePtr active_phase() const final;

  // Documentation inherited
  std::optional<rmf_traffic::Time> active_phase_start_time() const final;

  // Documentation inherited
  const std::vector<Phase::Pending>& pending_phases() const final;

  // Documentation inherited
  const ConstTagPtr& tag() const final;

  // Documentation inherited
  rmf_traffic::Duration estimate_remaining_time() const final;

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

  static const nlohmann::json_schema::json_validator backup_schema_validator;

private:

  /// _load_backup should only be used in the make(~) function. It will
  /// fast-forward the progress of the task to catch up to a backed up state,
  /// since the task is being restored from a task that was already in progress.
  ///
  /// \return false if the task needs to be aborted due to a bad backup_state,
  /// otherwise return true.
  void _load_backup(std::string backup_state);
  void _generate_pending_phases();

  void _finish_phase();
  void _begin_next_stage(std::optional<nlohmann::json> restore = std::nullopt);
  void _finish_task();

  void _prepare_cancellation_sequence(
    std::vector<Phase::ConstDescriptionPtr> sequence);

  void _issue_backup(
    Phase::Tag::Id source_phase_id,
    Phase::Active::Backup phase_backup) const;

  Backup _generate_backup(
    Phase::Tag::Id current_phase_id,
    Phase::Active::Backup phase_backup) const;

  Backup _empty_backup() const;

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
    _tag(std::make_shared<Tag>(
        booking,
        description.generate_header(_get_state(), *parameters))),
    _update(std::move(update)),
    _checkpoint(std::move(checkpoint)),
    _phase_finished(std::move(phase_finished)),
    _task_finished(std::move(task_finished)),
    _task_is_interrupted(nullptr),
    _pending_stages(Description::Implementation::get_stages(description)),
    _cancel_sequence_initial_id(_pending_stages.size()+1)
  {
    // Do nothing
  }

  Phase::ConstActivatorPtr _phase_activator;
  std::function<rmf_traffic::Time()> _clock;
  std::function<State()> _get_state;
  ConstParametersPtr _parameters;
  ConstTagPtr _tag;
  std::function<void(Phase::ConstSnapshotPtr)> _update;
  std::function<void(Backup)> _checkpoint;
  std::function<void(Phase::ConstCompletedPtr)> _phase_finished;
  std::function<void()> _task_finished;

  std::function<void()> _task_is_interrupted;
  std::optional<Resume> _resume_phase;

  std::list<ConstStagePtr> _pending_stages;
  std::vector<Phase::Pending> _pending_phases;

  ConstStagePtr _active_stage;
  Phase::ActivePtr _active_phase;
  std::optional<rmf_traffic::Time> _current_phase_start_time;

  std::list<ConstStagePtr> _completed_stages;
  std::vector<Phase::ConstCompletedPtr> _completed_phases;

  std::optional<Resume> _resume_interrupted_phase;
  std::optional<Phase::Tag::Id> _cancelled_on_phase = std::nullopt;
  bool _killed = false;
  bool _finished = false;

  mutable std::optional<uint64_t> _last_phase_backup_sequence_number;
  mutable uint64_t _next_task_backup_sequence_number = 0;

  const uint64_t _cancel_sequence_initial_id;
};

//==============================================================================
const nlohmann::json_schema::json_validator
Task::Active::backup_schema_validator =
  nlohmann::json_schema::json_validator(
  schemas::backup_PhaseSequenceTask_v0_1);

//==============================================================================
Task::Builder::Builder()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
auto Task::Builder::add_phase(
  Phase::ConstDescriptionPtr description,
  std::vector<Phase::ConstDescriptionPtr> cancellation_sequence) -> Builder&
{
  // NOTE(MXG): We give each phase the ID of _pimpl->stages.size()+1 because
  // the ID 0 is reserved for the RestoreFromBackup phase, in case that's ever
  // needed.
  _pimpl->stages.emplace_back(
    std::make_shared<Stage>(
      Stage{
        _pimpl->stages.size()+1,
        std::move(description),
        std::move(cancellation_sequence)
      }));

  return *this;
}

//==============================================================================
auto Task::Builder::build(
  std::string category,
  std::string detail) -> std::shared_ptr<Description>
{
  return Description::Implementation::make(
    std::move(category),
    std::move(detail),
    _pimpl->stages);
}

//==============================================================================
Task::ConstModelPtr Task::Description::make_model(
  rmf_traffic::Time earliest_start_time,
  const Parameters& parameters) const
{
  std::vector<Activity::ConstDescriptionPtr> descriptions;
  descriptions.reserve(_pimpl->stages.size());
  for (const auto& desc : _pimpl->stages)
    descriptions.push_back(desc->description);

  return std::make_shared<Model>(
    Activity::SequenceModel::make(
      std::move(descriptions),
      State(),
      parameters),
    earliest_start_time);
}

//==============================================================================
auto Task::Description::generate_info(
  const rmf_task::State&,
  const rmf_task::Parameters&) const -> Info
{
  return Info{
    category(),
    detail()
  };
}

//==============================================================================
const std::string& Task::Description::category() const
{
  return _pimpl->category;
}

//==============================================================================
Task::Description& Task::Description::category(std::string new_category)
{
  _pimpl->category = std::move(new_category);
  return *this;
}

//==============================================================================
const std::string& Task::Description::detail() const
{
  return _pimpl->detail;
}

//==============================================================================
Task::Description& Task::Description::detail(std::string new_category)
{
  _pimpl->detail = std::move(new_category);
  return *this;
}

//==============================================================================
Header Task::Description::generate_header(
  const State& initial_state,
  const Parameters& parameters) const
{
  const auto model = make_model(
    initial_state.time().value(),
    parameters);

  return Header(
    _pimpl->category,
    _pimpl->detail,
    model->invariant_duration());
}

//==============================================================================
Task::Description::Description()
{
  // Do nothing
}

//==============================================================================
rmf_task::Event::Status Task::Active::status_overview() const
{
  if (_active_phase)
    return _active_phase->final_event()->status();

  if (_completed_phases.empty() && _pending_phases.empty())
  {
    // This means the task had no phases..? So it's completed by default.
    return Event::Status::Completed;
  }

  if (_pending_phases.empty())
  {
    // There are no pending phases, so the status of this task should be
    // reflected by the status of the last phase.
    return _completed_phases.back()->snapshot()->final_event()->status();
  }

  // There is no active phase, but there are pending phases remaining. This
  // must mean this function is being called while the task phase is switching,
  // which could technically cause a data race. We'll just go ahead and say that
  // the status is Underway, but maybe we should consider making noise about
  // this.
  return Event::Status::Underway;
}

//==============================================================================
bool Task::Active::finished() const
{
  return _finished;
}

//==============================================================================
const std::vector<Phase::ConstCompletedPtr>&
Task::Active::completed_phases() const
{
  return _completed_phases;
}

//==============================================================================
Phase::ConstActivePtr Task::Active::active_phase() const
{
  return _active_phase;
}

//==============================================================================
std::optional<rmf_traffic::Time> Task::Active::active_phase_start_time() const
{
  return _current_phase_start_time;
}

//==============================================================================
const std::vector<Phase::Pending>& Task::Active::pending_phases() const
{
  return _pending_phases;
}

//==============================================================================
const Task::ConstTagPtr& Task::Active::tag() const
{
  return _tag;
}

//==============================================================================
rmf_traffic::Duration Task::Active::estimate_remaining_time() const
{
  if (_finished)
    return rmf_traffic::Duration(0);

  auto remaining_time =
    _active_phase ? _active_phase->estimate_remaining_time() :
    rmf_traffic::Duration(0);
  for (const auto& p : _pending_phases)
    remaining_time += p.tag()->header().original_duration_estimate();

  return remaining_time;
}

//==============================================================================
auto Task::Active::backup() const -> Backup
{
  if (!_active_phase || _finished)
    return _empty_backup();

  return _generate_backup(
    _active_phase->tag()->id(),
    _active_phase->backup());
}

//==============================================================================
auto Task::Active::interrupt(std::function<void()> task_is_interrupted)
-> Resume
{
  _task_is_interrupted = std::move(task_is_interrupted);
  _resume_phase = _active_phase->interrupt(_task_is_interrupted);

  return Resume::make(
    [self = weak_from_this()]()
    {
      const auto me = self.lock();
      if (!me)
        return;

      me->_task_is_interrupted = nullptr;
      if (me->_resume_phase.has_value())
        (*me->_resume_phase)();
      else
        me->_begin_next_stage();
    });
}

//==============================================================================
void Task::Active::cancel()
{
  if (_cancelled_on_phase.has_value())
  {
    // If this already has a value, then the task is already running through
    // its cancellation sequence.
    return;
  }

  if (_finished)
  {
    // If the task is already finished, then there is no need to do anything
    // for the cancellation.
    return;
  }

  _cancelled_on_phase = _active_phase->tag()->id();
  _prepare_cancellation_sequence(_active_stage->cancellation_sequence);
  _active_phase->cancel();
}

//==============================================================================
void Task::Active::kill()
{
  _killed = true;
  _active_phase->kill();
}

//==============================================================================
void Task::Active::skip(uint64_t phase_id, bool value)
{
  if (value && _active_phase->tag()->id() == phase_id)
  {
    // If we are being told to skip the active phase then we will simply tell
    // it to cancel and then move on to the remaining phases.
    _active_phase->cancel();
    return;
  }

  for (auto& p : _pending_phases)
  {
    if (phase_id == p.tag()->id())
    {
      p.will_be_skipped(value);
      return;
    }
  }
}

//==============================================================================
void Task::Active::rewind(uint64_t phase_id)
{
  assert(_completed_phases.size() == _completed_stages.size());
  std::size_t completed_index = 0;
  auto stage_it = _completed_stages.begin();
  while (stage_it != _completed_stages.end())
  {
    if ((*stage_it)->id == phase_id)
      break;

    ++completed_index;
  }

  if (stage_it == _completed_stages.end())
  {
    // TODO(MXG): Indicate to the user that they asked to rewind to a phase
    // that hasn't been reached yet.
    return;
  }

  _pending_stages.insert(
    _pending_stages.begin(),
    stage_it,
    _completed_stages.begin());

  // The currently active stage should also be put back into pending
  _pending_stages.push_back(_active_stage);
  _generate_pending_phases();

  // If we are supposed to rewind to an earlier stage, then we should cancel
  // the currently active one.
  _active_phase->cancel();
}

//==============================================================================
void Task::Active::_load_backup(std::string backup_state_str)
{
  const auto restore_phase = rmf_task::phases::RestoreBackup::Active::make(
    backup_state_str, rmf_traffic::Duration(0));

  const auto start_time = _clock();

  const auto failed_to_restore = [&]() -> void
    {
      _pending_stages.clear();
      _phase_finished(
        std::make_shared<rmf_task::Phase::Completed>(
          rmf_task::Phase::Snapshot::make(*restore_phase),
          start_time,
          _clock()));

      _finish_task();
    };

  const auto backup_state = nlohmann::json::parse(backup_state_str);
  if (const auto result =
    schemas::ErrorHandler::has_error(backup_schema_validator, backup_state))
  {
    restore_phase->parsing_failed(result->message);
    return failed_to_restore();
  }

  const auto finished_it = backup_state.find("finished");
  if (finished_it != backup_state.end())
  {
    auto finished = finished_it.value().get<bool>();
    if (finished)
    {
      _task_finished();
      return;
    }

    _generate_pending_phases();
    _begin_next_stage();
    return;
  }

  const auto& current_phase_json = backup_state["current_phase"];
  const auto& cancelled_from_json = current_phase_json["cancelled_from"];
  if (cancelled_from_json)
  {
    const auto cancelled_from = cancelled_from_json.get<uint64_t>();
    if (cancelled_from >= _cancel_sequence_initial_id)
    {
      restore_phase->parsing_failed(
        "Invalid value [" + std::to_string(cancelled_from)
        + "] for [cancelled_from]. Value must be less than ["
        + std::to_string(_cancel_sequence_initial_id) + "]");

      return failed_to_restore();
    }

    for (const auto& stage : _pending_stages)
    {
      if (stage->id == cancelled_from)
      {
        _prepare_cancellation_sequence(stage->cancellation_sequence);
        break;
      }
    }
  }

  const auto& current_phase_id_json = current_phase_json["id"];
  const auto current_phase_id = current_phase_id_json.get<uint64_t>();
  bool found_phase = false;
  while (!found_phase && !_pending_stages.empty())
  {
    const auto stage = _pending_stages.front();
    if (stage->id != current_phase_id)
    {
      _pending_stages.pop_front();
      continue;
    }

    found_phase = true;
  }

  if (_pending_stages.empty())
  {
    restore_phase->parsing_failed(
      "Invalid value [" + std::to_string(current_phase_id)
      + "] for [current_phase/id]. "
      "Value is higher than all available phase IDs.");

    return failed_to_restore();
  }

  const auto& skip_phases_json = backup_state["skip_phases"];
  if (skip_phases_json)
  {
    const auto skip_phases = skip_phases_json.get<std::vector<uint64_t>>();
    auto pending_it = _pending_phases.begin();
    const auto pending_end = _pending_phases.end();
    for (const auto& id : skip_phases)
    {
      if (id == 0)
      {
        // This should probably issue a warning, because this would be kind of
        // weird, but it's not really a problem
        continue;
      }

      while (pending_it != pending_end && pending_it->tag()->id() < id)
      {
        ++pending_it;
      }

      if (pending_it == pending_end || id < pending_it->tag()->id())
      {
        // This shouldn't happen, but it's not a critical error. In the worst
        // case, the operator needs to resend a skip command.
        restore_phase->update_log().warn(
          "Unexpected ordering of phase skip IDs");
        continue;
      }

      pending_it->will_be_skipped(true);
    }
  }

  _begin_next_stage(std::optional<nlohmann::json>(current_phase_json["state"]));
}

//==============================================================================
void Task::Active::_generate_pending_phases()
{
  auto state = _get_state();
  _pending_phases.reserve(_pending_stages.size());
  for (const auto& s : _pending_stages)
  {
    _pending_phases.emplace_back(
      std::make_shared<Phase::Tag>(
        s->id,
        s->description->generate_header(state, *_parameters)
      )
    );

    state = s->description->make_model(
      state, *_parameters)->invariant_finish_state();
  }
}

//==============================================================================
void Task::Active::_finish_phase()
{
  _completed_stages.push_back(_active_stage);
  _active_stage = nullptr;

  const auto phase_finish_time = _clock();
  const auto completed_phase = std::make_shared<Phase::Completed>(
    rmf_task::Phase::Snapshot::make(*_active_phase),
    _current_phase_start_time.value(),
    phase_finish_time);

  _completed_phases.push_back(completed_phase);
  _phase_finished(completed_phase);

  _begin_next_stage();
}

//==============================================================================
void Task::Active::_begin_next_stage(std::optional<nlohmann::json> restore)
{
  if (_task_is_interrupted)
  {
    // If we currently expect the task to be interrupted but we reach this
    // function, then the previous phase finished despite the fact that it
    // should have been interrupted (maybe it's a phase that cannot be
    // interrupted). So we will now signal that the task is interrupted and
    // refuse to move on to the next phase.
    _task_is_interrupted();
    return;
  }

  if (_resume_phase.has_value())
  {
    // If we were expecting to resume a phase but we wound up here instead, then
    // the phase that we tried to interrupt finished instead of being
    // interrupted. We will clear this field since it is no longer relevant.
    _resume_phase = std::nullopt;
  }

  if (_killed)
    return _finish_task();

  while (true)
  {
    if (_pending_stages.empty())
      return _finish_task();

    _active_stage = _pending_stages.front();
    assert(_active_stage->id == _pending_phases.front().tag()->id());
    const auto skip_phase = _pending_phases.front().will_be_skipped();

    _pending_stages.pop_front();
    auto tag = _pending_phases.front().tag();
    _pending_phases.erase(_pending_phases.begin());

    // Reset our memory of phase backup sequence number
    _last_phase_backup_sequence_number = std::nullopt;

    if (skip_phase)
    {
      // If we're supposed to skip this phase, then continue looping until we
      // reach a phase that we're not supposed to skip.
      continue;
    }

    const auto phase_id = tag->id();
    _current_phase_start_time = _clock();
    _active_phase = _phase_activator->activate(
      _get_state,
      _parameters,
      std::move(tag),
      *_active_stage->description,
      std::move(restore),
      [me = weak_from_this()](Phase::ConstSnapshotPtr snapshot)
      {
        if (const auto self = me.lock())
          self->_update(snapshot);
      },
      [me = weak_from_this(), id = phase_id](
        Phase::Active::Backup backup)
      {
        if (const auto self = me.lock())
          self->_issue_backup(id, std::move(backup));
      },
      [me = weak_from_this()]()
      {
        if (const auto self = me.lock())
          self->_finish_phase();
      });

    _update(Phase::Snapshot::make(*_active_phase));
    _issue_backup(phase_id, _active_phase->backup());
    return;
  }
}

//==============================================================================
void Task::Active::_finish_task()
{
  _finished = true;
  _task_finished();
}

//==============================================================================
void Task::Active::_issue_backup(
  Phase::Tag::Id source_phase_id,
  Phase::Active::Backup phase_backup) const
{
  if (source_phase_id != _active_phase->tag()->id())
  {
    // If this backup is for something other than the current phase, ignore it
    return;
  }

  if (_last_phase_backup_sequence_number.has_value())
  {
    const auto cutoff = *_last_phase_backup_sequence_number;
    if (rmf_utils::modular(phase_backup.sequence()).less_than_or_equal(cutoff))
    {
      // The current backup sequence number has already passed this one
      return;
    }
  }

  _last_phase_backup_sequence_number = phase_backup.sequence();
  _checkpoint(_generate_backup(source_phase_id, std::move(phase_backup)));
}

//==============================================================================
void Task::Active::_prepare_cancellation_sequence(
  std::vector<Phase::ConstDescriptionPtr> sequence)
{
  _pending_phases.clear();
  _pending_stages.clear();

  uint64_t next_stage_id = _cancel_sequence_initial_id;
  for (auto&& phase : sequence)
  {
    _pending_stages.emplace_back(
      std::make_shared<Stage>(
        Stage{
          next_stage_id++,
          std::move(phase),
          {}
        }));
  }

  _generate_pending_phases();
}

//==============================================================================
auto Task::Active::_generate_backup(
  Phase::Tag::Id current_phase_id,
  Phase::Active::Backup phase_backup) const -> Backup
{
  nlohmann::json current_phase;
  current_phase["id"] = current_phase_id;
  if (_cancelled_on_phase.has_value())
    current_phase["cancelled_from"] = *_cancelled_on_phase;

  current_phase["state"] = phase_backup.state();

  std::vector<uint64_t> skipping_phases;
  for (const auto& p : _pending_phases)
  {
    if (p.will_be_skipped())
      skipping_phases.push_back(p.tag()->id());
  }

  nlohmann::json root;
  root["schema_version"] = 1;
  root["current_phase"] = std::move(current_phase);
  root["skip_phases"] = std::move(skipping_phases);
  // TODO(MXG): Is there anything else we need to consider as part of the state?

  return Backup::make(_next_task_backup_sequence_number++, root.dump());
}

//==============================================================================
auto Task::Active::_empty_backup() const -> Backup
{
  // Either the task is finished or the first phase has not started. Either way
  // there is no phase information to provide for the backup. This special case
  // is handled by providing a "finished" property where true means the task
  // finished while false means the task has not yet started.
  nlohmann::json root;
  root["schema_version"] = 1;
  root["finished"] = _finished;

  return Backup::make(_next_task_backup_sequence_number++, root.dump());
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
    const std::function<State()>& get_state,
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

//==============================================================================
void Task::add(
  rmf_task::Activator& activator,
  Phase::ConstActivatorPtr phase_activator,
  std::function<rmf_traffic::Time()> clock)
{
  activator.add_activator<Task::Description>(
    make_activator(std::move(phase_activator), std::move(clock)));
}

} // namespace rmf_task_sequence
