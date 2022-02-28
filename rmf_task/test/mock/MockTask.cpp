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

#include "MockTask.hpp"

namespace test_rmf_task {

//==============================================================================
rmf_task::Event::Status MockTask::Active::status_overview() const
{
  if (_active_phase)
    return _active_phase->final_event()->status();

  return rmf_task::Event::Status::Completed;
}

//==============================================================================
bool MockTask::Active::finished() const
{
  if (_active_phase)
    _active_phase->final_event()->finished();

  return true;
}

//==============================================================================
auto MockTask::Active::completed_phases() const
-> const std::vector<Phase::ConstCompletedPtr>&
{
  return _completed_phases;
}

//==============================================================================
auto MockTask::Active::active_phase() const -> Phase::ConstActivePtr
{
  return _active_phase;
}

//==============================================================================
std::optional<rmf_traffic::Time>
MockTask::Active::active_phase_start_time() const
{
  return std::nullopt;
}

//==============================================================================
auto MockTask::Active::pending_phases() const
-> const std::vector<Phase::Pending>&
{
  return _pending_phases;
}

//==============================================================================
auto MockTask::Active::tag() const -> const ConstTagPtr&
{
  return _tag;
}

//==============================================================================
rmf_traffic::Duration MockTask::Active::estimate_remaining_time() const
{
  return _tag->header().original_duration_estimate();
}

//==============================================================================
auto MockTask::Active::backup() const -> Backup
{
  throw std::runtime_error("MockTask::Active::backup() not implemented");
}

//==============================================================================
auto MockTask::Active::interrupt(std::function<void()> task_is_interrupted)
-> Resume
{
  task_is_interrupted();
  return make_resumer([]() {});
}

//==============================================================================
void MockTask::Active::cancel()
{
  _pending_phases.clear();
  _task_finished();
}

//==============================================================================
void MockTask::Active::kill()
{
  _pending_phases.clear();
  _task_finished();
}

//==============================================================================
void MockTask::Active::skip(uint64_t, bool)
{
  throw std::runtime_error("MockTask::Active::skip(~) not implemented");
}

//==============================================================================
void MockTask::Active::rewind(uint64_t)
{
  throw std::runtime_error("MockTask::Active::rewind(~) not implemented");
}

//==============================================================================
MockTask::Active::Active(
  std::function<rmf_task::State()>,
  const rmf_task::ConstParametersPtr&,
  const Task::ConstBookingPtr& booking,
  std::function<void(Phase::ConstSnapshotPtr)> update,
  std::function<void(Task::Active::Backup)> checkpoint,
  std::function<void(Phase::ConstCompletedPtr)> phase_finished,
  std::function<void()> task_finished)
: _tag(std::make_shared<Tag>(
      booking,
      rmf_task::Header(
        "Mock Task", "Mocked up task", rmf_traffic::Duration(0)))),
  _update(std::move(update)),
  _checkpoint(std::move(checkpoint)),
  _phase_finished(std::move(phase_finished)),
  _task_finished(std::move(task_finished))
{
  // Do nothing
}

//==============================================================================
void MockTask::Active::add_pending_phase(
  std::string name,
  std::string detail,
  rmf_traffic::Duration estimate)
{
  _pending_phases.emplace_back(
    std::make_shared<Phase::Tag>(
      _next_phase_id++,
      rmf_task::Header(std::move(name), std::move(detail), estimate)));
}

//==============================================================================
void MockTask::Active::start_next_phase(rmf_traffic::Time current_time)
{
  if (_active_phase)
  {
    _phase_finished(
      std::make_shared<Phase::Completed>(
        rmf_task::Phase::Snapshot::make(*_active_phase),
        _active_phase->_start_time,
        current_time));
  }

  if (_pending_phases.empty())
    return _task_finished();

  const auto next_phase = _pending_phases.front();
  _pending_phases.erase(_pending_phases.begin());
  _active_phase = std::make_shared<MockPhase::Active>(
    current_time,
    next_phase.tag(),
    [update = _update](Phase::ConstSnapshotPtr u) { update(std::move(u)); },
    []() {});
}

//==============================================================================
void MockTask::Active::issue_backup()
{
  _checkpoint(backup());
}

} // namespace test_rmf_task
