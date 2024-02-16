/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#include "internal_CancellationPhase.hpp"

namespace rmf_task_sequence {
namespace phases {

//==============================================================================
std::shared_ptr<CancellationPhase> CancellationPhase::make(
  ConstTagPtr tag,
  std::shared_ptr<Phase::Active> phase)
{
  auto result = std::shared_ptr<CancellationPhase>(new CancellationPhase);
  result->_tag = std::move(tag);
  result->_state = rmf_task::events::SimpleEventState::make(
    (uint64_t)(-1),
    result->_tag->header().category(),
    result->_tag->header().detail(),
    rmf_task::Event::Status::Canceled,
    {},
    nullptr);

  result->_phase = std::move(phase);
  return result;
}

//==============================================================================
auto CancellationPhase::tag() const -> ConstTagPtr
{
  return _tag;
}

//==============================================================================
Event::ConstStatePtr CancellationPhase::final_event() const
{
  const auto child_event = _phase->final_event();
  _state->update_dependencies({child_event});
  const auto child_status = child_event->status();
  if (Event::Status::Blocked == child_status
    || Event::Status::Error == child_status
    || Event::Status::Failed == child_status)
  {
    // Promote the child status into the overall cancellation phase status
    // in case something needs the attention of the operator.
    _state->update_status(child_status);
  }
  else
  {
    _state->update_status(rmf_task::Event::Status::Canceled);
  }

  return _state;
}

//==============================================================================
rmf_traffic::Duration CancellationPhase::estimate_remaining_time() const
{
  return _phase->estimate_remaining_time();
}

//==============================================================================
auto CancellationPhase::backup() const -> Backup
{
  return _phase->backup();
}

//==============================================================================
auto CancellationPhase::interrupt(std::function<void()> task_is_interrupted)
-> Resume
{
  return _phase->interrupt(std::move(task_is_interrupted));
}

//==============================================================================
void CancellationPhase::cancel()
{
  _phase->cancel();
}

//==============================================================================
void CancellationPhase::kill()
{
  _phase->kill();
}

} // namespace phases
} // namespace rmf_task_sequence
