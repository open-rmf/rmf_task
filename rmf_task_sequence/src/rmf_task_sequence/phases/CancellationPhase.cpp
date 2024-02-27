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
  result->_tag = std::make_shared<Phase::Tag>(
    tag->id(),
    Header(
      "Cancellation in progress: " + tag->header().category(),
      "Cancellation in progress: " + tag->header().detail(),
      tag->header().original_duration_estimate()));

  result->_state = rmf_task::events::SimpleEventState::make(
    0,
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

  std::vector<Event::ConstStatePtr> queue;
  std::unordered_set<Event::ConstStatePtr> visited;
  uint64_t highest_index = 0;

  queue.push_back(child_event);
  while (!queue.empty())
  {
    const Event::ConstStatePtr e = queue.back();
    queue.pop_back();

    if (!e)
    {
      // A nullptr for some reason..?
      continue;
    }

    if (!visited.insert(e).second)
    {
      // This event state was already visited in the past
      continue;
    }

    for (const auto& d : e->dependencies())
    {
      queue.push_back(d);
    }

    highest_index = std::max(highest_index, e->id());
  }

  // Set the cancellation event ID to one higher than any other event currently
  // active in the tree. That way we can give it a valid event ID that doesn't
  // conflict with any other events in the phase.
  _state->modify_id(highest_index+1);

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
