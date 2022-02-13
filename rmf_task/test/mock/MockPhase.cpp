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

#include "MockPhase.hpp"

namespace test_rmf_task {

//==============================================================================
MockPhase::Active::Active(
  rmf_traffic::Time start_time_,
  ConstTagPtr tag_,
  std::function<void(Phase::ConstSnapshotPtr)> update_,
  std::function<void()> phase_finished_)
: _tag(std::move(tag_)),
  _event(std::make_shared<MockEvent>(
      0, "Mock Event", "This is a mocked up event")),
  _start_time(start_time_),
  _update(std::move(update_)),
  _phase_finished(std::move(phase_finished_))
{
  // Do nothing
}

//==============================================================================
rmf_task::Phase::ConstTagPtr MockPhase::Active::tag() const
{
  return _tag;
}

//==============================================================================
rmf_task::Event::ConstStatePtr MockPhase::Active::final_event() const
{
  return _event;
}

//==============================================================================
rmf_traffic::Duration MockPhase::Active::estimate_remaining_time() const
{
  return _tag->header().original_duration_estimate();
}

//==============================================================================
void MockPhase::Active::send_update() const
{
  _update(Phase::Snapshot::make(*this));
}

} // namespace test_rmf_task
