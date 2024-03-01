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

#ifndef SRC__RMF_TASK_SEQUENCE__PHASES__INTERNAL_CANCELLATION_PHASE_HPP
#define SRC__RMF_TASK_SEQUENCE__PHASES__INTERNAL_CANCELLATION_PHASE_HPP

#include <rmf_task/events/SimpleEventState.hpp>

#include <rmf_task_sequence/Phase.hpp>

namespace rmf_task_sequence {
namespace phases {

class CancellationPhase : public Phase::Active
{
public:
  using ConstTagPtr = rmf_task::Phase::ConstTagPtr;

  static std::shared_ptr<CancellationPhase> make(
    ConstTagPtr tag,
    std::shared_ptr<Phase::Active> phase);

  ConstTagPtr tag() const final;

  Event::ConstStatePtr final_event() const final;

  rmf_traffic::Duration estimate_remaining_time() const final;

  Backup backup() const final;

  Resume interrupt(std::function<void()> task_is_interrupted) final;

  void cancel() final;

  void kill() final;

private:
  CancellationPhase() = default;

  ConstTagPtr _tag;
  rmf_task::events::SimpleEventStatePtr _state;
  std::shared_ptr<Phase::Active> _phase;
};

} // namespace phases
} // namespace rmf_task_sequences

#endif // SRC__RMF_TASK_SEQUENCE__PHASES__INTERNAL_CANCELLATION_PHASE_HPP
