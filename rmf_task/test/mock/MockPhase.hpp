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

#ifndef TEST__MOCK__MOCKPHASE_HPP
#define TEST__MOCK__MOCKPHASE_HPP

#include <rmf_task/Phase.hpp>

#include "MockEvent.hpp"

namespace test_rmf_task {

//==============================================================================
/// A class that provides a mock implementation of a generic phase
class MockPhase : public rmf_task::Phase
{
public:

  class Active : public rmf_task::Phase::Active
  {
  public:

    Active(
      rmf_traffic::Time start_time_,
      ConstTagPtr tag_,
      std::function<void(Phase::ConstSnapshotPtr)> update_,
      std::function<void()> phase_finished_);

    ConstTagPtr tag() const final;
    rmf_task::Event::ConstStatePtr final_event() const final;
    rmf_traffic::Duration estimate_remaining_time() const final;

    void send_update() const;

    ConstTagPtr _tag;
    std::shared_ptr<MockEvent> _event;
    rmf_traffic::Time _start_time;
    std::function<void(Phase::ConstSnapshotPtr)> _update;
    std::function<void()> _phase_finished;
  };

};

} // namespace test_rmf_task

#endif // TEST__MOCK__MOCKPHASE_HPP
