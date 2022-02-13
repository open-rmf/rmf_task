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

#ifndef TEST__MOCK__MOCKDELIVERY_HPP
#define TEST__MOCK__MOCKDELIVERY_HPP

#include <rmf_task/Activator.hpp>
#include <rmf_task/requests/Delivery.hpp>

#include "MockTask.hpp"

namespace test_rmf_task {

//==============================================================================
/// A class that provides a mock implementation of an Active Delivery task
class MockDelivery : public rmf_task::Task
{
public:

  using Phase = rmf_task::Phase;
  using Description = rmf_task::requests::Delivery::Description;
  using Activator = rmf_task::Activator::Activate<Description>;

  static Activator make_activator();

  class Active : public MockTask::Active
  {
  public:

    template<typename... Args>
    Active(
      const Description& desc,
      std::optional<std::string> backup,
      Args&& ... args)
    : MockTask::Active(std::forward<Args>(args)...),
      _description(desc),
      _restored_state(std::move(backup))
    {
      // TODO(MXG): We could use the description, state, and parameters to
      // get the actual estimate for the pending phases
      using namespace std::chrono_literals;
      add_pending_phase("Go to pick up", "Pretending to go to a pick up point",
        1min);
      add_pending_phase("Pick up", "Pretending to pick something up", 30s);
      add_pending_phase("Go to drop off",
        "Pretending to go to a drop off point", 1min);
      add_pending_phase("Drop off", "Pretending to drop something off", 30s);

      if (_restored_state.has_value())
      {
        uint64_t phase_id = std::stoul(_restored_state.value());
        const uint64_t min_phase_id = 0;
        const uint64_t max_phase_id = pending_phases().size();
        if (phase_id < min_phase_id && phase_id > max_phase_id)
        {
          throw std::runtime_error("[MockDelivery::Active] phase_id given was " + std::to_string(
                    phase_id) + " not in range of " + std::to_string(
                    min_phase_id) + "," + std::to_string(max_phase_id) + ".");
        }
        _pending_phases.erase(_pending_phases.begin(),
          _pending_phases.begin()+phase_id);
      }
      start_next_phase(rmf_traffic::Time());
    }

    Backup backup() const override;

    Description _description;
    std::optional<std::string> _restored_state;
    mutable uint64_t _backup_seq = 0;
  };

};

} // namespace test_rmf_task

#endif // TEST__MOCK__MOCKDELIVERY_HPP
