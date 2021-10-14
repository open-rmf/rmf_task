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

#ifndef SRC__RMF_TASK_SEQUENCE__PHASES__INTERNAL_PAYLOADTRANSFER_HPP
#define SRC__RMF_TASK_SEQUENCE__PHASES__INTERNAL_PAYLOADTRANSFER_HPP

#include <rmf_task/Payload.hpp>
#include <rmf_task_sequence/Phase.hpp>
#include <rmf_task_sequence/phases/GoToPlace.hpp>
#include <rmf_task_sequence/phases/WaitFor.hpp>

#include <rmf_traffic/agv/Planner.hpp>

namespace rmf_task_sequence {
namespace phases {

//==============================================================================
class PayloadTransfer
{
public:

  using Location = rmf_traffic::agv::Plan::Goal;

  std::string target;
  Payload payload;
  GoToPlace::DescriptionPtr go_to_place;
  WaitFor::DescriptionPtr wait_for;
  std::vector<Phase::ConstDescriptionPtr> descriptions;

  PayloadTransfer(
    Location location_,
    std::string target_,
    Payload payload_,
    rmf_traffic::Duration loading_duration_estimate);

  Phase::ConstModelPtr make_model(
    State invariant_initial_state,
    const Parameters& parameters) const;

  Phase::ConstTagPtr make_tag(
    const std::string& type,
    Phase::Tag::Id id,
    const State& initial_state,
    const Parameters& parameters) const;
};

} // namespace phases
} // namespace rmf_task_sequence

#endif // SRC__RMF_TASK_SEQUENCE__PHASES__INTERNAL_PAYLOADTRANSFER_HPP
