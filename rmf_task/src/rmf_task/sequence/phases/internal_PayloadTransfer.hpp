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

#ifndef SRC__RMF_TASK__SEQUENCE__PHASES__INTERNAL_PAYLOADTRANSFER_HPP
#define SRC__RMF_TASK__SEQUENCE__PHASES__INTERNAL_PAYLOADTRANSFER_HPP

#include <rmf_task/Payload.hpp>
#include <rmf_task/sequence/Phase.hpp>
#include <rmf_task/sequence/phases/GoToPlace.hpp>

#include <rmf_traffic/agv/Planner.hpp>

namespace rmf_task {
namespace sequence {
namespace phases {

//==============================================================================
class PayloadTransfer
{
public:

  using Location = rmf_traffic::agv::Plan::Goal;

  std::string target;
  rmf_traffic::Duration transfer_duration;
  Phase::ConstDescriptionPtr go_to_place;

  class Model : public Phase::Model
  {
  public:

    static Phase::ConstModelPtr make(
      State invariant_initial_state,
      const Parameters& parameters,
      const Phase::ConstDescriptionPtr& go_to_place,
      rmf_traffic::Duration transfer_duration);

    std::optional<State> estimate_finish(
      State initial_state,
      const Constraints& constraints,
      const TravelEstimator& travel_estimator) const final;

    rmf_traffic::Duration invariant_duration() const final;

    State invariant_finish_state() const final;

  private:

    Model(
      Phase::ConstModelPtr go_to_place,
      rmf_traffic::Duration transfer_duration,
      double transfer_battery_drain);

    Phase::ConstModelPtr _go_to_place;
    rmf_traffic::Duration _transfer_duration;
    double _transfer_battery_drain;
  };

};

} // namespace phases
} // namespace sequence
} // namespace rmf_task

#endif // SRC__RMF_TASK__SEQUENCE__PHASES__INTERNAL_PAYLOADTRANSFER_HPP
