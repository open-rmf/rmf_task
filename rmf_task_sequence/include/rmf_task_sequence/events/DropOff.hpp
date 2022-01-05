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

#ifndef RMF_TASK_SEQUENCE__EVENTS__DROPOFF_HPP
#define RMF_TASK_SEQUENCE__EVENTS__DROPOFF_HPP

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_task/Payload.hpp>
#include <rmf_task_sequence/Event.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
/// A DropOff phase encompasses going to a location and transferring a payload
/// off of the robot.
class DropOff
{
public:

  using Location = rmf_traffic::agv::Plan::Goal;

  class Description;
  using DescriptionPtr = std::shared_ptr<Description>;

  class Model;
};

//==============================================================================
class DropOff::Description : public Event::Description
{
public:

  /// Make a DropOff phase description
  ///
  /// \param[in] drop_off_location
  ///   The location that the robot needs to get to for the drop-off
  ///
  /// \param[in] into_ingestor
  ///   The ingestor that will take care of unloading the items. We will
  ///   communicate with this ingestor to verify the success of unloading the
  ///   items.
  ///
  /// \param[in] payload
  ///   A description of what should be unloaded from the robot during drop-off
  ///
  /// \param[in] unloading_duration_estimate
  ///   An estimate for how long it will likely take to unload the items.
  static DescriptionPtr make(
    Location drop_off_location,
    std::string into_ingestor,
    Payload payload,
    rmf_traffic::Duration unloading_duration_estimate);

  /// Get the drop-off location
  const Location& drop_off_location() const;

  /// Set the drop-off location
  Description& drop_off_location(Location new_location);

  /// Get the ingestor to drop off into
  const std::string& into_ingestor() const;

  /// Set the ingestor to drop off into
  Description& into_ingestor(std::string new_ingestor);

  /// Get the Payload to drop off
  const Payload& payload() const;

  /// Set the Payload to drop off
  Description& payload(Payload new_payload);

  /// Get the unloading duration estimate
  rmf_traffic::Duration unloading_duration_estimate() const;

  /// Set the unloading duration estimate
  Description& unloading_duration_estimate(rmf_traffic::Duration new_duration);

  // Documentation inherited
  Activity::ConstModelPtr make_model(
    State invariant_initial_state,
    const Parameters& parameters) const final;

  // Documentation inherited
  Header generate_header(
    const State& initial_state,
    const Parameters& parameters) const final;

  class Implementation;
private:
  Description();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace events
} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__EVENTS__DROPOFF_HPP
