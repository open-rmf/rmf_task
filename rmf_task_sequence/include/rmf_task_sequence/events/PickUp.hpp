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

#ifndef RMF_TASK_SEQUENCE__EVENTS__PICKUP_HPP
#define RMF_TASK_SEQUENCE__EVENTS__PICKUP_HPP

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_task/Payload.hpp>
#include <rmf_task_sequence/Event.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
/// A PickUp phase encompasses going to a location and transferring a payload
/// into/onto the robot.
class PickUp
{
public:

  using Location = rmf_traffic::agv::Plan::Goal;

  class Description;
  using DescriptionPtr = std::shared_ptr<Description>;

  class Model;
};

//==============================================================================
class PickUp::Description : public Event::Description
{
public:

  /// Make a PickUp phase description
  ///
  /// \param[in] pickup_location
  ///   The location that the robot needs to get to for the pickup
  ///
  /// \param[in] from_dispenser
  ///   The dispenser that will take care of loading the items. We will
  ///   communicate with this dispenser to verify the success of loading the
  ///   items.
  ///
  /// \param[in] payload
  ///   A description of what should be loaded into the robot during the pick-up
  ///
  /// \param[in] loading_duration_estimate
  ///   An estimate for how long it will likely take to load the items.
  static DescriptionPtr make(
    Location pickup_location,
    std::string from_dispenser,
    Payload payload,
    rmf_traffic::Duration loading_duration_estimate);

  /// Get the pickup location
  const Location& pickup_location() const;

  /// Change the pickup location
  Description& pickup_location(Location new_location);

  /// Get the dispenser to pick up from
  const std::string& from_dispenser() const;

  /// Change the dispenser to pick up from
  Description& from_dispenser(std::string new_dispenser);

  /// Get the payload to pick up
  const Payload& payload() const;

  /// Change the payload to pick up
  Description& payload(Payload new_payload);

  /// Get the loading duration estimate
  rmf_traffic::Duration loading_duration_estimate() const;

  /// Change the loading duration estimate
  Description& loading_duration_estimate(rmf_traffic::Duration new_duration);

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

#endif // RMF_TASK_SEQUENCE__EVENTS__PICKUP_HPP
