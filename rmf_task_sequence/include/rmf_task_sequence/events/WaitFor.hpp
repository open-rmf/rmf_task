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

#ifndef RMF_TASK_SEQUENCE__EVENTS__WAITFOR_HPP
#define RMF_TASK_SEQUENCE__EVENTS__WAITFOR_HPP

#include <rmf_traffic/Time.hpp>

#include <rmf_task_sequence/Event.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
/// A WaitFor event encompasses having the robot sit in place and wait for a
/// period of time to pass.
///
/// The Model of this event may be useful for calculating the Models of other
/// phases that include a period of time where the robot is waiting for a
/// process to finish. E.g. the PickUp and DropOff Models use WaitFor::Model to
/// calculate how much the robot's battery drains while waiting for the payload
/// to be transferred.
class WaitFor
{
public:

  class Description;
  using DescriptionPtr = std::shared_ptr<Description>;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;

  class Model;
};

//==============================================================================
class WaitFor::Description : public Event::Description
{
public:

  /// Make a WaitFor phase description
  ///
  /// \param[in] wait_duration
  ///   The duration of time that the phase should be waiting.
  static DescriptionPtr make(rmf_traffic::Duration wait_duration);

  /// Constructor
  Description(rmf_traffic::Duration duration_);

  /// Get the duration of the wait
  rmf_traffic::Duration duration() const;

  /// Set the duration of the wait
  Description& duration(rmf_traffic::Duration new_duration);

  // Documentation inherited
  Activity::ConstModelPtr make_model(
    rmf_task::State invariant_initial_state,
    const rmf_task::Parameters& parameters) const final;

  // Documentation inherited
  rmf_task::Header generate_header(
    const rmf_task::State& initial_state,
    const rmf_task::Parameters& parameters) const final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace events
} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__PHASES__WAITFOR_HPP
