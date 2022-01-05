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

#ifndef RMF_TASK_SEQUENCE__EVENTS__WHILE_HPP
#define RMF_TASK_SEQUENCE__EVENTS__WHILE_HPP

#include <rmf_task_sequence/Event.hpp>

#include <functional>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
/// An event that remains active indefinitely until a condition is fullfiled
class While
{
public:

  class Description;
  using DescriptionPtr = std::shared_ptr<Description>;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;
};

//==============================================================================
class While::Description : public Event::Description
{
public:

  /// Make a While description.
  ///
  /// \param[in] event
  ///   The event that will be repeated indefinitely
  ///
  /// \param[in] completed
  ///   A callback that will return true when the event is completed
  ///
  /// \param[in] while_duration_estimate
  ///   An estimate of how long the event will take to complete
  static DescriptionPtr make(
    Event::ConstDescriptionPtr event,
    std::function<bool()> completed,
    rmf_traffic::Duration while_duration_estimate);

  /// Get the event
  const Event::ConstDescriptionPtr event() const;

  /// Set the event
  Description& event(Event::ConstDescriptionPtr new_event);

  /// Get the completed condition
  const std::function<bool()> completed() const;

  /// Set the completed condition
  Description& completed(std::function<bool()> new_completed);

  /// Get the duration estimate
  const rmf_traffic::Duration while_duration_estimate() const;

  /// Set the duration estimate
  Description& while_duration_estimate(rmf_traffic::Duration new_duration);

  // Documentation inherited
  Activity::ConstModelPtr make_model(
    State invariant_initial_state,
    const Parameters& parameters) const final;

  // Documentation inherited
  Header generate_header(
    const rmf_task::State& initial_state,
    const Parameters& parameters) const final;

  class Implementation;
private:
  Description();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace events
} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__EVENTS__WHILE_HPP
