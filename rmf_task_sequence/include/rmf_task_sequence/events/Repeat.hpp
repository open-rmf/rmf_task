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

#ifndef RMF_TASK_SEQUENCE__EVENTS__REPEAT_HPP
#define RMF_TASK_SEQUENCE__EVENTS__REPEAT_HPP

#include <rmf_task_sequence/Event.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
/// An event that repeats for a number of iterations
class Repeat
{
public:

  class Description;
  using DescriptionPtr = std::shared_ptr<Description>;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;
};

//==============================================================================
class Repeat::Description : public Event::Description
{
public:

  /// Make a Repeat description.
  ///
  /// \param[in] event
  ///   The event that will be repeated
  ///
  /// \param[in] repetitions
  ///   The number of times the event will be repeated
  static DescriptionPtr make(
    Event::ConstDescriptionPtr event,
    std::size_t repetitions);

  /// Get the event
  const Event::ConstDescriptionPtr event() const;

  /// Set the event
  Description& event(Event::ConstDescriptionPtr new_event);

  /// Get the repetitions
  std::size_t repetitions() const;

  /// Set the repetitions
  Description& repetitions(std::size_t new_repetitions);

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

#endif // RMF_TASK_SEQUENCE__EVENTS__REPEAT_HPP
