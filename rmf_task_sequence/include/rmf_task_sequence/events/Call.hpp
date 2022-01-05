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

#ifndef RMF_TASK_SEQUENCE__EVENTS__CALL_HPP
#define RMF_TASK_SEQUENCE__EVENTS__CALL_HPP

#include <rmf_task_sequence/Event.hpp>
#include <rmf_task_sequence/events/ContactCard.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
/// Make a phone call
class Call
{
public:
  class Description;
  using DescriptionPtr = std::shared_ptr<Description>;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;
};

//==============================================================================
class Call::Description : public Event::Description
{
public:

  /// Make a Call description.
  ///
  /// \param[in] contact
  ///   The contact of the entity to call
  ///
  /// \param[in] call_duration_estimate
  ///   An estimate for how long it will take for the call to complete
  static DescriptionPtr make(
    ContactCard contact,
    rmf_traffic::Duration call_duration_estimate);

  /// Get the contact
  const ContactCard& contact() const;

  /// Set the contact
  Description& contact(ContactCard new_contact);

  /// Get the call duration estimate
  rmf_traffic::Duration call_duration_estimate() const;

  /// Set the action duration estimate
  Description& call_duration_estimate(rmf_traffic::Duration new_duration);

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

#endif // RMF_TASK_SEQUENCE__EVENTS__CALL_HPP
