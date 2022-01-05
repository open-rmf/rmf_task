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

#include <rmf_task_sequence/events/Call.hpp>
#include <rmf_task_sequence/events/WaitFor.hpp>

#include <rmf_task_sequence/Activity.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
class Call::Description::Implementation
{
public:

  ContactCard contact;
  rmf_traffic::Duration call_duration_estimate;

};

//==============================================================================
auto Call::Description::make(
  ContactCard contact,
  rmf_traffic::Duration wait_duration) -> DescriptionPtr
{
  auto output = std::shared_ptr<Description>(new Description);
  output->_pimpl =
    rmf_utils::make_impl<Implementation>(
    Implementation{
      std::move(contact),
      wait_duration});

  return output;
}

//==============================================================================
auto Call::Description::contact() const -> const ContactCard&
{
  return _pimpl->contact;
}

//==============================================================================
auto Call::Description::contact(
  ContactCard new_contact)-> Description&
{
  _pimpl->contact = std::move(new_contact);
  return *this;
}

//==============================================================================
rmf_traffic::Duration Call::Description::call_duration_estimate() const
{
  return _pimpl->call_duration_estimate;
}

//==============================================================================
auto Call::Description::call_duration_estimate(
  rmf_traffic::Duration new_duration) -> Description&
{
  _pimpl->call_duration_estimate = new_duration;
  return *this;
}

//==============================================================================
Activity::ConstModelPtr Call::Description::make_model(
  State invariant_initial_state,
  const Parameters& parameters) const
{
  const auto description =
    WaitFor::Description::make(_pimpl->call_duration_estimate);
  return description->make_model(invariant_initial_state, parameters);
}

//==============================================================================
Header Call::Description::generate_header(
  const State&, const Parameters&) const
{
  const std::string& number = "+" +
    std::to_string(_pimpl->contact.number().country_code()) +
    " " + std::to_string(_pimpl->contact.number().number());

  return Header(
    "Calling",
    "Calling " + _pimpl->contact.name() + " at " + number,
    _pimpl->call_duration_estimate);
}

//==============================================================================
Call::Description::Description()
{
  // Do nothing
}

} // namespace phases
} // namespace rmf_task_sequence
