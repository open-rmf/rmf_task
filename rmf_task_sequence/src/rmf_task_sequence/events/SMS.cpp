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

#include <rmf_task_sequence/events/SMS.hpp>
#include <rmf_task_sequence/events/WaitFor.hpp>

#include <rmf_task_sequence/Activity.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
class SMS::Description::Implementation
{
public:

  std::string message;
  ContactCard contact;
  rmf_traffic::Duration sms_duration_estimate;
};

//==============================================================================
auto SMS::Description::make(
  const std::string& message,
  ContactCard contact,
  rmf_traffic::Duration wait_duration) -> DescriptionPtr
{
  auto output = std::shared_ptr<Description>(new Description);
  output->_pimpl =
    rmf_utils::make_impl<Implementation>(
    Implementation{
      message,
      std::move(contact),
      wait_duration});

  return output;
}

//==============================================================================
const std::string& SMS::Description::message() const
{
  return _pimpl->message;
}

//==============================================================================
auto SMS::Description::message(const std::string& new_message) -> Description&
{
  _pimpl->message = std::move(new_message);
  return *this;
}
//==============================================================================
auto SMS::Description::contact() const -> const ContactCard&
{
  return _pimpl->contact;
}

//==============================================================================
auto SMS::Description::contact(
  ContactCard new_contact)-> Description&
{
  _pimpl->contact = std::move(new_contact);
  return *this;
}

//==============================================================================
rmf_traffic::Duration SMS::Description::sms_duration_estimate() const
{
  return _pimpl->sms_duration_estimate;
}

//==============================================================================
auto SMS::Description::sms_duration_estimate(
  rmf_traffic::Duration new_duration) -> Description&
{
  _pimpl->sms_duration_estimate = new_duration;
  return *this;
}

//==============================================================================
Activity::ConstModelPtr SMS::Description::make_model(
  State invariant_initial_state,
  const Parameters& parameters) const
{
  const auto description =
    WaitFor::Description::make(_pimpl->sms_duration_estimate);
  return description->make_model(invariant_initial_state, parameters);
}

//==============================================================================
Header SMS::Description::generate_header(
  const State&, const Parameters&) const
{
  const std::string& number = "+" +
    std::to_string(_pimpl->contact.number().country_code()) +
    " " + std::to_string(_pimpl->contact.number().number());

  return Header(
    "Sending SMS",
    "Sending SMS to " + _pimpl->contact.name() + " at " + number,
    _pimpl->sms_duration_estimate);
}

//==============================================================================
SMS::Description::Description()
{
  // Do nothing
}

} // namespace phases
} // namespace rmf_task_sequence
