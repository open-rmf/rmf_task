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

#include <rmf_task_sequence/detail/ContactCard.hpp>

namespace rmf_task_sequence {
namespace detail {

//==============================================================================
class ContactCard::Implementation
{
public:

  std::string name;
  std::string address;
  std::string email;
  PhoneNumber number;
};

//==============================================================================
ContactCard::ContactCard(
  const std::string& name,
  const std::string& address,
  const std::string& email,
  PhoneNumber number)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        name,
        address,
        email,
        std::move(number)
      }))
{
  // Do nothing
}

//==============================================================================
const std::string& ContactCard::name() const
{
  return _pimpl->name;
}

//==============================================================================
ContactCard& ContactCard::name(const std::string& new_name)
{
  _pimpl->name = new_name;
  return *this;
}

//==============================================================================
const std::string& ContactCard::address() const
{
  return _pimpl->address;
}

//==============================================================================
ContactCard& ContactCard::address(const std::string& new_address)
{
  _pimpl->address = new_address;
  return *this;
}

//==============================================================================
const std::string& ContactCard::email() const
{
  return _pimpl->email;
}

//==============================================================================
ContactCard& ContactCard::email(const std::string& new_email)
{
  _pimpl->email = new_email;
  return *this;
}

//==============================================================================
const ContactCard::PhoneNumber& ContactCard::number() const
{
  return _pimpl->number;
}

//==============================================================================
ContactCard& ContactCard::number(ContactCard::PhoneNumber new_number)
{
  _pimpl->number = std::move(new_number);
  return *this;
}

} // namespace detail
} // namespace rmf_task_sequence
