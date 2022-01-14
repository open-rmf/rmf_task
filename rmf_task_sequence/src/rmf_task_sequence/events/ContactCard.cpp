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

#include <rmf_task_sequence/events/ContactCard.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
class ContactCard::PhoneNumber::Implementation
{
public:

  uint32_t country_code;
  uint32_t number;
};

//==============================================================================
ContactCard::PhoneNumber::PhoneNumber(
  uint32_t country_code,
  uint32_t number)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        country_code,
        number
      }))
{
  // Do nothing
}

//==============================================================================
uint32_t ContactCard::PhoneNumber::country_code() const
{
  return _pimpl->country_code;
}

//==============================================================================
auto ContactCard::PhoneNumber::country_code(uint32_t new_code)
-> PhoneNumber&
{
  _pimpl->country_code = new_code;
  return *this;
}

//==============================================================================
uint32_t ContactCard::PhoneNumber::number() const
{
  return _pimpl->number;
}

//==============================================================================
auto ContactCard::PhoneNumber::number(uint32_t new_number)
-> PhoneNumber&
{
  _pimpl->number = new_number;
  return *this;
}

//==============================================================================
class ContactCard::Implementation
{
public:

  std::string name;
  std::optional<std::string> address;
  std::optional<std::string> email;
  PhoneNumber number;
};

//==============================================================================
ContactCard::ContactCard(
  const std::string& name,
  std::optional<std::string> address,
  std::optional< std::string> email,
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
std::optional<std::string> ContactCard::address() const
{
  return _pimpl->address;
}

//==============================================================================
ContactCard& ContactCard::address(std::optional<std::string> new_address)
{
  _pimpl->address = new_address;
  return *this;
}

//==============================================================================
std::optional<std::string> ContactCard::email() const
{
  return _pimpl->email;
}

//==============================================================================
ContactCard& ContactCard::email(std::optional<std::string> new_email)
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

} // namespace events
} // namespace rmf_task_sequence
