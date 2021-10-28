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

#ifndef RMF_TASK_SEQUENCE__DETAIL__CONTACTCARD_HPP
#define RMF_TASK_SEQUENCE__DETAIL__CONTACTCARD_HPP

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_task_sequence {
namespace detail {

//==============================================================================
struct PhoneNumber
{
  /// The country code without "+""
  uint32_t country_code;

  /// The phone number
  uint32_t number;
};

//==============================================================================
/// Store contact details for telecommunication
class ContactCard
{
public:

  /// Constructor
  ///
  /// \param[in] name
  ///   The name of the contact
  ///
  /// \param[in] address
  ///   The physical address of the contact
  ///
  /// \param[in] email
  ///   The email address of the contact
  ///
  /// \param[in] country_code
  ///   The country code
  ContactCard(
    const std::string& name,
    const std::string& address,
    const std::string& email,
    PhoneNumber number);

  /// Get the name
  const std::string& name() const;

  /// Set the name
  ContactCard& name(const std::string& new_name);

  /// Get the address
  const std::string& address() const;

  /// Set the address
  ContactCard& address(const std::string& new_address);

  /// Get the email
  const std::string& email() const;

  /// Set the email
  ContactCard& email(const std::string& new_email);

  /// Get the phone number
  const PhoneNumber& number() const;

  /// Set the phone number
  ContactCard& number(PhoneNumber new_number);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace detail
} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__DETAIL__CONTACTCARD_HPP
