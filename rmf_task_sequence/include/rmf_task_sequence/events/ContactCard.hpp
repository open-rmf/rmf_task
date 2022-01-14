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

#ifndef RMF_TASK_SEQUENCE__EVENTS__CONTACTCARD_HPP
#define RMF_TASK_SEQUENCE__EVENTS__CONTACTCARD_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <optional>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
/// Store contact details for telecommunication
class ContactCard
{
public:

  class PhoneNumber
  {
  public:

    /// Constructor
    ///
    /// \param[in] country_code
    ///   The country code without "+""
    ///
    /// \param[in] number
    ///   The phone number
    PhoneNumber(uint32_t country_code, uint32_t number);

    /// Get the country code
    uint32_t country_code() const;

    /// Set the country code
    PhoneNumber& country_code(uint32_t new_code);

    /// Get the phone number
    uint32_t number() const;

    /// Set the phone number
    PhoneNumber& number(uint32_t new_number);

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

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
  /// \param[in] number
  ///   The phone number of the contact
  ContactCard(
    const std::string& name,
    std::optional<std::string> address,
    std::optional<std::string> email,
    PhoneNumber number);

  /// Get the name
  const std::string& name() const;

  /// Set the name
  ContactCard& name(const std::string& new_name);

  /// Get the address
  std::optional<std::string> address() const;

  /// Set the address
  ContactCard& address(std::optional<std::string> new_address);

  /// Get the email
  std::optional<std::string> email() const;

  /// Set the email
  ContactCard& email(std::optional<std::string> new_email);

  /// Get the phone number
  const PhoneNumber& number() const;

  /// Set the phone number
  ContactCard& number(PhoneNumber new_number);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace events
} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__EVENTS__CONTACTCARD_HPP
