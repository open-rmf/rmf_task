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

#ifndef RMF_TASK__VERSIONEDSTRING_HPP
#define RMF_TASK__VERSIONEDSTRING_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <string>

namespace rmf_task {

//==============================================================================
class VersionedString
{
public:

  class View;
  class Reader;

  /// Construct a versioned string
  ///
  /// \param[in] initial_value
  ///   The initial value of this versioned string
  VersionedString(std::string initial_value);

  /// Update the value of this versioned string
  ///
  /// \param[in] new_value
  ///   The new value for this versioned string
  void update(std::string new_value);

  /// Get a view of the current version of the string
  View view() const;

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// A snapshot view of a VersionedString. This is thread-safe to read even while
/// the VersionedString is being modified. Each VersionedString::Reader instance
/// will only view this object once; after the first viewing it will return a
/// nullptr.
///
/// The contents of this View can only be retrieved by a VersionedString::Reader
class VersionedString::View
{
public:
  class Implementation;
private:
  View();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class VersionedString::Reader
{
public:

  /// Construct a Reader
  Reader();

  /// Read from the View.
  ///
  /// If this Reader has never seen this View before, then this function will
  /// return a reference to the string that the View contains. Otherwise, if
  /// this Reader has seen this View before, then this function will return a
  /// nullptr.
  ///
  /// \param[in] view
  ///   The view that the Reader should look at
  std::shared_ptr<const std::string> read(const View& view);

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_task

#endif // RMF_TASK__VERSIONEDSTRING_HPP
