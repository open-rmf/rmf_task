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

#ifndef RMF_TASK__COMPOSITEDATA_HPP
#define RMF_TASK__COMPOSITEDATA_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <any>
#include <typeindex>

namespace rmf_task {

//==============================================================================
class CompositeData
{
public:

  /// Create an empty CompositeData
  CompositeData();

  /// The result of performing an insertion operation
  template<typename T>
  struct InsertResult
  {
    /// True if the value was inserted. This means that an entry of value T did
    /// not already exist before you performed the insertion.
    bool inserted;

    /// A reference to the value of type T that currently exists within the
    /// CompositeData.
    T* value;
  };

  /// Attempt to insert some data structure into the CompositeData. If a data
  /// structure of type T already exists in the CompositeData, then this
  /// function will have no effect, and InsertResult<T>::value will point to
  /// the value that already existed in the CompositeData.
  ///
  /// \param[in] value
  ///   The value to attempt to insert.
  template<typename T>
  InsertResult<T> insert(T&& value);

  /// Insert or assign some data structure into the CompositeData. If a data
  /// structure of type T already exists in the CompositeData, then this
  /// function will overwrite it with the new value.
  ///
  /// \param[in] value
  ///   The value to insert or assign.
  template<typename T>
  InsertResult<T> insert_or_assign(T&& value);

  /// Get a reference to a data structure of type T if one is available in the
  /// CompositeData. If one is not available, this will return a nullptr.
  template<typename T>
  T* get();

  /// Get a reference to an immutable data structure of type T if one is
  /// available in the CompositeData. If one is not available, this will return
  /// a nullptr.
  template<typename T>
  const T* get() const;

  /// Erase the data structure of type T if one is available in the
  /// CompositeData. This will return true if it was erased, or false if type T
  /// was not available.
  template<typename T>
  bool erase();

  /// Remove all data structures from this CompositeData
  void clear();

  class Implementation;
private:
  std::any* _get(std::type_index type);
  const std::any* _get(std::type_index type) const;
  InsertResult<std::any> _insert(std::any value, bool or_assign);
  bool _erase(std::type_index type);
  rmf_utils::impl_ptr<Implementation> _pimpl;
};


} // namespace rmf_task

#include <rmf_task/detail/impl_CompositeData.hpp>

#endif // RMF_TASK__COMPOSITEDATA_HPP
