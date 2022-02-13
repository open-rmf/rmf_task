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

#ifndef RMF_TASK__DETAIL__IMPL_COMPOSITEDATA_HPP
#define RMF_TASK__DETAIL__IMPL_COMPOSITEDATA_HPP

#include <rmf_task/CompositeData.hpp>

namespace rmf_task {

namespace detail {
//==============================================================================
template<typename T>
CompositeData::InsertResult<T> insertion_cast(
  CompositeData::InsertResult<std::any> result)
{
  return {result.inserted, std::any_cast<T>(result.value)};
}
} // namespace detail

//==============================================================================
template<typename T>
auto CompositeData::insert(T&& value) -> InsertResult<T>
{
  return detail::insertion_cast<T>(_insert(std::any(std::move(value)), false));
}

//==============================================================================
template<typename T>
auto CompositeData::insert_or_assign(T&& value) -> InsertResult<T>
{
  return detail::insertion_cast<T>(_insert(std::any(std::move(value)), true));
}

//==============================================================================
template<typename T>
CompositeData& CompositeData::with(T&& value)
{
  insert_or_assign<T>(std::forward<T>(value));
  return *this;
}

//==============================================================================
template<typename T>
T* CompositeData::get()
{
  return std::any_cast<T>(_get(typeid(T)));
}

//==============================================================================
template<typename T>
const T* CompositeData::get() const
{
  return std::any_cast<T>(_get(typeid(T)));
}

//==============================================================================
template<typename T>
bool CompositeData::erase()
{
  return _erase(typeid(T));
}

} // namespace rmf_task

#endif // RMF_TASK__DETAIL__IMPL_COMPOSITEDATA_HPP
