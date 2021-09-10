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


#ifndef SRC__RMF_TASK__DETAIL__INTERNAL_RESUME_HPP
#define SRC__RMF_TASK__DETAIL__INTERNAL_RESUME_HPP

#include <rmf_task/detail/Resume.hpp>

#include <functional>
#include <mutex>

namespace rmf_task {
namespace detail {

//==============================================================================
class Resume::Implementation
{
public:

  std::function<void()> callback;

  // We use a recursive mutex in case triggering the callback leads to a chain
  // that causes the Resume object to be triggered again or destroyed. We have
  // no way to prevent such a behavior from the implementation here, but we can
  // at least ensure that it does not cause a deadlock or an infinitely
  // recursive loop.
  mutable std::recursive_mutex mutex;
  mutable bool called = false;

  void trigger() const;

  static Resume make(std::function<void()> callback);

  Implementation(std::function<void()> callback);

  Implementation(const Implementation&) = delete;
  Implementation(Implementation&&) = delete;
  Implementation& operator=(const Implementation&) = delete;
  Implementation& operator=(Implementation&&) = delete;

  ~Implementation();
};

} // namespace detail
} // namespace rmf_task


#endif // SRC__RMF_TASK__DETAIL__INTERNAL_RESUME_HPP
