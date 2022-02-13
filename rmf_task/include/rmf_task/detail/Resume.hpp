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

#ifndef RMF_TASK__DETAIL__RESUME_HPP
#define RMF_TASK__DETAIL__RESUME_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <functional>

namespace rmf_task {
namespace detail {

//==============================================================================
class Resume
{
public:

  /// Make a Resume object. The callback will be triggered when the user
  /// triggers the Resume.
  static Resume make(std::function<void()> callback);

  /// Call this object to tell the Task to resume.
  void operator()() const;

  class Implementation;
private:
  Resume();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace detail
} // namespace rmf_task

#endif // RMF_TASK__DETAIL__RESUME_HPP
