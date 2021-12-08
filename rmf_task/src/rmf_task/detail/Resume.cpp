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

#include "internal_Resume.hpp"

namespace rmf_task {
namespace detail {

//==============================================================================
Resume Resume::make(std::function<void()> callback)
{
  return Implementation::make(std::move(callback));
}

//==============================================================================
void Resume::operator()() const
{
  _pimpl->trigger();
}

//==============================================================================
Resume::Resume()
{
  // Do nothing
}

//==============================================================================
void Resume::Implementation::trigger() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex);
  if (!called)
  {
    called = true;
    callback();
  }
}

//==============================================================================
Resume Resume::Implementation::make(std::function<void()> callback)
{
  Resume resumer;
  resumer._pimpl =
    rmf_utils::make_unique_impl<Implementation>(std::move(callback));

  return resumer;
}

//==============================================================================
Resume::Implementation::Implementation(std::function<void()> callback_)
: callback(std::move(callback_))
{
  // Do nothing
}

//==============================================================================
Resume::Implementation::~Implementation()
{
  trigger();
}

} // namespace detail
} // namespace rmf_task
