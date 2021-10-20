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

#include "detail/internal_Resume.hpp"

#include <rmf_task/Task.hpp>

namespace rmf_task {

//==============================================================================
class Task::Booking::Implementation
{
public:
  std::string id;
  rmf_traffic::Time earliest_start_time;
  rmf_task::ConstPriorityPtr priority;
  bool automatic;
};

//==============================================================================
Task::Booking::Booking(
  std::string id_,
  rmf_traffic::Time earliest_start_time_,
  ConstPriorityPtr priority_,
  bool automatic_)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(id_),
        earliest_start_time_,
        std::move(priority_),
        automatic_
      }))
{
  // Do nothing
}

//==============================================================================
const std::string& Task::Booking::id() const
{
  return _pimpl->id;
}

//==============================================================================
rmf_traffic::Time Task::Booking::earliest_start_time() const
{
  return _pimpl->earliest_start_time;
}

//==============================================================================
ConstPriorityPtr Task::Booking::priority() const
{
  return _pimpl->priority;
}

//==============================================================================
bool Task::Booking::automatic() const
{
  return _pimpl->automatic;
}

//==============================================================================
class Task::Tag::Implementation
{
public:
  ConstBookingPtr booking;
  Header header;
};

//==============================================================================
Task::Tag::Tag(
  ConstBookingPtr booking_,
  Header header_)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(booking_),
        std::move(header_)
      }))
{
  // Do nothing
}

//==============================================================================
auto Task::Tag::booking() const -> const ConstBookingPtr&
{
  return _pimpl->booking;
}

//==============================================================================
const Header& Task::Tag::header() const
{
  return _pimpl->header;
}

//==============================================================================
Task::Active::Resume Task::Active::make_resumer(std::function<void()> callback)
{
  return detail::Resume::Implementation::make(std::move(callback));
}

} // namespace rmf_task
