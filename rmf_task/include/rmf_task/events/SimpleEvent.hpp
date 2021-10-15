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

#ifndef RMF_TASK__EVENTS__SIMPLEEVENT_HPP
#define RMF_TASK__EVENTS__SIMPLEEVENT_HPP

#include <rmf_task/Event.hpp>

namespace rmf_task {
namespace events {

//==============================================================================
class SimpleEvent : public Event
{
public:

  // Documentation inherited
  Status status() const final;

  /// Update the status of this event
  SimpleEvent& update_status(Status new_status);

  // Documentation inherited




};

} // namespace events
} // namespace rmf_task

#endif // RMF_TASK__EVENTS__SIMPLEEVENT_HPP
