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
/// This class is the simplest possible implementation for directly managing the
/// required fields of the Event interface.
///
/// This may be useful if you have a Phase implementation that takes care of the
/// logic for tracking your event(s) but you still need an Event object to
/// satisfy the Phase interface's finish_event() function. Your Phase
/// implementation can create an instance of this class and then manage its
/// fields directly.
class SimpleEvent : public Event
{
public:

  static std::shared_ptr<SimpleEvent> make(
    std::string name,
    std::string detail,
    Status initial_status,
    std::vector<ConstEventPtr> dependencies = {});

  // Documentation inherited
  Status status() const final;

  /// Update the status of this event
  SimpleEvent& update_status(Status new_status);

  // Documentation inherited
  VersionedString::View name() const final;

  /// Update the name of this event
  SimpleEvent& update_name(std::string new_name);

  // Documentation inherited
  VersionedString::View detail() const final;

  /// Update the detail of this event
  SimpleEvent& update_detail(std::string new_detail);

  // Documentation inherited
  Log::View log() const final;

  /// Update the log
  Log& update_log();

  // Documentation inherited
  std::vector<ConstEventPtr> dependencies() const final;

  /// Update the dependencies
  SimpleEvent& update_dependencies(std::vector<ConstEventPtr> new_dependencies);

  class Implementation;
private:
  SimpleEvent();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace events
} // namespace rmf_task

#endif // RMF_TASK__EVENTS__SIMPLEEVENT_HPP
