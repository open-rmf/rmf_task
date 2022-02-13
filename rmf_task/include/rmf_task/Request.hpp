/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef RMF_TASK__REQUEST_HPP
#define RMF_TASK__REQUEST_HPP

#include <memory>

#include <rmf_task/Task.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_task {

//==============================================================================
class Request
{
public:
  /// Constructor
  ///
  /// \param[in] earliest_start_time
  ///   The desired start time for this request
  ///
  /// \param[in] priority
  ///   The priority for this request. This is provided by the Priority Scheme.
  ///   For requests that do not have any priority this is a nullptr.
  ///
  /// \param[in] description
  ///   The description for this request
  ///
  /// \param[in] automatic
  ///   True if this request is auto-generated
  //
  // TODO(MXG): Deprecate this constructor?
  Request(
    const std::string& id,
    rmf_traffic::Time earliest_start_time,
    ConstPriorityPtr priority,
    Task::ConstDescriptionPtr description,
    bool automatic = false);

  /// Constructor
  ///
  /// \param[in] booking
  ///   Booking information for this request
  ///
  /// \param[in] description
  ///   Description for this request
  Request(
    Task::ConstBookingPtr booking,
    Task::ConstDescriptionPtr description);

  /// Get the tag of this request
  const Task::ConstBookingPtr& booking() const;

  /// Get the description of this request
  const Task::ConstDescriptionPtr& description() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

using RequestPtr = std::shared_ptr<Request>;
using ConstRequestPtr = std::shared_ptr<const Request>;

} // namespace rmf_task

#endif // RMF_TASK__REQUEST_HPP
