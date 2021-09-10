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

#ifndef RMF_TASK__EXECUTE__CONDITION_HPP
#define RMF_TASK__EXECUTE__CONDITION_HPP

#include <rmf_task/Log.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace rmf_task {

class Condition;
using ConstConditionPtr = std::shared_ptr<const Condition>;

//==============================================================================
class Condition
{
public:

  /// A simple computer-friendly indicator of the current status of this
  /// condition. This enum may be used to automatically identify when a
  /// condition requires special attention, e.g. logging a warning or alerting
  /// an operator.
  enum class Status : uint32_t
  {
    /// The condition status has not been initialized. This is a sentinel value
    /// that should not generally be used.
    Uninitialized = 0,

    /// The condition is on standby. It cannot be satisfied yet, and that is its
    /// expected status.
    Standby,

    /// The condition is underway, and coming along as expected.
    Underway,

    /// The condition is underway but it has been temporarily delayed.
    Delayed,

    /// The condition is underway but it has been blocked. The blockage may
    /// require manual intervention to fix.
    Blocked,

    /// An error has occurred that the Task implementation does not know how to
    /// deal with. Manual intervention is needed to get the task back on track.
    Error,

    /// An operator has instructed this condition to be skipped
    Skipped,

    /// An operator has instructed this condition to be canceled
    Canceled,

    /// An operator has instructed this condition to be killed
    Killed,

    /// The condition cannot ever be satisfied, even with manual intervention.
    /// This may mean that the Task cannot be completed if it does not have
    /// an automated way to recover from this failure state.
    Failed,

    /// The condition is satisfied.
    Finished
  };

  /// The current Status of this condition
  virtual Status status() const = 0;

  /// Simple wrapper for identifying when a condition is finished
  inline bool finished() const { return status() == Status::Finished; }

  /// The "name" of this condition. Ideally a short, simple piece of text that
  /// helps a human being intuit what this condition is expecting at a glance.
  virtual std::string name() const = 0;

  /// A detailed explanation of this condition.
  virtual std::string detail() const = 0;

  /// A view of the event log for this condition.
  virtual Log::View log() const = 0;

  /// Get more granular subconditions of this condition, if any exist.
  virtual std::vector<ConstConditionPtr> subconditions() const = 0;

  class Snapshot;

  // Virtual destructor
  virtual ~Condition() = default;
};

//==============================================================================
/// A snapshot of the state of a condition. This snapshot can be read while the
/// original condition is arbitrarily changed, and there is no risk of a race
/// condition, as long as the snapshot is not being created while the condition
/// is changing.
class Condition::Snapshot : public Condition
{
public:

  /// Make a snapshot of the current state of a Condition
  static ConstConditionPtr make(const Condition& other);

  // Documentation inherited
  Status status() const final;

  // Documentation inherited
  std::string name() const final;

  // Documentation inherited
  std::string detail() const final;

  // Documentation inherited
  Log::View log() const final;

  // Documentation inherited
  std::vector<ConstConditionPtr> subconditions() const final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_task

#endif // RMF_TASK__EXECUTE__CONDITION_HPP
