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

#ifndef RMF_TASK__EVENT_HPP
#define RMF_TASK__EVENT_HPP

#include <rmf_task/Log.hpp>
#include <rmf_task/VersionedString.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace rmf_task {

//==============================================================================
class Event
{
public:

  /// A simple computer-friendly indicator of the current status of this
  /// event. This enum may be used to automatically identify when an
  /// event requires special attention, e.g. logging a warning or alerting
  /// an operator.
  enum class Status : uint32_t
  {
    /// The event status has not been initialized. This is a sentinel value
    /// that should not generally be used.
    Uninitialized = 0,

    /// The event is on standby. It cannot be started yet, and that is its
    /// expected status.
    Standby,

    /// The event is underway, and proceeding as expected.
    Underway,

    /// The event is underway but it has been temporarily delayed.
    Delayed,

    /// The event is underway but it has been blocked. The blockage may
    /// require manual intervention to fix.
    Blocked,

    /// An error has occurred that the Task implementation does not know how to
    /// deal with. Manual intervention is needed to get the task back on track.
    Error,

    /// An operator has instructed this event to be skipped
    Skipped,

    /// An operator has instructed this event to be canceled
    Canceled,

    /// An operator has instructed this event to be killed
    Killed,

    /// The event cannot ever finish correctly, even with manual intervention.
    /// This may mean that the Task cannot be completed if it does not have
    /// an automated way to recover from this failure state.
    Failed,

    /// The event is finished.
    Finished
  };

  class Active;
  using ConstActivePtr = std::shared_ptr<const Active>;

  class Snapshot;
  using ConstSnapshotPtr = std::shared_ptr<const Snapshot>;

};

//==============================================================================
/// The interface to an active event.
class Event::Active
{
public:

  using Status = Event::Status;
  using ConstActivePtr = Event::ConstActivePtr;

  /// The current Status of this event
  virtual Status status() const = 0;

  /// Simple wrapper for identifying when an event is finished
  inline bool finished() const { return status() == Status::Finished; }

  /// The "name" of this event. Ideally a short, simple piece of text that
  /// helps a human being intuit what this event is expecting at a glance.
  virtual VersionedString::View name() const = 0;

  /// A detailed explanation of this event.
  virtual VersionedString::View detail() const = 0;

  /// A view of the log for this event.
  virtual Log::View log() const = 0;

  /// Get more granular dependencies of this event, if any exist.
  virtual std::vector<ConstActivePtr> dependencies() const = 0;

  // Virtual destructor
  virtual ~Active() = default;
};

//==============================================================================
/// A snapshot of the state of an event. This snapshot can be read while the
/// original event is arbitrarily changed, and there is no risk of a race
/// condition, as long as the snapshot is not being created while the event
/// is changing.
class Event::Snapshot : public Event::Active
{
public:

  /// Make a snapshot of the current state of an Event
  static ConstSnapshotPtr make(const Active& other);

  // Documentation inherited
  Status status() const final;

  // Documentation inherited
  VersionedString::View name() const final;

  // Documentation inherited
  VersionedString::View detail() const final;

  // Documentation inherited
  Log::View log() const final;

  // Documentation inherited
  std::vector<ConstActivePtr> dependencies() const final;

  class Implementation;
private:
  Snapshot();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_task

#endif // RMF_TASK__EVENT_HPP
