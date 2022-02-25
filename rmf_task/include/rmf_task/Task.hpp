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

#ifndef RMF_TASK__TASK_HPP
#define RMF_TASK__TASK_HPP

#include <rmf_task/Header.hpp>
#include <rmf_task/Phase.hpp>
#include <rmf_task/detail/Backup.hpp>
#include <rmf_task/detail/Resume.hpp>
#include <rmf_task/Constraints.hpp>
#include <rmf_task/Parameters.hpp>
#include <rmf_task/State.hpp>
#include <rmf_task/Estimate.hpp>
#include <rmf_task/Priority.hpp>

#include <rmf_traffic/Time.hpp>

#include <memory>
#include <functional>

namespace rmf_task {

//==============================================================================
/// Pure abstract interface for an executable Task
class Task
{
public:

  // Declarations
  class Booking;
  using ConstBookingPtr = std::shared_ptr<const Booking>;

  class Tag;
  using ConstTagPtr = std::shared_ptr<const Tag>;

  class Model;
  using ConstModelPtr = std::shared_ptr<const Model>;

  class Description;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;

  class Active;
  using ActivePtr = std::shared_ptr<Active>;
};

//==============================================================================
/// Basic information about how the task was booked, e.g. what its name is,
/// when it should start, and what its priority is.
class Task::Booking
{
public:

  /// Constructor
  ///
  /// \param[in] id_
  ///   The identity of the booking
  ///
  /// \param[in] earliest_start_time_
  ///   The earliest time that the task may begin
  ///
  /// \param[in] priority_
  ///   The priority of the booking
  ///
  /// \param[in] automatic_
  ///   Whether this booking was automatically generated
  Booking(
    std::string id_,
    rmf_traffic::Time earliest_start_time_,
    ConstPriorityPtr priority_,
    bool automatic_ = false);

  /// The unique id for this booking
  const std::string& id() const;

  /// Get the earliest time that this booking may begin
  rmf_traffic::Time earliest_start_time() const;

  /// Get the priority of this booking
  ConstPriorityPtr priority() const;

  // Returns true if this booking was automatically generated
  bool automatic() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// Basic static information about the task.
class Task::Tag
{
public:

  /// Constructor
  Tag(
    ConstBookingPtr booking_,
    Header header_);

  /// The booking information of the request that this Task is carrying out
  const ConstBookingPtr& booking() const;

  /// The header for this Task
  const Header& header() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// An abstract interface for computing the estimate and invariant durations
/// of this request
class Task::Model
{
public:

  /// Estimate the state of the robot when the task is finished along with
  /// the time the robot has to wait before commencing the task
  virtual std::optional<Estimate> estimate_finish(
    const State& initial_state,
    const Constraints& task_planning_constraints,
    const TravelEstimator& travel_estimator) const = 0;

  /// Estimate the invariant component of the task's duration
  virtual rmf_traffic::Duration invariant_duration() const = 0;

  virtual ~Model() = default;
};

//==============================================================================
/// An abstract interface to define the specifics of this task. This
/// implemented description will differentiate this task from others.
class Task::Description
{
public:

  /// Generate a Model for the task based on the unique traits of this
  /// description
  ///
  /// \param[in] earliest_start_time
  ///   The earliest time this task should begin execution. This is usually
  ///   the requested start time for the task.
  ///
  /// \param[in] parameters
  ///   The parameters that describe this AGV
  virtual ConstModelPtr make_model(
    rmf_traffic::Time earliest_start_time,
    const Parameters& parameters) const = 0;

  struct Info
  {
    std::string category;
    std::string detail;
  };

  /// Generate a plain text info description for the task, given the predicted
  /// initial state and the task planning parameters.
  ///
  /// \param[in] initial_state
  ///   The predicted initial state for the task
  ///
  /// \param[in] parameters
  ///   The task planning parameters
  virtual Info generate_info(
    const State& initial_state,
    const Parameters& parameters) const = 0;

  // Virtual destructor
  virtual ~Description() = default;
};

//==============================================================================
class Task::Active
{
public:
  /// Backup data for the task. The state of the task is represented by a
  /// string. The meaning and format of the string is up to the Task
  /// implementation to decide.
  ///
  /// Each Backup is tagged with a sequence number. As the Task makes progress,
  /// it can issue new Backups with higher sequence numbers. Only the Backup
  /// with the highest sequence number will be kept.
  using Backup = detail::Backup;

  /// Get a quick overview status of how the task is going
  virtual Event::Status status_overview() const = 0;

  /// Check if this task is finished, which could include successful completion
  /// or cancellation.
  virtual bool finished() const = 0;

  /// Descriptions of the phases that have been completed
  virtual const std::vector<Phase::ConstCompletedPtr>&
  completed_phases() const = 0;

  /// Interface for the phase that is currently active
  virtual Phase::ConstActivePtr active_phase() const = 0;

  /// Time that the current active phase started
  virtual std::optional<rmf_traffic::Time> active_phase_start_time() const = 0;

  /// Descriptions of the phases that are expected in the future
  virtual const std::vector<Phase::Pending>& pending_phases() const = 0;

  /// The tag of this Task
  virtual const ConstTagPtr& tag() const = 0;

  /// Estimate the overall finishing time of the task
  virtual rmf_traffic::Duration estimate_remaining_time() const = 0;

  /// Get a backup for this Task
  virtual Backup backup() const = 0;

  /// The Resume class keeps track of when the Task is allowed to Resume.
  /// You can either call the Resume object's operator() or let the object
  /// expire to tell the Task that it may resume.
  using Resume = detail::Resume;

  /// Tell this Task that it needs to be interrupted. An interruption means
  /// the robot may be commanded to do other tasks before this task resumes.
  ///
  /// Interruptions may occur to allow operators to take manual control of the
  /// robot, or to engage automatic behaviors in response to emergencies, e.g.
  /// fire alarms or code blues.
  ///
  /// \param[in] task_is_interrupted
  ///   This callback will be triggered when the Task has reached a state where
  ///   it is okay to start issuing other commands to the robot.
  ///
  /// \return an object to inform the Task when it is allowed to resume.
  virtual Resume interrupt(std::function<void()> task_is_interrupted) = 0;

  // TODO(MXG): Should we have a pause() interface? It would be the same as
  // interrupt() except without the expectation that the robot will do any other
  // task before resuming.

  /// Tell the Task that it has been canceled. The behavior that follows a
  /// cancellation will vary between different Tasks, but generally it means
  /// that the robot should no longer try to complete its Task and should
  /// instead try to return itself to an unencumbered state as quickly as
  /// possible.
  ///
  /// The Task may continue to perform some phases after being canceled. The
  /// pending_phases are likely to change after the Task is canceled, being
  /// replaced with phases that will help to relieve the robot so it can
  /// return to an unencumbered state.
  ///
  /// The Task should continue to be tracked as normal. When its finished
  /// callback is triggered, the cancellation is complete.
  virtual void cancel() = 0;

  /// Kill this Task. The behavior that follows a kill will vary between
  /// different Tasks, but generally it means that the robot should be returned
  /// to a safe idle state as soon as possible, even if it remains encumbered by
  /// something related to this Task.
  ///
  /// The Task should continue to be tracked as normal. When its finished
  /// callback is triggered, the killing is complete.
  ///
  /// The kill() command supersedes the cancel() command. Calling cancel() after
  /// calling kill() will have no effect.
  virtual void kill() = 0;

  /// Skip a specific phase within the task. This can be issued by operators if
  /// manual intervention is needed to unblock a task.
  ///
  /// If a pending phase is specified, that phase will be skipped when the Task
  /// reaches it.
  ///
  /// \param[in] phase_id
  ///   The ID of the phase that should be skipped.
  ///
  /// \param[in] value
  ///   True if the phase should be skipped, false otherwise.
  virtual void skip(uint64_t phase_id, bool value = true) = 0;

  /// Rewind the Task to a specific phase. This can be issued by operators if
  /// a phase did not actually go as intended and needs to be repeated.
  ///
  /// It is possible that the Task will rewind further back than the specified
  /// phase_id if the specified phase depends on an earlier one. This is up to
  /// the discretion of the Task implementation.
  virtual void rewind(uint64_t phase_id) = 0;

  // Virtual destructor
  virtual ~Active() = default;

protected:

  /// Used by classes that inherit the Task interface to create a Resumer object
  ///
  /// \param[in] callback
  ///   Provide the callback that should be triggered when the Task is allowed
  ///   to resume
  static Resume make_resumer(std::function<void()> callback);
};

} // namespace rmf_task

#endif // RMF_TASK__TASK_HPP
