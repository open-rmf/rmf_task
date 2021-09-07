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

#ifndef RMF_TASK__EXECUTE__TASK_HPP
#define RMF_TASK__EXECUTE__TASK_HPP

#include <rmf_task/execute/Phase.hpp>
#include <rmf_task/detail/Backup.hpp>
#include <rmf_task/detail/Resume.hpp>
#include <rmf_task/Request.hpp>

#include <memory>
#include <functional>

namespace rmf_task {
namespace execute {

//==============================================================================
/// Pure abstract interface for an executable Task
class Task
{
public:

  // Declarations
  class Tag;

  /// Backup data for the task. The state of the task is represented by a
  /// string. The meaning and format of the string is up to the Task
  /// implementation to decide.
  ///
  /// Each Backup is tagged with a sequence number. As the Task makes progress,
  /// it can issue new Backups with higher sequence numbers. Only the Backup
  /// with the highest sequence number will be kept.
  class Backup : public detail::Backup {};

  /// Descriptions of the phases that have been completed
  virtual const std::vector<Phase::ConstCompletedPtr>&
  completed_phases() const = 0;

  /// Interface for the phase that is currently active
  virtual Phase::ConstActivePtr active_phase() const = 0;

  /// Descriptions of the phases that are expected in the future
  virtual std::vector<Phase::Pending> pending_phases() const = 0;

  /// The request tag of this Task
  virtual const Request::ConstTagPtr& tag() const = 0;

  /// Estimate the overall finishing time of the task
  virtual rmf_traffic::Time estimate_finish_time() const = 0;

  /// Get a backup for this Task
  virtual Backup backup() const = 0;

  /// The Resume class keeps track of when the Task is allowed to Resume.
  /// You can either call the Resume object's operator() or let the object
  /// expire to tell the Task that it may resume.
  class Resume : public detail::Resume {};

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
  virtual void skip(uint64_t phase_id, bool value=true) = 0;

  /// Rewind the Task to a specific phase. This can be issued by operators if
  /// a phase did not actually go as intended and needs to be repeated.
  ///
  /// It is possible that the Task will rewind further back than the specified
  /// phase_id if the specified phase depends on an earlier one. This is up to
  /// the discretion of the Task implementation.
  virtual void rewind(uint64_t phase_id) = 0;

  // Virtual destructor
  virtual ~Task() = default;

protected:

  /// Used by classes that inherit the Task interface to create a Resumer object
  ///
  /// \param[in] callback
  ///   Provide the callback that should be triggered when the Task is allowed
  ///   to resume
  static Resume make_resumer(std::function<void()> callback);
};

using ConstTaskPtr = std::shared_ptr<const Task>;

//==============================================================================
/// Basic static information about the task.
class Task::Tag
{
public:

  /// The original request that this Task is carrying out
  const Request::ConstTagPtr& request() const;

  /// The category of this Task.
  const std::string& category() const;

  /// Details about this Task.
  const std::string& detail() const;

  /// The original finish estimate of this Task.
  rmf_traffic::Time original_finish_estimate() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace execute
} // namespace rmf_task

#endif // RMF_TASK__TASK_HPP
