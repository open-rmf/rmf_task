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

#ifndef RMF_TASK__ACTIVATOR_HPP
#define RMF_TASK__ACTIVATOR_HPP

#include <rmf_task/Request.hpp>

namespace rmf_task {

//==============================================================================
/// A factory for generating Task::Active instances from requests.
class Activator
{
public:

  /// Construct an empty TaskFactory
  Activator();

  /// Signature for activating a task
  ///
  /// \tparam Description
  ///   A class that implements the Task::Description interface
  ///
  /// \param[in] get_state
  ///   A callback for retrieving the current state of the robot
  ///
  /// \param[in] parameters
  ///   A reference to the parameters for the robot
  ///
  /// \param[in] booking
  ///   An immutable reference to the booking information for the task
  ///
  /// \param[in] description
  ///   The down-casted description of the task
  ///
  /// \param[in] backup_state
  ///   The serialized backup state of the Task, if the Task is being restored
  ///   from a crash or disconnection. If the Task is not being restored, a
  ///   std::nullopt will be passed in here.
  ///
  /// \param[in] update
  ///   A callback that will be triggered when the task has a significant
  ///   update in its status.
  ///
  /// \param[in] checkpoint
  ///   A callback that will be triggered when the task has reached a task
  ///   checkpoint whose state is worth backing up.
  ///
  /// \param[in] finished
  ///   A callback that will be triggered when the task has finished.
  ///
  /// \return an active, running instance of the requested task.
  template<typename Description>
  using Activate =
    std::function<
    Task::ActivePtr(
      const std::function<State()>& get_state,
      const ConstParametersPtr& parameters,
      const Task::ConstBookingPtr& booking,
      const Description& description,
      std::optional<std::string> backup_state,
      std::function<void(Phase::ConstSnapshotPtr)> update,
      std::function<void(Task::Active::Backup)> checkpoint,
      std::function<void(Phase::ConstCompletedPtr)> phase_finished,
      std::function<void()> task_finished)
    >;

  /// Add a callback to convert from a Description into an active Task.
  ///
  /// \tparam Description
  ///   A class that implements the Request::Description interface
  ///
  /// \param[in] activator
  ///   A callback that activates a Task matching the Description
  template<typename Description>
  void add_activator(Activate<Description> activator);

  /// Activate a Task object based on a Request.
  ///
  /// \param[in] get_state
  ///   A callback for retrieving the current state of the robot
  ///
  /// \param[in] parameters
  ///   A reference to the parameters for the robot
  ///
  /// \param[in] request
  ///   The task request
  ///
  /// \param[in] update
  ///   A callback that will be triggered when the task has a significant update
  ///
  /// \param[in] checkpoint
  ///   A callback that will be triggered when the task has reached a task
  ///   checkpoint whose state is worth backing up.
  ///
  /// \param[in] phase_finished
  ///   A callback that will be triggered whenever a task phase is finished
  ///
  /// \param[in] task_finished
  ///   A callback that will be triggered when the task has finished
  ///
  /// \return an active, running instance of the requested task.
  Task::ActivePtr activate(
    const std::function<State()>& get_state,
    const ConstParametersPtr& parameters,
    const Request& request,
    std::function<void(Phase::ConstSnapshotPtr)> update,
    std::function<void(Task::Active::Backup)> checkpoint,
    std::function<void(Phase::ConstCompletedPtr)> phase_finished,
    std::function<void()> task_finished) const;

  /// Restore a Task that crashed or disconnected.
  ///
  /// \param[in] get_state
  ///   A callback for retrieving the current state of the robot
  ///
  /// \param[in] parameters
  ///   A reference to the parameters for the robot
  ///
  /// \param[in] request
  ///   The task request
  ///
  /// \param[in] backup_state
  ///   The serialized backup state of the Task
  ///
  /// \param[in] update
  ///   A callback that will be triggered when the task has a significant update
  ///
  /// \param[in] checkpoint
  ///   A callback that will be triggered when the task has reached a task
  ///   checkpoint whose state is worth backing up.
  ///
  /// \param[in] phase_finished
  ///   A callback that will be triggered whenever a task phase is finished
  ///
  /// \param[in] task_finished
  ///   A callback that will be triggered when the task has finished
  ///
  /// \return an active, running instance of the requested task.
  Task::ActivePtr restore(
    const std::function<State()>& get_state,
    const ConstParametersPtr& parameters,
    const Request& request,
    std::string backup_state,
    std::function<void(Phase::ConstSnapshotPtr)> update,
    std::function<void(Task::Active::Backup)> checkpoint,
    std::function<void(Phase::ConstCompletedPtr)> phase_finished,
    std::function<void()> task_finished) const;

  class Implementation;
private:

  /// \private
  void _add_activator(
    std::type_index type,
    Activate<Task::Description> activator);

  rmf_utils::impl_ptr<Implementation> _pimpl;
};

using ActivatorPtr = std::shared_ptr<Activator>;
using ConstActivatorPtr = std::shared_ptr<const Activator>;

} // namespace rmf_task

#include <rmf_task/detail/impl_Activator.hpp>

#endif // RMF_TASK__ACTIVATOR_HPP
