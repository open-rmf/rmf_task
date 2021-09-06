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

#ifndef RMF_TASK__TASKFACTORY_HPP
#define RMF_TASK__TASKFACTORY_HPP

#include <rmf_task/Request.hpp>
#include <rmf_task/execute/Task.hpp>


namespace rmf_task {
namespace execute {

//==============================================================================
/// A factory for generating Task instances from requests.
class TaskFactory
{
public:

  /// Construct an empty TaskFactory
  TaskFactory();

  /// Signature for activating a task
  ///
  /// \tparam Description
  ///   A class that implements the Request::Description interface
  ///
  /// \param[in] request
  ///   An immutable reference to the relevant task request
  ///
  /// \param[in] description
  ///   The down-casted description of the task
  ///
  /// \param[in] update
  ///   A callback that will be triggered when the task has a significant
  ///   update in its status.
  ///
  /// \param[in] finished
  ///   A callback that will be triggered when the task has finished.
  ///
  /// \return an active, running instance of the requested task.
  template<typename Description>
  using Activate =
    std::function<
    execute::ConstTaskPtr(
      const Request::ConstTagPtr& request,
      const Description& description,
      std::function<void(ConstConditionPtr)> update,
      std::function<void(ConstConditionPtr)> finished)
    >;

  /// Add a callback to convert from a Request into an active Task.
  ///
  /// \tparam Description
  ///   A class that implements the Request::Description interface
  template<typename Description>
  void add_activator(Activate<Description> activator);

  /// Activate a Task object based on a Request::Description.
  ///
  /// \param[in] request
  ///   The task request
  ///
  /// \param[in] update
  ///   A callback that will be triggered when the task has a significant update
  ///
  /// \param[in] finished
  ///   A callback that will be triggered when the task has finished
  ///
  /// \return an active, running instance of the requested task.
  std::shared_ptr<Task> activate(
    const Request& request,
    std::function<void(ConstConditionPtr)> update,
    std::function<void(ConstConditionPtr)> finished);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace execute
} // namespace rmf_task

#endif // RMF_TASK__TASKFACTORY_HPP
