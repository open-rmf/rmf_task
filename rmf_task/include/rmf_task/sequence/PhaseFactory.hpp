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

#ifndef RMF_TASK__SEQUENCE__PHASEFACTORY_HPP
#define RMF_TASK__SEQUENCE__PHASEFACTORY_HPP

#include <rmf_task/execute/Phase.hpp>
#include <rmf_task/sequence/Task.hpp>
#include <rmf_task/sequence/TaskDescription.hpp>

namespace rmf_task {
namespace sequence {

//==============================================================================
/// A factory for generating execute::Phase instances from descriptions.
class PhaseFactory
{
public:

  /// Construct an empty PhaseFactory
  PhaseFactory();

  /// Signature for activating a phase
  ///
  /// \tparam Description
  ///   A class that implements the sequence::PhaseDescription interface
  ///
  /// \param[in] description
  ///   An immutable reference to the relevant Description instance
  ///
  /// \param[in] update
  ///   A callback that will be triggered when the phase has a significant
  ///   update in its status.
  ///
  /// \param[in] finished
  ///   A callback that will be triggered when the phase has finished.
  ///
  /// \return an active, running instance of the described phase.
  template<typename Description>
  using Activate =
    std::function<
    execute::ConstActivePhasePtr(
      const Description& description,
      std::function<void(execute::ConstConditionPtr)> update,
      std::function<void(execute::ConstConditionPtr)> finished)
    >;

  /// Add a callback to convert from a PhaseDescription into an active phase.
  ///
  /// \tparam Description
  ///   A class that implements the sequence::PhaseDescription interface
  template<typename Description>
  void add_activator(Activate<Description> activator);

  /// Build a Task object based on a phase sequence description.
  ///
  /// \param[in] description
  ///   The description of the phase sequence
  ///
  /// \param[in] update
  ///   A callback that will be triggered when the task has a significant update
  ///
  /// \param[in] finished
  ///   A callback that will be triggered when the task has finished
  ///
  /// \return an active, running instance of the described task.
  std::shared_ptr<Task> build(
    ConstTaskDescriptionPtr description,
    std::function<void(execute::ConstConditionPtr)> update,
    std::function<void(execute::ConstConditionPtr)> finished) const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};


} // namespace sequence
} // namespace rmf_task

#endif // RMF_TASK__SEQUENCE__PHASEFACTORY_HPP
