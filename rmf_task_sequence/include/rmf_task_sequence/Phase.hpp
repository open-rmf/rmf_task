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

#ifndef RMF_TASK_SEQUENCE__PHASE_HPP
#define RMF_TASK_SEQUENCE__PHASE_HPP

#include <rmf_task/Phase.hpp>

#include <rmf_task_sequence/Event.hpp>
#include <rmf_task_sequence/typedefs.hpp>

#include <rmf_task_sequence/Activity.hpp>

namespace rmf_task_sequence {

//==============================================================================
/// A factory for generating execute::ActivePhase instances from descriptions.
class Phase : public rmf_task::Phase
{
public:

  // Declarations
  class Active;
  using ActivePtr = std::shared_ptr<Active>;

  class Activator;
  using ActivatorPtr = std::shared_ptr<Activator>;
  using ConstActivatorPtr = std::shared_ptr<const Activator>;

  class Description;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;
};

//==============================================================================
/// The interface for an Active phase within a phase sequence task.
class Phase::Active :
  public rmf_task::Phase::Active,
  public Activity::Active {};

//==============================================================================
class Phase::Activator
{
public:
  /// Construct an empty Activator
  Activator();

  /// Signature for activating a phase
  ///
  /// \tparam Description
  ///   A class that implements the sequence::PhaseDescription interface
  ///
  /// \param[in] get_state
  ///   A callback for getting the current state of the robot
  ///
  /// \param[in] parameters
  ///   A reference to the parameters for the robot
  ///
  /// \param[in] tag
  ///   The tag of this phase
  ///
  /// \param[in] description
  ///   An immutable reference to the relevant Description instance
  ///
  /// \param[in] backup_state
  ///   The serialized backup state of the Phase, if the Phase is being restored
  ///   from a crash or disconnection. If the Phase is not being restored, a
  ///   std::nullopt will be passed in here.
  ///
  /// \param[in] update
  ///   A callback that will be triggered when the phase has a significant
  ///   update in its status. The callback will be given a snapshot of the
  ///   active phase. This snapshot can be safely read in parallel to the phase
  ///   execution.
  ///
  /// \param[in] checkpoint
  ///   A callback that will be triggered when the phase has reached a task
  ///   checkpoint whose state is worth backing up.
  ///
  /// \param[in] finished
  ///   A callback that will be triggered when the phase has finished.
  ///
  /// \return an active, running instance of the described phase.
  template<typename Description>
  using Activate =
    std::function<
    ActivePtr(
      const std::function<State()>& get_state,
      const ConstParametersPtr& parameters,
      ConstTagPtr tag,
      const Description& description,
      std::optional<nlohmann::json> backup_state,
      std::function<void(rmf_task::Phase::ConstSnapshotPtr)> update,
      std::function<void(Active::Backup)> checkpoint,
      std::function<void()> finished)
    >;

  /// Add a callback to convert from a PhaseDescription into an active phase.
  ///
  /// \tparam Description
  ///   A class that implements the sequence::PhaseDescription interface
  template<typename Description>
  void add_activator(Activate<Description> activator);

  /// Activate a phase based on a description of the phase.
  ///
  /// \param[in] get_state
  ///   A callback for getting the current state of the robot
  ///
  /// \param[in] parameters
  ///   A reference to the parameters for the robot
  ///
  /// \param[in] tag
  ///   The tag of this phase
  ///
  /// \param[in] description
  ///   The description of the phase
  ///
  /// \param[in] backup_state
  ///   If the phase is being restored, pass its backup state in here. Otherwise
  ///   if the phase is being freshly activated, pass a nullopt.
  ///
  /// \param[in] update
  ///   A callback that will be triggered when the phase has a notable update.
  ///   The callback will be given a snapshot of the active phase.
  ///
  /// \param[in] checkpoint
  ///   A callback that will be triggered when the phase has reached a task
  ///   checkpoint whose state is worth backing up.
  ///
  /// \param[in] finished
  ///   A callback that will be triggered when the phase has finished.
  ///
  /// \return an active, running instance of the described phase.
  ActivePtr activate(
    const std::function<State()>& get_state,
    const ConstParametersPtr& parameters,
    ConstTagPtr tag,
    const Description& description,
    std::optional<nlohmann::json> backup_state,
    std::function<void(rmf_task::Phase::ConstSnapshotPtr)> update,
    std::function<void(Active::Backup)> checkpoint,
    std::function<void()> finished) const;

  class Implementation;
private:

  /// \private
  void _add_activator(
    std::type_index type,
    Activate<Phase::Description> activator);

  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Phase::Description : public Activity::Description {};

} // namespace rmf_task_sequence

#include <rmf_task_sequence/detail/impl_Phase.hpp>

#endif // RMF_TASK_SEQUENCE__PHASE_HPP
