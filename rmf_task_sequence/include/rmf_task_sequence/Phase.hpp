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

#include <rmf_task/State.hpp>
#include <rmf_task/Constraints.hpp>
#include <rmf_task/Parameters.hpp>
#include <rmf_task/Estimate.hpp>
#include <rmf_task/Phase.hpp>
#include <rmf_task/detail/Resume.hpp>
#include <rmf_task/detail/Backup.hpp>

#include <rmf_task_sequence/typedefs.hpp>

#include <yaml-cpp/yaml.h>

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

  class Model;
  using ConstModelPtr = std::shared_ptr<const Model>;

  class SequenceModel;
};

//==============================================================================
/// The interface for an Active phase within a phase sequence task.
class Phase::Active : public rmf_task::Phase::Active
{
public:

  /// Backup data for the Phase. The state of the phase is represented by a
  /// YAML::Node. The meaning and format of the Node is up to the phase
  /// implementation to decide.
  ///
  /// Each Backup is tagged with a sequence number. As the Phase makes progress,
  /// it can issue new Backups with higher sequence numbers. Only the Backup
  /// with the highest sequence number will be kept.
  class Backup;

  /// Get a backup for this Phase
  virtual Backup backup() const = 0;

  /// The Resume class keeps track of when the phase is allowed to Resume.
  /// You can either call the Resume object's operator() or let the object
  /// expire to tell the phase that it may resume.
  using Resume = rmf_task::detail::Resume;

  /// Tell this phase that it needs to be interrupted. An interruption means
  /// the robot may be commanded to do other tasks before this phase resumes.
  ///
  /// Interruptions may occur to allow operators to take manual control of the
  /// robot, or to engage automatic behaviors in response to emergencies, e.g.
  /// fire alarms or code blues.
  virtual Resume interrupt(std::function<void()> task_is_interrupted) = 0;

  /// Tell the phase that it has been canceled. The behavior that follows a
  /// cancellation will vary between different phases, but generally it means
  /// that the robot should no longer try to complete its Task and should
  /// instead try to return itself to an unencumbered state as quickly as
  /// possible.
  ///
  /// The phase may continue to perform some actions after being canceled.
  ///
  /// The phase should continue to be tracked as normal. When its finished
  /// callback is triggered, the cancellation is complete.
  virtual void cancel() = 0;

  /// Kill this phase. The behavior that follows a kill will vary between
  /// different phases, but generally it means that the robot should be returned
  /// to a safe idle state as soon as possible, even if it remains encumbered by
  /// something related to this Task.
  ///
  /// The phase should continue to be tracked as normal. When its finished
  /// callback is triggered, the killing is complete.
  ///
  /// The kill() command supersedes the cancel() command. Calling cancel() after
  /// calling kill() will have no effect.
  virtual void kill() = 0;

  // Virtual destructor
  virtual ~Active() = default;
};

//==============================================================================
class Phase::Active::Backup
{
public:

  /// Make a backup of the phase
  ///
  /// \param[in] seq
  ///   Sequence number. The Backup from this phase with the highest sequence
  ///   number will be held onto until a Backup with a higher sequence number is
  ///   issued.
  ///
  /// \param[in] state
  ///   A serialization of the phase's state. This will be used by
  ///   Phase::Activator when restoring a Task.
  static Backup make(uint64_t seq, YAML::Node state);

  /// Get the sequence number
  uint64_t sequence() const;

  /// Get the YAML representation of the backed up state
  const YAML::Node& state() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

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
  /// \param[in] finished
  ///   A callback that will be triggered when the phase has finished.
  ///
  /// \return an active, running instance of the described phase.
  template<typename Description>
  using Activate =
    std::function<
    ActivePtr(
      std::function<State()> get_state,
      ConstTagPtr tag,
      const Description& description,
      std::optional<std::string> backup_state,
      std::function<void(rmf_task::Phase::ConstSnapshotPtr)> update,
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
  /// \param[in] tag
  ///   The tag of this phase
  ///
  /// \param[in] description
  ///   The description of the phase
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
    std::function<State()> get_state,
    ConstTagPtr tag,
    const Description& description,
    std::function<void(rmf_task::Phase::ConstSnapshotPtr)> update,
    std::function<void(Active::Backup)> checkpoint,
    std::function<void()> finished) const;

  /// Restore a phase based on a description of the phase and its backup state.
  ///
  /// \param[in] get_state
  ///   A callback for getting the current state of the robot
  ///
  /// \param[in] tag
  ///   The tag of this phase
  ///
  /// \param[in] description
  ///   The description of the phase
  ///
  /// \param[in] backup_state
  ///   The serialized backup state of the phase
  ///
  /// \param[in] update
  ///   A callback that will triggered when the phase has a notable update.
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
  ActivePtr restore(
    std::function<State()> get_state,
    ConstTagPtr tag,
    const Description& description,
    const std::string& backup_state,
    std::function<void(rmf_task::Phase::ConstSnapshotPtr)> update,
    std::function<void(Active::Backup)> checkpoint,
    std::function<void()> finished) const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Phase::Description
{
public:

  /// Generate a Model for this phase based on its description, parameters, and
  /// the invariants of its initial state.
  ///
  /// \param[in] invariant_initial_state
  ///   A partial state that represents the state components which will
  ///   definitely be true when this phase begins.
  ///
  /// \param[in] parameters
  ///   The parameters for the robot.
  ///
  /// \return a model based on the given start state and parameters.
  virtual ConstModelPtr make_model(
    State invariant_initial_state,
    const Parameters& parameters) const = 0;

  /// Get the human-friendly information about this phase
  ///
  /// \param[in] initial_state
  ///   The expected initial state when the phase begins
  ///
  /// \param[in] constraints
  ///   Constraints on the robot during the phase
  virtual rmf_task::Phase::ConstTagPtr make_tag(
    rmf_task::Phase::Tag::Id id,
    const State& initial_state,
    const Parameters& parameters) const = 0;

  /// Serialize this phase description into a string.
  virtual YAML::Node serialize() const = 0;

  // Virtual destructor
  virtual ~Description() = default;
};

//==============================================================================
class Phase::Model
{
public:

  /// Estimate the state that the robot will have when the phase is finished.
  ///
  /// \param[in] initial_state
  ///   The expected initial state when the phase begins
  ///
  /// \param[in] constraints
  ///   Constraints on the robot during the phase
  ///
  /// \param[in]
  ///
  /// \return an estimated state for the robot when the phase is finished.
  virtual std::optional<State> estimate_finish(
    State initial_state,
    const Constraints& constraints,
    const TravelEstimator& travel_estimator) const = 0;

  /// Estimate the invariant component of the request's duration.
  virtual rmf_traffic::Duration invariant_duration() const = 0;

  /// Get the components of the finish state that this phase is guaranteed to
  /// result in once the phase is finished.
  virtual State invariant_finish_state() const = 0;

  // Virtual destructor
  virtual ~Model() = default;
};

//==============================================================================
/// An implementation of a Phase::Model that models a sequence of
/// Phase::Descriptions.
class Phase::SequenceModel : public Phase::Model
{
public:

  /// Make a SequenceModel by providing a vector of descriptions and the
  /// arguments that are given to Phase::Description::make_model(~).
  ///
  /// \param[in] descriptions
  ///   The Phase descriptions that are being modelled. The ordering of the
  ///   descriptions may impact model outcomes. The order of the descriptions
  ///   in the vector should reflect the actual order that the phases would
  ///   occur in.
  ///
  /// \param[in] invariant_initial_state
  ///   A partial state that represents the state components which will
  ///   definitely be true when this phase begins.
  ///
  /// \param[in] parameters
  ///   The parameters for the robot
  ///
  /// \return A Phase::Model implemented as a SequenceModel.
  static Phase::ConstModelPtr make(
    const std::vector<Phase::ConstDescriptionPtr>& descriptions,
    State invariant_initial_state,
    const Parameters& parameters);

  // Documentation inherited
  std::optional<rmf_task::State> estimate_finish(
    State initial_state,
    const Constraints& constraints,
    const TravelEstimator& travel_estimator) const final;

  // Documentation inherited
  rmf_traffic::Duration invariant_duration() const final;

  // Documentation inherited
  State invariant_finish_state() const final;

  class Implementation;
private:
  SequenceModel();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__PHASE_HPP
