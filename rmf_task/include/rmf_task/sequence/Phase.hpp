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

#ifndef RMF_TASK__SEQUENCE__PHASE_HPP
#define RMF_TASK__SEQUENCE__PHASE_HPP

#include <rmf_task/State.hpp>
#include <rmf_task/Constraints.hpp>
#include <rmf_task/Parameters.hpp>
#include <rmf_task/execute/Phase.hpp>
#include <rmf_task/detail/Resume.hpp>

namespace rmf_task {
namespace sequence {

//==============================================================================
/// A factory for generating execute::ActivePhase instances from descriptions.
class Phase
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
};

//==============================================================================
/// The interface for an Active phase within a phase sequence task.
class Phase::Active : public execute::Phase::Active
{
public:

  /// The Resume class keeps track of when the phase is allowed to Resume.
  /// You can either call the Resume object's operator() or let the object
  /// expire to tell the phase that it may resume.
  class Resume : public detail::Resume {};

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
class Phase::Activator
{
  /// Construct an empty Activator
  Activator();

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
      const Description& description,
      std::function<void(execute::Phase::ConstSnapshotPtr)> update,
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
  /// \param[in] description
  ///   The description of the phase
  ///
  /// \param[in] update
  ///   A callback that will be triggered when the phase has a notable update.
  ///   The callback will be given a snapshot of the active phase.
  ///
  /// \param[in] finished
  ///   A callback that will be triggered when the task has finished. A
  ///   completed
  ///
  /// \return an active, running instance of the described task.
  ActivePtr activate(
    const Description& description,
    std::function<void(execute::Phase::ConstSnapshotPtr)> update,
    std::function<void(execute::Phase::Completed)> finished) const;

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

  /// Get the human-friendly description of this phase as pending
  virtual execute::Phase::ConstTagPtr make_tag(
    const State& initial_state,
    const Parameters& parameters) const = 0;

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
  /// \return an estimated state for the robot when the phase is finished.
  virtual std::optional<State> estimate_finish(
    State initial_state,
    const Constraints& constraints) const = 0;

  /// Estimate the invariant component of the request's duration.
  virtual rmf_traffic::Duration invariant_duration() const = 0;

  /// Get the components of the finish state that this phase is guaranteed to
  /// result in once the phase is finished.
  virtual State invariant_finish_state() const = 0;

  // Virtual destructor
  virtual ~Model() = default;
};

} // namespace sequence
} // namespace rmf_task

#endif // RMF_TASK__SEQUENCE__PHASE_HPP
