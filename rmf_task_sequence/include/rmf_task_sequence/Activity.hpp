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

#ifndef RMF_TASK_SEQUENCE__ACTIVITY_HPP
#define RMF_TASK_SEQUENCE__ACTIVITY_HPP

#include <rmf_task/detail/Resume.hpp>
#include <rmf_task/Header.hpp>

#include <rmf_task_sequence/detail/Backup.hpp>
#include <rmf_task_sequence/typedefs.hpp>

namespace rmf_task_sequence {

//==============================================================================
/// The Activity namespace class provides abstract interfaces that are shared
/// between the Event and Phase namespace classes.
class Activity
{
public:

  class Active;

  class Description;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;

  class Model;
  using ConstModelPtr = std::shared_ptr<const Model>;

  class SequenceModel;
};

//==============================================================================
/// The interface for an active activity. This interface deals with backing up
/// the current state, interrupting the activity, and cancelling or killing it.
class Activity::Active
{
public:

  using Backup = detail::Backup;

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
class Activity::Description
{
public:

  /// Generate a Model for this Activity based on its description, parameters,
  /// and the invariants of its initial state.
  ///
  /// \param[in] invariant_initial_state
  ///   A partial state that represents the state components which will
  ///   definitely be true when this Activity begins.
  ///
  /// \param[in] parameters
  ///   The parameters for the robot.
  ///
  /// \return a model based on the given start state and parameters.
  virtual ConstModelPtr make_model(
    State invariant_initial_state,
    const Parameters& parameters) const = 0;

  /// Generate human-friendly header information for this Activity.
  ///
  /// \param[in] initial_state
  ///   The expected initial state when the activity begins
  ///
  /// \param[in] parameters
  ///   Parameters of the robot during the Activity
  virtual Header generate_header(
    const State& initial_state,
    const Parameters& parameters) const = 0;

  // Virtual destructor
  virtual ~Description() = default;
};

//==============================================================================
class Activity::Model
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
  virtual std::optional<Estimate> estimate_finish(
    State initial_state,
    rmf_traffic::Time earliest_arrival_time,
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
class Activity::SequenceModel : public Activity::Model
{
public:

  /// Make a SequenceModel by providing a vector of descriptions and the
  /// arguments that are given to Phase::Description::make_model(~).
  ///
  /// \param[in] descriptions
  ///   The Phase descriptions that are being modeled. The ordering of the
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
  static ConstModelPtr make(
    const std::vector<ConstDescriptionPtr>& descriptions,
    State invariant_initial_state,
    const Parameters& parameters);

  // Documentation inherited
  std::optional<rmf_task::Estimate> estimate_finish(
    State initial_state,
    rmf_traffic::Time earliest_arrival_time,
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

#endif // RMF_TASK_SEQUENCE__ACTIVITY_HPP
