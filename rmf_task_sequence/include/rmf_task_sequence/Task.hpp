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

#ifndef RMF_TASK__SEQUENCE__TASK_HPP
#define RMF_TASK__SEQUENCE__TASK_HPP

#include <rmf_task/Request.hpp>
#include <rmf_task/Task.hpp>
#include <rmf_task/Activator.hpp>
#include <rmf_task_sequence/Phase.hpp>

namespace rmf_task_sequence {

//==============================================================================
class Task : public rmf_task::Task
{
public:

  // Declaration
  class Builder;

  // Declaration
  class Active;

  // Declaration
  class Description;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;

  class Model;

  using Update = std::function<void(Phase::ConstSnapshotPtr snapshot)>;
  using PhaseFinished = std::function<void(Phase::ConstCompletedPtr)>;
  using TaskFinished = std::function<void()>;

  /// Make an activator for a phase sequence task. This activator can be given
  /// to
  ///
  /// \param[in] phase_activator
  ///   A phase activator
  static rmf_task::Activator::Activate<Description> make_activator(
    Phase::ConstActivatorPtr phase_activator);

};

//==============================================================================
class Task::Builder
{
public:

  /// Get the builder ready.
  Builder();

  /// Add a phase to the sequence.
  ///
  /// \param[in] description
  ///   A description of the phase
  ///
  /// \param[in] cancellation_sequence
  ///   This phase sequence will be run if the task is cancelled during this
  ///   phase.
  Builder& add_phase(
    Phase::ConstDescriptionPtr description,
    std::vector<Phase::ConstDescriptionPtr> cancellation_sequence);

  /// Generate a TaskDescription instance from the phases that have been given
  /// to the builder.
  ///
  /// \param[in] category
  ///   Task category information that will go into the Task::Tag
  ///
  /// \param[in] detail
  ///   Any detailed information that will go into the Task::Tag
  ConstDescriptionPtr build(
    std::string category,
    std::string detail);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Task::Description : public rmf_task::Task::Description
{
public:

  // Documentation inherited
  Task::ConstModelPtr make_model(
    rmf_traffic::Time earliest_start_time,
    const rmf_task::Parameters& parameters) const final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Task::Model : public rmf_task::Task::Model
{
public:

  // Documentation inherited
  std::optional<rmf_task::Estimate> estimate_finish(
    const rmf_task::State& initial_state,
    const rmf_task::Constraints& task_planning_constraints,
    const rmf_task::TravelEstimator& travel_estimator) const final;

  // Documentation inherited
  rmf_traffic::Duration invariant_duration() const final;

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_task_sequence

#endif // RMF_TASK__SEQUENCE__TASK_HPP
