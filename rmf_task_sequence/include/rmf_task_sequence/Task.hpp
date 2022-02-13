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

#ifndef RMF_TASK_SEQUENCE__TASK_HPP
#define RMF_TASK_SEQUENCE__TASK_HPP

#include <rmf_task/Request.hpp>
#include <rmf_task/Task.hpp>
#include <rmf_task/Activator.hpp>
#include <rmf_task_sequence/Phase.hpp>

#include <rmf_task_sequence/typedefs.hpp>

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
  using DescriptionPtr = std::shared_ptr<Description>;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;

  class Model;

  using Update = std::function<void(Phase::ConstSnapshotPtr snapshot)>;
  using PhaseFinished = std::function<void(Phase::ConstCompletedPtr)>;
  using TaskFinished = std::function<void()>;

  /// Make an activator for a phase sequence task. This activator can be given
  /// to the rmf_task::Activator class to activate phase sequence tasks from
  /// phase sequence descriptions.
  ///
  /// \param[in] phase_activator
  ///   A phase activator. It is recommended to fully initialize this phase
  ///   activator (add all supported phases) before passing it to this function.
  ///   The task activator will keep a reference to this phase activator, so
  ///   modifying it while a task is activating a phase could cause data races
  ///   and therefore undefined behavior.
  ///
  /// \param[in] clock
  ///   A callback that gives the current time when called.
  static rmf_task::Activator::Activate<Description> make_activator(
    Phase::ConstActivatorPtr phase_activator,
    std::function<rmf_traffic::Time()> clock);

  /// Add this task type to an Activator. This is an alternative to using
  /// make_activator(~).
  ///
  /// \param[in] activator
  ///   The task activator to add this task type to.
  ///
  /// \param[in] phase_activator
  ///   A phase activator. It is recommended to fully initialize this phase
  ///   activator (add all supported phases) before passing it to this function.
  ///   The task activator will keep a reference to this phase activator, so
  ///   modifying it while a task is activating a phase could cause data races
  ///   and therefore undefined behavior.
  ///
  /// \param[in] clock
  ///   A callback that gives the current time when called.
  static void add(
    rmf_task::Activator& activator,
    Phase::ConstActivatorPtr phase_activator,
    std::function<rmf_traffic::Time()> clock);

  /// Give an initializer the ability to build a sequence task for some other
  /// task description.
  ///
  /// This is useful when the described task is best implemented as a sequence
  /// of phases.
  ///
  /// \param[in] unfold_description
  ///   This will be used to unfold the other description into a task sequence
  ///   Description.
  ///
  /// \param[in] activator
  ///   This activator will be given the ability to unfold and activate the
  ///   OtherDesc type.
  ///
  /// \param[in] phase_activator
  ///   This phase activator will be used to activate the task
  ///
  /// \param[in] clock
  ///   A callback that gives the current time when called
  template<typename OtherDesc>
  static void unfold(
    std::function<Description(const OtherDesc&)> unfold_description,
    rmf_task::Activator& activator,
    Phase::ConstActivatorPtr phase_activator,
    std::function<rmf_traffic::Time()> clock);

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
  std::shared_ptr<Description> build(
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
    const Parameters& parameters) const final;

  // Documentation inherited
  Info generate_info(
    const State& initial_state,
    const Parameters& parameters) const final;

  /// Get the category for this task
  const std::string& category() const;

  /// Change the category for this task
  Description& category(std::string new_category);

  /// Get the details for this task
  const std::string& detail() const;

  /// Change the details for this task
  Description& detail(std::string new_detail);

  Header generate_header(
    const State& initial_state,
    const Parameters& parameters) const;

  class Implementation;
private:
  Description();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_task_sequence

#include <rmf_task_sequence/detail/impl_Task.hpp>

#endif // RMF_TASK_SEQUENCE__TASK_HPP
