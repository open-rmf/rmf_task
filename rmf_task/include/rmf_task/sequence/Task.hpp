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
#include <rmf_task/execute/Task.hpp>
#include <rmf_task/sequence/Phase.hpp>

namespace rmf_task {
namespace sequence {

//==============================================================================
class Task
{
public:

  // Declaration
  class Builder;

  // Declaration
  class Active;
  using ConstActivePtr = std::shared_ptr<const Active>;

  // Declaration
  class Description;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;

  using Update = std::function<void(execute::Phase::ConstSnapshotPtr snapshot)>;
  using PhaseFinished = std::function<void(execute::Phase::ConstCompletedPtr)>;
  using TaskFinished = std::function<void()>;

  /// Activate a phase sequence task
  ///
  /// \param[in] activator
  ///   A phase activator
  static ConstActivePtr activate(
    std::shared_ptr<const Phase::Activator> activator,
    const Description& description,
    Update update,
    PhaseFinished phase_finished,
    TaskFinished task_finished);

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
  void add_phase(
    Phase::ConstDescriptionPtr description,
    std::vector<Phase::ConstDescriptionPtr> cancellation_sequence);

  /// Generate a TaskDescription instance from the phases that have been given
  /// to the builder.
  ConstDescriptionPtr build();

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Task::Active
  : public execute::Task,
    public std::enable_shared_from_this<Active>
{
public:

  // Documentation inherited
  const std::vector<execute::Phase::ConstCompletedPtr>&
  completed_phases() const final;

  // Documentation inherited
  execute::Phase::ConstActivePtr active_phase() const final;

  // Documentation inherited
  std::vector<execute::Phase::Pending> pending_phases() const final;

  // Documentation inherited
  const Request::ConstTagPtr& tag() const final;

  // Documentation inherited
  const std::string& category() const final;

  // Documentation inherited
  const std::string& detail() const final;

  // Documentation inherited
  rmf_traffic::Time estimate_finish_time() const final;

  // Documentation inherited
  Resume interrupt(std::function<void()> task_is_interrupted) final;

  // Documentation inherited
  void cancel() final;

  // Documentation inherited
  void kill() final;

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Task::Description : public Request::Description
{
public:
  class Model;

  // Documentation inherited
  std::shared_ptr<Request::Model> make_model(
    rmf_traffic::Time earliest_start_time,
    const Parameters& parameters) const final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Task::Description::Model : public Request::Model
{
public:

  // Documentation inherited
  std::optional<Estimate> estimate_finish(
    const State& initial_state,
    const Constraints& task_planning_constraints,
    EstimateCache& estimate_cache) const final;

  // Documentation inherited
  rmf_traffic::Duration invariant_duration() const final;

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace sequence
} // namespace rmf_task

#endif // RMF_TASK__SEQUENCE__TASK_HPP
