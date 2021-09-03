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

#ifndef RMF_TASK__SEQUENCE__TASKDESCRIPTION_HPP
#define RMF_TASK__SEQUENCE__TASKDESCRIPTION_HPP

#include <rmf_task/Request.hpp>
#include <rmf_task/sequence/PhaseDescription.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_task {
namespace sequence {

//==============================================================================
/// The Request::Description implementation for a phase sequence task.
class TaskDescription : public Request::Description
{
public:

  class Builder;
  class Model;

  // Documentation inherited
  std::shared_ptr<Request::Model> make_model(
    rmf_traffic::Time earliest_start_time,
    const Parameters& parameters) const final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

using ConstTaskDescriptionPtr = std::shared_ptr<const TaskDescription>;

//==============================================================================
class TaskDescription::Builder
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
    ConstPhaseDescriptionPtr description,
    std::vector<ConstPhaseDescriptionPtr> cancellation_sequence);

  /// Generate a TaskDescription instance from the phases that have been given
  /// to the builder.
  ConstTaskDescriptionPtr build();

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// The implementation of a task description model for a phase sequence task.
class TaskDescription::Model : public Request::Model
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


#endif // RMF_TASK__SEQUENCE__TASKDESCRIPTION_HPP
