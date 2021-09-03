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

#ifndef RMF_TASK__SEQUENCE__PHASEDESCRIPTION_HPP
#define RMF_TASK__SEQUENCE__PHASEDESCRIPTION_HPP

#include <memory>
#include <optional>

#include <rmf_task/State.hpp>
#include <rmf_task/Constraints.hpp>
#include <rmf_task/Parameters.hpp>
#include <rmf_task/execute/Phase.hpp>

namespace rmf_task {
namespace sequence {

//==============================================================================
class PhaseDescription
{
public:

  /// An abstract interface for computing estimates and invariants related to
  /// this phase.
  class Model
  {
  public:

    /// Estimate the state that the robot will have when the phase is finished.
    ///
    /// \param[in] initial_state
    ///
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

  /// Generate a Model for this phase based on its description, parameters, and
  /// the invariants of its initial state.
  virtual std::shared_ptr<Model> make_model(
    State invariant_initial_state,
    const Parameters& parameters) const = 0;

  /// Get the human-friendly description of this phase as pending
  virtual const execute::PendingPhase& pending() const = 0;

  // Virtual destructor
  virtual ~PhaseDescription() = default;
};

using ConstPhaseDescriptionPtr = std::shared_ptr<const PhaseDescription>;

} // namespace sequence
} // namespace rmf_task

#endif // RMF_TASK__SEQUENCE__PHASEDESCRIPTION_HPP
