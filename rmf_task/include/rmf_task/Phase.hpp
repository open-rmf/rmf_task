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

#ifndef RMF_TASK__EXECUTE__PHASE_HPP
#define RMF_TASK__EXECUTE__PHASE_HPP

#include <rmf_task/Condition.hpp>

#include <rmf_traffic/Time.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <memory>

namespace rmf_task {

//==============================================================================
class Phase
{
public:

  class Tag;
  using ConstTagPtr = std::shared_ptr<const Tag>;

  class Completed;
  using ConstCompletedPtr = std::shared_ptr<const Completed>;

  class Active;
  using ConstActivePtr = std::shared_ptr<const Active>;

  class Snapshot;
  using ConstSnapshotPtr = std::shared_ptr<const Snapshot>;

  class Pending;
  using ConstPendingPtr = std::shared_ptr<const Pending>;
};

//==============================================================================
/// Basic static information about a phase. This information should go
/// unchanged from the Pending state, through the Active state, and into the
/// Completed state.
class Phase::Tag
{
public:

  using Id = uint64_t;

  /// Constructor
  ///
  /// \param[in] id_
  ///   ID of the phase. This phase ID must be unique within its Task instance.
  ///
  /// \param[in] name_
  ///   Name of the phase
  ///
  /// \param[in] detail_
  ///   Details about the phase
  ///
  /// \param[in] estimate_
  ///   The original (ideal) estimate of how long the phase will last
  Tag(
    Id id_,
    std::string name_,
    std::string detail_,
    rmf_traffic::Duration estimate_);

  /// Unique ID of the phase
  Id id() const;

  /// Name of the phase
  const std::string& name() const;

  /// Details about the phase
  const std::string& detail() const;

  /// The original (ideal) estimate of how long the phase will last
  rmf_traffic::Duration original_duration_estimate() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// Information about a phase that has been completed.
class Phase::Completed
{
public:

  /// Tag of the phase
  const ConstTagPtr& tag() const;

  /// The final log of the phase
  const Log::View& log() const;

  /// The actual time that the phase started
  rmf_traffic::Time start_time() const;

  /// The actual time that the phase finished.
  rmf_traffic::Time finish_time() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Phase::Active
{
public:

  /// Tag of the phase
  virtual ConstTagPtr tag() const = 0;

  /// The condition that needs to be satisfied for this phase to be complete
  virtual ConstConditionPtr finish_condition() const = 0;

  /// The estimated finish time for this phase
  virtual rmf_traffic::Time estimate_finish_time() const = 0;

  // Virtual destructor
  virtual ~Active() = default;
};

//==============================================================================
class Phase::Snapshot : public Phase::Active
{
public:

  /// Make a snapshot of an Active phase
  static ConstSnapshotPtr make(const Active& active);

  // Documentation inherited
  ConstTagPtr tag() const final;

  // Documentation inherited
  ConstConditionPtr finish_condition() const final;

  // Documentation inherited
  rmf_traffic::Time estimate_finish_time() const final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Phase::Pending
{
public:

  /// Tag of the phase
  const ConstTagPtr& tag() const;

  /// Check if this phase is set to be skipped
  bool will_be_skipped() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_task

#endif // RMF_TASK__EXECUTE__PHASE_HPP
