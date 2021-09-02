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

#include <rmf_task/execute/Condition.hpp>

#include <rmf_traffic/Time.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <memory>

namespace rmf_task {
namespace execute {

//==============================================================================
class CompletedPhase
{
public:

  const std::string& name() const;
  const std::string& detail() const;
  const std::vector<std::string>& issues() const;

  rmf_traffic::Time start_time() const;
  rmf_traffic::Time original_finish_estimate() const;
  rmf_traffic::Time finish_time() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class ActivePhase
{
public:

  /// The name of this phase.
  virtual std::string name() const = 0;

  /// Details about this phase
  virtual std::string detail() const = 0;

  /// The condition that needs to be satisfied for this phase to be complete
  virtual ConstConditionPtr finish_condition() const = 0;

  /// The estimated finish time for this phase
  virtual rmf_traffic::Time estimate_finish_time() const = 0;

  // Virtual destructor
  virtual ~ActivePhase() = default;
};

using ConstActivePhasePtr = std::shared_ptr<const ActivePhase>;

//==============================================================================
class PendingPhase
{
public:

  const std::string& name() const;
  const std::string& detail() const;
  rmf_traffic::Duration duration_estimate() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace execute
} // namespace rmf_task

#endif // RMF_TASK__EXECUTE__PHASE_HPP
