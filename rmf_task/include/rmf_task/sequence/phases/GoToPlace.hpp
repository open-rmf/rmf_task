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

#ifndef RMF_TASK__SEQUENCE__PHASES__GOTOPLACE_HPP
#define RMF_TASK__SEQUENCE__PHASES__GOTOPLACE_HPP

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_task/sequence/Phase.hpp>

namespace rmf_task {
namespace sequence {
namespace phases {

//==============================================================================
class GoToPlace
{
public:
  using Goal = rmf_traffic::agv::Plan::Goal;

  class Description;
  class Model;
};

//==============================================================================
class GoToPlace::Description : public Phase::Description
{
public:

  /// Make a GoToPlace description using a goal.
  static std::shared_ptr<Description> make(Goal goal);

  // Documentation inherited
  Phase::ConstModelPtr make_model(
    State invariant_initial_state,
    const Parameters& parameters) const final;

  // Documentation inherited
  execute::Phase::ConstTagPtr make_tag(
    execute::Phase::Tag::Id id,
    const State& initial_state,
    const Parameters& parameters) const final;

  /// Get the current goal for this description.
  const Goal& goal() const;

  /// Set the current goal for this description.
  Description& goal(Goal new_goal);

  class Implementation;
private:
  Description();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace phases
} // namespace sequence
} // namespace rmf_task

#endif // RMF_TASK__SEQUENCE__PHASES__GOTOPLACE_HPP
