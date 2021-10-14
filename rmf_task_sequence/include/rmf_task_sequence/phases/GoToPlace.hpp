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

#ifndef RMF_TASK_SEQUENCE__PHASES__GOTOPLACE_HPP
#define RMF_TASK_SEQUENCE__PHASES__GOTOPLACE_HPP

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_task_sequence/Phase.hpp>

namespace rmf_task_sequence {
namespace phases {

//==============================================================================
class GoToPlace
{
public:
  using Goal = rmf_traffic::agv::Plan::Goal;

  class Description;
  using DescriptionPtr = std::shared_ptr<Description>;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;

  class Model;
};

//==============================================================================
class GoToPlace::Description : public Phase::Description
{
public:

  /// Make a GoToPlace description using a goal.
  static DescriptionPtr make(Goal goal);

  /// Get the current goal for this description.
  const Goal& goal() const;

  /// Set the current goal for this description.
  Description& goal(Goal new_goal);

  /// Get the name of the goal. If the goal does not have an explicit name, it
  /// will be referred to as "#x" where "x" is the index number of the waypoint.
  std::string goal_name(const rmf_task::Parameters& parameters) const;

  // Documentation inherited
  Phase::ConstModelPtr make_model(
    rmf_task::State invariant_initial_state,
    const rmf_task::Parameters& parameters) const final;

  // Documentation inherited
  Phase::ConstTagPtr make_tag(
    Phase::Tag::Id id,
    const rmf_task::State& initial_state,
    const rmf_task::Parameters& parameters) const final;

  // Documentation inherited
  nlohmann::json serialize() const final;

  class Implementation;
private:
  Description();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace phases
} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__PHASES__GOTOPLACE_HPP
