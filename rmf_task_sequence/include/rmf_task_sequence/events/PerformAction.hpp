/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef RMF_TASK_SEQUENCE__EVENTS__PERFORMACTION_HPP
#define RMF_TASK_SEQUENCE__EVENTS__PERFORMACTION_HPP

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_task_sequence/Event.hpp>

#include <nlohmann/json.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
class PerformAction
{
public:
  using Location = rmf_traffic::agv::Plan::Goal;

  class Description;
  using DescriptionPtr = std::shared_ptr<Description>;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;

  class Model;
};

//==============================================================================
class PerformAction::Description : public Event::Description
{
public:

  /// Make a PerformAction description.
  ///
  /// \param[in] category
  ///   A category for this action
  ///
  /// \param[in] description
  ///   A json description of the action to perform
  ///
  /// \param[in] action_duration_estimate
  ///   An estimate for how long it will take for the action to complete
  ///
  /// \param[in] use_tool_sink
  ///   If true, battery drain from peripheral tools will be accounted for
  ///   while performing the action
  ///
  /// \param[in] expected_finish_location
  ///   An optional location to indicate where the robot will end up after
  ///   performing the action. Use nullopt to indicate that after performing
  ///   the action, the robot will be at its initial location.
  static DescriptionPtr make(
    const std::string& category,
    nlohmann::json description,
    rmf_traffic::Duration action_duration_estimate,
    bool use_tool_sink,
    std::optional<Location> expected_finish_location = std::nullopt);

  /// Get the category
  const std::string& category() const;

  /// Set the category
  Description& category(const std::string& new_category);

  /// Get the description
  const nlohmann::json& description() const;

  /// Set the description
  Description& description(const nlohmann::json& new_description);

  /// Get the action duration estimate
  const rmf_traffic::Duration& action_duration_estimate() const;

  /// Set the action duration estimate
  Description& action_duration_estimate(rmf_traffic::Duration new_duration);

  /// Check whether to account for battery drain from tools
  bool use_tool_sink() const;

  /// Set whether to account for battery drain from tools
  Description& use_tool_sink(bool use_tool);

  /// Get the expected finish location
  std::optional<Location> expected_finish_location() const;

  /// Set the expected finish location
  Description& expected_finish_location(std::optional<Location> new_location);

  // Documentation inherited
  Activity::ConstModelPtr make_model(
    State invariant_initial_state,
    const Parameters& parameters) const final;

  // Documentation inherited
  Header generate_header(
    const rmf_task::State& initial_state,
    const Parameters& parameters) const final;

  class Implementation;
private:
  Description();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace events
} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__EVENTS__PERFORMACTION_HPP
