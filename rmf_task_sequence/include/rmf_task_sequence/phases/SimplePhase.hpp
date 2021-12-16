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

#ifndef RMF_TASK_SEQUENCE__PHASES__SIMPLEPHASE_HPP
#define RMF_TASK_SEQUENCE__PHASES__SIMPLEPHASE_HPP

#include <rmf_task_sequence/Phase.hpp>
#include <rmf_task_sequence/Event.hpp>

namespace rmf_task_sequence {
namespace phases {

//==============================================================================
class SimplePhase : public Phase
{
public:

  class Active;

  class Description;
  using DescriptionPtr = std::shared_ptr<Description>;

  static void add(
    Phase::Activator& phase_activator,
    const Event::ConstInitializerPtr& event_initializer);

};

//==============================================================================
class SimplePhase::Description : public Phase::Description
{
public:

  /// Make a SimplePhase description.
  ///
  /// \param[in] final_event
  ///   This is the final event which determines when the phase is finished
  ///
  /// \param[in] category
  ///   Specify a custom category string. If this is null, then the phase will
  ///   borrow the category from the final_event.
  ///
  /// \param[in] detail
  ///   Specify a custom detail string for the phase. If this is null, then the
  ///   phase will borrow the detail from the final_event.
  static DescriptionPtr make(
    Event::ConstDescriptionPtr final_event,
    std::optional<std::string> category = std::nullopt,
    std::optional<std::string> detail = std::nullopt);

  /// Get the final event
  const Event::ConstDescriptionPtr& final_event() const;

  /// Set the final event
  Description& final_event(Event::ConstDescriptionPtr new_final_event);

  /// Get the category
  const std::optional<std::string>& category() const;

  /// Set the category
  Description& category(std::optional<std::string> new_category);

  /// Get the detail
  const std::optional<std::string>& detail() const;

  /// Set the detail
  Description& detail(std::optional<std::string> new_detail);

  // Documentation inherited
  Activity::ConstModelPtr make_model(
    State invariant_initial_state,
    const Parameters& parameters) const final;

  // Documentation inherited
  Header generate_header(
    const State& initial_state,
    const Parameters& parameters) const final;

  class Implementation;
private:
  Description();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace phases
} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__PHASES__SIMPLEPHASE_HPP
