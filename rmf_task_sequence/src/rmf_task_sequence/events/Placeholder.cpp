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

#include <rmf_task_sequence/events/Placeholder.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
class Placeholder::Description::Implementation
{
public:

  std::string category;
  std::string detail;

};

//==============================================================================
class Placeholder::Model : public Activity::Model
{
public:

  Model(State invariant_initial_state)
  : _invariant_finish_state(std::move(invariant_initial_state))
  {
    // Do nothing
  }

  std::optional<Estimate> estimate_finish(
    State initial_state,
    rmf_traffic::Time earliest_arrival_time,
    const Constraints&,
    const TravelEstimator&) const final
  {
    return Estimate(std::move(initial_state), earliest_arrival_time);
  }

  rmf_traffic::Duration invariant_duration() const final
  {
    return rmf_traffic::Duration(0);
  }

  State invariant_finish_state() const final
  {
    return _invariant_finish_state;
  }

  State _invariant_finish_state;
};

//==============================================================================
Placeholder::Description::Description(
  std::string category,
  std::string detail)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(category),
        std::move(detail)
      }))
{
  // Do nothing
}

//==============================================================================
Activity::ConstModelPtr Placeholder::Description::make_model(
  rmf_task::State invariant_initial_state,
  const rmf_task::Parameters&) const
{
  return std::make_shared<Model>(std::move(invariant_initial_state));
}

//==============================================================================
Header Placeholder::Description::generate_header(
  const State&, const Parameters&) const
{
  return Header(_pimpl->category, _pimpl->detail, rmf_traffic::Duration(0));
}

} // namespace events
} // namespace rmf_task_sequence
