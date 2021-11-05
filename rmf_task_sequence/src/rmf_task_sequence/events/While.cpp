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

#include <rmf_task_sequence/events/Repeat.hpp>
#include <rmf_task_sequence/events/While.hpp>
#include <rmf_task_sequence/Activity.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
class Model
{
  // No definition of Model class. Description::make_mode() will return a
  // Repeat::Model
};

//==============================================================================
class While::Description::Implementation
{
public:

  Event::ConstDescriptionPtr event;
  std::function<bool()> completed;
  rmf_traffic::Duration while_duration_estimate;
};

//==============================================================================
auto While::Description::make(
  Event::ConstDescriptionPtr event,
  std::function<bool()> completed,
  rmf_traffic::Duration while_duration_estimate) -> DescriptionPtr
{
  auto output = std::shared_ptr<Description>(new Description);
  output->_pimpl =
    rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(event),
        std::move(completed),
        while_duration_estimate});

  return output;
}

//==============================================================================
While::Description::Description()
{
  // Do nothing
}

//==============================================================================
auto While::Description::event() const -> const Event::ConstDescriptionPtr
{
  return _pimpl->event;
}

//==============================================================================
auto While::Description::event(
  Event::ConstDescriptionPtr new_event)-> Description&
{
  _pimpl->event = std::move(new_event);
  return *this;
}

//==============================================================================
const std::function<bool()> While::Description::completed() const
{
  return _pimpl->completed;
}

//==============================================================================
auto While::Description::completed(
  std::function<bool()> new_completed)-> Description&
{
  _pimpl->completed = std::move(new_completed);
  return *this;
}

//==============================================================================
const rmf_traffic::Duration While::Description::while_duration_estimate() const
{
  return _pimpl->while_duration_estimate;
}

//==============================================================================
auto While::Description::while_duration_estimate(
  rmf_traffic::Duration new_duration)-> Description&
{
  _pimpl->while_duration_estimate = new_duration;
  return *this;
}

//==============================================================================
Activity::ConstModelPtr While::Description::make_model(
  State invariant_initial_state,
  const Parameters& parameters) const
{
  const auto& event_header = _pimpl->event->generate_header(
    invariant_initial_state, parameters);
  const auto& event_estimate = event_header.original_duration_estimate();

  // TODO(YV) Warn users if while_duration_estimate is smaller than
  // event estimate
  const std::size_t repetitions = std::max((std::size_t)1, static_cast<
    std::size_t>(_pimpl->while_duration_estimate / event_estimate));

  const auto repeat_event = Repeat::Description::make(
    _pimpl->event,
    repetitions);

  if (repeat_event)
  {
    return repeat_event->make_model(
      invariant_initial_state,
      parameters);
  }

  return nullptr;
}

//==============================================================================
Header While::Description::generate_header(
  const State& state, const Parameters& parameters) const
{
  const auto& original_header =
    _pimpl->event->generate_header(state, parameters);
  const std::string& original_category = original_header.category();
  const auto& estimate = original_header.original_duration_estimate();

  const std::string& category = "While [" + original_category + " ]";
  const std::string& detail = category + " indefinitely";

  return Header(
    category,
    detail,
    _pimpl->while_duration_estimate);
}

} // namespace events
} // namespace rmf_task_sequence
