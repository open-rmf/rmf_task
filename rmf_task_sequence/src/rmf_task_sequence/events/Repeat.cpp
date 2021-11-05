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
#include <rmf_task_sequence/Activity.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================

// No definition of Model class. Description::make_mode() will return an
// Activity::SequenceModel

//==============================================================================
class Repeat::Description::Implementation
{
public:

  Event::ConstDescriptionPtr event;
  std::size_t repetitions;
};

//==============================================================================
auto Repeat::Description::make(
  Event::ConstDescriptionPtr event,
  std::size_t repetitions) -> DescriptionPtr
{
  auto output = std::shared_ptr<Description>(new Description);
  output->_pimpl =
    rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(event),
        repetitions});

  return output;
}

//==============================================================================
Repeat::Description::Description()
{
  // Do nothing
}

//==============================================================================
auto Repeat::Description::event() const -> const Event::ConstDescriptionPtr
{
  return _pimpl->event;
}

//==============================================================================
auto Repeat::Description::event(
  Event::ConstDescriptionPtr new_event)-> Description&
{
  _pimpl->event = std::move(new_event);
  return *this;
}

//==============================================================================
std::size_t Repeat::Description::repetitions() const
{
  return _pimpl->repetitions;
}

//==============================================================================
auto Repeat::Description::repetitions(
  std::size_t new_repetitions)-> Description&
{
  _pimpl->repetitions = new_repetitions;
  return *this;
}

//==============================================================================
Activity::ConstModelPtr Repeat::Description::make_model(
  State invariant_initial_state,
  const Parameters& parameters) const
{
  std::vector<Event::ConstDescriptionPtr> descriptions = {};
  for (std::size_t i = 0; i < _pimpl->repetitions; ++i)
    descriptions.push_back(_pimpl->event);

  return Activity::SequenceModel::make()
    descriptions,
    invariant_initial_state,
    parameters);
}

//==============================================================================
Header Repeat::Description::generate_header(
  const State&, const Parameters&) const
{
  const auto& original_header = _pimpl->event->generate_header();
  const std::string& original_category = original_header.category();
  const auto& estimate = original_header.original_duration_estimate();

  const std::string& category = "Repeating [" + original_category + " ]";
  const std::string& detail =
    " " + std::to_string(_pimpl->repetitions) + " times";

  return Header(
    category,
    detail,
    _pimpl->repetitions * estimate);
}
