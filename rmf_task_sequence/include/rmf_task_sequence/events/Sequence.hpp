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

#ifndef RMF_TASK_SEQUENCE__EVENTS__SEQUENCE_HPP
#define RMF_TASK_SEQUENCE__EVENTS__SEQUENCE_HPP

#include <rmf_task_sequence/Event.hpp>

#include <vector>
#include <optional>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
class Sequence : public Event
{
public:

  /// Make an initializer for Event Sequences
  ///
  /// \param[in] initializer
  ///   The Initializer that should be used to initialize other events
  static Initializer::Initialize<Sequence::Description>
  make_initializer(ConstInitializerPtr initializer);

  class Description;
};

//==============================================================================
class Sequence::Description : public Event::Description
{
public:

  using Elements = std::vector<Event::ConstDescriptionPtr>;

  /// Construct a Sequence Description
  ///
  /// \param[in] elements
  ///   These are the events that the sequence will run through. Each event in
  ///   the sequence will be in Standby mode until the previous one reaches a
  ///   finished status.
  ///
  /// \param[in] category
  ///   Optionally give a category to this sequence. If left unspecified, the
  ///   category will simply be "Sequence".
  ///
  /// \param[in] detail
  ///   Optionally give some detail to this sequence. If left unspecified, the
  ///   detail will simply aggregate the details of the dependencies.
  Description(
    Elements elements,
    std::optional<std::string> category = std::nullopt,
    std::optional<std::string> detail = std::nullopt);

  /// Get the elements of the sequence
  const Elements& elements() const;

  /// Change the elements of the sequence
  Description& elements(Elements new_elements);

  /// Get the category settings
  const std::optional<std::string>& category() const;

  /// Change the category settings
  Description& category(std::optional<std::string> new_category);

  /// Get the detail settings
  const std::optional<std::string>& detail() const;

  /// Change the detail settings
  Description& detail(std::optional<std::string> new_detail);

  // Documentation inherited
  Activity::ConstModelPtr make_model(
    rmf_task::State invariant_initial_state,
    const Parameters& parameters) const final;

  // Documentation inherited
  Header generate_header(
    const rmf_task::State& initial_state,
    const Parameters& parameters) const final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace events
} // namespace rmf_task_sequence


#endif // RMF_TASK_SEQUENCE__EVENTS__SEQUENCE_HPP
