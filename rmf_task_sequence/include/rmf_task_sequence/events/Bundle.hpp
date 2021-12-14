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

#ifndef RMF_TASK_SEQUENCE__EVENTS__BUNDLE_HPP
#define RMF_TASK_SEQUENCE__EVENTS__BUNDLE_HPP

#include <rmf_task_sequence/Event.hpp>
#include <rmf_task/events/SimpleEventState.hpp>

#include <vector>
#include <optional>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
class Bundle : public Event
{
public:

  enum Type
  {
    /// The bundle's dependencies will be executed one-by-one in sequence. The
    /// bundle will finished when the last of its events reaches a finished
    /// status.
    Sequence,

    /// The bundle will execute its dependencies in parallel and will finish
    /// when all of its dependencies are finished.
    // (Not implemented yet)
//    ParallelAll,

    /// The bundle will execute its dependencies in parallel and will finished
    /// when any (one or more) of its dependencies finishes.
    // (Not implemented yet)
//    ParallelAny
  };

  /// Give an initializer the ability to initialize event bundles
  ///
  /// \param[in] initializer
  ///   The Initializer that should be used to initialize other events, and
  ///   also will be given the ability to initialize event sequences. This is
  ///   equivalent to the overload of this function, but where add_to and
  ///   initialize_from are the same initializer.
  static void add(const Event::InitializerPtr& initializer);

  /// Give an initializer the ability to initialize event sequences
  ///
  /// \param[in] add_to
  ///   This Initializer will be given the ability to initialize event sequences
  ///
  /// \param[in] initialize_from
  ///   This Initializer will be used by the Event Sequence to initialize the
  ///   events that it depends on. This may be a different initializer than the
  ///   one that the event sequence is added to.
  static void add(
    Event::Initializer& add_to,
    const Event::ConstInitializerPtr& initialize_from);

  /// Create a Bundle on standby by directly providing the standby dependencies
  /// and the state object.
  ///
  /// \param[in] type
  ///   The type of bundle to activate
  ///
  /// \param[in] dependencies
  ///   The dependencies that are being bundled together
  ///
  /// \param[in] state
  ///   The state to modify as the bundle progresses. This class will not modify
  ///   the name or detail of the state.
  ///
  /// \param[in] update
  ///   The callback that will be triggered when the bundle has an update.
  ///
  static StandbyPtr standby(
    Type type,
    std::vector<StandbyPtr> dependencies,
    rmf_task::events::SimpleEventStatePtr state,
    std::function<void()> update);

  class Description;
};

//==============================================================================
class Bundle::Description : public Event::Description
{
public:

  using Dependencies = std::vector<Event::ConstDescriptionPtr>;

  /// Construct a Sequence Description
  ///
  /// \param[in] dependencies
  ///   These are the events that the bundle will depend on.
  ///
  /// \param[in] type
  ///   The type of the bundle, which determines its behavior.
  ///
  /// \param[in] category
  ///   Optionally give a category to this bundle. If left unspecified, the
  ///   category will be based on its type.
  ///
  /// \param[in] detail
  ///   Optionally give some detail to this bundle. If left unspecified, the
  ///   detail will simply aggregate the details of the dependencies.
  Description(
    Dependencies dependencies,
    Type type,
    std::optional<std::string> category = std::nullopt,
    std::optional<std::string> detail = std::nullopt);

  /// Get the elements of the sequence
  const Dependencies& dependencies() const;

  /// Change the elements of the sequence
  Description& dependencies(Dependencies new_elements);

  /// Get the type of bundle this is
  Type type() const;

  /// Change the type of bundle that this is
  Description& type(Type new_type);

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


#endif // RMF_TASK_SEQUENCE__EVENTS__BUNDLE_HPP
