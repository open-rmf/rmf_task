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

  class Description;

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
  ///   This Initializer will be used by the Bundle to initialize the
  ///   events that it depends on. This may be a different initializer than the
  ///   one that the event sequence is added to.
  static void add(
    Event::Initializer& add_to,
    const Event::ConstInitializerPtr& initialize_from);

  /// Given an event description, return a vector of other event descriptions.
  template<typename OtherDesc>
  using UnfoldDescription =
    std::function<Bundle::Description(const OtherDesc&)>;

  /// Give an initializer the ability to initialize an event bundle for some
  /// other event description. This is useful when you want a certain event to
  /// be implemented as a bundle of other events without requiring users to
  /// explicitly specify an event bundle.
  ///
  /// This is also useful if there is an event type that is safe to initialize
  /// when bundled in a specific way with other events but should not be run on
  /// its own. You can keep the Description type of that event private to the
  /// downstream user but load it into the initializer for this bundle.
  ///
  /// \param[in] unfold_description
  ///   This will be used to produce the bundle to create
  ///
  /// \param[in] add_to
  ///   This Initializer will be given the ability to initialize this type of
  ///   event bundle.
  ///
  /// \param[in] initialize_from
  ///   This Initializer will be used to initialize the dependencies of this
  ///   Bundle.
  template<typename OtherDesc>
  static void unfold(
    const UnfoldDescription<OtherDesc>& unfold_description,
    Event::Initializer& add_to,
    const Event::ConstInitializerPtr& initialize_from);

  using UpdateFn = std::function<void()>;
  using DependencySpecifiers = std::vector<std::function<StandbyPtr(UpdateFn)>>;

  /// Create a Bundle on standby by directly providing the standby dependencies
  /// and the state object.
  ///
  /// \param[in] type
  ///   The type of bundle to activate
  ///
  /// \param[in] dependencies
  ///   Factories for the dependencies that are being bundled together. Each
  ///   factory should take in an update callback and then give back the
  ///   StandbyPtr for the dependency.
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
    const DependencySpecifiers& dependencies,
    rmf_task::events::SimpleEventStatePtr state,
    std::function<void()> update);

  /// Initiate a Bundle in Standby mode. For advanced use only.
  ///
  /// \warning It is not recommended to use this function directly. You should
  /// consider using add(~) or unfold(~) with an initializer instead.
  static Event::StandbyPtr initiate(
    const Event::Initializer& initializer,
    const Event::AssignIDPtr& id,
    const std::function<rmf_task::State()>& get_state,
    const ConstParametersPtr& parameters,
    const Bundle::Description& description,
    std::function<void()> update);

  /// Restore a Bundle into Active mode. For advanced use only.
  ///
  /// \warning It is not recommended to use this function directly. You should
  /// consider using add(~) or unfold(~) with an initializer instead.
  static Event::ActivePtr restore(
    const Event::Initializer& initializer,
    const Event::AssignIDPtr& id,
    const std::function<rmf_task::State()>& get_state,
    const ConstParametersPtr& parameters,
    const Bundle::Description& description,
    const std::string& backup,
    std::function<void()> update,
    std::function<void()> checkpoint,
    std::function<void()> finished);
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

#include <rmf_task_sequence/events/detail/impl_Bundle.hpp>

#endif // RMF_TASK_SEQUENCE__EVENTS__BUNDLE_HPP
