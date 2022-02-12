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

#ifndef RMF_TASK_SEQUENCE__EVENT_HPP
#define RMF_TASK_SEQUENCE__EVENT_HPP

#include <rmf_task/Constraints.hpp>
#include <rmf_task/Parameters.hpp>
#include <rmf_task/Estimate.hpp>
#include <rmf_task/Event.hpp>

#include <rmf_task/detail/Resume.hpp>

#include <rmf_task_sequence/Activity.hpp>

#include <nlohmann/json.hpp>

namespace rmf_task_sequence {

//==============================================================================
class Event : public rmf_task::Event
{
public:

  // Event::Active simply uses the Activity::Active API
  class Active;
  using ActivePtr = std::shared_ptr<Active>;

  // Event::Description simply uses the Activity::Description API
  using Description = Activity::Description;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;

  class Standby;
  using StandbyPtr = std::shared_ptr<Standby>;

  class Initializer;
  using InitializerPtr = std::shared_ptr<Initializer>;
  using ConstInitializerPtr = std::shared_ptr<const Initializer>;
};

//==============================================================================
/// The interface of an event that is in a standby mode. This interface is what
/// will be provided by the Event::Initializer. When the right conditions are
/// met for the event to begin, the owner of the event should trigger the
/// begin() function.
class Event::Standby
{
public:

  /// Get the state of this event. The state object returned by this function
  /// must always be the same state object, and it must remain the relevant
  /// state object for this Event after begin(~) has been called.
  virtual ConstStatePtr state() const = 0;

  /// Estimate how long this event will take once it has started
  virtual rmf_traffic::Duration duration_estimate() const = 0;

  /// Tell this event to begin. This function should be implemented to always
  /// return the same Event::Active instance if it gets called more than once.
  ///
  /// \param[in] checkpoint
  ///   A callback that will be triggered when the event reaches a "checkpoint"
  ///   meaning that the task state should be backed up.
  ///
  /// \param[in] finished
  ///   A callback that will be triggered when the event reaches a Finished
  ///   state
  virtual ActivePtr begin(
    std::function<void()> checkpoint,
    std::function<void()> finished) = 0;

  // Virtual destructor
  virtual ~Standby() = default;
};

//==============================================================================
class Event::Active : public Activity::Active
{
public:

  /// Get the state of this event. The state object returned by this function
  /// must always be the same state object, and it must remain the same state
  /// object that would have been provided by the Event::Standby that kicked off
  /// this Event::Active.
  virtual ConstStatePtr state() const = 0;

  /// Estimate how much longer this event will take to complete
  virtual rmf_traffic::Duration remaining_time_estimate() const = 0;

  // Virtual destructor
  virtual ~Active() = default;
};

//==============================================================================
/// The Event::Initializer class is the Event equivalent to the
/// rmf_task::Activator class. It consumes an Event::Description and produces
///
class Event::Initializer
{
public:

  /// Construct an empty Initializer
  Initializer();

  /// Signature for initializing an Event
  ///
  /// \tparam Description
  ///   A class that implements the Event::Description interface
  ///
  /// \param[in] id
  ///   An object to help assign an ID to each event
  ///
  /// \param[in] get_state
  ///   A callback for retrieving the current state of the robot
  ///
  /// \param[in] parameters
  ///   A reference to the parameters for the robot
  ///
  /// \param[in] description
  ///   The down-casted description of the event
  ///
  /// \param[in] backup_state
  ///   The serialized backup state of the Event, if the Event is being restored
  ///   from a crash or disconnection. If the Event is not being restored, a
  ///   std::nullopt will be passed in here.
  ///
  /// \param[in] update
  ///   A callback that will be triggered when a notable change happens for this
  ///   event. \warning The update function must not be triggered during
  ///   initialization because upstream event handlers will not be ready to
  ///   handle it. The event state will always be checked right after
  ///   initialization is finished anyway, so there is no need to trigger this.
  ///
  /// \return an Event in a Standby state
  template<typename Description>
  using Initialize =
    std::function<
    StandbyPtr(
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const ConstParametersPtr& parameters,
      const Description& description,
      std::function<void()> update)
    >;

  /// Signature for restoring an Event
  ///
  /// \tparam Description
  ///   A class that implements the Event::Description interface
  ///
  /// \param[in] id
  ///   An object to help assign an ID to each event
  ///
  /// \param[in] get_state
  ///   A callback for retrieving the current state of the robot
  ///
  /// \param[in] parameters
  ///   A reference to the parameters for the robot
  ///
  /// \param[in] description
  ///   The down-casted description of the event
  ///
  /// \param[in] backup_state
  ///   The backup state of the Event.
  ///
  /// \param[in] update
  ///   A callback that will be triggered when a notable change happens for this
  ///   event. \warning The update function must not be triggered during
  ///   initialization because upstream event handlers will not be ready to
  ///   handle it. The event state will always be checked right after
  ///   initialization is finished anyway, so there is no need to trigger this.
  ///
  /// \param[in] checkpoint
  ///   A callback that will be triggered when the event reaches a "checkpoint"
  ///   meaning that the task state should be backed up.
  ///
  /// \param[in] finished
  ///   A callback that will be triggered when the event reaches a Finished
  ///   state
  ///
  /// \return a restored Event in an Active state
  template<typename Description>
  using Restore =
    std::function<
    ActivePtr(
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const ConstParametersPtr& parameters,
      const Description& description,
      const nlohmann::json& backup_state,
      std::function<void()> update,
      std::function<void()> checkpoint,
      std::function<void()> finished)
    >;

  /// Add a callback to convert from a Description to an event in standby mode
  template<typename Desc>
  void add(
    Initialize<Desc> initializer,
    Restore<Desc> restorer);

  /// Initialize an event
  ///
  /// \param[in] id
  ///   An object to help assign an ID to each event
  ///
  /// \param[in] get_state
  ///   A callback for retrieving the current state of the robot
  ///
  /// \param[in] parameters
  ///   A reference to the parameters for the robot
  ///
  /// \param[in] description
  ///   The description of the event
  ///
  /// \param[in] update
  ///   A callback that will be triggered when a notable change happens for this
  ///   event. \warning The update function must not be triggered during
  ///   initialization because upstream event handlers will not be ready to
  ///   handle it. The event state will always be checked right after
  ///   initialization is finished anyway, so there is no need to trigger this.
  ///
  /// \return an Event in a Standby state
  StandbyPtr initialize(
    const AssignIDPtr& id,
    const std::function<rmf_task::State()>& get_state,
    const ConstParametersPtr& parameters,
    const Event::Description& description,
    std::function<void()> update) const;

  /// Signature for restoring an Event
  ///
  /// \param[in] id
  ///   An object to help assign an ID to each event
  ///
  /// \param[in] get_state
  ///   A callback for retrieving the current state of the robot
  ///
  /// \param[in] parameters
  ///   A reference to the parameters for the robot
  ///
  /// \param[in] description
  ///   The down-casted description of the event
  ///
  /// \param[in] backup_state
  ///   The backup state of the Event.
  ///
  /// \param[in] update
  ///   A callback that will be triggered when a notable change happens for this
  ///   event. \warning The update function must not be triggered during
  ///   initialization because upstream event handlers will not be ready to
  ///   handle it. The event state will always be checked right after
  ///   initialization is finished anyway, so there is no need to trigger this.
  ///
  /// \param[in] checkpoint
  ///   A callback that will be triggered when the event reaches a "checkpoint"
  ///   meaning that the task state should be backed up.
  ///
  /// \param[in] finished
  ///   A callback that will be triggered when the event reaches a Finished
  ///   state
  ///
  /// \return a restored Event in an Active state
  ActivePtr restore(
    const AssignIDPtr& id,
    const std::function<rmf_task::State()>& get_state,
    const ConstParametersPtr& parameters,
    const Event::Description& description,
    const nlohmann::json& backup,
    std::function<void()> update,
    std::function<void()> checkpoint,
    std::function<void()> finished) const;

  class Implementation;
private:
  /// \private
  void _add(
    std::type_index type,
    Initialize<Event::Description> initializer,
    Restore<Event::Description> restorer);

  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_task_sequence

#include <rmf_task_sequence/detail/impl_Event.hpp>

#endif // RMF_TASK_SEQUENCE__EVENT_HPP
