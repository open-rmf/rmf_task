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

#include "internal_Sequence.hpp"

namespace rmf_task_sequence {
namespace events {
namespace internal {

//==============================================================================
Event::StandbyPtr Sequence::Standby::initiate(
  const Event::Initializer& initializer,
  const Event::AssignIDPtr& id,
  const std::function<rmf_task::State()>& get_state,
  const ConstParametersPtr& parameters,
  const Bundle::Description& description,
  std::function<void()> parent_update)
{
  auto state = make_state(id, description);
  const auto update = [parent_update, state]()
    {
      update_status(*state);
      parent_update();
    };

  std::vector<Event::StandbyPtr> dependencies;
  dependencies.reserve(description.dependencies().size());
  for (const auto& desc : description.dependencies())
  {
    auto element = initializer.initialize(
      id, get_state, parameters, *desc, update);

    dependencies.emplace_back(std::move(element));
  }

  // We reverse the dependencies so we can always pop them off the back of the
  // queue.
  std::reverse(dependencies.begin(), dependencies.end());

  return std::make_shared<Sequence::Standby>(
    std::move(dependencies), std::move(state), std::move(parent_update));
}

//==============================================================================
Event::StandbyPtr Sequence::Standby::initiate(
  const std::vector<MakeStandby>& dependencies_fn,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> parent_update)
{
  const auto update = [parent_update, state]()
    {
      update_status(*state);
      parent_update();
    };

  std::vector<Event::StandbyPtr> dependencies;
  dependencies.reserve(dependencies_fn.size());
  for (const auto& fn : dependencies_fn)
    dependencies.push_back(fn(update));

  // We reverse the dependencies so we can always pop them off the back of the
  // queue.
  std::reverse(dependencies.begin(), dependencies.end());

  return std::make_shared<Sequence::Standby>(
    std::move(dependencies), std::move(state), std::move(parent_update));
}

//==============================================================================
Event::ConstStatePtr Sequence::Standby::state() const
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration Sequence::Standby::duration_estimate() const
{
  auto estimate = rmf_traffic::Duration(0);
  for (const auto& element : _reverse_dependencies)
    estimate += element->duration_estimate();

  return estimate;
}

//==============================================================================
Sequence::Standby::Standby(
  std::vector<Event::StandbyPtr> reverse_dependencies,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> parent_update)
: _reverse_dependencies(std::move(reverse_dependencies)),
  _state(std::move(state)),
  _parent_update(std::move(parent_update))
{
  std::vector<rmf_task::Event::ConstStatePtr> state_deps;
  state_deps.reserve(_reverse_dependencies.size());
  const auto rbegin = _reverse_dependencies.rbegin();
  const auto rend = _reverse_dependencies.rend();
  for (auto rit = rbegin; rit != rend; ++rit)
    state_deps.push_back((*rit)->state());

  _state->update_dependencies(std::move(state_deps));
  update_status(*_state);
}

//==============================================================================
Event::ActivePtr Sequence::Standby::begin(
  std::function<void()> checkpoint,
  std::function<void()> finish)
{
  if (_active)
    return _active;

  _active = std::make_shared<Sequence::Active>(
    std::move(_reverse_dependencies), _state, _parent_update,
    std::move(checkpoint), std::move(finish));

  _active->next();
  return _active;
}

//==============================================================================
rmf_task::events::SimpleEventStatePtr Sequence::Standby::make_state(
  const Event::AssignIDPtr& id,
  const Bundle::Description& description)
{
  return rmf_task::events::SimpleEventState::make(
    id->assign(),
    description.category().value_or("Sequence"),
    description.detail().value_or(""),
    rmf_task::Event::Status::Standby);
}

//==============================================================================
void Sequence::Standby::update_status(rmf_task::events::SimpleEventState& state)
{
  if (state.status() == Event::Status::Canceled
    || state.status() == Event::Status::Killed
    || state.status() == Event::Status::Skipped)
    return;

  Event::Status status = Event::Status::Completed;
  for (const auto& dep : state.dependencies())
    status = Event::sequence_status(status, dep->status());

  state.update_status(status);
}

//==============================================================================
Event::ActivePtr Sequence::Active::restore(
  const Event::Initializer& initializer,
  const Event::AssignIDPtr& id,
  const std::function<rmf_task::State()>& get_state,
  const ConstParametersPtr& parameters,
  const Bundle::Description& description,
  const std::string& backup,
  std::function<void()> parent_update,
  std::function<void()> checkpoint,
  std::function<void()> finished)
{
  auto state = Sequence::Standby::make_state(id, description);
  const auto update = [parent_update = std::move(parent_update), state]()
    {
      Sequence::Standby::update_status(*state);
      parent_update();
    };

  std::vector<Event::StandbyPtr> dependencies;

  const auto backup_state = nlohmann::json::parse(backup);
  if (const auto result =
    schemas::ErrorHandler::has_error(backup_schema_validator, backup_state))
  {
    state->update_log().error(
      "Parsing failed while restoring backup: " + result->message
      + "\nOriginal backup state:\n```" + backup + "\n```");
    state->update_status(Event::Status::Error);
    return std::make_shared<Sequence::Active>(
      dependencies, std::move(state), nullptr, nullptr, nullptr);
  }

  const auto& current_event_json = backup_state["current_event"];
  const auto current_event_index =
    current_event_json["index"].get<uint64_t>();

  const auto& element_descriptions = description.dependencies();
  if (element_descriptions.size() <= current_event_index)
  {
    state->update_log().error(
      "Failed to restore backup. Index ["
      + std::to_string(current_event_index) + "] is too high for ["
      + std::to_string(description.dependencies().size())
      + "] event dependencies. Original text:\n```\n" + backup + "\n```");
    state->update_status(Event::Status::Error);
    return std::make_shared<Sequence::Active>(
      dependencies, std::move(state), nullptr, nullptr, nullptr);
  }

  auto active = std::make_shared<Sequence::Active>(
    current_event_index,
    std::move(state),
    parent_update,
    std::move(checkpoint),
    std::move(finished));

  const auto event_finished = [me = active->weak_from_this()]()
    {
      if (const auto self = me.lock())
        self->next();
    };

  const auto& current_event_state = current_event_json["state"];
  active->_current = initializer.restore(
    id,
    get_state,
    parameters,
    *element_descriptions.at(current_event_index),
    current_event_state,
    update,
    checkpoint,
    event_finished);
  state->add_dependency(active->state());

  const std::size_t num_elements = element_descriptions.size();
  for (std::size_t i = current_event_index + 1; i < num_elements; ++i)
  {
    const auto& desc = element_descriptions[i];
    auto element = initializer.initialize(
      id, get_state, parameters, *desc, update);

    active->_state->add_dependency(element->state());
    dependencies.emplace_back(std::move(element));
  }

  std::reverse(dependencies.begin(), dependencies.end());

  active->_reverse_remaining = std::move(dependencies);

  BoolGuard lock(active->_inside_next);
  while (active->_current->state()->finished())
  {
    if (active->_reverse_remaining.empty())
    {
      Sequence::Standby::update_status(*active->_state);
      return active;
    }

    ++active->_current_event_index_plus_one;
    const auto next_event = active->_reverse_remaining.back();
    active->_reverse_remaining.pop_back();
    active->_current = next_event->begin(active->_checkpoint, event_finished);
  }

  Sequence::Standby::update_status(*active->_state);
  return active;
}

//==============================================================================
Event::ConstStatePtr Sequence::Active::state() const
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration Sequence::Active::remaining_time_estimate() const
{
  auto estimate = rmf_traffic::Duration(0);
  if (_current)
    estimate += _current->remaining_time_estimate();

  for (const auto& element : _reverse_remaining)
    estimate += element->duration_estimate();

  return estimate;
}

//==============================================================================
Event::Active::Backup Sequence::Active::backup() const
{
  nlohmann::json current_event_json;
  current_event_json["index"] = _current_event_index_plus_one - 1;
  current_event_json["state"] = _current->backup().state();

  nlohmann::json backup_json;
  backup_json["schema_version"] = "0.1";
  backup_json["current_event"] = std::move(current_event_json);

  return Backup::make(_next_backup_sequence_number++, backup_json);
}

//==============================================================================
Event::Active::Resume Sequence::Active::interrupt(
  std::function<void()> task_is_interrupted)
{
  return _current->interrupt(std::move(task_is_interrupted));
}

//==============================================================================
void Sequence::Active::cancel()
{
  _reverse_remaining.clear();
  _state->update_status(Event::Status::Canceled);
  _current->cancel();
}

//==============================================================================
void Sequence::Active::kill()
{
  _reverse_remaining.clear();
  _state->update_status(Event::Status::Killed);
  _current->kill();
}

//==============================================================================
Sequence::Active::Active(
  std::vector<Event::StandbyPtr> dependencies,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> parent_update,
  std::function<void()> checkpoint,
  std::function<void()> finished)
: _current(nullptr),
  _reverse_remaining(dependencies),
  _state(std::move(state)),
  _parent_update(std::move(parent_update)),
  _checkpoint(std::move(checkpoint)),
  _sequence_finished(std::move(finished))
{
  // Do nothing
}

//==============================================================================
Sequence::Active::Active(
  uint64_t current_event_index,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> parent_update,
  std::function<void()> checkpoint,
  std::function<void()> finished)
: _current(nullptr),
  _current_event_index_plus_one(current_event_index+1),
  _state(std::move(state)),
  _parent_update(std::move(parent_update)),
  _checkpoint(std::move(checkpoint)),
  _sequence_finished(std::move(finished))
{
  // Do nothing
}

//==============================================================================
void Sequence::Active::next()
{
  if (_inside_next)
    return;

  BoolGuard lock(_inside_next);

  const auto event_finished = [me = weak_from_this()]()
    {
      if (const auto self = me.lock())
        self->next();
    };

  do
  {
    if (_reverse_remaining.empty())
    {
      Sequence::Standby::update_status(*_state);
      _sequence_finished();
      return;
    }

    ++_current_event_index_plus_one;
    const auto next_event = _reverse_remaining.back();
    _reverse_remaining.pop_back();
    _current = next_event->begin(_checkpoint, event_finished);
  } while (_current->state()->finished());

  Sequence::Standby::update_status(*_state);
  _parent_update();
  _checkpoint();
}

//==============================================================================
const nlohmann::json_schema::json_validator
Sequence::Active::backup_schema_validator =
  nlohmann::json_schema::json_validator(
  schemas::backup_EventSequence_v0_1);

} // namespace internal
} // namespace events
} // namespace rmf_task_sequence
