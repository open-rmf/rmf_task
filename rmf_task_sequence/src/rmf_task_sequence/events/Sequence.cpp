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

#include <rmf_task_sequence/events/Sequence.hpp>
#include <rmf_task/events/SimpleEvent.hpp>

#include <nlohmann/json.hpp>
#include <nlohmann/json-schema.hpp>

#include <rmf_task_sequence/schemas/ErrorHandler.hpp>
#include <rmf_task_sequence/schemas/backup_EventSequence_v0_1.hpp>

#include <vector>

namespace rmf_task_sequence {
namespace events {

namespace {
//==============================================================================
nlohmann::json convert_to_json(const std::string& input)
{
  nlohmann::json output;
  try
  {
    output = nlohmann::json::parse(input);
  }
  catch (const std::exception&)
  {
    output = input;
  }

  return output;
}
} // anonymous namespace

//==============================================================================
class Sequence::Description::Implementation
{
public:

  Elements elements;
  std::optional<std::string> category;
  std::optional<std::string> detail;

  std::string generate_category() const
  {
    if (category.has_value())
      return *category;

    return "Sequence";
  }

  Header generate_header(
    rmf_task::State initial_state,
    const Parameters& parameters) const
  {
    std::optional<std::vector<nlohmann::json>> detail_json;
    if (!detail.has_value())
      detail_json = std::vector<nlohmann::json>();

    rmf_traffic::Duration duration_estimate = rmf_traffic::Duration(0);

    for (const auto& element : elements)
    {
      const auto element_header =
        element->generate_header(initial_state, parameters);

      duration_estimate += element_header.original_duration_estimate();

      initial_state =
        element->make_model(initial_state, parameters)
        ->invariant_finish_state();

      if (detail_json.has_value())
      {
        nlohmann::json element_output;
        element_output["category"] = element_header.category();
        element_output["detail"] = convert_to_json(element_header.detail());
        detail_json->emplace_back(std::move(element_output));
      }
    }

    std::string output_detail;
    if (detail.has_value())
      output_detail = *detail;
    else
      output_detail = nlohmann::json(*detail_json).dump();

    return Header(
      generate_category(),
      std::move(output_detail),
      duration_estimate);
  }
};

//==============================================================================
Sequence::Description::Description(
  Elements elements,
  std::optional<std::string> category,
  std::optional<std::string> detail)
: _pimpl(rmf_utils::make_impl<Implementation>(
   Implementation{
     std::move(elements),
     std::move(category),
     std::move(detail)
   }))
{
  // Do nothing
}

//==============================================================================
auto Sequence::Description::elements() const -> const Elements&
{
  return _pimpl->elements;
}

//==============================================================================
auto Sequence::Description::elements(Elements new_elements) -> Description&
{
  _pimpl->elements = std::move(new_elements);
  return *this;
}

//==============================================================================
const std::optional<std::string>& Sequence::Description::category() const
{
  return _pimpl->category;
}

//==============================================================================
auto Sequence::Description::category(std::optional<std::string> new_category)
-> Description&
{
  _pimpl->category = std::move(new_category);
  return *this;
}

//==============================================================================
const std::optional<std::string>& Sequence::Description::detail() const
{
  return _pimpl->detail;
}

//==============================================================================
auto Sequence::Description::detail(std::optional<std::string> new_detail)
-> Description&
{
  _pimpl->detail = std::move(new_detail);
  return *this;
}

//==============================================================================
Activity::ConstModelPtr Sequence::Description::make_model(
  rmf_task::State invariant_initial_state,
  const Parameters& parameters) const
{
  return Activity::SequenceModel::make(
    _pimpl->elements,
    std::move(invariant_initial_state),
    parameters);
}

//==============================================================================
Header Sequence::Description::generate_header(
  const rmf_task::State& initial_state,
  const Parameters& parameters) const
{
  return _pimpl->generate_header(initial_state, parameters);
}

//==============================================================================
class BoolGuard
{
public:
  BoolGuard(bool& value)
  : _value(value)
  {
    _value = true;
  }

  ~BoolGuard()
  {
    _value = false;
  }

private:
  bool& _value;
};

//==============================================================================
class SequenceActive;

//==============================================================================
class SequenceStandby : public Event::Standby
{
public:

  static Event::StandbyPtr initiate(
    const Event::Initializer& initializer,
    const std::function<rmf_task::State()>& get_state,
    const ConstParametersPtr& parameters,
    const Sequence::Description& description,
    std::function<void()> parent_update)
  {
    auto state = make_state(description);
    const auto update = [parent_update = std::move(parent_update), state]()
      {
        update_status(*state);
        parent_update();
      };

    std::vector<Event::StandbyPtr> elements;
    for (const auto& desc : description.elements())
    {
      auto element = initializer.initialize(
        get_state, parameters, *desc, update);

      state->add_dependency(element->state());

      elements.emplace_back(std::move(element));
    }

    std::reverse(elements.begin(), elements.end());

    update_status(*state);
    return std::make_shared<SequenceStandby>(
      std::move(elements), std::move(state), std::move(parent_update));
  }

  Event::ConstStatePtr state() const final
  {
    return _state;
  }

  rmf_traffic::Duration duration_estimate() const final
  {
    auto estimate = rmf_traffic::Duration(0);
    for (const auto& element : _elements)
      estimate += element->duration_estimate();

    return estimate;
  }

  Event::ActivePtr begin(
    std::function<void()> checkpoint,
    std::function<void()> finish) final;

  SequenceStandby(
    std::vector<Event::StandbyPtr> elements,
    rmf_task::events::SimpleEventPtr state,
    std::function<void()> parent_update)
  : _elements(std::move(elements)),
    _state(std::move(state)),
    _parent_update(std::move(parent_update))
  {
    // Do nothing
  }

  static rmf_task::events::SimpleEventPtr make_state(
    const Sequence::Description& description)
  {
    return rmf_task::events::SimpleEvent::make(
      description.category().value_or("Sequence"),
      description.detail().value_or(""),
      rmf_task::Event::Status::Standby);
  }

  static void update_status(rmf_task::events::SimpleEvent& state)
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

private:

  std::vector<Event::StandbyPtr> _elements;
  rmf_task::events::SimpleEventPtr _state;
  std::function<void()> _parent_update;
  std::shared_ptr<SequenceActive> _active;
};

//==============================================================================
class SequenceActive
  : public Event::Active,
    public std::enable_shared_from_this<SequenceActive>
{
public:

  static Event::ActivePtr restore(
    const Event::Initializer& initializer,
    const std::function<rmf_task::State()>& get_state,
    const ConstParametersPtr& parameters,
    const Sequence::Description& description,
    const std::string& backup,
    std::function<void()> parent_update,
    std::function<void()> checkpoint,
    std::function<void()> finished)
  {
    auto state = SequenceStandby::make_state(description);
    const auto update = [parent_update = std::move(parent_update), state]()
      {
        SequenceStandby::update_status(*state);
        parent_update();
      };

    std::vector<Event::StandbyPtr> elements;

    const auto backup_state = nlohmann::json::parse(backup);
    if (const auto result =
      schemas::ErrorHandler::has_error(backup_schema_validator, backup_state))
    {
      state->update_log().error(
        "Parsing failed while restoring backup: " + result->message
        + "\nOriginal backup state:\n```" + backup + "\n```");
      state->update_status(Event::Status::Error);
      return std::make_shared<SequenceActive>(
        elements, std::move(state), nullptr, nullptr, nullptr);
    }

    const auto& current_event_json = backup_state["current_event"];
    const auto current_event_index =
      current_event_json["index"].get<uint64_t>();

    const auto& element_descriptions = description.elements();
    if (element_descriptions.size() <= current_event_index)
    {
      state->update_log().error(
        "Failed to restore backup. Index ["
        + std::to_string(current_event_index) + "] is too high for ["
        + std::to_string(description.elements().size()) + "] event elements. "
        "Original text:\n```\n" + backup + "\n```");
      state->update_status(Event::Status::Error);
      return std::make_shared<SequenceActive>(
        elements, std::move(state), nullptr, nullptr, nullptr);
    }

    auto active = std::make_shared<SequenceActive>(
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
      get_state,
      parameters,
      *element_descriptions.at(current_event_index),
      current_event_state,
      update,
      checkpoint,
      event_finished);
    state->add_dependency(active->state());

    for (std::size_t i = current_event_index+1; i < element_descriptions.size(); ++i)
    {
      const auto& desc = element_descriptions[i];
      auto element = initializer.initialize(
        get_state, parameters, *desc, update);

      active->_state->add_dependency(element->state());
      elements.emplace_back(std::move(element));
    }

    std::reverse(elements.begin(), elements.end());

    active->_reverse_remaining = std::move(elements);

    BoolGuard lock(active->_inside_next);
    while (active->_current->state()->finished())
    {
      if (active->_reverse_remaining.empty())
      {
        SequenceStandby::update_status(*active->_state);
        return active;
      }

      ++active->_current_event_index_plus_one;
      const auto next_event = active->_reverse_remaining.back();
      active->_reverse_remaining.pop_back();
      active->_current = next_event->begin(active->_checkpoint, event_finished);
    }

    SequenceStandby::update_status(*active->_state);
    return active;
  }

  Event::ConstStatePtr state() const final
  {
    return _state;
  }

  rmf_traffic::Duration remaining_time_estimate() const final
  {
    auto estimate = rmf_traffic::Duration(0);
    if (_current)
      estimate += _current->remaining_time_estimate();

    for (const auto& element : _reverse_remaining)
      estimate += element->duration_estimate();

    return estimate;
  }

  Backup backup() const final
  {
    nlohmann::json current_event_json;
    current_event_json["index"] = _current_event_index_plus_one - 1;
    current_event_json["state"] = _current->backup().state();

    nlohmann::json backup_json;
    backup_json["schema_version"] = "0.1";
    backup_json["current_event"] = std::move(current_event_json);

    return Backup::make(_next_backup_sequence_number++, backup_json);
  }

  Resume interrupt(std::function<void()> task_is_interrupted) final
  {
    return _current->interrupt(std::move(task_is_interrupted));
  }

  void cancel()
  {
    _reverse_remaining.clear();
    _state->update_status(Event::Status::Canceled);
    _current->cancel();
  }

  void kill()
  {
    _reverse_remaining.clear();
    _state->update_status(Event::Status::Killed);
    _current->kill();
  }

  SequenceActive(
    std::vector<Event::StandbyPtr> elements,
    rmf_task::events::SimpleEventPtr state,
    std::function<void()> parent_update,
    std::function<void()> checkpoint,
    std::function<void()> finished)
  : _current(nullptr),
    _reverse_remaining(elements),
    _state(std::move(state)),
    _parent_update(std::move(parent_update)),
    _checkpoint(std::move(checkpoint)),
    _sequence_finished(std::move(finished))
  {
    // Do nothing
  }

  SequenceActive(
    uint64_t current_event_index,
    rmf_task::events::SimpleEventPtr state,
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

  void next()
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
        SequenceStandby::update_status(*_state);
        _sequence_finished();
        return;
      }

      ++_current_event_index_plus_one;
      const auto next_event = _reverse_remaining.back();
      _reverse_remaining.pop_back();
      _current = next_event->begin(_checkpoint, event_finished);
    } while (_current->state()->finished());

    SequenceStandby::update_status(*_state);
    _parent_update();
  }

private:

  static const nlohmann::json_schema::json_validator backup_schema_validator;

  Event::ActivePtr _current;
  uint64_t _current_event_index_plus_one = 0;
  std::vector<Event::StandbyPtr> _reverse_remaining;
  rmf_task::events::SimpleEventPtr _state;
  std::function<void()> _parent_update;
  std::function<void()> _checkpoint;
  std::function<void()> _sequence_finished;

  // We need to make sure that next() never gets called recursively by events
  // that finish as soon as they are activated
  bool _inside_next = false;
  mutable uint64_t _next_backup_sequence_number = 0;
};

//==============================================================================
Event::ActivePtr SequenceStandby::begin(
  std::function<void()> checkpoint,
  std::function<void()> finish)
{
  if (_active)
    return _active;

  _active = std::make_shared<SequenceActive>(
    std::move(_elements), _state, _parent_update,
    std::move(checkpoint), std::move(finish));

  _active->next();
  return _active;
}

//==============================================================================
const nlohmann::json_schema::json_validator
SequenceActive::backup_schema_validator =
  nlohmann::json_schema::json_validator(
  schemas::backup_EventSequence_v0_1);

//==============================================================================
void Sequence::add(const Event::InitializerPtr& initializer)
{
  add(*initializer, initializer);
}

//==============================================================================
void Sequence::add(
  Event::Initializer& add_to,
  const Event::ConstInitializerPtr& initialize_from)
{
  add_to.add<Sequence::Description>(
    [initialize_from](
      const std::function<rmf_task::State()>& get_state,
      const ConstParametersPtr& parameters,
      const Sequence::Description& description,
      std::function<void()> update)
    {
      return SequenceStandby::initiate(
        *initialize_from,
        get_state,
        parameters,
        description,
        std::move(update));
    },
    [initialize_from](
      const std::function<rmf_task::State()>& get_state,
      const ConstParametersPtr& parameters,
      const Sequence::Description& description,
      const nlohmann::json& backup_state,
      std::function<void()> update,
      std::function<void()> checkpoint,
      std::function<void()> finished)
    {
      return SequenceActive::restore(
        *initialize_from,
        get_state,
        parameters,
        description,
        backup_state,
        std::move(update),
        std::move(checkpoint),
        std::move(finished));
    });
}

} // namespace events
} // namespace rmf_task_sequence
