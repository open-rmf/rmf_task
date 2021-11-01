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
class SequenceStandby : public Event::Standby
{
public:

  static Event::StandbyPtr make_fresh(
    const Event::ConstInitializerPtr& initializer,
    const std::function<rmf_task::State()>& get_state,
    const ConstParametersPtr& parameters,
    const Sequence::Description& description)
  {
    std::vector<Event::StandbyPtr> elements;
    for (const auto& desc : description.elements())
    {
      elements.push_back(
        initializer->initialize(get_state, parameters, *desc, std::nullopt));
    }

    return std::make_shared<SequenceStandby>(description, std::move(elements));
  }

  static Event::StandbyPtr make_restore(
    const Event::ConstInitializerPtr& initializer,
    const std::function<rmf_task::State()>& get_state,
    const ConstParametersPtr& parameters,
    const Sequence::Description& description,
    const std::string& backup)
  {


    std::vector<Event::StandbyPtr> elements;

  }


  const Event::ConstStatePtr& state() const final
  {
    return _state;
  }

  rmf_traffic::Duration duration_estimate() const final
  {

  }

  Event::ActivePtr begin(std::function<void ()> update, std::function<void ()> checkpoint) final
  {

  }

  SequenceStandby(
    const Sequence::Description& desc,
    std::vector<Event::StandbyPtr> elements)
  : _elements(std::move(elements))
  {
    std::vector<Event::ConstStatePtr> element_states;
    for (const auto& element : _elements)
      element_states.emplace_back(element->state());

    _state = rmf_task::events::SimpleEvent::make(
      desc.category().value_or("Sequence"),
      desc.detail().value_or(""),
      rmf_task::Event::Status::Standby,
      std::move(element_states));
  }

private:
  std::vector<Event::StandbyPtr> _elements;
  Event::ConstStatePtr _state;
};

} // namespace events
} // namespace rmf_task_sequence
