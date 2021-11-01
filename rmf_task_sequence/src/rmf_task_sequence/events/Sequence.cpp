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

} // namespace events
} // namespace rmf_task_sequence
