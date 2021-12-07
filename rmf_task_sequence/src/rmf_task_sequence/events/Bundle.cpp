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

#include <rmf_task_sequence/events/Bundle.hpp>
#include <rmf_task/events/SimpleEvent.hpp>

#include <nlohmann/json.hpp>
#include <nlohmann/json-schema.hpp>

#include <rmf_task_sequence/schemas/ErrorHandler.hpp>
#include <rmf_task_sequence/schemas/backup_EventSequence_v0_1.hpp>

#include "internal_Sequence.hpp"

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

//==============================================================================
Event::StandbyPtr initiate(
  const Event::Initializer& initializer,
  const std::function<rmf_task::State()>& get_state,
  const ConstParametersPtr& parameters,
  const Bundle::Description& description,
  std::function<void()> parent_update)
{
  if (description.type() == Bundle::Type::Sequence)
  {
    return internal::Sequence::Standby::initiate(
      initializer,
      get_state,
      parameters,
      description,
      std::move(parent_update));
  }

  throw std::runtime_error(
    "Bundle type not yet implemented: " + std::to_string(description.type()));
}

//==============================================================================
Event::ActivePtr restore(
  const Event::Initializer& initializer,
  const std::function<rmf_task::State()>& get_state,
  const ConstParametersPtr& parameters,
  const Bundle::Description& description,
  const std::string& backup,
  std::function<void()> parent_update,
  std::function<void()> checkpoint,
  std::function<void()> finished)
{
  if (description.type() == Bundle::Type::Sequence)
  {
    return internal::Sequence::Active::restore(
      initializer,
      get_state,
      parameters,
      description,
      backup,
      parent_update,
      checkpoint,
      finished);
  }

  throw std::runtime_error(
    "Bundle type not yet implemented: " + std::to_string(description.type()));
}

} // anonymous namespace

//==============================================================================
class Bundle::Description::Implementation
{
public:

  Dependencies dependencies;
  Type type;
  std::optional<std::string> category;
  std::optional<std::string> detail;

  std::string generate_category() const
  {
    if (category.has_value())
      return *category;

    switch (type)
    {
      case Type::Sequence:
        return "Sequence";
      case Type::ParallelAll:
        return "All of";
      case Type::ParallelAny:
        return "One of";
    }

    return "<?Undefined Bundle?>";
  }

  rmf_traffic::Duration adjust_estimate(
    std::optional<rmf_traffic::Duration> current_estimate,
    rmf_traffic::Duration next_dependency_estimate) const
  {
    if (current_estimate.has_value())
    {
      if (Type::ParallelAll == type)
        return std::max(*current_estimate, next_dependency_estimate);
      else if (Type::ParallelAny == type)
        return std::min(*current_estimate, next_dependency_estimate);
    }
    else
    {
      if (Type::ParallelAll == type || Type::ParallelAny == type)
        return next_dependency_estimate;
    }

    return current_estimate.value_or(rmf_traffic::Duration(0))
      + next_dependency_estimate;
  }

  Header generate_header(
    rmf_task::State initial_state,
    const Parameters& parameters) const
  {
    std::optional<std::vector<nlohmann::json>> detail_json;
    if (!detail.has_value())
      detail_json = std::vector<nlohmann::json>();

    std::optional<rmf_traffic::Duration> duration_estimate;

    for (const auto& element : dependencies)
    {
      const auto element_header =
        element->generate_header(initial_state, parameters);

      duration_estimate = adjust_estimate(
        duration_estimate, element_header.original_duration_estimate());

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
      duration_estimate.value_or(rmf_traffic::Duration(0)));
  }
};

//==============================================================================
Bundle::Description::Description(
  Dependencies dependencies,
  Type type,
  std::optional<std::string> category,
  std::optional<std::string> detail)
: _pimpl(rmf_utils::make_impl<Implementation>(
   Implementation{
     std::move(dependencies),
     type,
     std::move(category),
     std::move(detail)
   }))
{
  // Do nothing
}

//==============================================================================
auto Bundle::Description::dependencies() const -> const Dependencies&
{
  return _pimpl->dependencies;
}

//==============================================================================
auto Bundle::Description::dependencies(Dependencies new_dependencies)
-> Description&
{
  _pimpl->dependencies = std::move(new_dependencies);
  return *this;
}

//==============================================================================
const std::optional<std::string>& Bundle::Description::category() const
{
  return _pimpl->category;
}

//==============================================================================
auto Bundle::Description::category(std::optional<std::string> new_category)
-> Description&
{
  _pimpl->category = std::move(new_category);
  return *this;
}

//==============================================================================
const std::optional<std::string>& Bundle::Description::detail() const
{
  return _pimpl->detail;
}

//==============================================================================
auto Bundle::Description::detail(std::optional<std::string> new_detail)
-> Description&
{
  _pimpl->detail = std::move(new_detail);
  return *this;
}

//==============================================================================
Activity::ConstModelPtr Bundle::Description::make_model(
  rmf_task::State invariant_initial_state,
  const Parameters& parameters) const
{
  return Activity::SequenceModel::make(
    _pimpl->dependencies,
    std::move(invariant_initial_state),
    parameters);
}

//==============================================================================
Header Bundle::Description::generate_header(
  const rmf_task::State& initial_state,
  const Parameters& parameters) const
{
  return _pimpl->generate_header(initial_state, parameters);
}

//==============================================================================
void Bundle::add(const Event::InitializerPtr& initializer)
{
  add(*initializer, initializer);
}

//==============================================================================
void Bundle::add(
  Event::Initializer& add_to,
  const Event::ConstInitializerPtr& initialize_from)
{
  add_to.add<Bundle::Description>(
    [initialize_from](
      const std::function<rmf_task::State()>& get_state,
      const ConstParametersPtr& parameters,
      const Bundle::Description& description,
      std::function<void()> update)
    {
      return initiate(
        *initialize_from,
        get_state,
        parameters,
        description,
        std::move(update));
    },
    [initialize_from](
      const std::function<rmf_task::State()>& get_state,
      const ConstParametersPtr& parameters,
      const Bundle::Description& description,
      const nlohmann::json& backup_state,
      std::function<void()> update,
      std::function<void()> checkpoint,
      std::function<void()> finished)
    {
      return restore(
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
