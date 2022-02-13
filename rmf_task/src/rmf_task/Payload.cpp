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

#include <unordered_map>
#include <unordered_set>
#include <sstream>

#include <rmf_task/Payload.hpp>

namespace rmf_task {

//==============================================================================
class Payload::Implementation
{
public:

  std::vector<Component> components;

};

//==============================================================================
Payload::Payload(std::vector<Component> components)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{std::move(components)}))
{
  // Do nothing
}

//==============================================================================
auto Payload::components() const -> const std::vector<Component>&
{
  return _pimpl->components;
}

//==============================================================================
std::string Payload::brief(const std::string& compartment_prefix) const
{
  std::unordered_map<std::string, std::size_t> item_quantities;
  std::unordered_set<std::string> compartments;

  for (const auto& component : _pimpl->components)
  {
    auto it = item_quantities.insert({component.sku(), 0}).first;
    it->second += component.quantity();

    compartments.insert(component.compartment());
  }

  if (item_quantities.empty())
    return "nothing";

  std::stringstream ss;
  if (item_quantities.size() == 1)
  {
    const auto it = item_quantities.cbegin();
    const auto& type = it->first;
    const auto quantity = it->second;
    ss << quantity << " of [" << type << "]";
  }
  else
  {
    const auto num_types = item_quantities.size();
    std::size_t total_quantity = 0;
    for (const auto& item : item_quantities)
      total_quantity += item.second;

    ss << num_types << " types of items (" << total_quantity << " total units)";
  }

  if (compartments.size() == 1)
  {
    const auto it = compartments.cbegin();
    ss << " " << compartment_prefix << " [" << *it << "]";
  }
  else
  {
    const auto quantity = compartments.size();
    ss << " " << compartment_prefix << " " << quantity << " compartments";
  }

  return ss.str();
}

//==============================================================================
class Payload::Component::Implementation
{
public:

  std::string sku;
  uint32_t quantity;
  std::string compartment;

};

//==============================================================================
Payload::Component::Component(
  std::string sku,
  uint32_t quantity,
  std::string compartment)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(sku),
        quantity,
        std::move(compartment)
      }))
{
  // Do nothing
}

//==============================================================================
const std::string& Payload::Component::sku() const
{
  return _pimpl->sku;
}

//==============================================================================
uint32_t Payload::Component::quantity() const
{
  return _pimpl->quantity;
}

//==============================================================================
const std::string& Payload::Component::compartment() const
{
  return _pimpl->compartment;
}

} // namespace rmf_task
