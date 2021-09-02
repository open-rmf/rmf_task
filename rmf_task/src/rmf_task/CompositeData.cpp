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

#include <rmf_task/CompositeData.hpp>

#include <unordered_map>

namespace rmf_task {

//==============================================================================
class CompositeData::Implementation
{
public:

  std::unordered_map<std::type_index, std::any> data;

};

//==============================================================================
CompositeData::CompositeData()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
void CompositeData::clear()
{
  _pimpl->data.clear();
}

//==============================================================================
std::any* CompositeData::_get(std::type_index type)
{
  const auto it = _pimpl->data.find(type);
  if (it == _pimpl->data.end())
    return nullptr;

  return &it->second;
}

//==============================================================================
const std::any* CompositeData::_get(std::type_index type) const
{
  return const_cast<CompositeData*>(this)->_get(type);
}

//==============================================================================
auto CompositeData::_insert(std::any value, bool or_assign)
-> InsertResult<std::any>
{
  if (or_assign)
  {
    const auto insertion =
      _pimpl->data.insert_or_assign(value.type(), std::move(value));

    return {insertion.second, &insertion.first->second};
  }

  const auto insertion = _pimpl->data.insert({value.type(), std::move(value)});
  return {insertion.second, &insertion.first->second};
}

//==============================================================================
bool CompositeData::_erase(std::type_index type)
{
  return _pimpl->data.erase(type) > 0;
}

} // namespace rmf_task
