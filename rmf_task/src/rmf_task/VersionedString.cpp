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

#include <rmf_task/VersionedString.hpp>

#include <unordered_map>

namespace rmf_task {

//==============================================================================
class VersionedString::Implementation
{
public:

  Implementation(std::string initial_value)
  : value(std::make_shared<std::string>(std::move(initial_value)))
  {
    // Do nothing
  }

  using ValuePtr = std::shared_ptr<const std::string>;
  ValuePtr value;

  // The token is used to uniquely identify this VersionedString.
  struct Token {};
  using TokenPtr = std::shared_ptr<const Token>;
  TokenPtr token = std::make_shared<Token>();

  View make_view() const;
};

//==============================================================================
class VersionedString::View::Implementation
{
public:

  using ValuePtr = VersionedString::Implementation::ValuePtr;
  using TokenPtr = VersionedString::Implementation::TokenPtr;

  static View make(ValuePtr value, TokenPtr token)
  {
    View output;
    output._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(value),
        std::move(token)
      });

    return output;
  }

  static const Implementation& get(const View& view)
  {
    return *view._pimpl;
  }

  ValuePtr value;
  TokenPtr token;

};

//==============================================================================
class VersionedString::Reader::Implementation
{
public:

  using ValuePtr = VersionedString::Implementation::ValuePtr;
  using WeakValuePtr = ValuePtr::weak_type;

  using Token = VersionedString::Implementation::Token;
  using TokenPtr = VersionedString::Implementation::TokenPtr;
  using WeakTokenPtr = TokenPtr::weak_type;

  struct Memory
  {
    WeakValuePtr last_value;
    WeakTokenPtr token;

    Memory()
    {
      // Do nothing
    }
  };

  std::unordered_map<const Token*, Memory> memories;

  ValuePtr read(const View& view)
  {
    const auto& v = View::Implementation::get(view);
    const auto it = memories.insert({v.token.get(), Memory()}).first;
    auto& memory = it->second;
    if (memory.token.lock())
    {
      // If we can successfully lock the memory token, then we do remember this
      // same versioned string.
      if (const auto last_value = memory.last_value.lock())
      {
        if (last_value == v.value)
        {
          // If our memory of the last_value is still valid and matches with the
          // value that we are receiving now, then this value is a duplicate.
          return nullptr;
        }
      }
    }

    memory.token = v.token;
    memory.last_value = v.value;
    return v.value;
  }
};

//==============================================================================
auto VersionedString::Implementation::make_view() const -> View
{
  return VersionedString::View::Implementation::make(value, token);
}

//==============================================================================
VersionedString::VersionedString(std::string initial_value)
: _pimpl(rmf_utils::make_unique_impl<Implementation>(std::move(initial_value)))
{
  // Do nothing
}

//==============================================================================
void VersionedString::update(std::string new_value)
{
  _pimpl->value = std::make_shared<std::string>(std::move(new_value));
}

//==============================================================================
auto VersionedString::view() const -> View
{
  return _pimpl->make_view();
}

//==============================================================================
VersionedString::View::View()
{
  // Do nothing
}

//==============================================================================
VersionedString::Reader::Reader()
: _pimpl(rmf_utils::make_unique_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<const std::string> VersionedString::Reader::read(
  const View& view)
{
  return _pimpl->read(view);
}

} // namespace rmf_task
