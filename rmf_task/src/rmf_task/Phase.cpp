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

#include <rmf_task/Phase.hpp>

namespace rmf_task {

//==============================================================================
class Phase::Tag::Implementation
{
public:

  Id id;
  std::string name;
  std::string detail;
  rmf_traffic::Duration duration;

};

//==============================================================================
Phase::Tag::Tag(
  Id id_,
  std::string name_,
  std::string detail_,
  rmf_traffic::Duration estimate_)
: _pimpl(rmf_utils::make_impl<Implementation>(
    Implementation{
      id_,
      std::move(name_),
      std::move(detail_),
      estimate_
    }))
{
  // Do nothing
}

//==============================================================================
auto Phase::Tag::id() const -> Id
{
  return _pimpl->id;
}

//==============================================================================
const std::string& Phase::Tag::name() const
{
  return _pimpl->name;
}

//==============================================================================
const std::string& Phase::Tag::detail() const
{
  return _pimpl->detail;
}

//==============================================================================
rmf_traffic::Duration Phase::Tag::original_duration_estimate() const
{
  return _pimpl->duration;
}

//==============================================================================
class Phase::Pending::Implementation
{
public:

  ConstTagPtr tag;
  bool will_be_skipped = false;

};

//==============================================================================
Phase::Pending::Pending(ConstTagPtr tag)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{std::move(tag)}))
{
  // Do nothing
}

//==============================================================================
auto Phase::Pending::tag() const -> const ConstTagPtr&
{
  return _pimpl->tag;
}

//==============================================================================
bool Phase::Pending::will_be_skipped() const
{
  return _pimpl->will_be_skipped;
}

} // namespace rmf_task
