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

#include <rmf_task/Request.hpp>

namespace rmf_task {

//==============================================================================
class Request::Implementation
{
public:
  ConstTagPtr tag;
  ConstDescriptionPtr description;
};

//==============================================================================
Request::Request(
  const std::string& id,
  rmf_traffic::Time earliest_start_time,
  ConstPriorityPtr priority,
  ConstDescriptionPtr description,
  bool automatic)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation {
        std::make_shared<Tag>(
          id, earliest_start_time, std::move(priority), automatic),
        std::move(description)
      }))
{
  // Do nothing
}

//==============================================================================
class Request::Tag::Implementation
{
public:
  std::string id;
  rmf_traffic::Time earliest_start_time;
  rmf_task::ConstPriorityPtr priority;
  bool automatic;
};

//==============================================================================
Request::Tag::Tag(
  std::string id_,
  rmf_traffic::Time earliest_start_time_,
  ConstPriorityPtr priority_,
  bool automatic_)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(id_),
        earliest_start_time_,
        std::move(priority_),
        automatic_
      }))
{
  // Do nothing
}

//==============================================================================
const std::string& Request::Tag::id() const
{
  return _pimpl->id;
}

//==============================================================================
rmf_traffic::Time Request::Tag::earliest_start_time() const
{
  return _pimpl->earliest_start_time;
}

//==============================================================================
ConstPriorityPtr Request::Tag::priority() const
{
  return _pimpl->priority;
}

//==============================================================================
bool Request::Tag::automatic() const
{
  return _pimpl->automatic;
}

//==============================================================================
const Request::ConstTagPtr& Request::tag() const
{
  return _pimpl->tag;
}

//==============================================================================
const Request::ConstDescriptionPtr& Request::description() const
{
  return _pimpl->description;
}

} // namespace rmf_task
