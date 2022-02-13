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

#include <rmf_task/detail/Backup.hpp>

namespace rmf_task {
namespace detail {

//==============================================================================
class Backup::Implementation
{
public:
  uint64_t sequence;
  std::string state;
};

//==============================================================================
Backup Backup::make(uint64_t seq, std::string state)
{
  Backup output;
  output._pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{seq, std::move(state)});

  return output;
}

//==============================================================================
uint64_t Backup::sequence() const
{
  return _pimpl->sequence;
}

//==============================================================================
Backup& Backup::sequence(uint64_t seq)
{
  _pimpl->sequence = seq;
  return *this;
}

//==============================================================================
const std::string& Backup::state() const
{
  return _pimpl->state;
}

//==============================================================================
Backup& Backup::state(std::string new_state)
{
  _pimpl->state = std::move(new_state);
  return *this;
}

//==============================================================================
Backup::Backup()
{
  // Do nothing
}

} // namespace detail
} // namespace rmf_task
