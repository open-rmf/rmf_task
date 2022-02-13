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

#include <rmf_task_sequence/detail/Backup.hpp>

namespace rmf_task_sequence {
namespace detail {

//==============================================================================
class Backup::Implementation
{
public:

  uint64_t seq;
  nlohmann::json state;
};

//==============================================================================
Backup::Backup()
{
  // Do nothing
}

//==============================================================================
Backup Backup::make(uint64_t seq, nlohmann::json state)
{
  Backup backup;
  backup._pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{seq, state});
  return backup;
}

//==============================================================================
uint64_t Backup::sequence() const
{
  return _pimpl->seq;
}

//==============================================================================
const nlohmann::json& Backup::state() const
{
  return _pimpl->state;
}


} // namespace detail
} // namespace rmf_task_sequence