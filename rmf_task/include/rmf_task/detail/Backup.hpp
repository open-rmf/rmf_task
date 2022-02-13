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

#ifndef RMF_TASK__DETAIL__BACKUP_HPP
#define RMF_TASK__DETAIL__BACKUP_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <cstddef>
#include <string>

namespace rmf_task {
namespace detail {

//==============================================================================
class Backup
{
public:
  /// Make a Backup state
  ///
  /// \param[in] seq
  ///   Sequence number. The Backup from this phase with the highest sequence
  ///   number will be held onto until a Backup with a higher sequence number is
  ///   issued.
  ///
  /// \param[in] state
  ///   A serialization of the phase's state. This will be used by Activator
  ///   when restoring a Task.
  static Backup make(uint64_t seq, std::string state);

  /// Get the sequence number for this backup.
  uint64_t sequence() const;

  /// Set the sequence number for this backup.
  Backup& sequence(uint64_t seq);

  /// Get the serialized state for this backup.
  const std::string& state() const;

  /// Set the serialized state for this backup.
  Backup& state(std::string new_state);

  class Implementation;
private:
  Backup();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace detail
} // namespace rmf_task

#endif // RMF_TASK__DETAIL__BACKUP_HPP
