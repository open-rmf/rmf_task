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

#ifndef RMF_TASK_SEQUENCE__DETAIL__BACKUP_HPP
#define RMF_TASK_SEQUENCE__DETAIL__BACKUP_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <nlohmann/json.hpp>

namespace rmf_task_sequence {
namespace detail {

//==============================================================================
/// Backup data for a Phase or Event. The state is represented by a
/// nlohmann::json data structure. The meaning and format of the json structure
/// is up to the phase or event implementation to decide.
///
/// Each Backup is tagged with a sequence number. As the phase or event makes
/// progress, it can issue new Backups with higher sequence numbers. Only the
/// Backup with the highest sequence number will be kept.
class Backup
{
public:

  /// Make a backup of the phase
  ///
  /// \param[in] seq
  ///   Sequence number. The Backup from this phase with the highest sequence
  ///   number will be held onto until a Backup with a higher sequence number is
  ///   issued.
  ///
  /// \param[in] state
  ///   A serialization of the phase's state. This will be used by
  ///   Phase::Activator when restoring a Task.
  static Backup make(uint64_t seq, nlohmann::json state);

  /// Get the sequence number
  uint64_t sequence() const;

  /// Get the YAML representation of the backed up state
  const nlohmann::json& state() const;

  class Implementation;
private:
  Backup();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace detail
} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__DETAIL__BACKUP_HPP
