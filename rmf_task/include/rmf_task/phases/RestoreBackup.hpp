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

#ifndef RMF_TASK__PHASES__RESTOREBACKUP_HPP
#define RMF_TASK__PHASES__RESTOREBACKUP_HPP

#include <rmf_task/Phase.hpp>

namespace rmf_task {
namespace phases {

//==============================================================================
class RestoreBackup : public Phase
{
public:

  class Active;
  using ActivePtr = std::shared_ptr<Active>;

};

//==============================================================================
/// This is a special phase type designated for restoring the backup of a task.
///
/// This phase type uses a reserved phase ID of 0.
class RestoreBackup::Active : public Phase::Active
{
public:

  /// Make an active RestoreBackup phase
  static ActivePtr make(
    const std::string& backup_state_str,
    rmf_traffic::Duration estimated_remaining_time);

  // Documentation inherited
  ConstTagPtr tag() const final;

  // Documentation inherited
  Event::ConstStatePtr final_event() const final;

  // Documentation inherited
  rmf_traffic::Duration estimate_remaining_time() const final;

  /// Call this function if the parsing fails
  void parsing_failed(const std::string& error_message);

  /// Call this function if the restoration fails for some reason besides
  /// parsing
  void restoration_failed(const std::string& error_message);

  /// Call this function if the parsing succeeds
  void restoration_succeeded();

  /// Get the log to pass in some other kind of message
  Log& update_log();

  class Implementation;
private:
  Active();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace phases
} // namespace rmf_task

#endif // RMF_TASK__PHASES__RESTOREBACKUP_HPP
