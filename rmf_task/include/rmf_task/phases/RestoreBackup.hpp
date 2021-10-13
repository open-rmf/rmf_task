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

  class Condition;
  using ConditionPtr = std::shared_ptr<Condition>;
};

//==============================================================================
class RestoreBackup::Active : public Phase::Active
{
public:

  /// Make an active RestoreBackup phase
  static ActivePtr make(std::string backup_state_str);

  // Documentation inherited
  ConstTagPtr tag() const final;

  // Documentation inherited
  ConstConditionPtr finish_condition() const final;




  class Implementation;
private:
  Active();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace phases
} // namespace rmf_task

#endif // RMF_TASK__PHASES__RESTOREBACKUP_HPP
