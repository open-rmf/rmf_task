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

#ifndef RMF_TASK__BACKUPFILEMANAGER_HPP
#define RMF_TASK__BACKUPFILEMANAGER_HPP

#include <rmf_task/Task.hpp>

#include <filesystem>

namespace rmf_task {

//==============================================================================
class BackupFileManager
{
public:

  class Group;
  class Robot;

  /// Construct a BackupFileManager
  ///
  /// \param[in] root_directory
  ///   Specify the root directory that the backup files should live in
  BackupFileManager(std::filesystem::path root_directory,
    std::function<void(std::string)> info_logger = nullptr,
    std::function<void(std::string)> debug_logger = nullptr);

  /// Set whether any previously existing backups should be cleared out on
  /// startup. By default this behavior is turned OFF.
  ///
  /// \param[in] value
  ///   True if the behavior should be turned on; false if it should be turned
  ///   off.
  BackupFileManager& clear_on_startup(bool value = true);

  /// Set whether any currently existing backups should be cleared out on
  /// shutdown. By default this behavior is turned ON.
  ///
  /// \param[in] value
  ///   True if the behavior should be turned on; false if it should be turned
  ///   off.
  BackupFileManager& clear_on_shutdown(bool value = true);

  /// Make a group (a.k.a. fleet) to back up.
  std::shared_ptr<Group> make_group(std::string name);

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class BackupFileManager::Group
{
public:

  /// Make a handle to backup a robot for this group
  ///
  /// \param[in] name
  ///   The unique name of the robot that's being backed up
  std::shared_ptr<Robot> make_robot(std::string name);

  // TODO(MXG): Add an API for saving the task assignments of the Group. When
  // the Group is constructed/destructed, it should clear out those task
  // assignments, according to the RAII settings of its parent BackupFileManager
  // instance.

  class Implementation;
private:
  Group();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class BackupFileManager::Robot
{
public:

  /// Read a backup state from file if a backup file exists for this robot.
  /// If a backup does not exist, return a nullopt.
  std::optional<std::string> read() const;

  /// Write a backup to file
  void write(const Task::Active::Backup& backup);

  class Implementation;
private:
  Robot();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_task

#endif // RMF_TASK__BACKUPFILEMANAGER_HPP
