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

#include <filesystem>
#include <iostream>
#include <fstream>
#include <rmf_task/BackupFileManager.hpp>

#include <rmf_utils/Modular.hpp>

namespace rmf_task {

//==============================================================================
class BackupFileManager::Implementation
{
public:

  struct Settings
  {
    bool clear_on_startup = false;
    bool clear_on_shutdown = true;
    std::function<void(std::string)> info_logger = nullptr;
    std::function<void(std::string)> debug_logger = nullptr;
  };
  using ConstSettingsPtr = std::shared_ptr<const Settings>;

  Implementation(
    std::filesystem::path directory,
    std::function<void(std::string)> info,
    std::function<void(std::string)> debug)
  : root_directory(std::move(directory)),
    settings(std::make_shared<Settings>())
  {
    settings->info_logger = std::move(info);
    settings->debug_logger = std::move(debug);
  }

  const std::filesystem::path root_directory;
  std::shared_ptr<Settings> settings;

  std::unordered_map<std::string, std::weak_ptr<Group>> groups;
};

//==============================================================================
class BackupFileManager::Group::Implementation
{
public:

  using ConstSettingsPtr = BackupFileManager::Implementation::ConstSettingsPtr;

  Implementation(
    std::filesystem::path directory,
    ConstSettingsPtr settings)
  : group_directory(std::move(directory)),
    settings(std::move(settings))
  {
    std::filesystem::create_directories(this->group_directory);
  }

  template<typename... Args>
  static std::shared_ptr<Group> make(Args&& ... args)
  {
    Group output;
    output._pimpl = rmf_utils::make_unique_impl<Implementation>(
      std::forward<Args>(args)...);

    return std::make_shared<Group>(std::move(output));
  }

  const std::filesystem::path group_directory;
  ConstSettingsPtr settings;

  std::unordered_map<std::string, std::weak_ptr<Robot>> robots;
};

//==============================================================================
class BackupFileManager::Robot::Implementation
{
public:

  using ConstSettingsPtr = BackupFileManager::Implementation::ConstSettingsPtr;

  Implementation(
    std::filesystem::path directory,
    ConstSettingsPtr settings)
  : robot_directory(std::move(directory)),
    settings(std::move(settings))
  {
    if (this->settings->clear_on_startup)
      this->clear_backup();

    std::filesystem::create_directories(this->robot_directory);
  }

  template<typename... Args>
  static std::shared_ptr<Robot> make(Args&& ... args)
  {
    Robot output;
    output._pimpl = rmf_utils::make_unique_impl<Implementation>(
      std::forward<Args>(args)...);

    return std::make_shared<Robot>(std::move(output));
  }

  ~Implementation()
  {
    if (settings->clear_on_shutdown)
      clear_backup();
  }

  const std::filesystem::path robot_directory;
  ConstSettingsPtr settings;
  std::optional<uint64_t> last_seq;
  const std::string backup_file_name = "backup";
  const std::string pre_backup_file_name = ".backup";
  const std::string pre_backup_file_path = robot_directory /
    pre_backup_file_name;
  const std::string backup_file_path = robot_directory / backup_file_name;

  void write_if_new(const Task::Active::Backup& backup)
  {
    if (last_seq.has_value())
    {
      if (rmf_utils::modular(backup.sequence()).less_than_or_equal(*last_seq))
        return;
    }

    last_seq = backup.sequence();
    write(backup.state());
  }

  void log_debug(const std::string msg) const
  {
    if (settings->debug_logger)
    {
      settings->debug_logger(msg);
    }
    else
    {
      std::cout << msg << std::endl;
    }
  }

  void log_info(const std::string msg) const
  {
    if (settings->info_logger)
    {
      settings->info_logger(msg);
    }
    else
    {
      std::cout << msg << std::endl;
    }
  }

private:
  void write(const std::string& state)
  {
    std::ofstream pre_backup(pre_backup_file_path, std::ios::out);
    if (!pre_backup)
      throw std::runtime_error(
              "Could not open file " + pre_backup_file_path +
              " for pre_backup.");
    else
    {
      pre_backup << state;
      pre_backup.close();
      std::filesystem::rename(pre_backup_file_path, backup_file_path);
    }
  }

  void clear_backup()
  {
    if (std::filesystem::exists(robot_directory))
      std::filesystem::remove(pre_backup_file_path);
    std::filesystem::remove(backup_file_path);
  }

};

//==============================================================================
BackupFileManager::BackupFileManager(std::filesystem::path root_directory,
  std::function<void(std::string)> info,
  std::function<void(std::string)> debug)
: _pimpl(rmf_utils::make_unique_impl<Implementation>(
      std::move(root_directory),
      std::move(info),
      std::move(debug)))
{
  // Do nothing
}

//==============================================================================
BackupFileManager& BackupFileManager::clear_on_startup(bool value)
{
  _pimpl->settings->clear_on_startup = value;
  return *this;
}

//==============================================================================
BackupFileManager& BackupFileManager::clear_on_shutdown(bool value)
{
  _pimpl->settings->clear_on_shutdown = value;
  return *this;
}

//==============================================================================
auto BackupFileManager::make_group(std::string name) -> std::shared_ptr<Group>
{
  // TODO(MXG): Consider sanitizing the incoming name
  const auto insertion = _pimpl->groups.insert({name, std::weak_ptr<Group>()});
  const auto& it = insertion.first;
  if (!insertion.second)
  {
    auto group = it->second.lock();
    if (group)
      return group;
  }

  auto group = Group::Implementation::make(
    _pimpl->root_directory / std::filesystem::path(std::move(name)),
    _pimpl->settings);

  it->second = group;
  return group;
}

//==============================================================================
auto BackupFileManager::Group::make_robot(std::string name)
-> std::shared_ptr<Robot>
{
  const auto insertion = _pimpl->robots.insert({name, std::weak_ptr<Robot>()});
  const auto& it = insertion.first;
  if (!insertion.second)
  {
    auto robot = it->second.lock();
    if (robot)
      return robot;
  }

  auto robot = Robot::Implementation::make(
    _pimpl->group_directory / std::filesystem::path(std::move(name)),
    _pimpl->settings);

  it->second = robot;
  return robot;
}

//==============================================================================
BackupFileManager::Group::Group()
{
  // Do nothing
}

//==============================================================================
std::optional<std::string> BackupFileManager::Robot::read() const
{
  if (!std::filesystem::exists(_pimpl->robot_directory))
  {
    throw std::runtime_error("[BackupFileManager::Robot::read] Directory " +
            _pimpl->robot_directory.string() +
            " missing. This should not happen.");
  }

  if (std::filesystem::is_empty(_pimpl->robot_directory))
    return std::nullopt;

  // Check for foreign files
  auto directory_it = std::filesystem::directory_iterator(
    _pimpl->robot_directory);
  for (auto& p: directory_it)
  {
    auto filename = p.path().filename().string();
    if (filename.compare(_pimpl->backup_file_name) != 0 &&
      filename.compare(_pimpl->pre_backup_file_name) != 0)
    {
      throw std::runtime_error("[BackupFileManager::Robot::read] Foreign file " +
              filename + " found. This should be removed.");
    }
  }

  // At this point, file is either backup_file_name, or .backup_file_name, or both
  if (std::filesystem::exists(_pimpl->backup_file_path))
  {
    if (std::filesystem::exists(_pimpl->pre_backup_file_path))
    {
      //suspicious to have both backup files, something definitely broke in the previous run.
      _pimpl->log_debug(
        "[BackupFileManager::Robot::read] Multiple backup files found. This suggests an error with the previous backup run. Using the older edited backup file..");
      std::filesystem::remove(_pimpl->pre_backup_file_path);
    }

    std::ifstream backup(_pimpl->backup_file_path, std::ios::in);
    if (!backup)
      throw std::runtime_error(
              "Could not open file " + _pimpl->backup_file_path +
              " for backup.");
    else
    {
      std::stringstream buffer;
      buffer << backup.rdbuf();
      return std::optional(buffer.str());
    }
  }
  else
  {
    // At this point, we either have exactly .backup, or no files at all
    if (std::filesystem::exists(_pimpl->pre_backup_file_path))
    {
      std::filesystem::remove(_pimpl->pre_backup_file_path);
    }

    return std::nullopt;
  }

}

//==============================================================================
void BackupFileManager::Robot::write(const Task::Active::Backup& backup)
{
  _pimpl->write_if_new(backup);
}

//==============================================================================
BackupFileManager::Robot::Robot()
{
  // Do nothing
}

} // namespace rmf_task

