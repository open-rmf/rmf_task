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
  };
  using ConstSettingsPtr = std::shared_ptr<const Settings>;

  Implementation(std::filesystem::path directory)
  : root_directory(std::move(directory)),
    settings(std::make_shared<Settings>())
  {
    // Do nothing
  }

  std::filesystem::path root_directory;
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
    // Do nothing
  }

  template<typename... Args>
  static std::shared_ptr<Group> make(Args&&... args)
  {
    Group output;
    output._pimpl = rmf_utils::make_unique_impl<Implementation>(
      std::forward<Args>(args)...);

    return std::make_shared<Group>(std::move(output));
  }

  std::filesystem::path group_directory;
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
    if (settings->clear_on_startup)
      clear_backup();
  }

  template<typename... Args>
  static std::shared_ptr<Robot> make(Args&&... args)
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

  std::filesystem::path robot_directory;
  ConstSettingsPtr settings;
  std::optional<uint64_t> last_seq;

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

private:
  void write(const std::string& state)
  {
    throw std::runtime_error(
      "[BackupFileManager::Robot::Implementation::write] not implemented yet");
  }

  void clear_backup()
  {
    throw std::runtime_error(
      "[BackupFileManager::Robot::Implementation::clear_backup] "
      "not implemented yet");
  }
};

//==============================================================================
BackupFileManager::BackupFileManager(std::filesystem::path root_directory)
: _pimpl(rmf_utils::make_unique_impl<Implementation>(std::move(root_directory)))
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
  throw std::runtime_error(
    "[BackupFileManager::Robot::read] not implemented yet");
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
