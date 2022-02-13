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

#include <rmf_task/phases/RestoreBackup.hpp>
#include <rmf_task/events/SimpleEventState.hpp>

namespace rmf_task {
namespace phases {

//==============================================================================
class RestoreBackup::Active::Implementation
{
public:

  static ConstTagPtr make_tag()
  {
    return std::make_shared<Tag>(
      0,
      Header(
        "Restore from backup",
        "The task progress is being restored from a backed up state",
        rmf_traffic::Duration(0)));
  }

  Implementation(
    const std::string& backup_state_str,
    rmf_traffic::Duration estimated_remaining_time_)
  : tag(make_tag()),
    event(events::SimpleEventState::make(
        0,
        tag->header().category(),
        tag->header().detail(),
        rmf_task::Event::Status::Underway)),
    estimated_remaining_time(estimated_remaining_time_)
  {
    event->update_log().info(
      "Parsing backup state:\n```\n" + backup_state_str + "\n```");
  }

  ConstTagPtr tag;
  std::shared_ptr<events::SimpleEventState> event;
  rmf_traffic::Duration estimated_remaining_time;

};

//==============================================================================
auto RestoreBackup::Active::make(
  const std::string& backup_state_str,
  rmf_traffic::Duration estimated_remaining_time) -> ActivePtr
{
  Active output;
  output._pimpl = rmf_utils::make_unique_impl<Implementation>(
    std::move(backup_state_str), estimated_remaining_time);

  return std::make_shared<Active>(std::move(output));
}

//==============================================================================
auto RestoreBackup::Active::tag() const -> ConstTagPtr
{
  return _pimpl->tag;
}

//==============================================================================
Event::ConstStatePtr RestoreBackup::Active::final_event() const
{
  return _pimpl->event;
}

//==============================================================================
rmf_traffic::Duration RestoreBackup::Active::estimate_remaining_time() const
{
  return _pimpl->estimated_remaining_time;
}

//==============================================================================
void RestoreBackup::Active::parsing_failed(const std::string& error_message)
{
  _pimpl->event
  ->update_status(Event::Status::Error)
  .update_log().error("Parsing failed: " + error_message);
}

//==============================================================================
void RestoreBackup::Active::restoration_failed(const std::string& error_message)
{
  _pimpl->event
  ->update_status(Event::Status::Error)
  .update_log().error("Restoration failed: " + error_message);
}

//==============================================================================
void RestoreBackup::Active::restoration_succeeded()
{
  _pimpl->event->update_status(Event::Status::Completed);
}

//==============================================================================
Log& RestoreBackup::Active::update_log()
{
  return _pimpl->event->update_log();
}

//==============================================================================
RestoreBackup::Active::Active()
{
  // Do nothing
}

} // namespace phases
} // namespace rmf_task
