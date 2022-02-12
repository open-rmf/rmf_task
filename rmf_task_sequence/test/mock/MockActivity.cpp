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

#include <rmf_task_sequence/events/WaitFor.hpp>

#include "MockActivity.hpp"

namespace test_rmf_task_sequence {

//==============================================================================
MockActivity::Description::Description(std::shared_ptr<Controller> ctrl_)
: ctrl(std::move(ctrl_))
{
  // Do nothing
}

//==============================================================================
rmf_task_sequence::Activity::ConstModelPtr
MockActivity::Description::make_model(
  rmf_task::State invariant_initial_state,
  const rmf_task::Parameters& parameters) const
{
  return rmf_task_sequence::events::WaitFor::Description::make(
    rmf_traffic::Duration(0))->make_model(
    std::move(invariant_initial_state), parameters);
}

//==============================================================================
rmf_task::Header MockActivity::Description::generate_header(
  const rmf_task::State&,
  const rmf_task::Parameters&) const
{
  return rmf_task::Header(
    "Mock Activity", "Mocking an activity", rmf_traffic::Duration(0));
}

//==============================================================================
auto MockActivity::Standby::make(
  std::shared_ptr<Controller> ctrl,
  const rmf_task::Event::AssignIDPtr& id,
  std::function<void()> update) -> std::shared_ptr<Standby>
{
  auto standby = std::make_shared<Standby>();
  standby->_ctrl = std::move(ctrl);
  standby->_update = std::move(update);
  standby->_state_data = rmf_task::events::SimpleEventState::make(
    id->assign(), "Mock Activity", "Mocking an activity",
    rmf_task::Event::Status::Standby);

  return standby;
}

//==============================================================================
rmf_task::Event::ConstStatePtr MockActivity::Standby::state() const
{
  return _state_data;
}

//==============================================================================
rmf_traffic::Duration MockActivity::Standby::duration_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
rmf_task_sequence::Event::ActivePtr MockActivity::Standby::begin(
  std::function<void()> checkpoint,
  std::function<void()> finished)
{
  return Active::make(
    _ctrl,
    Signals{
      _update,
      std::move(checkpoint),
      std::move(finished)
    },
    nullptr,
    _state_data);
}

//==============================================================================
auto MockActivity::Active::make(
  std::shared_ptr<Controller> ctrl,
  Signals signals,
  const rmf_task::Event::AssignIDPtr& id,
  rmf_task::events::SimpleEventStatePtr event) -> std::shared_ptr<Active>
{
  auto output = std::make_shared<Active>();
  output->signals = std::move(signals);
  output->state_data = std::move(event);
  ctrl->active = output;

  if (!output->state_data)
  {
    output->state_data = rmf_task::events::SimpleEventState::make(
      id->assign(), "Mock Activity", "Mocking an activity",
      rmf_task::Event::Status::Underway);
  }
  else
  {
    output->state_data->update_status(rmf_task::Event::Status::Underway);
  }

  return output;
}

//==============================================================================
rmf_task::Event::ConstStatePtr MockActivity::Active::state() const
{
  return state_data;
}

//==============================================================================
rmf_traffic::Duration MockActivity::Active::remaining_time_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto MockActivity::Active::backup() const -> Backup
{
  return Backup::make(0, nlohmann::json());
}

//==============================================================================
auto MockActivity::Active::interrupt(std::function<void()> task_is_interrupted)
-> Resume
{
  state_data->update_status(rmf_task::Event::Status::Standby);
  task_is_interrupted();
  return Resume::make(
    [state_data = state_data]()
    {
      state_data->update_status(rmf_task::Event::Status::Underway);
    });
}

//==============================================================================
void MockActivity::Active::cancel()
{
  state_data->update_status(rmf_task::Event::Status::Canceled);
  signals.finished();
}

//==============================================================================
void MockActivity::Active::kill()
{
  state_data->update_status(rmf_task::Event::Status::Killed);
  signals.finished();
}

//==============================================================================
void MockActivity::Active::update(
  rmf_task::Event::Status status,
  std::string text)
{
  state_data->update_status(status);
  state_data->update_log().info(std::move(text));
  signals.update();
}

//==============================================================================
void MockActivity::Active::complete()
{
  state_data->update_status(rmf_task::Event::Status::Completed);
  state_data->update_log().info("Completed");
  signals.finished();
}

//==============================================================================
void MockActivity::add(
  const rmf_task_sequence::Event::InitializerPtr& initializer)
{
  initializer->add<MockActivity::Description>(
    [](
      const rmf_task::Event::AssignIDPtr& id,
      const std::function<rmf_task::State()>&,
      const rmf_task::ConstParametersPtr&,
      const MockActivity::Description& description,
      std::function<void()> update)
    {
      return Standby::make(description.ctrl, id, std::move(update));
    },
    [](
      const rmf_task::Event::AssignIDPtr& id,
      const std::function<rmf_task::State()>&,
      const rmf_task::ConstParametersPtr&,
      const MockActivity::Description& description,
      const nlohmann::json&,
      std::function<void()> update,
      std::function<void()> checkpoint,
      std::function<void()> finished)
    {
      return Active::make(
        description.ctrl,
        Signals{
          std::move(update),
          std::move(checkpoint),
          std::move(finished)
        },
        id);
    });
}

} // namespace test_rmf_task_sequence
