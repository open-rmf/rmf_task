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

#ifndef SRC__RMF_TASK_SEQUENCE__EVENTS__INTERNAL_SEQUENCE_HPP
#define SRC__RMF_TASK_SEQUENCE__EVENTS__INTERNAL_SEQUENCE_HPP

#include <rmf_task_sequence/events/Bundle.hpp>
#include <rmf_task/events/SimpleEventState.hpp>

#include <rmf_task_sequence/schemas/ErrorHandler.hpp>
#include <rmf_task_sequence/schemas/backup_EventSequence_v0_1.hpp>

namespace rmf_task_sequence {
namespace events {
namespace internal {

//==============================================================================
class BoolGuard
{
public:
  BoolGuard(bool& value)
  : _value(value)
  {
    _value = true;
  }

  ~BoolGuard()
  {
    _value = false;
  }

private:
  bool& _value;
};

//==============================================================================
class Sequence
{
public:

  class Standby;
  class Active;

};

//==============================================================================
class Sequence::Standby : public Event::Standby
{
public:

  static Event::StandbyPtr initiate(
    const Event::Initializer& initializer,
    const Event::AssignIDPtr& id,
    const std::function<rmf_task::State()>& get_state,
    const ConstParametersPtr& parameters,
    const Bundle::Description& description,
    std::function<void()> parent_update);

  using MakeStandby = std::function<Event::StandbyPtr(Bundle::UpdateFn)>;

  static Event::StandbyPtr initiate(
    const std::vector<MakeStandby>& dependencies,
    rmf_task::events::SimpleEventStatePtr state,
    std::function<void()> update);

  Event::ConstStatePtr state() const final;

  rmf_traffic::Duration duration_estimate() const final;

  Event::ActivePtr begin(
    std::function<void()> checkpoint,
    std::function<void()> finish) final;

  Standby(
    std::vector<Event::StandbyPtr> reverse_dependencies,
    rmf_task::events::SimpleEventStatePtr state,
    std::function<void()> parent_update);

  static rmf_task::events::SimpleEventStatePtr make_state(
    const Event::AssignIDPtr& id,
    const Bundle::Description& description);

  static void update_status(rmf_task::events::SimpleEventState& state);

private:

  std::vector<Event::StandbyPtr> _reverse_dependencies;
  rmf_task::events::SimpleEventStatePtr _state;
  std::function<void()> _parent_update;
  std::shared_ptr<Sequence::Active> _active;
};

//==============================================================================
class Sequence::Active
  : public Event::Active,
  public std::enable_shared_from_this<Sequence::Active>
{
public:

  static Event::ActivePtr restore(
    const Event::Initializer& initializer,
    const Event::AssignIDPtr& id,
    const std::function<rmf_task::State()>& get_state,
    const ConstParametersPtr& parameters,
    const Bundle::Description& description,
    const std::string& backup,
    std::function<void()> parent_update,
    std::function<void()> checkpoint,
    std::function<void()> finished);

  Event::ConstStatePtr state() const final;

  rmf_traffic::Duration remaining_time_estimate() const final;

  Backup backup() const final;

  Resume interrupt(std::function<void()> task_is_interrupted) final;

  void cancel();

  void kill();

  Active(
    std::vector<Event::StandbyPtr> dependencies,
    rmf_task::events::SimpleEventStatePtr state,
    std::function<void()> parent_update,
    std::function<void()> checkpoint,
    std::function<void()> finished);

  Active(
    uint64_t current_event_index,
    rmf_task::events::SimpleEventStatePtr state,
    std::function<void()> parent_update,
    std::function<void()> checkpoint,
    std::function<void()> finished);

  void next();

private:

  static const nlohmann::json_schema::json_validator backup_schema_validator;

  Event::ActivePtr _current;
  uint64_t _current_event_index_plus_one = 0;
  std::vector<Event::StandbyPtr> _reverse_remaining;
  rmf_task::events::SimpleEventStatePtr _state;
  std::function<void()> _parent_update;
  std::function<void()> _checkpoint;
  std::function<void()> _sequence_finished;

  // We need to make sure that next() never gets called recursively by events
  // that finish as soon as they are activated
  bool _inside_next = false;
  mutable uint64_t _next_backup_sequence_number = 0;
};

} // namespace internal
} // namespace events
} // namespace rmf_task_sequence

#endif // SRC__RMF_TASK_SEQUENCE__EVENTS__INTERNAL_SEQUENCE_HPP
