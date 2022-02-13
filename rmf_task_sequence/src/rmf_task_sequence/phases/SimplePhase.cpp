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

#include <rmf_task_sequence/phases/SimplePhase.hpp>

namespace rmf_task_sequence {
namespace phases {

//==============================================================================
class SimplePhase::Description::Implementation
{
public:

  std::optional<std::string> category;
  std::optional<std::string> detail;
  Event::ConstDescriptionPtr final_event;

  Header generate_header(
    const State& initial_state,
    const Parameters& parameters) const
  {
    const auto duration =
      final_event->make_model(initial_state, parameters)->invariant_duration();

    if (category.has_value() && detail.has_value())
      return Header(*category, *detail, duration);

    auto event_header = final_event->generate_header(initial_state, parameters);
    const std::string& c = category.has_value() ?
      *category : event_header.category();
    const std::string& d = detail.has_value() ?
      *detail : event_header.detail();

    return Header(c, d, duration);
  }
};

//==============================================================================
class SimplePhase::Active
  : public Phase::Active,
  public std::enable_shared_from_this<Active>
{
public:

  ConstTagPtr tag() const final
  {
    return _tag;
  }

  Event::ConstStatePtr final_event() const final
  {
    return _final_event->state();
  }

  rmf_traffic::Duration estimate_remaining_time() const final
  {
    return _final_event->remaining_time_estimate();
  }

  Backup backup() const final
  {
    return _final_event->backup();
  }

  Resume interrupt(std::function<void()> task_is_interrupted) final
  {
    return _final_event->interrupt(std::move(task_is_interrupted));
  }

  void cancel() final
  {
    _final_event->cancel();
  }

  void kill() final
  {
    _final_event->kill();
  }

  ConstTagPtr _tag;
  Event::ActivePtr _final_event;
};

//==============================================================================
void SimplePhase::add(
  Activator& phase_activator,
  const Event::ConstInitializerPtr& event_initializer)
{
  phase_activator.add_activator<SimplePhase::Description>(
    [event_initializer](
      const std::function<State()>& get_state,
      const ConstParametersPtr& parameters,
      ConstTagPtr tag,
      const SimplePhase::Description& desc,
      std::optional<nlohmann::json> backup_state,
      std::function<void(rmf_task::Phase::ConstSnapshotPtr)> phase_update,
      std::function<void(Active::Backup)> phase_checkpoint,
      std::function<void()> finished) -> ActivePtr
    {
      const auto phase = std::make_shared<Active>();
      assert(tag != nullptr);
      phase->_tag = tag;

      std::function<void()> event_update =
      [
        weak = phase->weak_from_this(),
        phase_update = std::move(phase_update)
      ]()
      {
        if (const auto phase = weak.lock())
        {
          if (phase->_final_event)
            phase_update(Phase::Snapshot::make(*phase));
        }
      };

      std::function<void()> event_checkpoint =
      [
        weak = phase->weak_from_this(),
        phase_checkpoint = std::move(phase_checkpoint)
      ]()
      {
        if (const auto phase = weak.lock())
        {
          if (phase->_final_event)
            phase_checkpoint(phase->backup());
        }
      };

      const auto assign_id = Event::AssignID::make();

      if (backup_state.has_value())
      {
        phase->_final_event = event_initializer->restore(
          assign_id,
          get_state,
          parameters,
          *desc.final_event(),
          *backup_state,
          std::move(event_update),
          std::move(event_checkpoint),
          std::move(finished));

        return phase;
      }

      const auto init_event = event_initializer->initialize(
        assign_id,
        get_state,
        parameters,
        *desc.final_event(),
        std::move(event_update));

      phase->_final_event = init_event->begin(
        std::move(event_checkpoint), std::move(finished));

      return phase;
    });
}

//==============================================================================
SimplePhase::DescriptionPtr SimplePhase::Description::make(
  Event::ConstDescriptionPtr final_event,
  std::optional<std::string> category,
  std::optional<std::string> detail)
{
  Description desc;
  desc._pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{
      std::move(category),
      std::move(detail),
      std::move(final_event)
    });

  return std::make_shared<Description>(std::move(desc));
}

//==============================================================================
const Event::ConstDescriptionPtr& SimplePhase::Description::final_event() const
{
  return _pimpl->final_event;
}

//==============================================================================
SimplePhase::Description& SimplePhase::Description::final_event(
  Event::ConstDescriptionPtr new_final_event)
{
  _pimpl->final_event = std::move(new_final_event);
  return *this;
}

//==============================================================================
const std::optional<std::string>& SimplePhase::Description::category() const
{
  return _pimpl->category;
}

//==============================================================================
SimplePhase::Description& SimplePhase::Description::category(
  std::optional<std::string> new_category)
{
  _pimpl->category = std::move(new_category);
  return *this;
}

//==============================================================================
const std::optional<std::string>& SimplePhase::Description::detail() const
{
  return _pimpl->detail;
}

//==============================================================================
SimplePhase::Description& SimplePhase::Description::detail(
  std::optional<std::string> new_detail)
{
  _pimpl->detail = std::move(new_detail);
  return *this;
}

//==============================================================================
Activity::ConstModelPtr SimplePhase::Description::make_model(
  State invariant_initial_state,
  const Parameters& parameters) const
{
  return _pimpl->final_event->make_model(
    std::move(invariant_initial_state),
    parameters);
}

//==============================================================================
Header SimplePhase::Description::generate_header(
  const State& initial_state,
  const Parameters& parameters) const
{
  return _pimpl->generate_header(initial_state, parameters);
}

//==============================================================================
SimplePhase::Description::Description()
{
  // Do nothing
}

} // namespace phases
} // namespace rmf_task_sequence
