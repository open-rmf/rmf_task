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

#ifndef TEST__MOCK__MOCKACTIVITY_HPP
#define TEST__MOCK__MOCKACTIVITY_HPP

#include <rmf_task_sequence/Event.hpp>
#include <rmf_task/events/SimpleEventState.hpp>

namespace test_rmf_task_sequence {

//==============================================================================
class MockActivity
{
public:

  class Active;
  class Standby;

  /// This class gives the test access to the active event that will be
  /// generated by the initializer.
  class Controller
  {
  public:

    std::shared_ptr<Standby> standby;
    std::shared_ptr<Active> active;

  };

  /// The mock event description simply passes along the controller object
  class Description : public rmf_task_sequence::Event::Description
  {
  public:

    Description(std::shared_ptr<Controller> ctrl);

    rmf_task_sequence::Activity::ConstModelPtr make_model(
      rmf_task::State invariant_initial_state,
      const rmf_task::Parameters& parameters) const final;

    rmf_task::Header generate_header(
      const rmf_task::State& initial_state,
      const rmf_task::Parameters& parameters) const final;

    std::shared_ptr<Controller> ctrl;
  };

  struct Signals
  {
    std::function<void()> update;
    std::function<void()> checkpoint;
    std::function<void()> finished;
  };

  class Standby : public rmf_task_sequence::Event::Standby
  {
  public:

    static std::shared_ptr<Standby> make(
      std::shared_ptr<Controller> ctrl,
      const rmf_task::Event::AssignIDPtr& id,
      std::function<void()> update);

    rmf_task::Event::ConstStatePtr state() const final;

    rmf_traffic::Duration duration_estimate() const final;

    rmf_task_sequence::Event::ActivePtr begin(
      std::function<void()> checkpoint,
      std::function<void()> finished) final;

  private:
    std::shared_ptr<Controller> _ctrl;
    std::function<void()> _update;
    rmf_task::events::SimpleEventStatePtr _state_data;
  };

  class Active : public rmf_task_sequence::Event::Active
  {
  public:

    static std::shared_ptr<Active> make(
      std::shared_ptr<Controller> ctrl,
      Signals sigs,
      const rmf_task::Event::AssignIDPtr& id,
      rmf_task::events::SimpleEventStatePtr event = nullptr);

    rmf_task::Event::ConstStatePtr state() const final;

    rmf_traffic::Duration remaining_time_estimate() const final;

    Backup backup() const;

    Resume interrupt(std::function<void()> task_is_interrupted) final;

    void cancel() final;

    void kill() final;

    Signals signals;
    rmf_task::events::SimpleEventStatePtr state_data;
  };

  static void add(const rmf_task_sequence::Event::InitializerPtr& initializer);
};

} // namespace test_rmf_task_sequence

#endif // TEST__MOCK__MOCKACTIVITY_HPP
