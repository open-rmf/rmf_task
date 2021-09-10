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

#include <list>

#include <rmf_task/sequence/Task.hpp>

namespace rmf_task {
namespace sequence {

namespace {
//==============================================================================
struct Stage
{
  Phase::
  Phase::ConstDescriptionPtr description;
  std::vector<Phase::ConstDescriptionPtr> cancellation_sequence;
};
} // anonymous namespace

//==============================================================================
class Task::Builder::Implementation
{
public:
  std::vector<Stage> stages;
};

//==============================================================================
class Task::Description::Implementation
{
public:

  std::vector<Stage> stages;

  static std::list<Stage> get_stages(const Description& desc)
  {
    return std::list<Stage>(
      desc._pimpl->stages.begin(),
      desc._pimpl->stages.end());
  }

};

//==============================================================================
class Task::Active
  : public execute::Task,
  public std::enable_shared_from_this<Active>
{
public:

  static execute::TaskPtr make(
    Phase::ConstActivatorPtr phase_activator,
    const Request::ConstTagPtr& tag,
    const Description& description,
    std::optional<std::string> backup_state,
    std::function<void(execute::Phase::ConstSnapshotPtr)> update,
    std::function<void(execute::Phase::ConstCompletedPtr)> phase_finished,
    std::function<void()> task_finished)
  {
    auto task = std::shared_ptr<Active>(
      new Active(
        std::move(phase_activator),
        tag,
        description,
        std::move(update),
        std::move(phase_finished),
        std::move(task_finished)));


  }

  // Documentation inherited
  const std::vector<execute::Phase::ConstCompletedPtr>&
  completed_phases() const final;

  // Documentation inherited
  execute::Phase::ConstActivePtr active_phase() const final;

  // Documentation inherited
  std::vector<execute::Phase::Pending> pending_phases() const final;

  // Documentation inherited
  const Request::ConstTagPtr& tag() const final;

  // Documentation inherited
  rmf_traffic::Time estimate_finish_time() const final;

  // Documentation inherited
  Backup backup() const final;

  // Documentation inherited
  Resume interrupt(std::function<void()> task_is_interrupted) final;

  // Documentation inherited
  void cancel() final;

  // Documentation inherited
  void kill() final;

  // Documentation inherited
  void skip(uint64_t phase_id, bool value = true) final;

  // Documentation inherited
  void rewind(uint64_t phase_id) final;

private:

  Active(
    Phase::ConstActivatorPtr phase_activator,
    const Request::ConstTagPtr& request,
    const Description& description,
    std::function<void(execute::Phase::ConstSnapshotPtr)> update,
    std::function<void(execute::Phase::ConstCompletedPtr)> phase_finished,
    std::function<void()> task_finished)
    : _phase_activator(std::move(phase_activator)),
      _tag(std::move(request)),
      _remaining_stages(Description::Implementation::get_stages(description)),
      _update(std::move(update)),
      _phase_finished(std::move(phase_finished)),
      _task_finished(std::move(task_finished))
  {
    // Do nothing
  }

  Phase::ConstActivatorPtr _phase_activator;
  Request::ConstTagPtr _tag;
  std::list<Stage> _remaining_stages;
  std::function<void(execute::Phase::ConstSnapshotPtr)> _update;
  std::function<void(execute::Phase::ConstCompletedPtr)> _phase_finished;
  std::function<void()> _task_finished;
};

//==============================================================================
auto Task::make_activator(Phase::ConstActivatorPtr phase_activator)
-> execute::TaskActivator::Activate<Description>
{

}

} // namespace sequence
} // namespace rmf_task
