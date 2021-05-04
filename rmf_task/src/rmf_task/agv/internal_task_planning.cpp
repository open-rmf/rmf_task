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

#include "internal_task_planning.hpp"

namespace rmf_task {
namespace agv {

// ============================================================================
void Candidates::update_map()
{
  for (auto it = _value_map.begin(); it != _value_map.end(); ++it)
  {
    const auto c = it->second.candidate;
    if (_candidate_map.size() <= c)
      _candidate_map.resize(c+1);

    _candidate_map[c] = it;
  }
}

// ============================================================================
Candidates::Candidates(Candidates::Map candidate_values)
: _value_map(std::move(candidate_values))
{
  update_map();
}

// ============================================================================
void Candidates::update_candidate(
  std::size_t candidate,
  State state,
  rmf_traffic::Time wait_until,
  State previous_state,
  bool require_charge_battery)
{
  const auto it = _candidate_map.at(candidate);
  _value_map.erase(it);
  _candidate_map[candidate] = _value_map.insert(
    {
      state.finish_time(),
      Entry{candidate, state, wait_until, previous_state, require_charge_battery}
    });
}

// ============================================================================
rmf_traffic::Time Candidates::best_finish_time() const
{
  assert(!_value_map.empty());
  return _value_map.begin()->first;
}

// ============================================================================
Candidates::Range Candidates::best_candidates() const
{
  assert(!_value_map.empty());

  Range range;
  range.begin = _value_map.begin();
  auto it = range.begin;
  while (it->first == range.begin->first)
    ++it;

  range.end = it;
  return range;
}

// ============================================================================
Candidates& Candidates::operator=(const Candidates& other)
{
  _value_map = other._value_map;
  update_map();
  return *this;
}

// ============================================================================
Candidates::Candidates(const Candidates& other)
{
  _value_map = other._value_map;
  update_map();
}

// ============================================================================
std::shared_ptr<Candidates> Candidates::make(
  const std::vector<State>& initial_states,
  const Constraints& constraints,
  const rmf_task::Request& request,
  const rmf_task::requests::ChargeBatteryDescription& charge_battery_desc,
  const std::shared_ptr<EstimateCache> estimate_cache,
  bool drain_battery,
  TaskPlanner::TaskPlannerError& error)
{
  Map initial_map;
  for (std::size_t i = 0; i < initial_states.size(); ++i)
  {
    const auto& state = initial_states[i];
    const auto finish = request.description()->estimate_finish(
      state, constraints, estimate_cache, drain_battery);
    if (finish.has_value())
    {
      initial_map.insert({
          finish.value().finish_state().finish_time(),
          Entry{
            i,
            finish.value().finish_state(),
            finish.value().wait_until(),
            state,
            false}});
    }
    else
    {
      auto battery_estimate =
        charge_battery_desc.estimate_finish(
        state, constraints, estimate_cache, drain_battery);
      if (battery_estimate.has_value())
      {
        auto new_finish = request.description()->estimate_finish(
          battery_estimate.value().finish_state(),
          constraints,
          estimate_cache,
          drain_battery);
        if (new_finish.has_value())
        {
          initial_map.insert(
            {new_finish.value().finish_state().finish_time(),
              Entry{
                i,
                new_finish.value().finish_state(),
                new_finish.value().wait_until(),
                state,
                true}});
        }
        else
        {
          error = TaskPlanner::TaskPlannerError::limited_capacity;
        }

      }
      else
      {
        // Control reaches here either if ChargeBattery::estimate_finish() was
        // called on initial state with full battery or low battery such that
        // agent is unable to make it back to the charger
        if (abs(
            state.battery_soc() - charge_battery_desc.max_charge_soc()) < 1e-3)
          error = TaskPlanner::TaskPlannerError::limited_capacity;
        else
          error = TaskPlanner::TaskPlannerError::low_battery;
      }
    }
  }

  if (initial_map.empty())
  {
    return nullptr;
  }

  std::shared_ptr<Candidates> candidates(
    new Candidates(std::move(initial_map)));
  return candidates;
}

// ============================================================================
PendingTask::PendingTask(
  ConstRequestPtr request_,
  Candidates candidates_)
: request(std::move(request_)),
  candidates(candidates_)
{
  // Do nothing
}

// ============================================================================
std::shared_ptr<PendingTask> PendingTask::make(
  const std::vector<rmf_task::agv::State>& initial_states,
  const rmf_task::agv::Constraints& constraints,
  const rmf_task::ConstRequestPtr request_,
  const rmf_task::Request::DescriptionPtr charge_battery_desc,
  const std::shared_ptr<EstimateCache> estimate_cache,
  bool drain_battery,
  TaskPlanner::TaskPlannerError& error)
{

  auto battery_desc = std::dynamic_pointer_cast<
    const rmf_task::requests::ChargeBatteryDescription>(charge_battery_desc);

  const auto candidates = Candidates::make(initial_states, constraints,
      *request_, *battery_desc, estimate_cache, drain_battery, error);

  if (!candidates)
    return nullptr;

  std::shared_ptr<PendingTask> pending_task(
    new PendingTask(request_, *candidates));
  return pending_task;
}

} // namespace agv
} // namespace rmf_task
