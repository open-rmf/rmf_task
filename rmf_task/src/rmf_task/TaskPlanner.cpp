/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <rmf_task/Estimate.hpp>
#include <rmf_task/State.hpp>
#include <rmf_task/BinaryPriorityScheme.hpp>
#include <rmf_task/requests/ChargeBattery.hpp>

#include "BinaryPriorityCostCalculator.hpp"

#include <rmf_traffic/Time.hpp>

#include <limits>
#include <queue>

namespace rmf_task {

//==============================================================================
class TaskPlanner::Configuration::Implementation
{
public:

  Parameters parameters;
  Constraints constraints;
  ConstCostCalculatorPtr cost_calculator;
};

//==============================================================================
TaskPlanner::Configuration::Configuration(
  Parameters parameters,
  Constraints constraints,
  ConstCostCalculatorPtr cost_calculator)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(parameters),
        std::move(constraints),
        std::move(cost_calculator),
      }))
{
  // Do nothing
}

//==============================================================================
const Parameters& TaskPlanner::Configuration::parameters() const
{
  return _pimpl->parameters;
}

//==============================================================================
auto TaskPlanner::Configuration::parameters(Parameters parameters)
-> Configuration&
{
  _pimpl->parameters = std::move(parameters);
  return *this;
}

//==============================================================================
const Constraints& TaskPlanner::Configuration::constraints() const
{
  return _pimpl->constraints;
}

//==============================================================================
auto TaskPlanner::Configuration::constraints(Constraints constraints)
-> Configuration&
{
  _pimpl->constraints = std::move(constraints);
  return *this;
}

//==============================================================================
const ConstCostCalculatorPtr&
TaskPlanner::Configuration::cost_calculator() const
{
  return _pimpl->cost_calculator;
}

//==============================================================================
auto TaskPlanner::Configuration::cost_calculator(
  ConstCostCalculatorPtr cost_calculator) -> Configuration&
{
  _pimpl->cost_calculator = std::move(cost_calculator);
  return *this;
}

//==============================================================================
class TaskPlanner::Options::Implementation
{
public:

  bool greedy;
  std::function<bool()> interrupter;
  ConstRequestFactoryPtr finishing_request;
};

//==============================================================================
TaskPlanner::Options::Options(
  bool greedy,
  std::function<bool()> interrupter,
  ConstRequestFactoryPtr finishing_request)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(greedy),
        std::move(interrupter),
        std::move(finishing_request)
      }))
{
  // Do nothing
}

//==============================================================================
auto TaskPlanner::Options::greedy(bool value) -> Options&
{
  _pimpl->greedy = value;
  return *this;
}

//==============================================================================
bool TaskPlanner::Options::greedy() const
{
  return _pimpl->greedy;
}

//==============================================================================
auto TaskPlanner::Options::interrupter(std::function<bool()> interrupter)
-> Options&
{
  _pimpl->interrupter = std::move(interrupter);
  return *this;
}

//==============================================================================
const std::function<bool()>& TaskPlanner::Options::interrupter() const
{
  return _pimpl->interrupter;
}

//==============================================================================
auto TaskPlanner::Options::finishing_request(
  ConstRequestFactoryPtr finishing_request) -> Options&
{
  _pimpl->finishing_request = std::move(finishing_request);
  return *this;
}

//==============================================================================
ConstRequestFactoryPtr TaskPlanner::Options::finishing_request() const
{
  return _pimpl->finishing_request;
}

//==============================================================================
class TaskPlanner::Assignment::Implementation
{
public:

  rmf_task::ConstRequestPtr request;
  State state;
  rmf_traffic::Time deployment_time;
};

//==============================================================================
TaskPlanner::Assignment::Assignment(
  rmf_task::ConstRequestPtr request,
  State state,
  rmf_traffic::Time deployment_time)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(request),
        std::move(state),
        deployment_time
      }))
{
  // Do nothing
}

//==============================================================================
const rmf_task::ConstRequestPtr& TaskPlanner::Assignment::request() const
{
  return _pimpl->request;
}

//==============================================================================
const State& TaskPlanner::Assignment::finish_state() const
{
  return _pimpl->state;
}

//==============================================================================
const rmf_traffic::Time TaskPlanner::Assignment::deployment_time() const
{
  return _pimpl->deployment_time;
}

//==============================================================================

namespace {

// ============================================================================
// The type of filter used for solving the task assignment problem
enum class FilterType
{
  Passthrough,
  Trie,
  Hash
};

// ============================================================================
class Filter
{
public:

  Filter(FilterType type, const std::size_t N_tasks)
  : _type(type),
    _set(N_tasks, AssignmentHash(N_tasks))
  {
    // Do nothing
  }

  bool ignore(const Node& node);

private:

  struct TaskTable;

  struct AgentTable
  {
    std::unordered_map<std::size_t, std::unique_ptr<TaskTable>> agent;
  };

  struct TaskTable
  {
    std::unordered_map<std::size_t, std::unique_ptr<AgentTable>> task;
  };

  struct AssignmentHash
  {
    AssignmentHash(std::size_t N)
    {
      // We add 1 to N because
      _shift = std::ceil(std::log2(N+1));
    }

    std::size_t operator()(const Node::AssignedTasks& assignments) const
    {
      std::size_t output = 0;
      std::size_t count = 0;
      for (const auto& a : assignments)
      {
        for (const auto& s : a)
        {
          // We add 1 to the task_id to differentiate between task_id == 0 and
          // a task being unassigned.
          const std::size_t id = s.internal_id + 1;
          output += id << (_shift * (count++));
        }
      }

      return output;
    }

    std::size_t _shift;
  };

  struct AssignmentEqual
  {
    bool operator()(
      const Node::AssignedTasks& A, const Node::AssignedTasks& B) const
    {
      if (A.size() != B.size())
        return false;

      for (std::size_t i = 0; i < A.size(); ++i)
      {
        const auto& a = A[i];
        const auto& b = B[i];

        if (a.size() != b.size())
          return false;

        for (std::size_t j = 0; j < a.size(); ++j)
        {
          if (a[j].internal_id != b[j].internal_id)
          {
            return false;
          }
        }
      }

      return true;
    }
  };

  using Set = std::unordered_set<Node::AssignedTasks, AssignmentHash,
      AssignmentEqual>;

  FilterType _type;
  AgentTable _root;
  Set _set;
};

bool Filter::ignore(const Node& node)
{
  if (_type == FilterType::Passthrough)
    return false;

  if (_type == FilterType::Hash)
    return !_set.insert(node.assigned_tasks).second;

  bool new_node = false;

  AgentTable* agent_table = &_root;
  std::size_t a = 0;
  std::size_t t = 0;
  while (a < node.assigned_tasks.size())
  {
    const auto& current_agent = node.assigned_tasks.at(a);

    if (t < current_agent.size())
    {
      const auto& task_id = current_agent[t].internal_id;
      const auto agent_insertion = agent_table->agent.insert({a, nullptr});
      if (agent_insertion.second)
        agent_insertion.first->second = std::make_unique<TaskTable>();

      auto* task_table = agent_insertion.first->second.get();

      const auto task_insertion = task_table->task.insert({task_id, nullptr});
      if (task_insertion.second)
      {
        new_node = true;
        task_insertion.first->second = std::make_unique<AgentTable>();
      }

      agent_table = task_insertion.first->second.get();
      ++t;
    }
    else
    {
      t = 0;
      ++a;
    }
  }

  return !new_node;
}

// ============================================================================
const rmf_traffic::Duration segmentation_threshold =
  rmf_traffic::time::from_seconds(1.0);

} // anonymous namespace

// ============================================================================
class TaskPlanner::Implementation
{
public:

  Configuration config;
  Options default_options;
  ConstTravelEstimatorPtr travel_estimator;
  bool check_priority = false;
  ConstCostCalculatorPtr cost_calculator = nullptr;

  ConstRequestPtr make_charging_request(rmf_traffic::Time start_time)
  {
    return rmf_task::requests::ChargeBattery::make(start_time);
  }

  TaskPlanner::Assignments prune_assignments(
    TaskPlanner::Assignments& assignments)
  {
    for (std::size_t a = 0; a < assignments.size(); ++a)
    {
      if (assignments[a].empty())
        continue;

      // Remove charging task at end of assignments if any
      // TODO(YV): Remove this after fixing the planner
      if (std::dynamic_pointer_cast<
          const rmf_task::requests::ChargeBattery::Description>(
          assignments[a].back().request()->description()))
        assignments[a].pop_back();
    }

    return assignments;
  }

  ConstNodePtr prune_assignments(ConstNodePtr parent)
  {
    auto node = std::make_shared<Node>(*parent);

    for (auto& agent : node->assigned_tasks)
    {
      if (agent.empty())
        continue;

      if (std::dynamic_pointer_cast<
          const rmf_task::requests::ChargeBattery::Description>(
          agent.back().assignment.request()->description()))
        agent.pop_back();
    }

    return node;
  }

  void append_finishing_request(
    const RequestFactory& factory,
    TaskPlanner::Assignments& complete_assignments)
  {
    for (auto& agent : complete_assignments)
    {
      if (agent.empty())
      {
        continue;
      }

      const auto& state = agent.back().finish_state();
      const auto request = factory.make_request(state);

      // TODO(YV) Currently we are unable to recursively call complete_solve()
      // here as the prune_assignments() function will remove any ChargeBattery
      // requests at the back of the assignments. But the finishing factory
      // could be a ChargeBattery request and hence this approach does not work.
      // When we fix the logic with unnecessary ChargeBattery tasks, we should
      // revist making this a recursive call.
      auto model = request->description()->make_model(
        state.time().value(),
        config.parameters());
      auto estimate = model->estimate_finish(
        state, config.constraints(), *travel_estimator);
      if (estimate.has_value())
      {
        agent.push_back(
          Assignment
          {
            request,
            estimate.value().finish_state(),
            estimate.value().wait_until()
          });
      }
      else
      {
        // Insufficient battery to perform the finishing request. We check if
        // adding a ChargeBattery task before will allow for it to be performed
        const auto charging_request =
          make_charging_request(state.time().value());
        const auto charge_battery_model =
          charging_request->description()->make_model(
          state.time().value(),
          config.parameters());
        const auto charge_battery_estimate =
          charge_battery_model->estimate_finish(
          state, config.constraints(), *travel_estimator);
        if (charge_battery_estimate.has_value())
        {
          model = request->description()->make_model(
            charge_battery_estimate.value().finish_state().time().value(),
            config.parameters());
          estimate = model->estimate_finish(
            charge_battery_estimate.value().finish_state(),
            config.constraints(),
            *travel_estimator);
          if (estimate.has_value())
          {
            // Append the ChargeBattery and finishing request
            agent.push_back(
              Assignment{
                charging_request,
                charge_battery_estimate.value().finish_state(),
                charge_battery_estimate.value().wait_until()
              });
            agent.push_back(
              Assignment
              {
                request,
                estimate.value().finish_state(),
                estimate.value().wait_until()
              });
          }
        }
      }
    }
  }

  Result complete_solve(
    rmf_traffic::Time time_now,
    std::vector<State>& initial_states,
    const std::vector<ConstRequestPtr>& requests,
    const std::function<bool()> interrupter,
    ConstRequestFactoryPtr finishing_request,
    bool greedy)
  {

    cost_calculator = config.cost_calculator() ? config.cost_calculator() :
      rmf_task::BinaryPriorityScheme::make_cost_calculator();

    // Also check if a high priority task exists among the requests.
    // If so the cost function for a node will be modified accordingly.
    for (const auto& request : requests)
    {
      if (request->booking()->priority())
      {
        check_priority = true;
        break;
      }
    }

    TaskPlannerError error;
    auto node = make_initial_node(
      initial_states, requests, time_now, error);
    if (!node)
      return error;

    TaskPlanner::Assignments complete_assignments;
    complete_assignments.resize(node->assigned_tasks.size());

    while (node)
    {
      if (greedy)
        node = greedy_solve(node, initial_states, time_now);
      else
        node = solve(node, initial_states,
            requests.size(), time_now, interrupter);

      if (!node)
        return {};

      // Here we prune assignments to remove any charging tasks at the back of
      // the assignment list
      node = prune_assignments(node);
      assert(complete_assignments.size() == node->assigned_tasks.size());
      for (std::size_t i = 0; i < complete_assignments.size(); ++i)
      {
        auto& all_assignments = complete_assignments[i];
        const auto& new_assignments = node->assigned_tasks[i];
        for (const auto& a : new_assignments)
        {
          all_assignments.push_back(a.assignment);
        }
      }

      if (node->unassigned_tasks.empty())
      {
        auto pruned_assignments = prune_assignments(complete_assignments);
        if (finishing_request != nullptr)
        {
          append_finishing_request(*finishing_request, pruned_assignments);
        }

        return pruned_assignments;
      }

      std::vector<ConstRequestPtr> new_tasks;
      for (const auto& u : node->unassigned_tasks)
        new_tasks.push_back(u.second.request);

      // copy final state estimates
      std::vector<State> estimates;
      rmf_traffic::agv::Plan::Start empty_new_location{
        time_now, 0, 0.0};
      estimates.resize(
        node->assigned_tasks.size(),
        State().load_basic(empty_new_location, 0, 0.0));
      for (std::size_t i = 0; i < node->assigned_tasks.size(); ++i)
      {
        const auto& assignments = node->assigned_tasks[i];
        if (assignments.empty())
          estimates[i] = initial_states[i];
        else
          estimates[i] = assignments.back().assignment.finish_state();
      }

      node = make_initial_node(
        estimates, new_tasks, time_now, error);
      if (!node)
        return error;
      initial_states = estimates;
    }

    // If a finishing_request is present, accommodate the request at the end of
    // the assignments for each agent
    if (finishing_request != nullptr)
    {
      append_finishing_request(*finishing_request, complete_assignments);
    }

    return complete_assignments;
  }

  ConstNodePtr make_initial_node(
    std::vector<State> initial_states,
    std::vector<ConstRequestPtr> requests,
    rmf_traffic::Time time_now,
    TaskPlannerError& error)
  {
    auto initial_node = std::make_shared<Node>();

    initial_node->assigned_tasks.resize(initial_states.size());

    for (const auto& request : requests)
    {
      // Generate a unique internal id for the request. Currently, multiple
      // requests with the same string id will be assigned different internal ids
      std::size_t internal_id = initial_node->get_available_internal_id();
      const auto pending_task = PendingTask::make(
        time_now,
        initial_states,
        config.constraints(),
        config.parameters(),
        request,
        *travel_estimator,
        error);

      if (!pending_task)
        return nullptr;

      initial_node->unassigned_tasks.insert(
        {
          internal_id,
          *pending_task
        });
    }

    initial_node->cost_estimate = cost_calculator->compute_cost(
      *initial_node, time_now, check_priority);

    initial_node->sort_invariants();

    initial_node->latest_time = [&]() -> rmf_traffic::Time
      {
        rmf_traffic::Time latest = rmf_traffic::Time::min();
        for (const auto& s : initial_states)
        {
          if (latest < s.time().value())
            latest = s.time().value();
        }

        return latest;
      } ();

    rmf_traffic::Time wait_until = rmf_traffic::Time::max();

    for (const auto& u : initial_node->unassigned_tasks)
    {
      const auto& range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it != range.end; ++it)
      {
        if (it->second.wait_until < wait_until)
          wait_until = it->second.wait_until;
      }
    }

    if (initial_node->latest_time < wait_until)
      initial_node->latest_time = wait_until;

    return initial_node;
  }

  rmf_traffic::Time get_latest_time(const Node& node)
  {
    rmf_traffic::Time latest = rmf_traffic::Time::min();
    for (const auto& a : node.assigned_tasks)
    {
      if (a.empty())
        continue;

      const auto finish_time =
        a.back().assignment.finish_state().time().value();

      if (latest < finish_time)
        latest = finish_time;
    }

    assert(latest > rmf_traffic::Time::min());
    return latest;
  }

  ConstNodePtr expand_candidate(
    const Candidates::Map::const_iterator& it,
    const Node::UnassignedTasks::value_type& u,
    const ConstNodePtr& parent,
    Filter* filter,
    rmf_traffic::Time time_now)
  {
    const auto& entry = it->second;
    const auto& constraints = config.constraints();

    if (parent->latest_time + segmentation_threshold < entry.wait_until)
    {

      // No need to assign task as timeline is not relevant
      return nullptr;
    }

    auto new_node = std::make_shared<Node>(*parent);

    // Assign the unassigned task after checking for implicit charging requests
    if (entry.require_charge_battery)
    {
      // Check if a battery task already precedes the latest assignment
      auto& assignments = new_node->assigned_tasks[entry.candidate];
      if (assignments.empty() || !std::dynamic_pointer_cast<
          const rmf_task::requests::ChargeBattery::Description>(
          assignments.back().assignment.request()->description()))
      {
        auto charge_battery = make_charging_request(
          entry.previous_state.time().value());
        const auto charge_battery_model =
          charge_battery->description()->make_model(
          charge_battery->booking()->earliest_start_time(),
          config.parameters());
        auto battery_estimate = charge_battery_model->estimate_finish(
          entry.previous_state, constraints, *travel_estimator);
        if (battery_estimate.has_value())
        {
          assignments.push_back(
            Node::AssignmentWrapper
            { u.first,
              Assignment
              {
                charge_battery,
                battery_estimate.value().finish_state(),
                battery_estimate.value().wait_until()
              }
            }
          );
        }
      }
    }
    new_node->assigned_tasks[entry.candidate].push_back(
      Node::AssignmentWrapper{u.first,
        Assignment{u.second.request, entry.state, entry.wait_until}});

    // Erase the assigned task from unassigned tasks
    new_node->pop_unassigned(u.first);

    // Update states of unassigned tasks for the candidate
    bool add_charger = false;
    for (auto& new_u : new_node->unassigned_tasks)
    {
      const auto finish =
        new_u.second.model->estimate_finish(
        entry.state, constraints, *travel_estimator);

      if (finish.has_value())
      {
        new_u.second.candidates.update_candidate(
          entry.candidate,
          finish.value().finish_state(),
          finish.value().wait_until(),
          entry.state,
          false);
      }
      else
      {
        // TODO(YV): Revisit this strategy
        // auto battery_estimate =
        //   config->charge_battery_request()->estimate_finish(entry.state, constraints);
        // if (battery_estimate.has_value())
        // {
        //   auto new_finish =
        //     new_u.second.request->estimate_finish(
        //       battery_estimate.value().finish_state(),
        //       constraints);
        //   assert(new_finish.has_value());
        //   new_u.second.candidates.update_candidate(
        //     entry.candidate,
        //     new_finish.value().finish_state(),
        //     new_finish.value().wait_until());
        // }
        // else
        // {
        //   // Unable to reach charger
        //   return nullptr;
        // }

        add_charger = true;
        break;
      }
    }

    if (add_charger)
    {
      auto charge_battery = make_charging_request(
        entry.state.time().value());

      const auto charge_battery_model =
        charge_battery->description()->make_model(
        charge_battery->booking()->earliest_start_time(),
        config.parameters());
      auto battery_estimate = charge_battery_model->estimate_finish(
        entry.state, constraints, *travel_estimator);
      if (battery_estimate.has_value())
      {
        new_node->assigned_tasks[entry.candidate].push_back(
          { new_node->get_available_internal_id(true),
            Assignment
            {
              charge_battery,
              battery_estimate.value().finish_state(),
              battery_estimate.value().wait_until()
            }});
        for (auto& new_u : new_node->unassigned_tasks)
        {
          const auto finish =
            new_u.second.model->estimate_finish(
            battery_estimate.value().finish_state(),
            constraints, *travel_estimator);
          if (finish.has_value())
          {
            new_u.second.candidates.update_candidate(
              entry.candidate, finish.value().finish_state(),
              finish.value().wait_until(), entry.state, false);
          }
          else
          {
            // We should stop expanding this node
            return nullptr;
          }
        }

      }
      else
      {
        // Agent cannot make it back to the charger
        return nullptr;
      }
    }

    // Update the cost estimate for new_node
    new_node->cost_estimate = cost_calculator->compute_cost(
      *new_node, time_now, check_priority);
    new_node->latest_time = get_latest_time(*new_node);

    // Apply filter
    if (filter && filter->ignore(*new_node))
    {
      return nullptr;
    }

    return new_node;

  }

  ConstNodePtr expand_charger(
    ConstNodePtr parent,
    const std::size_t agent,
    const std::vector<State>& initial_states,
    rmf_traffic::Time time_now)
  {
    auto new_node = std::make_shared<Node>(*parent);
    // Assign charging task to an agent
    State state = initial_states[agent];
    auto& assignments = new_node->assigned_tasks[agent];

    // If the assignment set for a candidate is empty we do not want to add a
    // charging task as this is taken care of in expand_candidate(). Without this
    // step there is chance for the planner to get stuck in an infinite loop when
    // a charging task is required before any other task can be assigned.
    if (assignments.empty())
      return nullptr;

    if (!assignments.empty())
    {
      if (std::dynamic_pointer_cast<
          const rmf_task::requests::ChargeBattery::Description>(
          assignments.back().assignment.request()->description()))
        return nullptr;
      state = assignments.back().assignment.finish_state();
    }

    auto charge_battery = make_charging_request(state.time().value());
    const auto charge_battery_model =
      charge_battery->description()->make_model(
      charge_battery->booking()->earliest_start_time(),
      config.parameters());
    auto estimate = charge_battery_model->estimate_finish(
      state, config.constraints(), *travel_estimator);
    if (estimate.has_value())
    {
      new_node->assigned_tasks[agent].push_back(
        Node::AssignmentWrapper
        {
          new_node->get_available_internal_id(true),
          Assignment
          {
            charge_battery,
            estimate.value().finish_state(),
            estimate.value().wait_until()
          }
        });
      for (auto& new_u : new_node->unassigned_tasks)
      {
        const auto finish =
          new_u.second.model->estimate_finish(
          estimate.value().finish_state(),
          config.constraints(), *travel_estimator);
        if (finish.has_value())
        {
          new_u.second.candidates.update_candidate(
            agent,
            finish.value().finish_state(),
            finish.value().wait_until(),
            state,
            false);
        }
        else
        {
          return nullptr;
        }
      }

      new_node->cost_estimate = cost_calculator->compute_cost(
        *new_node, time_now, check_priority);
      new_node->latest_time = get_latest_time(*new_node);
      return new_node;
    }

    return nullptr;
  }

  ConstNodePtr greedy_solve(
    ConstNodePtr node,
    const std::vector<State>& initial_states,
    rmf_traffic::Time time_now)
  {
    while (!finished(*node))
    {
      ConstNodePtr next_node = nullptr;
      for (const auto& u : node->unassigned_tasks)
      {
        const auto& range = u.second.candidates.best_candidates();
        for (auto it = range.begin; it != range.end; ++it)
        {
          if (auto n = expand_candidate(
              it, u, node, nullptr, time_now))
          {
            if (!next_node || (n->cost_estimate < next_node->cost_estimate))
            {
              next_node = std::move(n);
            }
          }
          else
          {
            // expand_candidate returned nullptr either due to start time
            // segmentation or insufficient charge to return to its charger.
            // For the later case, we aim to backtrack and assign a charging
            // task to the agent.
            if (node->latest_time + segmentation_threshold >
              it->second.wait_until)
            {
              auto parent_node = std::make_shared<Node>(*node);
              while (!parent_node->assigned_tasks[it->second.candidate].empty())
              {
                parent_node->assigned_tasks[it->second.candidate].pop_back();
                auto new_charge_node = expand_charger(
                  parent_node,
                  it->second.candidate,
                  initial_states,
                  time_now);
                if (new_charge_node)
                {
                  next_node = std::move(new_charge_node);
                  break;
                }
              }
            }
          }
        }
      }

      node = next_node;
      assert(node);
    }

    return node;
  }

  std::vector<ConstNodePtr> expand(
    ConstNodePtr parent,
    Filter& filter,
    const std::vector<State>& initial_states,
    rmf_traffic::Time time_now)
  {
    std::vector<ConstNodePtr> new_nodes;
    new_nodes.reserve(
      parent->unassigned_tasks.size() + parent->assigned_tasks.size());
    for (const auto& u : parent->unassigned_tasks)
    {
      const auto& range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it != range.end; it++)
      {
        if (auto new_node = expand_candidate(
            it, u, parent, &filter, time_now))
          new_nodes.push_back(std::move(new_node));
      }
    }

    // Assign charging task to each robot
    for (std::size_t i = 0; i < parent->assigned_tasks.size(); ++i)
    {
      if (auto new_node = expand_charger(
          parent, i, initial_states, time_now))
        new_nodes.push_back(std::move(new_node));
    }

    return new_nodes;
  }

  bool finished(const Node& node)
  {
    for (const auto& u : node.unassigned_tasks)
    {
      const auto range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it != range.end; ++it)
      {
        const auto wait_time = it->second.wait_until;
        if (wait_time <= node.latest_time + segmentation_threshold)
          return false;
      }
    }

    return true;
  }

  ConstNodePtr solve(
    ConstNodePtr initial_node,
    const std::vector<State>& initial_states,
    const std::size_t num_tasks,
    rmf_traffic::Time time_now,
    std::function<bool()> interrupter)
  {
    using PriorityQueue = std::priority_queue<
      ConstNodePtr,
      std::vector<ConstNodePtr>,
      LowestCostEstimate>;

    PriorityQueue priority_queue;
    priority_queue.push(std::move(initial_node));

    Filter filter{FilterType::Hash, num_tasks};
    ConstNodePtr top = nullptr;

    while (!priority_queue.empty() && !(interrupter && interrupter()))
    {
      top = priority_queue.top();

      // Pop the top of the priority queue
      priority_queue.pop();

      // Check if unassigned tasks is empty -> solution found
      if (finished(*top))
      {
        return top;
      }

      // Apply possible actions to expand the node
      const auto new_nodes = expand(
        top, filter, initial_states, time_now);

      // Add copies and with a newly assigned task to queue
      for (const auto& n : new_nodes)
        priority_queue.push(n);
    }

    return nullptr;
  }

};

// ============================================================================
TaskPlanner::TaskPlanner(
  Configuration configuration,
  Options default_options)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        configuration,
        default_options,
        std::make_shared<TravelEstimator>(configuration.parameters())
      }))
{
  // Do nothing
}

// ============================================================================
auto TaskPlanner::plan(
  rmf_traffic::Time time_now,
  std::vector<State> agents,
  std::vector<ConstRequestPtr> requests) -> Result
{
  return _pimpl->complete_solve(
    time_now,
    agents,
    requests,
    _pimpl->default_options.interrupter(),
    _pimpl->default_options.finishing_request(),
    _pimpl->default_options.greedy());
}

// ============================================================================
auto TaskPlanner::plan(
  rmf_traffic::Time time_now,
  std::vector<State> agents,
  std::vector<ConstRequestPtr> requests,
  Options options) -> Result
{
  return _pimpl->complete_solve(
    time_now,
    agents,
    requests,
    options.interrupter(),
    options.finishing_request(),
    options.greedy());
}

// ============================================================================
auto TaskPlanner::compute_cost(const Assignments& assignments) const -> double
{
  if (_pimpl->config.cost_calculator())
    return _pimpl->config.cost_calculator()->compute_cost(assignments);

  const auto cost_calculator =
    rmf_task::BinaryPriorityScheme::make_cost_calculator();
  return cost_calculator->compute_cost(assignments);
}

// ============================================================================
const rmf_task::TaskPlanner::Configuration& TaskPlanner::configuration()
const
{
  return _pimpl->config;
}

// ============================================================================
auto TaskPlanner::default_options() const -> const Options&
{
  return _pimpl->default_options;
}

// ============================================================================
auto TaskPlanner::default_options() -> Options&
{
  return _pimpl->default_options;
}

} // namespace rmf_task
