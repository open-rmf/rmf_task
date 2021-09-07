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

#ifndef RMF_TASK__AGV__TASKPLANNER_HPP
#define RMF_TASK__AGV__TASKPLANNER_HPP

#include <rmf_task/Request.hpp>
#include <rmf_task/RequestFactory.hpp>
#include <rmf_task/CostCalculator.hpp>
#include <rmf_task/Constraints.hpp>
#include <rmf_task/Parameters.hpp>
#include <rmf_task/State.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <vector>
#include <memory>
#include <functional>
#include <variant>

namespace rmf_task {

//==============================================================================
class TaskPlanner
{
public:

  /// The Configuration class contains planning parameters that are immutable
  /// for each TaskPlanner instance and should not change in between plans.
  class Configuration
  {
  public:
    /// Constructor
    ///
    /// \param[in] parameters
    ///   The parameters that describe the agents
    ///
    /// \param[in] constraints
    ///   The constraints that apply to the agents
    ///
    /// \param[in] cost_calculator
    ///   An object that tells the planner how to calculate cost
    Configuration(
      Parameters parameters,
      Constraints constraints,
      ConstCostCalculatorPtr cost_calculator);

    /// Get the parameters that describe the agents
    const Parameters& parameters() const;

    /// Set the parameters that describe the agents
    Configuration& parameters(Parameters parameters);

    /// Get the constraints that are applicable to the agents
    const Constraints& constraints() const;

    /// Set the constraints that are applicable to the agents
    Configuration& constraints(Constraints constraints);

    /// Get the CostCalculator
    const ConstCostCalculatorPtr& cost_calculator() const;

    /// Set the CostCalculator. If a nullptr is passed, the
    /// BinaryPriorityCostCalculator is used by the planner.
    Configuration& cost_calculator(ConstCostCalculatorPtr cost_calculator);

    class Implementation;

  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// The Options class contains planning parameters that can change between
  /// each planning attempt.
  class Options
  {
  public:

    /// Constructor
    ///
    /// \param[in] greedy
    ///   If true, a greedy approach will be used to solve for the task
    ///   assignments. Optimality is not guaranteed but the solution time may be
    ///   faster. If false, an A* based approach will be used within the planner
    ///   which guarantees optimality but may take longer to solve.
    ///
    /// \param[in] interrupter
    ///   A function that can determine whether the planning should be interrupted.
    ///
    /// \param[in] finishing_request
    ///   A request factory that generates a tailored task for each agent/AGV
    ///   to perform at the end of their assignments
    Options(
      bool greedy,
      std::function<bool()> interrupter = nullptr,
      ConstRequestFactoryPtr finishing_request = nullptr);

    /// Set whether a greedy approach should be used
    Options& greedy(bool value);

    /// Get whether a greedy approach will be used
    bool greedy() const;

    /// Set an interrupter callback that will indicate to the planner if it
    /// should stop trying to plan
    Options& interrupter(std::function<bool()> interrupter);

    /// Get the interrupter that will be used in this Options
    const std::function<bool()>& interrupter() const;

    /// Set the request factory that will generate a finishing task
    Options& finishing_request(ConstRequestFactoryPtr finishing_request);

    /// Get the request factory that will generate a finishing task
    ConstRequestFactoryPtr finishing_request() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  class Assignment
  {
  public:

    /// Constructor
    ///
    /// \param[in] request
    ///   The task request for this assignment
    ///
    /// \param[in] state
    ///   The state of the agent at the end of the assigned task
    ///
    /// \param[in] earliest_start_time
    ///   The earliest time the agent will begin exececuting this task
    Assignment(
      rmf_task::ConstRequestPtr request,
      State finish_state,
      rmf_traffic::Time deployment_time);

    // Get the request of this task
    const rmf_task::ConstRequestPtr& request() const;

    // Get a const reference to the predicted state at the end of the assignment
    const State& finish_state() const;

    // Get the time when the robot begins executing
    // this assignment
    const rmf_traffic::Time deployment_time() const;

    class Implementation;

  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  enum class TaskPlannerError
  {
    /// None of the agents in the initial states have sufficient initial charge
    /// to even head back to their charging stations. Manual intervention is
    /// needed to recharge one or more agents.
    low_battery,

    /// None of the agents in the initial states have sufficient battery
    /// capacity to accommodate one or more requests. This may be remedied by
    /// increasing the battery capacity or by lowering the threshold_soc in the
    /// state configs of the agents or by modifying the original request.
    limited_capacity
  };

  /// Container for assignments for each agent
  using Assignments = std::vector<std::vector<Assignment>>;
  using Result = std::variant<Assignments, TaskPlannerError>;

  /// Constructor
  ///
  /// \param[in] configuration
  ///   The configuration for the planner
  ///
  /// \param[in] default_options
  ///   Default options for the task planner to use when solving for assignments.
  ///   These options can be overriden each time a plan is requested.
  TaskPlanner(
    Configuration configuration,
    Options default_options);

  /// Get a const reference to configuration of this task planner
  const Configuration& configuration() const;

  /// Get a const reference to the default planning options.
  const Options& default_options() const;

  /// Get a mutable reference to the default planning options.
  Options& default_options();

  /// Generate assignments for requests among available agents. The default
  /// Options of this TaskPlanner instance will be used.
  ///
  /// \param[in] time_now
  ///   The current time when this plan is requested
  ///
  /// \param[in] agents
  ///   The initial states of the agents/AGVs that can undertake the requests
  ///
  /// \param[in] requests
  ///   The set of requests that need to be assigned among the agents/AGVs
  Result plan(
    rmf_traffic::Time time_now,
    std::vector<State> agents,
    std::vector<ConstRequestPtr> requests);

  /// Generate assignments for requests among available agents. Override the
  /// default parameters
  ///
  /// \param[in] time_now
  ///   The current time when this plan is requested
  ///
  /// \param[in] agents
  ///   The initial states of the agents/AGVs that can undertake the requests
  ///
  /// \param[in] requests
  ///   The set of requests that need to be assigned among the agents/AGVs
  ///
  /// \param[in] options
  ///   The options to use for this plan. This overrides the default Options of
  ///   the TaskPlanner instance
  Result plan(
    rmf_traffic::Time time_now,
    std::vector<State> agents,
    std::vector<ConstRequestPtr> requests,
    Options options);

  /// Compute the cost of a set of assignments
  double compute_cost(const Assignments& assignments) const;

  class Implementation;

private:
  rmf_utils::impl_ptr<Implementation> _pimpl;

};

} // namespace rmf_task

#endif // RMF_TASK__AGV__TASKPLANNER_HPP
