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
#include <rmf_task/CostCalculator.hpp>
#include <rmf_task/agv/Constraints.hpp>
#include <rmf_task/agv/Parameters.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <vector>
#include <memory>
#include <functional>
#include <variant>

namespace rmf_task {
namespace agv {

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
      State state,
      rmf_traffic::Time deployment_time);

    // Get the request of this task
    const rmf_task::ConstRequestPtr& request() const;

    // Get a const reference to the predicted state at the end of the assignment
    const State& state() const;

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
  /// The configuration for the planner
  TaskPlanner(const Configuration& configuration);

  /// Get the configuration of this task planner
  const Configuration& configuration() const;

  /// Get the greedy planner based assignments for a set of initial states and
  /// requests
  Result greedy_plan(
    rmf_traffic::Time time_now,
    std::vector<State> agents,
    std::vector<ConstRequestPtr> requests);

  /// Get the optimal planner based assignments for a set of initial states and
  /// requests
  /// \note When the number of requests exceed 10 for the same start time
  /// segment, this plan may take a while to be generated. Hence, it is
  /// recommended to call greedy_plan() method and use the greedy solution for bidding.
  /// If a bid is awarded, the optimal solution may be used for assignments.
  Result optimal_plan(
    rmf_traffic::Time time_now,
    std::vector<State> agents,
    std::vector<ConstRequestPtr> requests,
    std::function<bool()> interrupter);

  /// Compute the cost of a set of assignments
  double compute_cost(const Assignments& assignments) const;

  /// Retrieve the task planner cache
  const std::shared_ptr<EstimateCache>& estimate_cache() const;

  class Implementation;

private:
  rmf_utils::impl_ptr<Implementation> _pimpl;

};

} // namespace agv
} // namespace rmf_task

#endif // RMF_TASK__AGV__TASKPLANNER_HPP
