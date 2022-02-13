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

#ifndef RMF_TASK__ESTIMATE_HPP
#define RMF_TASK__ESTIMATE_HPP

#include <optional>
#include <utility>

#include <rmf_task/State.hpp>
#include <rmf_task/Parameters.hpp>
#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_utils/impl_ptr.hpp>

namespace rmf_task {

//==============================================================================
/// A class to store the time that the AGV should wait till before executing the
/// request and the state of the AGV after finishing the request.
/// Note: The wait time is different from the earliest_start_time specified in
/// the request definition. The wait time may be earlier to ensure that the AGV
/// arrvies at the first location of the request by the earliest_start_time
class Estimate
{
public:

  /// Constructor of an estimate of the request.
  ///
  /// \param[in] finish_state
  ///   Finish state of the robot once it completes the request.
  ///
  /// \param[in] wait_until
  ///   The ideal time the robot starts executing this request.
  Estimate(State finish_state, rmf_traffic::Time wait_until);

  /// Finish state of the robot once it completes the request.
  State finish_state() const;

  /// Sets a new finish state for the robot.
  Estimate& finish_state(State new_finish_state);

  /// The ideal time the robot starts executing this request.
  rmf_traffic::Time wait_until() const;

  /// Sets a new starting time for the robot to execute the request.
  Estimate& wait_until(rmf_traffic::Time new_wait_until);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// A class to estimate the cost of travelling between any two points in a
/// navigation graph. Results will be memoized for efficiency.
class TravelEstimator
{
public:

  /// Constructor
  ///
  /// \param[in] parameters
  ///   The parameters for the robot
  TravelEstimator(const Parameters& parameters);

  /// The result of a travel estimation
  class Result
  {
  public:

    /// How long the travelling will take
    rmf_traffic::Duration duration() const;

    /// How much the battery will drain while travelling
    double change_in_charge() const;

    class Implementation;
  private:
    Result();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Estimate the cost of travelling
  std::optional<Result> estimate(
    const rmf_traffic::agv::Plan::Start& start,
    const rmf_traffic::agv::Plan::Goal& goal) const;

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
using ConstTravelEstimatorPtr = std::shared_ptr<const TravelEstimator>;

} // namespace rmf_task

#endif // RMF_TASK__ESTIMATE_HPP
