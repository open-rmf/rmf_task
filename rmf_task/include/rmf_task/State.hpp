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

#ifndef RMF_TASK__STATE_HPP
#define RMF_TASK__STATE_HPP

#include <chrono>

#include <rmf_utils/impl_ptr.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_task/CompositeData.hpp>

namespace rmf_task {

//==============================================================================
class State : public CompositeData
{
public:

  /// The current waypoint of the robot state
  RMF_TASK_DEFINE_COMPONENT(std::size_t, CurrentWaypoint);
  std::optional<std::size_t> waypoint() const;
  State& waypoint(std::size_t new_waypoint);

  /// The current orientation of the robot state
  RMF_TASK_DEFINE_COMPONENT(double, CurrentOrientation);
  std::optional<double> orientation() const;
  State& orientation(double new_orientation);

  /// The current time for the robot
  RMF_TASK_DEFINE_COMPONENT(rmf_traffic::Time, CurrentTime);
  std::optional<rmf_traffic::Time> time() const;
  State& time(rmf_traffic::Time new_time);

  /// The dedicated charging point for this robot
  // TODO(MXG): Consider removing this field and using some kind of
  // ChargingStrategy abstract interface to determine where and how the robots
  // should be charging.
  RMF_TASK_DEFINE_COMPONENT(std::size_t, DedicatedChargingPoint);
  std::optional<std::size_t> dedicated_charging_waypoint() const;
  State& dedicated_charging_waypoint(std::size_t new_charging_waypoint);

  /// The current battery state of charge of the robot. This value is between
  /// 0.0 and 1.0.
  RMF_TASK_DEFINE_COMPONENT(double, CurrentBatterySoC);
  std::optional<double> battery_soc() const;
  State& battery_soc(double new_battery_soc);

  /// Load the basic state components expected for the planner.
  ///
  /// \param[in] location
  ///   The robot's initial location data.
  ///
  /// \param[in] charging_point
  ///   The robot's dedicated charging point.
  ///
  /// \param[in] battery_soc
  ///   The robot's initial battery state of charge.
  State& load_basic(
    const rmf_traffic::agv::Plan::Start& location,
    std::size_t charging_point,
    double battery_soc);

  /// Load the plan start into the State. The location info will be split into
  /// CurrentWaypoint, CurrentOrientation, and CurrentTime data.
  State& load(const rmf_traffic::agv::Plan::Start& location);

  /// Project an rmf_traffic::agv::Plan::Start from this State.
  ///
  /// If CurrentWaypoint is unavailable, this will return a std::nullopt. For
  /// any other components that are unavailable (CurrentOrientation or
  /// CurrentTime), the given default values will be used.
  ///
  /// \param[in] default_orientation
  ///   The orientation value that will be used if CurrentOrientation is not
  ///   available in this State.
  ///
  /// \param[in] default_time
  ///   The time value that will be used if default_time is not available in
  ///   this State.
  std::optional<rmf_traffic::agv::Plan::Start> project_plan_start(
    double default_orientation = 0.0,
    rmf_traffic::Time default_time = rmf_traffic::Time()) const;

  /// Extract an rmf_traffic::agv::Plan::Start from this State.
  ///
  /// If any necessary component is missing (i.e. CurrentWaypoint,
  /// CurrentOrientation, or CurrentTime) then this will return a std::nullopt.
  std::optional<rmf_traffic::agv::Plan::Start> extract_plan_start() const;
};

} // namespace rmf_task

#endif // RMF_TASK__AGV__STATE_HPP
