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

#ifndef RMF_TASK__REQUESTS__LOOP_HPP
#define RMF_TASK__REQUESTS__LOOP_HPP

#include <chrono>
#include <optional>
#include <string>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_battery/MotionPowerSink.hpp>
#include <rmf_battery/DevicePowerSink.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <rmf_task/agv/State.hpp>
#include <rmf_task/Request.hpp>
#include <rmf_task/Estimate.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
class LoopDescription : public rmf_task::Request::Description
{
public:

  using Start = rmf_traffic::agv::Planner::Start;

  static DescriptionPtr make(
    std::size_t start_waypoint,
    std::size_t finish_waypoint,
    std::size_t num_loops,
    rmf_battery::ConstMotionPowerSinkPtr motion_sink,
    rmf_battery::ConstDevicePowerSinkPtr ambient_sink,
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    rmf_traffic::Time start_time);

  std::optional<rmf_task::Estimate> estimate_finish(
    const agv::State& initial_state,
    const agv::Constraints& task_planning_constraints,
    const std::shared_ptr<EstimateCache> estimate_cache,
    bool drain_battery) const final;

  rmf_traffic::Duration invariant_duration() const final;

  /// Get the start waypoint of the loop in this request
  std::size_t start_waypoint() const;

  /// Get the finish waypoint of the loop in this request
  std::size_t finish_waypoint() const;

  /// Get the numbert of loops in this request
  std::size_t num_loops() const;

  /// Get the Start when the robot reaches the start_waypoint from an initial
  /// start
  Start loop_start(const Start& start) const;

  /// Get the Start when the robot reaches the finish_waypoint from an initial
  /// start
  Start loop_end(const Start& start) const;

  class Implementation;
private:
  LoopDescription();

  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Loop
{
public:
  static ConstRequestPtr make(
    const std::string& id,
    std::size_t start_waypoint,
    std::size_t finish_waypoint,
    std::size_t num_loops,
    rmf_battery::ConstMotionPowerSinkPtr motion_sink,
    rmf_battery::ConstDevicePowerSinkPtr ambient_sink,
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    rmf_traffic::Time start_time,
    ConstPriorityPtr priority = nullptr);
};

} // namespace tasks
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__LOOP_HPP
