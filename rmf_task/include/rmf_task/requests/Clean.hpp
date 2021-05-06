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

#ifndef RMF_TASK__REQUESTS__CLEAN_HPP
#define RMF_TASK__REQUESTS__CLEAN_HPP

#include <chrono>
#include <string>
#include <optional>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_battery/MotionPowerSink.hpp>
#include <rmf_battery/DevicePowerSink.hpp>

#include <rmf_task/agv/State.hpp>
#include <rmf_task/Request.hpp>
#include <rmf_task/Estimate.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
class Clean
{
public:

  class Model;

  class Description : public Request::Description
  {
  public:

    static DescriptionPtr make(
      std::size_t start_waypoint,
      std::size_t end_waypoint,
      const rmf_traffic::Trajectory& cleaning_path);

    std::shared_ptr<Request::Model> make_model(
      rmf_traffic::Time earliest_start_time,
      const agv::Parameters& parameters) const final;

    /// Get the start waypoint in this request
    std::size_t start_waypoint() const;

    /// Get the end waypoint in this request
    std::size_t end_waypoint() const;

    class Implementation;
  private:
    Description();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  static ConstRequestPtr make(
    std::size_t start_waypoint,
    std::size_t end_waypoint,
    const rmf_traffic::Trajectory& cleaning_path,
    const std::string& id,
    rmf_traffic::Time earliest_start_time,
    ConstPriorityPtr priority = nullptr);
};

} // namespace requests
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__CLEAN_HPP
