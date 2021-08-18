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

#ifndef RMF_TASK__REQUESTS__DELIVERY_HPP
#define RMF_TASK__REQUESTS__DELIVERY_HPP

#include <chrono>
#include <string>
#include <optional>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_battery/MotionPowerSink.hpp>
#include <rmf_battery/DevicePowerSink.hpp>

#include <rmf_task/agv/State.hpp>
#include <rmf_task/Request.hpp>
#include <rmf_task/Estimate.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
class Delivery
{
public:

  class Model;

  class Description : public Request::Description
  {
  public:

    using Start = rmf_traffic::agv::Planner::Start;

    static DescriptionPtr make(
      std::size_t pickup_waypoint,
      rmf_traffic::Duration pickup_duration,
      std::size_t dropoff_waypoint,
      rmf_traffic::Duration dropoff_duration);

    std::shared_ptr<Request::Model> make_model(
      rmf_traffic::Time earliest_start_time,
      const agv::Parameters& parameters) const final;

    /// Get the pickup waypoint in this request
    std::size_t pickup_waypoint() const;

    /// Get the duration over which delivery items are loaded
    rmf_traffic::Duration pickup_wait() const;

    /// Get the dropoff waypoint in this request
    std::size_t dropoff_waypoint() const;

    /// Get the duration over which delivery items are unloaded
    rmf_traffic::Duration dropoff_wait() const;

    class Implementation;
  private:
    Description();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };


  static ConstRequestPtr make(
    std::size_t pickup_waypoint,
    rmf_traffic::Duration pickup_wait,
    std::size_t dropoff_waypoint,
    rmf_traffic::Duration dropoff_wait,
    const std::string& id,
    rmf_traffic::Time earliest_start_time,
    ConstPriorityPtr priority = nullptr);
};

} // namespace requests
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__DELIVERY_HPP
