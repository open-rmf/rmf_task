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

#include <rmf_dispenser_msgs/msg/dispenser_request_item.hpp>

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

    using DispenserRequestItem = rmf_dispenser_msgs::msg::DispenserRequestItem;
    using Start = rmf_traffic::agv::Planner::Start;

    static DescriptionPtr make(
      std::size_t pickup_waypoint,
      std::string pickup_dispenser,
      std::size_t dropoff_waypoint,
      std::string dropoff_ingestor,
      std::vector<DispenserRequestItem> items);

    std::shared_ptr<Request::Model> make_model(
      rmf_traffic::Time earliest_start_time,
      const agv::Parameters& parameters) const final;

    /// Get the pickup waypoint in this request
    std::size_t pickup_waypoint() const;

    /// Get the name of the dispenser at the pickup waypoint
    const std::string& pickup_dispenser() const;

    /// Get the dropoff waypoint in this request
    std::size_t dropoff_waypoint() const;

    /// Get the name of the ingestor at the dropoff waypoint
    const std::string& dropoff_ingestor() const;

    /// Get the list of dispenser request items in this request
    const std::vector<DispenserRequestItem>&  items() const;

    class Implementation;
  private:
    Description();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  using DispenserRequestItem = rmf_dispenser_msgs::msg::DispenserRequestItem;

  static ConstRequestPtr make(
    std::size_t pickup_waypoint,
    std::string pickup_dispenser,
    std::size_t dropoff_waypoint,
    std::string dropoff_ingestor,
    std::vector<DispenserRequestItem> items,
    const std::string& id,
    rmf_traffic::Time earliest_start_time,
    ConstPriorityPtr priority = nullptr);
};

} // namespace requests
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__DELIVERY_HPP
