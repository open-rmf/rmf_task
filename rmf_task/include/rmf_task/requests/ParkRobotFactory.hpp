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

#ifndef RMF_TASK__REQUESTS__FACTORY__PARKROBOTFACTORY_HPP
#define RMF_TASK__REQUESTS__FACTORY__PARKROBOTFACTORY_HPP

#include <rmf_task/RequestFactory.hpp>
#include <rmf_task/State.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <optional>

namespace rmf_task {
namespace requests {

//==============================================================================
/// The ParkRobotFactory will generate a request for the AGV to return to
/// its desginated parking spot and remain idle there. This factory may be used
/// when AGVs should not remain idle at the location of their last task but
/// rather wait for new orders at their designated parking spots.
class ParkRobotFactory : public RequestFactory
{
public:

  /// Constructor
  ///
  /// \param[in] parking_waypoint
  ///   The graph index of the waypoint assigned to this AGV for parking.
  ///   If nullopt, the AGV will return to its charging_waypoint and remain idle
  ///   there. It will not wait for its battery to charge up before undertaking
  ///   new tasks.
  ParkRobotFactory(
    std::optional<std::size_t> parking_waypoint = std::nullopt);

  /// Documentation inherited
  ConstRequestPtr make_request(
    const State& state) const final;

  class Implementation;

private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace requests
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__FACTORY__PARKROBOTFACTORY_HPP
