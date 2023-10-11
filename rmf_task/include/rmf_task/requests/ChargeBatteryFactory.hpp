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

#ifndef RMF_TASK__REQUESTS__FACTORY__CHARGEBATTERYFACTORY_HPP
#define RMF_TASK__REQUESTS__FACTORY__CHARGEBATTERYFACTORY_HPP

#include <functional>
#include <optional>
#include <string>

#include <rmf_task/RequestFactory.hpp>
#include <rmf_task/State.hpp>
#include <rmf_traffic/Time.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
/// The ChargeBatteryFactory will generate a ChargeBattery request which
/// requires an AGV to return to its desginated charging_waypoint as specified
/// in its agv::State and wait till its battery charges up to the recharge_soc
/// configured in agv::Constraints recharge_soc specified in its
/// agv::Constraints
class ChargeBatteryFactory : public RequestFactory
{
public:

  /// Constructor
  ChargeBatteryFactory();

  /// Constructor
  ///
  /// \param[in] requester
  ///   The identifier of the entity that owns this RequestFactory, that will be
  ///   the designated requester of each new request.
  ///
  /// \param[in] time_now_cb
  ///   Callback function that returns the current time.
  explicit ChargeBatteryFactory(
    const std::string& requester,
    std::function<rmf_traffic::Time()> time_now_cb);

  /// Set the charging task to run indefinitely. This means it will never
  /// declare itself as finished and must instead be canceled. This can be used
  /// for idle tasks that are canceled automatically when a task request comes
  /// in. If indefinite is false, the robot will charge up to its designated
  /// recharge level.
  void set_indefinite(bool value);

  /// Does this factory produce charging requests that will run indefinitely?
  bool indefinite() const;

  /// Documentation inherited
  ConstRequestPtr make_request(const State& state) const final;

  class Implementation;

private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace requests
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__FACTORY__CHARGEBATTERYFACTORY_HPP
