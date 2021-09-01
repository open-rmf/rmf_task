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

#ifndef RMF_TASK__AGV__CONSTRAINTS_HPP
#define RMF_TASK__AGV__CONSTRAINTS_HPP

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_task {

//==============================================================================
/// A class that describes constraints that are common among the agents/AGVs
/// available for performing requests
class Constraints
{
public:

  /// Constructor
  ///
  /// \param[in] threshold_soc
  ///   Minimum charge level the vehicle is allowed to deplete to. This
  ///   value needs to be between 0.0 and 1.0.
  ///
  /// \param[in] recharge_soc
  ///   The charge level the vehicle should be recharged to. This
  ///   value needs to be between 0.0 and 1.0. Default value is 1.0.
  ///
  /// \param[in] drain_battery
  ///   If true, battery drain will be considered during task allocation and
  ///   ChargeBattery tasks will automatically be included if necessary.
  Constraints(
    double threshold_soc,
    double recharge_soc = 1.0,
    bool drain_battery = true);

  /// Gets the vehicle's state of charge threshold value.
  double threshold_soc() const;

  /// Sets the vehicle's state of charge threshold value. This value needs to be
  /// between 0.0 and 1.0.
  Constraints& threshold_soc(double threshold_soc);

  /// Gets the vehicle's state of charge recharge value.
  double recharge_soc() const;

  /// Sets the vehicle's recharge state of charge value. This value needs to be
  /// between 0.0 and 1.0.
  Constraints& recharge_soc(double recharge_soc);

  /// Get the value of drain_battery
  bool drain_battery() const;

  /// Set the value of drain_battery
  Constraints& drain_battery(bool drain_battery);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_task

#endif // RMF_TASK__AGV__CONSTRAINTS_HPP
