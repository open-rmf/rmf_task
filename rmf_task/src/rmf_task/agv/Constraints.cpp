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

#include <stdexcept>

#include <rmf_task/Constraints.hpp>

namespace rmf_task {

//==============================================================================
class Constraints::Implementation
{
public:
  double threshold_soc;
  double recharge_soc;
  bool drain_battery;
};

//==============================================================================
Constraints::Constraints(
  double threshold_soc,
  double recharge_soc,
  bool drain_battery)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation
      {
        threshold_soc,
        recharge_soc,
        drain_battery
      }))
{
  if (threshold_soc < 0.0 || threshold_soc > 1.0)
  {
    // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
    throw std::invalid_argument(
      "Battery State of Charge threshold needs to be between 0.0 and 1.0.");
    // *INDENT-ON*
  }

  if (recharge_soc < 0.0 || recharge_soc > 1.0)
  {
    // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
    throw std::invalid_argument(
      "Recharge State of Charge needs to be between 0.0 and 1.0.");
    // *INDENT-ON*
  }
}

//==============================================================================
double Constraints::threshold_soc() const
{
  return _pimpl->threshold_soc;
}

//==============================================================================
auto Constraints::threshold_soc(double threshold_soc) -> Constraints&
{
  if (threshold_soc < 0.0 || threshold_soc > 1.0)
  {
    // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
    throw std::invalid_argument(
      "Battery State of Charge threshold needs to be between 0.0 and 1.0.");
    // *INDENT-ON*
  }

  _pimpl->threshold_soc = threshold_soc;
  return *this;
}

//==============================================================================
double Constraints::recharge_soc() const
{
  return _pimpl->recharge_soc;
}

//==============================================================================
auto Constraints::recharge_soc(double recharge_soc) -> Constraints&
{
  if (recharge_soc < 0.0 || recharge_soc > 1.0)
  {
    // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
    throw std::invalid_argument(
      "Recharge State of Charge needs to be between 0.0 and 1.0.");
    // *INDENT-ON*
  }

  _pimpl->recharge_soc = recharge_soc;
  return *this;
}

//==============================================================================
bool Constraints::drain_battery() const
{
  return _pimpl->drain_battery;
}

//==============================================================================
auto Constraints::drain_battery(
  bool drain_battery) -> Constraints&
{
  _pimpl->drain_battery = drain_battery;
  return *this;
}

} // namespace rmf_task
