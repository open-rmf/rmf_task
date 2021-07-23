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

#include <rmf_task/RequestFactory.hpp>
#include <rmf_task/agv/State.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
class ChargeBatteryFactory : public RequestFactory
{
public:

  ChargeBatteryFactory();

  ConstRequestPtr make_request(
    const agv::State& state) const final;

  class Implementation;

private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace requests
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__FACTORY__CHARGEBATTERYFACTORY_HPP
