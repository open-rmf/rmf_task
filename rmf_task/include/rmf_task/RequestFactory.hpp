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

#ifndef RMF_TASK__REQUESTFACTORY_HPP
#define RMF_TASK__REQUESTFACTORY_HPP

#include <rmf_task/Request.hpp>

namespace rmf_task {

//==============================================================================
/// An abstract interface for generating a tailored request for an AGV given
class RequestFactory
{
public:

  /// Generate a request for the AGV given the state that the robot will have
  /// when it is ready to perform the request
  virtual ConstRequestPtr make_request(
    const State& state) const = 0;

  virtual ~RequestFactory() = default;
};

using RequestFactoryPtr = std::shared_ptr<RequestFactory>;
using ConstRequestFactoryPtr = std::shared_ptr<const RequestFactory>;

} // namespace rmf_task

#endif // RMF_TASK__REQUESTFACTORY_HPP
