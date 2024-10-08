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

#ifndef RMF_TASK__PRIORITY_HPP
#define RMF_TASK__PRIORITY_HPP

#include <memory>
#include <nlohmann/json.hpp>

namespace rmf_task {

//==============================================================================
/// Abstract interface to specify the priority for a request
class Priority
{
public:

  /// Serialize the priority as a json
  virtual nlohmann::json serialize() const = 0;
};

using PriorityPtr = std::shared_ptr<Priority>;
using ConstPriorityPtr = std::shared_ptr<const Priority>;

} // namespace rmf_task
# endif // RMF_TASK__PRIORITY_HPP
