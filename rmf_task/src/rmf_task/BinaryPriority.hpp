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

#ifndef SRC__RMF_TASK__BINARYPRIORITY_HPP
#define SRC__RMF_TASK__BINARYPRIORITY_HPP

#include <nlohmann/json.hpp>

#include <rmf_task/Priority.hpp>

namespace rmf_task {

// Sample implementation for binary prioritization scheme
class BinaryPriority : public Priority
{
public:

  /// Constructor
  BinaryPriority(std::size_t value);

  /// Get the value of this priority object
  std::size_t value() const;

  /// Serialize priority
  nlohmann::json serialize() const override;

private:
  std::size_t _value;
};

} // namespace rmf_task

#endif // SRC__RMF_TASK__BINARYPRIORITY_HPP
