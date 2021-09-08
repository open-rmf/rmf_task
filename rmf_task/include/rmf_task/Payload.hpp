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

#ifndef RMF_TASK__PAYLOAD_HPP
#define RMF_TASK__PAYLOAD_HPP

#include <string>
#include <vector>

namespace rmf_task {

//==============================================================================
class Payload
{
public:

  class Component;

  /// Components in the payload
  const std::vector<Component>& components() const;

private:

};

//==============================================================================
class Payload::Component
{
public:

  /// Stock Keeping Unit (SKU) for this component of the payload
  const std::string& sku() const;

  /// The quantity of the specified item in this component of the payload
  uint32_t quantity() const;

  /// The name of the compartment
  const std::string& compartment() const;
};

} // namespace rmf_task

#endif // RMF_TASK__PAYLOAD_HPP
