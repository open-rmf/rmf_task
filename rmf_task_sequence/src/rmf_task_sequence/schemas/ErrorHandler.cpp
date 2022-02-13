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

#include <rmf_task_sequence/schemas/ErrorHandler.hpp>

namespace rmf_task_sequence {
namespace schemas {

//==============================================================================
void ErrorHandler::error(
  const nlohmann::json::json_pointer& ptr,
  const nlohmann::json& instance,
  const std::string& message)
{
  failure = Info{ptr, instance, message};
}

//==============================================================================
auto ErrorHandler::has_error(
  const nlohmann::json_schema::json_validator& validator,
  const nlohmann::json json) -> std::optional<Info>
{
  ErrorHandler handler;
  validator.validate(json, handler);
  return handler.failure;
}

} // namespace schemas
} // namespace rmf_task_sequence
