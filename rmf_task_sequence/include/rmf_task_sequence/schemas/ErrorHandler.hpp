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

#ifndef RMF_TASK_SEQUENCE__SCHEMAS__ERRORHANDLER_HPP
#define RMF_TASK_SEQUENCE__SCHEMAS__ERRORHANDLER_HPP

#include <nlohmann/json-schema.hpp>

#include <optional>

namespace rmf_task_sequence {
namespace schemas {

//==============================================================================
class ErrorHandler : public nlohmann::json_schema::error_handler
{
public:

  void error(
    const nlohmann::json::json_pointer& ptr,
    const nlohmann::json& instance,
    const std::string& message) final;

  struct Info
  {
    nlohmann::json::json_pointer ptr;
    nlohmann::json instance;
    std::string message;
  };

  std::optional<Info> failure;

  static std::optional<Info> has_error(
    const nlohmann::json_schema::json_validator& validator,
    const nlohmann::json json);
};

} // namespace schemas
} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__SCHEMAS__ERRORHANDLER_HPP
