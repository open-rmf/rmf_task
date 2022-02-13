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

#ifndef RMF_TASK_SEQUENCE__TYPEDEFS_HPP
#define RMF_TASK_SEQUENCE__TYPEDEFS_HPP

#include <rmf_task/Header.hpp>
#include <rmf_task/State.hpp>
#include <rmf_task/Estimate.hpp>
#include <rmf_task/Parameters.hpp>
#include <rmf_task/Constraints.hpp>
#include <rmf_task/Estimate.hpp>
#include <rmf_task/Payload.hpp>

namespace rmf_task_sequence {

using Header = rmf_task::Header;
using State = rmf_task::State;
using Estimate = rmf_task::Estimate;
using Parameters = rmf_task::Parameters;
using ConstParametersPtr = rmf_task::ConstParametersPtr;
using Constraints = rmf_task::Constraints;
using TravelEstimator = rmf_task::TravelEstimator;
using Payload = rmf_task::Payload;

} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__TYPEDEFS_HPP
