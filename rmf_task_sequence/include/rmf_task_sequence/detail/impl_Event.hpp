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

#ifndef RMF_TASK_SEQUENCE__DETAIL__IMPL_EVENT_HPP
#define RMF_TASK_SEQUENCE__DETAIL__IMPL_EVENT_HPP

#include <rmf_task_sequence/Event.hpp>

namespace rmf_task_sequence {

//==============================================================================
template<typename Desc>
void Event::Initializer::add(
  Initialize<Desc> initializer,
  Restore<Desc> restorer)
{
  _add(
    typeid(Desc),
    [initializer](
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const ConstParametersPtr& parameters,
      const Event::Description& description,
      std::function<void()> update)
    {
      return initializer(
        id,
        get_state,
        parameters,
        static_cast<const Desc&>(description),
        std::move(update));
    },
    [restorer](
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const ConstParametersPtr& parameters,
      const Event::Description& description,
      const nlohmann::json& backup,
      std::function<void()> update,
      std::function<void()> checkpoint,
      std::function<void()> finished)
    {
      return restorer(
        id,
        get_state,
        parameters,
        static_cast<const Desc&>(description),
        backup,
        std::move(update),
        std::move(checkpoint),
        std::move(finished));
    });
}

} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__DETAIL__IMPL_EVENT_HPP
