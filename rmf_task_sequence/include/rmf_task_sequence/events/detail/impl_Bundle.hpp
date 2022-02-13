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

#ifndef RMF_TASK_SEQUENCE__EVENTS__DETAIL__IMPL_BUNDLE_HPP
#define RMF_TASK_SEQUENCE__EVENTS__DETAIL__IMPL_BUNDLE_HPP

#include <rmf_task_sequence/events/Bundle.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
template<typename OtherDesc>
void Bundle::unfold(
  const UnfoldDescription<OtherDesc>& unfold_description,
  Event::Initializer& add_to,
  const Event::ConstInitializerPtr& initialize_from)
{
  add_to.add<OtherDesc>(
    [initialize_from, unfold_description](
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const ConstParametersPtr& parameters,
      const OtherDesc& description,
      std::function<void()> update)
    {
      return initiate(
        *initialize_from,
        id,
        get_state,
        parameters,
        unfold_description(description),
        std::move(update));
    },
    [initialize_from, unfold_description](
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const ConstParametersPtr& parameters,
      const OtherDesc& description,
      const nlohmann::json& backup_state,
      std::function<void()> update,
      std::function<void()> checkpoint,
      std::function<void()> finished)
    {
      return restore(
        *initialize_from,
        id,
        get_state,
        parameters,
        unfold_description(description),
        backup_state,
        std::move(update),
        std::move(checkpoint),
        std::move(finished));
    });
}


} // namespace events
} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__EVENTS__DETAIL__IMPL_BUNDLE_HPP
