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
void Event::Initializer::add_initializer(Initialize<Desc> initializer)
{
  _add_initializer(
    typeid(Desc),
    [initializer](
      std::function<rmf_task::State()> get_state,
      const ConstParametersPtr& parameters,
      const Event::Description& description,
      std::optional<std::string> backup_state)
  {
    return initializer(
      std::move(get_state),
      parameters,
      static_cast<const Desc&>(description),
      std::move(backup_state));
  });
}

} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__DETAIL__IMPL_EVENT_HPP
