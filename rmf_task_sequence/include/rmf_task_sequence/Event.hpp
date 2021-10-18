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

#ifndef RMF_TASK_SEQUENCE__EVENT_HPP
#define RMF_TASK_SEQUENCE__EVENT_HPP

#include <rmf_task/Constraints.hpp>
#include <rmf_task/Parameters.hpp>
#include <rmf_task/Estimate.hpp>
#include <rmf_task/Event.hpp>

#include <rmf_task/detail/Resume.hpp>

#include <rmf_task_sequence/detail/Backup.hpp>
#include <rmf_task_sequence/typedefs.hpp>

namespace rmf_task_sequence {

//==============================================================================
class Event : public rmf_task::Event
{
public:

  class Active;
  using ActivePtr = std::shared_ptr<Active>;

  class Activator;
  using ActivatorPtr = std::shared_ptr<Activator>;
  using ConstActivatorPtr = std::shared_ptr<const Activator>;

  class Description;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;

  class Model;
  using ConstModelPtr = std::shared_ptr<const Model>;
};

//==============================================================================
class Event::Active : public rmf_task::Event::Active
{
public:

  using Backup = detail::Backup;

  virtual Backup backup() const = 0;

};

} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__EVENT_HPP
