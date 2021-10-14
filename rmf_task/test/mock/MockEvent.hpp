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

#ifndef TEST__MOCK__MOCKEVENT_HPP
#define TEST__MOCK__MOCKEVENT_HPP

#include <rmf_task/Event.hpp>

namespace test_rmf_task {

//==============================================================================
class MockEvent : public rmf_task::Event
{
public:

  MockEvent(
    std::string name_,
    std::string detail_,
    Status initial_status = Status::Standby);

  // Interface
  Status status() const final;
  std::string name() const final;
  std::string detail() const final;
  rmf_task::Log::View log() const final;
  std::vector<rmf_task::ConstEventPtr> dependencies() const final;

  // Fields
  Status _status;
  std::string _name;
  std::string _detail;
  rmf_task::Log _log;
  std::vector<std::shared_ptr<MockEvent>> _dependencies;

};

} // namespace test_rmf_task

#endif // TEST__MOCK__MOCKCONDITION_HPP
