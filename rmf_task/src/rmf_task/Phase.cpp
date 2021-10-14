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

#include <rmf_task/Phase.hpp>

namespace rmf_task {

//==============================================================================
class Phase::Tag::Implementation
{
public:

  Id id;
  std::string name;
  std::string detail;
  rmf_traffic::Duration duration;

};

//==============================================================================
Phase::Tag::Tag(
  Id id_,
  std::string name_,
  std::string detail_,
  rmf_traffic::Duration estimate_)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        id_,
        std::move(name_),
        std::move(detail_),
        estimate_
      }))
{
  // Do nothing
}

//==============================================================================
auto Phase::Tag::id() const -> Id
{
  return _pimpl->id;
}

//==============================================================================
const std::string& Phase::Tag::name() const
{
  return _pimpl->name;
}

//==============================================================================
const std::string& Phase::Tag::detail() const
{
  return _pimpl->detail;
}

//==============================================================================
rmf_traffic::Duration Phase::Tag::original_duration_estimate() const
{
  return _pimpl->duration;
}

//==============================================================================
class Phase::Completed::Implementation
{
public:

  ConstTagPtr tag;
  ConstSnapshotPtr snapshot;
  rmf_traffic::Time start;
  rmf_traffic::Time finish;
};

//==============================================================================
Phase::Completed::Completed(
  ConstTagPtr tag_,
  ConstSnapshotPtr snapshot_,
  rmf_traffic::Time start_,
  rmf_traffic::Time finish_)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(tag_),
        std::move(snapshot_),
        start_,
        finish_
      }))
{
  // Do nothing
}

//==============================================================================
auto Phase::Completed::tag() const -> const ConstTagPtr&
{
  return _pimpl->tag;
}

//==============================================================================
auto Phase::Completed::snapshot() const -> const ConstSnapshotPtr&
{
  return _pimpl->snapshot;
}

//==============================================================================
rmf_traffic::Time Phase::Completed::start_time() const
{
  return _pimpl->start;
}

//==============================================================================
rmf_traffic::Time Phase::Completed::finish_time() const
{
  return _pimpl->finish;
}

//==============================================================================
class Phase::Snapshot::Implementation
{
public:
  ConstTagPtr tag;
  ConstConditionPtr finish_condition;
  rmf_traffic::Time estimated_finish_time;
};

//==============================================================================
Phase::ConstSnapshotPtr Phase::Snapshot::make(const Active& active)
{
  Snapshot output;
  output._pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{
      active.tag(),
      Condition::Snapshot::make(*active.finish_condition()),
      active.estimate_finish_time()
    });

  return std::make_shared<Snapshot>(std::move(output));
}

//==============================================================================
Phase::ConstTagPtr Phase::Snapshot::tag() const
{
  return _pimpl->tag;
}

//==============================================================================
ConstConditionPtr Phase::Snapshot::finish_condition() const
{
  return _pimpl->finish_condition;
}

//==============================================================================
rmf_traffic::Time Phase::Snapshot::estimate_finish_time() const
{
  return _pimpl->estimated_finish_time;
}

//==============================================================================
Phase::Snapshot::Snapshot()
{
  // Do nothing
}

//==============================================================================
class Phase::Pending::Implementation
{
public:

  ConstTagPtr tag;
  bool will_be_skipped = false;

};

//==============================================================================
Phase::Pending::Pending(ConstTagPtr tag)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{std::move(tag)}))
{
  // Do nothing
}

//==============================================================================
auto Phase::Pending::tag() const -> const ConstTagPtr&
{
  return _pimpl->tag;
}

//==============================================================================
auto Phase::Pending::will_be_skipped(bool value) -> Pending&
{
  _pimpl->will_be_skipped = value;
  return *this;
}

//==============================================================================
bool Phase::Pending::will_be_skipped() const
{
  return _pimpl->will_be_skipped;
}

} // namespace rmf_task
