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
  Header header;

};

//==============================================================================
Phase::Tag::Tag(
  Id id_,
  Header header_)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        id_,
        std::move(header_)
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
const Header& Phase::Tag::header() const
{
  return _pimpl->header;
}

//==============================================================================
class Phase::Completed::Implementation
{
public:

  ConstSnapshotPtr snapshot;
  rmf_traffic::Time start;
  rmf_traffic::Time finish;
};

//==============================================================================
Phase::Completed::Completed(
  ConstSnapshotPtr snapshot_,
  rmf_traffic::Time start_,
  rmf_traffic::Time finish_)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(snapshot_),
        start_,
        finish_
      }))
{
  // Do nothing
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
  Event::ConstStatePtr finish_event;
  rmf_traffic::Duration estimated_remaining_time;
};

//==============================================================================
Phase::ConstSnapshotPtr Phase::Snapshot::make(const Active& active)
{
  Snapshot output;
  output._pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{
      active.tag(),
      Event::Snapshot::make(*active.final_event()),
      active.estimate_remaining_time()
    });

  return std::make_shared<Snapshot>(std::move(output));
}

//==============================================================================
Phase::ConstTagPtr Phase::Snapshot::tag() const
{
  return _pimpl->tag;
}

//==============================================================================
Event::ConstStatePtr Phase::Snapshot::final_event() const
{
  return _pimpl->finish_event;
}

//==============================================================================
rmf_traffic::Duration Phase::Snapshot::estimate_remaining_time() const
{
  return _pimpl->estimated_remaining_time;
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
