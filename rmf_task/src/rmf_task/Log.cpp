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

#include <rmf_task/Log.hpp>

#include <stdexcept>
#include <list>
#include <mutex>

namespace rmf_task {

//==============================================================================
struct LogData
{
  mutable std::mutex mutex;
  std::list<Log::Entry> entries;
};

//==============================================================================
class Log::Implementation
{
public:
  std::function<rmf_traffic::Time()> clock;
  std::shared_ptr<LogData> data;
  uint32_t seq = 0;

  Implementation(std::function<rmf_traffic::Time()> clock_)
  : clock(std::move(clock_)),
    data(std::make_shared<LogData>())
  {
    if (!clock)
    {
      clock = []()
        {
          return rmf_traffic::Time(
            rmf_traffic::Duration(
              std::chrono::system_clock::now().time_since_epoch()));
        };
    }
  }

};

//==============================================================================
class Log::Entry::Implementation
{
public:

  static Entry make(
    Tier tier,
    uint32_t seq,
    rmf_traffic::Time time,
    std::string text)
  {
    Log::Entry output;
    output._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{
        tier,
        seq,
        time,
        std::move(text)
      });

    return output;
  }

  Tier tier;
  uint32_t seq;
  rmf_traffic::Time time;
  std::string text;

};

//==============================================================================
class Log::View::Implementation
{
public:

  static View make(const Log& log)
  {
    View output;

    if (log._pimpl->data->entries.empty())
    {
      output._pimpl = rmf_utils::make_impl<Implementation>(
        Implementation{log._pimpl->data, std::nullopt});
    }
    else
    {
      output._pimpl = rmf_utils::make_impl<Implementation>(
        Implementation{
          log._pimpl->data,
          log._pimpl->data->entries.cbegin()
        });
    }

    return output;
  }

  static const Implementation& get(const View& view)
  {
    return *view._pimpl;
  }

  std::shared_ptr<const LogData> shared;

  /// begin is the iterator for the first entry in the entire log
  std::optional<std::list<Log::Entry>::const_iterator> begin;
};

//==============================================================================
class Log::Reader::Implementation
{
public:

  using OptListIterator = std::optional<std::list<Log::Entry>::const_iterator>;
  struct Memory
  {
    std::weak_ptr<const LogData> weak;
    std::shared_ptr<OptListIterator> last;

    Memory()
      : last(std::make_shared<OptListIterator>(std::nullopt))
    {
      // Do nothing
    }
  };

  std::unordered_map<const void*, Memory> memories;

  Iterable read(const View& view);
};

//==============================================================================
class Log::Reader::Iterable::Implementation
{
public:
  using base_iterator = std::list<Log::Entry>::const_iterator;
  std::shared_ptr<const LogData> shared;
  std::optional<iterator> begin;

  static Log::Reader::Iterable make(std::shared_ptr<const LogData> shared,
    std::optional<base_iterator> begin);
};

//==============================================================================
class Log::Reader::Iterable::iterator::Implementation
{
public:
  using base_iterator = std::list<Log::Entry>::const_iterator;
  base_iterator it;
  std::shared_ptr<const LogData> data;

  static iterator make(base_iterator it, std::shared_ptr<const LogData> data)
  {
    iterator output;
    output._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{std::move(it), std::move(data)});

    return output;
  }

  static iterator end()
  {
    return iterator();
  }
};

//==============================================================================
Log::Reader::Iterable Log::Reader::Iterable::Implementation::make(
  std::shared_ptr<const LogData> shared,
  std::optional<base_iterator> begin)
{
  Iterable iterable;

  if (begin.has_value())
  {
    iterable._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(shared),
        iterator::Implementation::make(*begin, shared)
      });
  }
  else
  {
    iterable._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(shared),
        iterator::Implementation::end()
      });
  }

  return iterable;
}

//==============================================================================
auto Log::Reader::Implementation::read(const View& view) -> Iterable
{
  const auto& v = View::Implementation::get(view);
  const auto it = memories.insert({v.shared.get(), Memory()}).first;
  auto& memory = it->second;
  if (memory.weak.lock())
  {
    if (!memory.last->has_value())
      *memory.last = v.begin;
    else
      ++(**memory.last);
  }
  else
  {
    memory.weak = v.shared;
    *memory.last = v.begin;
  }

  auto iterable = Iterable::Implementation::make(v.shared, memory.last);

  return iterable;
}

//==============================================================================
Log::Log(std::function<rmf_traffic::Time()> clock)
: _pimpl(rmf_utils::make_impl<Implementation>(std::move(clock)))
{
  // Do nothing
}

//==============================================================================
void Log::info(std::string text)
{
  push(Tier::Info, std::move(text));
}

//==============================================================================
void Log::warn(std::string text)
{
  push(Tier::Warning, std::move(text));
}

//==============================================================================
void Log::error(std::string text)
{
  push(Tier::Error, std::move(text));
}

//==============================================================================
void Log::push(Tier tier, std::string text)
{
  if (Tier::Uninitialized == tier)
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Log::push] Tier was set to Uninitialized, which is illegal.");
    // *INDENT-ON*
  }

  _pimpl->data->emplace_back(
    Entry::Implementation::make(
      tier, _pimpl->seq++, _pimpl->clock(), std::move(text)));
}

//==============================================================================
void Log::insert(Log::Entry entry)
{
  _pimpl->data->emplace_back(std::move(entry));
}

//==============================================================================
Log::View Log::view() const
{
  return View::Implementation::make(*this);
}

//==============================================================================
auto Log::Entry::tier() const -> Tier
{
  return _pimpl->tier;
}

//==============================================================================
uint32_t Log::Entry::seq() const
{
  return _pimpl->seq;
}

//==============================================================================
rmf_traffic::Time Log::Entry::time() const
{
  return _pimpl->time;
}

//==============================================================================
const std::string& Log::Entry::text() const
{
  return _pimpl->text;
}

//==============================================================================
Log::Entry::Entry()
{
  // Do nothing
}

//==============================================================================
Log::View::View()
{
  // Do nothing
}

//==============================================================================
Log::Reader::Reader()
: _pimpl(rmf_utils::make_unique_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
auto Log::Reader::read(const View& view) -> Iterable
{
  return _pimpl->read(view);
}

//==============================================================================
auto Log::Reader::Iterable::begin() const -> iterator
{
  return _pimpl->begin.value_or(iterator::Implementation::end());
}

//==============================================================================
auto Log::Reader::Iterable::end() const -> iterator
{
  return iterator::Implementation::end();
}

//==============================================================================
auto Log::Reader::Iterable::iterator::operator*() const -> const Entry&
{
  return *_pimpl->it;
}

//==============================================================================
auto Log::Reader::Iterable::iterator::operator->() const -> const Entry*
{
  return &(*_pimpl->it);
}

//==============================================================================
auto Log::Reader::Iterable::iterator::operator++() -> iterator&
{
  if (!_pimpl.get())
    return *this;

  if (_pimpl->it == _pimpl->last)
    _pimpl = nullptr;
  else
    ++_pimpl->it;

  return *this;
}

//==============================================================================
auto Log::Reader::Iterable::iterator::operator++(int) -> iterator
{
  auto original = *this;
  ++(*this);
  return original;
}

//==============================================================================
bool Log::Reader::Iterable::iterator::operator==(const iterator& other) const
{
  if (!_pimpl.get() || !other._pimpl.get())
    return _pimpl.get() == other._pimpl.get();

  return _pimpl->it == other._pimpl->it;
}

//==============================================================================
bool Log::Reader::Iterable::iterator::operator!=(const iterator& other) const
{
  return !(*this == other);
}

//==============================================================================
Log::Reader::Iterable::iterator::iterator()
{
  // Do nothing
}

} // namespace rmf_task
