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

#ifndef RMF_TASK__LOG_HPP
#define RMF_TASK__LOG_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <functional>
#include <memory>
#include <string>

#include <rmf_traffic/Time.hpp>

namespace rmf_task {

class Log;
using ConstLogPtr = std::shared_ptr<const Log>;

//==============================================================================
class Log
{
public:
  // Inner class declarations. See below for their definitions.
  class Entry;
  class View;
  class Reader;

  /// A computer-friendly ranking of how serious the log entry is.
  enum class Tier : uint32_t
  {
    /// This is a sentinel value that should not generally be used.
    Uninitialized = 0,

    /// An expected occurrence took place.
    Info,

    /// An unexpected, problematic occurrence took place, but it can be
    /// recovered from. Human attention is recommended but not necessary.
    Warning,

    /// A problem happened, and humans should be alerted.
    Error
  };

  /// Construct a log.
  ///
  /// \param[in] clock
  ///   Specify a clock for this log to use. If nullptr is given, then
  ///   std::chrono::system_clock::now() will be used.
  Log(std::function<rmf_traffic::Time()> clock = nullptr);

  // TODO(MXG): Should we have a debug log option?

  /// Add an informational entry to the log.
  void info(std::string text);

  /// Add a warning to the log.
  void warn(std::string text);

  /// Add an error to the log.
  void error(std::string text);

  /// Push an entry of the specified severity.
  void push(Tier tier, std::string text);

  /// Insert an arbitrary entry into the log.
  void insert(Log::Entry entry);

  /// Get a View of the current state of this log. Any new entries that are
  /// added after calling this function will not be visible to the View that
  /// is returned.
  View view() const;

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// A single entry within the log.
class Log::Entry
{
public:

  /// What was the tier of this entry.
  Tier tier() const;

  /// Sequence number for this log entry. This increments once for each new
  /// log entry, until overflowing and wrapping around to 0.
  uint32_t seq() const;

  /// What was the timestamp of this entry.
  rmf_traffic::Time time() const;

  /// What was the text of this entry.
  const std::string& text() const;

  class Implementation;
private:
  Entry();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// A snapshot view of the log's contents. This is thread-safe to read even
/// while new entries are being added to the log, but those new entries will
/// not be seen by this View. You must retrieve a new View to see new entries.
class Log::View
{
public:
  class Implementation;
private:
  View();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// A Reader that can iterate through the Views of Logs. The Reader will keep
/// track of which Entries have already been viewed, so every Entry read by a
/// single Reader instance is unique.
class Log::Reader
{
public:

  /// Construct a Reader
  Reader();

  class Iterable;

  /// Create an object that can iterate through the entries of a View. Any
  /// entries that have been read by this Reader in the past will be skipped.
  /// This can be used in a range-based for loop, e.g.:
  ///
  /// \code
  /// for (const auto& entry : reader.read(view))
  /// {
  ///   std::cout << entry.text() << std::endl;
  /// }
  /// \endcode
  Iterable read(const View& view);

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Log::Reader::Iterable
{
public:

  class iterator;
  using const_iterator = iterator;

  /// Get the beginning iterator of the read
  iterator begin() const;

  /// Get the ending iterator of the read
  iterator end() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Log::Reader::Iterable::iterator
{
public:

  /// Dereference operator
  const Entry& operator*() const;

  /// Drill-down operator
  const Entry* operator->() const;

  /// Pre-increment operator: ++it
  ///
  /// \note This is more efficient than the post-increment operator.
  ///
  /// \warning It is undefined behavior to perform this operation on an iterator
  /// that is equal to Log::Reader::Iterable::end().
  ///
  /// \return a reference to the iterator itself
  iterator& operator++();

  /// Post-increment operator: it++
  ///
  /// \warning It is undefined behavior to perform this operation on an iterator
  /// that is equal to Log::Reader::Iterable::end().
  ///
  /// \return a copy of the iterator before it was incremented.
  iterator operator++(int);

  /// Equality comparison operator
  bool operator==(const iterator& other) const;

  /// Inequality comparison operator
  bool operator!=(const iterator& other) const;

  class Implementation;
private:
  iterator();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_task

#endif // RMF_TASK__LOG_HPP
