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

#include <rmf_utils/catch.hpp>

#include <rmf_task/Log.hpp>

#include <random>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <optional>

SCENARIO("Writing and reading logs")
{
  rmf_task::Log log;
  rmf_task::Log::Reader reader;

  std::size_t expected_count = 0;
  std::size_t count = 0;
  for (const auto& entry : reader.read(log.view()))
  {
    CHECK(entry.seq() == count);
    ++count;
  }

  CHECK(count == expected_count);

  log.info("Test text");
  log.info("More test text");
  log.warn("Some warning text");

  expected_count = 3;
  count = 0;
  for (const auto& entry : reader.read(log.view()))
  {
    CHECK(entry.seq() == count);
    ++count;
  }

  CHECK(count == expected_count);

  expected_count = 0;
  count = 0;
  for (const auto& entry : reader.read(log.view()))
  {
    (void)(entry);
    ++count;
  }

  CHECK(count == expected_count);
}

struct SyncView
{
  bool ready_for_new_view = false;
  std::condition_variable new_view_ready;
  std::mutex mutex;
  std::optional<rmf_task::Log::View> view;
};

//==============================================================================
SCENARIO("Multi-threaded read/write with synced view")
{
  std::shared_ptr<SyncView> sync = std::make_shared<SyncView>();
  std::shared_ptr<std::atomic_bool> test_finished =
    std::make_shared<std::atomic_bool>(false);

  auto all_seqs = std::make_shared<std::vector<uint32_t>>();
  auto all_text = std::make_shared<std::vector<std::string>>();
  auto log = std::make_shared<rmf_task::Log>();

  std::thread producer(
    [](
      std::shared_ptr<rmf_task::Log> log,
      std::shared_ptr<SyncView> sync,
      std::shared_ptr<std::atomic_bool> test_finished)
    {
      std::random_device r;
      std::default_random_engine eng(r());
      std::uniform_real_distribution<double> entry_dist(0, 1);
      std::uniform_int_distribution<uint> tier_dist(
        static_cast<uint>(rmf_task::Log::Tier::Info),
        static_cast<uint>(rmf_task::Log::Tier::Error));

      std::size_t counter = 0;
      while (!test_finished->load())
      {
        if (entry_dist(eng) > 0.95)
        {
          const auto tier = static_cast<rmf_task::Log::Tier>(tier_dist(eng));
          log->push(tier, "This is log #" + std::to_string(counter++));
        }

        std::unique_lock<std::mutex> lock(sync->mutex, std::defer_lock);
        if (lock.try_lock())
        {
          if (sync->ready_for_new_view)
          {
            sync->view = log->view();
            sync->ready_for_new_view = false;
            lock.unlock();
            sync->new_view_ready.notify_all();
          }
        }
      }
    }, log, sync, test_finished);

  std::thread consumer(
    [](
      std::shared_ptr<std::vector<uint32_t>> all_seqs,
      std::shared_ptr<std::vector<std::string>> all_text,
      std::shared_ptr<SyncView> sync,
      std::shared_ptr<std::atomic_bool> test_finished)
    {
      rmf_task::Log::Reader reader;
      while (!test_finished->load())
      {
        std::size_t new_entry_count = 0;
        std::unique_lock<std::mutex> lock(sync->mutex);
        if (sync->view.has_value())
        {
          for (const auto& entry : reader.read(*sync->view))
          {
            ++new_entry_count;
            all_seqs->push_back(entry.seq());
            all_text->push_back(entry.text());
          }
        }

        sync->view = std::nullopt;
        sync->ready_for_new_view = true;
        sync->new_view_ready.wait(
          lock,
          [sync, test_finished]()
          {
            return sync->view.has_value() || test_finished->load();
          });
      }
    }, all_seqs, all_text, sync, test_finished);

  std::this_thread::sleep_for(std::chrono::seconds(1));
  test_finished->store(true);

  // Use this condition variable to wake up the consumer, in case it's waiting
  sync->new_view_ready.notify_all();

  producer.join();
  consumer.join();

  std::size_t index = 0;
  for (const auto& entry : rmf_task::Log::Reader().read(log->view()))
  {
    if (index < all_seqs->size())
      CHECK((*all_seqs)[index] == entry.seq());

    if (index < all_text->size())
      CHECK((*all_text)[index] == entry.text());

    ++index;
  }
}

//==============================================================================
SCENARIO("Multi-threaded read/write without syncing")
{

  std::shared_ptr<std::atomic_bool> test_finished =
    std::make_shared<std::atomic_bool>(false);

  auto all_seqs = std::make_shared<std::vector<uint32_t>>();
  auto all_text = std::make_shared<std::vector<std::string>>();
  auto log = std::make_shared<rmf_task::Log>();

  std::thread producer(
    [](
      std::shared_ptr<rmf_task::Log> log,
      std::shared_ptr<std::atomic_bool> test_finished)
    {
      std::random_device r;
      std::default_random_engine eng(r());
      std::uniform_real_distribution<double> entry_dist(0, 1);
      std::uniform_int_distribution<uint> tier_dist(
        static_cast<uint>(rmf_task::Log::Tier::Info),
        static_cast<uint>(rmf_task::Log::Tier::Error));

      std::size_t counter = 0;
      while (!test_finished->load())
      {
        if (entry_dist(eng) > 0.95)
        {
          const auto tier = static_cast<rmf_task::Log::Tier>(tier_dist(eng));
          log->push(tier, "This is log #" + std::to_string(counter++));
        }
      }
    }, log, test_finished);

  std::thread consumer(
    [](
      std::shared_ptr<rmf_task::Log> log,
      std::shared_ptr<std::vector<uint32_t>> all_seqs,
      std::shared_ptr<std::vector<std::string>> all_text,
      std::shared_ptr<std::atomic_bool> test_finished)
    {
      rmf_task::Log::Reader reader;
      while (!test_finished->load())
      {
        for (const auto& entry : reader.read(log->view()))
        {
          all_seqs->push_back(entry.seq());
          all_text->push_back(entry.text());
        }
      }
    }, log, all_seqs, all_text, test_finished);

  std::this_thread::sleep_for(std::chrono::seconds(1));
  test_finished->store(true);

  producer.join();
  consumer.join();

  std::size_t index = 0;
  for (const auto& entry : rmf_task::Log::Reader().read(log->view()))
  {
    if (index < all_seqs->size())
      CHECK((*all_seqs)[index] == entry.seq());

    if (index < all_text->size())
      CHECK((*all_text)[index] == entry.text());

    ++index;
  }
}
