#pragma once

#include <vector>
#include <thread>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>
#include <future>
#include <iostream>

class ThreadPool
{
public:

  ThreadPool(int numThreads);
  ~ThreadPool();

  template<class T>
  std::future<T> postTask(
    std::function<T()> task);

private:

  void taskLoop();

  std::vector<std::thread> m_threads;
  std::deque<std::function<void()>> m_queue;
  std::mutex m_queueMutex;
  std::condition_variable m_queueCondition;
  std::atomic<bool> m_stopThreads {false};
};

template<class T>
std::future<T> ThreadPool::postTask(
  std::function<T()> task)
{
  auto promise = std::make_shared<std::promise<T>>();
  auto future = promise->get_future();

  std::unique_lock<std::mutex> lock(m_queueMutex);
  m_queue.push_back(
    [promise, task]()
    {
      try
      {
        T result = task();
        promise->set_value(result);
      }
      catch (std::exception &e)
      {
        std::cerr << e.what() << std::endl;
        promise->set_value({});
      }
    }
  );

  m_queueCondition.notify_all();

  return future;
}

