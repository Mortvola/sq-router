#include "ThreadPool.h"
#include <iostream>

ThreadPool::ThreadPool(int numThreads)
{
  for (int i = 0; i < numThreads; i++)
  {
    m_threads.push_back(std::thread(&ThreadPool::taskLoop, this));
  }
}

ThreadPool::~ThreadPool()
{
  m_stopThreads = true;

  m_queueCondition.notify_all();

  for (auto &thread: m_threads)
  {
    thread.join();
  }
}

void ThreadPool::taskLoop()
{
  for (;;)
  {
    {
      std::unique_lock<std::mutex> lock(m_queueMutex);

      if (m_queue.size() > 0)
      {
        auto task = m_queue.front();
        m_queue.pop_front();

        lock.unlock();

        try
        {
          task();
        }
        catch(const std::exception& e)
        {
          std::cerr << e.what() << '\n';
        }
      }
    }

    {
      std::unique_lock<std::mutex> lock(m_queueMutex);

      if (m_queue.size() == 0)
      {
        if (m_stopThreads)
        {
          break;
        }

        m_queueCondition.wait(lock,
          [this]
          {
            return m_queue.size() != 0 || m_stopThreads;
          });
      }
    }
  }
}

