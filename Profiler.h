#pragma once

#include <string>
#include <chrono>
#include <iostream>

class Profiler
{
public:
    Profiler (const std::string &name)
    :
      m_name(name)
    {}

    ~Profiler()
    {
      printResults();
    }

    void start ()
    {
      m_start = std::chrono::steady_clock::now();
    }

    void stop ()
    {
      m_duration += std::chrono::steady_clock::now() - m_start;
      m_samples++;
      // if (m_samples >= 1000)
      // {
      //   printResults();
      // }
    }

    void printResults()
    {
      if (m_samples == 0)
      {
        std::cerr << m_name << ": rate: N/A, duration: 0, samples: 0" << std::endl;
      }
      else
      {
        auto seconds = std::chrono::duration<double>(m_duration).count();
        auto rate = m_samples / seconds;

        std::cerr << m_name
          << ": rate: " << rate 
          << " per second, duration: " << seconds
          << ", samples: " << m_samples
          << std::endl;
      }

      m_duration = std::chrono::nanoseconds(0);
      m_samples = 0;
    }

private:

    std::string m_name;
    std::chrono::steady_clock::time_point m_start {};
    std::chrono::steady_clock::duration m_duration {};
    int m_samples {0};
};

