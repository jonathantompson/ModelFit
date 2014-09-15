//
//  clock.h
//
//  Created by Jonathan Tompson on 3/30/12.
//
//  getTime() is threadsafe
//

#pragma once

#include <mutex>

#if defined(__APPLE_CC__)
#include <mach/mach.h>
#include <mach/mach_time.h>

namespace jtil { 
namespace clk {

  class Clk {
  public:
    Clk() {
      m_cntStart = mach_absolute_time();
      (void) mach_timebase_info(&sTimebaseInfo);
    }

    double getTime() {
      lock_.lock();
      m_cntCurrent = mach_absolute_time();
      elapsedNano = (m_cntCurrent-m_cntStart) * 
        sTimebaseInfo.numer / sTimebaseInfo.denom;
      double ret_val = static_cast<double>(elapsedNano) / 1000000000.0;
      lock_.unlock();
      return ret_val;
    }

  private:
    std::mutex lock_;
    uint64_t m_cntStart;
    uint64_t m_cntCurrent;
    uint64_t elapsedNano;
    mach_timebase_info_data_t sTimebaseInfo;

    // Non-copyable, non-assignable.
    Clk(Clk&);
    Clk& operator=(const Clk&);
  };

};  // namespace clk 
};  // namespace jtil

#elif defined(__GNUC__)
#include <time.h>

namespace jtil {
namespace clk {

  class Clk {
  public:
    Clk() {
      clock_gettime(CLOCK_REALTIME, &m_cntStart);
    }

    double getTime() {
      lock_.lock();
      clock_gettime(CLOCK_REALTIME, &m_cntCurrent);
      elapsedNano = (double)(m_cntCurrent.tv_nsec-m_cntStart.tv_nsec) /
          1000000000.0;
      elapsedSec = (double)(m_cntCurrent.tv_sec-m_cntStart.tv_sec);
      double ret_val = elapsedNano + elapsedSec;
      lock_.unlock();
      return ret_val;
    }

  private:
    std::mutex lock_;
    timespec m_cntStart;
    timespec m_cntCurrent;
    double elapsedNano;
    double elapsedSec;

    // Non-copyable, non-assignable.
    Clk(Clk&);
    Clk& operator=(const Clk&);
  };

};  // namespace clk
};  // namespace jtil

#endif

#if defined( __WIN32__ ) || defined( _WIN32 ) || defined( WIN32 )
#include <profileapi.h>

namespace jtil {
namespace clk {

  class Clk {
  public:
    Clk() {
      QueryPerformanceFrequency(reinterpret_cast<LARGE_INTEGER*>(&m_cntFreq));
      m_secsPerCnt = 1.0 / static_cast<double>(m_cntFreq);
      QueryPerformanceCounter(reinterpret_cast<LARGE_INTEGER*>(&m_cntStart));
    }

    double getTime() {
      lock_.lock();
      QueryPerformanceCounter(reinterpret_cast<LARGE_INTEGER*>(&m_cntCurrent));
      double elapsedSec = static_cast<double>(m_cntCurrent - m_cntStart) * 
        static_cast<double>(m_secsPerCnt);
      lock_.unlock();
      return elapsedSec;
    }

  private:
    std::mutex lock_;
    double elapsedSec;
    __int64 m_cntFreq,  // Frequency of the precision counter
      m_cntStart,  // Current counter value
      m_cntCurrent;
    double  m_secsPerCnt;  // counter period

    // Non-copyable, non-assignable.
    Clk(Clk&);
    Clk& operator=(const Clk&);
  };

};  // namespace clk
};  // namespace jtil

#endif
