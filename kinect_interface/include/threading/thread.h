//
//  thread.h
//
//  Created by Jonathan Tompson on 7/10/12.
//
//  A simple wrapper around std::thread for spawning threads from a Callback<>
//  

#pragma once

#include <thread>
#include "threading/callback.h"

namespace jtil {
namespace threading {

  // Returns the thread id of the thread create to run 'body'. The
  // latter ownership is determined by whether the callback is call-once
  // or not. Internally, 'body' is only invoked once.
  // If a thread cannot be created, this call will exit()
  std::thread MakeThread(threading::Callback<void>* body);

  // Name the current thread.
  void SetThreadName(const char* thread_name);
  
  // Creates a void* equivilient of the std::thread::id class
  void* GetThreadID(std::thread* thread);

};  // namespace base
};  // namespace jtil
