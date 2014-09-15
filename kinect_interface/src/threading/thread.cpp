#include <stdio.h>  // perror
#include <stdlib.h>  // exit
#include <thread>
#include <sstream>
#if defined(WIN32) || defined(_WIN32)
  #include <windows.h>
#elif defined(__APPLE__)
  #include <pthread.h>
#else
  #include <sys/prctl.h>
#endif
#include "threading/thread.h"

using std::thread;

namespace jtil {
namespace threading {
  
  static void* threadFunction(void* arg) {
    (* reinterpret_cast<Callback<void>*>(arg))();
    return 0;
  }
  
  thread MakeThread(Callback<void>* body) {
    void* arg = reinterpret_cast<void*>(body);
    thread tid = thread(threadFunction, arg);
    return tid;
  }
  
  void* GetThreadID(std::thread* thread) {
    std::stringstream ios; 
    ios << thread->get_id();
    void* tid;
    ios >> tid;
    return tid;
  }

#if defined(WIN32) || defined(_WIN32)
  // SetThreadName - Windows implementation
  void SetThreadName(char const * thread_name) {
#pragma pack(push,8)
    struct THREADNAME_INFO {
      DWORD dwType; // Must be 0x1000.
      LPCSTR szName; // Pointer to name (in user addr space).
      DWORD dwThreadID; // Thread ID (-1=caller thread).
      DWORD dwFlags; // Reserved for future use, must be zero.
    };
#pragma pack(pop)

    THREADNAME_INFO info;
    info.dwType = 0x1000;
    info.szName = thread_name;
    info.dwThreadID = -1;		// this thread
    info.dwFlags = 0;

    __try {
      const DWORD MS_VC_EXCEPTION=0x406D1388;
      RaiseException( MS_VC_EXCEPTION, 0, sizeof(info)/sizeof(ULONG_PTR), (ULONG_PTR*)&info );
    } __except(EXCEPTION_EXECUTE_HANDLER) {
    }
  }
#elif defined(__APPLE__)

  // SetThreadName - OS X implementation
  void SetThreadName(char const * thread_name)
  {
    pthread_setname_np(thread_name);
  }

#else

  // SetThreadName - Linux implementation
  void SetThreadName(char const * thread_name)
  {
    prctl(PR_SET_NAME, "thread_name", 0, 0, 0);
  }

#endif
  
}  // namespace threading
}  // namespace jtil
