//
//  thread_pool.h
//
//  Created by Jonathan Tompson on 7/10/12.
//
//  Origionally this was a pthreads thread pool (linux only) which was written
//  for my Spring 2012 NYU class Mulitcore Programming.  I ported the pthreads
//  calls to C++11 std::thread.  
//
//  It makes use of a small optimization where, new tasks will be handed 
//  directly to idle threads if any are idle, otherwise it will be placed on the
//  queue for execution (with appropriate overhead).

#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include "threading/callback.h"
#include "threading/thread_pool.h"
#include "threading/callback_queue.h"

namespace jtil {
namespace threading {
  
  class ThreadPool {
  public:
    
    explicit ThreadPool(const int num_workers);
    
    // ~ThreadPool() REQUIRES: stop() have completed executing.
    ~ThreadPool();
    
    // addTask() - Requests the execution of 'task' on an undetermined worker 
    // thread.
    void addTask(Callback<void>* task);
    
    // Waits for all the workers to finish processing the ongoing tasks
    // and stop then stop the pool. This call may be issued from within
    // a worker thread itself.  Stop is blocking, and once stop is called, the
    // thread pool cannot be restarted.
    void stop();
    
    // Returns the current size of the dispatch queue (pending tasks waiting to
    // be executed).
    int count() const;
    
    // If stop was called by another thread (potentially a thread in the thread
    // pool itself), this is essentially a blocking method which waits for the 
    // other stop to finish.  Importantly, the wait does not busy wait!
    void waitForStopFinish();

    inline const int& num_workers() const { return num_workers_; }
    
  private:
    mutable std::mutex queue_lock_;
    mutable std::mutex stop_finished_lock_;
    CallbackQueue<Callback<void>*> callback_queue_;  // tasks waiting 4 execut.
    std::thread* worker_ids_;
    Callback<void>* stopCB_once_;
    Callback<void>* stopCB_many_;
    int num_workers_;
    bool stop_called_;
    bool stop_finished_;
    std::condition_variable stop_finished_cv_;

    // A "hack" to fix a race in Win32 --> We want stop worker threads to be
    // able to execute the stop function.  Unforuntately, the std::mutex will
    // throw an assertion failure if we call the mutex destructor before the
    // worker thread has fully shutdown.
    std::thread* thread_to_join_in_destructor_;  
    
    // New for ThreadPool
    std::condition_variable* worker_cvs_;  // Array of cv's - 1 for each worker
    Callback<void>** idle_worker_tasks_;
    CallbackQueue<int> idle_worker_queue_;  // Queue of idle workers
    
    // This is the main worker thread:
    void workerMain(const int thread_index);  
    
    // Non-copyable, non-assignable.
    ThreadPool(const ThreadPool&);
    ThreadPool& operator=(const ThreadPool&);
  };
  
};  // namespace threading
};  // namespace jtil
