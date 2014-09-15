#include "threading/thread_pool.h"
#include "threading/thread.h"

#if defined(WIN32) || defined(_WIN32)
  #define snprintf _snprintf
#endif

namespace jtil {
namespace threading {
  
  using threading::Callback;
  using threading::MakeCallableOnce;
  
  ThreadPool::ThreadPool(int num_workers) {
    queue_lock_.lock();  // Prevent tasks from being added until workers spawn
    // Spawn worker threads
    stop_called_ = false;
    stop_finished_ = false;
    thread_to_join_in_destructor_ = NULL;
    num_workers_ = num_workers;
    worker_ids_ = new std::thread[num_workers_];
    stopCB_many_ = MakeCallableMany(&ThreadPool::stop, this);
    stopCB_once_ = MakeCallableOnce(&ThreadPool::stop, this);
    worker_cvs_ = new std::condition_variable[num_workers_];
    
    // Spawn the worker threads, worker threads are just themselves callbacks
    idle_worker_tasks_ = new Callback<void>*[num_workers_];
    for (int i = 0; i < num_workers_; i ++) {
      Callback<void>* worker_callback_ =
      MakeCallableOnce(&ThreadPool::workerMain, this, i);
      worker_ids_[i] = MakeThread(worker_callback_);
      idle_worker_tasks_[i] = NULL;
    }
    queue_lock_.unlock();
  }
  
  // REQUIRES: stop() have completed executing. --> Class spec
  ThreadPool::~ThreadPool() {
    std::unique_lock<std::mutex> unique_lock(stop_finished_lock_);
    while (!stop_finished_) {  // Avoid race condition with stop function
      stop_finished_cv_.wait(unique_lock);
    }
    unique_lock.unlock();

    if (thread_to_join_in_destructor_) {
      thread_to_join_in_destructor_->join();
    }
    
    // Now grab the lock and release data
    queue_lock_.lock();
    Callback<void>* p_cur_method;
    while (!callback_queue_.empty()) {
      p_cur_method = callback_queue_.dequeue();
      if (p_cur_method->once()) {
        delete p_cur_method;  // Delete only once callbacks
      }
    }
    delete stopCB_many_;
    delete stopCB_once_;
    delete[] worker_ids_;
    delete[] worker_cvs_;
    delete[] idle_worker_tasks_;
    queue_lock_.unlock();
  }
  
  void ThreadPool::waitForStopFinish() {
    std::unique_lock<std::mutex> unique_lock(stop_finished_lock_);
    while (!stop_finished_) {  // Avoid race condition with stop function
      stop_finished_cv_.wait(unique_lock);
    }
    unique_lock.unlock();
  }
  
  // Waits for all the workers to finish processing the pending tasks
  // and stop then stop the pool. This call may be issued from within
  // a worker thread itself.
  void ThreadPool::stop() {
    queue_lock_.lock();
    if (stop_called_) {  // Just in case 2 active tasks both call stop
      queue_lock_.unlock();
      return;
    }
    stop_called_ = true;
    // We need to manually iterate through all the worker cb's to broadcast
    for (int i = 0; i < num_workers_; i ++) {
      worker_cvs_[i].notify_all();
    }
    queue_lock_.unlock();
    
    // Wait for worker threads to finish
    for (int i = 0; i < num_workers_; i ++) {
      if (worker_ids_[i].get_id() != std::this_thread::get_id()) {
        worker_ids_[i].join();
      } else {
        thread_to_join_in_destructor_ = &worker_ids_[i];
      }
    }
    
    // Now all the workers (except potentially this one) are finished.
    // Let the destructor know that it can release TP information.
    stop_finished_lock_.lock();
    stop_finished_ = true;
    stop_finished_cv_.notify_all();
    stop_finished_lock_.unlock();
  }
  
  void ThreadPool::addTask(Callback<void>* task) {
    // Get queue lock, enqueue the task then release lock
    queue_lock_.lock();
    
    // Check for idle workers, and wake up the worker if it is idle
    if (!stop_called_ && !idle_worker_queue_.empty()) {
      int idle_worker = idle_worker_queue_.dequeue();
      idle_worker_tasks_[idle_worker] = task;
      worker_cvs_[idle_worker].notify_all();
    } else {
      // Otherwise add the queue to the task and someone will pick it up.
      callback_queue_.enqueue(task);
    }
    queue_lock_.unlock();
  }
  
  int ThreadPool::count() const {
    queue_lock_.lock();
    int count_val = callback_queue_.size();
    queue_lock_.unlock();
    return count_val;
  }
  
  void ThreadPool::workerMain(int thread_index) {
    char thread_name[32];
    snprintf(thread_name, 32, "ThreadPool::workerMain - %d", thread_index);
    SetThreadName(thread_name);
    std::unique_lock<std::mutex> unique_lock(queue_lock_);
    // When in the loop we have the lock, unless otherwise noted
    while (!stop_called_) {
      // Check if any remaining tasks are queued
      if (callback_queue_.empty()) {
        idle_worker_queue_.enqueue(thread_index);  // Go onto the idle list
        worker_cvs_[thread_index].wait(unique_lock);
        // If we were woken, then there must be a task waiting for us (unless
        // a stop was called).
        if (idle_worker_tasks_[thread_index]) {
          unique_lock.unlock();
          // Manipulating idle_worker_tasks_ outside the lock is safe since
          // addTask wont update unless the worker thread is on the wait queue
          // (which it cannot be at this point).
          (*idle_worker_tasks_[thread_index])();
          idle_worker_tasks_[thread_index] = NULL;
          unique_lock.lock();
        }
      } else {
        // Otherwise, dequeue a task and execute it
        Callback<void>* taskBody = callback_queue_.dequeue();
        // THREAD DOESN'T HAVE LOCK HERE
        // This structure was the result of a conversation with Prof. Lerner.
        // Want to avoid race condition between stop and destructor.
        // Prof. Lerner: "If you do a == b, it will be comparing the pointers,
        // both for TP::stop() and for the tp instance for you."
        unique_lock.unlock();
        if ((taskBody == stopCB_once_) ||
            (taskBody == stopCB_many_)) {
          (*taskBody)();
          return;  // Return without touching tp (may have been destroyed!)
        } else {
          (*taskBody)();
        }
        unique_lock.lock();  // RE-ACQUIRE LOCK before iterating again
      }
    }
    unique_lock.unlock();
  }
  
}  // namespace threading
}  // namespace jtil
