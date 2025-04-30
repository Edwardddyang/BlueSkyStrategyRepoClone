/* 
Mimick a counting semaphore to limit the number of threads
that can run in parallel at any given time
*/

#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iostream>

class ThreadManager {
 private:
  unsigned int max_threads;
  // This must be atomic because multiple threads can still write to it at the same time
  std::atomic<int> active_threads;
  std::mutex mutex;
  std::condition_variable var;

 public:
  explicit ThreadManager(int max_threads) : max_threads(max_threads), active_threads(0) {}
  explicit ThreadManager() : max_threads(0), active_threads(0) {}
  void set_max_threads(int threads) {
    if (active_threads > 0) {
      std::cout << "Cannot modify active threads while threads are active" << std::endl;
      exit(1);
    }
    max_threads = threads;
  }

  // Increment active_threads and make progress on the thread
  void acquire() {
    // Use unique_lock instead of lock_guard since condition var::wait requires
    // a unique_lock type.
    std::unique_lock<std::mutex> lock(mutex);
    // Note that var::wait() releases the lock while waiting
    var.wait(lock, [this] {return active_threads < max_threads;});
    active_threads += 1;
    // Lock released
  }

  // Thread has finished activity, decrement active_threads, and wake up the next waiting thread
  void release() {
    std::lock_guard<std::mutex> lock(mutex);
    active_threads -= 1;
    var.notify_one();
    // Lock released
  }
};
