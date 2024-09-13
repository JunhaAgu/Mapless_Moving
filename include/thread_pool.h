#ifndef _THREAD_POOL_H_
#define _THREAD_POOL_H_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

#include <pthread.h>
#include <sched.h>

class ThreadPool {
 public:
  enum ThreadPoolStatus { kRun = 0, kKill = 1 };

 public:
  explicit ThreadPool(const int num_thread);
  explicit ThreadPool(const int num_thread,
                      const std::vector<int>& cpu_affinity_numbers);
  ~ThreadPool();
  void EnqueueTask(std::function<void()> job);
  template <typename Func, typename... Args>
  std::future<typename std::invoke_result<Func, Args...>::type>
  EnqueueTaskAndGetResultInFuture(Func&& func, Args&&... args);
  int GetNumOfOngoingTasks();
  int GetNumOfQueuedTasks();
  int GetNumOfTotalThreads();
  int GetNumOfAwaitingThreads();

 private:
  void WakeUpOneThread();
  void WakeUpAllThreads();
  bool AllocateCpuProcessorForEachThread(std::thread& input_thread,
                                         const int cpu_core_index);
  void RunProcessForWorkerThread();

 private:
  std::vector<std::thread> worker_thread_list_;
  std::queue<std::function<void()>> task_queue_;

  std::mutex mutex_;
  std::condition_variable condition_variable_;
  std::atomic<ThreadPoolStatus> thread_pool_status_;

  int num_of_ongoing_tasks_{0};
};

template <typename Func, typename... Args>
std::future<typename std::invoke_result<Func, Args...>::type>  // from C++17
ThreadPool::EnqueueTaskAndGetResultInFuture(Func&& func, Args&&... args) {
  using ReturnType = typename std::invoke_result<Func, Args...>::type;

  auto task_ptr = std::make_shared<std::packaged_task<ReturnType()>>(
      std::bind(std::forward<Func>(func),
                std::forward<Args>(args)...));  // bind for perfect forwarding
  std::future<ReturnType> future_result = task_ptr->get_future();
  // lhs: std::future, rhs: std::promise

  mutex_.lock();
  task_queue_.push([task_ptr]() { (*task_ptr)(); });
  mutex_.unlock();

  WakeUpOneThread();

  return future_result;
}

#endif