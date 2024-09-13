#include "thread_pool.h"

#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <sstream>
#include <string>

using namespace std ::chrono_literals;

void PrintInfoImpl(const std::string& str, const std::string& func_str) {
  std::stringstream ss;
  ss << "[ThreadId:" << std::this_thread::get_id() << "][INFO] \"" << func_str
     << "()\": " << str << "\n";
  std::cerr << ss.str();
}
#define PrintInfo(str) PrintInfoImpl(str, std::string(__func__));

void ThrowErrorImpl(const std::string& str, const std::string& func_str) {
  std::stringstream ss;
  ss << "[ThreadId:" << std::this_thread::get_id() << "][Error] \"" << func_str
     << "()\": " << str << "\n";
  throw std::runtime_error(ss.str());
}
#define ThrowError(str) ThrowErrorImpl(str, std::string(__func__));

ThreadPool::ThreadPool(int num_threads_in_pool)
    : thread_pool_status_(ThreadPoolStatus::kRun) {
  worker_thread_list_.reserve(num_threads_in_pool);
  for (int index = 0; index < num_threads_in_pool; ++index) {
    PrintInfo("========== thread is generating... ==========");
    worker_thread_list_.emplace_back([this]() { RunProcessForWorkerThread(); });
  }
}

ThreadPool::ThreadPool(const int num_threads_in_pool,
                       const std::vector<int>& cpu_affinity_numbers)
    : thread_pool_status_(ThreadPoolStatus::kRun) {
  if (num_threads_in_pool != static_cast<int>(cpu_affinity_numbers.size()))
    ThrowError("cpu_affinity_numbers.size() != num_threads");

  worker_thread_list_.reserve(num_threads_in_pool);
  for (int index = 0; index < num_threads_in_pool; ++index) {
    PrintInfo("========== Thread is generated... ==========");
    worker_thread_list_.emplace_back([this]() { RunProcessForWorkerThread(); });
    AllocateCpuProcessorForEachThread(worker_thread_list_[index],
                                      cpu_affinity_numbers[index]);
  }
}

ThreadPool::~ThreadPool() {
  thread_pool_status_ = ThreadPoolStatus::kKill;
  WakeUpAllThreads();
  for (auto& worker_thread : worker_thread_list_) {
    PrintInfo("--- JOINING THE THREAD");
    while (!worker_thread.joinable()) WakeUpAllThreads();
    worker_thread.join();
    PrintInfo("Joining is done!");
  }
}

void ThreadPool::RunProcessForWorkerThread() {
  PrintInfo("The thread is initialized.");
  while (true) {
    std::unique_lock<std::mutex> local_lock(mutex_);
    condition_variable_.wait(local_lock, [this]() {
      return (!task_queue_.empty() ||
              (thread_pool_status_ == ThreadPoolStatus::kKill));
    });

    if (thread_pool_status_ == ThreadPoolStatus::kKill) {
      PrintInfo(
          "The thread captured stop sign. The thread is going to join...");
      break;
    }

    // Get a job
    std::function<void()> new_task = std::move(task_queue_.front());
    task_queue_.pop();
    ++num_of_ongoing_tasks_;
    local_lock.unlock();

    // Do the job!
    new_task();

    local_lock.lock();
    --num_of_ongoing_tasks_;
    local_lock.unlock();
  }
  PrintInfo("The thread is end.");
}

void ThreadPool::EnqueueTask(std::function<void()> task) {
  if (thread_pool_status_ == ThreadPoolStatus::kKill) {
    throw std::runtime_error("Thread pool 사용 중지.");
  }

  mutex_.lock();
  task_queue_.push(std::move(task));
  mutex_.unlock();

  WakeUpOneThread();
}

int ThreadPool::GetNumOfOngoingTasks() {
  std::unique_lock<std::mutex> local_lock(mutex_);
  return num_of_ongoing_tasks_;
}

int ThreadPool::GetNumOfQueuedTasks() {
  std::unique_lock<std::mutex> local_lock(mutex_);
  return static_cast<int>(task_queue_.size());
}

int ThreadPool::GetNumOfTotalThreads() {
  std::unique_lock<std::mutex> lock(mutex_);
  return static_cast<int>(worker_thread_list_.size());
}

int ThreadPool::GetNumOfAwaitingThreads() {
  std::unique_lock<std::mutex> lock(mutex_);
  return (static_cast<int>(worker_thread_list_.size()) - num_of_ongoing_tasks_);
}

bool ThreadPool::AllocateCpuProcessorForEachThread(std::thread& input_thread,
                                                   const int cpu_core_index) {
  const int num_max_threads_for_this_cpu =
      static_cast<int>(std::thread::hardware_concurrency());
  if (cpu_core_index >= num_max_threads_for_this_cpu) {
    PrintInfo("Exceed the maximum logical CPU number!");
    return false;
  }

  cpu_set_t cpu_set;
  CPU_ZERO(&cpu_set);
  CPU_SET(cpu_core_index, &cpu_set);
  int result = pthread_setaffinity_np(input_thread.native_handle(),
                                      sizeof(cpu_set_t), &cpu_set);
  if (result != 0) {
    PrintInfo("Error calling pthread_setaffinity_np: " +
              std::to_string(result));
    return false;
  }

  PrintInfo("The thread is confined to CPU Core [" +
            std::to_string(cpu_core_index) + "]");
  return true;
}

void ThreadPool::WakeUpOneThread() { condition_variable_.notify_one(); }

void ThreadPool::WakeUpAllThreads() { condition_variable_.notify_all(); }
