#pragma once
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

namespace PoseGraph {
class ThreadPool {
public:
  ThreadPool(size_t num_threads);
  ~ThreadPool();

  void enqueue(std::function<void()> task);
  void waitForCompletion();

private:
  std::vector<std::thread> workers;
  std::queue<std::function<void()>> tasks;
  std::mutex queue_mutex;
  std::condition_variable condition;
  std::atomic<int> active_tasks;
  bool stop;

  void worker();
};
} // namespace PoseGraph