#include "threadpool.hpp"
#include <iostream>

namespace PoseGraph {
ThreadPool::ThreadPool(size_t num_threads) : stop(false), active_tasks(0) {
  for (size_t i = 0; i < num_threads; ++i) {
    workers.emplace_back(&ThreadPool::worker, this);
  }
}

ThreadPool::~ThreadPool() {
  {
    std::unique_lock<std::mutex> lock(queue_mutex);
    stop = true;
  }
  condition.notify_all();
  for (auto &worker : workers) {
    worker.join();
  }
}

void ThreadPool::enqueue(std::function<void()> task) {
  {
    std::unique_lock<std::mutex> lock(queue_mutex);
    tasks.push(std::move(task));
    active_tasks++;
    std::cout << "enqueued!\n";
  }
  condition.notify_all();
}

void ThreadPool::waitForCompletion() {
  std::unique_lock<std::mutex> lock(queue_mutex);
  condition.wait(lock, [this] { return tasks.empty() && active_tasks == 0; });
}

void ThreadPool::worker() {
  while (true) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      condition.wait(lock, [this] { return stop || !tasks.empty(); });
      if (stop && tasks.empty()) {
        return;
      }
      task = std::move(tasks.front());
      tasks.pop();
    }
    task();
    active_tasks--;
    condition.notify_all();
  }
}
} // namespace PoseGraph