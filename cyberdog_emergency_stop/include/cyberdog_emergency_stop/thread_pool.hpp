// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CYBERDOG_EMERGENCY_STOP__THREAD_POOL_HPP_
#define CYBERDOG_EMERGENCY_STOP__THREAD_POOL_HPP_

#include <iostream>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>
#include <utility>
#include <memory>

namespace cyberdog
{

namespace thread_pool
{
class ThreadPool
{
public:
  explicit ThreadPool(size_t numThreads)
  {
    for (size_t i = 0; i < numThreads; ++i) {
      // 工程线程 执行 任务队列中的任务
      threads_.emplace_back(
        [this]() {
          while (true) {
            std::function<void()> task;
            {
              // 任务队列为空时， 阻塞工作线程
              std::unique_lock<std::mutex> lock(this->queue_mutex_);
              // cond_var.wait被唤醒后还要多判断一个 bool 变量，一定要条件成立才会结束等待，否则继续等待。可避免某些情景下的虚假唤醒  //  NOLINT
              // template<typename _Predicate>
              // void wait(unique_lock<mutex>& __lock, _Predicate __p)
              this->condition_.wait(lock, [this] {return this->stop_ || !this->tasks_.empty();});

              if (this->stop_ && this->tasks_.empty()) {
                return;
              }
              // 取从任务队列中 取出 任务
              task = std::move(this->tasks_.front());

              // 将取出的任务 从任务队列中 删除
              this->tasks_.pop();
            }

            // 执行任务
            task();
          }
        });
    }
  }

  // 线程池提供的API，将 一个个 待处理的任务 添加到 任务队列
  template<class F, class ... Args>
  // 该函数的参数是 一个可调用对象f 和 它的参数args，返回值是一个std::future对象，表示任务的执行结果
  auto enqueue(F && f, Args && ... args)->std::future<typename std::result_of<F(Args...)>::type>
  {
    // 使用std::result_of模板类获取f的返回值类型
    using return_type = typename std::result_of<F(Args...)>::type;

    // 使用std::packaged_task将f和args绑定成一个任务task
    auto task =
      std::make_shared<std::packaged_task<return_type()>>(
      std::bind(
        std::forward<F>(f),
        std::forward<Args>(args)...));
    std::future<return_type> res = task->get_future();

    // 将task加入到任务队列tasks_中，并通过condition_唤醒一个等待的线程去执行任务
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);

      if (stop_) {
        throw std::runtime_error("enqueue on stopped ThreadPool");
      }

      tasks_.emplace([task]() {(*task)();});
    }
    condition_.notify_one();

    // 最后返回std::future对象，表示该任务的执行结果
    return res;
  }

  ~ThreadPool()
  {
    // 首先获取一个互斥锁，然后将停止标志设置为true，表示线程池需要停止。
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      stop_ = true;
    }

    // 通过条件变量通知所有等待的线程，让它们退出等待状态。
    condition_.notify_all();

    // 最后，使用一个循环遍历所有线程，
    // 并调用join()函数等待线程结束。这样可以确保所有线程都已经退出，从而保证线程池的安全退出。
    for (std::thread & thread : threads_) {
      thread.join();
    }
  }

private:
  // 工作线程
  std::vector<std::thread> threads_;

  // 任务队列
  std::queue<std::function<void()>> tasks_;

  std::mutex queue_mutex_;
  std::condition_variable condition_;
  bool stop_ {false};
};
}  // namespace thread_pool
}  // namespace cyberdog

#endif  // CYBERDOG_EMERGENCY_STOP__THREAD_POOL_HPP_
