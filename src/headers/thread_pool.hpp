// Minimal C++17 thread pool for void() jobs.
#ifndef THREAD_POOL_HPP_
#define THREAD_POOL_HPP_

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

class ThreadPool {
public:
    explicit ThreadPool(std::size_t thread_count = std::thread::hardware_concurrency())
        : stop_(false), tasks_in_flight_(0) {
        if (thread_count == 0) thread_count = 1;
        workers_.reserve(thread_count);
        for (std::size_t i = 0; i < thread_count; ++i) {
            workers_.emplace_back([this]() { this->worker_loop_(); });
        }
    }

    ~ThreadPool() {
        wait();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& t : workers_) {
            if (t.joinable()) t.join();
        }
    }

    void enqueue(std::function<void()> job) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tasks_.push(std::move(job));
        }
        cv_.notify_one();
    }

    void wait() {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_done_.wait(lock, [this]() { return tasks_.empty() && tasks_in_flight_ == 0; });
    }

    // Clear all queued (not yet started) tasks. Running tasks are unaffected.
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::queue<std::function<void()>> empty;
        std::swap(tasks_, empty);
        // don't notify cv_done_ here; running tasks will update as they finish
    }

    std::size_t thread_count() const {
        return workers_.size();
    }

private:
    void worker_loop_() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this]() { return stop_ || !tasks_.empty(); });
                if (stop_ && tasks_.empty()) return;
                task = std::move(tasks_.front());
                tasks_.pop();
                ++tasks_in_flight_;
            }

            task();

            {
                std::lock_guard<std::mutex> lock(mutex_);
                --tasks_in_flight_;
                if (tasks_.empty() && tasks_in_flight_ == 0) {
                    cv_done_.notify_all();
                }
            }
        }
    }

    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::condition_variable cv_done_;
    bool stop_;
    std::size_t tasks_in_flight_;
};

#endif // THREAD_POOL_HPP_
