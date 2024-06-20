#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

class ThreadPool {
  public:
    ThreadPool(int numThreads) : stop(false) {
        for (int i = 0; i < numThreads; ++i) {
            workers.emplace_back([this] {
                while (true) {
                    std::function<void()> task;

                    {
                        std::unique_lock<std::mutex> lock(queueMutex);
                        condition.wait(lock, [this] { return stop || !tasks.empty(); });

                        if (stop && tasks.empty()) {
                            return;
                        }

                        task = std::move(tasks.front());
                        tasks.pop();
                    }

                    task();
                }
            });
        }
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            stop = true;
        }

        condition.notify_all();

        for (std::thread &worker : workers) {
            worker.join();
        }
    }

    template <class F, class... Args>
    void enqueue(F &&f, Args &&...args) {
        {
            auto task = std::bind(std::forward<F>(f), std::forward<Args>(args)...);
            std::unique_lock<std::mutex> lock(queueMutex);
            tasks.emplace(task);
        }

        condition.notify_one();
    }

    // template <class F>
    // void enqueue(F &&f) {
    //     {
    //         std::unique_lock<std::mutex> lock(queueMutex);
    //         if (stop) {
    //             throw std::runtime_error("enqueue on stopped ThreadPool");
    //         }

    //         tasks.emplace(std::forward<F>(f));
    //     }

    //     condition.notify_one();
    // }

    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;

  private:
    // std::vector<std::thread> workers;
    // std::queue<std::function<void()>> tasks;
    std::mutex queueMutex;
    std::condition_variable condition;
    bool stop;
};