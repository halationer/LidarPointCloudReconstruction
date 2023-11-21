#ifndef THREAD_POOL
#define THREAD_POOL

#include<queue>
#include<functional>
#include<thread>
#include<vector>
#include<memory>
#include<mutex>

#include<future>
#include<atomic>
#include<semaphore.h>

namespace Tools {

class Task {

private:
    friend class ThreadPool;
    std::function<void(void)> main;
    std::promise<void> result;
    std::atomic<bool> finished = false;

public:
    Task(std::function<void(void)> func){
        main = [this, func]() {
            func();
            result.set_value();
        };
    }
    void Join() {
        if(finished) return;
        result.get_future().wait();
        finished = true;
    }
};

typedef std::shared_ptr<Task> TaskPtr;

class ThreadPool {

private:
    sem_t waitSemaphore;
    std::atomic<bool> shutdown = false;
    std::mutex waitListLock;
    std::queue<TaskPtr> waitList;
    std::vector<std::shared_ptr<std::thread>> threads;

public:
    ThreadPool(size_t size);
    ~ThreadPool();
    void AddTask(TaskPtr task);
    TaskPtr AddTask(std::function<void(void)> func); 

private:
    void DoTask();
};

}

#endif