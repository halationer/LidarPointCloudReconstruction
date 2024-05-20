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
    std::atomic<bool> shutdown = false;
    std::vector<std::shared_ptr<std::thread>> threads;
    void DoTask();

public:
    ThreadPool(size_t size);
    virtual ~ThreadPool();
    virtual void AddTask(TaskPtr task);
    virtual TaskPtr AddTask(std::function<void(void)> func); 

protected:
    sem_t waitSemaphore;
    std::mutex waitListLock;
    std::queue<TaskPtr> waitList;
    void WakeUpAnyThread();
};

}

#endif