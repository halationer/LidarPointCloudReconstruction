#include"ThreadPool.h"

#include<iostream>
#include<shared_mutex>

namespace Tools {

ThreadPool::ThreadPool(size_t size) {
    sem_init(&waitSemaphore, 0, 0);
    for(size_t i = 0; i < size; ++i) {
        threads.emplace_back(new std::thread([this](){this->DoTask();}));
    }
}

ThreadPool::~ThreadPool() {
    shutdown = true;
    // semaphore tells thread to shutdown
    for(auto& thread : threads) {
        sem_post(&waitSemaphore);
    }
    for(auto& thread : threads) {
        thread->join();
    }
    sem_destroy(&waitSemaphore);
}

void ThreadPool::AddTask(TaskPtr task) {

    std::unique_lock<std::mutex> lock(waitListLock);
    waitList.push(task);
    // semaphore tells the task is comming
    sem_post(&waitSemaphore);
}

TaskPtr ThreadPool::AddTask(std::function<void(void)> func) {

    TaskPtr task(new Task(func));
	AddTask(task);
    return task;
}

void ThreadPool::DoTask() {

    while(!shutdown) {

        std::unique_lock<std::mutex> lock(waitListLock);
        if(waitList.size() > 0) {
            
            auto function = waitList.front();
            waitList.pop();
            lock.unlock();

            function->main();
        }
        else {

            lock.unlock();

            sem_wait(&waitSemaphore);
        }

    }
}

}