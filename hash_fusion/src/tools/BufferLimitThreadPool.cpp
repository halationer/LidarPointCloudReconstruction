#include"BufferLimitThreadPool.h"

#include<iostream>
#include<shared_mutex>

namespace Tools {

BufferLimitThreadPool::BufferLimitThreadPool(size_t poolSize, size_t bufferSize) : ThreadPool(poolSize) {

    this->bufferSize = bufferSize;
}

BufferLimitThreadPool::~BufferLimitThreadPool() {

    std::cout << "## BUFFER LIMIT THREAD POOL ALREADY RELEASED ##" << std::endl;
}

void BufferLimitThreadPool::AddTask(TaskPtr task) {

    std::unique_lock<std::mutex> lock(waitListLock);
    waitList.push(task);
    while(waitList.size() > this->bufferSize) {
        waitList.pop();
    }
    WakeUpAnyThread();
}

}