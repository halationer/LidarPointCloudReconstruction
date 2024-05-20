#ifndef BUFFER_LIMIT_THREAD_POOL
#define BUFFER_LIMIT_THREAD_POOL

#include "ThreadPool.h"

namespace Tools {

// if the buffer is full, drop the old tasks
class BufferLimitThreadPool : public ThreadPool {

private:
    // buffer size of tasks
    size_t bufferSize;

public:
    BufferLimitThreadPool(size_t poolSize, size_t bufferSize);
    virtual ~BufferLimitThreadPool();
    virtual void AddTask(TaskPtr task) override;
    // virtual TaskPtr AddTask(std::function<void(void)> func) override; // same as ThreadPool
};

}

#endif