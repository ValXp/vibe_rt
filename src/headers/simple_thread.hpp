#ifndef SIMPLE_THREAD_HPP_
#define SIMPLE_THREAD_HPP_

#include <pthread.h>

template <class W, class R>
class Thread {
public:
    Thread() : work_container(this) {}

    // Implementer must delete work in DoWork when done.
    virtual R* DoWork(W* work) = 0;

    // We take ownership of work.
    void Run(W* work) {
        work_container.work = work;
        pthread_create(&_thread, NULL, &Thread::work_fn, static_cast<void*>(&work_container));
    }

    void Join() { pthread_join(_thread, NULL); }

private:
    class WorkContainer {
    public:
        explicit WorkContainer(Thread* thread) : work(nullptr), thread(thread) {}
        W* work;
        Thread* thread;
    };
    WorkContainer work_container;
    pthread_t _thread{};

    static void* work_fn(void *ptr) {
        WorkContainer* work_container = static_cast<WorkContainer*>(ptr);
        void* result = static_cast<void *>(work_container->thread->DoWork(work_container->work));
        work_container->work = nullptr; // ownership transferred to DoWork
        return result;
    }
};

#endif // SIMPLE_THREAD_HPP_

