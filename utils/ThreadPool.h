#ifndef THREADPOOL_H
#define THREADPOOL_H

#include <atomic>

/**
 * @brief A basic ThreadPool class
 */
class ThreadPool
{
public:
   ThreadPool();
private:
   std::atomic_bool done;
};

#endif // THREADPOOL_H
