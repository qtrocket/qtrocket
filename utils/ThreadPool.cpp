#include "ThreadPool.h"

#include <cstdint>

ThreadPool::ThreadPool()
   : done(false),
     q(),
     threads(),
     joiner(threads)
{
   const std::size_t threadCount = std::thread::hardware_concurrency();

   try
   {
      for(size_t i = 0; i < threadCount; ++i)
      {
         threads.push_back(std::thread(&ThreadPool::worker, this));
      }
   }
   catch(...)
   {
      done = true;
      throw;
   }
}

ThreadPool::~ThreadPool()
{
   done = true;
}
void ThreadPool::worker()
{
   while(!done)
   {
      std::function<void()> task;
      if(q.tryPop(task))
      {
         task();
      }
      else
      {
         std::this_thread::yield();
      }
   }
}
