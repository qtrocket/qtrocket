#ifndef THREADPOOL_H
#define THREADPOOL_H

/// \cond
// C headers
// C++ headers
#include <atomic>
#include <functional>
#include <thread>
#include <vector>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "utils/TSQueue.h"


/**
 * @brief A basic ThreadPool class
 */
class ThreadPool
{
public:
   ThreadPool();
   ~ThreadPool();

   template<typename FunctionType>
   void submit(FunctionType f)
   {
      q.push(std::function<void()>(f));
   }

private:
   class JoinThreads
   {
   public:
      explicit JoinThreads(std::vector<std::thread>& inThreads)
         : threads(inThreads)
      {}

      ~JoinThreads()
      {
         for(auto& i : threads)
         {
            if(i.joinable())
            {
               i.join();
            }
         }
      }
   private:
      std::vector<std::thread>& threads;
   };

   std::atomic_bool done;
   TSQueue<std::function<void()>> q;
   std::vector<std::thread> threads;

   JoinThreads joiner;

   void worker();

};



#endif // THREADPOOL_H
