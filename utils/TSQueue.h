#ifndef TSQUEUE_H
#define TSQUEUE_H

/// \cond
// C headers
// C++ headers
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>

// 3rd party headers
/// \endcond

/**
 * @brief The TSQueue class is a very basic thread-safe queue
 */
template<typename T>
class TSQueue
{
public:
   TSQueue()
      : mtx(),
        q(),
        cv()
   {}

   void push(T newVal)
   {
      std::lock_guard<std::mutex> lck(mtx);
      q.push(newVal);
      cv.notify_one();
   }

   void waitAndPop(T& val)
   {
      std::unique_lock<std::mutex> lck(mtx);
      cv.wait(lck, [this]{return !q.empty(); });
      val = std::move(q.front());
      q.pop();
   }

   std::shared_ptr<T> waitAndPop()
   {
      std::unique_lock<std::mutex> lck(mtx);
      cv.wait(lck, [this] { return !q.empty(); });
      std::shared_ptr<T> res(std::make_shared<T>(std::move(q.front())));
      q.pop();
      return res;
   }

   bool tryPop(T& val)
   {
      std::unique_lock<std::mutex> lck(mtx);
      if(q.empty())
      {
         return false;
      }
      val = std::move(q.front());
      q.pop();
      return true;
   }

   std::shared_ptr<T> tryPop()
   {
      std::unique_lock<std::mutex> lck(mtx);
      if(q.empty())
      {
         return std::shared_ptr<T>(); // nullptr
      }
      std::shared_ptr<T> retVal(std::move(q.front()));
      q.pop();
      return retVal;
   }

   bool empty() const
   {
      std::lock_guard<std::mutex> lck(mtx);
      return q.empty();
   }


private:
   mutable std::mutex mtx;
   std::queue<T> q;
   std::condition_variable cv;
};

#endif // TSQUEUE_H
