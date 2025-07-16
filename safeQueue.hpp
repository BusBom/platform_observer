#ifndef SAFE_QUEUE_HPP
#define SAFE_QUEUE_HPP

#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class ThreadSafeQueue
{
public:
    // 큐에 아이템을 추가 (생산자 호출)
    void push(T item)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(std::move(item));
        cond_.notify_one(); // 대기 중인 소비자 스레드 하나를 깨움
    }

    // 큐에서 아이템을 꺼내옴 (소비자 호출)
    // 큐가 비어있으면 아이템이 들어올 때까지 대기
    void wait_and_pop(T &item)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this]
                   { return !queue_.empty(); });
        item = std::move(queue_.front());
        queue_.pop();
    }

    // 큐에서 아이템을 꺼내려고 시도 (소비자 호출)
    // 큐가 비어있으면 기다리지 않고 즉시 false 반환
    bool try_pop(T &item)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty())
        {
            return false;
        }
        item = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    size_t size()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
};

#endif