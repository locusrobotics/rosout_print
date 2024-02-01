/**
 *  \file       thread_safe_queue.h
 *  \author     Jim Won <jwon@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2017, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 */
#ifndef ROSOUT_PRINT_THREAD_SAFE_QUEUE_H
#define ROSOUT_PRINT_THREAD_SAFE_QUEUE_H

#include <boost/thread/condition_variable.hpp>
#include <queue>

template <typename T>
class ThreadSafeQueue
{
private:
    boost::condition_variable cond_;
    mutable boost::mutex mutex_;
    std::queue<T> queue_;

public:
    ThreadSafeQueue()
    {
    }
    void push(T value)
    {
        boost::lock_guard<boost::mutex> lock(mutex_);
        queue_.push(value);
        cond_.notify_one();
    }
    void waitAndPop(T &value)
    {
        boost::unique_lock<boost::mutex> lock(mutex_);
        auto queue_not_empty = [this] { return !queue_.empty(); };
        cond_.wait(lock, queue_not_empty);
        value = queue_.front();
        queue_.pop();
    }
    bool tryPop(T &value)
    {
        boost::lock_guard<boost::mutex> lock(mutex_);
        if (queue_.empty())
        {
            return false;
        }
        value = queue_.front();
        queue_.pop();
        return true;
    }
    bool empty() const
    {
        boost::lock_guard<boost::mutex> lock(mutex_);
        return queue_.empty();
    }
};

#endif  // ROSOUT_PRINT_THREAD_SAFE_QUEUE_H
