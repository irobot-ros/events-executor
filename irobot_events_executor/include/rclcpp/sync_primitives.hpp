//  Copyright (C) 2022, iRobot Corporation.

#ifndef RCLCPP__SYNC_PRIMITIVES_HPP_
#define RCLCPP__SYNC_PRIMITIVES_HPP_

#include <cstdint>
#include <sys/time.h>
#include <pthread.h>

namespace rclcpp
{

class RecursiveMutex
{
public:
  RecursiveMutex()
  {
    pthread_mutexattr_t attr;
    int ret = pthread_mutexattr_init(&attr);
    assert(ret == 0);
    ret = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    assert(ret == 0);
    ret = pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
    assert(ret == 0);
    ret = pthread_mutex_init(&m_mutex, &attr);
    assert(ret == 0);
    ret = pthread_mutexattr_destroy(&attr);
    assert(ret == 0);
    (void)ret;
  }

  // Not copyable or movable
  RecursiveMutex(const RecursiveMutex &) = delete;
  RecursiveMutex(RecursiveMutex &&) = delete;
  RecursiveMutex &operator=(const RecursiveMutex &) = delete;
  RecursiveMutex &operator=(RecursiveMutex &&) = delete;

  ~RecursiveMutex()
  {
    int ret = pthread_mutex_destroy(&m_mutex);
    assert(ret == 0);
    (void)ret;
  }

  void lock()
  {
    int ret = pthread_mutex_lock(&m_mutex);
    assert(ret == 0);
    (void)ret;
  }

  void unlock()
  {
    int ret = pthread_mutex_unlock(&m_mutex);
    assert(ret == 0);
    (void)ret;
  }

  pthread_mutex_t& native_handle()
  {
    return m_mutex;
  }

private:
  pthread_mutex_t m_mutex{};
};

class Lock
{
public:
  explicit Lock(RecursiveMutex& mutex)
  : m_mutex(&mutex), m_lock_cnt(1)
  {
    m_mutex->lock();
  }

  // Not copyable
  Lock(const Lock&) = delete;
  Lock &operator=(const Lock &) = delete;

  /** Unlock the mutex associated with this lock. */
  ~Lock()
  {
    while (m_lock_cnt > 0) {
        m_mutex->unlock();
        --m_lock_cnt;
    }
  }

private:
  RecursiveMutex *m_mutex;
  int m_lock_cnt;
};

class ConditionVariable
{
public:
  ConditionVariable()
  {
    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    int ret = pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    assert(ret == 0);
    ret = pthread_cond_init(&m_cond, &attr);
    assert(ret == 0);
    ret = pthread_condattr_destroy(&attr);
    assert(ret == 0);
    (void)ret;
  }

  ~ConditionVariable()
  {
    int ret = pthread_cond_destroy(&m_cond);
    assert(ret == 0);
    (void)ret;
  }

  void notify_one()
  {
    pthread_cond_signal(&m_cond);
  }

  void wait(RecursiveMutex& mutex)
  {
    pthread_cond_wait(&this->m_cond, &mutex.native_handle());
  }

  bool wait(RecursiveMutex& mutex, uint64_t nanoseconds)
  {
    // Determine the absolute system time when timeout occurs.
    timespec start {};
    clock_gettime(CLOCK_MONOTONIC, &start);

    static constexpr uint64_t NS_PER_SECONDS = 1000000000;

    timespec wait {};
    wait.tv_nsec = nanoseconds % NS_PER_SECONDS;
    wait.tv_sec = nanoseconds / NS_PER_SECONDS;

    timespec timeout {};
    const uint64_t nanoseconds_sum = start.tv_nsec + wait.tv_nsec;
    timeout.tv_nsec = nanoseconds_sum % NS_PER_SECONDS;
    timeout.tv_sec = start.tv_sec + wait.tv_sec + nanoseconds_sum / NS_PER_SECONDS;

    return pthread_cond_timedwait(&this->m_cond, &mutex.native_handle(), &timeout) == 0;
  }

private:
  pthread_cond_t m_cond{};
};

}  // namespace rclcpp

#endif  // RCLCPP__SYNC_PRIMITIVES_HPP_
