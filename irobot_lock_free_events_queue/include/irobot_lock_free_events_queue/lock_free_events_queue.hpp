// Copyright 2022 iRobot Corporation. All Rights Reserved

#ifndef IROBOT_LOCK_FREE_EVENTS_QUEUE__LOCK_FREE_EVENTS_QUEUE_HPP_
#define IROBOT_LOCK_FREE_EVENTS_QUEUE__LOCK_FREE_EVENTS_QUEUE_HPP_

#include "irobot_lock_free_events_queue/concurrent_queue/blockingconcurrentqueue.h"
#include "rclcpp/executors/events_executor/events_queue.hpp"

/**
 * @brief This class implements an EventsQueue as a simple wrapper around
 * the blockingconcurrentqueue.h
 * See https://github.com/cameron314/concurrentqueue
 * It does not perform any checks about the size of queue, which can grow
 * unbounded without being pruned. (there are options about this, read the docs).
 * This implementation is lock free, producers and consumers can use the queue
 * concurrently without the need for synchronization mechanisms. The use of this
 * queue aims to fix the issue of publishers being blocked by the executor extracting
 * events from the queue in a different thread, causing expensive mutex contention.
 */
class LockFreeEventsQueue : public rclcpp::executors::EventsQueue
{
public:
  RCLCPP_PUBLIC
  ~LockFreeEventsQueue() override
  {
    // It's important that all threads have finished using the queue
    // and the memory effects have fully propagated, before it is destructed.
    // Consume all events
    rclcpp::executors::ExecutorEvent event;
    while (event_queue_.try_dequeue(event)) {}
  }

  /**
   * @brief enqueue event into the queue
   * @param event The event to enqueue into the queue
   */
  RCLCPP_PUBLIC
  void
  enqueue(const rclcpp::executors::ExecutorEvent & event) override
  {
    rclcpp::executors::ExecutorEvent single_event = event;
    single_event.num_events = 1;
    for (size_t ev = 0; ev < event.num_events; ev++) {
      event_queue_.enqueue(single_event);
    }
  }

  /**
   * @brief waits for an event until timeout
   * @return true if event, false if timeout
   */
  RCLCPP_PUBLIC
  bool
  dequeue(
    rclcpp::executors::ExecutorEvent & event,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max()) override
  {    
    if (timeout != std::chrono::nanoseconds::max()) {
      return event_queue_.wait_dequeue_timed(event, timeout);
    }

    // If no timeout specified, just wait for an event to arrive
    event_queue_.wait_dequeue(event);
    return true;
  }

  /**
   * @brief Test whether queue is empty
   * @return true if the queue's size is 0, false otherwise.
   */
  RCLCPP_PUBLIC
  bool
  empty() const override
  {
    return event_queue_.size_approx() == 0;
  }

  /**
   * @brief Returns the number of elements in the queue.
   * This estimate is only accurate if the queue has completely
   * stabilized before it is called
   * @return the number of elements in the queue.
   */
  RCLCPP_PUBLIC
  size_t
  size() const override
  {
    return event_queue_.size_approx();
  }

private:
  moodycamel::BlockingConcurrentQueue<rclcpp::executors::ExecutorEvent> event_queue_;
};

#endif  // IROBOT_LOCK_FREE_EVENTS_QUEUE__LOCK_FREE_EVENTS_QUEUE_HPP_
