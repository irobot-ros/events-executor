// Copyright 2022 iRobot Corporation. All Rights Reserved

#ifndef RCLCPP__EXECUTORS__EVENTS_EXECUTOR__EVENTS_EXECUTOR_NOTIFY_WAITABLE_HPP_
#define RCLCPP__EXECUTORS__EVENTS_EXECUTOR__EVENTS_EXECUTOR_NOTIFY_WAITABLE_HPP_

#include <list>
#include <memory>

#include "rcl/guard_condition.h"
#include "rclcpp/executors/events_executor/event_waitable.hpp"

namespace rclcpp
{
namespace executors
{

/**
 * @brief This class provides an EventWaitable that allows to
 * wake up an EventsExecutor when a guard condition is notified.
 */
class EventsExecutorNotifyWaitable final : public EventWaitable
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(EventsExecutorNotifyWaitable)

  // Constructor
  RCLCPP_PUBLIC
  EventsExecutorNotifyWaitable() = default;

  // Destructor
  RCLCPP_PUBLIC
  virtual ~EventsExecutorNotifyWaitable()
  {
    for (auto & gc : notify_guard_conditions_) {
      gc->set_on_trigger_callback(nullptr);
    }
  }

  // The function is a no-op, since we only care of waking up the executor
  RCLCPP_PUBLIC
  void
  execute(std::shared_ptr<void> & data) override
  {
    (void)data;
  }

  RCLCPP_PUBLIC
  void
  add_guard_condition(rclcpp::GuardCondition * guard_condition)
  {
    notify_guard_conditions_.push_back(guard_condition);
  }

  RCLCPP_PUBLIC
  void
  set_on_ready_callback(std::function<void(size_t, int)> callback) override
  {
    // The second argument of the callback should identify which guard condition
    // triggered the event. However it's not relevant here as we only
    // care about waking up the executor, so we pass a 0.
    auto gc_callback = [callback](size_t count) {
        callback(count, 0);
      };

    for (auto gc : notify_guard_conditions_) {
      gc->set_on_trigger_callback(gc_callback);
    }
  }

  RCLCPP_PUBLIC
  std::shared_ptr<void>
  take_data() override
  {
    // This waitable doesn't handle any data
    return nullptr;
  }

  RCLCPP_PUBLIC
  std::shared_ptr<void>
  take_data_by_entity_id(size_t id) override
  {
    (void) id;
    return take_data();
  }

private:
  std::list<rclcpp::GuardCondition *> notify_guard_conditions_;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__EVENTS_EXECUTOR__EVENTS_EXECUTOR_NOTIFY_WAITABLE_HPP_
