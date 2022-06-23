// Copyright 2022 iRobot Corporation. All Rights Reserved

#ifndef RCLCPP__EXECUTORS__EVENTS_EXECUTOR__EVENT_WAITABLE_HPP_
#define RCLCPP__EXECUTORS__EVENTS_EXECUTOR__EVENT_WAITABLE_HPP_

#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace executors
{

/**
 * @brief This class provides a wrapper around the waitable object, that is
 * meant to be used with the EventsExecutor.
 * The waitset related methods are stubbed out as they should not be called.
 * This class is abstract as the execute method of rclcpp::Waitable is not implemented.
 * Nodes who want to implement a custom EventWaitable, can derive from this class and override
 * the execute method.
 */
class EventWaitable : public rclcpp::Waitable
{
public:
  // Constructor
  RCLCPP_PUBLIC
  EventWaitable() = default;

  // Destructor
  RCLCPP_PUBLIC
  virtual ~EventWaitable() = default;

  // Stub API: not used by EventsExecutor
  RCLCPP_PUBLIC
  bool
  is_ready(rcl_wait_set_t * wait_set) override final
  {
    (void)wait_set;
    throw std::runtime_error("EventWaitable can't be checked if it's ready");
    return false;
  }

  // Stub API: not used by EventsExecutor
  RCLCPP_PUBLIC
  void
  add_to_wait_set(rcl_wait_set_t * wait_set) override final
  {
    (void)wait_set;
    throw std::runtime_error("EventWaitable can't be added to a wait_set");
  }
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__EVENTS_EXECUTOR__EVENT_WAITABLE_HPP_
