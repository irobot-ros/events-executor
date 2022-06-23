// Copyright 2022 iRobot Corporation. All Rights Reserved

#ifndef RCLCPP__EXECUTORS__EVENTS_EXECUTOR__EVENTS_EXECUTOR_EVENT_TYPES_HPP_
#define RCLCPP__EXECUTORS__EVENTS_EXECUTOR__EVENTS_EXECUTOR_EVENT_TYPES_HPP_

namespace rclcpp
{
namespace executors
{

enum ExecutorEventType
{
  SUBSCRIPTION_EVENT,
  SERVICE_EVENT,
  CLIENT_EVENT,
  WAITABLE_EVENT
};

struct ExecutorEvent
{
  const void * exec_entity_id;
  int gen_entity_id;
  ExecutorEventType type;
  size_t num_events;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__EVENTS_EXECUTOR__EVENTS_EXECUTOR_EVENT_TYPES_HPP_
