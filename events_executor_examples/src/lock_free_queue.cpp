// Copyright 2022 iRobot Corporation. All Rights Reserved

#include <memory>

#include "irobot_lock_free_events_queue/lock_free_events_queue.hpp"

#include "rclcpp/executors/events_executor/events_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "example_nodes.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto publisher_node = std::make_shared<MinimalPublisher>("A");
  auto subscriber_node = std::make_shared<MinimalSubscriber>();

  auto lock_free_queue = std::make_unique<LockFreeEventsQueue>();
  auto executor = std::make_shared<rclcpp::executors::EventsExecutor>(std::move(lock_free_queue));

  executor->add_node(publisher_node);
  executor->add_node(subscriber_node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
