// Copyright 2022 iRobot Corporation. All Rights Reserved

#include <memory>
#include <thread>
#include <chrono>


#include "rclcpp/executors/events_executor/events_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "example_nodes.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto publisher_node = std::make_shared<MinimalPublisher>();
  auto subscriber_node = std::make_shared<MinimalSubscriber>();

  auto executor = std::make_shared<rclcpp::executors::EventsExecutor>();

  executor->add_node(publisher_node);
  executor->add_node(subscriber_node);

  std::thread spinner([&](){executor->spin();});
  
  std::this_thread::sleep_for(std::chrono::seconds(2));
  std::cout << "cancel timer" << std::endl;
  publisher_node->cancel_timer();

  std::this_thread::sleep_for(std::chrono::seconds(2));
  std::cout << "reset timer" << std::endl;
  publisher_node->reset_timer();

  std::this_thread::sleep_for(std::chrono::seconds(2));
  std::cout << "cancel timer" << std::endl;
  publisher_node->cancel_timer();
  rclcpp::shutdown();
  spinner.join();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  return 0;
}
