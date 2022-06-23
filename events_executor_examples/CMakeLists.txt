cmake_minimum_required(VERSION 3.5)
project(events_executor_examples)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(irobot_events_executor REQUIRED)
find_package(irobot_lock_free_events_queue REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(hello_events_executor src/hello_events_executor.cpp)
ament_target_dependencies(hello_events_executor
  "irobot_events_executor"
  "rclcpp"
  "std_msgs"
)

add_executable(lock_free_queue src/lock_free_queue.cpp)
ament_target_dependencies(lock_free_queue
  "irobot_events_executor"
  "irobot_lock_free_events_queue"
  "rclcpp"
  "std_msgs"
)

install(TARGETS
  hello_events_executor
  lock_free_queue
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
