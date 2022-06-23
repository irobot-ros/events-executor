cmake_minimum_required(VERSION 3.5)
project(irobot_events_executor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)

add_library(irobot_events_executor SHARED)

target_sources(irobot_events_executor
  PRIVATE
  src/rclcpp/executors/events_executor/events_executor_entities_collector.cpp
  src/rclcpp/executors/events_executor/events_executor.cpp
  src/rclcpp/executors/entities_collector_base.cpp
  src/rclcpp/timers_manager.cpp
)

target_include_directories(irobot_events_executor
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(irobot_events_executor
  "rclcpp"
  "rcpputils"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

install(TARGETS
  irobot_events_executor EXPORT irobot_events_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

# Specific order: dependents before dependencies
ament_export_dependencies(rclcpp)
ament_export_dependencies(rcpputils)
ament_package()
