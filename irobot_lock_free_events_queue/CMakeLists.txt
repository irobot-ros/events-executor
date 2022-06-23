cmake_minimum_required(VERSION 3.5)
project(irobot_lock_free_events_queue)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(irobot_events_executor REQUIRED)

add_library(irobot_lock_free_events_queue INTERFACE)

target_include_directories(irobot_lock_free_events_queue
  INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(irobot_lock_free_events_queue INTERFACE
  "irobot_events_executor"
  "rclcpp"
)

install(TARGETS
  irobot_lock_free_events_queue
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
#ament_export_libraries(${PROJECT_NAME})
# Export modern CMake targets
#ament_export_targets(${PROJECT_NAME})

# Specific order: dependents before dependencies
ament_export_dependencies(irobot_events_executor)
ament_export_dependencies(rclcpp)
ament_package()
