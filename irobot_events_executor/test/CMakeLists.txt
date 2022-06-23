find_package(ament_cmake_gtest REQUIRED)
find_package(test_msgs REQUIRED)

ament_add_gtest(test_events_executor test_events_executor.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_events_executor)
  ament_target_dependencies(test_events_executor
    "rcl"
    "test_msgs")
  target_link_libraries(test_events_executor ${PROJECT_NAME})
endif()

ament_add_gtest(test_events_queue test_events_queue.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_events_queue)
  target_link_libraries(test_events_queue ${PROJECT_NAME})
endif()

ament_add_gtest(test_executors test_executors.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
  TIMEOUT 180)
if(TARGET test_executors)
  ament_target_dependencies(test_executors
    "rcl"
    "test_msgs")
  target_link_libraries(test_executors ${PROJECT_NAME})
endif()
