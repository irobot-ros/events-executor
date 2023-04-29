# events-executor

## The events-executor is now merged in rclcpp: https://github.com/ros2/rclcpp/pull/2155 and this repository is now deprecated


This repository contains a C++ implementation of the `EventsExecutor` designed by iRobot.
It is a new type of executor compatible with the ROS 2 `rclcpp` library.

The `EventsExecutor` design is based on the following principles:

 1. You don't pay for what you don't use.
 2. The abstractions between the application and the RMW should have minimal overhead.
 3. Extensibility.

![Executors Comparison](executors-comparison.png)

The following data have been measured using the iRobot [ros2-performance](https://github.com/irobot-ros/ros2-performance) framework and show how, for a 10 nodes system, the `EventsExecutor` reduces both CPU and latency by 75% with respect to the default `SingleThreadedExecutor` and by 50% with respect to the `StaticSingleThreadedExecutor`.

These results have been obtained using the default implementation of the `EventsExecutor` events queue, i.e. an `std::queue`.
The extensibility of the `EventsExecutor` comes from the fact that this core components can be re-implemented by the users.
For example in this repository we also include an extension that uses a lock-free queue, based on this great [concurrent queue](https://github.com/cameron314/concurrentqueue) implementation. 
Other extensions would allow to bound the queue or enforce deterministic execution constraints.

To know more about the design of the Events Executor, refer to https://github.com/ros2/design/pull/305.

## Known bugs and limitations

The executor has some known bugs and limitations when used together with standard ROS 2 core libraries.
These are described here, together with how to fix them.

 - The executor is not notified when a ROS 2 timer is reset. This means that the timer may not be triggered anymore. To fix this bug:
    1. Use the `humble-future` branch of this repository
    2. Cherry-pick https://github.com/ros2/rclcpp/pull/1979
    3. Cherry-pick https://github.com/ros2/rcl/pull/995
 - Enabling Intra-Process Optimization in rclcpp may result in [a runtime exception](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/include/rclcpp/experimental/buffers/ring_buffer_implementation.hpp#L90). To fix this bug:
   1. Cherry-pick https://github.com/ros2/rclcpp/pull/2061.
 - Segmentation fault if using compiler optimizations (e.g. `-DCMAKE_BUILD_TYPE=Release`). To fix this bug:
   1. Cherry-pick https://github.com/ros2/rclcpp/pull/2059


## Instructions

This repository provides libraries that can be built and installed alongside a standard ROS 2 installation and give access to the `EventsExecutor` class and its related components.

To use the `EventsExecutor` you just need to build the `irobot_events_executor` project and add it as a dependency in your application.

The `irobot_lock_free_events_queue` is an optional package that provides access to a more performant implementation of the `EventsExecutor` events queue.
It also serves as an example of how the `EventsExecutor` can be customized and extended.

## Examples

The repository contains some example applications showing how to use the `EventsExecutor` in your application.

To build and run the examples you can do the following:

```
docker run -it osrf/ros:humble-desktop bash
sudo apt-get update && sudo apt-get upgrade && sudo apt-get install ros-humble-test-msgs
mkdir -p /root/ws/src
cd /root/ws/src
git clone https://github.com/irobot-ros/events-executor.git
cd /root/ws
colcon build
source install/setup.sh
ros2 run events_executor_examples hello_events_executor
```

## Branches

 - `main`: this is the default branch and it is compatible with standard `humble` and `rolling` ROS 2 systems.
 - `humble-future`: this is an experimental branch that requires to apply some changes to core ROS 2 libraries before being able to compile it.
 - `humble-future-gcc8`: this branch builds on top of `humble-future`, and in addition it is compatible with gcc8 compilers.
