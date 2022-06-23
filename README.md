# events-executor

This repository contains a C++ implementation of the `EventsExecutor` designed by iRobot.

This is a new type of executor compatible with the ROS 2 `rclcpp` library.
It does not rely on the waitset as the default executors, but rather it uses the recently introduced "listener APIs".

The main motivation for this executor is to greatly improve the performance of ROS 2 (in particular reducing latency and CPU usage).

## Instructions

This repository provides libraries that can be built/installed alongside a standard ROS 2 installation and give access to the `EventsExecutor` class and its related components.

To use the `EventsExecutor` you just need to build the `irobot_events_executor` project and add it as a dependency in your application.

The `irobot_lock_free_events_queue` is an optional package that provides access to a more performant implementation of the `EventsExecutor` events queue.
It also serves as an example of how the `EventsExecutor` can be customized and extended.

## Examples

The repository contains some example applications showing how to use the `EventsExecutor` in your application.

To build and run the examples you can do the following:

```
docker run -it osrf/ros:humble-desktop bash
mkdir -p /root/ws/src
cd /root/ws/src
git clone https://github.com/irobot-ros/events-executor.git
cd ..
colcon build
source install/setup.sh
ros2 run events_executor_examples hello_events_executor
```

## rclcpp PRs

List of Pull Requests that are introducing the `EventsExecutor` in the ROS 2 core repositories.

#### Merged PRs

 - https://github.com/ros2/rclcpp/pull/1579
 - https://github.com/ros2/rcl/pull/839
 - https://github.com/ros2/rmw/pull/286
 - https://github.com/ros2/rmw_connextdds/pull/44
 - https://github.com/ros2/rmw_cyclonedds/pull/256
 - https://github.com/ros2/rmw_fastrtps/pull/468
 - https://github.com/ros2/rmw_implementation/pull/161

#### Open PRs

 - https://github.com/ros2/design/pull/305
 - https://github.com/ros2/rclcpp/pull/1891

## Known bugs and limitations

 - The `EventsExecutor` is not notified when a ROS 2 timer is reset.

## Troubleshooting
