cmake_minimum_required(VERSION 2.8.3)
project(maze_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  duckietown_msgs
  sensor_msgs
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)