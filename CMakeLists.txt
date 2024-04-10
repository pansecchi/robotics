cmake_minimum_required(VERSION 3.0.2)
project(first_project)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(sub1 src/gps_to_odom.cpp)
target_link_libraries(sub1 ${catkin_LIBRARIES})

