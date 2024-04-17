cmake_minimum_required(VERSION 3.0.2)
project(first_project)

find_package(catkin REQUIRED COMPONENTS 
roscpp 
std_msgs 
sensor_msgs
tf
tf2_ros
dynamic_reconfigure
)


generate_dynamic_reconfigure_options(
 cfg/parameter.cfg
)

## Declare a catkin package
catkin_package( 
CATKIN_DEPENDS roscpp  nav_msgs tf2_ros tf
  )

## Build talker and listener

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(gps_to_odom src/gps_to_odom.cpp)
target_link_libraries(gps_to_odom ${catkin_LIBRARIES})

add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})

add_executable(lidar_remap src/lidar_remap.cpp)
add_dependencies(lidar_remap ${PROJECT_NAME}_gencfg)  
target_link_libraries(lidar_remap ${catkin_LIBRARIES})

