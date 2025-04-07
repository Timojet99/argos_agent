cmake_minimum_required(VERSION 2.8.3)
project(freicar_perception)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
#  roscpp
  rospy
  std_msgs
#  dynamic_reconfigure
  geometry_msgs
  nav_msgs
#  message_generation
  tf
  visualization_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
#  raiscar_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
        CATKIN_DEPENDS
        roscpp std_msgs
        geometry_msgs
        nav_msgs
        tf2
        tf2_ros
        rospy
)

###########
## Build ##

#include_directories(
#        include
#        ${catkin_INCLUDE_DIRS}
#        ${geos_INCLUDE_DIRS}
#)

#add_executable(pure_pursuit src/pid.cpp src/controller.cpp src/pure_pursuit.cpp)
#target_link_libraries(pure_pursuit ${catkin_LIBRARIES}  ${geos_LIBRARIES})
