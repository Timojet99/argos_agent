cmake_minimum_required(VERSION 2.8.3)
project(raiscar_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  nav_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ControlReport.msg
  ControlCommand.msg
  BatteryReport.msg
  PowerReport.msg
  BatteryCalibration.msg
  ControllerPath.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES raiscar
   CATKIN_DEPENDS message_runtime roscpp rospy sensor_msgs std_msgs nav_msgs
#  DEPENDS system_lib
)
