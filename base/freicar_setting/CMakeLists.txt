cmake_minimum_required(VERSION 2.8.3)
project(freicar_setting)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


## Find catkin macros and libraries
find_package(Threads REQUIRED)
# Carla related stuff
if (NOT DEFINED ENV{CARLA_CXX_API})
    message(FATAL_ERROR "environment variable CARLA_CXX_API is not defined. export it before compiling")
endif ()
FILE(GLOB CARLA_LIBRARIES $ENV{CARLA_CXX_API}/lib/*.a)
# message("${CARLA_LIBRARIES}")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
	rosgraph_msgs
)

find_package(  yaml-cpp REQUIRED )

catkin_package(CATKIN_DEPENDS
	roscpp
	roslib
	rosgraph_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	$ENV{CARLA_CXX_API}/include
	$ENV{CARLA_CXX_API}/include/system
	${SUBDIRS}
		${YAML_CPP_INCLUDE_DIR}
)
# ros_carla_agent node ---------------------------------------------
add_executable(${PROJECT_NAME}_node src/main.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${CMAKE_THREAD_LIBS_INIT}
										   ${catkin_LIBRARIES}
										   ${CARLA_LIBRARIES}
										   $ENV{CARLA_CXX_API}/lib/libDetourCrowd.a
										   $ENV{CARLA_CXX_API}/lib/libDetour.a
										   $ENV{CARLA_CXX_API}/lib/librpc.a
											${YAML_CPP_LIBRARIES}
										   yaml-cpp
										   pthread
)
