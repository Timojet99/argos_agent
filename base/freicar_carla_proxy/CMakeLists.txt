cmake_minimum_required(VERSION 2.8.3)
project(freicar_carla_proxy)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")
# set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


## Find catkin macros and libraries
find_package(Threads REQUIRED)
find_package(Threads REQUIRED)

find_program(LSB_RELEASE_EXEC lsb_release)
execute_process(COMMAND ${LSB_RELEASE_EXEC} -ds
    OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if (${LSB_RELEASE_ID_SHORT} VERSION_GREATER_EQUAL "Ubuntu 20.04 LTS")
	message("Ubuntu 20.04 detected...")
        find_package(OpenCV REQUIRED)
else()
	message("Ubuntu < 20.04 detected....")
        find_package(OpenCV 3 REQUIRED)
endif()

find_package (Eigen3 3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS)

# Carla related stuff
if (NOT DEFINED ENV{CARLA_CXX_API})
    message(FATAL_ERROR "environment variable CARLA_CXX_API is not defined. export it before compiling")
endif ()
FILE(GLOB CARLA_LIBRARIES $ENV{CARLA_CXX_API}/lib/*.a)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
	std_msgs
	nav_msgs
	geometry_msgs
		tf2_ros
		tf2
		tf2_eigen
		tf2_geometry_msgs
	cv_bridge
	image_transport
	raiscar_msgs
	sensor_msgs
		message_generation
)

find_package(  yaml-cpp REQUIRED )

add_service_files(
		FILES
		ResetPosition.srv
		SetPosition.srv
)

generate_messages(
		DEPENDENCIES
		std_msgs
		geometry_msgs
)

catkin_package(CATKIN_DEPENDS
	roscpp
	roslib
	std_msgs
	nav_msgs
	geometry_msgs
		tf2_ros
		tf2
		tf2_eigen
		tf2_geometry_msgs
	cv_bridge
	image_transport
	sensor_msgs
		message_generation
		message_runtime
		INCLUDE_DIRS include
		LIBRARIES freicar_bicycle_model
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	$ENV{CARLA_CXX_API}/include
	$ENV{CARLA_CXX_API}/include/system
	${SUBDIRS}
	${PCL_INCLUDE_DIRS}
		${YAML_CPP_INCLUDE_DIR}
)

add_library(freicar_bicycle_model SHARED
		src/bicycle_model.cpp)

set_target_properties(
		freicar_bicycle_model PROPERTIES CXX_STANDARD 14  # allegedly important.
)

target_link_libraries(freicar_bicycle_model Eigen3::Eigen ${catkin_LIBRARIES})


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# aiscar_airsim_agent node ---------------------------------------------
add_executable(${PROJECT_NAME}_node src/main.cpp
									src/freicar_carla_proxy.cpp
		                            src/pid_controller.cpp
)



add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node ${CMAKE_THREAD_LIBS_INIT}
										   ${catkin_LIBRARIES}
										   ${CARLA_LIBRARIES}
										   $ENV{CARLA_CXX_API}/lib/libDetourCrowd.a
										   $ENV{CARLA_CXX_API}/lib/libDetour.a
										   $ENV{CARLA_CXX_API}/lib/librpc.a
										   Eigen3::Eigen
										   yaml-cpp
										   pthread
										   ${PCL_LIBRARIES}
										   ${OpenCV_LIBS}
											${YAML_CPP_LIBRARIES}
										   freicar_bicycle_model
										   # png
										   # jpeg
										   # tiff
)

install(TARGETS freicar_bicycle_model
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
		)
install(DIRECTORY include/${PROJECT_NAME}/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		)
