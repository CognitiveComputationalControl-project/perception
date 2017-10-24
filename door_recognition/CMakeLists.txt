cmake_minimum_required(VERSION 2.8.3)
project(door_recognition)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
 


## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
  roscpp
	pcl_ros
	pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
  message_generation
)
find_package(PCL 1.7.2 REQUIRED)
add_service_files(
   FILES
   ObjectLocalize.srv
 
)

generate_messages(
   DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    std_srvs  
)
catkin_package(

#  INCLUDE_DIRS include
#  LIBRARIES door_recognition
  CATKIN_DEPENDS     tmc_yolo2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/door_recognition.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(door_client src/findit.cpp src/quat_helper.cpp)
target_link_libraries(door_client ${catkin_LIBRARIES} ${PCL_LIBRARIES})


