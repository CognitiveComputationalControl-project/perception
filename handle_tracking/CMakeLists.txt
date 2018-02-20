cmake_minimum_required(VERSION 2.8.3)
project(handle_tracking)	

SET(SOURCES src/object_scan.cpp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  message_generation
  std_msgs
  std_srvs
  tf
  pcl_ros
  pcl_conversions
  handle_detector
)

#find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
find_package(PCL 1.7 REQUIRED)
find_package(handle_detector REQUIRED)

add_service_files(
   FILES
   objectfinder.srv
   localize_handle.srv
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
   std_srvs
   sensor_msgs
   visualization_msgs
)


include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)
include_directories(src)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES humantracking
#  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS
  	roscpp
  	geometry_msgs
  	sensor_msgs
    std_msgs
    message_runtime
    tf

 )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(scanner src/scanning_test.cpp ${SOURCES})
add_dependencies(scanner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scanner ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(scanner_running src/object_scan.cpp ${SOURCES})
#add_dependencies(scanner_running ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(scanner_running ${catkin_LIBRARIES} ${PCL_LIBRARIES})
