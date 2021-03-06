cmake_minimum_required(VERSION 2.8.3)
project(handle_detector)

## find catkin and required packages
find_package(catkin REQUIRED COMPONENTS 
  rospy
  roscpp 
  std_msgs 
  tf
  tf_conversions
  eigen_conversions 
  geometry_msgs 
  message_generation   
  pcl_ros
  pcl_conversions

  visualization_msgs)

## find Lapack
find_package(LAPACK REQUIRED)

## find Eigen package
#find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
include_directories(${Eigen_INCLUDE_DIRS})

## compiler optimization flags
set(CMAKE_CXX_FLAGS "-DNDEBUG -O3 -fopenmp -Wno-deprecated")

## add messages
add_message_files(
  FILES
  CylinderMsg.msg
  CylinderArrayMsg.msg
  HandleListMsg.msg
)
add_service_files(
   FILES
   localize_handle.srv
#   Service2.srv
 )
generate_messages(
	DEPENDENCIES 
	geometry_msgs    
	std_msgs
      sensor_msgs
	   visualization_msgs)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS 
    eigen_conversions 
    geometry_msgs 
    message_runtime 
    roscpp 
    pcl_ros
    pcl_conversions
    std_msgs 
    tf
    tf_conversions
    visualization_msgs
  DEPENDS
    eigen3
    LAPACK)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include)

## create executables
add_executable(${PROJECT_NAME}_localization src/localization.cpp)
add_dependencies(${PROJECT_NAME}_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(${PROJECT_NAME}_importance_sampling src/importance_sampling.cpp)

## create libraries
add_library(${PROJECT_NAME}_affordances src/affordances.cpp)
add_library(${PROJECT_NAME}_cylindrical_shell src/cylindrical_shell.cpp)
add_library(${PROJECT_NAME}_messages src/messages.cpp)
add_library(${PROJECT_NAME}_sampling src/sampling.cpp)
add_library(${PROJECT_NAME}_sampling_visualizer src/sampling_visualizer.cpp)
add_library(${PROJECT_NAME}_visualizer src/visualizer.cpp)

## add dependencies
add_dependencies(${PROJECT_NAME}_messages ${PROJECT_NAME}_gencpp)

## link libraries to localization executable
target_link_libraries(${PROJECT_NAME}_localization ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_localization ${PROJECT_NAME}_affordances)
target_link_libraries(${PROJECT_NAME}_localization ${PROJECT_NAME}_visualizer)
target_link_libraries(${PROJECT_NAME}_localization ${PROJECT_NAME}_messages)

## link libraries to importance_sampling executable
target_link_libraries(${PROJECT_NAME}_importance_sampling ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_importance_sampling ${PROJECT_NAME}_affordances)
target_link_libraries(${PROJECT_NAME}_importance_sampling ${PROJECT_NAME}_visualizer)
target_link_libraries(${PROJECT_NAME}_importance_sampling ${PROJECT_NAME}_messages)
target_link_libraries(${PROJECT_NAME}_importance_sampling ${PROJECT_NAME}_sampling)

## link libraries to affordances library
target_link_libraries(${PROJECT_NAME}_affordances ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_affordances ${PROJECT_NAME}_cylindrical_shell)
target_link_libraries(${PROJECT_NAME}_affordances lapack)

## link libraries to messages library
target_link_libraries(${PROJECT_NAME}_messages ${catkin_LIBRARIES})

## link libraries to sampling library
target_link_libraries(${PROJECT_NAME}_sampling ${PROJECT_NAME}_affordances)
target_link_libraries(${PROJECT_NAME}_sampling ${PROJECT_NAME}_sampling_visualizer)

## install targets
install(TARGETS ${PROJECT_NAME}_localization ${PROJECT_NAME}_importance_sampling 
    ${PROJECT_NAME}_affordances ${PROJECT_NAME}_sampling ${PROJECT_NAME}_sampling_visualizer
    ${PROJECT_NAME}_visualizer ${PROJECT_NAME}_messages ${PROJECT_NAME}_cylindrical_shell 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

## install header files  
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
