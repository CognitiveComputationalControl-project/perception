cmake_minimum_required(VERSION 2.8.3)
project(angle_detector)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(PCL 1.7.2 REQUIRED)
###################################
## catkin specific configuration ##
###################################
    
catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(detector src/detector.cpp)


target_link_libraries(detector
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)




#############
## Install ##
#############

install(TARGETS detector 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
