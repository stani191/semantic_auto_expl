cmake_minimum_required(VERSION 2.8.3)
project(autonomous_expl)

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

set(autonomous_expl_DEPENDENCIES
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  cv_bridge
  image_transport
  tf
)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  ${autonomous_expl_DEPENDENCIES}
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES autonomous_expl
  CATKIN_DEPENDS ${autonomous_expl_DEPENDENCIES}
  #DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OpenCV_LIBS} 
)

 add_executable(autonomous_expl_node 
    src/AutoExplorationApp.cpp
    src/AutoExplorationNode.cpp
    src/DistanceTransformer.cpp
    src/FrontierExtractor.cpp
    src/TargetSelector.cpp 
 )
 
 target_link_libraries(autonomous_expl_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${OpenCV_LIBRARIES}  
 )

#############
## Install ##
#############

install(TARGETS autonomous_expl_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_autonomous_expl.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
