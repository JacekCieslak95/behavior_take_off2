cmake_minimum_required(VERSION 2.8.3)
project(behavior_take_off2)

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)

# Directories definition
set(BEHAVIOR_TAKE_OFF2_SOURCE_DIR
  src/source
)
	
set(BEHAVIOR_TAKE_OFF2_INCLUDE_DIR
  src/include
) 

# Files declaration
set(BEHAVIOR_TAKE_OFF2_SOURCE_FILES
  ${BEHAVIOR_TAKE_OFF2_SOURCE_DIR}/behavior_take_off2.cpp
  ${BEHAVIOR_TAKE_OFF2_SOURCE_DIR}/behavior_take_off2_main.cpp
)

set(BEHAVIOR_TAKE_OFF2_HEADER_FILES
  ${BEHAVIOR_TAKE_OFF2_INCLUDE_DIR}/behavior_take_off2.h
)

### Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS 
  roscpp 
  std_msgs
  behavior_process
  droneMsgsROS
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${BEHAVIOR_TAKE_OFF2_INCLUDE_DIR}
  CATKIN_DEPENDS
  roscpp 
  std_msgs
  drone_process
  behavior_process
  droneMsgsROS
  geometry_msgs
  DEPENDS yaml-cpp
)

###########
## Build ##
###########			       	     
include_directories(
  ${BEHAVIOR_TAKE_OFF2_INCLUDE_DIR}
  ${BEHAVIOR_TAKE_OFF2_SOURCE_DIR}
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(behavior_take_off2_lib ${BEHAVIOR_TAKE_OFF2_SOURCE_FILES} ${BEHAVIOR_TAKE_OFF2_HEADER_FILES})
add_dependencies(behavior_take_off_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_take_off2_lib ${catkin_LIBRARIES})
target_link_libraries(behavior_take_off2_lib yaml-cpp)

add_executable(behavior_take_off2 ${BEHAVIOR_TAKE_OFF2_SOURCE_DIR}/behavior_take_off2_main.cpp)
add_dependencies(behavior_take_off ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_take_off2 behavior_take_off2_lib)
target_link_libraries(behavior_take_off2 ${catkin_LIBRARIES})