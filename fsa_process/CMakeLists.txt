cmake_minimum_required(VERSION 2.8.3)
project(fsa_process)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

set(FSA_PROCESS_SOURCE_DIR
  src/source
)

set(FSA_PROCESS_INCLUDE_DIR
  src/include
) 

set(FSA_PROCESS_SOURCE_FILES
  ${FSA_PROCESS_SOURCE_DIR}/fsa_process.cpp
  ${FSA_PROCESS_SOURCE_DIR}/fsa_process_main.cpp
)

set(FSA_PROCESS_HEADER_FILES
  ${FSA_PROCESS_INCLUDE_DIR}/fsa_process.h
)

find_package(catkin REQUIRED COMPONENTS
  droneMsgsROS
  geometry_msgs
  roscpp
  std_msgs
  robot_process
  cmake_modules
  lib_cvgloggerROS
  roslib

) 


catkin_package(
  INCLUDE_DIRS ${FSA_PROCESS_INCLUDE_DIR}
  CATKIN_DEPENDS
  roscpp 
  std_msgs
  drone_process
  droneMsgsROS
  DEPENDS yaml-cpp
  robot_process
  roslib

)

include_directories(
  ${FSA_PROCESS_INCLUDE_DIR}
  ${FSA_PROCESS_SOURCE_DIR}
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)


add_library(fsa_process_lib ${FSA_PROCESS_SOURCE_FILES} ${FSA_PROCESS_HEADER_FILES})
add_dependencies(fsa_process_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(fsa_process_lib ${catkin_LIBRARIES})
target_link_libraries(fsa_process_lib yaml-cpp)

add_executable(fsa_process ${FSA_PROCESS_SOURCE_DIR}/fsa_process_main.cpp)
add_dependencies(fsa_process ${catkin_EXPORTED_TARGETS})
target_link_libraries(fsa_process fsa_process_lib)
target_link_libraries(fsa_process ${catkin_LIBRARIES})

