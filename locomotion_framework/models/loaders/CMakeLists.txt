cmake_minimum_required(VERSION 2.8.3)
project(loaders)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# The version number, project name, add c++ flags
set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 14)

set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

option(ROS ON)
option(XBOT ON)

find_package(catkin REQUIRED)

LIST( APPEND CATKIN_DEPENDS_SET robot_class)

include_directories(${CMAKE_BINARY_DIR})
configure_file(
    ${CMAKE_SOURCE_DIR}/config.h.in
    ${CMAKE_SOURCE_DIR}/include/mwoibn/loaders/config.h
)

if(ROS)
        add_definitions(-DROS)
        LIST( APPEND CATKIN_DEPENDS_SET robot_class_ros_extension)
endif()
if(XBOT)
        add_definitions(-DXBOT)
        LIST( APPEND CATKIN_DEPENDS_SET robot_class_xbot)
endif()


find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)


catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${CMAKE_BINARY_DIR}
  LIBRARIES
        ${INSTALL_TARGETS}
  CATKIN_DEPENDS
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Mark cpp header files for installation
 install(DIRECTORY include/mwoibn/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mwoibn/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
