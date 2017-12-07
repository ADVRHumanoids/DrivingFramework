cmake_minimum_required(VERSION 2.8.3)
project(communication_modules_ros_extension)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

option(CUSTOM_CONTROLLER ON)
option(LOGGER ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

set(SOURCE_FILES "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

LIST( APPEND CATKIN_DEPENDS_SET robot_class)

if(LOGGER)

LIST( APPEND CATKIN_DEPENDS_SET simple_log)

endif(LOGGER)

find_package(RBDL REQUIRED)
find_package(Eigen3 REQUIRED)

if(CUSTOM_CONTROLLER)

LIST( APPEND CATKIN_DEPENDS_SET roscpp custom_controller)
LIST( APPEND SOURCE_FILES src/custom_controller.cpp src/ros_controller.cpp src/velocity_controller.cpp)

endif(CUSTOM_CONTROLLER)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)

catkin_package(
  INCLUDE_DIRS include
        ${catkin_INCLUDE_DIRS}
        ${RBDL_INCLUDE_DIR}
  LIBRARIES
        communication_modules_ros
        ${RBDL_LIBRARY}
  CATKIN_DEPENDS
        roscpp
        ${CATKIN_DEPENDS_SET}
  DEPENDS
        ${DEPENDS_OPTIONAL}
        Eigen3
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(communication_modules_ros ${SOURCE_FILES})
add_dependencies(communication_modules_ros ${catkin_EXPORTED_TARGETS})
target_link_libraries(communication_modules_ros
    ${catkin_LIBRARIES}
 )

## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS communication_modules_ros
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/mwoibn/communication_modules
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mwoibn/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

