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

set(CMAKE_CXX_STANDARD 14)

option(ROS_CONTROL OFF)
option(LOGGER ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

include_directories(${CMAKE_INSTALL_PREFIX})
configure_file(
    ${CMAKE_SOURCE_DIR}/config.h.in
    ${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mwoibn/communication_modules/config.h
)

set(SOURCE_FILES "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")
set(LIBRARIES "")

LIST( APPEND CATKIN_DEPENDS_SET robot_class)

if(LOGGER)

LIST( APPEND CATKIN_DEPENDS_SET simple_log)

endif(LOGGER)

LIST( APPEND CATKIN_DEPENDS_SET roscpp)


if(ROS_CONTROL)
	LIST( APPEND CATKIN_DEPENDS_SET custom_controller custom_services custom_messages)
	LIST( APPEND CATKIN_DEPENDS_BUILD message_generation )
        LIST( APPEND SOURCE_FILES src/custom_controller.cpp  src/velocity_controller.cpp)
endif(ROS_CONTROL)

LIST( APPEND SOURCE_FILES src/ros_controller.cpp)

LIST( APPEND LIBRARIES communication_modules_ros)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)

 add_message_files(DIRECTORY msg
   FILES
   linear_point.msg
)
generate_messages(
   DEPENDENCIES std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
        ${catkin_INCLUDE_DIRS}                
        ${PROJECT_SOURCE_DIR}/../../third_party/range-v3/include
  LIBRARIES
        ${LIBRARIES}
  CATKIN_DEPENDS
        roscpp
        ${CATKIN_DEPENDS_SET}
        message_runtime
  DEPENDS
        ${DEPENDS_OPTIONAL}
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
  ${PROJECT_SOURCE_DIR}/../../third_party/range-v3/include  
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
