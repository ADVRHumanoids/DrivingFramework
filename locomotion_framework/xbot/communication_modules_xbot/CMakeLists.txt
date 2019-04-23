cmake_minimum_required(VERSION 2.8.3)
project(communication_modules_xbot)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

set(SOURCE_FILES "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")
set(INSTALL_TARGETS "")

LIST( APPEND INSTALL_TARGETS  communication_modules_xbot )
LIST( APPEND CATKIN_DEPENDS_SET robot_class)
LIST( APPEND SOURCE_FILES src/xbot_lower_level.cpp src/xbot_feedback_online.cpp src/xbot_operational_euler.cpp)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)

find_package(XBotInterface REQUIRED)
find_package(XBotCore-interfaces REQUIRED)

catkin_package(
  INCLUDE_DIRS include
        ${catkin_INCLUDE_DIRS}
        ${XBotInterface_INCLUDE_DIRS}
        ${XBotCore-interfaces_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/../../third_party/range-v3/include
  LIBRARIES
        communication_modules_xbot
        ${catkin_LIBRARIES}
        ${XBotInterface_LIBRARIES}
  CATKIN_DEPENDS
        ${CATKIN_DEPENDS_SET}
  DEPENDS
        XBotInterface
        XBotCore-interfaces
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
  ${XBotInterface_INCLUDE_DIRS}
  ${XBotCore-interfaces_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../third_party/range-v3/include
)

add_library(communication_modules_xbot ${SOURCE_FILES})
add_dependencies(communication_modules_xbot ${catkin_EXPORTED_TARGETS})
target_link_libraries(communication_modules_xbot
        ${catkin_LIBRARIES}
        ${XBotInterface_LIBRARIES}
 )

## Install ##
#############
install(TARGETS ${INSTALL_TARGETS}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
 install(DIRECTORY include/mwoibn/communication_modules
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mwoibn/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )



