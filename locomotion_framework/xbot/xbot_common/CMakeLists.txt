cmake_minimum_required(VERSION 2.8.3)
project(mwoibn_xbot_common)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
#set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

LIST( APPEND INSTALL_TARGETS xbot_logger)
LIST( APPEND CATKIN_DEPENDS_SET common)

find_package(XBotInterface REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)


catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES
        xbot_logger
  CATKIN_DEPENDS
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
        XBotInterface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${XBotInterface_INCLUDE_DIRS}
)

add_library(xbot_logger
   src/xbot_logger.cpp
)

add_dependencies(xbot_logger 
   ${catkin_EXPORTED_TARGETS}
   ${XBotInterface_EXPORTED_TARGETS}
)

target_link_libraries(xbot_logger
   ${catkin_LIBRARIES}
   ${XBotInterface_LIBRARIES}
)

## Mark executables and/or libraries for installation
 install(TARGETS ${INSTALL_TARGETS}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
 install(DIRECTORY include/mwoibn/common
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mwoibn/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
