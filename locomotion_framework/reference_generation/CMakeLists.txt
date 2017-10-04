cmake_minimum_required(VERSION 2.8.3)
project(reference_generation)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp)

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

add_definitions(-std=c++11)

option(LOGGER OFF)

message(STATUS " Flags:")
message(STATUS " LOGGER ${LOGGER}")

set(LOGGER_INCLUDE_DIRS "")
set(LOGGER_LIBRARIES "")
set(LOGGER_EXPORTED_TARGETS "")


if(LOGGER)
find_package(simple_log REQUIRED)
LIST(APPEND LOGGER_LIBRARIES 
	${simple_log_LIBRARIES}
)
LIST(APPEND LOGGER_INCLUDE_DIRS 
	${simple_log_INCLUDE_DIRS}
)
LIST(APPEND LOGGER_EXPORTED_TARGETS 
	${simple_log_EXPORTED_TARGETS}
)
endif(LOGGER)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

find_package(Eigen3 REQUIRED)
find_package(MathGeoLib REQUIRED)

catkin_package(
  
  INCLUDE_DIRS 
	include 
	${CATKIN_INCLUDE_DIRS}
	${LOGGER_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${MathGeoLib_INCLUDE_DIRS}
  LIBRARIES 
	reference_generation
	${CATKIN_LIBRARIES}
	${LOGGER_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${MathGeoLib_LIBRARIES}
  DEPENDS 
	Eigen3
        MathGeoLib
)


###########
## Build ##
###########

include_directories(
include
${Eigen3_INCLUDE_DIRS}
${LOGGER_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
${MathGeoLib_INCLUDE_DIRS}
)


## Declare a C++ library
 add_library(reference_generation
   src/line.cpp
   src/utils.cpp
   src/local_circle.cpp
 )

 add_dependencies(reference_generation ${catkin_EXPORTED_TARGETS} ${LOGGER_EXPORTED_TARGETS} ${MathGeoLib_EXPORTED_TARGETS})

 target_link_libraries(reference_generation
   ${catkin_LIBRARIES}
   ${LOGGER_LIBRARIES}
   ${EIGEN3_LIBRARIES}
   ${MathGeoLib_LIBRARIES}
)


#############
## Install ##   
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS reference_generation
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

#############
## Testing ##
#############


