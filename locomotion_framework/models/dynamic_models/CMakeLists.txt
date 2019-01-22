cmake_minimum_required(VERSION 2.8.3)
project(dynamic_models)

# The version number, project name, add c++ flags
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

find_package(catkin REQUIRED robot_class)

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

catkin_package(
  INCLUDE_DIRS
	include
	${LOGGER_INCLUDE_DIRS}
  LIBRARIES
	${LOGGER_LIBRARIES}
	CATKIN_DEPENDS
		robot_class
	DEPENDS
)


###########
## Build ##
###########

include_directories(
include
${catkin_INCLUDE_DIRS}
${LOGGER_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


#############
## Install ##
#############

## Mark cpp header files for installation
 install(DIRECTORY include/mwoibn/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mwoibn/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
