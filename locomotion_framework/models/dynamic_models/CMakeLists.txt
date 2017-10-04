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
find_package(RBDL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS 
	include 
	${catkin_INCLUDE_DIRS} 
	${RBDL_INCLUDE_DIR} 
	${RBDL_URDFReader_INCLUDE_DIR} 
	${LOGGER_INCLUDE_DIRS}
  LIBRARIES 
	${catkin_LIBRARIES} 
	${RBDL_LIBRARY} 
	${RBDL_URDFReader_LIBRARY} 
	${LOGGER_LIBRARIES}
  DEPENDS 
	Eigen3
)


###########
## Build ##
###########

include_directories(
include
${catkin_INCLUDE_DIRS}
${RBDL_INCLUDE_DIR}
${RBDL_URDFReader_INCLUDE_DIR}
${Eigen3_INCLUDE_DIRS}
${LOGGER_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


#############
## Install ##
#############

 install(DIRECTORY include
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
