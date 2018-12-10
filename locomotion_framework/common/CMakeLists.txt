cmake_minimum_required(VERSION 2.8.3)
project(common)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED rbdl_utils eigen_utils mwoibn_std_utils)

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

add_definitions(-std=c++11)

option(LOGGER OFF)

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

message(STATUS " Flags:")
message(STATUS " LOGGER ${LOGGER}")

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )
find_package(RBDL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
		include
		${catkin_INCLUDE_DIRS}
		${RBDL_INCLUDE_DIR}
		${LOGGER_INCLUDE_DIRS}
  LIBRARIES
		mwoibn_common
		${catkin_LIBRARIES}
		${RBDL_LIBRARY}
		${LOGGER_LIBRARIES}
  DEPENDS
		Eigen3
	CATKIN_DEPENDS
		rbdl_utils
		eigen_utils
		mwoibn_std_utils
)

include_directories(
include
${catkin_INCLUDE_DIRS}
${RBDL_INCLUDE_DIR}
${RBDL_URDFReader_INCLUDE_DIR}
${Eigen3_INCLUDE_DIRS}
${LOGGER_INCLUDE_DIRS}
)

add_library(mwoibn_common
	src/state.cpp
)

add_dependencies(mwoibn_common ${catkin_EXPORTED_TARGETS} ${LOGGER_EXPORTED_TARGETS})

target_link_libraries(mwoibn_common
	${catkin_LIBRARIES}
	${RBDL_LIBRARY}
	${RBDL_URDFReader_LIBRARY}
	${LOGGER_LIBRARIES}
)


# Mark executables and/or libraries for installation
 install(TARGETS mwoibn_common
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/mwoibn/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mwoibn/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
