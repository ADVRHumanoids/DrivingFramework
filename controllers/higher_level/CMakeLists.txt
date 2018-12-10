cmake_minimum_required(VERSION 2.8.3)
project(mgnss_higher_level)

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

LIST( APPEND INSTALL_TARGETS steering)
LIST( APPEND CATKIN_DEPENDS_SET hierarchical_control robot_class)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)


catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES
        steering
  CATKIN_DEPENDS
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(steering
   src/steering_v2.cpp
   src/steering_v4.cpp
   src/steering_v5.cpp
   src/steering_v6.cpp
   src/steering_v7.cpp
   src/steering_v8.cpp
   src/steering_reactif.cpp
#   src/steering_reactif_zmp.cpp
 )

add_dependencies(steering ${catkin_EXPORTED_TARGETS})

target_link_libraries(steering
   ${catkin_LIBRARIES}
 )

 ## Mark executables and/or libraries for installation
  install(TARGETS ${INSTALL_TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

 # Mark cpp header files for installation
  install(DIRECTORY include/mgnss/higher_level
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mgnss/
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )
