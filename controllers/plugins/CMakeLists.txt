cmake_minimum_required(VERSION 2.8.3)
project(plugins)

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(ENABLE_XENO CACHE BOOL "ON")
if ( ${ENABLE_XENO} )
   find_package(Xenomai)
   find_package(RTnet)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined ${CMAKE_SHARED_LINKER_FLAGS}")
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

include_directories(${CMAKE_BINARY_DIR})
configure_file(
    ${CMAKE_SOURCE_DIR}/config.h.in
    ${CMAKE_BINARY_DIR}/config.h
)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )
include(MacroYCMInstallLibrary)

set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

find_package(Eigen3 REQUIRED)
find_package(XCM REQUIRED)

LIST( APPEND CATKIN_DEPENDS_SET
  robot_class_xbot
  modules
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)

###########
## Build ##
###########

#set(MASTER_PATH ${CMAKE_INSTALL_PREFIX})
#set(CMAKE_INSTALL_PREFIX ${ADVR_SUPERBUILD_DIR})

#message(WARNING "CMAKE_INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX})

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${XCM_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS 
        include 
        ${catkin_INCLUDE_DIRS}   
        ${XCM_INCLUDE_DIRS}
  LIBRARIES 
        BasePlugin
        ${XCM_LIBRARIES}
  CATKIN_DEPENDS 
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
)
################################################
############## CONSTANT REFERENCE  #############
################################################

add_library(BasePlugin SHARED
    src/xbot_base.cpp
    src/ros_base.cpp
)

target_link_libraries(BasePlugin ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(BasePlugin ${catkin_EXPORTED_TARGETS} ${XCM_EXPORTED_TARGETS} ${Eigen3_EXPORTED_TARGETS})

LIST( APPEND INSTALL_TARGETS BasePlugin )

#################################################
#################### INSTALL ####################
#################################################

## Mark executables and/or libraries for installation
 install(TARGETS ${INSTALL_TARGETS}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
 install(DIRECTORY include/mgnss/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mgnss/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

#################################################
############### RESTORE VARIABLES ###############
#################################################

#set(CMAKE_INSTALL_PREFIX ${MASTER_PATH})

# install(DIRECTORY include/mgnss/${PROJECT_NAME}
#   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mgnss/
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
