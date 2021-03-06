cmake_minimum_required(VERSION 2.8.3)
project(plugins)

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )
#LIST( APPEND CMAKE_MODULE_PATH ${XBOTCORE} )

#message(WARNING "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

set(ENABLE_XENO CACHE BOOL "ON")
if ( ${ENABLE_XENO} )
   find_package(Xenomai REQUIRED)
   find_package(RTnet REQUIRED)
endif()

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined ${CMAKE_SHARED_LINKER_FLAGS}")

include_directories(${CMAKE_BINARY_DIR})
configure_file(
    ${CMAKE_SOURCE_DIR}/config.h.in
    ${CMAKE_BINARY_DIR}/config.h
)

include(MacroYCMInstallLibrary)

set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

find_package(Eigen3 REQUIRED)

if(XBOT)
find_package(XCM REQUIRED)
LIST( APPEND CATKIN_DEPENDS_SET
  robot_class_xbot
  mwoibn_xbot_common
)
else()
  set (XCM_INCLUDE_DIRS "")
  set (XCM_LIBRARIES "")
  set (XCM_EXPORTED_TARGETS "")
endif()

LIST( APPEND CATKIN_DEPENDS_SET
  modules
  roscpp
  mwoibn_ros_common
  loaders

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
  ${PROJECT_SOURCE_DIR}/../../locomotion_framework/third_party/range-v3/include
  ${catkin_INCLUDE_DIRS}
  ${XCM_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
        include
        ${catkin_INCLUDE_DIRS}
        ${XCM_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/../../locomotion_framework/third_party/range-v3/include
  LIBRARIES
        BasePlugin
        ${XCM_LIBRARIES}
  CATKIN_DEPENDS
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
)
add_library(BasePlugin SHARED
    src/xbot_base_v2.cpp
    src/ros_base.cpp
    src/ros_shared.cpp
    src/xbot_shared.cpp
)

if ( Xenomai_FOUND )
    message(WARNING "set_xeno_flags")
    set_xeno_flags(BasePlugin)
endif()

target_link_libraries(BasePlugin ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} -ldl)
add_dependencies(BasePlugin ${catkin_EXPORTED_TARGETS} ${XCM_EXPORTED_TARGETS} ${Eigen3_EXPORTED_TARGETS})

LIST( APPEND INSTALL_TARGETS BasePlugin )

LIST( APPEND INSTALL_TARGETS
  plugin
)
add_executable(plugin src/ros.cpp)
add_dependencies(plugin BasePlugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(plugin
    ${catkin_LIBRARIES}
    BasePlugin
    -lboost_program_options
    ${XCM_LIBRARIES}
 )

#################################################
#################### INSTALL ####################
#################################################
 install(TARGETS ${INSTALL_TARGETS}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
## Mark executables and/or libraries for installation
if(XBOT)
 install(TARGETS ${INSTALL_TARGETS}
   ARCHIVE DESTINATION ${ADVR_SUPERBUILD_DIR}/build/install/lib/
   LIBRARY DESTINATION ${ADVR_SUPERBUILD_DIR}/build/install/lib/
   RUNTIME DESTINATION ${ADVR_SUPERBUILD_DIR}/build/install/bin/
 )
# Mark cpp header files for installation
 install(DIRECTORY include/mgnss/${PROJECT_NAME}
   DESTINATION ${ADVR_SUPERBUILD_DIR}/build/install/include/mgnss/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
 endif()

#################################################
############### RESTORE VARIABLES ###############
#################################################

set(CMAKE_INSTALL_PREFIX ${MASTER_PATH})

 install(DIRECTORY include/mgnss/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mgnss/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
