cmake_minimum_required(VERSION 2.8.3)
project(xbot_plugins)

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
include(MacroYCMInstallLibrary)

set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

find_package(Eigen3 REQUIRED)

LIST( APPEND CATKIN_DEPENDS_SET
  robot_class_xbot
  dynamic_models
  gravity_compensation
  motor_side_reference
  hierarchical_control
  controllers
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
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
  ${XCM_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

################################################
############## CONSTANT REFERENCE  #############
################################################
add_library(RtMyTest SHARED
    src/rt_my_test.cpp
)
if ( Xenomai_FOUND )
    set_xeno_flags(${PROJECT_NAME})
endif()
target_link_libraries(RtMyTest ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
ycm_library_install(RtMyTest 1 0 0)

################################################
##### GENERAL CNETRALIZED CONTROLLER plugin ####
################################################
add_library(CentralizedController SHARED
    src/centralized_controller.cpp
)
target_link_libraries(CentralizedController ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

#install
ycm_library_install(CentralizedController 1 0 0)

################################################
##### ONLINE CNETRALIZED CONTROLLER plugin #####
################################################

add_library(OnlineCentralizedControllerPlugin SHARED
    src/online_centralized_controller_plugin.cpp
)
target_link_libraries(OnlineCentralizedControllerPlugin ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
ycm_library_install(OnlineCentralizedControllerPlugin 1 0 0)

################################################
################# GRAVITY TEST #################
################################################
add_library(GravityTest SHARED
    src/gravity_test.cpp
)
target_link_libraries(GravityTest ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
ycm_library_install(GravityTest 1 0 0)

################################################
#################### DRIVING ###################
################################################

add_library(WheeledController SHARED
    src/wheeled_controller.cpp
)
target_link_libraries(WheeledController ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
ycm_library_install(WheeledController 1 0 0)

################################################
################### COMBINED ###################
################################################

add_library(CombinedController SHARED
    src/combined_controller.cpp
)
target_link_libraries(CombinedController ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
ycm_library_install(CombinedController 1 0 0)

## Mark executables and/or libraries for installation
 install(TARGETS ${INSTALL_TARGETS}
   ARCHIVE DESTINATION ${ADVR_SUPERBUILD_DIR}/build/install/lib/
   LIBRARY DESTINATION ${ADVR_SUPERBUILD_DIR}/build/install/lib/
   RUNTIME DESTINATION ${ADVR_SUPERBUILD_DIR}/build/install/bin/
 )

# Mark cpp header files for installation
 install(DIRECTORY ${ADVR_SUPERBUILD_DIR}/build/install/include/
   DESTINATION ${ADVR_GLOBAL_INCLUDE_DESTINATION}/mgnss/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
