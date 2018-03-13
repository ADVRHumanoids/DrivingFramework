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

set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined ${CMAKE_SHARED_LINKER_FLAGS}")


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
  custom_services
  custom_messages
  state_estimation
  plugins
  ros_callbacks
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)

###########
## Build ##
###########

set(MASTER_PATH ${CMAKE_INSTALL_PREFIX})
set(CMAKE_INSTALL_PREFIX ${ADVR_SUPERBUILD_DIR})

message(WARNING "CMAKE_INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX})

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

#if ( Xenomai_FOUND )
#    set_xeno_flags(MyTest)
#endif()
#add_library(MyTest SHARED
#    src/rt_my_test.cpp
#)
#target_link_libraries(MyTest ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
#add_dependencies(MyTest ${catkin_EXPORTED_TARGETS} ${XCM_EXPORTED_TARGETS} ${Eigen3_EXPORTED_TARGETS})
#ycm_library_install(MyTest 1 0 0)

#LIST( APPEND INSTALL_TARGETS MyTest )

################################################
############## CONSTANT REFERENCES #############
################################################

if ( Xenomai_FOUND )
    set_xeno_flags(JointStates)
endif()
add_library(JointStates SHARED
    src/joint_states.cpp
)

target_link_libraries(JointStates ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(JointStates ${catkin_EXPORTED_TARGETS} ${XCM_EXPORTED_TARGETS} ${Eigen3_EXPORTED_TARGETS})
ycm_library_install(JointStates 1 0 0)

LIST( APPEND INSTALL_TARGETS JointStates )

################################################
################### ODOMETRY ###################
################################################

if ( Xenomai_FOUND )
    set_xeno_flags(Odometry)
endif()
add_library(Odometry SHARED
    src/odometry.cpp
)

target_link_libraries(Odometry ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(Odometry ${catkin_EXPORTED_TARGETS} ${XCM_EXPORTED_TARGETS} ${Eigen3_EXPORTED_TARGETS})
ycm_library_install(Odometry 1 0 0)

LIST( APPEND INSTALL_TARGETS Odometry )



################################################
################## WHEELS NEW ##################
################################################

if ( Xenomai_FOUND )
    set_xeno_flags(WheelsV2)
endif()
add_library(WheelsV2 SHARED
    src/wheeled_controller_v2.cpp
)

target_link_libraries(WheelsV2 ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(WheelsV2 ${catkin_EXPORTED_TARGETS} ${XCM_EXPORTED_TARGETS} ${Eigen3_EXPORTED_TARGETS})
ycm_library_install(WheelsV2 1 0 0)

LIST( APPEND INSTALL_TARGETS WheelsV2 )



#################################################
###### GENERAL CNETRALIZED CONTROLLER plugin ####
#################################################
#add_library(CentralizedController SHARED
#    src/centralized_controller.cpp
#)
#target_link_libraries(CentralizedController ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

##install
#ycm_library_install(CentralizedController 1 0 0)

#################################################
###### ONLINE CNETRALIZED CONTROLLER plugin #####
#################################################

#add_library(OnlineCentralizedControllerPlugin SHARED
#    src/online_centralized_controller_plugin.cpp
#)
#target_link_libraries(OnlineCentralizedControllerPlugin ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
#ycm_library_install(OnlineCentralizedControllerPlugin 1 0 0)

#################################################
################## GRAVITY TEST #################
#################################################
#add_library(GravityTest SHARED
#    src/gravity_test.cpp
#)
#target_link_libraries(GravityTest ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
#ycm_library_install(GravityTest 1 0 0)

#################################################
##################### DRIVING ###################
#################################################

#add_library(WheeledController SHARED
#    src/wheeled_controller.cpp
#)
#target_link_libraries(WheeledController ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
#ycm_library_install(WheeledController 1 0 0)

#################################################
#################### COMBINED ###################
#################################################

#add_library(CombinedController SHARED
#    src/combined_controller.cpp
#)
#target_link_libraries(CombinedController ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
#ycm_library_install(CombinedController 1 0 0)

#################################################
#################### INSTALL ####################
#################################################

# Mark executables and/or libraries for installation
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

#################################################
############### RESTORE VARIABLES ###############
#################################################

set(CMAKE_INSTALL_PREFIX ${MASTER_PATH})

 install(DIRECTORY include/mgnss/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mgnss/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
