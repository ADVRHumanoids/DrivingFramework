cmake_minimum_required(VERSION 2.8.3)
project(mgnss_plugins_wheels)

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/configs )

set(ENABLE_XENO CACHE BOOL "ON")
if ( ${ENABLE_XENO} )
#   message(WARNING "enable_xeno")
   find_package(Xenomai REQUIRED)
   find_package(RTnet REQUIRED)
endif()

find_package(XCM REQUIRED)

# set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined ${CMAKE_SHARED_LINKER_FLAGS}")

#include(MacroYCMInstallLibrary)

option(VISUALZATION_TOOLS ON)
option(COLLISION_CHECK OFF)
option(TRAJECTORY_GENERATION OFF)
option(CENTER_OF_MASS_CONTROLLER ON)
option(GRAVITY_TEST ON)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

include_directories(
  ${CMAKE_BINARY_DIR}
  ${PROJECT_SOURCE_DIR}/../../locomotion_framework/third_party/range-v3/include
)
configure_file(
    ${CMAKE_SOURCE_DIR}/config.h.in
    ${CMAKE_BINARY_DIR}/config.h
)

set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

LIST( APPEND CATKIN_DEPENDS_SET
  mgnss_controllers_wheels
  plugins
  mgnss_ros_callbacks_wheels
  robot_class_ros_extension
  robot_class_xbot
  mgnss_utils
)

if(VISUALIZATION_TOOLS)
	add_definitions(-DVISUALIZATION_TOOLS)
endif(VISUALIZATION_TOOLS)



find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)

#set(MASTER_PATH ${CMAKE_INSTALL_PREFIX})
#set(CMAKE_INSTALL_PREFIX ${ADVR_SUPERBUILD_DIR})


## Generate services in the 'srv' folder
#if(COLLISION_CHECK)
# add_service_files(
#   FILES
#   string.srv
# )
#
### Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   # Or other packages containing msgs
# )
#
#endif(COLLISION_CHECK)

catkin_package(
  INCLUDE_DIRS
    include
    ${Eigen3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${XCM_INCLUDE_DIRS}
  LIBRARIES
    ${XCM_LIBRARIES}
  CATKIN_DEPENDS
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
)

include_directories(
#  /usr/local/include
  include
  ${catkin_INCLUDE_DIRS}
  ${XCM_INCLUDE_DIRS}
)

#get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#foreach(dir ${dirs})
#  message(WARNING "dir='${dir}'")
#endforeach()



#    LIST( APPEND INSTALL_TARGETS NwheelsWorld )
#
#    add_library(NwheelsWorld SHARED
#       src/plugins/wheels_world.cpp
#     )
#    add_dependencies(NwheelsWorld ${catkin_EXPORTED_TARGETS})
#    target_link_libraries(NwheelsWorld
#       ${catkin_LIBRARIES}
#       ${XCM_LIBRARIES}
#     )
#
#  if ( Xenomai_FOUND )
#    message(WARNING "set_xeno_flags")
#    set_xeno_flags(NwheelsWorld)
#  endif()

#### USED
    LIST( APPEND INSTALL_TARGETS Nwheels3 )

    add_library(Nwheels3 SHARED
       src/plugins/wheels_3.cpp
     )
    add_dependencies(Nwheels3 ${catkin_EXPORTED_TARGETS})
    target_link_libraries(Nwheels3
       ${catkin_LIBRARIES}
       ${XCM_LIBRARIES}
     )

  if ( Xenomai_FOUND )
    message(WARNING "set_xeno_flags")
    set_xeno_flags(Nwheels3)
  endif()

#     LIST( APPEND INSTALL_TARGETS NwheelsReactif )
#
#     add_library(NwheelsReactif SHARED
#        src/plugins/wheels_reactif.cpp
#      )
#     add_dependencies(NwheelsReactif ${catkin_EXPORTED_TARGETS})
#     target_link_libraries(NwheelsReactif
#        ${catkin_LIBRARIES}
#        ${XCM_LIBRARIES}
#    )
#
#  if ( Xenomai_FOUND )
#    message(WARNING "set_xeno_flags")
#    set_xeno_flags(NwheelsReactif)
#  endif()

# LIST( APPEND INSTALL_TARGETS NwheelsMerge )
#
#      add_library(NwheelsMerge SHARED
#         src/plugins/wheels_merge.cpp
#       )
#      add_dependencies(NwheelsMerge ${catkin_EXPORTED_TARGETS})
#      target_link_libraries(NwheelsMerge
#         ${catkin_LIBRARIES}
#         ${XCM_LIBRARIES}
#     )
#
#  if ( Xenomai_FOUND )
#    message(WARNING "set_xeno_flags")
#    set_xeno_flags(NwheelsMerge)
#  endif()

#### USED
#      LIST( APPEND INSTALL_TARGETS NwheelsZMP )
#
#      add_library(NwheelsZMP SHARED
#         src/plugins/wheels_zmp.cpp
#       )
#      add_dependencies(NwheelsZMP ${catkin_EXPORTED_TARGETS})
#      target_link_libraries(NwheelsZMP
#         ${catkin_LIBRARIES}
#         ${XCM_LIBRARIES}
#    )

#  if ( Xenomai_FOUND )
#    message(WARNING "set_xeno_flags")
#    set_xeno_flags(NwheelsZMP)
#  endif()

  LIST( APPEND INSTALL_TARGETS NwheelsZMPII )

  add_library(NwheelsZMPII SHARED
     src/plugins/wheels_zmp_II.cpp
   )
  add_dependencies(NwheelsZMPII ${catkin_EXPORTED_TARGETS})
  target_link_libraries(NwheelsZMPII
     ${catkin_LIBRARIES}
     ${XCM_LIBRARIES}
)

if ( Xenomai_FOUND )
message(WARNING "set_xeno_flags")
set_xeno_flags(NwheelsZMPII)
endif()

#### NOT USED
# LIST( APPEND INSTALL_TARGETS NwheelsZMPIII )
#
# add_library(NwheelsZMPIII SHARED
#   src/plugins/wheels_zmp_III.cpp
# )
# add_dependencies(NwheelsZMPIII ${catkin_EXPORTED_TARGETS})
# target_link_libraries(NwheelsZMPIII
#   ${catkin_LIBRARIES}
#   ${XCM_LIBRARIES}
#)
#
#if ( Xenomai_FOUND )
#message(WARNING "set_xeno_flags")
#set_xeno_flags(NwheelsZMPIII)
#endif()


LIST( APPEND INSTALL_TARGETS NwheelsZMPIV )

add_library(NwheelsZMPIV SHARED
   src/plugins/wheels_zmp_IV.cpp
 )
add_dependencies(NwheelsZMPIV ${catkin_EXPORTED_TARGETS})
target_link_libraries(NwheelsZMPIV
   ${catkin_LIBRARIES}
   ${XCM_LIBRARIES}
)

if ( Xenomai_FOUND )
message(WARNING "set_xeno_flags")
set_xeno_flags(NwheelsZMPIV)
endif()

#### USED
LIST( APPEND INSTALL_TARGETS NwheelsSecondOrder )

add_library(NwheelsSecondOrder SHARED
   src/plugins/wheels_second_order.cpp
)
add_dependencies(NwheelsSecondOrder ${catkin_EXPORTED_TARGETS})
target_link_libraries(NwheelsSecondOrder
   ${catkin_LIBRARIES}
   ${XCM_LIBRARIES}
)

if ( Xenomai_FOUND )
message(WARNING "set_xeno_flags")
set_xeno_flags(NwheelsSecondOrder)
endif()



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

#message(WARNING "ADVR_SUPERBUILD_DIR: " ${ADVR_SUPERBUILD_DIR})

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

# set(CMAKE_INSTALL_PREFIX ${MASTER_PATH})

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
