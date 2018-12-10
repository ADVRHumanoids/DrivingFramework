cmake_minimum_required(VERSION 2.8.3)
project(nrt_software)

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

option(VISUALZATION_TOOLS ON)
option(COLLISION_CHECK OFF)
option(TRAJECTORY_GENERATION OFF)
option(CENTER_OF_MASS_CONTROLLER ON)
option(GRAVITY_TEST ON)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

include_directories(${CMAKE_BINARY_DIR})
configure_file(
    ${CMAKE_SOURCE_DIR}/config.h.in
    ${CMAKE_BINARY_DIR}/config.h
)

set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

LIST( APPEND CATKIN_DEPENDS_SET
  controllers
  state_estimation
)

if(VISUALIZATION_TOOLS)
	add_definitions(-DVISUALIZATION_TOOLS)
endif(VISUALIZATION_TOOLS)

  LIST( APPEND CATKIN_DEPENDS_SET
        plugins
        ros_callbacks
        )

if(COLLISION_CHECK)
  LIST( APPEND INSTALL_TARGETS collision_check )
  LIST( APPEND CATKIN_DEPENDS_SET
    roscpp
    robot_class_ros_extension
    visualization_tools
    std_msgs
  )
  LIST( APPEND CATKIN_DEPENDS_BUILD message_generation)
endif(COLLISION_CHECK)

if(TRAJECTORY_GENERATION)
#find_package(MathGeoLib REQUIRED)

LIST( APPEND INSTALL_TARGETS simple_trajectory )
LIST( APPEND CATKIN_DEPENDS_SET
  roscpp
  robot_class_ros_extension
  visualization_tools
  point_handling
  hierarchical_control
  collision_model_ros_extension
#  reference_generation
)

endif(TRAJECTORY_GENERATION)

if(CENTER_OF_MASS_CONTROLLER)

LIST( APPEND INSTALL_TARGETS
  com_test
)
LIST( APPEND CATKIN_DEPENDS_SET
  roscpp
  robot_class_ros_extension
  robot_class_xbot
  hierarchical_control
  custom_services
  communication
  visualization_tools
)

endif(CENTER_OF_MASS_CONTROLLER)

LIST( APPEND INSTALL_TARGETS
  shared
)

if(GRAVITY_TEST)

LIST( APPEND INSTALL_TARGETS
  gravity_test
)
LIST( APPEND CATKIN_DEPENDS_SET
  roscpp
  robot_class_ros_extension
  robot_class_xbot
  hierarchical_control
  custom_services
)

endif(GRAVITY_TEST)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)

## Generate services in the 'srv' folder
if(COLLISION_CHECK)
 add_service_files(
   FILES
   string.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   # Or other packages containing msgs
 )

endif(COLLISION_CHECK)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#  ${XBotCore-interfaces_INCLUDE_DIRS}
)

LIST( APPEND INSTALL_TARGETS joint_states )

add_library(joint_states SHARED
   src/plugins/joint_states.cpp
 )
add_dependencies(joint_states ${catkin_EXPORTED_TARGETS})
target_link_libraries(joint_states
   ${catkin_LIBRARIES}
 )

 LIST( APPEND INSTALL_TARGETS centralized )

 add_library(centralized SHARED
    src/plugins/centralized.cpp
  )
 add_dependencies(centralized ${catkin_EXPORTED_TARGETS})
 target_link_libraries(centralized
    ${catkin_LIBRARIES}
  )

 LIST( APPEND INSTALL_TARGETS odometry )

 add_library(odometry SHARED
    src/plugins/odometry_2.cpp
  )
 add_dependencies(odometry ${catkin_EXPORTED_TARGETS})
 target_link_libraries(odometry
    ${catkin_LIBRARIES}
  )


  LIST( APPEND INSTALL_TARGETS NwheelsWorld )

  add_library(NwheelsWorld SHARED
     src/plugins/wheels_world.cpp
   )
  add_dependencies(NwheelsWorld ${catkin_EXPORTED_TARGETS})
  target_link_libraries(NwheelsWorld
     ${catkin_LIBRARIES}
   )

  LIST( APPEND INSTALL_TARGETS Nwheels3 )

  add_library(Nwheels3 SHARED
     src/plugins/wheels_3.cpp
   )
  add_dependencies(Nwheels3 ${catkin_EXPORTED_TARGETS})
  target_link_libraries(Nwheels3
     ${catkin_LIBRARIES}
   )

   LIST( APPEND INSTALL_TARGETS NwheelsReactif )

   add_library(NwheelsReactif SHARED
      src/plugins/wheels_reactif.cpp
    )
   add_dependencies(NwheelsReactif ${catkin_EXPORTED_TARGETS})
   target_link_libraries(NwheelsReactif
      ${catkin_LIBRARIES}
    )

    LIST( APPEND INSTALL_TARGETS NwheelsMerge )

    add_library(NwheelsMerge SHARED
       src/plugins/wheels_merge.cpp
     )
    add_dependencies(NwheelsMerge ${catkin_EXPORTED_TARGETS})
    target_link_libraries(NwheelsMerge
       ${catkin_LIBRARIES}
     )

    LIST( APPEND INSTALL_TARGETS NwheelsZMP )

    add_library(NwheelsZMP SHARED
       src/plugins/wheels_zmp.cpp
     )
    add_dependencies(NwheelsZMP ${catkin_EXPORTED_TARGETS})
    target_link_libraries(NwheelsZMP
       ${catkin_LIBRARIES}
     )

   LIST( APPEND INSTALL_TARGETS ground_forces )

   add_library(ground_forces SHARED
      src/plugins/ground_forces.cpp
    )
   add_dependencies(ground_forces ${catkin_EXPORTED_TARGETS})
   target_link_libraries(ground_forces
      ${catkin_LIBRARIES}
    )

    LIST( APPEND INSTALL_TARGETS gravity_compensation )

    add_library(gravity_compensation SHARED
       src/plugins/gravity_compensation.cpp
     )
    add_dependencies(gravity_compensation ${catkin_EXPORTED_TARGETS})
    target_link_libraries(gravity_compensation
       ${catkin_LIBRARIES}
     )

    LIST( APPEND INSTALL_TARGETS acceleration )

    add_library(acceleration SHARED
       src/plugins/acceleration_test.cpp
     )
    add_dependencies(acceleration ${catkin_EXPORTED_TARGETS})
    target_link_libraries(acceleration
       ${catkin_LIBRARIES}
     )

if(COLLISION_CHECK)
add_executable(collision_check src/collision_check.cpp)
add_dependencies(collision_check ${catkin_EXPORTED_TARGETS})
target_link_libraries(collision_check
    ${catkin_LIBRARIES}
 )
endif(COLLISION_CHECK)

if(TRAJECTORY_GENERATION)
add_executable(simple_trajectory src/simple_trajectory.cpp)
add_dependencies(simple_trajectory ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_trajectory
    ${catkin_LIBRARIES}
 )
endif(TRAJECTORY_GENERATION)


if(CENTER_OF_MASS_CONTROLLER)

add_executable(com_test src/com_test.cpp)
add_dependencies(com_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(com_test
    ${catkin_LIBRARIES}
 )

endif(CENTER_OF_MASS_CONTROLLER)#############

if(GRAVITY_TEST)

add_executable(gravity_test src/gravity_test.cpp)
add_dependencies(gravity_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(gravity_test
    ${catkin_LIBRARIES}
 )
endif(GRAVITY_TEST)


 add_executable(shared src/exec/shared.cpp)
 add_dependencies(shared ${catkin_EXPORTED_TARGETS})
 target_link_libraries(shared
     ${catkin_LIBRARIES}
  )
#add_executable(contact_test src/contact_test.cpp)
#add_dependencies(contact_test ${catkin_EXPORTED_TARGETS})
#target_link_libraries(contact_test
#    ${catkin_LIBRARIES}
# )


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
