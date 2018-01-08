cmake_minimum_required(VERSION 2.8.3)
project(nrt_software)

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

option(VISUALZATION_TOOLS ON)
option(COLLISION_CHECK OFF)
option(TRAJECTORY_GENERATION OFF)
option(CENTRALIZED_CONTROLLER ON)
option(CENTER_OF_MASS_CONTROLLER ON)
option(WHEELS_MOTION ON)
option(TEST_TEMP ON)
option(GRAVITY_TEST ON)
option(ODOMETRY ON)
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

if(VISUALIZATION_TOOLS)
	add_definitions(-DVISUALIZATION_TOOLS)
endif(VISUALIZATION_TOOLS)

  LIST( APPEND CATKIN_DEPENDS_SET
        loaders
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

if(CENTRALIZED_CONTROLLER)
  LIST( APPEND INSTALL_TARGETS 
    centralized_controller
    online_centralized_controller
  )
  LIST( APPEND CATKIN_DEPENDS_SET
    roscpp
    robot_class_ros_extension
    robot_class_xbot
    controllers
    gravity_compensation
    motor_side_reference
    std_srvs
 )
endif(CENTRALIZED_CONTROLLER)

if(CENTER_OF_MASS_CONTROLLER)

LIST( APPEND INSTALL_TARGETS
#  hierarchical_controller_with_events
  com_test
)
LIST( APPEND CATKIN_DEPENDS_SET 
  roscpp
  robot_class_ros_extension
  robot_class_xbot
  hierarchical_control
  custom_services
#  reference_generation
  communication
  visualization_tools
)

endif(CENTER_OF_MASS_CONTROLLER)

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
#  reference_generation
)

endif(GRAVITY_TEST)


if(WHEELS_MOTION)

  LIST( APPEND INSTALL_TARGETS
#    wheeled_locomotion_6
    wheels_cut
    wheels_cut_v2
    wheels_event
    wheels_full
#    wheeled_motion_run
  )
  LIST( APPEND CATKIN_DEPENDS_SET
    roscpp
    robot_class_ros_extension
    robot_class_xbot
    hierarchical_control
    custom_services
#    reference_generation
    controllers
    communication
    visualization_tools
  )

endif(WHEELS_MOTION)

if(TEST_TEMP)

  LIST( APPEND INSTALL_TARGETS
    my_test
    contact_test
    joint_states
  )
  LIST( APPEND CATKIN_DEPENDS_SET
    roscpp
    robot_class_xbot
    robot_class_ros_extension
    hierarchical_control
    custom_services
#    reference_generation
  )
  LIST( APPEND CATKIN_DEPENDS_SET
    visualization_tools
  )

endif(TEST_TEMP)

if(ODOMETRY)
  LIST( APPEND INSTALL_TARGETS
    odometry
  )
  LIST( APPEND CATKIN_DEPENDS_SET
    roscpp
    robot_class_xbot
    robot_class_ros_extension
    odometry
  )

endif(ODOMETRY)

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
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS 
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
#  ${XBotCore-interfaces_INCLUDE_DIRS}
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

if(CENTRALIZED_CONTROLLER)
add_executable(centralized_controller src/centralized_controller.cpp)
add_dependencies(centralized_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(centralized_controller
    ${catkin_LIBRARIES}
 )

add_executable(online_centralized_controller src/online_centralized_controller.cpp)
add_dependencies(online_centralized_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(online_centralized_controller
    ${catkin_LIBRARIES}
 )

endif(CENTRALIZED_CONTROLLER)


if(CENTER_OF_MASS_CONTROLLER)

#add_executable(hierarchical_controller_with_events src/hierarchical_controller_with_events.cpp)
#add_dependencies(hierarchical_controller_with_events ${catkin_EXPORTED_TARGETS})
#target_link_libraries(hierarchical_controller_with_events
#    ${catkin_LIBRARIES}
# )

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




if(WHEELS_MOTION)

#add_executable(wheeled_locomotion_6 src/wheeled_locomotion_v6.cpp)
#add_dependencies(wheeled_locomotion_6 ${catkin_EXPORTED_TARGETS})
#target_link_libraries(wheeled_locomotion_6
#    ${catkin_LIBRARIES}
# )

add_executable(wheels_cut src/wheels_cut.cpp)
add_dependencies(wheels_cut ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheels_cut
    ${catkin_LIBRARIES}
 )

add_executable(wheels_cut_v2 src/wheels_cut_v2.cpp)
add_dependencies(wheels_cut_v2 ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheels_cut_v2
    ${catkin_LIBRARIES}
 )

add_executable(wheels_full src/wheels_full.cpp)
add_dependencies(wheels_full ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheels_full
    ${catkin_LIBRARIES}
 )

add_executable(wheels_event src/wheels_event.cpp)
add_dependencies(wheels_event ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheels_event
    ${catkin_LIBRARIES}
 )

endif(WHEELS_MOTION)#############

if(ODOMETRY)
add_executable(odometry src/odometry.cpp)
add_dependencies(odometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry
    ${catkin_LIBRARIES}
 )
endif(ODOMETRY)

if(TEST_TEMP)
add_executable(my_test src/my_test.cpp)
add_dependencies(my_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(my_test
    ${catkin_LIBRARIES}
 )

add_executable(joint_states src/joint_states.cpp)
add_dependencies(joint_states ${catkin_EXPORTED_TARGETS})
target_link_libraries(joint_states
    ${catkin_LIBRARIES}
 )

add_executable(contact_test src/contact_test.cpp)
add_dependencies(contact_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(contact_test
    ${catkin_LIBRARIES}
 )
endif(TEST_TEMP)

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
