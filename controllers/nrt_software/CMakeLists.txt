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
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

if(VISUALIZATION_TOOLS)
	add_definitions(-DVISUALIZATION_TOOLS)
endif(VISUALIZATION_TOOLS)


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
  reference_generation
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
  hierarchical_controller_with_events
)
LIST( APPEND CATKIN_DEPENDS_SET 
  roscpp
  robot_class_ros_extension
  robot_class_xbot
  hierarchical_control
  custom_services
  reference_generation
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
  reference_generation
)

endif(GRAVITY_TEST)


if(WHEELS_MOTION)

  LIST( APPEND INSTALL_TARGETS
    wheeled_locomotion_6
    wheels_cut
    wheeled_motion_run
  )
  LIST( APPEND CATKIN_DEPENDS_SET
    roscpp
    robot_class_ros_extension
    robot_class_xbot
    hierarchical_control
    custom_services
    reference_generation
    controllers
    communication
    visualization_tools
  )

endif(WHEELS_MOTION)

if(TEST_TEMP)

  LIST( APPEND INSTALL_TARGETS
    my_test
  )
  LIST( APPEND CATKIN_DEPENDS_SET
    roscpp
    robot_class_xbot
    robot_class_ros_extension
    hierarchical_control
    custom_services
    reference_generation
  )
endif(TEST_TEMP)

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
  LIBRARIES
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

add_executable(hierarchical_controller_with_events src/hierarchical_controller_with_events.cpp)
add_dependencies(hierarchical_controller_with_events ${catkin_EXPORTED_TARGETS})
target_link_libraries(hierarchical_controller_with_events
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

add_executable(wheeled_locomotion_6 src/wheeled_locomotion_v6.cpp)
add_dependencies(wheeled_locomotion_6 ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheeled_locomotion_6
    ${catkin_LIBRARIES}
 )

add_executable(wheels_cut src/wheels_cut.cpp)
add_dependencies(wheels_cut ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheels_cut
    ${catkin_LIBRARIES}
 )

add_executable(wheeled_motion_run src/wheeled_motion_run.cpp)
add_dependencies(wheeled_motion_run ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheeled_motion_run
    ${catkin_LIBRARIES}
 )
#add_executable(wheeled_locomotion_7 src/wheeled_locomotion_v7.cpp)
#add_dependencies(wheeled_locomotion_7 ${catkin_EXPORTED_TARGETS})
#target_link_libraries(wheeled_locomotion_7
#    ${catkin_LIBRARIES}
# )

#add_executable(wheeled_locomotion_8 src/wheeled_locomotion_v8.cpp)
#add_dependencies(wheeled_locomotion_8 ${catkin_EXPORTED_TARGETS})
#target_link_libraries(wheeled_locomotion_8
#    ${catkin_LIBRARIES}
# )

#add_executable(wheeled_locomotion_6a src/wheeled_locomotion_v6_a.cpp)
#add_dependencies(wheeled_locomotion_6a ${catkin_EXPORTED_TARGETS})
#target_link_libraries(wheeled_locomotion_6a
#    ${catkin_LIBRARIES}
# )

#add_executable(wheeled_locomotion_7a src/wheeled_locomotion_v7_a.cpp)
#add_dependencies(wheeled_locomotion_7a ${catkin_EXPORTED_TARGETS})
#target_link_libraries(wheeled_locomotion_7a
#    ${catkin_LIBRARIES}
# )

#add_executable(wheeled_locomotion_8a src/wheeled_locomotion_v8_a.cpp)
#add_dependencies(wheeled_locomotion_8a ${catkin_EXPORTED_TARGETS})
#target_link_libraries(wheeled_locomotion_8a
#    ${catkin_LIBRARIES}
# )
#add_executable(test_flat_reference src/test_flat_reference.cpp)
#add_dependencies(test_flat_reference ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_flat_reference
#    ${catkin_LIBRARIES}
# )
endif(WHEELS_MOTION)#############

if(TEST_TEMP)
add_executable(my_test src/my_test.cpp)
add_dependencies(my_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(my_test
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
 install(DIRECTORY include/mgnss/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
