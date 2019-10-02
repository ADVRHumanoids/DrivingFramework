cmake_minimum_required(VERSION 2.8.3)
project(communication)
option(XBOT ON)

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 14)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )


set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

if(HANDLERS)
 LIST( APPEND INSTALL_TARGETS events_handler)
 LIST( APPEND CATKIN_DEPENDS_SET robot_class robot_class_xbot hierarchical_control    reference_generation 
)
endif(HANDLERS)

if(XBOT)
  find_package(XBotCore-interfaces REQUIRED)
  find_package(XBotInterface REQUIRED)

  LIST( APPEND INSTALL_TARGETS gravity_test_communication base_to_rt odometry_to_rt)
  LIST( APPEND CATKIN_DEPENDS_SET robot_class gazebo_msgs roscpp tf custom_services custom_messages)

endif(XBOT)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${XBotCore-interfaces_INCLUDE_DIRS} ${XBotInterface_INCLUDE_DIRS}
  LIBRARIES
#        events_handler
  CATKIN_DEPENDS 
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${XBotCore-interfaces_INCLUDE_DIRS}
  ${XBotInterface_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../locomotion_framework/third_party/range-v3/include
)


if(HANDLERS)

add_library(events_handler
   src/events_handler.cpp
   src/basic_handler.cpp
 )

add_dependencies(events_handler ${catkin_EXPORTED_TARGETS})

target_link_libraries(events_handler
   ${catkin_LIBRARIES}
 )

endif(HANDLERS)

if(XBOT)
add_executable(gravity_test_communication src/gravity_test_communication.cpp)
add_dependencies(gravity_test_communication ${catkin_EXPORTED_TARGETS} ${XBotCore-interfaces_EXPORTED_TARGETS} ${XBotInterface_EXPORTED_TARGETS})
target_link_libraries(gravity_test_communication
    ${catkin_LIBRARIES}
    ${XBotCore-interfaces_LIBRARIES} 
    ${XBotInterface_LIBRARIES}
 )
#
add_executable(base_to_rt src/gazebo_base_to_RT.cpp)
add_dependencies(base_to_rt ${catkin_EXPORTED_TARGETS} ${XBotCore-interfaces_EXPORTED_TARGETS} ${XBotInterface_EXPORTED_TARGETS})
target_link_libraries(base_to_rt
    ${catkin_LIBRARIES}
    ${XBotCore-interfaces_LIBRARIES} 
    ${XBotInterface_LIBRARIES}
 )

add_executable(odometry_to_rt src/odometry_to_RT.cpp)
add_dependencies(odometry_to_rt ${catkin_EXPORTED_TARGETS} ${XBotCore-interfaces_EXPORTED_TARGETS} ${XBotInterface_EXPORTED_TARGETS})
target_link_libraries(odometry_to_rt
    ${catkin_LIBRARIES}
    ${XBotCore-interfaces_LIBRARIES} 
    ${XBotInterface_LIBRARIES}
 )
#
##add_executable(wheels_references src/wheels_refs.cpp)
##add_dependencies(wheels_references ${catkin_EXPORTED_TARGETS} ${XBotCore-interfaces_EXPORTED_TARGETS})
##target_link_libraries(wheels_references
##    ${catkin_LIBRARIES}
## )
#
endif(XBOT)

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
