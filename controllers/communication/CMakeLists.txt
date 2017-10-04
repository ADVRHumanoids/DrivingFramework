cmake_minimum_required(VERSION 2.8.3)
project(communication)
option(XBOT ON)

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )


set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

if(HANDLERS)
 LIST( APPEND INSTALL_TARGETS events_handler)
 LIST( APPEND CATKIN_DEPENDS_SET robot_class robot_class_xbot hierarchical_control reference_generation )
endif(HANDLERS)

if(XBOT)
  find_package(XBotCore-interfaces)
  LIST( APPEND INSTALL_TARGETS gravity_test_communication base_to_rt wheels_references)
  LIST( APPEND CATKIN_DEPENDS_SET robot_class gazebo_msgs roscpp tf controllers custom_services)
endif(XBOT)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${XBotCore-interfaces_INCLUDE_DIRS}
  LIBRARIES
        events_handler
  CATKIN_DEPENDS 
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${XBotCore-interfaces_INCLUDE_DIRS}
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
add_dependencies(gravity_test_communication ${catkin_EXPORTED_TARGETS} ${XBotCore-interfaces_EXPORTED_TARGETS})
target_link_libraries(gravity_test_communication
    ${catkin_LIBRARIES}
 )

add_executable(base_to_rt src/gazebo_base_to_RT.cpp)
add_dependencies(base_to_rt ${catkin_EXPORTED_TARGETS} ${XBotCore-interfaces_EXPORTED_TARGETS})
target_link_libraries(base_to_rt
    ${catkin_LIBRARIES}
 )

add_executable(wheels_references src/wheels_refs.cpp)
add_dependencies(wheels_references ${catkin_EXPORTED_TARGETS} ${XBotCore-interfaces_EXPORTED_TARGETS})
target_link_libraries(wheels_references
    ${catkin_LIBRARIES}
 )

endif(XBOT)

## Mark executables and/or libraries for installation
 install(TARGETS ${INSTALL_TARGETS}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
 install(DIRECTORY include
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
