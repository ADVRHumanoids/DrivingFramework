cmake_minimum_required(VERSION 2.8.3)
project(mgnss_ros_callbacks_wheels)


set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

# set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

LIST( APPEND INSTALL_TARGETS ros_callbacks_wheels)
LIST( APPEND CATKIN_DEPENDS_SET mgnss_controllers_wheels custom_services custom_messages mgnss_utils)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)


catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES
        ros_callbacks_wheels
#        centralized_controllers
#        joint_space_controllers
  CATKIN_DEPENDS
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../locomotion_framework/third_party/range-v3/include
)

add_library(ros_callbacks_wheels
   src/wheels_controller_extend.cpp
   src/wheels_controller_events.cpp
   src/wheels_controller_second_order.cpp
 )

add_dependencies(ros_callbacks_wheels ${catkin_EXPORTED_TARGETS})

target_link_libraries(ros_callbacks_wheels
   ${catkin_LIBRARIES}
 )

#MESSAGE( WARNING "CMAKE_INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX} )
#MESSAGE( WARNING "CMAKE_DEVEL_PREFIX: " ${CMAKE_DEVEL_PREFIX} )

## Mark executables and/or libraries for installation
 install(TARGETS ${INSTALL_TARGETS}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
 install(DIRECTORY include/mgnss/ros_callbacks
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mgnss/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
