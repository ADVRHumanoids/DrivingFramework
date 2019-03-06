cmake_minimum_required(VERSION 2.8.3)
project(controllers)


set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

LIST( APPEND INSTALL_TARGETS mgnss_controllers_wheels mgnss_controllers_centralized mgnss_controllers_joint_space)
LIST( APPEND CATKIN_DEPENDS_SET hierarchical_control dynamic_models gravity_compensation motor_side_reference modules mgnss_higher_level robot_class)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)


catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES
        mgnss_controllers_wheels
        mgnss_controllers_centralized
        mgnss_controllers_joint_space
  CATKIN_DEPENDS
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(mgnss_controllers_wheels

   src/wheels_controller.cpp
   src/wheels_controller_extend.cpp
#   src/wheeled_motion_event_v3.cpp
#   src/wheels_second_order.cpp

#   src/wheeled_motion_actions.cpp
#   src/wheeled_motion_merge_v1.cpp
#   src/wheeled_motion_world.cpp
#   src/wheels_reactif.cpp
   src/wheels_zmp.cpp
 )

add_dependencies(mgnss_controllers_wheels ${catkin_EXPORTED_TARGETS})

target_link_libraries(mgnss_controllers_wheels
   ${catkin_LIBRARIES}
 )


add_library(mgnss_controllers_centralized
   src/online_centralized_controller.cpp
   src/compliance_compensation.cpp
   src/centralized_controller.cpp
 )

add_dependencies(mgnss_controllers_centralized ${catkin_EXPORTED_TARGETS})

target_link_libraries(mgnss_controllers_centralized
   ${catkin_LIBRARIES}
 )


add_library(mgnss_controllers_joint_space
   src/joint_states.cpp
 )

add_dependencies(mgnss_controllers_joint_space ${catkin_EXPORTED_TARGETS})

target_link_libraries(mgnss_controllers_joint_space
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
 install(DIRECTORY include/mgnss/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mgnss/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
