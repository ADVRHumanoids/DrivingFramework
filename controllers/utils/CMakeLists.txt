cmake_minimum_required(VERSION 2.8.3)
project(mgnss_utils)


set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 14)


LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

set(INSTALL_TARGETS "")
set(PYTHON_INSTALL "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

option(VISUALZATION_TOOLS ON)

FUNCTION( prepend_paths SOURCE_FILES PARENT_DIR)
  FOREACH( SOURCE_FILE ${${SOURCE_FILES}} )
    SET( MODIFIED ${MODIFIED} ${PARENT_DIR}/${SOURCE_FILE} )
  ENDFOREACH()
  SET( ${SOURCE_FILES} ${MODIFIED} PARENT_SCOPE )
ENDFUNCTION()

if(VISUALIZATION_TOOLS)

 add_definitions(-DVISUALIZATION_TOOLS)
if(ROS_CONTROL)
 LIST( APPEND INSTALL_TARGETS real_robot_tracker)
 LIST( APPEND CATKIN_DEPENDS_SET visualization_tools robot_point_handling roscpp robot_class_ros_extension message_runtime)
endif(ROS_CONTROL)
endif(VISUALIZATION_TOOLS)

# LIST( APPEND PYTHON_INSTALL circular square support discontinuous discontinuous_odom turn_iit straight_iit support_iit turn_iit_odom turn_iit_rt straight_iit_rt support_iit_rt circular_RT support_RT dampings_test discontinuous_vel events_vel straight_iit_july)
# prepend_paths(PYTHON_INSTALL wheels_references)
# LIST( APPEND PYTHON_INSTALL circular_reference_wheels collision_test load_fixed_transforms )

 LIST( APPEND PYTHON_INSTALL events_vel straight_iit_july reference try apply_force)
 prepend_paths(PYTHON_INSTALL wheels_references)
 LIST( APPEND PYTHON_INSTALL circular_reference_wheels collision_test load_fixed_transforms )


# LIST( APPEND CATKIN_DEPENDS_SET message_runtime)
 LIST( APPEND CATKIN_DEPENDS_BUILD message_generation )
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)


# Generate services in the 'srv' folder
add_service_files(
   FILES
   point.srv
 )

generate_messages()

catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS} launch
  LIBRARIES
  CATKIN_DEPENDS
	${CATKIN_DEPENDS_SET}
        message_runtime
  DEPENDS
	${DEPENDS_OPTIONAL}
)



## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   # Or other packages containing msgs
# )

include_directories(
  ${catkin_INCLUDE_DIRS}
  launch
)

if(VISUALIZATION_TOOLS)
if(ROS_CONTROL)
add_executable(real_robot_tracker src/real_robot_tracker.cpp)
add_dependencies(real_robot_tracker ${catkin_EXPORTED_TARGETS})
target_link_libraries(real_robot_tracker
    ${catkin_LIBRARIES}
 )
endif(ROS_CONTROL)
endif(VISUALIZATION_TOOLS)

## Mark executables and/or libraries for installation
 install(TARGETS ${INSTALL_TARGETS}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(DIRECTORY include/mgnss/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mgnss/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

prepend_paths(PYTHON_INSTALL nodes)

catkin_install_python(PROGRAMS ${PYTHON_INSTALL}
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
