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

LIST( APPEND INSTALL_TARGETS wheeled_motion centralized_controllers joint_space_controllers)
LIST( APPEND CATKIN_DEPENDS_SET hierarchical_control dynamic_models gravity_compensation motor_side_reference modules)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)


catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES
        wheeled_motion
        centralized_controllers
        joint_space_controllers
  CATKIN_DEPENDS
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(wheeled_motion
   src/steering.cpp                 
   src/steering_v2.cpp                 
   src/steering_v3.cpp     
   src/steering_v4.cpp
   src/steering_v5.cpp                             
   src/wheeled_motion.cpp     
   src/wheeled_references.cpp 
   src/wheeled_references_v2.cpp
   src/wheeled_references_v3.cpp
   src/wheeled_motion_full.cpp
   src/wheeled_motion_com.cpp
   src/wheeled_motion_event.cpp
   src/wheeled_motion_event_v2.cpp
 )

add_dependencies(wheeled_motion ${catkin_EXPORTED_TARGETS})

target_link_libraries(wheeled_motion
   ${catkin_LIBRARIES}
 )


add_library(centralized_controllers
   src/online_centralized_controller.cpp
 )

add_dependencies(centralized_controllers ${catkin_EXPORTED_TARGETS})

target_link_libraries(centralized_controllers
   ${catkin_LIBRARIES}
 )


add_library(joint_space_controllers
   src/joint_states.cpp 
 )

add_dependencies(joint_space_controllers wheeled_motion ${catkin_EXPORTED_TARGETS})

target_link_libraries(joint_space_controllers
   wheeled_motion
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
