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

LIST( APPEND INSTALL_TARGETS wheeled_motion centralized_controllers)
LIST( APPEND CATKIN_DEPENDS_SET robot_class hierarchical_control dynamic_models gravity_compensation motor_side_reference)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)


catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES
        wheeled_motion
        centralized_controllers
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
   src/wheeled_motion.cpp
   src/wheeled_references.cpp
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
