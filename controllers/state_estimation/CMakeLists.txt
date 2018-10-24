cmake_minimum_required(VERSION 2.8.3)
project(state_estimation)


set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

LIST( APPEND INSTALL_TARGETS odometry ground_forces acceleration_test)
LIST( APPEND CATKIN_DEPENDS_SET robot_class robot_point_handling filters modules)
LIST( APPEND CATKIN_DEPENDS_SET dynamic_models)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)


catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES
        odometry
        ground_forces
        acceleration_test
  CATKIN_DEPENDS
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(odometry
   src/odometry.cpp
   src/odometry_v2.cpp
)

add_dependencies(odometry
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(odometry
   ${catkin_LIBRARIES}
)

add_library(ground_forces
   src/ground_forces.cpp
)

add_dependencies(ground_forces
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ground_forces
   ${catkin_LIBRARIES}
)

add_library(acceleration_test
   src/acceleration_test.cpp
)

add_dependencies(acceleration_test
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(acceleration_test
   ${catkin_LIBRARIES}
)

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
