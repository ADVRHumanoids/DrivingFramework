cmake_minimum_required(VERSION 2.8.3)
project(point_handling)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

option(MAKE_TEST_LOG "If on running test will generate new resources files, DO NOT OVERWIRTE THEM WITHOUT CLEAR AND JUSTIFIED REASON" ON)
option(LOGGER ON)

set(CATKIN_DEPENDS_SET "")

if(MAKE_TEST_LOG)
  add_definitions(-DMAKE_LOG)
endif()

if(LOGGER)
  LIST(APPEND CATKIN_DEPENDS_SET simple_log)
endif()

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

find_package(catkin REQUIRED ${CATKIN_DEPENDS_SET} common)

## System dependencies are found with CMake's conventions
##find_package(RBDL REQUIRED)
##find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES
    points_handler
  CATKIN_DEPENDS ${CATKIN_DEPENDS_SET}
  INCLUDE_DIRS
    include
    ${catkin_INCLUDE_DIRS}
)

 add_library(points_handler
   #src/base_points_handler.cpp

   src/position.cpp
   src/rotation.cpp
   src/orientation.cpp
   src/linear_velocity.cpp
   src/angular_velocity.cpp
   src/spatial_velocity.cpp
   src/frame.cpp
   src/force.cpp
   src/torque.cpp
   src/wrench.cpp
 )

 add_dependencies(points_handler ${catkin_EXPORTED_TARGETS})

 target_link_libraries(points_handler
   ${catkin_LIBRARIES}
 )

 


#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS points_handler
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
## Mark cpp header files for installation
 install(DIRECTORY include/mwoibn/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mwoibn/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
 find_package(roslib REQUIRED)
 find_package(centauro REQUIRED)
 find_package(RBDL REQUIRED)
 find_package(Eigen3 REQUIRED)

 catkin_add_gtest(${PROJECT_NAME}-test
	test/src/test_point_handling.cpp
	test/src/test_point_position.cpp
	test/src/test_position_handler.cpp
	test/src/test_orientation_handler.cpp
	test/src/test_full_state_handler.cpp

#	test/src/test_robot_handler.cpp

)

 if(TARGET ${PROJECT_NAME}-test)
  include_directories(${roslib_INCLUDE_DIRS} ${centauro_INCLUDE_DIRS} test/include)
  add_dependencies(${PROJECT_NAME}-test ${catkin_EXPORTED_TARGETS} ${roslib_EXPORTED_TARGETS} ${centauro_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}-test
        points_handler
        ${RBDL_LIBRARY}
        ${RBDL_URDFReader_LIBRARY}
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        ${centauro_LIBRARIES}
   )
 endif()

endif(CATKIN_ENABLE_TESTING)
