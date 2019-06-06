cmake_minimum_required(VERSION 2.8.3)
project(robot_point_handling)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

set(CMAKE_CXX_STANDARD 14)

option(MAKE_TEST_LOG "If on running test will generate new resources files, DO NOT OVERWIRTE THEM WITHOUT CLEAR AND JUSTIFIED REASON" OFF)
if(MAKE_TEST_LOG)
  add_definitions(-DMAKE_LOG)
endif()

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )
find_package(catkin REQUIRED
        robot_class point_handling)
include_directories(
  include
  ${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES
    points_handler

  INCLUDE_DIRS
    include
    ${catkin_INCLUDE_DIRS} 
  CATKIN_DEPENDS
    robot_class point_handling
)


#############
## Install ##
#############

## Mark cpp header files for installation
 install(DIRECTORY include/mwoibn/point_handling
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
 find_package(tests_common REQUIRED)
 find_package(RBDL REQUIRED)
 find_package(Eigen3 REQUIRED)

 catkin_add_gtest(${PROJECT_NAME}-test 
	test/src/test_point_handling.cpp
	test/src/test_robot_handler.cpp
 )

 if(TARGET ${PROJECT_NAME}-test)
  include_directories(${roslib_INCLUDE_DIRS} ${centauro_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${tests_common_INCLUDE_DIRS} test/include)
  add_dependencies(${PROJECT_NAME}-test ${catkin_EXPORTED_TARGETS} ${roslib_EXPORTED_TARGETS} ${centauro_EXPORTED_TARGETS} ${tests_common_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}-test
        ${RBDL_LIBRARY}
        ${RBDL_URDFReader_LIBRARY}
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        ${centauro_LIBRARIES}
        ${tests_common_LIBRARIES}
   )
 endif()

endif(CATKIN_ENABLE_TESTING)
