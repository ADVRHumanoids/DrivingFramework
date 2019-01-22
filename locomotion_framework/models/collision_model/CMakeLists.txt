cmake_minimum_required(VERSION 2.8.3)
project(collision_model)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  simple_log
  robot_class
  robot_point_handling
  urdf
  srdfdom
)
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )
find_package(RBDL REQUIRED)
find_package(Eigen3 REQUIRED)

message(WARNING RBDL_INCLUDE_DIRS $RBDL_INCLUDE_DIRS)


LIST( APPEND /usr/lib/x86_64-linux-gnu)
find_package(sch-core REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_collision
  CATKIN_DEPENDS robot_class robot_point_handling
  DEPENDS sch-core
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${sch-core_INCLUDE_DIRS}
)

 add_library(robot_collision
   src/robot_collision.cpp
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
 add_dependencies(robot_collision ${catkin_EXPORTED_TARGETS})

 target_link_libraries(robot_collision
   ${catkin_LIBRARIES}
   ${RBDL_LIBRARY}
   ${sch-core_LIBRARIES}
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS robot_collision
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

 LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )
 find_package(RBDL REQUIRED)
 find_package(Eigen3 REQUIRED)

 catkin_add_gtest(${PROJECT_NAME}-test test/src/test_collision_model.cpp)

 if(TARGET ${PROJECT_NAME}-test)
  include_directories(${roslib_INCLUDE_DIRS} ${centauro_INCLUDE_DIRS})
  add_dependencies(${PROJECT_NAME}-test ${catkin_EXPORTED_TARGETS} ${roslib_EXPORTED_TARGETS} ${centauro_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}-test
        robot_collision
        ${RBDL_LIBRARY}
        ${RBDL_URDFReader_LIBRARY}
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        ${centauro_LIBRARIES}
   )
 endif()

endif(CATKIN_ENABLE_TESTING)
