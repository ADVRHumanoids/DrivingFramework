cmake_minimum_required(VERSION 2.8.3)
project(gravity_compensation)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  simple_log
  robot_class
  eigen_utils
  basic_controllers
  dynamic_models
)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

catkin_package(
  INCLUDE_DIRS
	include
  LIBRARIES
    gravity_compensation_controllers
  CATKIN_DEPENDS
	roscpp
	robot_class
	eigen_utils
	simple_log
        basic_controllers
        dynamic_models
  DEPENDS
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../third_party/range-v3/include
)

## Declare a C++ library
   add_library(gravity_compensation_controllers
        src/simple_qr_gravity_compensation.cpp
        )

   add_dependencies(gravity_compensation_controllers ${catkin_EXPORTED_TARGETS})

   target_link_libraries(gravity_compensation_controllers
     ${catkin_LIBRARIES}
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
)


if(CATKIN_ENABLE_TESTING)

  include_directories(test/include)
#if(ROS)
  find_package(rostest REQUIRED)
  find_package(tests_common REQUIRED)

  find_package(centauro_gazebo REQUIRED)
  find_package(centauro_control REQUIRED)
  find_package(robot_class_ros_extension REQUIRED)
  include_directories(
    ${robot_class_ros_extension_INCLUDE_DIRS}
    ${tests_common_INCLUDE_DIRS}
    ${centauro_gazebo_INCLUDE_DIRS}
    ${centauro_control_INCLUDE_DIRS})
  add_rostest_gtest(gravity_compensation_test test/launch/test_gravity_compensation.test test/src/test_simple_qr_gravity_compensation.cpp)

 if(TARGET gravity_compensation_test)
  add_dependencies(gravity_compensation_test
    ${catkin_EXPORTED_TARGETS}
    ${robot_class_ros_extension_EXPORTED_TARGETS}
    ${centauro_gazebo_EXPORTED_TARGETS}
    ${centauro_control_EXPORTED_TARGETS}
    ${ROBOT_ROS_EXPORTED_TARGETS}
    ${tests_common_EXPORTED_TARGETS}
    )
  target_link_libraries(gravity_compensation_test
        ${catkin_LIBRARIES}
        ${robot_class_ros_extension_LIBRARIES}
        ${centauro_gazebo_LIBRARIES}
        ${centauro_control_LIBRARIES}
        ${tests_common_LIBRARIES}
   )
 endif()

endif()

#############
## Install ##
#############

# Mark cpp header files for installation
 install(DIRECTORY include/mwoibn/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mwoibn/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

 ## Mark executables and/or libraries for installation
  install(TARGETS gravity_compensation_controllers
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
