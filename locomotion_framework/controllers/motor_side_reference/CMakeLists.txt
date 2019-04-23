cmake_minimum_required(VERSION 2.8.3)
project(motor_side_reference)

add_definitions(-std=c++11)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  simple_log
  basic_controllers
  eigen_utils
  dynamic_models
)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )


catkin_package(
  INCLUDE_DIRS
	include
    ${PROJECT_SOURCE_DIR}/../../third_party/range-v3/include    
  LIBRARIES
	motor_side_reference
  CATKIN_DEPENDS
        basic_controllers
	eigen_utils
	simple_log
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
   add_library(motor_side_reference

        src/sea_reference.cpp
        src/sea_compensation.cpp
)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
   add_dependencies(motor_side_reference ${catkin_EXPORTED_TARGETS})

   target_link_libraries(motor_side_reference
     ${catkin_LIBRARIES}
   )


if(CATKIN_ENABLE_TESTING)

 find_package(rostest REQUIRED)
 find_package(tests_common REQUIRED)
 find_package(robot_class_ros_extension REQUIRED)
 find_package(gravity_compensation REQUIRED)

 add_rostest_gtest(motor_side_reference_test test/launch/test_motor_side_reference.test test/src/test_motor_side_reference.cpp test/src/test_sea_reference.cpp)

 if(TARGET motor_side_reference_test)
  include_directories(
    ${gravity_compensation_INCLUDE_DIRS}
    ${tests_common_INCLUDE_DIRS}
    ${robot_class_ros_extension_INCLUDE_DIRS})
  add_dependencies(motor_side_reference_test
    ${catkin_EXPORTED_TARGETS}
    ${gravity_compensation_EXPORTED_TARGETS}
    ${tests_common_EXPORTED_TARGETS}
    ${robot_class_ros_extension_EXPORTED_TARGETS})
  target_link_libraries(motor_side_reference_test
    motor_side_reference
    ${catkin_LIBRARIES}
    ${gravity_compensation_LIBRARIES}
    ${tests_common_LIBRARIES}
    ${robot_class_ros_extension_LIBRARIES}
   )
 endif()
endif()

#############
## Install ##
#############

# Mark executables and/or libraries for installation
 install(TARGETS motor_side_reference
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
 install(DIRECTORY include/mwoibn/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mwoibn/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
