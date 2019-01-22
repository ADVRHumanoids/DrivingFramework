cmake_minimum_required(VERSION 2.8.3)
project(robot_class_ros_extension)
include(CMakeDependentOption)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

# The version number, project name, add c++ flags
set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

add_definitions(-std=c++11)

option(LOGGER OFF)
# option(ROS ON)

#cmake_dependent_option(ROS_NRT "ROS_NRT" OFF
#                       "ROS" OFF)

message(STATUS " Flags:")
message(STATUS " LOGGER ${LOGGER}")


set(LOGGER_INCLUDE_DIRS "")
set(LOGGER_LIBRARIES "")
set(LOGGER_EXPORTED_TARGETS "")

#set(ROBOT_ROS_INCLUDE_DIRS "")
#set(ROBOT_ROS_LIBRARIES "")
#set(ROBOT_ROS_EXPORTED_TARGETS "")

#set(ROBOT_ROS_NRT_INCLUDE_DIRS "")
#set(ROBOT_ROS_NRT_LIBRARIES "")
#set(ROBOT_ROS_NRT_EXPORTED_TARGETS "")

set(INSTALL_TARGETS "")

find_package(catkin REQUIRED robot_class)

if(LOGGER)
find_package(simple_log REQUIRED)
LIST(APPEND LOGGER_LIBRARIES
	${simple_log_LIBRARIES}
)
LIST(APPEND LOGGER_INCLUDE_DIRS
	${simple_log_INCLUDE_DIRS}
)
LIST(APPEND LOGGER_EXPORTED_TARGETS
	${simple_log_EXPORTED_TARGETS}
)
endif(LOGGER)

#if(ROS)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(roscpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

LIST(APPEND ROBOT_ROS_LIBRARIES
	${urdf_LIBRARIES}
	${srdfdom_LIBRARIES}
	${roscpp_LIBRARIES}
	${sensor_msgs_LIBRARIES}
	${gazebo_msgs_LIBRARIES}
)
LIST(APPEND ROBOT_ROS_INCLUDE_DIRS
	${urdf_INCLUDE_DIRS}
	${srdfdom_INCLUDE_DIRS}
	${roscpp_INCLUDE_DIRS}
	${sensor_msgs_INCLUDE_DIRS}
        ${gazebo_msgs_INCLUDE_DIRS}
)
LIST(APPEND ROBOT_ROS_EXPORTED_TARGETS
	${urdf_EXPORTED_TARGETS}
	${srdfdom_EXPORTED_TARGETS}
	${roscpp_EXPORTED_TARGETS}
	${sensor_msgs_EXPORTED_TARGETS}
        ${gazebo_msgs_EXPORTED_TARGETS}
)
#endif(ROS)

#if(ROS_NRT)

find_package(communication_modules_ros_extension REQUIRED)
find_package(custom_messages REQUIRED)
find_package(std_srvs REQUIRED)

LIST(APPEND ROBOT_ROS_NRT_LIBRARIES
        ${communication_modules_ros_extension_LIBRARIES}
	${custom_messages_LIBRARIES}
	${std_srvs_LIBRARIES}
)
LIST(APPEND ROBOT_ROS_NRT_INCLUDE_DIRS
        ${communication_modules_ros_extension_INCLUDE_DIRS}
	${custom_messages_INCLUDE_DIRS}
	${std_srvs_INCLUDE_DIRS}
  )
LIST(APPEND ROBOT_ROS_NRT_EXPORTED_TARGETS
        ${communication_modules_ros_extension_EXPORTED_TARGETS}
	${custom_messages_EXPORTED_TARGETS}
	${std_srvs_EXPORTED_TARGETS}
)
#endif(ROS_NRT)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
	include
	${catkin_INCLUDE_DIRS}
	${ROBOT_ROS_INCLUDE_DIRS}
	${ROBOT_ROS_NRT_INCLUDE_DIRS}
	${LOGGER_INCLUDE_DIRS}
  LIBRARIES
	${PROJECT_NAME}
	${ROBOT_ROS_LIBRARIES}
	${ROBOT_ROS_NRT_LIBRARIES}
	${LOGGER_LIBRARIES}
  CATKIN_DEPENDS
	robot_class
        communication_modules_ros_extension
)

include_directories(
include
${catkin_INCLUDE_DIRS}
${ROBOT_ROS_INCLUDE_DIRS}
${ROBOT_ROS_NRT_INCLUDE_DIRS}
${LOGGER_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(ROBOT_CLASS_SOURCE "")
#if(ROS)
	LIST(APPEND ROBOT_CLASS_SOURCE src/robot_ros.cpp)
#endif(ROS)
#if(ROS_NRT)
	LIST(APPEND ROBOT_CLASS_SOURCE src/robot_ros_nrt.cpp)
#endif(ROS_NRT)
	LIST(APPEND ROBOT_CLASS_SOURCE src/contact_ros.cpp)

## Declare a C++ library
 add_library(robot_class_ros_extension
   ${ROBOT_CLASS_SOURCE}
 )

 add_dependencies(robot_class_ros_extension ${catkin_EXPORTED_TARGETS} ${ROBOT_ROS_EXPORTED_TARGETS} ${ROBOT_ROS_NRT_EXPORTED_TARGETS} ${LOGGER_EXPORTED_TARGETS})
 target_compile_definitions(robot_class_ros_extension PUBLIC RBDL_BUILD_ADDON_URDFREADER)
 target_link_libraries(robot_class_ros_extension
   ${catkin_LIBRARIES}
   ${ROBOT_ROS_LIBRARIES}
   ${ROBOT_ROS_NRT_LIBRARIES}
   ${LOGGER_LIBRARIES}
)

LIST(APPEND INSTALL_TARGETS
	robot_class_ros_extension
)

# Mark executables and/or libraries for installation
 install(TARGETS ${INSTALL_TARGETS}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/mwoibn/robot_class
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mwoibn/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )


if(CATKIN_ENABLE_TESTING)

#if(ROS)
  find_package(rostest REQUIRED)
  find_package(centauro_gazebo REQUIRED)
  find_package(centauro_control REQUIRED)
  find_package(tests_common REQUIRED)


  include_directories(${centauro_gazebo_INCLUDE_DIRS} ${centauro_control_INCLUDE_DIRS}  ${tests_common_INCLUDE_DIRS})

 add_rostest_gtest(robot_ros_nrt_test test/launch/test_robot_ros_nrt.test test/src/test_robot_ros_nrt.cpp test/src/test_robot_ros.cpp  test/src/test_contacts.cpp)

 if(TARGET robot_ros_nrt_test)

  add_dependencies(robot_ros_nrt_test
    ${centauro_gazebo_EXPORTED_TARGETS}
    ${tests_common_EXPORTED_TARGETS}
    ${centauro_control_EXPORTED_TARGETS}
    ${ROBOT_ROS_EXPORTED_TARGETS}
    ${ROBOT_ROS_NRT_EXPORTED_TARGETS})

  target_link_libraries(robot_ros_nrt_test
        ${PROJECT_NAME}
        ${ROBOT_ROS_LIBRARIES}
        ${ROBOT_ROS_NRT_LIBRARIES}
        ${centauro_gazebo_LIBRARIES}
        ${centauro_control_LIBRARIES}
        ${tests_common_LIBRARIES}
   )
 endif()

endif(CATKIN_ENABLE_TESTING)
